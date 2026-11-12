#!/usr/bin/env python3
"""Build a Gazebo collision world from the current real prior map, without editing it."""
import argparse
import hashlib
import json
import math
import re
import struct
from pathlib import Path
from xml.sax.saxutils import escape

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_MAP = ROOT / 'maps_output/20261110_152945/obstacle_band'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_binary_pcd(path):
    """Read scalar XYZ from PCL binary PCD, including padded point records."""
    values, header = {}, []
    with Path(path).open('rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD has no DATA header')
            header.append(line)
            words = line.decode('ascii').strip().split()
            if words and not words[0].startswith('#'):
                values[words[0]] = words[1:]
                if words[0] == 'DATA':
                    break
        if values['DATA'] != ['binary']:
            raise ValueError('Use an uncompressed binary PCD')
        fields = values['FIELDS']
        sizes, counts = list(map(int, values['SIZE'])), list(map(int, values.get('COUNT', ['1'] * len(fields))))
        if not len(fields) == len(sizes) == len(counts) == len(values['TYPE']):
            raise ValueError('Invalid PCD field definitions')
        definitions, used = [], set()
        for index, (name, size, kind, count) in enumerate(zip(fields, sizes, values['TYPE'], counts)):
            name = name if name not in used else f'padding_{index}'
            used.add(name)
            code = {'F': 'f', 'I': 'i', 'U': 'u'}[kind] + str(size)
            definitions.append((name, '<' + code, (count,)) if count > 1 else (name, '<' + code))
        dtype = np.dtype(definitions)
        payload = stream.read()
        count = int(values['POINTS'][0])
        if len(payload) != count * dtype.itemsize:
            raise ValueError('PCD payload size mismatch')
    points = np.frombuffer(payload, dtype=dtype)
    for axis in ('x', 'y', 'z'):
        if axis not in fields or points[axis].ndim != 1:
            raise ValueError('PCD requires scalar XYZ fields')
    return header, points


def start_pose(directory):
    values = dict(re.findall(r'^export (NAV_INITIAL_[A-Z]+)=([-+\deE.]+)\s*$',
                             (directory / 'start_pose.env').read_text(), re.M))
    return [float(values['NAV_INITIAL_' + key]) for key in ('X', 'Y', 'YAW')]


def voxel_surface(xyz, resolution):
    """Return only exposed voxel faces, with outward triangle winding."""
    occupied = set(map(tuple, np.unique(np.floor(xyz / resolution).astype(np.int64), axis=0)))
    faces = [
        ((-1, 0, 0), [(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)]),
        ((1, 0, 0), [(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)]),
        ((0, -1, 0), [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)]),
        ((0, 1, 0), [(0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)]),
        ((0, 0, -1), [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)]),
        ((0, 0, 1), [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]),
    ]
    triangles = []
    for cell in sorted(occupied):
        for normal, offsets in faces:
            neighbor = tuple(cell[i] + normal[i] for i in range(3))
            if neighbor not in occupied:
                corners = (np.asarray(offsets) + cell) * resolution
                triangles.extend((corners[[0, 1, 2]], corners[[0, 2, 3]]))
    return len(occupied), np.asarray(triangles, dtype=np.float32)


def write_stl(path, triangles):
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    records = np.zeros(len(triangles), dtype=[('normal', '<f4', (3,)),
                                            ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    records['normal'], records['vertices'] = normals, triangles
    with path.open('wb') as stream:
        stream.write(b'Real prior map exposed voxel surface'.ljust(80, b'\0'))
        stream.write(struct.pack('<I', len(triangles)))
        stream.write(records.tobytes())


def build(directory, output, resolution=.05):
    directory, output = directory.resolve(), output.resolve()
    if not math.isfinite(resolution) or not .02 <= resolution <= .2:
        raise ValueError('Voxel resolution must be between 0.02 and 0.2 m')
    if output == directory or directory.is_relative_to(output):
        raise ValueError('Use a separate output directory')
    config = yaml.safe_load((directory / 'map.yaml').read_text())
    image_path = directory / config['image']
    sources = {name: digest(directory / name) for name in (
        'map.yaml', 'map.pcd', 'height_filter_metadata.json', 'start_pose.env')}
    sources['map_image'] = digest(image_path)
    sources['builder'] = digest(Path(__file__))
    signature = hashlib.sha256(json.dumps(
        [sources, resolution, str(directory), str(output)], sort_keys=True).encode()).hexdigest()
    manifest_path = output / 'manifest.json'
    if manifest_path.is_file():
        old = json.loads(manifest_path.read_text())
        if (old.get('signature') == signature and all(
                (output / name).is_file() and digest(output / name) == checksum
                for name, checksum in old['artifacts'].items())):
            return old
    metadata = json.loads((directory / 'height_filter_metadata.json').read_text())
    if metadata['output_pcd_sha256'] != sources['map.pcd']:
        raise ValueError('Height metadata does not match the localization PCD')
    coefficients = [float(metadata['plane'][k]) for k in ('a', 'b', 'c_m')]
    if not all(math.isfinite(v) for v in coefficients):
        raise ValueError('Nonfinite floor plane')
    _, points = read_binary_pcd(directory / 'map.pcd')
    xyz = np.column_stack([points[k] for k in ('x', 'y', 'z')])
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz):
        raise ValueError('Map contains no finite points')
    output.mkdir(parents=True, exist_ok=True)
    count, triangles = voxel_surface(xyz, resolution)
    mesh_path = output / 'obstacles.stl'
    write_stl(mesh_path, triangles)
    mesh_uri = escape(mesh_path.as_uri())
    # A fitted plane in a saved LIO map is not evidence of physical terrain.
    # Use a horizontal floor; preserve the original PCD and crop metadata for
    # localization instead of turning their coordinate tilt into a real slope.
    world = f'''<?xml version="1.0"?>
<sdf version="1.6"><world name="real_prior_replica">
  <gravity>0 0 -9.81</gravity>
  <physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
  <light name="sun" type="directional"><pose>0 0 20 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.5 0.1 -0.9</direction></light>
  <model name="recorded_obstacles"><static>true</static><link name="map">
    <collision name="map"><geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry></collision>
    <visual name="map"><geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>
      <material><ambient>0.5 0.6 0.7 1</ambient><diffuse>0.5 0.6 0.7 1</diffuse></material></visual>
  </link></model>
  <model name="ground_plane"><static>true</static><link name="floor">
    <collision name="floor"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
      <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision>
    <visual name="floor"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
      <material><ambient>0.8 0.8 0.8 1</ambient></material></visual>
  </link></model>
</world></sdf>
'''
    (output / 'replica.world').write_text(world)
    pose = start_pose(directory)
    result = {'signature': signature, 'source_map': str(directory), 'source_hashes': sources,
              'voxel_resolution_m': resolution, 'voxel_count': count, 'triangle_count': len(triangles),
              'initial_pose': pose, 'spawn_z': .01,
              'floor_plane': {'a': 0., 'b': 0., 'c_m': 0.},
              'source_map_ground_fit': metadata['plane'],
              'scope': 'Recorded obstacle-band voxel surface plus horizontal floor; approximate sensor/physics geometry',
              'artifacts': {name: digest(output / name) for name in ('obstacles.stl', 'replica.world')}}
    manifest_path.write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', type=Path, default=DEFAULT_MAP)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--resolution', type=float, default=.05)
    args = parser.parse_args()
    print(json.dumps(build(args.map_dir, args.output_dir or args.map_dir / 'simulation', args.resolution), indent=2))
