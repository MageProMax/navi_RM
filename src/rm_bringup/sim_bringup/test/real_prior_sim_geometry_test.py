#!/usr/bin/env python3
"""Collision geometry: closed outward surface, no interior faces, bounded error."""
import sys
import unittest
import json
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from build_real_prior_sim import voxel_surface, build, digest, read_binary_pcd


class GeometryTest(unittest.TestCase):
    def test_two_adjacent_voxels_remove_shared_faces(self):
        count, triangles = voxel_surface(np.array([[.025, .025, .025], [.075, .025, .025]]), .05)
        self.assertEqual(count, 2)
        self.assertEqual(len(triangles), 20)
        self.assertFalse(np.any(np.all(np.isclose(triangles[:, :, 0], .05), axis=1)))
        # Every boundary edge belongs to exactly two triangles: watertight.
        edges = {}
        for tri in triangles:
            for i, j in ((0, 1), (1, 2), (2, 0)):
                edge = tuple(sorted((tuple(tri[i]), tuple(tri[j]))))
                edges[edge] = edges.get(edge, 0) + 1
        self.assertEqual(set(edges.values()), {2})

    def test_negative_map_coordinates_and_outward_normals(self):
        point = np.array([[-.012, -.206, .122]])
        count, triangles = voxel_surface(point, .05)
        self.assertEqual((count, len(triangles)), (1, 12))
        low = np.floor(point[0] / .05) * .05
        np.testing.assert_allclose(triangles.min(axis=(0, 1)), low, atol=1e-7)
        np.testing.assert_allclose(triangles.max(axis=(0, 1)), low + .05, atol=1e-7)
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        self.assertTrue(np.all(np.sum(normals * (triangles.mean(axis=1) - low - .025), axis=1) > 0))

    def test_source_preservation_cache_and_corrupt_artifact_rebuild(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / 'source'
            source.mkdir()
            xyz = np.array([[1., 2., .5], [1.01, 2., .51]], dtype='<f4')
            (source / 'map.pcd').write_bytes(
                b'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                b'WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n' + xyz.tobytes())
            (source / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
            (source / 'map.yaml').write_text('image: map.pgm\n')
            (source / 'start_pose.env').write_text(
                'export NAV_INITIAL_X=-0.01\nexport NAV_INITIAL_Y=-0.2\nexport NAV_INITIAL_YAW=0.\n')
            metadata = {'output_pcd_sha256': digest(source / 'map.pcd'),
                        'plane': {'a': .01, 'b': -.02, 'c_m': .03}}
            (source / 'height_filter_metadata.json').write_text(json.dumps(metadata))
            before = {path.name: path.read_bytes() for path in source.iterdir()}
            output = source / 'simulation'
            first = build(source, output)
            self.assertEqual(first['voxel_count'], 1)
            self.assertEqual(first['triangle_count'], 12)
            self.assertAlmostEqual(first['spawn_z'], .01)
            self.assertEqual(first['floor_plane'], {'a': 0., 'b': 0., 'c_m': 0.})
            stamp = (output / 'obstacles.stl').stat().st_mtime_ns
            self.assertEqual(build(source, output), first)
            self.assertEqual((output / 'obstacles.stl').stat().st_mtime_ns, stamp)
            (output / 'obstacles.stl').write_bytes(b'broken')
            repaired = build(source, output)
            self.assertEqual(digest(output / 'obstacles.stl'), repaired['artifacts']['obstacles.stl'])
            for name, contents in before.items():
                self.assertEqual((source / name).read_bytes(), contents)
            metadata['output_pcd_sha256'] = 'wrong'
            (source / 'height_filter_metadata.json').write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError, 'does not match'):
                build(source, output)


if __name__ == '__main__':
    unittest.main()
