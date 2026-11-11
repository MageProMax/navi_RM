#!/usr/bin/env python3
"""Measured clearing geometry and obstacle-disappearance message regressions."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from accumulated_scan import AccumulatedScan, clearing_endpoints


class ClearingGeometryTest(unittest.TestCase):
    origin = np.array([0., .22157, .33018])

    def rays(self, points, plane=None):
        if plane is None:
            plane = np.zeros(3)
        return clearing_endpoints(np.asarray(points, dtype=float).reshape(-1, 3),
                                  self.origin, plane, .0087, .10, .50)

    def test_floor_and_ceiling_stop_at_height_band(self):
        floor = np.array([8., self.origin[1], 0.])
        end = self.rays([floor])[0]
        fraction = (.33018 - .135) / .33018
        np.testing.assert_allclose(end, self.origin + fraction * (floor - self.origin))
        self.assertLess(end[0], 5.)
        ceiling = np.array([8., self.origin[1], 3.])
        end = self.rays([ceiling])[0]
        self.assertAlmostEqual(end[2], .465)
        self.assertLess(end[0], .5)

    def test_tilted_floor_uses_relative_height_and_actual_ray(self):
        plane = np.array([.05, -.02, .01])
        floor = np.array([6., 1., 0.])
        floor[2] = floor[:2] @ plane[:2] + plane[2]
        end = self.rays([floor], plane)[0]
        self.assertAlmostEqual(end[2] - end[:2] @ plane[:2] - plane[2], .135)
        np.testing.assert_allclose(np.cross(end-self.origin, floor-self.origin), 0., atol=1e-12)

    def test_current_obstacle_blocks_background_and_neighbour_bin(self):
        obstacle = [2., self.origin[1], .25]
        # Both a ground ray and a neighbouring almost-horizontal distant ray
        # would pass the current obstacle without the occlusion cap.
        points = [obstacle, [8., self.origin[1], 0.],
                  [8., self.origin[1]+.07, .6]]
        ends = self.rays(points)
        self.assertGreater(len(ends), 0)
        distance = np.linalg.norm(ends[:, :2]-self.origin[:2], axis=1)
        self.assertTrue(np.all(distance <= 1.9 + 1e-12))

    def test_missing_or_untrusted_measurements_never_create_rays(self):
        self.assertEqual(len(self.rays([])), 0)
        self.assertEqual(len(self.rays([[math.nan, 0., 0.]])), 0)
        self.assertEqual(len(clearing_endpoints(np.array([[8., 0., 0.]]),
                         self.origin, None, .0087, .10, .50)), 0)
        # A floor placing the sensor outside the obstacle band is unusable.
        self.assertEqual(len(self.rays([[8., 0., 0.]], np.array([0., 0., .4]))), 0)

    def test_range_limit_and_unmeasured_directions(self):
        ends = self.rays([[20., self.origin[1], .33018]])
        self.assertEqual(len(ends), 1)
        np.testing.assert_allclose(ends[0], [10., self.origin[1], .33018])


class ClearingPublicationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = AccumulatedScan()
        self.node.history = 0.
        self.node.period = .1
        self.node.minimum_bins = 1
        self.node.high = .5
        self.node.increment = .0087
        self.node.clearing_enabled = True
        self.scans, self.clouds = [], []
        self.node.pub = SimpleNamespace(publish=self.scans.append)
        self.node.clearing_pub = SimpleNamespace(publish=self.clouds.append)
        transform = TransformStamped()
        transform.transform.rotation.w = 1.
        self.node.buffer = SimpleNamespace(lookup_transform=lambda *args: transform)

    def tearDown(self):
        self.node.destroy_node()

    def feed(self, points, seconds):
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=seconds+.01))
        header = Header(stamp=Time(seconds=seconds).to_msg(), frame_id='base_link')
        self.node.pending.append(point_cloud2.create_cloud_xyz32(header, points))
        self.node.tick()

    def test_obstacle_departure_replaces_marking_and_clearing_observations(self):
        x, y = np.meshgrid(np.linspace(-3, 3, 41), np.linspace(-3, 3, 41))
        floor = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
        floor = np.vstack((floor, [[8., .22157, 0.]]))
        self.feed(np.vstack((floor, [[2., .22157, .25]])), 100.)
        self.assertTrue(np.isfinite(self.scans[-1].ranges).any())
        self.feed(floor, 100.11)
        self.assertEqual(len(self.scans), 2)
        self.assertTrue(np.isnan(self.scans[-1].ranges).all())
        self.assertEqual(len(self.clouds), 2)
        self.assertEqual(self.clouds[-1].header, self.scans[-1].header)
        points = point_cloud2.read_points(self.clouds[-1], field_names=('x', 'y', 'z'))
        self.assertTrue(np.isfinite(points['x']).all())
        self.assertTrue(np.any((points['x'] > 4.) & (abs(points['y']-.22157) < .01)))
        # Total loss of returns replaces both observations but claims no free space.
        self.feed([], 100.22)
        self.assertEqual(len(self.scans), 3)
        self.assertTrue(np.isnan(self.scans[-1].ranges).all())
        self.assertEqual(self.clouds[-1].width, 0)


if __name__ == '__main__':
    unittest.main()
