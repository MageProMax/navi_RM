#!/usr/bin/env python3
"""Exercise actual cloud projection and moving-frame ground cache with ROS messages."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from accumulated_scan import AccumulatedScan


class GroundScanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node=AccumulatedScan()
        self.node.history=0.;self.node.period=.1;self.node.minimum_bins=1
        self.output=[]
        self.node.pub=SimpleNamespace(publish=self.output.append)

    def tearDown(self):
        self.node.destroy_node()

    def test_cloud_projection_has_wall_without_tilted_ground_arc(self):
        x,y=np.meshgrid(np.linspace(-3,3,41),np.linspace(-3,3,41))
        floor=np.column_stack((x.ravel(),y.ravel(),(.05*x+.01*y).ravel()))
        y=np.linspace(-.8,.8,80)
        wall=np.column_stack((np.full(len(y),1.8),y,.05*1.8+.01*y+.12))
        transform=TransformStamped();transform.transform.rotation.w=1.
        self.node.buffer=SimpleNamespace(lookup_transform=lambda *args:transform)
        self.node.get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=100.01))
        cloud=create_cloud_xyz32(Header(stamp=Time(seconds=100.).to_msg(),frame_id='base_link'),
                                 np.vstack((floor,wall)))
        self.node.pending.append(cloud);self.node.tick()
        self.assertEqual(len(self.output),1)
        scan=self.output[-1];ranges=np.asarray(scan.ranges);valid=np.isfinite(ranges)
        angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
        self.assertGreater(np.count_nonzero(valid),10)
        # Every obstacle endpoint belongs to the true low wall, not a floor arc.
        np.testing.assert_allclose(ranges[valid]*np.cos(angles[valid]),1.8,atol=.02)
        self.assertTrue(np.all(np.isnan(ranges[~valid])))

    def test_cached_floor_moves_in_odom_and_expires(self):
        x,y=np.meshgrid(np.linspace(-3,3,41),np.linspace(-3,3,41))
        floor=np.column_stack((x.ravel(),y.ravel(),(.04*x+.01*y).ravel()))
        self.node.ground_relative(floor,floor,np.eye(3),np.zeros(3),100.)
        self.assertIsNotNone(self.node.ground_model)
        r=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        t=np.array([.2,.1,0.])
        world=np.array([[2.,1.,.04*2+.01+.12],[-2.,1.,-.04*2+.01+.16]])
        body=(world-t)@r
        normalized=self.node.ground_relative(body,body,r,t,100.2)
        np.testing.assert_allclose(normalized[:,2],[.12,.16],atol=1e-9)
        np.testing.assert_array_equal(normalized[:,:2],body[:,:2])
        expired=self.node.ground_relative(body,body,r,t,100.8)
        np.testing.assert_array_equal(expired,body)

    def test_disabled_or_missing_ground_preserves_original_slice(self):
        points=np.array([[2.,1.,.2],[1.,2.,.3]])
        no_floor=self.node.ground_relative(points,points,np.eye(3),np.zeros(3),100.)
        np.testing.assert_array_equal(no_floor,points)
        self.node.ground_enabled=False
        np.testing.assert_array_equal(self.node.ground_relative(
            points,points,np.eye(3),np.zeros(3),100.),points)


if __name__=='__main__':
    unittest.main()
