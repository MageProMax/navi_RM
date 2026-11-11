#!/usr/bin/env python3
"""Ground leakage regressions based on geometry, independent of ROS or hardware."""
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from ground_filter import estimate_ground, relative_height


def floor(a=.035,b=-.02,c=.01):
    x,y=np.meshgrid(np.linspace(-3,3,41),np.linspace(-3,3,41))
    z=a*x+b*y+c+np.random.default_rng(4).normal(0,.004,x.shape)
    return np.column_stack((x.ravel(),y.ravel(),z.ravel()))


class GroundFilterTest(unittest.TestCase):
    def test_tilted_floor_rejected_but_low_obstacle_preserved(self):
        ground=floor()
        self.assertTrue(np.any((ground[:,2]>=.1)&(ground[:,2]<=.5)))
        obstacle=np.array([[2.,1.,.035*2-.02+.01+.12],
                           [-2.,-1.,-.035*2+.02+.01+.16]])
        points=np.vstack((ground,obstacle))
        plane=estimate_ground(points)
        self.assertIsNotNone(plane)
        corrected=relative_height(points,plane)
        self.assertLess(np.abs(corrected[:-2,2]).max(),.025)
        np.testing.assert_allclose(corrected[-2:,2],[.12,.16],atol=.005)
        np.testing.assert_array_equal(corrected[:,:2],points[:,:2])

    def test_wall_and_outliers_do_not_rotate_ground_fit(self):
        ground=floor()
        y,z=np.meshgrid(np.linspace(-2,2,51),np.linspace(.1,.5,40))
        wall=np.column_stack((np.full(y.size,1.5),y.ravel(),z.ravel()))
        outliers=np.random.default_rng(7).uniform([-3,-3,-.3],[3,3,.3],(150,3))
        plane=estimate_ground(np.vstack((ground,wall,outliers)))
        self.assertIsNotNone(plane)
        np.testing.assert_allclose(plane,[.035,-.02,.01],atol=.007)

    def test_elevated_surface_cannot_become_floor(self):
        self.assertIsNone(estimate_ground(floor(0,0,.12)))
        self.assertIsNone(estimate_ground(floor(0,0,1.5)))

    def test_vertical_wall_and_narrow_line_have_no_ground(self):
        y,z=np.meshgrid(np.linspace(-2,2,51),np.linspace(-.1,.3,41))
        wall=np.column_stack((np.ones(y.size),y.ravel(),z.ravel()))
        self.assertIsNone(estimate_ground(wall))
        x=np.linspace(-3,3,100)
        self.assertIsNone(estimate_ground(np.column_stack((x,x*0,x*.02))))

    def test_steep_surface_is_not_filtered(self):
        self.assertIsNone(estimate_ground(floor(.4,0,0)))

    def test_sparse_and_nonfinite_data_fall_back(self):
        self.assertIsNone(estimate_ground(np.zeros((0,3))))
        self.assertIsNone(estimate_ground(np.zeros((8,3))))
        bad=np.full((50,3),np.nan)
        self.assertIsNone(estimate_ground(bad))
        self.assertIsNotNone(estimate_ground(np.vstack((floor(),bad))))


if __name__=='__main__':
    unittest.main()
