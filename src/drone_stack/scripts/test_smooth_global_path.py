#!/usr/bin/env python3
"""Offline checks for forward route progress and the executed-curve export."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
import rospy
from geometry_msgs.msg import Point
from scipy.interpolate import BSpline
from safe_global_path import SafeGlobalPath, ReferenceSafetyCells
from flight_manager import FlightManager

class SmoothReference(unittest.TestCase):
    def setUp(self):
        self.route=SafeGlobalPath.__new__(SafeGlobalPath)
        self.route.origin=[-15.,-15.,.5];self.route.res=.1
        self.route.lower=[-8.,-8.,.5];self.route.upper=[8.,8.,2.5]
        self.route.geometry=SimpleNamespace(map_origin=self.route.origin,map_resolution=.1)
        self.cells=(ReferenceSafetyCells(set()),set())

    def test_corner_has_continuous_forward_direction(self):
        route=self.route.smooth_route([(0,0,1.2),(2,0,1.2),(2,2,1.2)],self.cells)
        self.assertIsNotNone(route)
        d=np.diff(np.asarray(route),axis=0);d=d[np.linalg.norm(d,axis=1)>1e-7]
        unit=d/np.linalg.norm(d,axis=1)[:,None]
        angles=np.degrees(np.arccos(np.clip(np.sum(unit[:-1]*unit[1:],axis=1),-1,1)))
        self.assertLess(float(angles.max()),8.)
        self.assertTrue(self.route.route_safe(route,self.cells))

    def test_short_join_has_no_oversized_handles(self):
        tail=[(0.,0.,1.2),(.03,0.,1.2),(.06,0.,1.2)]
        route=self.route.connect_reference((0.,.04,1.2),tail,self.cells)
        self.assertGreater(len(route),2)
        d=np.diff(np.asarray(route),axis=0);u=d/np.linalg.norm(d,axis=1)[:,None]
        angles=np.degrees(np.arccos(np.clip(np.sum(u[:-1]*u[1:],axis=1),-1,1)))
        self.assertLess(float(angles.max()),8.)
        self.assertGreaterEqual(min(p[0] for p in route),-1e-8)
        self.assertLessEqual(max(p[0] for p in route),.06+1e-8)

    def test_passed_waypoint_is_removed(self):
        tail,progress=self.route.project_forward((2.2,0,1.2),[(0,0,1.2),(2,0,1.2),(4,0,1.2)])
        self.assertAlmostEqual(progress,2.2)
        self.assertTrue(all(p[0]>=2.2-1e-9 for p in tail))

    def test_progress_never_rewinds(self):
        route=self.route.smooth_route([(0,0,1.2),(2,0,1.2),(2,2,1.2)],self.cells)
        progress=0.
        for p in route[::9]:
            _,next_progress=self.route.project_forward(p,route,progress)
            self.assertGreaterEqual(next_progress,progress);progress=next_progress
        _,next_progress=self.route.project_forward(route[3],route,progress)
        self.assertGreaterEqual(next_progress,progress)

    def test_padding_matches_extra_collision_margin(self):
        cells=ReferenceSafetyCells({(0,0,0)})
        self.assertIn((1,0,0),cells)
        self.assertNotIn((2,0,0),cells)

    def test_no_unsafe_sharp_fallback(self):
        self.assertIsNone(self.route.smooth_route([(0,0,1.2),(1,0,1.2),(0,0,1.2)],self.cells))
        cells=(ReferenceSafetyCells({(160,150,7)}),set())
        self.assertIsNone(self.route.smooth_route([(0,0,1.2),(2,0,1.2)],cells))

    def test_export_is_exact_adopted_spline(self):
        f=FlightManager.__new__(FlightManager)
        f.executed_path_pub=Mock();f.local_display_curve=None;f.local_display_samples=None;f.local_display_last=rospy.Time(0)
        curve=SimpleNamespace(order=3,knots=[0,0,0,0,2,2,2,2],
                              pos_pts=[Point(0,0,1.2),Point(1,0,1.2),Point(1,1,1.2),Point(2,1,1.2)],start_time=rospy.Time(99))
        with patch('rospy.Time.now',return_value=rospy.Time(100)):
            f.publish_execution_curve(curve)
            msg=f.executed_path_pub.publish.call_args[0][0]
            expected=BSpline(curve.knots,[[p.x,p.y,p.z] for p in curve.pos_pts],3)(1.)
            p=msg.poses[0].pose.position
            np.testing.assert_allclose([p.x,p.y,p.z],expected,atol=1e-12)
            self.assertEqual(msg.header.frame_id,'odom')
            f.publish_execution_curve()
            self.assertEqual(len(f.executed_path_pub.publish.call_args[0][0].poses),0)

if __name__=='__main__':unittest.main()
