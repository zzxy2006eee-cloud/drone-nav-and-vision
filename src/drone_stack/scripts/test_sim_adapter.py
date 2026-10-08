#!/usr/bin/env python3
"""Exercise geometry fault without simulator truth or changing scan timing."""
import copy
import math
import unittest
from unittest.mock import Mock
import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud,Imu
from std_srvs.srv import SetBoolRequest
from sim_livox_adapter import Adapter


class AdapterTests(unittest.TestCase):
    def setUp(self):
        rospy.rostime.set_rostime_initialized(True)
        self.a=Adapter.__new__(Adapter)
        self.a.publisher=Mock();self.a.imu_publisher=Mock()
        self.a.drop_lidar=False;self.a.drop_imu=False;self.a.geometry_loss=False
        self.a.startup_settle=2.;self.a.step=1;self.a.scan_period_ns=100000000
        self.a.min_useful_range=.45;self.a.max_range=40.
        self.cloud=PointCloud();self.cloud.header.stamp=rospy.Time.from_sec(10.)
        # Nominal R_y(30) sensor mounting: horizontal walls, floor and roof.
        for x,y,z in [(3,0,0),(0,3,0),(2,0,-1),(2,0,1)]:
            self.cloud.points.append(Point32(math.sqrt(.75)*x-.5*z,y,.5*x+math.sqrt(.75)*z))

    def test_normal_keeps_all_returns(self):
        self.a.on_cloud(copy.deepcopy(self.cloud))
        out=self.a.publisher.publish.call_args[0][0]
        self.assertEqual(out.point_num,4)
        self.assertEqual(out.header.stamp,rospy.Time.from_sec(9.9))
        self.assertTrue(all(p.offset_time==100000000 for p in out.points))

    def test_fault_keeps_wall_returns_and_timing(self):
        self.a.set_geometry_fault(SetBoolRequest(data=True))
        self.a.on_cloud(copy.deepcopy(self.cloud))
        out=self.a.publisher.publish.call_args[0][0]
        self.assertEqual(out.point_num,2)
        self.assertEqual([p.line for p in out.points],[0,1])
        self.assertEqual(out.header.stamp,rospy.Time.from_sec(9.9))
        self.assertTrue(all(p.offset_time==100000000 for p in out.points))

    def test_fault_preserves_imu(self):
        self.a.geometry_loss=True
        msg=Imu();msg.header.stamp=rospy.Time.from_sec(10.)
        self.a.on_imu(msg)
        self.a.imu_publisher.publish.assert_called_once_with(msg)

    def test_restore_keeps_normal_geometry(self):
        self.a.set_geometry_fault(SetBoolRequest(data=True))
        self.a.set_geometry_fault(SetBoolRequest(data=False))
        self.a.on_cloud(copy.deepcopy(self.cloud))
        self.assertEqual(self.a.publisher.publish.call_args[0][0].point_num,4)

if __name__=='__main__':unittest.main()
