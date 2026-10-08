#!/usr/bin/env python3
"""Offline model, coordinate, image and LIO bridge contract checks."""
import importlib.util
import math
from pathlib import Path
from collections import deque
from unittest.mock import Mock, patch
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import rospy
import tf.transformations as tr
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('lio_bridge', ROOT/'scripts/lio_bridge.py')
bridge_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge_module)


class Contracts(unittest.TestCase):
    def setUp(self):
        self.clock = patch('rospy.Time.now', return_value=rospy.Time(100))
        self.clock.start(); self.addCleanup(self.clock.stop)
        self.model = ET.parse(ROOT/'models/inspection_quad/inspection_quad.sdf.jinja').getroot()
        self.b = bridge_module.LioBridge.__new__(bridge_module.LioBridge)
        b = self.b
        b.max_age=.6; b.future_tolerance=.02; b.last_stamp=rospy.Time(0)
        b.pose_fault=False;b.max_pose_step=.5;b.max_pose_speed=3;b.max_rotation_step=.3;b.max_rotation_speed=4
        b.last_clock=rospy.Time(100)
        b.world_align=tr.euler_matrix(0,math.pi/6,0)
        b.base_sensor=tr.translation_matrix([.27,0,.1])@b.world_align
        b.sensor_base=np.linalg.inv(b.base_sensor)
        b.pose_cov=[.0025,.0025,.0025,.01,.01,.0025]
        b.lio_poses=deque(maxlen=30); b.fcu_poses=deque(maxlen=100)
        for name in ['odom_pub','mavros_pub','cloud_pub','cloud_pose_pub','fcu_cloud_pub','fcu_odom_pub','valid_pub','tf_pub']:
            setattr(b,name,Mock())

    def observation(self):
        msg=Odometry();msg.header.frame_id='camera_init';msg.header.stamp=rospy.Time(100)
        msg.pose.pose.orientation.w=1
        return msg

    def test_degenerate_native_covariance_stops_external_pose(self):
        m=self.observation();m.pose.covariance[14]=1e6
        self.b.on_odom(m)
        self.assertTrue(self.b.pose_fault)
        self.assertFalse(self.b.valid_pub.publish.call_args.args[0].data)
        self.b.mavros_pub.publish.assert_not_called()
        self.b.on_odom(self.observation())
        self.b.mavros_pub.publish.assert_not_called()

    def test_clock_regression_latches_bridge_fault(self):
        from rosgraph_msgs.msg import Clock
        self.b.on_clock(Clock(clock=rospy.Time(99)))
        self.assertTrue(self.b.pose_fault)
        self.assertFalse(self.b.valid_pub.publish.call_args.args[0].data)
        self.b.on_clock(Clock(clock=rospy.Time(100)))
        self.b.on_odom(self.observation())
        self.b.mavros_pub.publish.assert_not_called()

    def test_acquisition_time_regression_latches_bridge_fault(self):
        self.b.on_odom(self.observation())
        self.b.mavros_pub.reset_mock()
        msg=self.observation();msg.header.stamp=rospy.Time(99,900000000)
        self.b.on_odom(msg)
        self.assertTrue(self.b.pose_fault)
        self.b.mavros_pub.publish.assert_not_called()

    def test_duplicate_pose_does_not_latch_fault(self):
        self.b.on_odom(self.observation())
        self.b.on_odom(self.observation())
        self.assertFalse(self.b.pose_fault)

    def test_sensor_mount_and_independent_imu_match(self):
        lidar=self.model.find(".//sensor[@name='mid360_sim']")
        imu=self.model.find(".//sensor[@name='mid360_imu']")
        pose=[float(x) for x in lidar.findtext('pose').split()]
        np.testing.assert_allclose(pose,[.27,0,.1,0,math.pi/6,0],atol=1e-9)
        self.assertEqual(lidar.findtext('pose'),imu.findtext('pose'))
        self.assertNotEqual(imu.findtext('pose'),self.model.find(".//sensor[@name='px4_imu']").findtext('pose'))

    def test_lidar_axes_pitch_down(self):
        np.testing.assert_allclose(self.b.world_align[:3,:3]@[1,0,0],[math.sqrt(3)/2,0,-.5],atol=1e-12)

    def test_forward_downward_rays_clear_vehicle_body(self):
        # A centrally mounted sensor was only 45 mm above the body box:
        # most downward rays hit the vehicle, losing ground at higher flight.
        sensor=self.model.find(".//sensor[@name='mid360_sim']")
        xyz=np.array([float(v) for v in sensor.findtext('pose').split()[:3]])
        collision=self.model.find(".//collision[@name='base_link_inertia_collision']")
        half=np.array([float(v)/2 for v in collision.findtext('geometry/box/size').split()])
        self.assertGreater(xyz[0],half[0])
        for elevation in np.linspace(-7,20,12):
            direction=self.b.world_align[:3,:3]@np.array([math.cos(math.radians(elevation)),0,math.sin(math.radians(elevation))])
            self.assertGreater(direction[0],0)
            self.assertLess(direction[2],0)
            # Positive-X rays start in front of the body's front plane and
            # move away from it for their entire path.
            self.assertTrue(np.all(xyz[0]+np.linspace(0,40,100)*direction[0]>half[0]))

    def test_cloud_static_obstacles_are_retained(self):
        launch=ET.parse(ROOT/'launch/ego.launch').getroot()
        parameter=launch.find("param[@name='/ego_planner_node/grid_map/retain_cloud_obstacles']")
        self.assertIsNotNone(parameter)
        self.assertEqual(parameter.get('value'),'true')

    def test_cloud_memory_waits_for_localization_ready(self):
        launch=ET.parse(ROOT/'launch/ego.launch').getroot()
        parameter=launch.find("param[@name='/ego_planner_node/grid_map/cloud_memory_start_topic']")
        self.assertIsNotNone(parameter)
        self.assertEqual(parameter.get('value'),'/drone/flight_state')

    def test_lidar_simulation_scan_contract(self):
        lidar=self.model.find(".//sensor[@name='mid360_sim']")
        self.assertEqual(float(lidar.findtext('update_rate')),10)
        self.assertEqual(int(lidar.findtext('ray/scan/horizontal/samples'))*int(lidar.findtext('ray/scan/vertical/samples')),20160)
        self.assertAlmostEqual(math.degrees(float(lidar.findtext('ray/scan/vertical/min_angle'))),-7,places=5)
        self.assertAlmostEqual(math.degrees(float(lidar.findtext('ray/scan/vertical/max_angle'))),52,places=5)
        self.assertEqual(float(lidar.findtext('ray/range/max')),40)

    def test_no_gps_barometer_magnetometer_sensors(self):
        self.assertFalse({x.get('type') for x in self.model.findall('.//sensor')} & {'gps','altimeter','magnetometer'})
        names=' '.join(x.get('filename','') for x in self.model.findall('.//plugin')).lower()
        for forbidden in ['gps_plugin','barometer_plugin','magnetometer_plugin']:self.assertNotIn(forbidden,names)

    def test_simulation_fusion_parameters(self):
        params={}
        for line in (ROOT/'config/10052_gazebo-classic_inspection_quad').read_text().splitlines():
            parts=line.split()
            if len(parts)==4 and parts[:2]==['param','set']:params[parts[2]]=float(parts[3])
        for name,value in {'EKF2_EV_CTRL':11,'EKF2_HGT_REF':3,'EKF2_GPS_CTRL':0,'EKF2_BARO_CTRL':0,'SYS_HAS_BARO':0,'SYS_HAS_MAG':0}.items():
            self.assertEqual(params[name],value)
        self.assertEqual(int(params['EKF2_EV_CTRL'])&4,0)
        self.assertEqual(params['BAT1_V_LOAD_DROP'],0)
        self.assertEqual(params['COM_LOW_BAT_ACT'],2)

    def test_sensor_pose_becomes_body_pose_without_ned_conversion(self):
        desired=tr.translation_matrix([2,3,1])@tr.euler_matrix(.1,-.05,.7)
        sensor=np.linalg.inv(self.b.world_align)@desired@self.b.base_sensor
        msg=self.observation();msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=sensor[:3,3]
        q=tr.quaternion_from_matrix(sensor)
        msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=q
        self.b.on_odom(msg)
        out=self.b.mavros_pub.publish.call_args[0][0]
        np.testing.assert_allclose(bridge_module.LioBridge.pose_matrix(out.pose.pose),desired,atol=1e-12)
        self.assertEqual(out.header.frame_id,'odom');self.assertEqual(out.child_frame_id,'base_link')

    def test_output_does_not_advertise_lio_velocity(self):
        self.b.on_odom(self.observation())
        msg=self.b.mavros_pub.publish.call_args[0][0]
        self.assertEqual(msg.twist.twist.linear.x,0)
        for i in range(6):self.assertEqual(msg.twist.covariance[i*7],1e6)

    def test_stale_and_future_lio_poses_not_forwarded(self):
        for stamp in [99,100.1]:
            msg=self.observation();msg.header.stamp=rospy.Time.from_sec(stamp)
            self.b.on_odom(msg)
        self.b.mavros_pub.publish.assert_not_called()

    def test_future_pose_is_not_marked_healthy(self):
        self.b.last_stamp=rospy.Time(101);self.b.on_health(None)
        self.assertFalse(self.b.valid_pub.publish.call_args[0][0].data)

    def test_invalid_quaternion_not_forwarded(self):
        msg=self.observation();msg.pose.pose.orientation.w=2;self.b.on_odom(msg)
        self.b.mavros_pub.publish.assert_not_called()

    def test_position_jump_not_forwarded_and_fault_remains_latched(self):
        self.b.on_odom(self.observation());self.b.mavros_pub.reset_mock()
        msg=self.observation();msg.header.stamp=rospy.Time.from_sec(100.01);msg.pose.pose.position.x=5
        self.b.on_odom(msg)
        self.assertTrue(self.b.pose_fault);self.b.mavros_pub.publish.assert_not_called()
        self.assertFalse(self.b.valid_pub.publish.call_args[0][0].data)
        self.b.on_health(None);self.assertFalse(self.b.valid_pub.publish.call_args[0][0].data)
        self.b.on_odom(self.observation());self.b.mavros_pub.publish.assert_not_called()

    def test_rotation_jump_not_forwarded(self):
        self.b.on_odom(self.observation());self.b.mavros_pub.reset_mock()
        msg=self.observation();msg.header.stamp=rospy.Time.from_sec(100.01)
        msg.pose.pose.orientation.z=math.sqrt(.5);msg.pose.pose.orientation.w=math.sqrt(.5)
        self.b.on_odom(msg);self.assertTrue(self.b.pose_fault)
        self.b.mavros_pub.publish.assert_not_called()

    def test_small_motion_not_treated_as_pose_jump(self):
        self.b.on_odom(self.observation());self.b.mavros_pub.reset_mock()
        msg=self.observation();msg.header.stamp=rospy.Time.from_sec(100.01);msg.pose.pose.position.x=.01
        self.b.on_odom(msg)
        self.assertFalse(self.b.pose_fault);self.b.mavros_pub.publish.assert_called_once()

    def test_cloud_numeric_coordinates_match_fcu_body_pose(self):
        lio=tr.translation_matrix([2,3,1])@tr.euler_matrix(0,0,.2)
        fcu=tr.translation_matrix([5,-1,1.2])@tr.euler_matrix(0,0,.6)
        self.b.lio_poses.append((rospy.Time(100),lio));self.b.fcu_poses.append((rospy.Time(100),fcu));self.b.last_stamp=rospy.Time(100)
        point=np.array([3,3,1]); raw=np.linalg.inv(self.b.world_align)@np.r_[point,1]
        msg=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time(100),frame_id='camera_init'),[raw[:3]])
        self.b.on_cloud(msg)
        out=self.b.fcu_cloud_pub.publish.call_args[0][0]
        expected=fcu@np.linalg.inv(lio)@np.r_[point,1]
        np.testing.assert_allclose(list(point_cloud2.read_points(out,field_names=('x','y','z')))[0],expected[:3],atol=1e-6)
        self.assertEqual(out.header.stamp,msg.header.stamp);self.assertEqual(out.header.frame_id,'odom')
        paired=self.b.cloud_pose_pub.publish.call_args.args[0]
        self.assertEqual(paired.header.stamp,out.header.stamp)
        self.assertEqual(paired.header.frame_id,out.header.frame_id)
        np.testing.assert_allclose([paired.pose.position.x,paired.pose.position.y,paired.pose.position.z],fcu[:3,3])

    def test_unsynchronized_cloud_not_sent_to_planner(self):
        self.b.last_stamp=rospy.Time(100)
        msg=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time(100),frame_id='camera_init'),[(1,0,0)])
        self.b.on_cloud(msg);self.b.fcu_cloud_pub.publish.assert_not_called()

    def test_free_space_body_extent_matches_sim_body(self):
        node=self.model.find(".//link[@name='base_link']/collision[@name='base_link_inertia_collision']/geometry/box/size")
        size=np.array([float(v) for v in node.text.split()])
        tree=ET.parse(ROOT/'launch/ego.launch')
        import yaml
        extent=yaml.safe_load(tree.find("rosparam[@param='/ego_planner_node/grid_map/cloud_body_half_extent']").text)
        np.testing.assert_allclose(size,2*np.array(extent))
        config=yaml.safe_load((ROOT/'config/faster_lio_sim.yaml').read_text())
        self.assertTrue(config['publish']['dense_publish_en'])

    def test_camera_optical_axes(self):
        tree=ET.parse(ROOT/'launch/cameras.launch')
        poses={n.get('name'):[float(x) for x in n.get('args').split()[:6]] for n in tree.findall("node[@type='static_transform_publisher']")}
        for name,expected in [('front',[1,0,0]),('down',[0,0,-1])]:
            def rotation(key):
                p=poses[key];return tr.euler_matrix(p[5],p[4],p[3])[:3,:3]
            np.testing.assert_allclose(rotation('body_to_'+name+'_camera')@rotation(name+'_camera_optical')@[0,0,1],expected,atol=1e-9)

    def test_faultable_nodes_do_not_teardown_other_nodes(self):
        for filename in ['stack.launch','lio.launch']:
            for node in ET.parse(ROOT/'launch'/filename).findall('node'):
                if node.get('name') in ['laserMapping','drone_lio_bridge','drone_flight_manager','mavros']:
                    self.assertNotEqual(node.get('required'),'true')

    def test_mavros_isolated_node_retains_standard_px4_configuration(self):
        node=ET.parse(ROOT/'launch/stack.launch').find("node[@name='mavros']")
        self.assertIsNotNone(node)
        self.assertEqual(node.get('required'),'false')
        files={x.get('file') for x in node.findall('rosparam')}
        self.assertEqual(files,{'$(find mavros)/launch/px4_config.yaml','$(find mavros)/launch/px4_pluginlists.yaml'})


if __name__ == '__main__':
    unittest.main(verbosity=2)
