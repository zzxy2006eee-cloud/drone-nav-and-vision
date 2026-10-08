#!/usr/bin/env python3
"""Deterministic command/state safety tests; no MAVROS services are called."""
import importlib.util
import copy
import os
import itertools
import random
import math
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus, PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from quadrotor_msgs.msg import PositionCommand
from std_srvs.srv import SetBoolRequest

spec = importlib.util.spec_from_file_location('flight_manager', Path(__file__).with_name('flight_manager.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Safety(unittest.TestCase):
    def setUp(self):
        self.clock = patch('rospy.Time.now', return_value=rospy.Time(100))
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.f = module.FlightManager.__new__(module.FlightManager)
        f = self.f
        f.lock = threading.RLock()
        f.phase = 'HOLD'; f.authorized = True
        f.state = State(connected=True, armed=True, mode='OFFBOARD')
        f.extended = ExtendedState(landed_state=ExtendedState.LANDED_STATE_IN_AIR)
        f.odom = Odometry(); f.odom.header.stamp=rospy.Time(100)
        f.odom.pose.pose.position.z=1;f.odom.pose.pose.orientation.w=1
        f.lio_valid=True; f.lio_health_time=rospy.Time(100)
        f.startup_min_time=65;f.future_tolerance=.02;f.max_pose_age=.7
        f.clock_fault=False;f.last_clock=rospy.Time(100)
        f.goal_min=[-8,-8,.5];f.goal_max=[8,8,2.5];f.max_takeoff_height=2
        f.goal_tolerance=.15;f.goal_speed_tolerance=.15
        f.map_shape=[300,300,80];f.map_resolution=.1;f.map_origin=[-15,-15,.5];f.map_time=rospy.Time(100);f.occupied_cells=set()
        f.max_map_age=2.0;f.require_observed_free=False;f.observed_free=set();f.free_time=rospy.Time(0)
        f.max_traj_age=.5;f.max_setpoint_step=.75;f.max_command_speed=.75
        f.estimator=EstimatorStatus();f.estimator.header.stamp=rospy.Time(100)
        f.pending_estimator=None
        for flag in ['pos_horiz_rel_status_flag','pos_vert_abs_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag']:
            setattr(f.estimator,flag,True)
        f.approved_spline_id=4;f.pending_spline=None;f.approved_curve=None;f.prefix_check_time=rospy.Time(0);f.known_horizon=3.0
        f.latest_command_id=3;f.required_trajectory_id=4;f.pending_goal=None;f.active_goal=None
        f.enabled_pub=Mock();f.phase_pub=Mock();f.auth_pub=Mock();f.error_pub=Mock()
        f.setpoint_pub=Mock();f.hold_pose=f.copy_pose();f.trajectory=None;f.goal_reached_since=None
        f.planner_time=rospy.Time(100);f.max_planner_age=.3
        f.goal_pub=Mock();f.goal_start=rospy.Time(100);f.trajectory_time=rospy.Time(0)
        f.future_command_since=None
        f.heartbeat_pub=Mock()
        f.arm_client=Mock(return_value=SimpleNamespace(success=True))

    def test_planner_loss_stops_fresh_trajectory_without_waiting_for_traj_server(self):
        self.navigate()
        self.f.goal_start=rospy.Time(99)
        self.f.planner_time=rospy.Time(99.69)
        self.f.tick(None)
        self.assertEqual(self.f.phase,'HOLD')
        self.assertIn('planner heartbeat',self.f.last_error)
        self.f.setpoint_pub.publish.assert_called()

    def test_future_duplicate_and_stale_planner_messages_cannot_refresh_health(self):
        msg=SimpleNamespace(header=Header(stamp=rospy.Time(100)))
        self.f.planner_time=rospy.Time(99.9)
        self.f.on_planner_heartbeat(msg)
        self.assertEqual(self.f.planner_time,rospy.Time(100))
        for stamp in [100,100.001,99.95,99]:
            msg.header.stamp=rospy.Time(stamp)
            self.f.on_planner_heartbeat(msg)
            self.assertEqual(self.f.planner_time,rospy.Time(100))

    def test_new_goal_allows_bounded_planner_startup(self):
        self.navigate();self.f.planner_time=rospy.Time(0)
        self.f.tick(None)
        self.assertEqual(self.f.phase,'NAVIGATING')

    def test_command_waits_for_full_spline_approval(self):
        self.navigate()
        self.f.approved_spline_id=None
        self.f.on_trajectory(self.command())
        self.assertIsNone(self.f.trajectory)
        self.assertEqual(self.f.phase,'NAVIGATING')

    def test_whole_spline_voxel_collision_stops_before_execution(self):
        from ego_planner.msg import Bspline
        from geometry_msgs.msg import Point
        self.navigate()
        self.f.approved_spline_id=None
        self.f.occupied_cells={self.f.map_cell([.5,0,1])}
        b=Bspline(order=3,traj_id=4,start_time=rospy.Time(100),knots=[0,0,0,0,1,1,1,1],
                  pos_pts=[Point(x=x,y=0,z=1) for x in [0,1/3,2/3,1]])
        self.f.on_spline(b)
        self.assertEqual(self.f.phase,'HOLD')
        self.assertIn('Full EGO trajectory',self.f.last_error)
        self.assertIsNone(self.f.trajectory)

    def test_clear_whole_spline_approves_matching_commands(self):
        from ego_planner.msg import Bspline
        from geometry_msgs.msg import Point
        self.navigate()
        self.f.approved_spline_id=None
        b=Bspline(order=3,traj_id=4,start_time=rospy.Time(100),knots=[0,0,0,0,1,1,1,1],
                  pos_pts=[Point(x=x,y=0,z=1) for x in [0,1/3,2/3,1]])
        self.f.on_spline(b)
        self.assertEqual(self.f.approved_spline_id,4)
        self.f.on_trajectory(self.command())
        self.assertIsNotNone(self.f.trajectory)

    def spline(self, stamp):
        from ego_planner.msg import Bspline
        from geometry_msgs.msg import Point
        return Bspline(order=3,traj_id=4,start_time=stamp,knots=[0,0,0,0,1,1,1,1],
                       pos_pts=[Point(x=x,y=0,z=1) for x in [0,1/3,2/3,1]])

    def test_future_spline_waits_for_clock_before_commands(self):
        self.navigate();self.f.approved_spline_id=None
        self.f.on_spline(self.spline(rospy.Time(100,30000000)))
        self.assertEqual(self.f.phase,'NAVIGATING')
        self.assertIsNotNone(self.f.pending_spline)
        self.f.on_trajectory(self.command())
        self.assertIsNone(self.f.trajectory)
        with patch('rospy.Time.now',return_value=rospy.Time(100,30000000)):
            self.f.on_trajectory(self.command())
        self.assertEqual(self.f.approved_spline_id,4)
        self.assertIsNotNone(self.f.trajectory)

    def test_far_future_spline_rejected(self):
        self.navigate();self.f.approved_spline_id=None
        self.f.on_spline(self.spline(rospy.Time(101)))
        self.assertEqual(self.f.phase,'HOLD')
        self.assertIn('age=',self.f.last_error)

    def test_pending_spline_expired_before_activation_rejected(self):
        self.navigate();self.f.approved_spline_id=None
        self.f.on_spline(self.spline(rospy.Time(100,30000000)))
        self.f.odom.header.stamp=rospy.Time(101)
        self.f.lio_health_time=rospy.Time(101)
        self.f.estimator.header.stamp=rospy.Time(101)
        with patch('rospy.Time.now',return_value=rospy.Time(101)):
            self.f.activate_pending_spline()
        self.assertEqual(self.f.phase,'HOLD')
        self.assertIsNone(self.f.approved_spline_id)

    def test_pending_spline_never_activates_in_next_goal_generation(self):
        self.navigate();self.f.approved_spline_id=None
        self.f.on_spline(self.spline(rospy.Time(100,30000000)))
        self.f.required_trajectory_id=5
        with patch('rospy.Time.now',return_value=rospy.Time(100,30000000)):
            self.f.activate_pending_spline()
        self.assertIsNone(self.f.approved_spline_id)
        self.assertIsNone(self.f.pending_spline)

    def test_pending_spline_requires_fresh_map_at_activation(self):
        self.navigate();self.f.approved_spline_id=None
        self.f.on_spline(self.spline(rospy.Time(100,30000000)))
        self.f.map_time=rospy.Time(97)
        with patch('rospy.Time.now',return_value=rospy.Time(100,30000000)):
            self.f.activate_pending_spline()
        self.assertEqual(self.f.phase,'HOLD')
        self.assertIsNone(self.f.approved_spline_id)

    def test_unknown_goal_queued_without_executing_unchecked_curve(self):
        self.f.require_observed_free=True;self.f.free_time=rospy.Time(100)
        self.assertTrue(self.f.local_goal(self.goal()).success)
        self.assertEqual(self.f.phase,'NAVIGATING')
        self.assertIsNone(self.f.trajectory)

    def test_stale_free_map_rejects_goal(self):
        self.f.require_observed_free=True;self.f.free_time=rospy.Time(97)
        self.assertIn('fresh observed',self.f.local_goal(self.goal()).message)

    def test_unknown_full_curve_rejected(self):
        self.navigate();self.f.approved_spline_id=None
        self.f.require_observed_free=True;self.f.free_time=rospy.Time(100)
        self.f.on_spline(self.spline(rospy.Time(100)))
        self.assertEqual(self.f.phase,'HOLD')
        self.assertIn('unobserved',self.f.last_error)

    def test_lookahead_stops_before_unknown_future_motion(self):
        from trajectory_guard import PackedCells
        self.navigate();self.f.require_observed_free=True;self.f.free_time=rospy.Time(100)
        self.f.approved_curve=self.spline(rospy.Time(100))
        self.f.on_trajectory(self.command())
        self.assertEqual(self.f.phase,'HOLD')
        self.assertIsNone(self.f.trajectory)
        self.assertIn('lookahead',self.f.last_error)

    def test_lookahead_does_not_refresh_command_without_known_current_segment(self):
        self.navigate();self.f.require_observed_free=True;self.f.free_time=rospy.Time(100)
        self.f.approved_curve=self.spline(rospy.Time(100))
        self.f.observed_free={(x,y,z) for x in range(149,161) for y in [149,150] for z in [4,5]}
        self.f.on_trajectory(self.command())
        self.assertIsNotNone(self.f.trajectory)
        self.assertEqual(self.f.phase,'NAVIGATING')

    def test_unknown_current_segment_rejected(self):
        self.f.require_observed_free=True;self.f.free_time=rospy.Time(100)
        self.assertIn('unobserved',self.f.command_error(self.command()))

    def goal(self,x=2,y=0,z=1,frame='odom'):
        p=PoseStamped();p.header.frame_id=frame;p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=z
        return SimpleNamespace(goal=p)

    def command(self,x=.3,y=0,z=1):
        c=PositionCommand();c.header.stamp=rospy.Time(100);c.header.frame_id='odom'
        c.trajectory_id=4;c.trajectory_flag=PositionCommand.TRAJECTORY_STATUS_READY
        c.position.x=x;c.position.y=y;c.position.z=z
        return c

    def navigate(self):
        self.f.phase='NAVIGATING';self.f.active_goal=self.goal().goal

    def test_future_estimator_waits_without_refreshing_health(self):
        future=copy.deepcopy(self.f.estimator)
        future.header.stamp=rospy.Time(100,26000000)
        previous=self.f.estimator
        self.f.on_estimator(future)
        self.assertIs(self.f.estimator,previous)
        self.assertIs(self.f.pending_estimator,future)
        self.f.fresh_pose()
        self.assertIs(self.f.estimator,previous)
        with patch('rospy.Time.now',return_value=rospy.Time(100,26000000)):
            self.assertTrue(self.f.fresh_pose())
        self.assertIs(self.f.estimator,future)
        self.assertIsNone(self.f.pending_estimator)

    def test_large_future_estimator_never_queued(self):
        future=copy.deepcopy(self.f.estimator);future.header.stamp=rospy.Time(101)
        self.f.on_estimator(future)
        self.assertIsNone(self.f.pending_estimator)

    def test_clock_regression_latches_invalid_position(self):
        self.f.on_clock(SimpleNamespace(clock=rospy.Time(99)))
        self.assertTrue(self.f.clock_fault)
        self.assertFalse(self.f.fresh_pose())
        self.f.on_clock(SimpleNamespace(clock=rospy.Time(100)))
        self.assertFalse(self.f.fresh_pose())

    def test_small_clock_reordering_does_not_latch_fault(self):
        self.f.on_clock(SimpleNamespace(clock=rospy.Time(99,980000000)))
        self.assertFalse(self.f.clock_fault)

    def test_safe_trajectory_accepted(self):
        self.navigate();c=self.command();self.f.on_trajectory(c)
        self.assertIs(self.f.trajectory,c)
        self.assertEqual(self.f.trajectory_time,c.header.stamp)

    def test_expired_command_rejected_without_refreshing_receipt_time(self):
        self.navigate();c=self.command();c.header.stamp=rospy.Time(99)
        self.f.on_trajectory(c)
        self.assertEqual(self.f.phase,'HOLD');self.assertIsNone(self.f.trajectory)
        self.f.setpoint_pub.publish.assert_called_once()

    def test_future_command_rejected(self):
        self.navigate();c=self.command();c.header.stamp=rospy.Time.from_sec(100.1)
        self.f.on_trajectory(c);self.assertIsNone(self.f.trajectory)
        self.assertEqual(self.f.phase,'NAVIGATING')
        self.f.future_command_since=rospy.Time(99,500000000)
        self.f.tick(None);self.assertEqual(self.f.phase,'HOLD')
        self.f.setpoint_pub.publish.assert_called_once()

    def test_transient_future_command_keeps_last_valid_command(self):
        self.navigate();valid=self.command();self.f.on_trajectory(valid)
        ahead=self.command();ahead.header.stamp=rospy.Time(100,26000000)
        self.f.on_trajectory(ahead);self.assertIs(self.f.trajectory,valid)
        self.f.tick(None);self.assertEqual(self.f.phase,'NAVIGATING')
        self.f.setpoint_pub.publish.assert_called_once()
        self.f.on_trajectory(valid);self.assertIsNone(self.f.future_command_since)

    def test_large_future_command_cancels_immediately(self):
        self.navigate();c=self.command();c.header.stamp=rospy.Time(101)
        self.f.on_trajectory(c);self.assertEqual(self.f.phase,'HOLD')

    def test_future_fcu_sample_does_not_replace_valid_pose(self):
        previous=self.f.odom;msg=Odometry();msg.header.frame_id='map'
        msg.header.stamp=rospy.Time(100,26000000)
        self.f.on_odom(msg);self.assertIs(self.f.odom,previous)
        self.assertTrue(self.f.fresh_pose())

    def test_exact_future_boundary_command_accepted(self):
        self.navigate();c=self.command();c.header.stamp=rospy.Time(100,20000000)
        self.f.on_trajectory(c)
        self.assertIs(self.f.trajectory,c)
        self.assertEqual(self.f.phase,'NAVIGATING')

    def test_one_nanosecond_beyond_future_boundary_rejected(self):
        self.navigate();c=self.command();c.header.stamp=rospy.Time(100,20000001)
        self.f.on_trajectory(c);self.assertIsNone(self.f.trajectory)

    def test_exact_future_boundary_pose_and_map_accepted(self):
        self.f.odom.header.stamp=rospy.Time(100,20000000)
        self.f.estimator.header.stamp=rospy.Time(100,20000000)
        self.f.map_time=rospy.Time(100,20000000)
        self.assertTrue(self.f.fresh_pose());self.assertTrue(self.f.fresh_map())

    def test_old_generation_ignored(self):
        self.navigate();c=self.command();c.trajectory_id=3
        self.f.on_trajectory(c);self.assertIsNone(self.f.trajectory)
        self.assertEqual(self.f.phase,'NAVIGATING')

    def test_bounds_and_nan_commands_cancel_navigation(self):
        for values in [(.3,0,2.6),(.3,0,.4),(8.1,0,1),(float('nan'),0,1)]:
            with self.subTest(values=values):
                self.navigate();self.f.on_trajectory(self.command(*values))
                self.assertEqual(self.f.phase,'HOLD');self.assertIsNone(self.f.trajectory)

    def test_setpoint_jump_rejected(self):
        self.navigate();self.f.on_trajectory(self.command(1.1))
        self.assertEqual(self.f.phase,'HOLD')
        self.assertIn('jump',self.f.last_error)

    def test_navigation_uses_enu_velocity_feedforward(self):
        self.navigate();c=self.command();c.velocity.x=.3;c.velocity.y=-.2;c.velocity.z=.1
        self.f.on_trajectory(c);self.f.tick(None)
        target=self.f.setpoint_pub.publish.call_args[0][0]
        self.assertEqual([target.velocity.x,target.velocity.y,target.velocity.z],[.3,-.2,.1])
        self.assertEqual(target.type_mask & (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ),0)
        self.assertNotEqual(target.type_mask & PositionTarget.IGNORE_AFX,0)

    def test_nonfinite_or_excessive_velocity_cancelled(self):
        for speed in [float('nan'),float('inf'),.751]:
            with self.subTest(speed=speed):
                self.navigate();c=self.command();c.velocity.x=speed
                self.f.on_trajectory(c);self.assertEqual(self.f.phase,'HOLD')

    def test_occupied_endpoint_rejected(self):
        self.navigate();self.f.occupied_cells.add(self.f.map_cell([.3,0,1]))
        self.f.on_trajectory(self.command());self.assertEqual(self.f.phase,'HOLD')

    def test_collision_between_free_endpoints_rejected(self):
        self.navigate();self.f.occupied_cells.add(self.f.map_cell([.25,0,1]))
        self.f.on_trajectory(self.command(.6))
        self.assertEqual(self.f.phase,'HOLD')
        self.assertIn('intersects',self.f.last_error)

    def test_new_obstacle_rechecked_before_execution(self):
        self.navigate();self.f.on_trajectory(self.command(.6))
        self.f.occupied_cells.add(self.f.map_cell([.25,0,1]))
        self.f.tick(None)
        self.assertEqual(self.f.phase,'HOLD')
        self.f.setpoint_pub.publish.assert_called_once()
        self.assertEqual(self.f.setpoint_pub.publish.call_args[0][0].position.x,0)

    def test_trajectory_stream_loss_holds_and_continues_setpoints(self):
        self.navigate();self.f.trajectory=self.command();self.f.trajectory_time=rospy.Time(99)
        self.f.tick(None);self.assertEqual(self.f.phase,'HOLD')
        self.f.setpoint_pub.publish.assert_called_once()

    def test_no_initial_trajectory_times_out_with_setpoint(self):
        self.navigate();self.f.goal_start=rospy.Time(94)
        self.f.tick(None);self.assertEqual(self.f.phase,'HOLD')
        self.f.setpoint_pub.publish.assert_called_once()

    def test_goal_arrival_holds_without_warning_and_keeps_stream(self):
        self.navigate();self.f.active_goal=self.goal(0,0,1).goal
        self.f.goal_reached_since=rospy.Time(98)
        self.f.tick(None);self.assertEqual(self.f.phase,'HOLD')
        self.f.setpoint_pub.publish.assert_called_once();self.f.error_pub.publish.assert_not_called()

    def test_goal_arrival_holds_exact_goal_instead_of_remaining_error(self):
        self.navigate();self.f.active_goal=self.goal(0,0,1).goal
        self.f.odom.pose.pose.position.x=.09;self.f.odom.pose.pose.position.z=1.04
        self.f.goal_reached_since=rospy.Time(98)
        self.f.tick(None)
        self.assertEqual(self.f.phase,'HOLD')
        self.assertEqual(self.f.hold_pose.position.x,0)
        self.assertEqual(self.f.hold_pose.position.z,1)
        command=self.f.setpoint_pub.publish.call_args.args[0]
        self.assertEqual(command.position.x,0);self.assertEqual(command.position.z,1)

    def test_goal_arrival_rechecks_new_occupancy_before_holding_goal(self):
        self.navigate();self.f.active_goal=self.goal(0,0,1).goal
        self.f.occupied_cells={self.f.map_cell([0,0,1])}
        self.f.goal_reached_since=rospy.Time(98);self.f.start_landing=Mock()
        self.f.tick(None)
        self.f.start_landing.assert_called_once()
        self.assertIn('Goal holding segment is unsafe',self.f.start_landing.call_args.args[0])
        self.f.setpoint_pub.publish.assert_not_called()

    def test_corner_tie_visits_side_voxels(self):
        self.f.map_origin=[0,0,0];self.f.map_resolution=1
        cells=set(self.f.segment_cells([.5,.5,.5],[1.5,1.5,1.5]))
        self.assertTrue(set(itertools.product([0,1],repeat=3))<=cells)

    def test_line_on_face_checks_both_sides(self):
        self.f.map_origin=[0,0,0];self.f.map_resolution=1
        cells=set(self.f.segment_cells([.5,1,.5],[1.5,1,.5]))
        self.assertTrue({(0,0,0),(0,1,0),(1,0,0),(1,1,0)}<=cells)

    def test_negative_direction_boundary_endpoint(self):
        self.f.map_origin=[0,0,0];self.f.map_resolution=1
        self.assertEqual(set(self.f.segment_cells([1.5,.5,.5],[1,.5,.5])),{(1,0,0),(0,0,0)})

    def test_stationary_corner_checks_all_neighbours(self):
        self.f.map_origin=[0,0,0];self.f.map_resolution=1
        self.assertEqual(set(self.f.segment_cells([1,1,1],[1,1,1])),set(itertools.product([0,1],repeat=3)))

    def test_grid_conversion_roundoff_at_face(self):
        cells=set(self.f.segment_cells([-14.9,-14.95,.55],[-14.9,-14.95,.55]))
        self.assertEqual(cells,{(0,0,0),(1,0,0)})

    def test_random_segments_match_independent_closed_box_intersections(self):
        self.f.map_origin=[0,0,0];self.f.map_resolution=1
        rng=random.Random(17)
        for _ in range(500):
            a=[rng.uniform(-2,2) for _ in range(3)];b=[rng.uniform(-2,2) for _ in range(3)]
            low=[math.floor(min(u,v))-1 for u,v in zip(a,b)]
            high=[math.floor(max(u,v))+1 for u,v in zip(a,b)]
            expected=set()
            for cell in itertools.product(*(range(lo,hi+1) for lo,hi in zip(low,high))):
                enter,leave=0.,1.
                for i in range(3):
                    t0=(cell[i]-a[i])/(b[i]-a[i]);t1=(cell[i]+1-a[i])/(b[i]-a[i])
                    enter=max(enter,min(t0,t1));leave=min(leave,max(t0,t1))
                if enter<=leave:expected.add(cell)
            self.assertEqual(set(self.f.segment_cells(a,b)),expected)

    def test_localizing_heartbeat_does_not_claim_ready(self):
        self.f.phase='LOCALIZING';self.f.on_state(State(connected=True,armed=False))
        self.assertEqual(self.f.phase,'LOCALIZING')

    def test_small_future_clock_tolerated(self):
        self.f.odom.header.stamp=rospy.Time.from_sec(100.004)
        self.assertTrue(self.f.fresh_pose())

    def test_large_future_clock_rejected(self):
        self.f.odom.header.stamp=rospy.Time.from_sec(100.1)
        self.assertFalse(self.f.fresh_pose())

    def test_stale_health_rejected(self):
        self.f.lio_health_time=rospy.Time(99)
        self.assertFalse(self.f.fresh_pose())

    def test_stale_fcu_pose_rejected(self):
        self.f.odom.header.stamp=rospy.Time(98)
        self.assertFalse(self.f.fresh_pose())

    def test_invalid_estimator_rejected(self):
        self.f.estimator.velocity_vert_status_flag=False
        self.assertFalse(self.f.fresh_pose())

    def test_bounds_and_nan_rejected(self):
        for args in [(float('nan'),0,1),(2,0,.2),(2,0,3),(20,0,1),(2,0,1,'map')]:
            with self.subTest(args=args):self.assertFalse(self.f.local_goal(self.goal(*args)).success)
        self.f.enabled_pub.publish.assert_not_called()

    def test_unauthorized_goal_rejected(self):
        self.f.authorized=False
        self.assertFalse(self.f.local_goal(self.goal()).success)

    def test_occupied_goal_rejected(self):
        self.f.occupied_cells.add(self.f.map_cell([2,0,1]))
        self.assertFalse(self.f.local_goal(self.goal()).success)

    def test_stale_map_rejected(self):
        self.f.map_time=rospy.Time(95)
        self.assertFalse(self.f.local_goal(self.goal()).success)

    def test_zero_map_stamp_rejected(self):
        self.f.map_time=rospy.Time(0)
        self.assertFalse(self.f.local_goal(self.goal()).success)

    def test_future_map_stamp_rejected(self):
        self.f.map_time=rospy.Time.from_sec(100.1)
        self.assertFalse(self.f.local_goal(self.goal()).success)

    def test_map_callback_preserves_acquisition_time(self):
        msg=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time(99),frame_id='odom'),[(2.05,.05,1.05)])
        self.f.on_map(msg)
        self.assertEqual(self.f.map_time,rospy.Time(99))
        self.assertIn(self.f.map_cell([2.05,.05,1.05]),self.f.occupied_cells)

    def test_wrong_map_frame_does_not_replace_valid_map(self):
        msg=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time(99),frame_id='map'),[(2,0,1)])
        self.f.on_map(msg)
        self.assertEqual(self.f.map_time,rospy.Time(100))
        self.assertFalse(self.f.occupied_cells)

    def test_republication_does_not_refresh_stale_map(self):
        msg=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time(97),frame_id='odom'),[(2,0,1)])
        self.f.on_map(msg)
        self.f.on_map(msg)
        self.assertFalse(self.f.local_goal(self.goal(3)).success)

    def test_stale_map_during_navigation_holds_and_streams_setpoint(self):
        self.f.phase='NAVIGATING';self.f.map_time=rospy.Time(95)
        self.f.active_goal=self.goal().goal;self.f.pending_goal=self.goal().goal
        self.f.tick(None)
        self.assertEqual(self.f.phase,'HOLD')
        self.assertIsNone(self.f.pending_goal)
        self.assertIsNone(self.f.active_goal)
        self.f.setpoint_pub.publish.assert_called_once()
        self.assertIn('Obstacle map became stale',self.f.last_error)

    @unittest.skipUnless(os.environ.get('DRONE_MAP_FIXTURE_DIR'), 'Requires compiled C++ map fixtures')
    def test_cpp_map_serialization_in_python_flight_manager(self):
        directory=Path(os.environ['DRONE_MAP_FIXTURE_DIR'])
        legacy=PointCloud2().deserialize((directory/'legacy_zero_map.bin').read_bytes())
        self.f.on_map(legacy)
        self.assertFalse(self.f.local_goal(self.goal(3)).success)
        fresh=PointCloud2().deserialize((directory/'fresh_map.bin').read_bytes())
        self.f.on_map(fresh)
        self.assertEqual(self.f.map_time,rospy.Time(99,950123456))
        self.assertFalse(self.f.local_goal(self.goal(2.05,.05,1.05)).success)
        self.assertTrue(self.f.local_goal(self.goal(3)).success)

    def test_valid_goal_advances_trajectory_generation(self):
        self.assertTrue(self.f.local_goal(self.goal()).success)
        self.assertEqual(self.f.phase,'NAVIGATING');self.assertEqual(self.f.required_trajectory_id,4)

    def test_airborne_disarm_rejected(self):
        self.assertFalse(self.f.disarm(None).success)
        self.f.arm_client.assert_not_called()

    def test_ground_disarm_allowed(self):
        self.f.extended.landed_state=ExtendedState.LANDED_STATE_ON_GROUND
        self.assertTrue(self.f.disarm(None).success)
        self.f.arm_client.assert_called_once_with(False)

    def test_ground_contact_alone_does_not_complete_landing(self):
        self.f.phase='LANDING'
        self.f.extended.landed_state=ExtendedState.LANDED_STATE_ON_GROUND
        self.f.tick(None)
        self.assertEqual(self.f.phase,'LANDING')

    def test_ground_landing_completes_only_after_disarming(self):
        self.f.phase='LANDING';self.f.state.armed=False
        self.f.extended.landed_state=ExtendedState.LANDED_STATE_ON_GROUND
        self.f.tick(None)
        self.assertEqual(self.f.phase,'READY')

    def test_invalid_takeoff_rejected(self):
        self.f.phase='ARMED'
        for height in [-1,0,float('nan'),3]:
            self.assertFalse(self.f.takeoff(SimpleNamespace(height_m=height)).success)

    def test_repeated_takeoff_rejected(self):
        self.f.phase='TAKEOFF'
        self.assertFalse(self.f.takeoff(SimpleNamespace(height_m=1)).success)

    def test_ground_authorization_revoke_disarms(self):
        self.f.phase='ARMED';self.f.extended.landed_state=ExtendedState.LANDED_STATE_ON_GROUND
        self.f.set_authorized(SetBoolRequest(data=False))
        self.f.arm_client.assert_called_once_with(False)
        self.assertFalse(self.f.authorized)

    def test_airborne_authorization_revoke_lands(self):
        self.f.start_landing=Mock()
        self.f.set_authorized(SetBoolRequest(data=False))
        self.f.start_landing.assert_called_once()


if __name__ == '__main__':
    unittest.main(verbosity=2)
