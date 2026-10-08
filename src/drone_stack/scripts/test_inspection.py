#!/usr/bin/env python3
"""Offline geometry and safety-state regressions. Does not create ROS nodes."""
import math
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
from inspection_geometry import (inside,validate_polygon,strips,orient_strips,raster,
    footprint,automatic_spacing,split_free,forward_projection,route_window)


CAMERA=dict(width=640,height=480,fx=381.36114963,fy=381.36114963,cx=320.,cy=240.,
    xyz_body=[0.,0.,-.15],optical_rotation_body=[0.,-1.,0.,-1.,0.,0.,0.,0.,-1.])


class GeometryTests(unittest.TestCase):
    def test_rectangle_area(self):
        _,area=validate_polygon([[0,0],[3,0],[3,2],[0,2]])
        self.assertAlmostEqual(area,6.)

    def test_self_intersection_rejected(self):
        with self.assertRaises(ValueError):validate_polygon([[0,0],[2,2],[0,2],[2,0]])

    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError):validate_polygon([[0,0],[2,0],[0,float('nan')]])

    def test_spacing_is_exact(self):
        lanes=strips([[0,0],[3,0],[3,2.1],[0,2.1]],.5,0.)
        self.assertEqual(len(lanes),5)
        self.assertTrue(np.allclose(np.diff([a[1] for a,b in lanes]),.5))
        self.assertGreater(lanes[0][1][0],lanes[0][0][0])
        self.assertLess(lanes[1][1][0],lanes[1][0][0])

    def test_narrow_polygon_one_lane(self):
        self.assertEqual(len(strips([[0,0],[2,0],[2,.2],[0,.2]],.5,0.)),1)

    def test_concave_region_clips_lanes(self):
        polygon=[[0,0],[3,0],[3,3],[2,3],[2,1],[1,1],[1,3],[0,3]]
        lanes=strips(polygon,.5,0.)
        for a,b in lanes:self.assertTrue(inside([(a+b)/2],polygon)[0])

    def test_entry_variants(self):
        lanes=strips([[0,0],[2,0],[2,2],[0,2]],.5,0.)
        variants=[orient_strips(lanes,i) for i in range(4)]
        self.assertEqual(len({tuple(v[0][0]) for v in variants}),4)

    def test_auto_spacing_mount_and_ground(self):
        spacing,width=automatic_spacing(1.,CAMERA,0.,.2)
        self.assertAlmostEqual(width,.85*640/381.36114963)
        self.assertAlmostEqual(spacing,width*.8)
        raised,_=automatic_spacing(1.,CAMERA,-.17,.2)
        self.assertGreater(raised,spacing)

    def test_camera_too_low(self):
        with self.assertRaises(ValueError):automatic_spacing(.17,CAMERA,0.,.2)

    def test_footprint_faces_ground(self):
        intrinsics=[CAMERA[k] for k in ['fx','fy','cx','cy','width','height']]
        visible=footprint([0,0,.85],np.array(CAMERA['optical_rotation_body']).reshape(3,3),intrinsics,0.)
        self.assertTrue(inside([[0,0]],visible)[0])
        self.assertIsNone(footprint([0,0,.85],np.eye(3),intrinsics,0.))

    def test_raster_union_does_not_double_count(self):
        xy,mask,_,_=raster([[0,0],[1,0],[1,1],[0,1]],.1)
        covered=np.zeros(len(xy),dtype=bool)
        for _ in range(3):covered |= inside(xy,[[0,0],[1,0],[1,1],[0,1]])&mask
        self.assertEqual(covered.sum(),100)

    def test_obstacle_splits_strip(self):
        safe=lambda a,b:not any(.8<=p[0]<=1.2 for p in [a,b])
        result=split_free([0,0],[2,0],1.,safe)
        self.assertEqual(len(result),2)
        self.assertLess(result[0][1][0],.8)
        self.assertGreater(result[1][0][0],1.2)

    def test_rolling_window_continues(self):
        route=[(0,0,1),(5,0,1),(5,5,1)]
        projection,arc=forward_projection([0,0,1],route,0.,.35)
        window,continuous=route_window(route,projection,arc,6.)
        self.assertTrue(continuous);self.assertEqual(window[-1],(5.,1.,1.))

    def test_final_window_stops(self):
        route=[(0,0,1),(5,0,1)]
        projection,arc=forward_projection([4,0,1],route,3.9,.35)
        window,continuous=route_window(route,projection,arc,12.)
        self.assertFalse(continuous);self.assertEqual(window[-1],route[-1])

    def test_projection_cannot_jump_to_later_parallel_lane(self):
        route=[(0,0,1),(5,0,1),(5,.5,1),(0,.5,1)]
        projection,_=forward_projection([.2,.45,1],route,0.,.35)
        self.assertLessEqual(projection[1],.35)


class SafetyStateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rospy
        from inspection_manager import InspectionManager
        cls.rospy=rospy;cls.manager_type=InspectionManager

    def node(self):
        n=self.manager_type.__new__(self.manager_type)
        n.lock=threading.RLock();n.state='SCANNING';n.task_id=1;n.serial=0
        n.phase='NAVIGATING';n.health='HEALTHY';n.flight_error='';n.health_reason=''
        n.error_wall=0.;n.started_wall=time.monotonic()
        n.epoch=1;n.plan={'epoch':1,'route':[(0,0,1),(2,0,1)],'missed':[],
            'covered':np.zeros(1,dtype=bool),'eligible':np.ones(1,dtype=bool),'region_mask':np.ones(1,dtype=bool)}
        n.progress=0.;n.started=self.rospy.Time(79);n.issues={};n.record_dir=None
        n.record_failed=False;n.last_status_wall=time.monotonic()+100
        n.static_ready=Mock(return_value=True);n.same_transform=Mock(return_value=True)
        n.camera_valid=Mock(return_value=True);n.request_hold=Mock();n.summary=Mock()
        n.camera_error=Mock(return_value='下视相机断流')
        n.record=Mock();n.camera_timeout=.5;n.res=.1;n.completion_ratio=.99
        return n

    def test_camera_failure_holds_and_preserves_plan(self):
        n=self.node();plan=n.plan;n.camera_valid.return_value=False
        with patch.object(self.rospy.Time,'now',return_value=self.rospy.Time(80)):n.tick()
        self.assertEqual(n.state,'PAUSED');self.assertIs(n.plan,plan);n.request_hold.assert_called_once()
        self.assertIn('CAMERA',n.issues);self.assertEqual(n.task_id,0)

    def test_severe_flight_failure_does_not_override_landing(self):
        n=self.node();n.phase='LANDING'
        with patch.object(self.rospy.Time,'now',return_value=self.rospy.Time(80)):n.tick()
        self.assertEqual(n.state,'FAILED');n.request_hold.assert_not_called();self.assertIn('FLIGHT',n.issues)

    def test_map_change_pauses(self):
        n=self.node();n.epoch=2
        with patch.object(self.rospy.Time,'now',return_value=self.rospy.Time(80)):n.tick()
        self.assertEqual(n.state,'PAUSED');self.assertIn('MAP',n.issues)

    def test_fault_recovery_does_not_automatically_resume(self):
        n=self.node();n.state='PAUSED';n.task_id=0;n.issues['CAMERA']=dict(code='CAMERA')
        with patch.object(self.rospy.Time,'now',return_value=self.rospy.Time(80)):n.tick()
        self.assertEqual(n.state,'PAUSED');self.assertNotIn('CAMERA',n.issues)

    def test_no_static_map_rejects_plan(self):
        from drone_stack.srv import PlanInspectionRequest
        n=self.node();n.state='IDLE';n.static_ready.return_value=False
        with patch.object(self.rospy.Time,'now',return_value=self.rospy.Time(80)):
            reply=n.plan_request(PlanInspectionRequest())
        self.assertFalse(reply.success);self.assertIn('预建地图',reply.message)

    def test_partial_completion_is_not_success(self):
        n=self.node()
        with patch.object(self.rospy.Time,'now',return_value=self.rospy.Time(80)):n.finish()
        self.assertEqual(n.state,'COMPLETED_PARTIAL');self.assertIn('PARTIAL',n.issues)

    def test_record_overflow_warns_without_landing(self):
        import queue
        n=self.node();n.record_dir=Path('/tmp/unused_inspection_test');n.io_queue=queue.Queue(maxsize=1)
        n.io_queue.put(('occupied',None,None))
        # Use the real record method rather than the fixture's mock.
        with patch.object(self.rospy.Time,'now',return_value=self.rospy.Time(80)):
            self.manager_type.record(n,'frame',{})
        self.assertTrue(n.record_failed);self.assertIn('RECORD',n.issues)
        self.assertEqual(n.state,'SCANNING');n.request_hold.assert_not_called()


class RouteTests(unittest.TestCase):
    def fixture(self,with_obstacle):
        import rospy
        from inspection_manager import InspectionManager
        from trajectory_guard import PackedCells
        node=InspectionManager.__new__(InspectionManager)
        node.origin=[-8.,-8.,.5];node.map_res=.1;node.shape=[160,160,30]
        node.timeout=20.;node.serial=1
        bitmap=np.zeros(math.prod(node.shape),dtype=bool)
        if with_obstacle:
            grid=bitmap.reshape(node.shape);grid[76:84,76:84,:20]=True
        occupied=PackedCells(bitmap,node.shape,bitmap=True)
        plan={'lines_odom':[(tuple((*a,1.)),tuple((*b,1.))) for a,b in
            strips([[-2,-2],[2,-2],[2,2],[-2,2]],.5,0.)], 'finished_lines':set()}
        return node,plan,occupied,rospy

    def test_open_region_builds_real_continuous_reference(self):
        node,plan,occupied,rospy=self.fixture(False)
        with patch.object(rospy,'get_param',side_effect=lambda k,d=None:d):
            route,sections,missed,kept=node.build_route(plan,[-3,-2,1],occupied,1)
            planner,cells=node.planner(occupied)
        self.assertEqual(len(kept),8);self.assertEqual(missed,[])
        self.assertTrue(planner.route_safe(route,cells))
        self.assertEqual(sum(s['kind']=='SCANNING' for s in sections),8)

    def test_pillar_is_avoided_in_reference(self):
        node,plan,occupied,rospy=self.fixture(True)
        with patch.object(rospy,'get_param',side_effect=lambda k,d=None:d):
            route,sections,missed,kept=node.build_route(plan,[-3,-2,1],occupied,1)
            planner,cells=node.planner(occupied)
        self.assertTrue(planner.route_safe(route,cells));self.assertGreater(len(kept),0)
        self.assertGreater(sum(s['kind']=='SCANNING' for s in sections),len(kept))

    def test_canceled_route_generation_exits(self):
        node,plan,occupied,rospy=self.fixture(False);node.serial=2
        with patch.object(rospy,'get_param',side_effect=lambda k,d=None:d):
            with self.assertRaises(RuntimeError):node.build_route(plan,[-3,-2,1],occupied,1)


class InterfaceTests(unittest.TestCase):
    def test_interrupted_update_cannot_restart_navigation(self):
        from types import SimpleNamespace
        from drone_stack.srv import ExecuteInspectionRouteRequest
        from flight_manager import FlightManager
        from geometry_msgs.msg import PoseStamped
        n=FlightManager.__new__(FlightManager);n.lock=threading.RLock()
        n.map_alignment_ready=True;n.navigation_map_mode='PRIOR_NAV';n.authorized=True
        n.airborne=lambda:True;n.fresh_pose=lambda:True;n.fresh_map=lambda:True;n.fresh_free_map=lambda:True
        n.lio_quality='HEALTHY';n.state=SimpleNamespace(mode='OFFBOARD');n.phase='HOLD';n.inspection_task=0
        n.in_flight_volume=lambda p:True
        request=ExecuteInspectionRouteRequest(task_id=123,speed=.5);request.route.header.frame_id='odom'
        request.route.poses=[PoseStamped(),PoseStamped()]
        reply=n.execute_inspection_route(request)
        self.assertFalse(reply.success);self.assertIn('操作员',reply.message)

    def test_stale_inspection_heartbeat_cannot_refresh_health(self):
        import rospy,json
        from std_msgs.msg import String
        from flight_manager import FlightManager
        n=FlightManager.__new__(FlightManager);n.inspection_heartbeat=rospy.Time(80)
        with patch.object(rospy.Time,'now',return_value=rospy.Time(81)):
            n.on_inspection_status(String(json.dumps({'stamp_ns':79*10**9})))
            self.assertEqual(n.inspection_heartbeat,rospy.Time(80))
            n.on_inspection_status(String(json.dumps({'stamp_ns':81*10**9})))
            self.assertEqual(n.inspection_heartbeat,rospy.Time(81))

    def test_inspection_does_not_accept_ordinary_goal_chord(self):
        from ego_planner.msg import GlobalRoute
        from flight_manager import FlightManager
        n=FlightManager.__new__(FlightManager);n.planner_condition=threading.Condition()
        n.route_sequence=12;n.inspection_task=12;n.global_route=None
        msg=GlobalRoute(task_id=12);msg.path.header.frame_id='odom'
        n.on_global_route(msg)
        self.assertIsNone(n.global_route)

    def test_blocked_prefix_is_not_executable_reference(self):
        from ego_planner.msg import GlobalRoute
        from flight_manager import FlightManager
        n=FlightManager.__new__(FlightManager);n.planner_condition=threading.Condition()
        n.route_sequence=12;n.inspection_task=12;n.global_route=object()
        msg=GlobalRoute(task_id=12,inspection=True,blocked=True);msg.path.header.frame_id='odom'
        n.on_global_route(msg)
        self.assertIsNone(n.global_route)

    def test_near_final_endpoint_on_an_earlier_lane_is_not_arrival(self):
        from flight_manager import FlightManager
        from nav_msgs.msg import Path as RosPath
        from geometry_msgs.msg import PoseStamped
        n=FlightManager.__new__(FlightManager);n.active_goal=PoseStamped();n.inspection_continuous=False
        n.inspection_task=12;n.global_route=RosPath()
        for x,y in [(0,0),(4,0),(4,.5),(0,.5)]:
            p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y;n.global_route.poses.append(p)
        self.assertFalse(n.tick_goal_arrival(None))

    def test_partial_yellow_path_keeps_continue_semantics_and_block_flag(self):
        import rospy
        from ego_planner.msg import GlobalRoute
        from safe_global_path import SafeGlobalPath
        n=SafeGlobalPath.__new__(SafeGlobalPath)
        n.task_sequence=12;n.route_version=1;n.reference_progress=0.;n.goal=(2,0,1)
        n.inspection_intent=GlobalRoute(task_id=12,inspection=True,continuous=True)
        n.free=None;n.pub=Mock();n.status=Mock();n.progress_pub=Mock();n.route_pub=Mock();n.marker_pub=Mock()
        with patch.object(rospy.Time,'now',return_value=rospy.Time(80)):
            n.publish([(0,0,1),(1,0,1)],'blocked prefix')
        msg=n.route_pub.publish.call_args[0][0]
        self.assertTrue(msg.inspection);self.assertTrue(msg.continuous);self.assertTrue(msg.blocked)


if __name__=='__main__':unittest.main(verbosity=2)
