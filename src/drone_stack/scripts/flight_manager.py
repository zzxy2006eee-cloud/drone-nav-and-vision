#!/usr/bin/env python3
"""Single owner of PX4 OFFBOARD setpoints and operator flight commands."""
import math
import json
import os
import sys
import threading
import time
import copy
from types import SimpleNamespace
import numpy as np
from itertools import combinations

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import EstimatorStatus, ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from rosgraph_msgs.msg import Clock
from quadrotor_msgs.msg import PositionCommand
from ego_planner.msg import Bspline, DataDisp, GlobalRoute
from ego_planner.srv import StartPlanning, StartPlanningRequest
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trajectory_guard import validate_curve, PackedCells, curve_geometry
from drone_stack.srv import ExecuteInspectionRoute, ExecuteInspectionRouteResponse
from drone_stack.srv import (ExecuteAiGoal,ExecuteAiGoalResponse,CancelNavigation,CancelNavigationResponse,
    ExecuteAiAction,ExecuteAiActionResponse,TakeoffRequest)
from ego_planner.msg import GlobalRoute
from voxel_map_client import VoxelMapClient
from std_msgs.msg import Bool, String, Header, Float64, Float64MultiArray
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse

from drone_stack.srv import (SetNavigationSpeed, SetNavigationSpeedResponse, QueueGoal, QueueGoalResponse, SetMaxFlightHeight, SetMaxFlightHeightResponse, LocalGoal, LocalGoalResponse, Takeoff,
                             TakeoffResponse)


class FlightManager:
    def __init__(self):
        self.lock = threading.RLock()
        self.planner_condition = threading.Condition(self.lock)
        self.planner_job = None
        self.planner_serial = 0
        self.planner_ack_wall = 0.0
        self.first_session_curve = False
        self.navigation_wait_reason = ''
        self.last_wait_reason_published = None
        self.authorized = False
        self.map_alignment_ready = False
        self.navigation_map_mode = "UNKNOWN"
        self.phase = 'DISCONNECTED'
        self.last_error = ''
        self.state = State()
        self.extended = ExtendedState()
        self.odom = None
        self.estimator = None
        self.pending_estimator = None
        self.lio_valid = False
        self.lio_health_time = rospy.Time(0)
        self.lio_quality = 'UNKNOWN'
        self.lio_quality_time = rospy.Time(0)
        self.health_fault_since = None
        self.last_healthy_pose = None
        self.health_landing_latched = False
        self.lio_severe = False
        self.last_health_stamp_ns = 0
        self.trajectory = None
        self.latest_command_id = 0
        self.last_spline_start = rospy.Time(0)
        self.required_trajectory_id = 1
        self.navigation_generation = 0
        self.approved_spline_id = None
        self.approved_curve = None
        self.heading_curve_cache = None
        self.prefix_check_time = rospy.Time(0)
        self.known_horizon = rospy.get_param('~known_execution_horizon_s',3.0)
        self.pending_spline = None
        self.trajectory_time = rospy.Time(0)
        self.future_command_since = None
        self.goal_start = rospy.Time(0)
        self.planner_time = rospy.Time(0)
        self.health_hold_s=float(rospy.get_param('/drone/health_hold_s',1.0))
        self.health_land_s=float(rospy.get_param('/drone/health_land_s',2.0))
        if not all(math.isfinite(v) for v in [self.health_hold_s,self.health_land_s]) or not 0<self.health_hold_s<self.health_land_s:
            raise ValueError('Health HOLD must precede landing')
        self.max_planner_age = rospy.get_param('~max_planner_age_s', 1.0)
        if not math.isfinite(self.max_planner_age) or not 0 < self.max_planner_age <= 1.0:
            raise ValueError('Planner heartbeat age must be finite and at most 1.0 seconds')
        self.pending_goal = None
        self.active_goal = None
        self.route_sequence = 0
        self.global_route = None
        self.inspection_task = 0
        self.inspection_reserved = False
        self.inspection_continuous = False
        self.inspection_revision = 0
        self.inspection_heartbeat=rospy.Time(0)
        self.ai_session='';self.ai_control=False;self.ai_seen_wall=0.;self.ai_instance='';self.ai_counter=-1
        self.ai_navigation_id=0;self.ai_navigation_session='';self.ai_flight_owned='';self.ai_flight_session='';self.ai_expiry_reported=False
        self.ai_task_canceled=False
        self.ai_request_id='';self.ai_task_active=False;self.ai_navigation_request='';self.ai_flight_request=''
        self.inspection_route_pub = rospy.Publisher('/drone/inspection_route', GlobalRoute, queue_size=1, latch=True)
        self.next_navigation_speed = 0.5
        self.active_navigation_speed = 0.5
        self.goal_configuration_busy = False
        self.space_wait_started = rospy.Time(0)
        # Internal navigation stages keep one Qt goal alive while turning.
        self.navigation_stage = 'WAITING'
        self.navigation_heading_ready = False
        self.navigation_heading = 0.0
        self.heading_target = 0.0
        self.heading_started = rospy.Time(0)
        self.heading_last_tick = rospy.Time(0)
        self.heading_stable_since = None
        self.heading_brake_since = None
        self.heading_braked = False
        self.heading_half_cone = math.radians(60.0)
        self.heading_rate = math.radians(30.0)
        self.heading_tolerance = math.radians(10.0)
        self.goal_reached_since = None
        self.goal_arrival_pose = None
        self.arrival_started = None
        self.arrival_outside_since = None
        self.arrival_curve = None
        self.terminal_stage = 'IDLE'
        self.terminal_approach_distance = float(rospy.get_param('/drone/terminal_approach_distance_m', .8))
        self.terminal_deceleration = float(rospy.get_param('/drone/terminal_deceleration_mps2', .4))
        self.terminal_settle_time = float(rospy.get_param('/drone/terminal_settle_time_s', 3.0))
        if (not math.isfinite(self.terminal_approach_distance) or not .3 <= self.terminal_approach_distance <= 2. or
                not math.isfinite(self.terminal_deceleration) or not 0 < self.terminal_deceleration <= .5 or
                not math.isfinite(self.terminal_settle_time) or not 1. <= self.terminal_settle_time <= 5.):
            raise ValueError('Invalid terminal convergence parameters')
        self.terminal_last_curve_id = None
        self.terminal_curve_cache = None
        self.goal_tolerance = rospy.get_param('~goal_tolerance_m', 0.15)
        self.goal_speed_tolerance = rospy.get_param('~goal_speed_tolerance_mps', 0.15)
        self.arrival_exit_tolerance = float(rospy.get_param('~arrival_exit_tolerance_m', .20))
        if not self.goal_tolerance < self.arrival_exit_tolerance <= .30:
            raise ValueError('Arrival exit tolerance must exceed final tolerance and be at most 0.30 m')
        self.goal_min = rospy.get_param('~goal_min_xyz', [-8.0, -8.0, 0.5])
        self.goal_max = rospy.get_param('~goal_max_xyz', [8.0, 8.0, 2.5])
        self.configured_max_height = self.goal_max[2]
        self.ceiling_margin = .20
        self.max_flight_height = float(rospy.get_param('/drone/max_flight_height_m', self.configured_max_height))
        if not .8 <= self.max_flight_height <= self.configured_max_height:
            raise ValueError('Virtual ceiling must be between 0.8 m and configured maximum')
        self.goal_max[2] = self.max_flight_height-self.ceiling_margin
        self.goal_queue = []
        self.queue_dispatch_after = None
        self.max_queue_points = 100
        self.yaw_last_tick = rospy.Time(0)
        self.max_takeoff_height = rospy.get_param('~max_takeoff_height_m', 2.0)
        self.map_resolution = rospy.get_param('/ego_planner_node/grid_map/resolution', 0.1)
        self.map_origin = [-rospy.get_param('/ego_planner_node/grid_map/map_size_x',30.0)/2,
                           -rospy.get_param('/ego_planner_node/grid_map/map_size_y',30.0)/2,
                           rospy.get_param('/ego_planner_node/grid_map/ground_height',0.5)]
        self.map_shape = [math.ceil(rospy.get_param('/ego_planner_node/grid_map/map_size_'+axis, size)/self.map_resolution)
                          for axis,size in zip('xyz',[30.0,30.0,8.0])]
        self.occupied_cells = set()
        self.map_time = rospy.Time(0)
        self.require_observed_free = rospy.get_param('~require_observed_free', False)
        self.observed_free = set()
        self.free_time = rospy.Time(0)
        self.max_map_age = rospy.get_param('~max_map_age_s', 2.0)
        self.max_setpoint_step = rospy.get_param('~max_setpoint_step_m', 0.75)
        self.max_command_speed = rospy.get_param('~max_command_speed_mps', 1.5)
        self.prime_start = rospy.Time(0)
        self.arm_start = rospy.Time(0)
        self.rotate_start = None
        self.rotate_yaw_start = 0.0
        self.rotate_rate = math.radians(10.0)
        self.hold_pose = None
        self.takeoff_pose = None
        self.takeoff_start = rospy.Time(0)
        self.takeoff_start_z = 0.0
        self.takeoff_speed = rospy.get_param('~takeoff_speed_mps', 0.3)
        self.land_start = rospy.Time(0)
        self.max_pose_age = rospy.get_param('~max_pose_age_s', 0.35)
        self.max_traj_age = rospy.get_param('~max_traj_age_s', 0.35)
        # SITL only: PX4 requires 500 timesync exchanges before it honours
        # acquisition timestamps. Real hardware uses normal synchronisation.
        self.startup_min_time = rospy.get_param('~startup_min_time_s', 0.0)
        self.future_tolerance = rospy.get_param('~future_tolerance_s', 0.02)
        if not math.isfinite(self.known_horizon) or self.known_horizon < 3.0:
            raise ValueError('Known execution horizon must be at least 3 seconds for the current speed/latency bounds')
        self.clock_fault = False
        self.last_clock = rospy.Time(0)
        self.descent_rate = rospy.get_param('~fallback_descent_rate_mps', 0.35)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1,
                                           tcp_nodelay=True)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        self.terminal_stage_pub = rospy.Publisher('/drone/navigation_terminal_stage', String, queue_size=1, latch=True)
        self.queue_pub = rospy.Publisher('/drone/goal_queue', Path, queue_size=1, latch=True)
        self.ceiling_pub = rospy.Publisher('/drone/max_flight_height', Float64, queue_size=1, latch=True)
        self.executed_path_pub = rospy.Publisher('/drone/executed_local_path', Path, queue_size=1, latch=True)
        self.local_display_curve = None
        self.local_display_samples = None
        self.local_display_last = rospy.Time(0)
        self.navigation_goal_pub = rospy.Publisher('/drone/navigation_goal', PoseStamped, queue_size=1, latch=True)
        self.enabled_pub = rospy.Publisher('/drone/planning_enabled', Bool, queue_size=1, latch=True)
        self.phase_pub = rospy.Publisher('/drone/flight_state', String, queue_size=1, latch=True)
        self.wait_reason_pub = rospy.Publisher('/drone/navigation_wait_reason', String, queue_size=1, latch=True)
        self.navigation_stage_pub = rospy.Publisher('/drone/navigation_stage', String, queue_size=1, latch=True)
        self.last_navigation_stage_published = None
        self.auth_pub = rospy.Publisher('/drone/authorized', Bool, queue_size=1, latch=True)
        self.error_pub = rospy.Publisher('/drone/flight_error', String, queue_size=1, latch=True)
        self.flight_health_pub = rospy.Publisher('/drone/flight_health', String, queue_size=1, latch=True)
        self.flight_snapshot_pub = rospy.Publisher('/drone/flight_health_snapshot', String, queue_size=1, latch=True)
        self.flight_valid_pub = rospy.Publisher('/drone/flight_health_valid', Bool, queue_size=1, latch=True)
        self.heartbeat_pub = rospy.Publisher('/drone/manager_heartbeat', Header, queue_size=1)
        self.planner_heartbeat_diagnostic = (rospy.Publisher('/drone/planner_heartbeat_diagnostic',Float64MultiArray,queue_size=10)
            if rospy.get_param('~record_heartbeat_diagnostics',False) else None)
        rospy.Subscriber('/mavros/state', State, self.on_state, queue_size=10)
        rospy.Subscriber('/mavros/extended_state', ExtendedState, self.on_extended, queue_size=10)
        rospy.Subscriber('/drone/map_mode', String, lambda m: setattr(self,'navigation_map_mode',m.data), queue_size=1)
        rospy.Subscriber('/drone/map_alignment_ready', Bool, lambda m: setattr(self, 'map_alignment_ready', m.data), queue_size=1)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.on_odom, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/mavros/estimator_status', EstimatorStatus, self.on_estimator, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/drone/lio/health', String, self.on_lio_health, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.on_trajectory, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/data_display', DataDisp, self.on_planner_heartbeat, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/bspline', Bspline, self.on_spline, queue_size=10, tcp_nodelay=True)
        rospy.Subscriber('/drone/global_route', GlobalRoute, self.on_global_route, queue_size=1, tcp_nodelay=True)
        self.voxel_client=VoxelMapClient(self.map_origin,self.map_resolution,self.map_shape,
                                         self.on_voxel_map,self.invalidate_voxel_map)
        rospy.Service('/drone/set_authorized', SetBool, self.set_authorized)
        rospy.Service('/drone/arm', Trigger, self.arm)
        rospy.Service('/drone/disarm', Trigger, self.disarm)
        rospy.Service('/drone/takeoff', Takeoff, self.takeoff)
        rospy.Service('/drone/hold', Trigger, self.hold)
        rospy.Service('/drone/cancel_goal', Trigger, self.hold)
        rospy.Service('/drone/land', Trigger, self.land)
        rospy.Service('/drone/rotate_once', Trigger, self.rotate_once)
        rospy.Service('/drone/local_goal', LocalGoal, self.local_goal)
        rospy.Service('/drone/queue_goal', QueueGoal, self.queue_goal)
        rospy.Service('/drone/execute_inspection_route', ExecuteInspectionRoute, self.execute_inspection_route)
        rospy.Subscriber('/drone/inspection/exclusive',Bool,lambda m:setattr(self,'inspection_reserved',m.data),queue_size=1)
        rospy.Subscriber('/drone/inspection/status',String,self.on_inspection_status,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber('/drone/ai/status',String,self.on_ai_status,queue_size=1,tcp_nodelay=True)
        rospy.Service('/drone/execute_ai_goal',ExecuteAiGoal,self.execute_ai_goal)
        rospy.Service('/drone/cancel_navigation',CancelNavigation,self.cancel_navigation)
        rospy.Service('/drone/execute_ai_action',ExecuteAiAction,self.execute_ai_action)
        rospy.Service('/drone/set_max_flight_height', SetMaxFlightHeight, self.set_max_flight_height)
        rospy.Service('/drone/set_navigation_speed', SetNavigationSpeed, self.set_navigation_speed)
        self.planner_client = rospy.ServiceProxy('/planning/start', StartPlanning)
        self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.suspend_planner()
        self.publish_goal_queue()
        self.terminal_stage_pub.publish(String(data=self.terminal_stage))
        self.ceiling_pub.publish(Float64(data=self.max_flight_height))
        self.auth_pub.publish(Bool(data=False))
        rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=1, tcp_nodelay=True)
        threading.Thread(target=self.planner_worker, daemon=True).start()
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.tick, reset=True)

    def suspend_planner(self):
        self.publish_execution_curve()
        self.enabled_pub.publish(Bool(data=False))
        with self.planner_condition:
            self.planner_serial += 1
            self.planner_job = (self.planner_serial, self.navigation_generation, None, self.active_navigation_speed)
            self.planner_condition.notify()

    def publish_execution_curve(self, curve=None):
        """Export the exact adopted B-spline; never optimize a display path."""
        now = rospy.Time.now()
        message = Path()
        message.header = Header(stamp=now, frame_id='odom')
        if curve is None:
            self.local_display_curve = None
            self.local_display_samples = None
            self.executed_path_pub.publish(message)
            self.local_display_last = now
            return
        if curve is self.local_display_curve and (now-self.local_display_last).to_sec()<.25:
            return
        if curve is not self.local_display_curve:
            spline = curve_geometry(curve)[3]
            begin, end = curve.knots[curve.order], curve.knots[len(curve.pos_pts)]
            times = np.linspace(begin,end,min(2048,max(2,int(math.ceil((end-begin)/.025))+1)))
            self.local_display_samples = (spline,times,spline(times),begin,end)
            self.local_display_curve = curve
        spline,times,points,begin,end = self.local_display_samples
        current = min(end,max(begin,begin+(now-curve.start_time).to_sec()))
        first = int(np.searchsorted(times,current,side='right'))
        for xyz in [spline(current)]+list(points[first:]):
            pose = PoseStamped();pose.header=message.header;pose.pose.orientation.w=1.
            pose.pose.position=Point(x=float(xyz[0]),y=float(xyz[1]),z=float(xyz[2]))
            message.poses.append(pose)
        self.executed_path_pub.publish(message)
        self.local_display_last = now

    def request_planner(self, goal):
        self.navigation_stage = 'PLANNING'
        self.navigation_wait_reason = '等待统一全局路线，再准备局部规划'
        self.goal_start = rospy.Time.now()
        self.planner_time = rospy.Time(0)
        self.planner_ack_wall = time.monotonic()
        with self.planner_condition:
            self.planner_serial += 1
            self.planner_job = (self.planner_serial, self.navigation_generation, copy.deepcopy(goal), self.active_navigation_speed)
            self.planner_condition.notify()

    def planner_worker(self):
        # Network ACKs run outside the timer lock. Jobs are serialized and a
        # superseded ACK can never publish enabled=true for a cancelled task.
        while not rospy.is_shutdown():
            with self.planner_condition:
                while self.planner_job is None and not rospy.is_shutdown():
                    self.planner_condition.wait(.5)
                if rospy.is_shutdown(): return
                serial, generation, goal, speed = self.planner_job
                self.planner_job = None
            req=StartPlanningRequest(enabled=goal is not None, speed_mps=speed, generation=serial)
            req.inspection=bool(self.inspection_task)
            req.continuous=self.inspection_continuous
            if goal is not None:
                req.goal=goal
                # The global route is authoritative for both display and EGO.
                # Wait outside the flight timer lock; cancellation invalidates
                # the serial while held setpoints continue at 30 Hz.
                deadline=time.monotonic()+45.
                with self.planner_condition:
                    while serial==self.planner_serial and not self.route_ready(goal) and time.monotonic()<deadline:
                        self.planner_condition.wait(.1)
                    if serial!=self.planner_serial:continue
                    if not self.route_ready(goal):
                        self.wait_for_space('Unified global route not available yet')
                        continue
                    req.route=copy.deepcopy(self.global_route)
                    if req.inspection:
                        req.goal.pose=copy.deepcopy(req.route.poses[-1].pose)
                        req.continuous=self.inspection_continuous
            try:
                rospy.wait_for_service('/planning/start', timeout=5.)
                reply=self.planner_client(req)
                reason='' if reply.success and reply.generation==serial else reply.message or 'Planning ACK generation mismatch'
            except (rospy.ServiceException, rospy.ROSException) as exc:
                reason='Planning service unavailable: '+str(exc)
            with self.lock:
                if serial!=self.planner_serial or generation!=self.navigation_generation or goal is None:
                    continue
                if self.phase!='NAVIGATING' or self.navigation_stage!='PLANNING': continue
                if reason:
                    self.wait_for_space(reason)
                    continue
                if not self.fresh_pose() or not self.fresh_map() or not self.fresh_free_map():
                    self.stop_navigation('Localization/map expired while preparing new planning session')
                    continue
                self.required_trajectory_id=max(self.latest_command_id+1,reply.next_trajectory_id)
                self.goal_start=rospy.Time.now()
                self.first_session_curve=True
                self.navigation_stage='WAITING'
                self.navigation_wait_reason='新目标已确认，等待轨迹与前方自由空间检查'
                self.enabled_pub.publish(Bool(data=True))
                rospy.loginfo('Planning session ACK generation=%s; accepting new curve id >= %s',
                              serial,self.required_trajectory_id)

    def on_global_route(self, message):
        with self.planner_condition:
            if message.path.header.frame_id!='odom' or message.task_id!=self.route_sequence:return
            if self.inspection_task and not message.inspection:return
            if message.inspection and message.blocked:
                self.global_route=None;self.planner_condition.notify_all();return
            self.global_route=message.path
            if message.inspection and self.inspection_task and self.active_goal is not None and message.path.poses:
                self.active_goal.pose=copy.deepcopy(message.path.poses[-1].pose)
                self.inspection_continuous=message.continuous
            self.planner_condition.notify_all()

    def route_ready(self, goal):
        route=self.global_route
        if route is None or self.route_sequence!=goal.header.stamp.to_nsec() or len(route.poses)<2:return False
        age=(rospy.Time.now()-route.header.stamp).to_sec()
        a=route.poses[-1].pose.position;b=goal.pose.position
        return -.02<=age<=2. and (bool(self.inspection_task) or math.dist([a.x,a.y,a.z],[b.x,b.y,b.z])<.02)

    def execute_inspection_route(self, request):
        with self.lock:
            route=copy.deepcopy(request.route)
            if (not self.map_alignment_ready or self.navigation_map_mode!='PRIOR_NAV' or
                    not self.authorized or not self.airborne() or not self.fresh_pose() or
                    self.lio_quality!='HEALTHY' or self.state.mode!='OFFBOARD' or
                    not self.fresh_map() or not self.fresh_free_map()):
                return ExecuteInspectionRouteResponse(False,'巡检需要预建地图、授权及健康 OFFBOARD 悬停/导航',0)
            if (route.header.frame_id!='odom' or not 2<=len(route.poses)<=2048 or
                    not .1<=request.speed<=1. or
                    not all(self.in_flight_volume([p.pose.position.x,p.pose.position.y,p.pose.position.z]) for p in route.poses)):
                return ExecuteInspectionRouteResponse(False,'巡检路线或速度非法',0)
            updating=bool(request.task_id and request.task_id==self.inspection_task and self.phase=='NAVIGATING')
            if request.task_id and not updating:
                return ExecuteInspectionRouteResponse(False,'巡检执行会话已中断，必须由操作员点击继续',0)
            if not updating:
                if self.phase!='HOLD' or self.goal_queue:
                    return ExecuteInspectionRouteResponse(False,'请先取消普通目标并进入 HOLD',0)
                # Mark ownership before the asynchronous ACK worker starts.
                self.inspection_task=1;self.inspection_continuous=request.continuous
                previous_speed=self.next_navigation_speed
                self.next_navigation_speed=request.speed
                goal=copy.deepcopy(route.poses[-1]);goal.header=copy.deepcopy(route.header)
                try:reply=self.local_goal(type('InspectionGoal',(),{'goal':goal})(),inspection=True)
                finally:self.next_navigation_speed=previous_speed
                if not reply.success:
                    self.inspection_task=0;self.inspection_continuous=False
                    return ExecuteInspectionRouteResponse(False,reply.message,0)
                self.inspection_task=self.route_sequence
                self.inspection_heartbeat=rospy.Time.now()
            self.inspection_continuous=request.continuous
            self.inspection_revision+=1
            self.active_goal.pose=copy.deepcopy(route.poses[-1].pose)
            self.inspection_route_pub.publish(GlobalRoute(task_id=self.route_sequence,path=route,
                inspection=True,continuous=request.continuous,revision=self.inspection_revision))
            return ExecuteInspectionRouteResponse(True,'巡检控制参考已提交',self.route_sequence)

    def on_inspection_status(self,message):
        try:
            data=json.loads(message.data);ns=int(data['stamp_ns'])
            stamp=rospy.Time(ns//1000000000,ns%1000000000)
            age=(rospy.Time.now()-stamp).to_sec()
            if -.02<=age<=1. and stamp>self.inspection_heartbeat:self.inspection_heartbeat=stamp
        except (ValueError,KeyError,TypeError):pass

    def on_ai_status(self,m):
        try:
            data=json.loads(m.data);instance=data['instance_id'];counter=int(data['heartbeat_seq'])
            if instance==self.ai_instance and counter<=self.ai_counter:return
            self.ai_instance=instance;self.ai_counter=counter;self.ai_session=data.get('session_id','')
            self.ai_control=bool(data.get('control_authorized') and data.get('client_alive'))
            self.ai_task_canceled=data.get('state') in ('CANCELED','FAILED')
            self.ai_request_id=data.get('request_id','');self.ai_task_active=bool(data.get('busy') and data.get('state') in ('PARSING','EXECUTING'))
            self.ai_seen_wall=time.monotonic()
        except (ValueError,KeyError,TypeError):pass

    def ai_valid(self,session):
        return bool(session and session==self.ai_session and self.ai_control and time.monotonic()-self.ai_seen_wall<=3.)

    def execute_ai_goal(self,req):
        with self.lock:
            if not self.ai_valid(req.session_id):return ExecuteAiGoalResponse(False,'AI执行授权或连接失效',0)
            if req.request_id!=self.ai_request_id or not self.ai_task_active:return ExecuteAiGoalResponse(False,'AI任务代次未就绪或已停止',0)
            reply=self.local_goal(req)
            if reply.success:self.ai_navigation_id=self.route_sequence;self.ai_navigation_session=req.session_id;self.ai_navigation_request=req.request_id
            return ExecuteAiGoalResponse(reply.success,reply.message,self.route_sequence if reply.success else 0)

    def cancel_navigation(self,req):
        with self.lock:
            if not req.task_id or req.task_id!=self.route_sequence or self.phase!='NAVIGATING':
                return CancelNavigationResponse(False,'任务已结束或被替换，没有取消其它任务')
            reply=self.hold(None)
            return CancelNavigationResponse(reply.success,reply.message)

    def execute_ai_action(self,req):
        with self.lock:
            stopping_owned=(req.action in ('hold','disarm') and self.ai_flight_owned and req.session_id==self.ai_flight_session and req.request_id==self.ai_flight_request)
            if not stopping_owned:
                if not self.ai_valid(req.session_id):return ExecuteAiActionResponse(False,'AI执行授权或连接失效')
                if req.request_id!=self.ai_request_id or not self.ai_task_active:return ExecuteAiActionResponse(False,'AI任务代次未就绪或已停止')
            if req.action=='hold' and self.phase in ('LANDING','DESCENDING','FAILSAFE'):
                return ExecuteAiActionResponse(False,'已有降落/失效保护继续执行，AI不能改为HOLD')
            if req.action=='disarm' and self.phase in ('TAKEOFF','HOLD','NAVIGATING','ROTATING','DESCENDING'):
                return ExecuteAiActionResponse(False,'AI不能在飞行阶段锁定，请先降落')
            if req.action=='arm':reply=self.arm(None,ai=True)
            elif req.action=='takeoff':reply=self.takeoff(TakeoffRequest(height_m=req.height_m),ai=True)
            elif req.action=='land':reply=self.land(None)
            elif req.action=='hold':reply=self.hold(None)
            elif req.action=='disarm':reply=self.disarm(None)
            else:return ExecuteAiActionResponse(False,'不支持的AI飞行动作')
            if reply.success and req.action in ('arm','takeoff'):
                self.ai_flight_owned=req.action;self.ai_flight_session=req.session_id;self.ai_expiry_reported=False
                self.ai_flight_request=req.request_id
            return ExecuteAiActionResponse(reply.success,reply.message)

    def on_clock(self, msg):
        with self.lock:
            if (self.last_clock-msg.clock).to_nsec()/1e9 > self.future_tolerance:
                self.clock_fault = True
            self.last_clock = msg.clock

    def on_planner_heartbeat(self, msg):
        with self.lock:
            stamp=msg.header.stamp
            age=(rospy.Time.now()-stamp).to_nsec()/1e9
            # Replayed, duplicate, stale or future messages cannot refresh health.
            accepted=stamp.to_nsec()>0 and 0<=age<=self.max_planner_age and stamp>self.planner_time
            if accepted:
                self.planner_time=stamp
            if self.planner_heartbeat_diagnostic is not None:
                self.planner_heartbeat_diagnostic.publish(Float64MultiArray(data=[
                    rospy.Time.now().to_sec(),stamp.to_sec(),age,float(accepted),
                    self.planner_time.to_sec(),self.goal_start.to_sec(),float(self.phase=='NAVIGATING')]))

    def set_phase(self, phase):
        if phase in ('HOLD','LANDING','DESCENDING','FAILSAFE','READY','DISCONNECTED'):
            self.inspection_task=0;self.inspection_continuous=False
            self.ai_navigation_id=0
        if phase in ('HOLD','LANDING','DESCENDING','FAILSAFE','DISCONNECTED'):self.ai_flight_owned=''
        if self.phase != phase:
            rospy.loginfo('Flight state: %s -> %s', self.phase, phase)
            self.phase = phase
            self.phase_pub.publish(String(data=phase))

    def error(self, message):
        self.last_error = message
        self.error_pub.publish(String(data=message))
        rospy.logwarn(message)

    def fresh_pose(self, require_startup=True):
        now = rospy.Time.now()
        # ESTIMATOR_STATUS is a low-rate stream. Dropping one slightly early
        # sample outright can create a false two-second outage. A pending
        # sample cannot refresh health until its acquisition time has arrived.
        pending=self.pending_estimator
        if pending is not None:
            age=(now-pending.header.stamp).to_nsec()/1e9
            if age>=0:
                if age<=2.0 and (self.estimator is None or pending.header.stamp>self.estimator.header.stamp):
                    self.estimator=pending
                self.pending_estimator=None
        estimator = self.estimator
        # Duration.to_sec() forms -1 + fractional seconds for negative ages,
        # rounding an exact -20 ms to -0.020000000000000018. Convert the signed
        # integer nanoseconds once, so the inclusive boundary stays inclusive.
        estimator_age = (now-estimator.header.stamp).to_nsec()/1e9 if estimator is not None else float('inf')
        odom_age = (now-self.odom.header.stamp).to_nsec()/1e9 if self.odom is not None else float('inf')
        health_age = (now-self.lio_health_time).to_nsec()/1e9
        quality_age = (now-self.lio_quality_time).to_nsec()/1e9
        estimator_ok = (estimator is not None and
                        -self.future_tolerance <= estimator_age <= 2.0 and
                        (estimator.pos_horiz_rel_status_flag or estimator.pos_horiz_abs_status_flag) and
                        (estimator.pos_vert_abs_status_flag or estimator.pos_vert_agl_status_flag) and
                        estimator.velocity_horiz_status_flag and estimator.velocity_vert_status_flag)
        self.pose_health_detail = ('odom_age=%.6f estimator_age=%.6f health_age=%.6f '
                                   'lio_valid=%s estimator_ok=%s connected=%s now=%.9f quality=%s quality_age=%.6f' %
                                   (odom_age, estimator_age, health_age, self.lio_valid,
                                    estimator_ok, self.state.connected, now.to_sec(), self.lio_quality, quality_age))
        return (not self.clock_fault and (not require_startup or now.to_sec() >= self.startup_min_time) and
                self.odom is not None and self.lio_valid and self.state.connected and estimator_ok and
                0 <= health_age <= 0.5 and
                0 <= quality_age <= 0.5 and self.lio_quality not in ('SEVERE', 'UNKNOWN') and
                -self.future_tolerance <= odom_age <= self.max_pose_age)

    def airborne(self):
        return (self.state.armed and
                (self.extended.landed_state == ExtendedState.LANDED_STATE_IN_AIR or
                 self.phase in ('TAKEOFF', 'HOLD', 'NAVIGATING', 'ROTATING', 'LANDING', 'DESCENDING')))

    def current_pose(self):
        return self.odom.pose.pose if self.odom else None

    def on_state(self, msg):
        with self.lock:
            was_armed=self.state.armed
            self.state = msg
            if was_armed and not msg.armed:self.ai_flight_owned='';self.ai_expiry_reported=False
            if not msg.connected:
                self.set_phase('DISCONNECTED')
            elif not msg.armed and self.phase not in (
                    'DISCONNECTED', 'LOCALIZING', 'READY', 'PRIMING_ARM', 'WAIT_OFFBOARD', 'WAIT_ARM'):
                self.suspend_planner()
                self.pending_goal = None
                self.set_phase('READY')

    def on_extended(self, msg):
        with self.lock:
            self.extended = msg

    def on_odom(self, msg):
        with self.lock:
            if msg.header.frame_id not in ('odom', 'map'):
                self.error('PX4 local odometry is not in expected ROS ENU frame')
                return
            if (rospy.Time.now()-msg.header.stamp).to_nsec()/1e9 < -self.future_tolerance:
                rospy.logwarn_throttle(2.0, 'Dropping future FCU odometry; keeping last valid sample')
                return
            self.odom = msg

    def on_estimator(self, msg):
        with self.lock:
            age=(rospy.Time.now()-msg.header.stamp).to_nsec()/1e9
            if age < -self.future_tolerance:
                if age>=-.1 and (self.pending_estimator is None or msg.header.stamp>self.pending_estimator.header.stamp):
                    self.pending_estimator=msg
                return
            if self.estimator is None or msg.header.stamp>self.estimator.header.stamp:
                self.estimator = msg

    def on_lio_health(self, msg):
        try:
            data = json.loads(msg.data)
            stamp_ns = int(data['stamp_ns'])
            quality = data['quality']
            if (quality not in ('HEALTHY', 'WARNING', 'HOLD', 'SEVERE', 'UNKNOWN') or
                    type(data['valid']) is not bool or type(data['severe']) is not bool):
                return
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            age = (rospy.Time.now().to_nsec()-stamp_ns)/1e9
            if stamp_ns <= self.last_health_stamp_ns or not -self.future_tolerance <= age <= .5:
                return
            self.last_health_stamp_ns = stamp_ns
            stamp = rospy.Time(stamp_ns//1000000000, stamp_ns%1000000000)
            self.lio_valid = data['valid']
            self.lio_severe = data['severe']
            self.lio_quality = quality
            self.lio_health_time = stamp
            self.lio_quality_time = stamp

    def on_lio(self, msg):
        with self.lock:
            self.lio_valid = msg.data
            self.lio_health_time = rospy.Time.now()

    def on_lio_quality(self, msg):
        with self.lock:
            self.lio_quality = msg.data if msg.data in ('HEALTHY', 'WARNING', 'HOLD', 'SEVERE') else 'UNKNOWN'
            self.lio_quality_time = rospy.Time.now()

    def map_cell(self, values):
        return tuple(math.floor((value-origin)/self.map_resolution) for value,origin in zip(values,self.map_origin))

    def fresh_map(self):
        return (self.map_time != rospy.Time(0) and
                -self.future_tolerance <= (rospy.Time.now()-self.map_time).to_nsec()/1e9 <= self.max_map_age)

    def fresh_free_map(self):
        return (not self.require_observed_free or
                (self.free_time != rospy.Time(0) and
                 -self.future_tolerance <= (rospy.Time.now()-self.free_time).to_nsec()/1e9 <= self.max_map_age))

    def in_flight_volume(self, values):
        return (all(math.isfinite(v) for v in values) and
                all(lo <= v <= hi for v, lo, hi in zip(values, self.goal_min, self.goal_max)))

    def segment_cells(self, start, end):
        """Traverse every closed voxel touched by a line, including edge ties.

        Uniform sampling can miss a short intersection near a voxel corner.
        DDA checks crossed faces and all neighbours at simultaneous crossings.
        """
        def grid(values):
            result = [(v-o)/self.map_resolution for v, o in zip(values, self.map_origin)]
            return [float(round(v)) if abs(v-round(v)) < 1e-10 else v for v in result]
        a, b = grid(start), grid(end)
        cell = [math.floor(v) for v in a]
        delta = [v-u for u, v in zip(a, b)]
        step = [1 if v > 0 else -1 if v < 0 else 0 for v in delta]
        stationary_faces = [i for i in range(3) if step[i] == 0 and abs(a[i]-round(a[i])) < 1e-10]
        start_faces = [i for i in range(3) if abs(a[i]-round(a[i])) < 1e-10]

        def neighbours(base, faces):
            for size in range(len(faces)+1):
                for axes in combinations(faces, size):
                    value = list(base)
                    for axis in axes:
                        value[axis] -= 1
                    yield tuple(value)

        yield from neighbours(cell, start_faces)
        crossing = [((cell[i]+1-a[i])/delta[i] if step[i] > 0 else
                     (cell[i]-a[i])/delta[i] if step[i] < 0 else math.inf)
                    for i in range(3)]
        interval = [abs(1/v) if v else math.inf for v in delta]
        while True:
            t = min(crossing)
            if t > 1.0+1e-10:
                break
            axes = [i for i in range(3) if abs(crossing[i]-t) < 1e-10]
            for size in range(1, len(axes)+1):
                for selected in combinations(axes, size):
                    other = list(cell)
                    for axis in selected:
                        other[axis] += step[axis]
                    yield from neighbours(other, stationary_faces)
            for axis in axes:
                cell[axis] += step[axis]
                crossing[axis] += interval[axis]

    def command_error(self, msg):
        age = (rospy.Time.now()-msg.header.stamp).to_nsec()/1e9
        if (msg.header.frame_id != 'odom' or msg.header.stamp == rospy.Time(0) or
                not -self.future_tolerance <= age <= self.max_traj_age):
            return ('Invalid or stale trajectory acquisition time/frame: '
                    'age=%.6f s stamp=%.9f frame=%s' %
                    (age, msg.header.stamp.to_sec(), msg.header.frame_id))
        values = [msg.position.x, msg.position.y, msg.position.z]
        if not self.in_flight_volume(values) or not math.isfinite(msg.yaw):
            return 'Trajectory setpoint is outside configured flight volume or nonfinite'
        velocity = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
        if (not all(math.isfinite(v) for v in velocity) or
                math.sqrt(sum(v*v for v in velocity)) > min(self.max_command_speed, 1.5*self.active_navigation_speed)):
            return 'Trajectory velocity is nonfinite or exceeds configured speed limit'
        if not self.fresh_map():
            return 'Obstacle map became stale; navigation cancelled'
        p = self.current_pose().position
        start = [p.x, p.y, p.z]
        if not self.in_flight_volume(start):
            return 'Vehicle is outside configured navigation volume'
        if math.sqrt(sum((v-u)**2 for u, v in zip(start, values))) > self.max_setpoint_step:
            return 'Trajectory setpoint jump exceeds tracking limit'
        if any(cell in self.occupied_cells for cell in self.segment_cells(start, values)):
            return 'Trajectory segment intersects an inflated obstacle voxel'
        if self.require_observed_free and self.navigation_heading_ready and not self.trajectory_needs_turn(msg):
            if not self.fresh_free_map():
                return 'Observed free-space map became stale; navigation cancelled'
            if any(cell not in self.observed_free for cell in self.segment_cells(start, values)):
                return 'Trajectory segment enters unobserved space'
        return ''

    def stop_navigation(self, reason, hold_pose=None, preserve_queue=False):
        if not preserve_queue:
            self.clear_goal_queue()
        health_grace = (self.airborne() and not self.clock_fault and not self.lio_severe and
                        self.lio_quality != 'SEVERE' and self.state.connected and
                        (self.health_fault_since is None or
                         (rospy.Time.now()-self.health_fault_since).to_nsec()/1e9 < self.health_land_s))
        response = self.hold(None, health_override=health_grace, preserve_queue=preserve_queue)
        if response.success:
            if hold_pose is not None:
                self.hold_pose = hold_pose
            if reason:
                self.error(reason)
            self.setpoint_pub.publish(self.setpoint(self.hold_pose))
        else:
            self.start_landing(reason + '; safe hold unavailable')

    def wait_for_space(self, reason):
        """Suspend a moving curve without completing the operator's task."""
        self.hold_pose = self.copy_pose()
        self.set_pose_yaw(self.hold_pose, self.navigation_heading)
        self.navigation_generation += 1
        self.required_trajectory_id = self.latest_command_id + 1
        self.suspend_planner()
        self.trajectory = None
        self.pending_spline = None
        self.approved_spline_id = None
        self.approved_curve = None
        self.pending_goal = None
        self.future_command_since = None
        self.navigation_stage = 'SPACE_WAIT'
        self.navigation_wait_reason = '悬停等待扫描或可行路线，目标保留'
        self.space_wait_started = rospy.Time.now()
        self.error(reason + '; hovering, goal retained, waiting for scans/replan')
        self.setpoint_pub.publish(self.setpoint(self.hold_pose))

    def navigation_command_failed(self, reason):
        if 'unobserved space' in reason or 'intersects an inflated obstacle voxel' in reason:
            self.wait_for_space(reason)
        else:
            self.stop_navigation(reason)

    def on_voxel_map(self,occupied,free,stamp,epoch,revision):
        # Decode/copy happens outside the control lock. Swap one coherent pair.
        with self.lock:
            if stamp>=self.map_time:
                self.occupied_cells=occupied;self.observed_free=free
                self.map_time=stamp;self.free_time=stamp

    def invalidate_voxel_map(self):
        with self.lock:self.map_time=rospy.Time(0);self.free_time=rospy.Time(0)

    def on_spline(self, msg):
        with self.lock:
            if self.navigation_stage == 'ARRIVAL_CONFIRM':
                return
            # Generation comes from a fresh source spline, never from an old
            # trajectory server refreshing its endpoint command timestamp.
            now = rospy.Time.now()
            age = (now-msg.start_time).to_nsec()/1e9
            if msg.start_time == rospy.Time(0) or not -self.max_traj_age <= age <= self.max_traj_age:
                rospy.logwarn_throttle(2.0, 'Ignoring stale source spline id=%s age=%.6fs', msg.traj_id, age)
                return
            if msg.start_time < self.last_spline_start:
                return
            if msg.start_time > self.last_spline_start and msg.traj_id < self.latest_command_id:
                self.latest_command_id = msg.traj_id
                self.last_spline_start = msg.start_time
                if self.phase == 'NAVIGATING':
                    self.stop_navigation('EGO planner trajectory counter reset; reselect goal after HOLD')
                return
            self.last_spline_start = msg.start_time
            self.latest_command_id = max(self.latest_command_id, msg.traj_id)
            if self.phase != 'NAVIGATING':
                return
            if self.navigation_stage in ('TURNING', 'SPACE_WAIT', 'PLANNING'):
                return
            if msg.start_time < self.goal_start or msg.traj_id < self.required_trajectory_id:
                rospy.logwarn_throttle(2.0, 'Ignoring previous-goal spline id=%s required=%s start=%.9f goal=%.9f',
                                       msg.traj_id, self.required_trajectory_id, msg.start_time.to_sec(), self.goal_start.to_sec())
                return
            if self.approved_spline_id is not None and msg.traj_id <= self.approved_spline_id:
                return
            generation = self.navigation_generation
            age = (rospy.Time.now()-msg.start_time).to_nsec()/1e9
            if (msg.start_time == rospy.Time(0) or
                    not -self.max_traj_age <= age <= self.max_traj_age or not self.fresh_map() or not self.fresh_free_map()):
                self.stop_navigation('Full EGO trajectory has stale time or obstacle map: age=%.6fs map_age=%.6fs' %
                                     (age, (rospy.Time.now()-self.map_time).to_nsec()/1e9))
                return
            occupied = self.occupied_cells
            origin, resolution = self.map_origin[:], self.map_resolution
            lower, upper = self.goal_min[:], self.goal_max[:]
        # Keep the 30 Hz setpoint timer available while checking the curve.
        stats = {}
        reason = validate_curve(msg, occupied, origin, resolution, lower, upper, stats=stats)
        # Unknown portions may be planned. Observed-free checks are applied
        # immediately before execution, after the initial hover/heading turn.
        if reason or stats['wall_s'] > .03:
            rospy.loginfo('Full spline %s validation: %s; wall=%.4fs cpu=%.4fs nodes=%s',
                          msg.traj_id, reason or 'approved', stats['wall_s'], stats['cpu_s'], stats['nodes'])
        with self.lock:
            if self.phase != 'NAVIGATING' or generation != self.navigation_generation:
                return
            if not reason and self.first_session_curve:
                spline=curve_geometry(msg)[3]
                start=spline(msg.knots[msg.order]);p=self.current_pose().position
                gap=math.dist(start,[p.x,p.y,p.z])
                if gap>.25:
                    self.wait_for_space('New-session curve start displaced %.3f m from held pose' % gap)
                    return
                self.first_session_curve=False
            if reason:
                self.navigation_command_failed(reason)
            else:
                # A spline and /clock use independent TCPROS connections.
                # Validate geometry now, but never activate before its start.
                self.pending_spline = (msg.traj_id, msg.start_time, generation, rospy.Time.now(), msg)
                self.activate_pending_spline()

    def activate_pending_spline(self):
        pending = self.pending_spline
        if pending is None:
            return
        traj_id, start, generation, received, curve = pending
        now = rospy.Time.now()
        age = (now-start).to_nsec()/1e9
        if self.phase != 'NAVIGATING' or generation != self.navigation_generation:
            self.pending_spline = None
        elif age > self.max_traj_age or (age < 0 and (now-received).to_nsec()/1e9 >= self.max_traj_age):
            self.pending_spline = None
            self.stop_navigation('Full EGO trajectory clock mismatch or expiry before activation')
        elif age >= 0:
            if not self.fresh_map() or not self.fresh_free_map():
                self.pending_spline = None
                self.stop_navigation('Obstacle map became stale before full trajectory activation')
            else:
                self.approved_spline_id = traj_id
                self.approved_curve = curve
                self.navigation_stage='TRACKING' if self.navigation_heading_ready else 'WAITING'
                self.navigation_wait_reason='轨迹已确认，检查前方自由空间'
                self.prefix_check_time = rospy.Time(0)
                self.pending_spline = None

    def on_trajectory(self, msg):
        if self.check_executable_prefix(msg):
            self.accept_command(msg)

    def check_executable_prefix(self, msg):
        with self.lock:
            if self.navigation_stage == 'ARRIVAL_CONFIRM':
                return False
            if self.phase == 'NAVIGATING' and self.navigation_stage in ('TURNING', 'SPACE_WAIT', 'PLANNING'):
                return False
            self.activate_pending_spline()
            if (self.phase != 'NAVIGATING' or not self.require_observed_free or not self.navigation_heading_ready or
                    self.trajectory_needs_turn(msg) or
                    msg.trajectory_id != self.approved_spline_id):
                return True
            now=rospy.Time.now()
            if self.prefix_check_time != rospy.Time(0) and (now-self.prefix_check_time).to_nsec()/1e9 < .1:
                return True
            curve=self.approved_curve
            generation=self.navigation_generation
            if curve is None or not self.fresh_map() or not self.fresh_free_map():
                self.stop_navigation('Missing curve or fresh map for executable trajectory prefix')
                return False
            free,occupied=self.observed_free,self.occupied_cells
            origin,resolution=self.map_origin[:],self.map_resolution
            lower,upper=self.goal_min[:],self.goal_max[:]
            offset=max(0.0,(now-curve.start_time).to_nsec()/1e9)
            v=self.odom.twist.twist.linear
            speed=max(self.active_navigation_speed, math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z))
            # Braking at the configured 0.5 m/s^2, plus acquisition and
            # command age margins. Never shorten the minimum scan horizon.
            horizon=max(self.known_horizon, speed/.5+self.max_pose_age+self.max_traj_age+.25)
        # Do not block continuous OFFBOARD publication while checking lookahead.
        reason=validate_curve(curve,occupied,origin,resolution,lower,upper,observed_free=free,
                              curve_window=(offset,offset+horizon))
        with self.lock:
            if (self.phase != 'NAVIGATING' or generation != self.navigation_generation or
                    msg.trajectory_id != self.approved_spline_id):
                return False
            if reason:
                self.navigation_command_failed(reason+'; executable lookahead')
                return False
            if not self.fresh_map() or not self.fresh_free_map():
                self.stop_navigation('Map expired during executable trajectory prefix check')
                return False
            self.prefix_check_time=now
            return True

    def accept_command(self, msg):
        with self.lock:
            if self.navigation_stage == 'ARRIVAL_CONFIRM':
                return
            if self.phase == 'NAVIGATING' and self.navigation_stage in ('TURNING', 'SPACE_WAIT', 'PLANNING'):
                return
            if (msg.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY or
                    msg.header.frame_id != 'odom' or msg.trajectory_id < self.required_trajectory_id):
                return
            if self.phase != 'NAVIGATING':
                # An endpoint republished with a fresh header is not a newly
                # generated spline and must not raise the next-goal threshold.
                return
            self.activate_pending_spline()
            if self.phase != 'NAVIGATING' or msg.trajectory_id != self.approved_spline_id:
                return
            age = (rospy.Time.now()-msg.header.stamp).to_nsec()/1e9
            if -self.max_traj_age <= age < -self.future_tolerance:
                # Separate topics have independent TCPROS delivery. Never use
                # an ahead-of-clock command, but a single delayed clock packet
                # must not cancel an otherwise fresh, safe command stream.
                if self.future_command_since is None:
                    self.future_command_since = rospy.Time.now()
                rospy.logwarn_throttle(2.0, 'Dropping future EGO command (age %.6f s)', age)
                return
            reason = self.command_error(msg)
            if reason:
                self.navigation_command_failed(reason)
                return
            self.future_command_since = None
            self.latest_command_id = max(self.latest_command_id, msg.trajectory_id)
            self.trajectory = msg
            self.trajectory_time = msg.header.stamp

    def set_authorized(self, request):
        with self.lock:
            if not request.data and self.airborne():
                self.start_landing('Operator revoked authorization')
            elif not request.data and self.phase in ('PRIMING_ARM', 'WAIT_OFFBOARD', 'WAIT_ARM'):
                self.set_phase('READY')
            elif not request.data and self.state.armed:
                self.disarm(None)
            self.authorized = request.data
            if request.data and self.ai_expiry_reported and not self.state.armed:self.ai_flight_owned=''
            self.auth_pub.publish(Bool(data=self.authorized))
            return SetBoolResponse(success=True, message='Authorized' if self.authorized else 'Authorization revoked')

    def arm(self, _request, ai=False):
        with self.lock:
            if (not self.map_alignment_ready or not self.authorized or not self.fresh_pose() or self.state.armed or
                    self.airborne() or self.phase != 'READY'):
                return TriggerResponse(success=False, message='Need authorization, fresh LIO/PX4 pose, and landed state')
            self.hold_pose = self.copy_pose()
            self.prime_start = rospy.Time.now()
            self.set_phase('PRIMING_ARM')
            if not ai:self.ai_flight_owned=''
            return TriggerResponse(success=True, message='Priming OFFBOARD setpoints, then arming')

    def takeoff(self, request, ai=False):
        with self.lock:
            if self.lio_quality != 'HEALTHY':
                return TakeoffResponse(success=False, message='Takeoff requires healthy LIO geometry')
            if not self.authorized or not self.state.armed or not self.fresh_pose():
                return TakeoffResponse(success=False, message='Need authorization, ARM and fresh position')
            if (self.phase != 'ARMED' or self.state.mode != 'OFFBOARD' or
                    not math.isfinite(request.height_m) or not 0 < request.height_m <= self.max_takeoff_height):
                return TakeoffResponse(success=False, message='Invalid state or takeoff height')
            self.hold_pose = self.copy_pose()
            self.takeoff_pose = self.copy_pose()
            if self.hold_pose.position.z+request.height_m > self.goal_max[2]:
                return TakeoffResponse(False, 'Takeoff target exceeds virtual ceiling safety margin')
            self.takeoff_pose.position.z += request.height_m
            self.takeoff_start_z = self.hold_pose.position.z
            self.takeoff_start = rospy.Time.now()
            self.set_phase('TAKEOFF')
            if not ai:self.ai_flight_owned=''
            return TakeoffResponse(success=True, message='Takeoff setpoint streaming')

    def disarm(self, _request):
        with self.lock:
            if not self.state.armed or self.extended.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
                return TriggerResponse(success=False, message='Disarm requires confirmed ground contact and armed state')
            try:
                reply = self.arm_client(False)
                return TriggerResponse(success=reply.success, message='Ground disarm requested' if reply.success else 'PX4 rejected disarm')
            except rospy.ServiceException as exc:
                return TriggerResponse(success=False, message=str(exc))

    def hold(self, _request, health_override=False, preserve_queue=False):
        with self.lock:
            if not self.airborne() or self.odom is None or (not health_override and not self.fresh_pose()):
                return TriggerResponse(success=False, message='Hold requires airborne vehicle and valid position')
            if not preserve_queue:
                self.clear_goal_queue()
            self.suspend_planner()
            self.trajectory = None
            self.pending_spline = None
            self.future_command_since = None
            self.navigation_generation += 1
            self.required_trajectory_id = self.latest_command_id + 1
            self.pending_goal = None
            self.active_goal = None
            self.goal_arrival_pose = None
            self.set_terminal_stage('IDLE')
            self.navigation_stage = 'WAITING'
            self.navigation_heading_ready = False
            self.goal_reached_since = None
            self.hold_pose = (copy.deepcopy(self.last_healthy_pose) if health_override and self.last_healthy_pose is not None
                              else self.copy_pose())
            self.set_phase('HOLD')
            return TriggerResponse(success=True, message='Current position held; previous trajectory ignored')

    def rotate_once(self, _request):
        with self.lock:
            if self.lio_quality != 'HEALTHY':
                return TriggerResponse(success=False, message='Rotation requires healthy LIO geometry')
            if (not self.map_alignment_ready or not self.authorized or not self.airborne() or not self.fresh_pose() or
                    self.phase != 'HOLD' or self.state.mode != 'OFFBOARD' or self.goal_queue):
                return TriggerResponse(success=False, message='Rotation requires authorized airborne HOLD and fresh pose')
            self.hold_pose = self.copy_pose()
            q = self.hold_pose.orientation
            self.rotate_yaw_start = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.rotate_start = rospy.Time.now()
            self.suspend_planner()
            self.set_phase('ROTATING')
            return TriggerResponse(success=True, message='Holding position and rotating 360 degrees at 10 deg/s')

    def goal_error(self, goal):
        p = goal.pose.position
        values = [p.x, p.y, p.z]
        if not self.map_alignment_ready:
            return 'Map initial pose has not been confirmed'
        if self.lio_quality != 'HEALTHY':
            return 'New goal requires recovered LIO geometry'
        if (not self.authorized or not self.fresh_pose() or not self.airborne() or
                self.phase != 'HOLD' or goal.header.frame_id != 'odom' or
                not all(math.isfinite(v) for v in values)):
            return 'Need authorized airborne HOLD, valid odom, and finite ENU goal'
        if not self.in_flight_volume(values):
            return 'Goal is outside configured flight volume'
        if not self.fresh_map() or not self.fresh_free_map():
            return 'Need fresh obstacle and observed free-space maps before navigation'
        if any(cell in self.occupied_cells for cell in self.segment_cells(values, values)):
            return 'Goal is inside an inflated obstacle voxel'
        return ''

    def local_goal(self, request, preserve_queue=False, inspection=False):
        with self.lock:
            if self.inspection_reserved and not inspection:
                return LocalGoalResponse(False,'请先取消巡检任务再发送普通目标')
            goal=copy.deepcopy(request.goal)
            reason=self.goal_error(goal)
            if reason:return LocalGoalResponse(False,reason)
            if not preserve_queue:self.clear_goal_queue()
            self.suspend_planner()
            self.active_navigation_speed=self.next_navigation_speed
            self.trajectory=None;self.pending_spline=None;self.future_command_since=None
            self.navigation_generation+=1
            ns=max(rospy.Time.now().to_nsec(),self.route_sequence+1)
            goal.header.stamp=rospy.Time(ns//1000000000,ns%1000000000)
            self.route_sequence=goal.header.stamp.to_nsec()
            self.global_route=None
            self.required_trajectory_id=self.latest_command_id+1
            self.active_goal=goal;self.pending_goal=None
            self.navigation_heading_ready=False
            self.hold_pose=self.copy_pose()
            self.navigation_heading=self.pose_yaw(self.hold_pose)
            self.approved_spline_id=None;self.approved_curve=None
            self.goal_reached_since=None
            self.goal_arrival_pose=None
            self.terminal_last_curve_id=None
            self.set_terminal_stage('CRUISE')
            self.set_phase('NAVIGATING')
            self.navigation_goal_pub.publish(goal)
            self.yaw_last_tick=rospy.Time.now()
            self.request_planner(goal)
            self.publish_goal_queue()
            return LocalGoalResponse(True,'当前目标开始规划，速度 %.2f m/s' % self.active_navigation_speed)

    def publish_goal_queue(self):
        message=Path();message.header=Header(stamp=rospy.Time.now(),frame_id='odom')
        message.poses=([copy.deepcopy(self.active_goal)] if self.active_goal is not None else [])+[
            copy.deepcopy(goal) for goal in self.goal_queue]
        self.queue_pub.publish(message)

    def clear_goal_queue(self):
        self.goal_queue=[]
        self.queue_dispatch_after=None
        self.queue_pub.publish(Path(header=Header(stamp=rospy.Time.now(),frame_id='odom')))

    def queue_goal(self, request):
        with self.lock:
            if self.inspection_task or self.inspection_reserved:
                return QueueGoalResponse(False,'巡检正在执行，请先暂停或取消巡检',0)
            goal=copy.deepcopy(request.goal);p=goal.pose.position
            if (not self.map_alignment_ready or not self.authorized or not self.airborne() or not self.fresh_pose() or
                    self.lio_quality!='HEALTHY' or self.state.mode!='OFFBOARD' or
                    self.phase not in ('HOLD','NAVIGATING')):
                return QueueGoalResponse(False,'Queue requires healthy authorized airborne HOLD/NAVIGATING',len(self.goal_queue))
            if (goal.header.frame_id!='odom' or not self.in_flight_volume([p.x,p.y,p.z]) or
                    not self.fresh_map() or not self.fresh_free_map()):
                return QueueGoalResponse(False,'Invalid ENU goal, ceiling, or stale map',len(self.goal_queue))
            if any(c in self.occupied_cells for c in self.segment_cells([p.x,p.y,p.z],[p.x,p.y,p.z])):
                return QueueGoalResponse(False,'此点位于已知障碍内，已拒绝；其余目标队列保留',len(self.goal_queue)+(self.active_goal is not None))
            if len(self.goal_queue)+(self.active_goal is not None)>=self.max_queue_points:
                return QueueGoalResponse(False,'Queue limit is 100 points',len(self.goal_queue))
            if self.phase=='HOLD' and not self.goal_queue:
                reply=self.local_goal(request)
                return QueueGoalResponse(reply.success,reply.message,1 if reply.success else 0)
            # No publication to navigation_goal, no search or EGO call for future points.
            self.goal_queue.append(goal);self.publish_goal_queue()
            return QueueGoalResponse(True,'目标已加入队列；上一点到达后才开始规划此点',
                                     len(self.goal_queue)+(self.active_goal is not None))

    def dispatch_queued_goal(self, now):
        if not self.goal_queue or self.queue_dispatch_after is None or now<self.queue_dispatch_after:
            return
        if not self.fresh_pose() or self.lio_quality!='HEALTHY' or self.health_fault_since is not None:
            return
        # A blocked queued point is skipped only in online mapping mode.
        while self.goal_queue:
            goal=self.goal_queue[0]
            reason=self.goal_error(goal)
            if reason=='Goal is inside an inflated obstacle voxel' and self.navigation_map_mode=='MAPPING':
                p=goal.pose.position;self.goal_queue.pop(0)
                self.error('跳过障碍内目标 [%.3f %.3f %.3f]；继续后续队列'%(p.x,p.y,p.z))
                self.publish_goal_queue()
                continue
            if reason:
                self.clear_goal_queue();self.error('Queued mission cancelled: '+reason);return
            break
        else:
            self.queue_dispatch_after=None;self.publish_goal_queue();return
        self.goal_queue.pop(0);self.queue_dispatch_after=None
        reply=self.local_goal(SimpleNamespace(goal=goal),preserve_queue=True)
        if not reply.success:
            self.clear_goal_queue();self.error('Queued mission cancelled: '+reply.message)

    def set_max_flight_height(self, request):
        with self.lock:
            height=request.height_m
            if not math.isfinite(height) or not .8<=height<=self.configured_max_height:
                return SetMaxFlightHeightResponse(False,self.max_flight_height,'Height must be 0.8..%.2f m ENU Z'%self.configured_max_height)
            ground=(not self.state.armed and self.extended.landed_state==ExtendedState.LANDED_STATE_ON_GROUND)
            if not ground and (self.phase!='HOLD' or not self.fresh_pose() or self.goal_queue or
                               self.lio_quality!='HEALTHY' or self.state.mode!='OFFBOARD'):
                return SetMaxFlightHeightResponse(False,self.max_flight_height,'Change ceiling on ground or in healthy HOLD')
            if not ground and self.current_pose().position.z>height-self.ceiling_margin:
                return SetMaxFlightHeightResponse(False,self.max_flight_height,'Ceiling must leave 0.20 m above current altitude')
            self.max_flight_height=height;self.goal_max[2]=height-self.ceiling_margin
            rospy.set_param('/drone/max_flight_height_m',height)
            self.ceiling_pub.publish(Float64(data=height))
            return SetMaxFlightHeightResponse(True,height,'最高高度已设置：%.2f m（ENU Z），目标与规划保留0.20 m裕量'%height)

    def follow_navigation_heading(self, heading, now):
        dt=max(0.,min(.10,(now-self.yaw_last_tick).to_sec())) if self.yaw_last_tick!=rospy.Time(0) else 0.
        self.yaw_last_tick=now
        delta=self.angle_delta(heading,self.navigation_heading)
        self.navigation_heading=self.angle_delta(self.navigation_heading+max(-self.heading_rate*dt,min(self.heading_rate*dt,delta)),0.)

    def set_navigation_speed(self, request):
        with self.lock:
            if not math.isfinite(request.speed_mps) or not 0.1 <= request.speed_mps <= 1.0:
                return SetNavigationSpeedResponse(False, self.next_navigation_speed,
                                                  'Speed must be between 0.1 and 1.0 m/s')
            self.next_navigation_speed = request.speed_mps
            return SetNavigationSpeedResponse(True, self.next_navigation_speed,
                                              'Next navigation task speed: %.2f m/s' % self.next_navigation_speed)

    def land(self, _request):
        with self.lock:
            if not self.state.armed:
                return TriggerResponse(success=False, message='Vehicle is not armed')
            self.start_landing('Operator requested landing')
            return TriggerResponse(success=True, message=self.phase)

    def start_landing(self, reason):
        self.set_terminal_stage('IDLE')
        self.clear_goal_queue()
        self.navigation_generation += 1
        self.suspend_planner()
        self.trajectory = None
        self.pending_goal = None
        self.error(reason)
        self.active_goal = None
        self.navigation_stage = 'WAITING'
        self.navigation_heading_ready = False
        self.goal_reached_since = None
        try:
            reply = self.mode_client(custom_mode='AUTO.LAND')
            if reply.mode_sent:
                self.set_phase('LANDING')
                return
        except rospy.ServiceException as exc:
            self.error('AUTO.LAND service failed: ' + str(exc))
        if self.fresh_pose():
            self.hold_pose = self.copy_pose()
            self.land_start = rospy.Time.now()
            self.set_phase('DESCENDING')
        else:
            self.set_phase('FAILSAFE')
            self.error('AUTO.LAND rejected and local position is unavailable; PX4 failsafe/manual takeover required')

    def copy_pose(self):
        import copy
        return copy.deepcopy(self.current_pose())

    @staticmethod
    def angle_delta(target, current):
        return math.atan2(math.sin(target-current), math.cos(target-current))

    @staticmethod
    def pose_yaw(pose):
        q = pose.orientation
        return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    @staticmethod
    def set_pose_yaw(pose, yaw):
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

    def motion_heading(self, command):
        """Forward XY tangent of the approved curve in increasing knot time."""
        curve = self.approved_curve
        if curve is None or command.trajectory_id != self.approved_spline_id:
            return None
        cached = self.heading_curve_cache
        if cached is None or cached[0] is not curve:
            spline = curve_geometry(curve)[3]
            begin, end = curve.knots[curve.order], curve.knots[len(curve.pos_pts)]
            times = np.linspace(begin, end, min(1025, max(65, int((end-begin)/.025)+1)))
            xy = spline(times)[:, :2]
            arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))))
            cached = (curve, spline.derivative(), begin, end, times, arc)
            self.heading_curve_cache = cached
        _, tangent, begin, end, times, arc = cached
        # Match the parameter of the position command being considered, not
        # the measured vehicle velocity or the direction of tracking error.
        t = min(end, max(begin, begin+(command.header.stamp-curve.start_time).to_sec()))
        if not self.navigation_heading_ready or t-begin <= .3:
            # A stationary start has a zero tangent. Find a directed tangent
            # about 10 cm along the same curve, avoiding the initial transient.
            current_arc = float(np.interp(t, times, arc))
            if arc[-1]-current_arc < .005:
                return None
            index = int(np.searchsorted(arc, min(current_arc+.10, arc[-1]), side='left'))
            # Endpoint derivative is zero too; approach it from within the
            # forward portion without reversing the route's direction.
            t = min(float(times[index]), end-min(.025, (end-begin)*.01))
        direction = tangent(t)
        minimum_speed = .05 if self.navigation_heading_ready and t-begin > .3 else .001
        if not np.isfinite(direction).all() or math.hypot(direction[0], direction[1]) < minimum_speed:
            return None  # Pure vertical/finished curve: retain the current yaw.
        return math.atan2(direction[1], direction[0])

    def trajectory_needs_turn(self, msg):
        heading = self.motion_heading(msg)
        return heading is not None and (
            not self.navigation_heading_ready or
            abs(self.angle_delta(heading, self.navigation_heading)) > self.heading_half_cone or
            abs(self.angle_delta(heading, self.pose_yaw(self.current_pose()))) > self.heading_half_cone)

    def begin_heading_alignment(self, heading):
        self.hold_pose = self.copy_pose()
        self.navigation_heading = self.pose_yaw(self.hold_pose)
        self.heading_target = self.angle_delta(heading, 0.0)
        self.heading_started = rospy.Time.now()
        self.heading_last_tick = self.heading_started
        self.heading_stable_since = None
        self.heading_brake_since = None
        self.heading_braked = False
        self.navigation_stage = 'TURNING'
        self.navigation_wait_reason = '正在悬停转向，等待机头与速度稳定'
        # Cancel the moving curve, while retaining the operator's goal.
        self.navigation_generation += 1
        self.required_trajectory_id = self.latest_command_id + 1
        self.suspend_planner()
        self.trajectory = None
        self.pending_spline = None
        self.approved_spline_id = None
        self.approved_curve = None
        self.pending_goal = None
        self.future_command_since = None
        rospy.loginfo('Navigation heading alignment: target=%.2f deg; position held, goal retained',
                      math.degrees(self.heading_target))

    def tick_heading_alignment(self, now):
        if (now-self.heading_started).to_sec() > 20.0:
            self.stop_navigation('Heading alignment timed out; navigation cancelled')
            return
        v = self.odom.twist.twist.linear
        speed = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
        if not self.heading_braked:
            self.setpoint_pub.publish(self.setpoint(self.hold_pose))
            if speed > 0.1:
                self.heading_brake_since = None
            elif self.heading_brake_since is None:
                self.heading_brake_since = now
            elif (now-self.heading_brake_since).to_sec() >= 0.2:
                self.heading_braked = True
            self.heading_last_tick = now
            return
        dt = max(0.0, min(0.1, (now-self.heading_last_tick).to_sec()))
        self.heading_last_tick = now
        delta = self.angle_delta(self.heading_target, self.navigation_heading)
        step = max(-self.heading_rate*dt, min(self.heading_rate*dt, delta))
        self.navigation_heading = self.angle_delta(self.navigation_heading+step, 0.0)
        self.set_pose_yaw(self.hold_pose, self.navigation_heading)
        self.setpoint_pub.publish(self.setpoint(self.hold_pose))
        actual_error = abs(self.angle_delta(self.heading_target, self.pose_yaw(self.current_pose())))
        # Wait for braking, actual attitude and a new scan during the turn.
        aligned = (actual_error <= self.heading_tolerance and speed <= 0.1 and
                   self.map_time >= self.heading_started and
                   abs(self.angle_delta(self.heading_target, self.navigation_heading)) < 1e-3 and
                   self.lio_quality == 'HEALTHY')
        if not aligned:
            self.navigation_wait_reason = '转向后等待朝向、速度和定位稳定'
            self.heading_stable_since = None
            return
        if self.heading_stable_since is None:
            self.heading_stable_since = now
            return
        if (now-self.heading_stable_since).to_sec() < 0.3:
            return
        if self.map_time < self.heading_stable_since:
            self.navigation_wait_reason = '朝向已稳定，等待对准方向后的新扫描'
            return
        if self.active_goal is None:
            self.stop_navigation('Navigation goal disappeared during heading alignment')
            return
        import copy
        goal = copy.deepcopy(self.active_goal)
        # Retain the original task stamp across heading/replanning sessions.
        self.navigation_heading = self.heading_target
        self.yaw_last_tick = now
        self.navigation_heading_ready = True
        self.pending_goal = None
        self.request_planner(goal)
        rospy.loginfo('Navigation heading aligned at %.2f deg; preparing a new current-pose planning session',
                      math.degrees(self.navigation_heading))

    def set_terminal_stage(self, stage):
        if self.terminal_stage != stage:
            self.terminal_stage = stage
            self.terminal_stage_pub.publish(String(data=stage))
            rospy.loginfo('Terminal navigation stage: %s', stage)

    def tick_goal_arrival(self, now):
        """Strict arrival dwell with a separate, bounded convergence exit gate."""
        if self.active_goal is None or self.inspection_continuous:
            return False
        if self.inspection_task and self.global_route is None:return False
        if self.inspection_task and self.global_route is not None:
            # Adjacent coverage lanes can pass close to the final endpoint.
            # Only the remaining directed arc, not Euclidean proximity, ends a mission.
            pts=[p.pose.position for p in self.global_route.poses]
            if sum(math.dist([a.x,a.y,a.z],[b.x,b.y,b.z]) for a,b in zip(pts,pts[1:]))>.20:
                return False
        p = self.current_pose().position
        g = self.active_goal.pose.position
        v = self.odom.twist.twist.linear
        distance = math.dist([p.x, p.y, p.z], [g.x, g.y, g.z])
        speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        fresh = self.fresh_pose()
        reached = fresh and distance <= self.goal_tolerance and speed <= self.goal_speed_tolerance
        confirming = self.navigation_stage == 'ARRIVAL_CONFIRM'
        if reached and not confirming:
            self.arrival_started = now
            self.arrival_outside_since = None
            self.arrival_curve = self.approved_curve
            self.goal_arrival_pose = self.copy_pose()
            # Only use the endpoint of the SAME previously approved final curve.
            # Otherwise hold measured pose; never invent an unchecked shortcut.
            if self.arrival_curve is not None:
                c = self.arrival_curve
                f = curve_geometry(c)[3]
                endpoint = f(c.knots[len(c.pos_pts)])
                if (math.dist(endpoint, [g.x,g.y,g.z]) <= .02 and
                        np.linalg.norm(f.derivative()(c.knots[len(c.pos_pts)])) <= .03):
                    self.goal_arrival_pose.position = Point(*map(float, endpoint))
            self.navigation_generation += 1
            self.required_trajectory_id = self.latest_command_id + 1
            self.suspend_planner()
            self.trajectory = None
            self.pending_spline = None
            self.approved_curve = None
            self.approved_spline_id = None
            self.future_command_since = None
            self.navigation_stage = 'ARRIVAL_CONFIRM'
            self.goal_reached_since = now
            confirming = True
            rospy.loginfo('Arrival confirmation entered: distance=%.6f speed=%.6f, exit=%.3f',
                          distance, speed, self.arrival_exit_tolerance)
        if not confirming:
            self.goal_reached_since = None
            return False
        # Data/velocity flicker or crossing 15 cm resets dwell, not planning/yaw.
        if reached:
            if self.goal_reached_since is None:
                self.goal_reached_since = now
            if (now-self.goal_reached_since).to_sec() >= 1.0:
                rospy.loginfo('Goal reached [%.3f %.3f %.3f]; remaining queued points %d',
                              g.x, g.y, g.z, len(self.goal_queue))
                self.stop_navigation('', hold_pose=self.goal_arrival_pose, preserve_queue=True)
                self.set_terminal_stage('COMPLETED')
                self.queue_dispatch_after = now + rospy.Duration(.30)
                self.publish_goal_queue()
                self.arrival_curve = None
                return True
        else:
            self.goal_reached_since = None
        outside = fresh and distance > self.arrival_exit_tolerance
        if outside:
            if self.arrival_outside_since is None:
                self.arrival_outside_since = now
        else:
            self.arrival_outside_since = None
        persistent_exit = self.arrival_outside_since is not None and (now-self.arrival_outside_since).to_sec() >= .5
        timed_out = (now-self.arrival_started).to_sec() > self.terminal_settle_time + 1.0
        if fresh and (persistent_exit or timed_out):
            self.hold_pose = self.copy_pose()
            self.navigation_heading = self.pose_yaw(self.hold_pose)
            self.goal_arrival_pose = None
            self.arrival_curve = None
            self.set_terminal_stage('APPROACH')
            self.request_planner(copy.deepcopy(self.active_goal))
            rospy.loginfo('Arrival convergence retry: distance=%.6f speed=%.6f reason=%s',
                          distance, speed, 'outside exit band for 0.5 s' if persistent_exit else 'bounded convergence timeout')
            return True
        # Independent collision/free-space/volume checks apply on EVERY tick.
        # Tiny terminal corrections preserve yaw; ±60° turning is for travel.
        command = PositionCommand(header=Header(stamp=now, frame_id='odom'))
        command.position = self.goal_arrival_pose.position
        command.yaw = self.pose_yaw(self.goal_arrival_pose)
        reason = self.command_error(command) if fresh else 'arrival pose temporarily stale'
        endpoint = [command.position.x,command.position.y,command.position.z]
        if not reason and self.require_observed_free:
            if not self.fresh_free_map() or any(c not in self.observed_free for c in self.segment_cells([p.x,p.y,p.z],endpoint)):
                reason = 'Arrival correction enters unobserved space'
        if reason:
            # Never continue an unsafe correction or turn based on its error.
            self.setpoint_pub.publish(self.setpoint(self.copy_pose()))
            self.navigation_wait_reason = '终点收敛暂停：'+reason
        else:
            target = self.setpoint(self.goal_arrival_pose)
            target.type_mask &= ~(PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ)
            self.setpoint_pub.publish(target)
        self.set_terminal_stage('ARRIVAL_CONFIRM')
        if not reason:
            self.navigation_wait_reason = '终点收敛：15cm内低速稳定1秒；轻微越界不立即重规划'
        return True

    def update_terminal_approach(self):
        if self.inspection_continuous:return
        if self.terminal_stage != 'CRUISE' or self.active_goal is None or self.global_route is None:
            return
        p = self.current_pose().position
        limit = max(self.terminal_approach_distance,
                    self.active_navigation_speed**2/(2*self.terminal_deceleration)+.3)
        g = self.active_goal.pose.position
        if math.dist([p.x, p.y, p.z], [g.x, g.y, g.z]) > limit:
            return
        points = [[x.pose.position.x, x.pose.position.y, x.pose.position.z] for x in self.global_route.poses]
        if len(points) < 2:
            return
        remaining = math.dist([p.x, p.y, p.z], points[0]) + sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        if remaining <= limit:
            self.set_terminal_stage('APPROACH')

    def tick_terminal_endpoint(self, now):
        if self.inspection_continuous:return False
        """Settle at the SAME approved zero-speed curve endpoint, bounded in time.

        No new path or direct unchecked goal command is manufactured. Each
        small tracking segment still passes occupied/observed-free checks.
        """
        curve = self.approved_curve
        if curve is None or self.active_goal is None or not self.navigation_heading_ready:
            return False
        duration = curve.knots[len(curve.pos_pts)]-curve.knots[curve.order]
        elapsed = (now-curve.start_time).to_sec()
        if elapsed < duration:
            return False
        # Cache values from this exact approved control spline.
        if self.terminal_curve_cache is None or self.terminal_curve_cache[0] is not curve:
            spline = curve_geometry(curve)[3]
            end_time = curve.knots[len(curve.pos_pts)]
            self.terminal_curve_cache = (curve, spline(end_time), spline.derivative()(end_time))
        _, endpoint, velocity = self.terminal_curve_cache
        g = self.active_goal.pose.position
        p = self.current_pose().position
        if math.dist(endpoint, [g.x, g.y, g.z]) > .02 or float(np.linalg.norm(velocity)) > .03:
            return False
        if math.dist(endpoint, [p.x, p.y, p.z]) > min(.4, self.max_setpoint_step):
            return False
        if elapsed-duration > self.terminal_settle_time:
            self.wait_for_space('Terminal convergence timed out; requesting a fresh short trajectory')
            return True
        # Do not command a backward correction beyond arrival tolerance. A
        # new actual EGO curve supplies any required turn, never tracking error.
        dx, dy = float(endpoint[0])-p.x, float(endpoint[1])-p.y
        if math.hypot(dx, dy) > self.goal_tolerance and abs(self.angle_delta(math.atan2(dy, dx), self.pose_yaw(self.current_pose()))) > self.heading_half_cone:
            return False
        command = PositionCommand()
        command.header = Header(stamp=now, frame_id='odom')
        command.trajectory_id = self.approved_spline_id
        command.yaw = self.navigation_heading
        command.position = Point(x=float(endpoint[0]), y=float(endpoint[1]), z=float(endpoint[2]))
        reason = self.command_error(command)
        start = [p.x, p.y, p.z]
        if not reason and self.require_observed_free and any(c not in self.observed_free for c in self.segment_cells(start, endpoint)):
            reason = 'Terminal tracking segment enters unobserved space'
        if reason:
            self.navigation_command_failed(reason)
            return True
        if self.terminal_last_curve_id != self.approved_spline_id:
            rospy.loginfo('Settling approved terminal curve %d, endpoint error %.3f m',
                          self.approved_spline_id, math.dist(endpoint, start))
            self.terminal_last_curve_id = self.approved_spline_id
        self.set_terminal_stage('CONVERGING')
        self.navigation_wait_reason = '末段已平稳停止，持续跟踪批准曲线终点以收敛位置'
        pose = self.copy_pose()
        pose.position = command.position
        self.set_pose_yaw(pose, self.navigation_heading)
        target = self.setpoint(pose)
        target.type_mask &= ~(PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ)
        # The approved endpoint has zero desired velocity; this is feedforward,
        # never an external velocity measurement for the PX4 EKF.
        self.setpoint_pub.publish(target)
        self.publish_execution_curve(curve)
        return True

    @staticmethod
    def setpoint(pose):
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'odom'
        # MAVLink enum names NED; numeric ROS fields remain ENU. MAVROS converts.
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW_RATE)
        target.position = pose.position
        q = pose.orientation
        target.yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return target

    @staticmethod
    def arming_setpoint(pose):
        # A ground position target can request takeoff after an EKF altitude
        # reset. Zero ENU velocity keeps OFFBOARD primed without requesting
        # ascent; only the explicit takeoff command enables position control.
        target = FlightManager.setpoint(pose)
        target.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                            PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW_RATE)
        return target

    def tick(self, event):
        with self.lock:
            now = rospy.Time.now()
            if self.last_navigation_stage_published != self.navigation_stage:
                self.navigation_stage_pub.publish(String(data=self.navigation_stage))
                self.last_navigation_stage_published = self.navigation_stage
            if self.last_wait_reason_published != self.navigation_wait_reason:
                self.wait_reason_pub.publish(String(data=self.navigation_wait_reason))
                self.last_wait_reason_published=self.navigation_wait_reason
            self.heartbeat_pub.publish(Header(stamp=now, frame_id='odom'))
            pose_ok = self.fresh_pose(require_startup=False)
            severe = self.clock_fault or self.lio_severe or self.lio_quality == 'SEVERE' or (self.airborne() and not self.state.connected)
            if not pose_ok:
                if self.health_fault_since is None:
                    self.health_fault_since = now
            else:
                self.health_fault_since = None
                self.last_healthy_pose = self.copy_pose()
            if not self.state.armed:
                self.health_landing_latched = False
                if self.goal_queue or self.active_goal is not None:
                    self.clear_goal_queue()
                    self.active_goal=None
                    self.publish_goal_queue()
            elapsed = max(0., (now-self.health_fault_since).to_nsec()/1e9) if self.health_fault_since is not None else 0.
            quality = ('SEVERE' if severe or elapsed >= self.health_land_s or self.health_landing_latched else
                       'HOLD' if elapsed >= self.health_hold_s or self.lio_quality == 'HOLD' else
                       'WARNING' if not pose_ok or self.lio_quality == 'WARNING' else 'HEALTHY')
            self.flight_health_pub.publish(String(data=quality))
            display_valid = quality != 'SEVERE' and (pose_ok or self.airborne())
            self.flight_valid_pub.publish(Bool(data=display_valid))
            self.flight_snapshot_pub.publish(String(data=json.dumps(dict(
                stamp_ns=now.to_nsec(), quality=quality, valid=display_valid,
                reason=self.pose_health_detail))))
            if self.airborne() and self.phase not in ('LANDING', 'DESCENDING', 'FAILSAFE'):
                if quality == 'SEVERE':
                    self.health_landing_latched = True
                    self.start_landing('Severe health fault or health unavailable for %.1f s: '%self.health_land_s+self.pose_health_detail)
                elif quality == 'HOLD' and (self.phase != 'HOLD' or self.goal_queue):
                    # Freeze at the last accepted pose; never use invalid/new data
                    # to resume the cancelled task. Keep sending OFFBOARD targets.
                    reply = self.hold(None, health_override=True)
                    if reply.success:
                        self.error('Health degraded for %.1f s; task cancelled, holding: '%self.health_hold_s+self.pose_health_detail)
            task_canceled=(self.ai_task_canceled or self.ai_request_id!=self.ai_flight_request) and self.phase in ('PRIMING_ARM','WAIT_OFFBOARD','WAIT_ARM','TAKEOFF')
            if self.state.connected and self.ai_flight_owned and (self.ai_expiry_reported or task_canceled or not self.ai_valid(self.ai_flight_session)):
                if not self.ai_expiry_reported:
                    self.ai_expiry_reported=True;self.error('AI执行授权或连接失效，停止AI起飞/解锁流程')
                if not self.state.armed:
                    self.authorized=False;self.auth_pub.publish(Bool(False));self.set_phase('READY')
                elif self.extended.landed_state==ExtendedState.LANDED_STATE_ON_GROUND and self.phase in ('PRIMING_ARM','WAIT_OFFBOARD','WAIT_ARM','ARMED','READY'):
                    if time.monotonic()-getattr(self,'ai_cancel_attempt',0.)>=.5:
                        self.ai_cancel_attempt=time.monotonic();self.disarm(None)
                    self.setpoint_pub.publish(self.arming_setpoint(self.hold_pose))
                elif self.fresh_pose():self.hold(None)
                else:self.start_landing('AI执行连接失效且位置不可用')
                return
            if not self.state.connected:
                return
            if not self.state.armed and self.phase in ('DISCONNECTED', 'LOCALIZING', 'READY'):
                self.set_phase('READY' if self.fresh_pose() else 'LOCALIZING')
            if (self.airborne() and self.phase not in ('LANDING','DESCENDING','FAILSAFE') and
                    self.odom is not None and self.current_pose().position.z > self.max_flight_height+.05):
                self.start_landing('Virtual ceiling exceeded; landing')
            if self.phase == 'PRIMING_ARM':
                if not self.authorized or not self.fresh_pose():
                    self.set_phase('READY')
                    self.error('Arming cancelled because localization is unavailable')
                    return
                self.setpoint_pub.publish(self.arming_setpoint(self.hold_pose))
                if (now - self.prime_start).to_sec() >= 1.0:
                    try:
                        reply = self.mode_client(custom_mode='OFFBOARD')
                        if reply.mode_sent:
                            self.set_phase('WAIT_OFFBOARD')
                        else:
                            self.error('PX4 rejected OFFBOARD')
                            self.set_phase('READY')
                    except rospy.ServiceException as exc:
                        self.error('OFFBOARD service failed: ' + str(exc))
                        self.set_phase('READY')
                return
            if self.phase == 'WAIT_OFFBOARD':
                self.setpoint_pub.publish(self.arming_setpoint(self.hold_pose))
                if not self.authorized or not self.fresh_pose():
                    self.set_phase('READY')
                    self.error('Arming cancelled because localization is unavailable')
                elif self.state.mode == 'OFFBOARD':
                    try:
                        response = self.arm_client(True)
                        if response.success:
                            # The service ACK can precede /mavros/state by a
                            # heartbeat period. Confirm actual ARM before the
                            # UI enables takeoff or reports the ARMED phase.
                            self.arm_start = now
                            self.set_phase('WAIT_ARM')
                        else:
                            self.error('PX4 arming denied, result code %s' % response.result)
                            self.set_phase('READY')
                    except rospy.ServiceException as exc:
                        self.error('Arming service failed: ' + str(exc))
                        self.set_phase('READY')
                elif (now - self.prime_start).to_sec() > 4.0:
                    self.error('PX4 did not enter OFFBOARD within 4 seconds')
                    self.set_phase('READY')
                return
            if self.phase == 'WAIT_ARM':
                self.setpoint_pub.publish(self.arming_setpoint(self.hold_pose))
                if self.state.armed:
                    self.set_phase('ARMED')
                elif (now - self.arm_start).to_sec() > 4.0:
                    self.set_phase('READY')
                    self.error('PX4 ARM acknowledgement was not confirmed by state')
                return
            if self.phase == 'ARMED':
                if self.state.mode != 'OFFBOARD' or not self.fresh_pose():
                    self.error('OFFBOARD or localization lost before takeoff')
                    self.set_phase('READY')
                else:
                    self.setpoint_pub.publish(self.arming_setpoint(self.hold_pose))
                return
            if self.phase == 'TAKEOFF':
                if self.state.mode != 'OFFBOARD':
                    self.start_landing('PX4 left OFFBOARD during takeoff')
                    return
                import copy
                commanded = copy.deepcopy(self.takeoff_pose)
                commanded.position.z = min(self.takeoff_pose.position.z,
                    self.takeoff_start_z + self.takeoff_speed * (now - self.takeoff_start).to_sec())
                self.setpoint_pub.publish(self.setpoint(commanded))
                if (commanded.position.z == self.takeoff_pose.position.z and
                    abs(self.current_pose().position.z - self.takeoff_pose.position.z) < 0.10 and
                    abs(self.odom.twist.twist.linear.z) < 0.15):
                    self.hold_pose = copy.deepcopy(self.takeoff_pose)
                    self.set_phase('HOLD')
                return
            if self.phase == 'NAVIGATING':
                if self.ai_navigation_id==self.route_sequence and (self.ai_task_canceled or self.ai_request_id!=self.ai_navigation_request or not self.ai_valid(self.ai_navigation_session)):
                    self.stop_navigation('AI control authorization/connection lost; navigation cancelled')
                    return
                if self.inspection_task and (now-self.inspection_heartbeat).to_sec()>1.:
                    self.stop_navigation('Inspection manager heartbeat became stale; navigation cancelled')
                    return
                if self.state.mode != 'OFFBOARD':
                    self.start_landing('PX4 left OFFBOARD during navigation')
                    return
                if not self.fresh_map() or not self.fresh_free_map():
                    self.stop_navigation('Obstacle/free map became stale; navigation cancelled')
                    return
                if self.active_goal:
                    g = self.active_goal.pose.position
                    values = [g.x, g.y, g.z]
                    if any(c in self.occupied_cells for c in self.segment_cells(values, values)):
                        if self.navigation_map_mode=='MAPPING':
                            self.stop_navigation('跳过当前目标：实时地图确认其位于障碍内；继续后续队列',preserve_queue=True)
                            if self.phase=='HOLD':
                                self.queue_dispatch_after=now+rospy.Duration(.30)
                                self.publish_goal_queue()
                        else:
                            self.stop_navigation('Goal is now inside a known inflated obstacle; navigation cancelled')
                        return
                if self.tick_goal_arrival(now):
                    return
                self.activate_pending_spline()
                if self.phase != 'NAVIGATING':
                    return
                self.update_terminal_approach()
                if self.navigation_stage == 'PLANNING':
                    self.setpoint_pub.publish(self.setpoint(self.hold_pose))
                    if time.monotonic()-self.planner_ack_wall>50.:
                        self.stop_navigation('New planning session ACK timed out')
                    return
                if self.navigation_stage == 'SPACE_WAIT':
                    self.setpoint_pub.publish(self.setpoint(self.hold_pose))
                    v = self.odom.twist.twist.linear
                    speed = math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
                    if ((now-self.space_wait_started).to_sec() >= 2.0 and speed <= .1 and
                            self.map_time > self.space_wait_started and
                            self.free_time > self.space_wait_started and self.lio_quality == 'HEALTHY'):
                        import copy
                        goal = copy.deepcopy(self.active_goal)
                        # The task identity stays unchanged during retries.
                        self.request_planner(goal)
                        rospy.loginfo('Fresh observations received; preparing retained navigation goal')
                    return
                if self.navigation_stage == 'TURNING':
                    if not self.fresh_map():
                        self.stop_navigation('Obstacle map became stale during heading alignment')
                    else:
                        self.tick_heading_alignment(now)
                    return
                if (self.future_command_since is not None and
                        (now-self.future_command_since).to_nsec()/1e9 >= self.max_traj_age):
                    self.stop_navigation('EGO command clock mismatch persisted; navigation cancelled')
                    return
                planner_age=(now-self.planner_time).to_nsec()/1e9
                startup_age=(now-self.goal_start).to_nsec()/1e9
                if (startup_age>self.max_planner_age and
                        (self.planner_time<self.goal_start or not 0<=planner_age<=self.max_planner_age)):
                    self.stop_navigation('EGO planner heartbeat became stale; navigation cancelled')
                    return
                if not self.fresh_map():
                    self.stop_navigation('Obstacle map became stale; navigation cancelled')
                    return
                if self.tick_terminal_endpoint(now):
                    return
                if self.approved_curve is not None:
                    curve = self.approved_curve
                    duration = curve.knots[len(curve.pos_pts)]-curve.knots[3]
                    if (now-curve.start_time).to_sec() > duration+.5:
                        self.wait_for_space('Local curve completed before final goal; requesting another local plan')
                        return
                if self.trajectory and (now - self.trajectory_time).to_sec() <= self.max_traj_age:
                    reason = self.command_error(self.trajectory)
                    if reason:
                        self.navigation_command_failed(reason)
                        return
                    heading = self.motion_heading(self.trajectory)
                    actual_yaw = self.pose_yaw(self.current_pose())
                    if heading is not None and (
                            not self.navigation_heading_ready or
                            abs(self.angle_delta(heading, self.navigation_heading)) > self.heading_half_cone or
                            abs(self.angle_delta(heading, actual_yaw)) > self.heading_half_cone):
                        self.begin_heading_alignment(heading)
                        self.setpoint_pub.publish(self.setpoint(self.hold_pose))
                        return
                    if heading is None and not self.navigation_heading_ready:
                        # No horizontal motion: vertical travel keeps its yaw.
                        self.navigation_heading = actual_yaw
                        p = self.current_pose().position
                        g = self.active_goal.pose.position if self.active_goal else p
                        if math.hypot(g.x-p.x, g.y-p.y) > 0.03:
                            self.setpoint_pub.publish(self.setpoint(self.hold_pose))
                            return
                    if not self.navigation_heading_ready:
                        self.navigation_heading_ready = True
                        # Pure vertical travel also requires the observed-free
                        # check before any moving command is published.
                        reason = self.command_error(self.trajectory)
                        if reason:
                            self.navigation_command_failed(reason)
                            return
                    if heading is not None:
                        self.follow_navigation_heading(heading, now)
                    else:
                        self.yaw_last_tick = now
                    pose = self.copy_pose()
                    pose.position = self.trajectory.position
                    self.set_pose_yaw(pose, self.navigation_heading)
                    target = self.setpoint(pose)
                    # EGO's desired velocity is a controller feedforward, not
                    # an external velocity observation for the EKF. Position-
                    # only tracking lags ~v/Kp and can cut an obstacle corner.
                    target.type_mask &= ~(PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                                          PositionTarget.IGNORE_VZ)
                    target.velocity.x = self.trajectory.velocity.x
                    target.velocity.y = self.trajectory.velocity.y
                    target.velocity.z = self.trajectory.velocity.z
                    self.navigation_wait_reason=('终点接近：沿减速EGO曲线执行' if self.terminal_stage == 'APPROACH' else '沿审批路线执行')
                    self.setpoint_pub.publish(target)
                    self.publish_execution_curve(self.approved_curve)
                elif self.trajectory is not None:
                    self.stop_navigation('EGO-Planner trajectory stream became stale')
                elif (now - self.goal_start).to_sec() > 5.0:
                    # Without an approved route there is no route tangent;
                    # keep holding/replanning instead of turning to the goal.
                    self.wait_for_space('No collision-free local trajectory available yet')
                else:
                    self.setpoint_pub.publish(self.setpoint(self.hold_pose))
                return
            if self.phase == 'ROTATING':
                if self.state.mode != 'OFFBOARD':
                    self.start_landing('PX4 left OFFBOARD during rotation')
                    return
                elapsed = max(0.0, (now-self.rotate_start).to_sec())
                angle = min(2*math.pi, elapsed*self.rotate_rate)
                q = tf.transformations.quaternion_from_euler(0, 0, self.rotate_yaw_start+angle)
                self.hold_pose.orientation.x, self.hold_pose.orientation.y, self.hold_pose.orientation.z, self.hold_pose.orientation.w = q
                self.setpoint_pub.publish(self.setpoint(self.hold_pose))
                if angle >= 2*math.pi:
                    self.set_phase('HOLD')
                return
            if self.phase == 'HOLD':
                if self.state.mode != 'OFFBOARD':
                    self.start_landing('PX4 left OFFBOARD during hold')
                    return
                self.setpoint_pub.publish(self.setpoint(self.hold_pose))
                self.dispatch_queued_goal(now)
            if self.phase == 'DESCENDING':
                if not self.fresh_pose():
                    self.set_phase('FAILSAFE')
                    self.error('Position lost during fallback descent')
                    return
                if self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                    if not self.state.armed:
                        self.set_phase('READY')
                    return
                descent_dt=max(0.0,min(.1,(event.current_real-event.last_real).to_nsec()/1e9))
                self.hold_pose.position.z -= self.descent_rate * descent_dt
                self.setpoint_pub.publish(self.setpoint(self.hold_pose))
            if self.phase == 'LANDING' and not self.state.armed and self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                self.set_phase('READY')


if __name__ == '__main__':
    # Register the hint before init_node establishes its automatic /clock
    # connection. Changing a hint afterwards does not renegotiate an already
    # connected TCPROS socket. This callback-free subscriber does not set time;
    # rospy's built-in simulation clock callback retains sole ownership.
    clock_transport = rospy.Subscriber('/clock', Clock, queue_size=1, tcp_nodelay=True)
    rospy.init_node('drone_flight_manager')
    FlightManager()
    rospy.spin()
