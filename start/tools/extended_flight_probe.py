#!/usr/bin/env python3
"""Execute recorded scenario tests through the production flight services."""
import argparse
import copy
import json
import math
import os
import sys
import threading
import time
from pathlib import Path
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from std_msgs.msg import Bool
from rosgraph_msgs.msg import Clock,Log
from std_srvs.srv import Trigger,SetBool
from drone_stack.srv import LocalGoal

FAULT_NODES={'kill_planner':'/ego_planner_node','kill_traj':'/traj_server',
             'kill_lio':'/laserMapping','kill_bridge':'/drone_lio_bridge',
             'kill_manager':'/drone_flight_manager','kill_mavros':'/mavros'}
INPUT_FAULTS=['pose_jump','timestamp_regression','clock_reset']
GEOMETRY_FAULTS=['geometry_loss']
COMM_FAULTS=['mavlink_drop']
GUI_CASES=['gui_camera_loss']
ADDITIONAL_CASES=['high_lio_loss','gui_land']


def px4_mode(custom_mode):
    """PX4 v1.15 custom-mode fields, independent of legacy base-mode flags."""
    main=(int(custom_mode)>>16)&255
    sub=(int(custom_mode)>>24)&255
    if main==6:
        return 'OFFBOARD'
    if main==4:
        return {1:'READY',2:'TAKEOFF',3:'LOITER',4:'MISSION',5:'RTL',6:'LAND',
                8:'FOLLOW_TARGET',9:'PRECLAND'}.get(sub,'AUTO_UNKNOWN')
    return {1:'MANUAL',2:'ALTCTL',3:'POSCTL',5:'ACRO',7:'STABILIZED',10:'TERMINATION'}.get(main,'UNKNOWN')


class NativeMonitor:
    """Read a separate PX4 link; never infer FCU state from stale ROS caches."""
    def __init__(self):
        sys.path.insert(0,str(Path(__file__).parent/'python_deps'))
        from pymavlink import mavutil
        self.mavutil=mavutil
        self.link=mavutil.mavlink_connection('udpin:127.0.0.1:14550',source_system=254)
        self.last=None
        self.alive=True
        self.records=[]
        self.worker=threading.Thread(target=self.run,daemon=True)
        self.worker.start()

    def run(self):
        while self.alive:
            message=self.link.recv_match(type='HEARTBEAT',blocking=True,timeout=.5)
            if message is None or message.get_srcSystem()!=1 or message.get_srcComponent()!=1:
                continue
            entry={'wall_monotonic':time.monotonic(),'sim_t':rospy.Time.now().to_sec(),'mode':px4_mode(message.custom_mode),
                   'custom_mode':int(message.custom_mode),'base_mode':int(message.base_mode),
                   'armed':bool(message.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}
            self.last=entry
            self.records.append(entry)

    def fresh(self):
        # SITL heartbeats follow simulation time. Rendering can slow or pause
        # the world; retain a separate wall deadline to detect a frozen link.
        return (self.last is not None and
                0<=rospy.Time.now().to_sec()-self.last['sim_t']<2 and
                time.monotonic()-self.last['wall_monotonic']<10)

    def close(self):
        self.alive=False
        self.worker.join(timeout=2)
        self.link.close()
        directory=os.environ.get('DRONE_EXPERIMENT_DIR')
        if directory:
            (Path(directory)/'native_heartbeats.json').write_text(json.dumps(self.records,indent=2)+'\n')

    def emergency_land(self):
        self.link.mav.command_long_send(1,1,self.mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,
            self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,4,6,0,0,0,0)


class Probe:
    def __init__(self):
        self.data = {}
        self.error_sequence = 0
        self.bridge_logs = []
        self.observation_loss = None
        self.phase_changes = []
        self.hover_prefix = None
        self.subscribers = []
        self.subscribers.append(rospy.Subscriber('/rosout',Log,
            lambda m:self.bridge_logs.append(m.msg) if m.name=='/drone_lio_bridge' else None,queue_size=100))
        for topic, typ, key in [('/drone/flight_state', String, 'phase'),
                               ('/drone/flight_error', String, 'error'),
                               ('/mavros/state', State, 'state'),
                               ('/mavros/local_position/odom', Odometry, 'odom'),
                               ('/drone/lio/odom', Odometry, 'lio'),
                               ('/Odometry', Odometry, 'raw_lio'),
                               ('/drone/lio/valid', Bool, 'lio_valid'),
                               ('/gazebo/model_states', ModelStates, 'truth')]:
            self.subscribers.append(rospy.Subscriber(topic, typ,
                lambda m, k=key: self.receive(k,m), queue_size=1, tcp_nodelay=True))
        self.wait(lambda: all(k in self.data for k in ['phase','state','odom','lio','truth']), 15)
        self.wait(lambda: self.phase == 'HOLD', 10)

    def receive(self,key,message):
        self.data[key]=message
        if key=='error':self.error_sequence+=1
        if key=='phase':self.phase_changes.append((rospy.Time.now().to_sec(),message.data))
        if key=='raw_lio' and self.observation_loss is None and np.trace(np.asarray(message.pose.covariance).reshape(6,6)[:3,:3])>1e5:
            self.observation_loss={'stamp':message.header.stamp.to_sec(),
                'receipt':rospy.Time.now().to_sec(),
                'truth':self.position('truth').tolist() if 'truth' in self.data else None}

    @property
    def phase(self):
        return self.data['phase'].data

    def wait(self, predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            time.sleep(.05)
        raise RuntimeError('Timed out; phase=' + str(self.data.get('phase')) +
                           ', error=' + str(self.data.get('error')))

    def position(self, key='odom'):
        if key == 'truth':
            m = self.data[key]
            p = m.pose[m.name.index('inspection_quad')].position
        else:
            p = self.data[key].pose.pose.position
        return np.array([p.x,p.y,p.z])

    def goal(self, values, frame='odom'):
        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = map(float, values)
        goal.pose.orientation.w = 1
        reply = rospy.ServiceProxy('/drone/local_goal', LocalGoal)(goal)
        return {'position': list(map(float, values)), 'accepted': reply.success,
                'message': reply.message, 'frame': frame}

    def hover(self, seconds):
        start_t = self.position('truth')
        start_l = self.position('lio')
        poses = []
        errors = []
        self.hover_prefix = None
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.phase != 'HOLD' or self.data['state'].mode != 'OFFBOARD':
                raise RuntimeError('Hover lost HOLD/OFFBOARD: '+self.phase)
            truth = self.position('truth')
            poses.append(truth)
            errors.append(np.linalg.norm((self.position('lio')-start_l)-(truth-start_t)))
            self.hover_prefix={'truth_range_xyz_m':np.ptp(np.array(poses),axis=0).tolist(),
                               'max_lio_displacement_error_m':float(max(errors))}
            time.sleep(.1)
        spread = np.ptp(np.array(poses), axis=0)
        result = {'seconds_wall':seconds, 'truth_range_xyz_m':spread.tolist(),
                  'max_lio_displacement_error_m':float(max(errors))}
        result['passed'] = bool(spread[0]<.20 and spread[1]<.20 and spread[2]<.15 and max(errors)<.10)
        if not result['passed']:
            raise RuntimeError('Hover out of bounds: '+str(result))
        return result

    def navigate(self, target):
        error_sequence=self.error_sequence
        request = self.goal(target)
        if not request['accepted']:
            raise RuntimeError('Goal rejected: '+str(request))
        # The service response and phase topic use different connections.
        # Await the requested transition before interpreting the previous
        # HOLD message as a cancellation of this new goal.
        self.wait(lambda:self.phase=='NAVIGATING',3)
        begin = time.monotonic()
        deadline = begin + 90
        trace = []
        while time.monotonic() < deadline:
            error = float(np.linalg.norm(self.position()-target))
            trace.append({'sim_t':rospy.Time.now().to_sec(), 'truth':self.position('truth').tolist(),
                          'odom':self.position().tolist(), 'phase':self.phase, 'goal_error_m':error})
            if self.phase == 'HOLD':
                if error >= .20:
                    raise RuntimeError('Navigation cancelled away from goal: '+str(self.data.get('error')))
                # HOLD also means an execution protection stopped the flight.
                # Allow the separate error connection to deliver its reason;
                # proximity alone cannot certify successful goal completion.
                time.sleep(.25)
                if self.phase!='HOLD' or self.data['state'].mode!='OFFBOARD':
                    raise RuntimeError('Unexpected flight state at arrival: '+self.phase)
                if self.error_sequence>error_sequence and self.data['error'].data:
                    raise RuntimeError('Navigation protected before arrival: '+self.data['error'].data)
                return {'request':request,'passed':True,'goal_error_m':error,
                        'elapsed_wall_s':time.monotonic()-begin,'trace':trace}
            if self.phase != 'NAVIGATING' or self.data['state'].mode != 'OFFBOARD':
                raise RuntimeError('Unexpected flight state: '+self.phase)
            time.sleep(.1)
        raise RuntimeError('Goal did not finish within 90 wall seconds')

    def blocked_goal(self, origin):
        start=self.position('truth')
        reply=self.goal(origin+np.array([3.2,0,0]))
        if reply['accepted']:
            # As for navigate(), the service and phase topic are separate
            # connections. Do not certify the HOLD from before this request.
            self.wait(lambda:self.phase=='NAVIGATING',3)
            self.wait(lambda:self.phase in ('HOLD','LANDING','DESCENDING','FAILSAFE'),35)
        elif reply.get('message') not in ('Goal is in unobserved space', 'Goal is inside an inflated obstacle voxel'):
            raise RuntimeError('Blocked test rejected for unrelated readiness failure: '+str(reply))
        if self.position('truth')[0]>=2.30:
            raise RuntimeError('Blocked goal did not stop safely before wall')
        if self.phase in ('LANDING','DESCENDING','FAILSAFE'):
            return self.observability_landing(reply,start)
        if self.phase!='HOLD':
            raise RuntimeError('Blocked goal did not stop safely')
        try:
            hover=self.hover(20)
        except RuntimeError:
            if self.observation_loss is None:
                raise
            prefix=self.hover_prefix
            if prefix is not None and (any(v>=limit for v,limit in zip(prefix['truth_range_xyz_m'],[.20,.20,.15])) or
                                       prefix['max_lio_displacement_error_m']>=.10):
                raise RuntimeError('Hover exceeded limits before observability protection: '+str(prefix))
            return self.observability_landing(reply,start)
        error=self.data.get('error')
        if reply['accepted'] and (error is None or not any(reason in error.data for reason in
                ('intersects an inflated obstacle voxel','enters unobserved space','outside configured flight volume'))):
            raise RuntimeError('Blocked test stopped for unrelated failure: '+str(error))
        return {'request':reply,'final_truth':self.position('truth').tolist(),
                'start_truth':start.tolist(),'hover':hover,
                'protection_reason':error.data if error is not None else None}

    def observability_landing(self, reply, initial_truth):
        loss=self.observation_loss
        if loss is None or loss['truth'] is None:
            raise RuntimeError('Landing without verified native LIO observability loss')
        self.wait(lambda:any('position observations are degenerate' in s for s in self.bridge_logs),3)
        events=[t for t,phase in self.phase_changes if t>=loss['stamp'] and phase in ('LANDING','DESCENDING','FAILSAFE')]
        if not events:raise RuntimeError('No protective landing transition after observability loss')
        monitor=NativeMonitor()
        try:
            self.wait(lambda:monitor.fresh(),5)
            landing=self.protected_landing(monitor,np.array(loss['truth']),loss['stamp'],
                {'case':'blocked_observability_loss','native_covariance_rejected':True},
                initial_reaction=events[0]-loss['stamp'])
            if self.data['lio_valid'].data:raise RuntimeError('Degenerate bridge resumed before restart')
            return {'request':reply,'start_truth':initial_truth.tolist(),'observability_loss':loss,
                    'observability_landing':landing,'hover_before_loss':self.hover_prefix,
                    'bridge_fault_logs':list(self.bridge_logs),'bridge_remained_invalid_after_landing':True}
        finally:
            monitor.close()

    def protected_landing(self, monitor, start, begin, injection, limit=1.5, initial_reaction=None):
        trace=[];reaction=initial_reaction;deadline=time.monotonic()+55
        while time.monotonic()<deadline:
            if not monitor.fresh():
                raise RuntimeError('Independent PX4 heartbeat missing')
            trace.append({'sim_t':rospy.Time.now().to_sec(),'truth':self.position('truth').tolist(),
                          'phase':self.phase,'native_mode':monitor.last['mode'],
                          'native_armed':monitor.last['armed']})
            if self.phase in ('LANDING','DESCENDING','FAILSAFE') and reaction is None:
                reaction=rospy.Time.now().to_sec()-begin
            if not monitor.last['armed']:
                break
            time.sleep(.05)
        positions=np.array([start.tolist()]+[r['truth'] for r in trace])
        rise=float(max(0,positions[:,2].max()-start[2]))
        drift=float(np.linalg.norm(positions[:,:2]-start[:2],axis=1).max())
        if (reaction is None or not 0<=reaction<=limit or monitor.last['armed'] or
                self.position('truth')[2]>.25 or rise>.15 or drift>.30):
            raise RuntimeError('Protection outside limits: '+str((reaction,rise,drift,monitor.last)))
        return {'passed':True,'injection':injection,'reaction_sim_s':reaction,
                'max_rise_m':rise,'max_horizontal_drift_m':drift,'trace':trace,
                'native_heartbeats':monitor.records}

    def geometry_fault(self):
        monitor=NativeMonitor()
        try:
            self.wait(lambda:monitor.fresh() and monitor.last['armed'] and monitor.last['mode']=='OFFBOARD',8)
            self.wait(lambda:'lio_valid' in self.data and self.data['lio_valid'].data,5)
            start=self.position('truth');begin=rospy.Time.now().to_sec()
            reply=rospy.ServiceProxy('/drone/sim/faults/lidar_geometry',SetBool)(True)
            if not reply.success:raise RuntimeError('Geometry fault injection failed')
            self.wait(lambda:self.observation_loss is not None and not self.data['lio_valid'].data,3)
            self.wait(lambda:any('position observations are degenerate' in s for s in self.bridge_logs),2)
            result=self.protected_landing(monitor,start,begin,
                {'case':'geometry_loss','fault_start_sim_s':begin,'streams_preserved':True})
            result['observability_loss']=self.observation_loss
            result['bridge_fault_logs']=list(self.bridge_logs)
            result['bridge_remained_invalid_after_landing']=not self.data['lio_valid'].data
            if not result['bridge_remained_invalid_after_landing']:
                raise RuntimeError('Degenerate bridge automatically resumed')
            return result
        finally:
            monitor.close()

    def input_fault(self, case):
        monitor=NativeMonitor()
        try:
            self.wait(lambda:monitor.fresh() and monitor.last['armed'] and monitor.last['mode']=='OFFBOARD',8)
            self.wait(lambda:'raw_lio' in self.data and 'lio_valid' in self.data,5)
            publisher=rospy.Publisher('/clock' if case=='clock_reset' else '/Odometry',
                Clock if case=='clock_reset' else Odometry,queue_size=1,tcp_nodelay=True)
            self.wait(lambda:publisher.get_num_connections()>0,5)
            start=self.position('truth');begin=rospy.Time.now().to_sec()
            message=copy.deepcopy(self.data['raw_lio'])
            original=message.header.stamp.to_sec()
            if case=='pose_jump':
                message.header.stamp+=rospy.Duration(.001)
                message.pose.pose.position.x+=2.0
            elif case=='timestamp_regression':
                message.header.stamp-=rospy.Duration(.10)
            else:
                message=Clock(clock=rospy.Time.from_sec(begin-1.0))
            publisher.publish(message)
            # All clock subscriptions share Gazebo's concurrent high-rate
            # channel; retry a bounded burst until invalidity is observed.
            if case=='clock_reset':
                for _ in range(10):
                    if not self.data['lio_valid'].data:
                        break
                    time.sleep(.02)
                    publisher.publish(message)
            self.wait(lambda:not self.data['lio_valid'].data,3)
            expected={'pose_jump':'Rejected LIO pose jump',
                      'timestamp_regression':'LIO acquisition time moved backwards',
                      'clock_reset':'ROS clock moved backwards'}[case]
            self.wait(lambda:any(expected in text for text in self.bridge_logs),2)
            result=self.protected_landing(monitor,start,begin,
                {'case':case,'original_lio_stamp':original,'fault_start_sim_s':begin})
            result['bridge_fault_logs']=list(self.bridge_logs)
            result['bridge_remained_invalid_after_landing']=not self.data['lio_valid'].data
            if not result['bridge_remained_invalid_after_landing']:
                raise RuntimeError('Faulted bridge automatically resumed')
            publisher.unregister()
            return result
        finally:
            monitor.close()

    def communication_fault(self):
        monitor=NativeMonitor()
        try:
            self.wait(lambda:monitor.fresh() and monitor.last['armed'] and monitor.last['mode']=='OFFBOARD',8)
            start=self.position('truth');begin=rospy.Time.now().to_sec()
            service=rospy.ServiceProxy('/drone/sim/faults/mavlink_drop',SetBool)
            reply=service(True)
            if not reply.success:raise RuntimeError('Packet drop failed')
            try:
                self.wait(lambda:monitor.fresh() and monitor.last['mode']=='LAND',8)
                observed=rospy.Time.now().to_sec()-begin
            finally:
                service(False)
            result=self.protected_landing(monitor,start,begin,
                {'case':'mavlink_drop','both_directions':True,'restore_after_native_land':True},limit=3)
            result['native_land_observed_sim_s']=observed
            result['limits']='Temporary bidirectional link interruption; restoration only after native LAND.'
            return result
        except Exception:
            if monitor.fresh() and monitor.last['armed']:
                monitor.emergency_land()
            raise
        finally:
            monitor.close()

    def camera_fault(self):
        import rosnode
        stopped,failed=rosnode.kill_nodes(['/sim_camera_processor'])
        if failed or '/sim_camera_processor' not in stopped:
            raise RuntimeError('Could not stop camera processor')
        return {'passed':True,'stopped_node':'/sim_camera_processor','hover':self.hover(20),
                'limits':'Processed camera stream loss; image sensor hardware dropout is a separate test.'}

    def gui_landing(self, output):
        import subprocess
        monitor=NativeMonitor()
        try:
            self.wait(lambda:monitor.fresh() and monitor.last['armed'] and monitor.last['mode']=='OFFBOARD',8)
            start=self.position('truth')
            snapshot=Path(output).with_name('gui_land_accessibility.json')
            subprocess.run(['gnome-screenshot','-f',str(Path(output).with_name('gui_before_land.png'))],check=True,timeout=10)
            begin=rospy.Time.now().to_sec()
            # Exercise the actual Qt accessible action; no direct ROS landing service.
            subprocess.run([sys.executable,str(Path(__file__).with_name('gui_snapshot.py')),
                            str(snapshot),'--click-land'],check=True,timeout=15)
            evidence=json.loads(snapshot.read_text())
            if not evidence.get('landing_action',{}).get('activated'):
                raise RuntimeError('Actual GUI landing action was not activated')
            result=self.protected_landing(monitor,start,begin,'actual_Qt_landing_button',limit=4.0)
            result['gui_evidence']=str(snapshot)
            return result
        finally:
            monitor.close()

    def node_fault(self, case):
        import rosnode
        monitor=NativeMonitor()
        trace=[]
        try:
            self.wait(lambda:monitor.fresh() and monitor.last['armed'] and monitor.last['mode']=='OFFBOARD',8)
            if case in ('kill_planner','kill_traj'):
                reply=self.goal(self.position()+np.array([.9,.4,0]))
                if not reply['accepted']:
                    raise RuntimeError('Fault-test navigation rejected: '+str(reply))
                self.wait(lambda:self.phase=='NAVIGATING',3)
                time.sleep(1)
            start=self.position('truth')
            begin=rospy.Time.now().to_sec()
            stopped,failed=rosnode.kill_nodes([FAULT_NODES[case]])
            if failed or FAULT_NODES[case] not in stopped:
                raise RuntimeError('Node shutdown request failed: '+str(failed))
            deadline=time.monotonic()+55
            reaction=None
            while time.monotonic()<deadline:
                if not monitor.fresh():
                    raise RuntimeError('Independent PX4 heartbeat missing')
                trace.append({'sim_t':rospy.Time.now().to_sec(),'truth':self.position('truth').tolist(),
                              'phase':self.phase,'native_mode':monitor.last['mode'],
                              'native_armed':monitor.last['armed']})
                protected=(self.phase=='HOLD' if case in ('kill_planner','kill_traj') else
                           (monitor.last['mode']=='LAND' if case in ('kill_manager','kill_mavros') else
                            self.phase in ('LANDING','DESCENDING','FAILSAFE')))
                if protected and reaction is None:
                    reaction=rospy.Time.now().to_sec()-begin
                if case in ('kill_planner','kill_traj') and protected:
                    break
                if not monitor.last['armed']:
                    break
                time.sleep(.05)
            positions=np.array([start.tolist()]+[r['truth'] for r in trace])
            rise=float(max(0,positions[:,2].max()-start[2]))
            drift=float(np.linalg.norm(positions[:,:2]-start[:2],axis=1).max())
            limit=2.5 if case=='kill_planner' else (3.0 if case in ('kill_manager','kill_mavros') else 1.5)
            if reaction is None or reaction>limit:
                raise RuntimeError('Fault response late or missing: '+str(reaction))
            if case in ('kill_planner','kill_traj'):
                if self.phase!='HOLD' or monitor.last['mode']!='OFFBOARD':
                    raise RuntimeError('Planner loss did not maintain OFFBOARD HOLD')
                hold=self.hover(20)
            else:
                if monitor.last['armed'] or self.position('truth')[2]>.25 or rise>.15 or drift>.30:
                    raise RuntimeError('Node loss landing outside limits: '+str((rise,drift,monitor.last)))
                hold=None
            return {'passed':True,'stopped_node':FAULT_NODES[case],'reaction_sim_s':reaction,
                    'max_rise_m':rise,'max_horizontal_drift_m':drift,'trace':trace,
                    'native_heartbeats':monitor.records,'hover':hold,
                    'reaction_measurement':'heartbeat_observation_plus_required_native_ULog_audit' if case in ('kill_manager','kill_mavros') else 'manager_phase',
                    'limits':'Permanent node shutdown; independent PX4 heartbeat and Gazebo truth.'}
        except Exception:
            if monitor.fresh() and monitor.last['armed']:
                monitor.emergency_land()
                # Cleanup cannot turn the failed protection test into a pass.
                deadline=time.monotonic()+55
                while time.monotonic()<deadline and monitor.last['armed']:
                    time.sleep(.1)
            raise
        finally:
            monitor.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case',choices=['multi_goal','corridor','three_d','blocked','boundaries','stress']+list(FAULT_NODES)+INPUT_FAULTS+GEOMETRY_FAULTS+COMM_FAULTS+GUI_CASES+ADDITIONAL_CASES)
    parser.add_argument('output')
    args=parser.parse_args()
    rospy.init_node('extended_flight_probe',anonymous=True)
    result={'case':args.case,'passed':False,'steps':[]}
    try:
        p=Probe()
        origin=p.position()
        if args.case=='high_lio_loss':
            target=origin.copy();target[2]=2.45
            step=p.navigate(target);result['steps'].append(step)
            step['hover']=p.hover(20)
            result['steps'].append(p.node_fault('kill_lio'))
            offsets=[]
        elif args.case=='gui_land':
            result['steps'].append(p.gui_landing(args.output))
            offsets=[]
        elif args.case in GUI_CASES:
            result['steps'].append(p.camera_fault())
            offsets=[]
        elif args.case in COMM_FAULTS:
            result['steps'].append(p.communication_fault())
            offsets=[]
        elif args.case in GEOMETRY_FAULTS:
            result['steps'].append(p.geometry_fault())
            offsets=[]
        elif args.case in INPUT_FAULTS:
            result['steps'].append(p.input_fault(args.case))
            offsets=[]
        elif args.case in FAULT_NODES:
            result['steps'].append(p.node_fault(args.case))
            offsets=[]
        elif args.case=='multi_goal':
            offsets=[[.7,0,0],[.4,.6,.5],[0,0,.9],[-.5,.3,-.2],[0,0,0]]
        elif args.case=='corridor':
            offsets=[[3.2,0,0],[0,0,0]]
        elif args.case=='three_d':
            # Pass over the 1 m barrier with clearance, then descend before
            # crossing the beam whose underside is 2.1 m above the floor.
            offsets=[[.5,0,.45],[2.7,0,.45],[3.7,0,0],[0,0,0]]
        elif args.case=='stress':
            # Ten out-and-return missions and a final ten-minute hover.
            offsets=([[3.2,0,0],[0,0,0]])*10
        elif args.case=='boundaries':
            rejected=[]
            for values,frame in [([8.001,0,1],'odom'),([-8.001,0,1],'odom'),
                                 ([0,8.001,1],'odom'),([0,-8.001,1],'odom'),
                                 ([0,0,.499],'odom'),([0,0,2.501],'odom'),
                                 ([math.nan,0,1],'odom'),([0,0,1],'map')]:
                reply=p.goal(values,frame)
                if reply['accepted']:
                    raise RuntimeError('Illegal goal accepted: '+str(reply))
                rejected.append(reply)
            result['steps'].append({'rejected':rejected,'hover':p.hover(20)})
            offsets=[]
        else:
            result['steps'].append(p.blocked_goal(origin))
            offsets=[]
        for offset in offsets:
            step=p.navigate(origin+np.array(offset))
            result['steps'].append(step)
            step['hover']=p.hover(20)
        if args.case=='stress':
            result['steps'].append({'long_hover':p.hover(600)})
        result['passed']=True
    except Exception as exc:
        result['error']=str(exc)
    finally:
        Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='steps'},indent=2),flush=True)
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':
    main()
