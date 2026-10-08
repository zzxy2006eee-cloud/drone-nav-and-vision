#!/usr/bin/env python3
import json, time, sys, threading, math, os
from pathlib import Path
import numpy as np
import rospy
from scipy.interpolate import BSpline
from scipy.spatial import cKDTree
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs import point_cloud2
from ego_planner.msg import Bspline
from drone_stack.srv import Takeoff, LocalGoal

rospy.init_node('drone_validation', anonymous=True)
d = {}; trajectories=[]; fault_trace=None
def save(k):
    def callback(msg):
        d[k]=msg
        trace=fault_trace
        if k=='truth' and trace is not None and 'inspection_quad' in msg.name:
            p=msg.pose[msg.name.index('inspection_quad')].position
            trace.append({'t':rospy.Time.now().to_sec(),'truth':[p.x,p.y,p.z],
                          'phase':d['phase'].data if 'phase' in d else None,
                          'mode':d['state'].mode if 'state' in d else None})
    return callback
for topic, typ, key in [('/mavros/local_position/odom',Odometry,'odom'),('/drone/lio/odom',Odometry,'lio'),('/gazebo/model_states',ModelStates,'truth'),('/mavros/state',State,'state'),('/drone/flight_state',String,'phase'),('/drone/flight_error',String,'error'),('/grid_map/occupancy_inflate_safety',PointCloud2,'map')]:
    rospy.Subscriber(topic,typ,save(key),queue_size=1)
rospy.Subscriber('/planning/bspline',Bspline,lambda msg: trajectories.append(msg),queue_size=20)
def wait(predicate, seconds=30):
    until=time.monotonic()+seconds
    while time.monotonic()<until and not rospy.is_shutdown():
        if predicate(): return
        time.sleep(.05)
    failure={'passed':False,'reason':'timeout','phase':phase_name() if 'phase' in d else None,
             'error':d['error'].data if 'error' in d else None}
    if 'truth' in d:
        failure['truth_model_names']=list(d['truth'].name)
        if 'inspection_quad' in d['truth'].name:
            failure['truth']=truth().tolist()
    if 'odom' in d: failure['position']=current().tolist()
    output(failure)
    if 'state' in d and d['state'].armed and sys.argv[1]!='land':
        rospy.ServiceProxy('/drone/land',Trigger)()
    raise RuntimeError('Timeout waiting; '+str(failure))
def call(name,typ,*args):
    result=rospy.ServiceProxy(name,typ)(*args)
    print(name,result,flush=True)
    if not result.success: raise RuntimeError(str(result))
    return result
def xyz(p): return [p.x,p.y,p.z]
def current(): return np.array(xyz(d['odom'].pose.pose.position))
def truth():
    msg=d['truth']; return np.array(xyz(msg.pose[msg.name.index('inspection_quad')].position))
def phase_name(): return d['phase'].data if 'phase' in d else 'MANAGER_UNAVAILABLE'
def phase(s): return phase_name()==s
def output(result):
    result['recorded_wall_time']=time.strftime('%Y-%m-%dT%H:%M:%S%z')
    result['recorded_sim_time']=rospy.Time.now().to_sec()
    print(json.dumps({k:v for k,v in result.items() if k != 'trace'},indent=2),flush=True)
    with open('/tmp/drone_validation_'+sys.argv[1]+'.json','w') as f: json.dump(result,f,indent=2)
    directory=os.environ.get('DRONE_EXPERIMENT_DIR')
    if directory:
        Path(directory).mkdir(parents=True,exist_ok=True)
        with open(Path(directory)/(sys.argv[1]+'_'+time.strftime('%Y%m%d_%H%M%S')+'.json'),'w') as f:
            json.dump(result,f,indent=2)
mode=sys.argv[1]
required=('odom','truth','state') if mode=='land' else ('odom','lio','truth','state','phase')
wait(lambda: all(k in d for k in required),120)
if mode=='takeoff':
    wait(lambda: phase('READY'),180)
    ground_truth = truth()
    ground_lio = np.array(xyz(d['lio'].pose.pose.position))
    ground_local = current()
    call('/drone/set_authorized',SetBool,True)
    call('/drone/arm',Trigger)
    wait(lambda: phase('ARMED') and d['state'].armed and d['state'].mode == 'OFFBOARD',20)
    time.sleep(3)
    armed_truth = truth()
    if armed_truth[2] - ground_truth[2] > .05:
        call('/drone/land',Trigger)
        raise RuntimeError('Vehicle rose before explicit takeoff: '+str(armed_truth-ground_truth))
    call('/drone/takeoff',Takeoff,1.0)
    wait(lambda: phase('HOLD'),40)
    time.sleep(5)
    lio_delta = np.array(xyz(d['lio'].pose.pose.position)) - ground_lio
    truth_delta = truth() - ground_truth
    output({'phase':phase_name(),'mode':d['state'].mode,'position':current().tolist(),'truth':truth().tolist(),'ground_truth':ground_truth.tolist(),'armed_truth':armed_truth.tolist(),'ground_local':ground_local.tolist(),'lio_delta':lio_delta.tolist(),'truth_delta':truth_delta.tolist(),'lio_truth_delta_error':float(np.linalg.norm(lio_delta-truth_delta))})
    if np.linalg.norm(lio_delta-truth_delta) > .10 or abs(truth_delta[2]-1.0)>.15:
        call('/drone/land',Trigger)
        raise RuntimeError('Takeoff height or LIO displacement failed acceptance')
elif mode=='dry':
    wait(lambda: phase('HOLD') and 'map' in d,20)
    start=current(); goal=PoseStamped(); goal.header.frame_id='odom'; goal.pose.position.x=start[0]+3.2; goal.pose.position.y=start[1]; goal.pose.position.z=start[2]; goal.pose.orientation.w=1
    enable=rospy.Publisher('/drone/planning_enabled',Bool,queue_size=1,latch=True)
    pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
    time.sleep(1); enable.publish(True); time.sleep(.5); goal.header.stamp=rospy.Time.now(); pub.publish(goal)
    wait(lambda: len(trajectories)>0,20); time.sleep(3)
    msg=trajectories[-1]; points=np.array([xyz(p) for p in msg.pos_pts]); knots=np.array(msg.knots); spline=BSpline(knots,points,msg.order)
    samples=spline(np.linspace(knots[msg.order],knots[-msg.order-1],1500))
    cloud=d['map']; occupied=np.array(list(point_cloud2.read_points(cloud,field_names=('x','y','z'),skip_nans=True)))
    resolution=rospy.get_param('/ego_planner_node/grid_map/resolution'); origin=np.array([-15.,-15.,rospy.get_param('/ego_planner_node/grid_map/ground_height')])
    cells={tuple(v) for v in np.floor((occupied-origin)/resolution).astype(int)}
    collisions=sum(tuple(v) in cells for v in np.floor((samples-origin)/resolution).astype(int))
    result={'start':start.tolist(),'goal':xyz(goal.pose.position),'trajectory_id':msg.traj_id,'duration':float(knots[-msg.order-1]-knots[msg.order]),'sample_min':samples.min(axis=0).tolist(),'sample_max':samples.max(axis=0).tolist(),'occupied_count':len(occupied),'collision_samples':collisions,'min_voxel_distance':float(cKDTree(occupied).query(samples)[0].min()),'phase':phase_name()}
    output(result)
    np.savez('/tmp/drone_dry_trajectory.npz',samples=samples,occupied=occupied)
    call('/drone/hold',Trigger)
    bounds_min=np.array(rospy.get_param('/drone_flight_manager/goal_min_xyz',[-8.,-8.,.5]))
    bounds_max=np.array(rospy.get_param('/drone_flight_manager/goal_max_xyz',[8.,8.,2.5]))
    if collisions or np.any(samples.min(axis=0)<bounds_min) or np.any(samples.max(axis=0)>bounds_max):
        raise RuntimeError('Dry trajectory violates occupied voxels or map floor; do not execute')
elif mode=='flight':
    wait(lambda: phase('HOLD'),20)
    start=current(); goal=PoseStamped(); goal.header.frame_id='odom'; goal.pose.position.x=start[0]+3.2; goal.pose.position.y=start[1]; goal.pose.position.z=start[2]; goal.pose.orientation.w=1
    call('/drone/local_goal',LocalGoal,goal)
    until=time.monotonic()+85; minimum=100.; trace=[]; reached=False; abort=''
    while time.monotonic()<until:
        p=current(); g=truth(); radius=float(np.linalg.norm(g[:2]-[3,1])); minimum=min(minimum,radius)
        trace.append({'t':rospy.Time.now().to_sec(),'local':p.tolist(),'truth':g.tolist(),'radius':radius,'phase':phase_name(),'mode':d['state'].mode})
        if radius<.65: abort='pillar clearance below 0.65m'; break
        if phase_name() in ('LANDING','DESCENDING','FAILSAFE','READY'): abort='unexpected flight phase '+phase_name(); break
        if np.linalg.norm(p-np.array(xyz(goal.pose.position)))<.2 and phase('HOLD'): reached=True; break
        if phase('HOLD') and rospy.Time.now().to_sec()-trace[0]['t']>1.0:
            abort='Navigation cancelled: '+(d['error'].data if 'error' in d else 'no error message'); break
        time.sleep(.1)
    if phase('HOLD') or phase('NAVIGATING'): call('/drone/hold',Trigger)
    output({'reached':reached,'abort':abort,'min_truth_pillar_radius':minimum,'start':start.tolist(),'goal':xyz(goal.pose.position),'final_local':current().tolist(),'final_truth':truth().tolist(),'trace':trace})
    if not reached or abort:
        raise RuntimeError('Flight did not pass: '+(abort or 'goal timeout'))
elif mode=='hold_check':
    wait(lambda: phase('HOLD'),20)
    duration=float(sys.argv[2]) if len(sys.argv)>2 else 60.0
    records=[]; until=time.monotonic()+duration
    while time.monotonic()<until:
        records.append({'local':current().tolist(),'lio':xyz(d['lio'].pose.pose.position),'truth':truth().tolist(),'mode':d['state'].mode,'phase':phase_name(),'lio_age':(rospy.Time.now()-d['lio'].header.stamp).to_sec(),'odom_age':(rospy.Time.now()-d['odom'].header.stamp).to_sec()})
        time.sleep(.1)
    lio=np.array([r['lio'] for r in records]); ground=np.array([r['truth'] for r in records]); errors=(lio-lio[0])-(ground-ground[0])
    result={'duration_wall_s':duration,'samples':len(records),'modes':sorted({r['mode'] for r in records}),'phases':sorted({r['phase'] for r in records}),'local_peak_to_peak':np.ptp([r['local'] for r in records],axis=0).tolist(),'lio_peak_to_peak':np.ptp(lio,axis=0).tolist(),'truth_peak_to_peak':np.ptp(ground,axis=0).tolist(),'max_lio_truth_displacement_error':float(np.linalg.norm(errors,axis=1).max()),'max_lio_age':max(r['lio_age'] for r in records),'max_odom_age':max(r['odom_age'] for r in records),'trace':records}
    output(result)
    if (result['max_lio_truth_displacement_error']>.10 or result['truth_peak_to_peak'][2]>.15 or
        max(result['truth_peak_to_peak'][:2])>.20 or
        result['modes']!=['OFFBOARD'] or result['phases']!=['HOLD']):
        call('/drone/land',Trigger)
        raise RuntimeError('Hover stability or LIO consistency failed')
elif mode=='land':
    if not d['state'].connected:
        raise RuntimeError('Cannot verify disarming through disconnected MAVROS')
    if not d['state'].armed:
        grounded=truth()[2]<.25
        output({'armed':False,'skipped':True,'passed':bool(grounded),'truth':truth().tolist()})
        if not grounded:raise RuntimeError('Unarmed vehicle is not safely grounded')
        sys.exit(0)
    start=truth();records=[];until=time.monotonic()+55
    try:
        call('/drone/land',Trigger)
    except (rospy.ServiceException, RuntimeError):
        from mavros_msgs.srv import SetMode
        reply=rospy.ServiceProxy('/mavros/set_mode',SetMode)(custom_mode='AUTO.LAND')
        if not reply.mode_sent:raise RuntimeError('Manager unavailable and PX4 rejected AUTO.LAND')
        print('Manager unavailable; direct PX4 AUTO.LAND requested',flush=True)
    while d['state'].armed and time.monotonic()<until:
        records.append({'t':rospy.Time.now().to_sec(),'truth':truth().tolist(),
                        'local':current().tolist(),'phase':phase_name(),'mode':d['state'].mode})
        time.sleep(.1)
    positions=np.array([r['truth'] for r in records]+[truth().tolist()])
    rise=float(max(0,positions[:,2].max()-start[2]))
    drift=float(np.linalg.norm(positions[:,:2]-start[:2],axis=1).max())
    passed=d['state'].connected and not d['state'].armed and truth()[2]<.25 and rise<=.15 and drift<=.30
    output({'armed':d['state'].armed,'phase':phase_name(),'truth':truth().tolist(),
            'max_rise_m':rise,'max_horizontal_drift_m':drift,'passed':passed,'trace':records})
    if not passed:raise RuntimeError('Landing path failed acceptance')
elif mode=='cameras':
    import cv2, tf
    from cv_bridge import CvBridge
    listener=tf.TransformListener(); time.sleep(1)
    results={}; directory=Path(os.environ.get('DRONE_EXPERIMENT_DIR','/tmp'))
    directory.mkdir(parents=True,exist_ok=True)
    for name in ['front','down']:
        topic='/drone/'+name
        img=rospy.wait_for_message(topic+'/image_processed',Image,timeout=5)
        info=rospy.wait_for_message(topic+'/camera_info',CameraInfo,timeout=5)
        detections=json.loads(rospy.wait_for_message(topic+'/detections',String,timeout=5).data)
        listener.waitForTransform('base_link',img.header.frame_id,rospy.Time(0),rospy.Duration(3))
        pos,q=listener.lookupTransform('base_link',img.header.frame_id,rospy.Time(0))
        axis=tf.transformations.quaternion_matrix(q)[:3,:3].dot([0,0,1])
        wanted=np.array([1,0,0] if name=='front' else [0,0,-1])
        valid=(img.header.frame_id==info.header.frame_id and img.encoding=='bgr8' and
               img.width==640 and img.height==480 and np.linalg.norm(axis-wanted)<1e-5)
        cv2.imwrite(str(directory/(name+'_processed.png')),CvBridge().imgmsg_to_cv2(img,'bgr8'))
        results[name]={'frame':img.header.frame_id,'optical_forward_in_body':axis.tolist(),
                       'detections':detections['detections'],'passed':bool(valid)}
    output({'cameras':results,'passed':all(x['passed'] for x in results.values())})
    if not all(x['passed'] for x in results.values()):raise RuntimeError('Camera frame/image pipeline failed')
elif mode in ('ground_safety','airborne_safety'):
    airborne=mode=='airborne_safety'
    wait(lambda: phase('HOLD') if airborne else not d['state'].armed,20)
    if not airborne:call('/drone/set_authorized',SetBool,False)
    checks=[]
    requests=[('/drone/takeoff',Takeoff,(float('nan'),)),('/drone/takeoff',Takeoff,(-1.,)),
              ('/drone/takeoff',Takeoff,(3.,))]
    requests.append(('/drone/disarm' if airborne else '/drone/land',Trigger,()))
    if not airborne:requests.append(('/drone/arm',Trigger,()))
    for name,typ,args in requests:
        response=rospy.ServiceProxy(name,typ)(*args)
        checks.append({'service':name,'accepted':response.success,'message':response.message})
    for values,frame in [([20,0,1],'odom'),([0,0,.1],'odom'),([0,0,1],'map'),([float('nan'),0,1],'odom')]:
        goal=PoseStamped();goal.header.frame_id=frame;goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=values
        response=rospy.ServiceProxy('/drone/local_goal',LocalGoal)(goal)
        checks.append({'service':'local_goal','goal':values,'frame':frame,'accepted':response.success,'message':response.message})
    passed=all(not r['accepted'] for r in checks)
    output({'passed':passed,'checks':checks,'phase':phase_name()})
    if not passed:
        if d['state'].armed:rospy.ServiceProxy('/drone/land',Trigger)()
        raise RuntimeError('An illegal operation was accepted')
elif mode in ('fault_lidar','fault_imu','auth_revoked','offboard_loss','low_battery'):
    from mavros_msgs.srv import SetMode,ParamSet
    from mavros_msgs.msg import ParamValue
    wait(lambda:phase('HOLD'),20)
    start=truth();start_t=rospy.Time.now().to_sec();fault=None;fault_trace=[]
    if mode.startswith('fault_'):
        fault='/drone/sim/faults/'+mode.split('_')[1]
        call(fault,SetBool,True)
    elif mode=='auth_revoked':call('/drone/set_authorized',SetBool,False)
    elif mode=='offboard_loss':
        # Stop the sole setpoint owner, while keeping LIO and MAVROS alive.
        # This exercises actual PX4 OFFBOARD stream loss. LOITER requires
        # global position and POSCTL requires manual input, both absent here.
        import rosnode
        stopped,failed=rosnode.kill_nodes(['/drone_flight_manager'])
        if failed or '/drone_flight_manager' not in stopped:
            raise RuntimeError('Could not stop the setpoint owner: '+str(failed))
    else:
        for name,value in [('SIM_BAT_MIN_PCT',0.0),('SIM_BAT_DRAIN',1.0)]:
            reply=rospy.ServiceProxy('/mavros/param/set',ParamSet)(param_id=name,value=ParamValue(real=value))
            print('Battery injection',name,reply,flush=True)
            if not reply.success or abs(reply.value.real-value)>1e-6:
                raise RuntimeError('Battery injection parameter readback failed: '+name)
    try:
        if mode=='offboard_loss':
            # The manager is absent: its cached phase cannot confirm recovery.
            wait(lambda:d['state'].mode in ('AUTO.LAND','AUTO.DESCEND'),8)
        else:
            wait(lambda:phase('LANDING') or phase('DESCENDING') or phase('FAILSAFE'),30 if mode=='low_battery' else 8)
        reaction=rospy.Time.now().to_sec()-start_t
    finally:
        if fault:call(fault,SetBool,False)
        if mode=='low_battery':
            for name,value in [('SIM_BAT_MIN_PCT',50.0),('SIM_BAT_DRAIN',60.0)]:
                rospy.ServiceProxy('/mavros/param/set',ParamSet)(param_id=name,value=ParamValue(real=value))
    deadline=time.monotonic()+55
    while d['state'].armed and time.monotonic()<deadline:
        time.sleep(.1)
    trace=list(fault_trace);fault_trace=None
    positions=np.array([r['truth'] for r in trace]+[truth().tolist()])
    rise=float(max(0,positions[:,2].max()-start[2]));drift=float(np.linalg.norm(positions[:,:2]-start[:2],axis=1).max())
    # A 1 Hz heartbeat can report the native mode transition up to one second
    # late. For stream loss the cold-start runner additionally REQUIRES a PX4
    # ULog audit with the original 1.5 s native response limit.
    reaction_limit=15 if mode=='low_battery' else (3.0 if mode=='offboard_loss' else 1.5)
    passed=not d['state'].armed and reaction<=reaction_limit and rise<=.15 and drift<=.3
    output({'passed':passed,'reaction_sim_s':reaction,'max_rise_m':rise,'max_horizontal_drift_m':drift,
            'armed':d['state'].armed,'final_phase':'MANAGER_STOPPED' if mode=='offboard_loss' else phase_name(),'trace':trace,
            'fault_start_sim_s':start_t,'reaction_measurement':'ROS_mode_observation_requires_native_ULog_audit' if mode=='offboard_loss' else 'manager_phase',
            'fault_injection': 'stop_setpoint_owner_actual_OFFBOARD_loss' if mode=='offboard_loss' else mode})
    if not passed:raise RuntimeError('Fault response or landing failed acceptance')
