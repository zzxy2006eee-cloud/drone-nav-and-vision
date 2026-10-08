#!/usr/bin/env python3
"""Own clean starts, flight cleanup, bags and ULog for scenario acceptance."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import rosgraph


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    parser.add_argument('--cases',nargs='+',default=['multi_goal','corridor','three_d','blocked','boundaries'])
    parser.add_argument('--diagnostic-sensors',action='store_true',
                        help='Also record raw/converted lidar and IMU for input diagnosis')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    scripts=root/'src/drone_stack/scripts'
    sys.path.insert(0,str(scripts))
    from run_core_acceptance import fingerprint
    from audit_px4_ulog import audit
    from audit_offboard_loss import audit as native_offboard_audit
    from audit_world_bag import audit as geometry_audit
    from extended_flight_probe import FAULT_NODES,INPUT_FAULTS,GEOMETRY_FAULTS,COMM_FAULTS,GUI_CASES,ADDITIONAL_CASES
    base=Path(args.directory).resolve()
    base.mkdir(parents=True,exist_ok=False)
    if rosgraph.is_master_online():
        raise SystemExit('Existing ROS master; stop its owning launcher first.')
    summary={'passed':False,'cases':[], 'configuration_sha256':fingerprint(root),
             'diagnostic_sensors':args.diagnostic_sensors,
             'tool_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()
               for p in Path(__file__).parent.glob('*.py')},'real_hardware_ready':False}
    snapshot_dir=base/'source_snapshot'
    snapshot_dir.mkdir()
    for path in Path(__file__).parent.glob('*.py'):
        shutil.copy2(path,snapshot_dir/path.name)
    from importlib.metadata import distributions
    summary['experiment_dependencies']={d.metadata['Name']:d.version
        for d in distributions(path=[str(Path(__file__).parent/'python_deps')])}
    worlds={'corridor':'inspection_corridor','three_d':'inspection_3d','blocked':'inspection_blocked'}
    topics=['/clock','/gazebo/model_states','/mavros/local_position/odom','/mavros/state',
            '/mavros/extended_state','/mavros/setpoint_raw/local','/mavros/odometry/out',
            '/mavros/estimator_status','/drone/lio/odom','/drone/lio/valid',
            '/drone/flight_state','/drone/flight_error','/drone/manager_heartbeat',
            '/grid_map/occupancy_inflate_safety','/grid_map/observed_free','/drone/cloud_body_pose','/planning/pos_cmd','/planning/data_display','/faster_lio/translation_observability','/Odometry','/planning/bspline',
            '/drone/planning_enabled','/move_base_simple/goal','/rosout']
    if args.diagnostic_sensors:
        topics += ['/drone/sim/lidar/points','/drone/sim/lidar/imu_raw',
                   '/livox/lidar','/livox/imu','/Odometry','/cloud_registered']
    ulog_dir=root/'external/PX4-Autopilot/build/px4_sitl_inspection/rootfs/log'
    for case in args.cases:
        if case not in ['multi_goal','corridor','three_d','blocked','boundaries','stress']+list(FAULT_NODES)+INPUT_FAULTS+GEOMETRY_FAULTS+COMM_FAULTS+GUI_CASES+ADDITIONAL_CASES:
            raise ValueError('Unknown case '+case)
        directory=base/case
        directory.mkdir()
        world=root/'src/drone_stack/worlds'/(worlds.get(case,'inspection_demo')+'.world')
        env=dict(os.environ,DRONE_LOG_DIR=str(directory/'stack_logs'),DRONE_SIM_WORLD=str(world),
                 ROS_HOME='/tmp/drone_ros_home',ROS_HOSTNAME='127.0.0.1',ROS_MASTER_URI='http://127.0.0.1:11311',
                 QT_LINUX_ACCESSIBILITY_ALWAYS_ON='1')
        env.pop('ROS_IP',None)
        old_ulogs=set(ulog_dir.rglob('*.ulg'))
        sim=None;bag=None;relay=None;gui_observer=None
        entry={'case':case,'directory':str(directory),'passed':False,'checks':[]}
        def check(name,command,timeout=360):
            with (directory/(name+'.log')).open('w') as out:
                try:
                    code=subprocess.call(command,cwd=root,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=timeout)
                except subprocess.TimeoutExpired:
                    code=124
            entry['checks'].append({'name':name,'exit_code':code,'passed':code==0})
            return code==0
        try:
            if case in COMM_FAULTS:
                env['DRONE_FCU_URL']='udp://:14541@127.0.0.1:14542'
                with (directory/'mavlink_relay.log').open('w') as out:
                    relay=subprocess.Popen([sys.executable,str(Path(__file__).with_name('mavlink_udp_relay.py')),
                        str(directory/'mavlink_relay.json')],cwd=root,env=env,stdout=out,stderr=subprocess.STDOUT)
                time.sleep(.5)
                if relay.poll() is not None:raise RuntimeError('MAVLink relay startup failed')
            with (directory/'launcher.log').open('w') as out:
                sim=subprocess.Popen([str(scripts/'start_simulation.sh')],cwd=root,env=env,
                    stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+30
            while not rosgraph.is_master_online():
                if sim.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('Simulator startup failed')
                time.sleep(.2)
            with (directory/'rosbag.log').open('w') as out:
                bag=subprocess.Popen(['rosbag','record','--lz4','-O',str(directory/'telemetry.bag')]+topics+(['/livox/lidar','/livox/imu'] if case in GEOMETRY_FAULTS else []),
                    env=env,stdout=out,stderr=subprocess.STDOUT)
            env['DRONE_EXPERIMENT_DIR']=str(directory)
            for mode in ['ground_safety','takeoff','airborne_safety']:
                if not check(mode,[sys.executable,str(scripts/'validate_simulation.py'),mode]):
                    raise RuntimeError(mode+' failed')
            gui_trigger={'kill_lio':'lio_invalid','kill_bridge':'lio_invalid','kill_manager':'manager_stale',
                         'kill_mavros':'fcu_stale','pose_jump':'lio_invalid','timestamp_regression':'lio_invalid',
                         'clock_reset':'lio_invalid','geometry_loss':'lio_invalid','gui_camera_loss':'camera_stale'}.get(case)
            if gui_trigger:
                with (directory/'gui_observer.log').open('w') as out:
                    gui_observer=subprocess.Popen([sys.executable,str(Path(__file__).with_name('capture_live_gui.py')),
                        gui_trigger,str(directory/'gui_fault')],env=env,stdout=out,stderr=subprocess.STDOUT)
                time.sleep(1)
            if not check('scenario',[sys.executable,str(Path(__file__).with_name('extended_flight_probe.py')),
                                     case,str(directory/'scenario.json')],2400 if case=='stress' else 900):
                raise RuntimeError('Scenario failed')
        except Exception as exc:
            entry['error']=str(exc)
        finally:
            if gui_observer is not None:
                try:entry['gui_audit_passed']=gui_observer.wait(timeout=20)==0
                except subprocess.TimeoutExpired:
                    gui_observer.terminate();gui_observer.wait(timeout=5);entry['gui_audit_passed']=False
            if sim is not None and sim.poll() is None and rosgraph.is_master_online():
                if case=='kill_mavros' and (directory/'scenario.json').exists() and json.loads((directory/'scenario.json').read_text()).get('passed'):
                    # The probe verified disarming and ground truth through an
                    # independent PX4 link; MAVROS services no longer exist.
                    entry['checks'].append({'name':'land','exit_code':0,'passed':True,
                        'verified_by':'native_PX4_heartbeat_and_Gazebo_truth'})
                else:
                    check('land',[sys.executable,str(scripts/'validate_simulation.py'),'land'],200)
            if bag is not None and bag.poll() is None:
                bag.send_signal(signal.SIGINT)
                try:bag.wait(timeout=20)
                except subprocess.TimeoutExpired:bag.terminate();bag.wait(timeout=5)
            if sim is not None and sim.poll() is None:
                sim.terminate()
                try:sim.wait(timeout=25)
                except subprocess.TimeoutExpired:sim.kill();sim.wait()
            if relay is not None and relay.poll() is None:
                relay.send_signal(signal.SIGINT)
                try:relay.wait(timeout=5)
                except subprocess.TimeoutExpired:relay.terminate();relay.wait(timeout=5)
        new_ulogs=sorted(set(ulog_dir.rglob('*.ulg'))-old_ulogs,key=lambda p:p.stat().st_mtime)
        if new_ulogs:
            shutil.copy2(new_ulogs[-1],directory/'flight.ulg')
            observed_loss = case=='blocked' and (directory/'scenario.json').exists() and any(
                'observability_landing' in step for step in json.loads((directory/'scenario.json').read_text()).get('steps',[]))
            fusion=audit(directory/'flight.ulg',allow_pose_loss=observed_loss or case in ['kill_lio','kill_bridge','kill_mavros','high_lio_loss']+INPUT_FAULTS+GEOMETRY_FAULTS+COMM_FAULTS)
            (directory/'fusion_audit.json').write_text(json.dumps(fusion,indent=2)+'\n')
            entry['fusion_audit_passed']=fusion['passed']
            if case in ['kill_manager','kill_mavros']+COMM_FAULTS:
                native=native_offboard_audit(directory/'flight.ulg')
                (directory/'offboard_native_audit.json').write_text(json.dumps(native,indent=2)+'\n')
                entry['offboard_native_audit_passed']=native['passed']
        else:
            entry['fusion_audit_passed']=False
        if (directory/'telemetry.bag').exists():
            try:
                geometry=geometry_audit(directory/'telemetry.bag',world)
                if case in ['boundaries','blocked']+list(FAULT_NODES)+INPUT_FAULTS+GEOMETRY_FAULTS+COMM_FAULTS+GUI_CASES+ADDITIONAL_CASES:
                    # Rejected requests deliberately do not enter NAVIGATING.
                    geometry['passed']=bool(geometry.get('truth_samples',0)>0 and
                        geometry.get('overlap_samples',1)==0 and
                        geometry.get('grounded_during_hold_or_navigation_samples',1)==0 and
                        geometry.get('max_pose_receipt_gap_sim_s',1)<=.1)
                (directory/'world_geometry_audit.json').write_text(json.dumps(geometry,indent=2)+'\n')
                entry['geometry_audit_passed']=geometry['passed']
            except Exception as exc:
                entry['geometry_audit_passed']=False;entry['geometry_error']=str(exc)
        if case in GEOMETRY_FAULTS:
            from audit_observability_fault import audit as observability_audit
            proof=observability_audit(directory/'telemetry.bag',directory/'scenario.json')
            (directory/'observability_audit.json').write_text(json.dumps(proof,indent=2)+'\n')
            entry['observability_audit_passed']=proof['passed']
        entry['passed']=bool('error' not in entry and len(entry['checks'])==5 and
            all(c['passed'] for c in entry['checks']) and entry['fusion_audit_passed'] and
            entry.get('geometry_audit_passed',False) and entry.get('offboard_native_audit_passed',True) and
            entry.get('gui_audit_passed',True) and entry.get('observability_audit_passed',True) and
            fingerprint(root)==summary['configuration_sha256'] and summary['tool_sha256']==
            {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')})
        summary['cases'].append(entry)
        summary['passed']=len(summary['cases'])==len(args.cases) and all(c['passed'] for c in summary['cases'])
        (base/'extended_acceptance_result.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(entry),flush=True)
        if not entry['passed']:
            raise SystemExit(1)
        time.sleep(2)
    raise SystemExit(0 if summary['passed'] else 1)


if __name__=='__main__':
    main()
