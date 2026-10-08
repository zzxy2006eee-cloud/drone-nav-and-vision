#!/usr/bin/env python3
"""Own clean SITL restarts and raw telemetry for repeatable regressions."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import rosgraph

parser=argparse.ArgumentParser()
parser.add_argument('directory');parser.add_argument('--rounds',type=int,default=3)
parser.add_argument('--hover-seconds',type=float,default=60)
parser.add_argument('--navigation',action='store_true')
parser.add_argument('--fault-case',choices=['fault_lidar','fault_imu','auth_revoked','offboard_loss','low_battery'])
args=parser.parse_args()
root=Path(__file__).resolve().parents[3]
base=Path(args.directory).resolve()
base.mkdir(parents=True,exist_ok=True)
if rosgraph.is_master_online():raise SystemExit('Stop the existing simulation launcher before cold-start testing.')
topics=['/drone/lio/odom','/mavros/local_position/odom','/gazebo/model_states',
        '/drone/sim/lidar/imu_raw','/mavros/imu/data_raw','/mavros/setpoint_raw/local',
        '/mavros/state','/drone/flight_state','/drone/flight_error','/drone/lio/valid',
        '/planning/pos_cmd','/drone/front/detections','/drone/down/detections',
        '/clock','/drone/cloud_fcu_world','/grid_map/occupancy_inflate','/grid_map/occupancy_inflate_safety',
        '/faster_lio/translation_observability','/Odometry','/planning/bspline','/drone/planning_enabled','/move_base_simple/goal',
        '/mavros/estimator_status','/mavros/timesync_status','/mavros/odometry/out',
        '/drone/manager_heartbeat','/mavros/extended_state']
results=[]
ulog_root=root/'external/PX4-Autopilot/build/px4_sitl_inspection/rootfs/log'
for index in range(1,args.rounds+1):
    directory=base/('round_%02d'%index);directory.mkdir(exist_ok=False)
    env=dict(os.environ,DRONE_LOG_DIR=str(directory/'stack_logs'))
    sim=None;bag=None
    previous_ulogs=set(ulog_root.rglob('*.ulg'))
    with (directory/'launcher.log').open('w') as out:
        try:
            sim=subprocess.Popen([str(root/'src/drone_stack/scripts/start_simulation.sh')],cwd=root,env=env,
                                 stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+30
            while not rosgraph.is_master_online():
                if sim.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Simulator startup failed')
                time.sleep(.2)
            with (directory/'rosbag.log').open('w') as bag_out:
                bag=subprocess.Popen(['rosbag','record','--lz4','-O',str(directory/'telemetry.bag')]+topics,
                                     env=env,stdout=bag_out,stderr=subprocess.STDOUT)
                command=[sys.executable,str(Path(__file__).with_name('run_sim_regression.py')),str(directory),
                         '--hover-seconds',str(args.hover_seconds)]
                if args.navigation:command.append('--navigation')
                if args.fault_case:command.extend(['--fault-case',args.fault_case])
                code=subprocess.call(command,env=env)
            results.append({'round':index,'passed':code==0,'directory':str(directory)})
        except Exception as exc:
            results.append({'round':index,'passed':False,'error':str(exc),'directory':str(directory)})
        finally:
            if bag and bag.poll() is None:
                bag.send_signal(signal.SIGINT)
                try:bag.wait(timeout=15)
                except subprocess.TimeoutExpired:bag.terminate();bag.wait(timeout=5)
            if sim and sim.poll() is None:
                sim.terminate()
                try:sim.wait(timeout=25)
                except subprocess.TimeoutExpired:sim.kill();sim.wait()
    new_ulogs=sorted(set(ulog_root.rglob('*.ulg'))-previous_ulogs,key=lambda path:path.stat().st_mtime)
    if new_ulogs:
        ulog=directory/'flight.ulg';shutil.copy2(new_ulogs[-1],ulog)
        command=[sys.executable,str(Path(__file__).with_name('audit_px4_ulog.py')),str(ulog),str(directory/'fusion_audit.json')]
        if args.fault_case in ('fault_lidar','fault_imu'):command.append('--allow-external-pose-loss')
        with (directory/'fusion_audit.log').open('w') as audit_log:
            fusion_code=subprocess.call(command,stdout=audit_log,stderr=subprocess.STDOUT)
        results[-1]['fusion_audit_passed']=fusion_code==0
        results[-1]['passed']=results[-1]['passed'] and fusion_code==0
        if args.fault_case=='offboard_loss':
            with (directory/'offboard_native_audit.log').open('w') as audit_log:
                offboard_code=subprocess.call([sys.executable,str(Path(__file__).with_name('audit_offboard_loss.py')),
                    str(ulog),str(directory/'offboard_native_audit.json')],stdout=audit_log,stderr=subprocess.STDOUT)
            results[-1]['offboard_native_audit_passed']=offboard_code==0
            results[-1]['passed']=results[-1]['passed'] and offboard_code==0
    elif results[-1]['passed']:
        results[-1]['passed']=False;results[-1]['error']='No new PX4 flight log; cannot verify fusion sources'
    print(json.dumps(results[-1]),flush=True)
    (base/'cold_start_result.json').write_text(json.dumps({'rounds':results,'passed':
        len(results)==args.rounds and all(r['passed'] for r in results)},indent=2))
    if not results[-1]['passed']:break
    time.sleep(2)
sys.exit(0 if len(results)==args.rounds and all(r['passed'] for r in results) else 1)
