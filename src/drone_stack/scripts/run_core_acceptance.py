#!/usr/bin/env python3
"""Run the first acceptance phase: three navigation starts and five fault cases.

This phase alone does not certify the remaining scenario/UI/stress checklist.
Local transport failures are recorded separately from actual flight failures.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time


def fingerprint(root):
    paths=[]
    for directory in ['config','launch','scripts','models','worlds','patches']:
        paths.extend(p for p in (root/'src/drone_stack'/directory).rglob('*')
                     if p.is_file() and '__pycache__' not in p.parts)
    paths.extend((root/'src/ego_planner/src/planner').rglob('*.cpp'))
    paths.extend((root/'src/ego_planner/src/planner').rglob('*.h'))
    paths.extend(root/path for path in ['devel/lib/libplan_env.so',
        'devel/lib/libbspline_opt.so','devel/lib/libpath_searching.so',
        'devel/lib/libtraj_utils.so','devel/lib/ego_planner/traj_server',
        'devel/lib/ego_planner/ego_planner_node'])
    for pattern in ['*.cc','*.cpp','*.h','*.hpp']:
        paths.extend((root/'src/faster_lio_main/src').rglob(pattern))
        paths.extend((root/'src/faster_lio_main/include').rglob(pattern))
    paths.extend([root/'src/faster_lio_main/CMakeLists.txt',root/'src/faster_lio_main/package.xml',
                  root/'devel/lib/faster_lio/run_mapping_online',root/'devel/lib/libfaster_lio.so',
                  root/'external/PX4-Autopilot/build/px4_sitl_inspection/bin/px4',
                  root/'external/PX4-Autopilot/build/px4_sitl_inspection/build_gazebo-classic/libgazebo_imu_plugin.so'])
    paths.extend((root/'src/drone_stack/src').rglob('*.cpp'))
    paths.extend([root/'src/drone_stack/package.xml',root/'src/drone_stack/CMakeLists.txt',
                  root/'devel/lib/drone_stack/drone_operator_gui'])
    digest=hashlib.sha256()
    for path in sorted(set(paths)):
        digest.update(str(path.relative_to(root)).encode());digest.update(path.read_bytes())
    return digest.hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory');parser.add_argument('--resume',action='store_true');args=parser.parse_args()
    root=Path(__file__).resolve().parents[3];directory=Path(args.directory).resolve()
    directory.mkdir(parents=True,exist_ok=True);summary_path=directory/'core_acceptance_result.json'
    version=fingerprint(root)
    cases=[('three_cold_navigation',['--rounds','3','--hover-seconds','60','--navigation'])]
    cases += [(fault,['--rounds','1','--hover-seconds','60','--fault-case',fault]) for fault in
              ['fault_lidar','fault_imu','auth_revoked','offboard_loss','low_battery']]
    summary={'configuration_sha256':version,'status':'not_started','passed':False,'real_hardware_ready':False,
             'cases':[],'remaining_acceptance':['multiple_altitudes_and_consecutive_goals',
             'corridor_3d_blocked_and_boundaries','node_and_communication_loss','pose_jump_and_time_reset',
             'live_GUI_fault_states','long_duration_stress']}
    if summary_path.exists():
        if not args.resume:raise SystemExit('Existing report: use a new directory, or --resume with identical configuration.')
        previous=json.loads(summary_path.read_text())
        if previous['configuration_sha256']!=version:raise SystemExit('Configuration changed; preserve the old run and use a new directory.')
        summary=previous
    def save():
        summary['updated_at']=time.strftime('%Y-%m-%dT%H:%M:%S%z')
        summary_path.write_text(json.dumps(summary,indent=2)+'\n')
    try:
        for kind in [socket.SOCK_STREAM,socket.SOCK_DGRAM]:
            with socket.socket(socket.AF_INET,kind) as transport:transport.bind(('127.0.0.1',0))
    except OSError as exc:
        summary.update(status='blocked_environment',transport_error=str(exc));save()
        print('Local ROS/PX4 transport unavailable:',exc,flush=True);raise SystemExit(2)
    summary['status']='running';summary.pop('transport_error',None);save()
    for name,options in cases:
        if any(c['name']==name and c['passed'] for c in summary['cases']):continue
        attempt=len([c for c in summary['cases'] if c['name']==name])+1
        output=directory/(name+'_attempt_%02d'%attempt)
        while output.exists():
            attempt+=1;output=directory/(name+'_attempt_%02d'%attempt)
        env=dict(os.environ,ROS_HOME='/tmp/drone_ros_home',ROS_HOSTNAME='127.0.0.1',
                 ROS_MASTER_URI='http://127.0.0.1:11311');env.pop('ROS_IP',None)
        command=[sys.executable,str(Path(__file__).with_name('run_cold_start_tests.py')),str(output)]+options
        code=subprocess.call(command,cwd=root,env=env)
        summary['cases'].append({'name':name,'passed':code==0,'directory':str(output),'exit_code':code})
        if code:
            summary['status']='failed';save();raise SystemExit(1)
        save()
    summary['passed']=all(any(c['name']==name and c['passed'] for c in summary['cases']) for name,_ in cases)
    summary['status']='core_passed_remaining_acceptance_pending';save()
    print('Core phase passed. Scenario, node-loss, GUI and stress acceptance remain.',flush=True)


if __name__=='__main__':main()
