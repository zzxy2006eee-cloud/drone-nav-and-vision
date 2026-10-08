#!/usr/bin/env python3
"""Continue extended acceptance only after an identical core version passes."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('core_directory');p.add_argument('extended_directory');a=p.parse_args()
    root=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(root/'src/drone_stack/scripts'))
    from run_core_acceptance import fingerprint
    report=Path(a.core_directory)/'core_acceptance_result.json'
    deadline=time.monotonic()+5400
    while time.monotonic()<deadline:
        result=json.loads(report.read_text())
        if result['configuration_sha256']!=fingerprint(root):
            raise SystemExit('Production version changed while awaiting core acceptance')
        if result['status']=='core_passed_remaining_acceptance_pending' and result['passed']:
            break
        if result['status'] not in ('running','not_started'):
            raise SystemExit('Core did not pass: '+result['status'])
        time.sleep(10)
    else:
        raise SystemExit('Core acceptance did not finish within the recorded wait deadline')
    cases=['multi_goal','corridor','three_d','blocked','boundaries','kill_planner','kill_traj',
           'kill_lio','kill_bridge','kill_manager','kill_mavros','mavlink_drop','pose_jump',
           'timestamp_regression','clock_reset','geometry_loss','gui_camera_loss','stress',
           'high_lio_loss','gui_land']
    print('Core passed; starting '+str(len(cases))+' extended flight cases',flush=True)
    code=subprocess.call([sys.executable,str(Path(__file__).with_name('run_extended_scenarios.py')),
                          a.extended_directory,'--cases']+cases,cwd=root,env=os.environ)
    raise SystemExit(code)


if __name__=='__main__':main()
