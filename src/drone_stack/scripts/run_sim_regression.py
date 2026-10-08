#!/usr/bin/env python3
"""Run against SITL; save failures and always attempt verified landing."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

def run_check(check, directory, env, timeout=360):
    start=time.monotonic()
    log=directory/(check[0]+'_'+time.strftime('%Y%m%d_%H%M%S')+'.log')
    result={'check':check,'log':str(log)}
    script=Path(__file__).with_name('validate_simulation.py')
    with log.open('w') as out:
        try:
            process=subprocess.run([sys.executable,str(script)]+check,env=env,
                                   stdout=out,stderr=subprocess.STDOUT,timeout=timeout)
            result['exit_code']=process.returncode
        except subprocess.TimeoutExpired as exc:
            result.update(exit_code=124,error=str(exc))
        except OSError as exc:
            result.update(exit_code=125,error=str(exc))
    result['elapsed_wall_s']=time.monotonic()-start
    print(json.dumps(result),flush=True)
    return result


def run_suite(checks, directory, env):
    results=[]
    try:
        for check in checks:
            result=run_check(check,directory,env);results.append(result)
            if result['exit_code']:break
    finally:
        # Connect for up to 120 s and descend for up to 55 s. Keep cleanup
        # failures in suite_result.json instead of losing the whole report.
        results.append(run_check(['land'],directory,env,timeout=200))
        summary={'passed':len(results)==len(checks)+1 and all(r['exit_code']==0 for r in results),
                 'checks':results}
        (directory/'suite_result.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory');parser.add_argument('--hover-seconds',type=float,default=90)
    parser.add_argument('--navigation',action='store_true')
    parser.add_argument('--fault-case',choices=['fault_lidar','fault_imu','auth_revoked','offboard_loss','low_battery'])
    args=parser.parse_args()
    directory=Path(args.directory).resolve();directory.mkdir(parents=True,exist_ok=True)
    env=dict(os.environ,DRONE_EXPERIMENT_DIR=str(directory))
    checks=[['ground_safety'],['takeoff'],['cameras'],['airborne_safety'],['hold_check',str(args.hover_seconds)]]
    if args.navigation:checks += [['dry'],['flight'],['hold_check','60']]
    if args.fault_case:checks += [[args.fault_case]]
    summary=run_suite(checks,directory,env)
    print(json.dumps(summary),flush=True)
    raise SystemExit(0 if summary['passed'] else 1)


if __name__=='__main__':main()
