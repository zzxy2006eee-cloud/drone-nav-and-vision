#!/usr/bin/env python3
"""Verify native PX4 OFFBOARD-loss latency from its own logged clock."""
import argparse
import json
from pathlib import Path
import numpy as np
from pyulog import ULog


def audit(path):
    ulog=ULog(str(path),message_name_filter_list=['vehicle_status','offboard_control_mode'])
    records={d.name:d.data for d in ulog.data_list}
    if not all(k in records for k in ['vehicle_status','offboard_control_mode']):
        return {'passed':False,'reason':'Required native PX4 topics missing'}
    status=records['vehicle_status'];control=records['offboard_control_mode']
    offboard=np.flatnonzero(status['nav_state']==14)
    landing=np.flatnonzero((status['nav_state']==18)&np.asarray(status['failsafe'],dtype=bool))
    if not len(offboard) or not len(landing):
        return {'passed':False,'reason':'No OFFBOARD followed by failsafe AUTO_LAND'}
    first=int(landing[0])
    stamp=int(status.get('nav_state_timestamp',status['timestamp'])[first])
    prior=control['timestamp'][control['timestamp']<stamp]
    if not len(prior):
        return {'passed':False,'reason':'No logged control input preceding landing'}
    latest=int(prior[-1]);latency=(stamp-latest)/1e6
    disarmed=bool(np.any((status['timestamp']>stamp)&(status['arming_state']==1)))
    had_offboard=bool(np.any(status['timestamp'][offboard]<stamp))
    return {'source_ulog':str(Path(path).resolve()),'passed':had_offboard and disarmed and 0<latency<=1.5,
            'last_logged_offboard_control_sim_s':latest/1e6,'failsafe_auto_land_sim_s':stamp/1e6,
            'response_from_last_logged_control_s':latency,'response_limit_s':1.5,
            'native_disarming_after_landing':disarmed,
            'limits':'Uses PX4 clock. Decimated control logging makes the measured response conservative; '
                     'does not certify ground contact or horizontal drift.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulog');parser.add_argument('output');args=parser.parse_args()
    result=audit(args.ulog)
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
