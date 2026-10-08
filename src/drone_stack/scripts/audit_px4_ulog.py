#!/usr/bin/env python3
"""Read PX4's actual EKF fusion flags from a flight log, without ROS networking."""
import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def audit(path, allow_pose_loss=False):
    log = ULog(str(path), message_name_filter_list=['estimator_status_flags'])
    records = [record for record in log.data_list if record.name == 'estimator_status_flags']
    if len(records) != 1:
        return {'passed': False, 'reason': 'Expected exactly one EKF instance in the simulation'}
    flags = records[0].data
    required = ['cs_ev_pos', 'cs_ev_hgt', 'cs_ev_yaw']
    forbidden = ['cs_ev_vel', 'cs_gps', 'cs_gps_hgt', 'cs_gps_yaw',
                 'cs_baro_hgt', 'cs_rng_hgt', 'cs_opt_flow', 'cs_mag']
    missing = [name for name in required+forbidden+['cs_in_air'] if name not in flags]
    if missing:
        return {'passed': False, 'reason': 'Missing required logged flags', 'missing': missing}
    airborne = np.asarray(flags['cs_in_air'], dtype=bool)
    airborne_count = int(airborne.sum())
    required_counts = {name: int(np.count_nonzero(flags[name][airborne])) for name in required}
    forbidden_counts = {name: int(np.count_nonzero(flags[name])) for name in forbidden}
    parameters = {name: log.initial_parameters.get(name) for name in
                  ['EKF2_EV_CTRL', 'EKF2_HGT_REF', 'EKF2_GPS_CTRL', 'EKF2_BARO_CTRL']}
    pose_ok = (all(count > 0 for count in required_counts.values()) if allow_pose_loss else
               all(count == airborne_count for count in required_counts.values()))
    passed = (airborne_count > 0 and pose_ok and
              all(count == 0 for count in forbidden_counts.values()) and
              parameters == {'EKF2_EV_CTRL': 11, 'EKF2_HGT_REF': 3, 'EKF2_GPS_CTRL': 0, 'EKF2_BARO_CTRL': 0})
    return {'source_ulog': str(Path(path).resolve()), 'passed': passed,
            'airborne_flag_samples': airborne_count, 'required_active_airborne_counts': required_counts,
            'forbidden_active_whole_log_counts': forbidden_counts, 'initial_parameters': parameters,
            'allow_pose_loss_for_fault_experiment': allow_pose_loss,
            'limits': 'This is a fusion audit; it does not certify navigation, fault handling, or real hardware.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ulog'); parser.add_argument('output')
    parser.add_argument('--allow-external-pose-loss', action='store_true')
    args = parser.parse_args()
    result = audit(args.ulog, args.allow_external_pose_loss)
    Path(args.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
