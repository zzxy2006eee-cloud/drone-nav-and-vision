#!/usr/bin/env python3
"""Audit recorded ROS timestamps and OFFBOARD setpoint gaps without a ROS master."""
import argparse
import json
from pathlib import Path

import numpy as np
import rosbag


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('output')
    args = parser.parse_args()
    watched = ['/mavros/state', '/drone/flight_state', '/drone/flight_error',
               '/mavros/setpoint_raw/local', '/drone/lio/odom',
               '/mavros/local_position/odom', '/drone/cloud_fcu_world',
               '/grid_map/occupancy_inflate', '/clock']
    stats = {}
    transitions = []
    errors = []
    offboard = False
    phase = None
    previous_setpoint = None
    gaps = []
    with rosbag.Bag(args.bag) as bag:
        available = bag.get_type_and_topic_info()[1]
        for topic, message, received in bag.read_messages(topics=watched):
            t = received.to_sec()
            if topic == '/mavros/state':
                offboard = message.armed and message.mode == 'OFFBOARD'
                if not offboard:
                    previous_setpoint = None
            elif topic == '/drone/flight_state':
                phase = message.data
                transitions.append({'sim_time_s': t, 'phase': phase})
            elif topic == '/drone/flight_error':
                errors.append({'sim_time_s': t, 'error': message.data})
            if topic == '/mavros/setpoint_raw/local' and offboard:
                if previous_setpoint is not None:
                    gaps.append({'sim_time_s': t, 'gap_s': t-previous_setpoint, 'phase': phase})
                previous_setpoint = t
            if hasattr(message, 'header'):
                entry = stats.setdefault(topic, {'count': 0, 'zero_stamps': 0,
                                                'frames': set(), 'ages': []})
                entry['count'] += 1
                entry['frames'].add(message.header.frame_id)
                if message.header.stamp.to_nsec() == 0:
                    entry['zero_stamps'] += 1
                else:
                    entry['ages'].append(t-message.header.stamp.to_sec())
        missing = [topic for topic in watched if topic not in available]
    for entry in stats.values():
        ages = entry.pop('ages')
        entry['frames'] = sorted(entry['frames'])
        if ages:
            entry['receipt_minus_header_s'] = {
                'min': float(min(ages)), 'p50': float(np.percentile(ages, 50)),
                'p95': float(np.percentile(ages, 95)), 'max': float(max(ages))}
    result = {'source_bag': str(Path(args.bag).resolve()),
              'kind': 'offline_audit_of_existing_experiment',
              'topics': stats, 'missing_topics': missing,
              'phase_transitions': transitions, 'recorded_errors': errors,
              'armed_offboard_setpoint_gaps': {
                  'samples': len(gaps),
                  'largest': sorted(gaps, key=lambda gap: gap['gap_s'], reverse=True)[:10]},
              'limits': ['No new flight performed.',
                         'Missing map topics prevent historical map timestamp measurement.',
                         'Recorded receipt gaps do not prove PX4 received each setpoint.']}
    Path(args.output).write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({'output': args.output, 'missing_topics': missing,
                      'max_recorded_offboard_setpoint_gap_s': max(
                          (gap['gap_s'] for gap in gaps), default=None)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
