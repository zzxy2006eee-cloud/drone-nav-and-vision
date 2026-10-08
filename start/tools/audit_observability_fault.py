#!/usr/bin/env python3
"""Require sensor, covariance, bridge and landing evidence for geometry loss."""
import json
import math
from pathlib import Path
import rosbag


def audit(bag_path, scenario_path):
    result={'passed':False,'checks':{}}
    try:
        scenario=json.loads(Path(scenario_path).read_text())
        step=scenario['steps'][0]
        begin=step['injection']['fault_start_sim_s']
        bad=[];bad_receipts=[];filtered=[];diag=[];ev=[];lidar=[];imu=[];invalid=[];logs=[]
        topics=['/Odometry','/faster_lio/translation_observability','/mavros/odometry/out',
                '/livox/lidar','/livox/imu','/drone/lio/valid','/rosout']
        with rosbag.Bag(str(bag_path)) as bag:
            for topic,m,t in bag.read_messages(topics=topics):
                if topic=='/Odometry':
                    trace=sum(m.pose.covariance[i] for i in (0,7,14))
                    if trace>1e5:
                        bad.append(m.header.stamp.to_sec());bad_receipts.append(t.to_sec())
                elif topic=='/faster_lio/translation_observability':
                    if len(m.data)==6:diag.append(list(m.data))
                elif topic=='/mavros/odometry/out':ev.append(m.header.stamp.to_sec())
                elif topic=='/livox/lidar':
                    stamp=m.header.stamp.to_sec()+.1
                    if begin<=t.to_sec()<=begin+1.5 and m.point_num>=2 and all(
                            abs(-.5*p.x+math.sqrt(.75)*p.z)<.20001 for p in m.points):
                        filtered.append(stamp)
                    if begin+.3<=stamp<=begin+1.3:
                        lidar.append({'stamp':stamp,'points':m.point_num,
                            'valid':m.point_num==len(m.points) and m.point_num>=2 and
                            all(p.offset_time==100000000 and p.line<4 and
                                all(math.isfinite(v) for v in (p.x,p.y,p.z)) and
                                abs(-.5*p.x+math.sqrt(.75)*p.z)<.20001 for p in m.points)})
                elif topic=='/livox/imu':
                    if begin+.3<=m.header.stamp.to_sec()<=begin+1.3:imu.append(m.header.stamp.to_sec())
                elif topic=='/drone/lio/valid' and not m.data:invalid.append(t.to_sec())
                elif topic=='/rosout' and 'position observations are degenerate' in m.msg:logs.append(m.msg)
        first=min(bad) if bad else None
        checks={
            # Fault acts on callback delivery, which can contain an older
            # queued scan. Match its acquisition time to filtered input and
            # require the bad pose receipt after injection; do not shift time.
            'native_covariance_inflated':first is not None and
                begin<=min(bad_receipts)<=begin+1.5 and
                any(abs(first-stamp)<1e-5 for stamp in filtered),
            'current_plane_information_weak':first is not None and any(abs(d[0]-first)<1e-5 and d[1]<50 and d[5]>=1e5 for d in diag),
            'no_ev_pose_after_invalid_scan':first is not None and bool(ev) and max(ev)<first+1e-6,
            'bridge_invalid_and_cause_logged':bool(invalid and logs) and step.get('bridge_remained_invalid_after_landing',False),
            'lidar_stream_and_original_timing_preserved':len(lidar)>=3 and all(x['valid'] for x in lidar),
            'independent_imu_continues':len(imu)>=30,
            'protective_landing_verified':scenario.get('passed',False) and step.get('passed',False)}
        result.update(checks=checks,first_invalid_scan=first,fault_start=begin,
                      first_invalid_receipt=min(bad_receipts) if bad_receipts else None,
                      initial_filtered_scan_stamps=filtered,
                      last_ev_stamp=max(ev) if ev else None,lidar_samples=lidar,imu_samples=len(imu))
        result['passed']=all(checks.values())
    except Exception as exc:result['error']=str(exc)
    return result
