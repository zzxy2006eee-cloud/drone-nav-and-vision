#!/usr/bin/env python3
"""Audit recorded air poses against all static geometry in the selected world."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import rosbag
from sim_world_geometry import WorldGeometry


def audit(bag_path,world_path):
    world=WorldGeometry(world_path)
    phase=None;odom=None;positions=[];quaternions=[];times=[];phases=[]
    tracking=[];feedforward=[];max_gap=0.;previous=None
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,m,t in bag.read_messages(topics=['/drone/flight_state','/gazebo/model_states',
                                                  '/mavros/local_position/odom','/mavros/setpoint_raw/local']):
            if topic=='/drone/flight_state':
                phase=m.data
                if phase not in ('TAKEOFF','HOLD','NAVIGATING'):previous=None
            elif topic=='/mavros/local_position/odom':odom=m.pose.pose.position
            elif topic=='/mavros/setpoint_raw/local' and phase=='NAVIGATING' and odom is not None:
                tracking.append(float(np.linalg.norm(np.array([m.position.x,m.position.y,m.position.z])-
                                                     [odom.x,odom.y,odom.z])))
                feedforward.append(not bool(m.type_mask&m.IGNORE_VX))
            elif topic=='/gazebo/model_states' and phase in ('TAKEOFF','HOLD','NAVIGATING'):
                if 'inspection_quad' not in m.name:raise ValueError('Vehicle missing from truth message')
                pose=m.pose[m.name.index('inspection_quad')];p=pose.position;q=pose.orientation
                positions.append([p.x,p.y,p.z]);quaternions.append([q.x,q.y,q.z,q.w]);times.append(t.to_sec());phases.append(phase)
                if previous is not None:max_gap=max(max_gap,t.to_sec()-previous)
                previous=t.to_sec()
    if not positions:return {'passed':False,'reason':'No air poses recorded'}
    gaps,closest=world.check_many(positions,quaternions)
    overlap=np.flatnonzero(gaps<=0)
    index=int(gaps.argmin())
    ground_violations=[i for i,(p,state) in enumerate(zip(positions,phases)) if state in ('HOLD','NAVIGATING') and p[2]<.25]
    passed=len(overlap)==0 and len(ground_violations)==0 and max_gap<=.1 and 'NAVIGATING' in phases
    tools=[Path(__file__),Path(__file__).with_name('sim_world_geometry.py')]
    result={'source_bag':str(Path(bag_path).resolve()),'source_world':str(world.path),
            'world_sha256':hashlib.sha256(world.path.read_bytes()).hexdigest(),
            'tools_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in tools},
            'passed':passed,'truth_samples':len(positions),'static_collision_boxes':len(world.boxes),
            'vehicle_envelope_half_m':world.vehicle_half.tolist(),'max_pose_receipt_gap_sim_s':max_gap,
            'minimum_separating_axis_gap_m':float(gaps[index]),'closest_obstacle':world.boxes[closest[index]][0],
            'overlap_samples':len(overlap),'grounded_during_hold_or_navigation_samples':len(ground_violations),
            'first_overlap':[{'time':times[i],'position':positions[i],'phase':phases[i],
                             'obstacle':world.boxes[closest[i]][0]} for i in overlap[:10]],
            'limits':['Finite sampled envelope check, not a continuous collision proof.',
                      'Ground contact during takeoff/landing is not a static obstacle overlap.',
                      'Tracking compares latest received FCU pose without timestamp interpolation.']}
    if tracking:
        result['navigation_tracking']={'samples':len(tracking),'max_error_m':max(tracking),
                                      'p95_error_m':float(np.percentile(tracking,95)),
                                      'velocity_feedforward_samples':sum(feedforward)}
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag');p.add_argument('world');p.add_argument('output');args=p.parse_args()
    result=audit(args.bag,args.world);Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':main()
