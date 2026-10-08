#!/usr/bin/env python3
"""Replay previously recorded body odometry through the current LIO bridge.

This reconstructs the corresponding sensor pose; it cannot re-run Faster-LIO
or certify live flight, because these bags do not contain the original scans.
"""
import argparse
import importlib.util
import json
from collections import deque
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import rosbag
import rospy
import tf.transformations as tr
from nav_msgs.msg import Odometry


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag');parser.add_argument('output');args=parser.parse_args()
    spec=importlib.util.spec_from_file_location('lio_bridge',Path(__file__).with_name('lio_bridge.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    b=module.LioBridge.__new__(module.LioBridge)
    b.max_age=.6;b.future_tolerance=.02;b.last_stamp=rospy.Time(0);b.pose_fault=False
    b.max_pose_step=.5;b.max_pose_speed=3.;b.max_rotation_step=.3;b.max_rotation_speed=4.
    b.world_align=tr.euler_matrix(0,np.pi/6,0)
    b.base_sensor=tr.translation_matrix([.02,0,.1])@b.world_align;b.sensor_base=np.linalg.inv(b.base_sensor)
    b.pose_cov=[.0025,.0025,.0025,.01,.01,.0025];b.lio_poses=deque(maxlen=30)
    b.odom_pub=Mock();b.mavros_pub=Mock();b.tf_pub=Mock();b.valid_pub=Mock()
    count=0;rejected=[];max_error=0
    with rosbag.Bag(args.bag) as bag:
        for _,record,t in bag.read_messages(topics=['/drone/lio/odom']):
            count+=1;body=module.LioBridge.pose_matrix(record.pose.pose)
            sensor=np.linalg.inv(b.world_align)@body@b.base_sensor
            msg=Odometry();msg.header.frame_id='camera_init';msg.header.stamp=record.header.stamp
            msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=sensor[:3,3]
            q=tr.quaternion_from_matrix(sensor)
            msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=q
            before=b.mavros_pub.publish.call_count
            with patch('rospy.Time.now',return_value=t):b.on_odom(msg)
            if b.mavros_pub.publish.call_count==before:
                rejected.append({'index':count,'stamp':msg.header.stamp.to_sec(),
                                 'age_s':(t-msg.header.stamp).to_sec(),'pose_fault':b.pose_fault})
            else:
                output=b.mavros_pub.publish.call_args[0][0]
                error=np.linalg.norm(module.LioBridge.pose_matrix(output.pose.pose)-body)
                max_error=max(max_error,float(error))
    result={'source_bag':str(Path(args.bag).resolve()),'kind':'historical_odometry_replay_without_new_flight',
            'samples':count,'rejected':rejected,'latched_pose_fault':b.pose_fault,
            'max_reconstruction_matrix_error':max_error,
            'passed':count>0 and not rejected and not b.pose_fault and max_error<1e-9}
    Path(args.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':main()
