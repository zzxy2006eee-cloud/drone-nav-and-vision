#!/usr/bin/env python3
"""Check actual EGO synchronized ray callbacks without launching an aircraft."""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = Path(args.directory).resolve()
    out.mkdir(parents=True, exist_ok=False)
    if rosgraph.is_master_online():
        raise SystemExit('Existing ROS master')
    launch = out/'test.launch'
    launch.write_text('<launch><include file="'+str(root/'src/drone_stack/launch/ego.launch')+'"/>'
        '<rosparam param="/ego_planner_node/grid_map/cloud_sensor_xyz_body">[0,0,0]</rosparam></launch>')
    env = dict(os.environ, ROS_HOSTNAME='127.0.0.1', ROS_MASTER_URI='http://127.0.0.1:11311')
    env.pop('ROS_IP', None)
    report = {'passed': False, 'checks': {}, 'limits': 'Synthetic rays through actual EGO callbacks, no flight.'}
    process = None
    try:
        with (out/'launch.log').open('w') as log:
            process = subprocess.Popen(['roslaunch',str(launch)],env=env,stdout=log,
                                       stderr=subprocess.STDOUT,start_new_session=True)
        def wait(condition):
            deadline = time.monotonic()+15
            while time.monotonic()<deadline:
                if condition(): return
                if process.poll() is not None: raise RuntimeError('EGO exited')
                time.sleep(.03)
            raise RuntimeError('Condition timed out')
        wait(rosgraph.is_master_online)
        rospy.init_node('observed_free_contract',anonymous=True)
        data = {}
        subscriptions = [rospy.Subscriber(topic,PointCloud2,lambda m,k=key:data.update({k:m}),
                          queue_size=1,tcp_nodelay=True)
                         for topic,key in [('/grid_map/observed_free','free'),
                                           ('/grid_map/occupancy_inflate_safety','occupied')]]
        clouds = rospy.Publisher('/drone/cloud_fcu_world',PointCloud2,queue_size=1)
        poses = rospy.Publisher('/drone/cloud_body_pose',PoseStamped,queue_size=1)
        phases = rospy.Publisher('/drone/flight_state',String,queue_size=1,latch=True)
        wait(lambda:all(p.get_num_connections()>0 for p in [clouds,poses,phases]))
        time.sleep(.3)
        def pair(x, stamp=None):
            stamp = stamp or rospy.Time.now()
            h = Header(stamp=stamp,frame_id='odom')
            pose = PoseStamped(header=h)
            pose.pose.position.x=.05;pose.pose.position.y=.05;pose.pose.position.z=1.05
            pose.pose.orientation.w=1
            cloud = point_cloud2.create_cloud_xyz32(h,[[x,.05,1.05]])
            poses.publish(pose);clouds.publish(cloud)
            wait(lambda:'free' in data and data['free'].header.stamp==stamp)
            return pose,cloud
        def cell_present(x):
            return any(abs(px-x)<.01 and abs(py-.05)<.01 and abs(pz-1.05)<.01
                       for px,py,pz in point_cloud2.read_points(data['free'],field_names=('x','y','z')))
        pair(-2.05)
        assert cell_present(-1.05)
        phases.publish(String(data='READY'));time.sleep(.3)
        pose,cloud = pair(2.05)
        report['checks']['ray_middle_known_free']=cell_present(1.05)
        report['checks']['body_blind_voxel_known_self_space']=any(abs(x-.15)<.01 and abs(y-.05)<.01 and abs(z-1.15)<.01
            for x,y,z in point_cloud2.read_points(data['free'],field_names=('x','y','z')))
        report['checks']['return_and_behind_not_free']=not cell_present(2.05) and not cell_present(3.05)
        report['checks']['ready_clears_initial_free_memory']=not cell_present(-1.05)
        previous = cloud.header.stamp
        cloud.header=copy.deepcopy(cloud.header)
        pose.header.stamp=rospy.Time.now();cloud.header.stamp=pose.header.stamp+rospy.Duration(.01)
        poses.publish(pose);clouds.publish(cloud);time.sleep(.3)
        report['checks']['mismatched_capture_times_do_not_update']=data['free'].header.stamp==previous
        pose.header.stamp=cloud.header.stamp;poses.publish(pose)
        wait(lambda:data['free'].header.stamp==cloud.header.stamp)
        phases.publish(String(data='HOLD'))
        pair(-2.05)
        phases.publish(String(data='READY'));time.sleep(.3)
        report['checks']['later_ready_keeps_observed_memory']=cell_present(-1.05) and cell_present(1.05)
        previous = data['free'].header.stamp
        h = Header(stamp=rospy.Time.now(),frame_id='odom')
        pose.header=h;poses.publish(pose)
        clouds.publish(point_cloud2.create_cloud_xyz32(h,[]));time.sleep(.3)
        report['checks']['empty_cloud_does_not_refresh']=data['free'].header.stamp==previous
        report['passed']=all(report['checks'].values()) and len(report['checks'])==7
    except Exception as exc:
        report['error']=str(exc)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__': main()
