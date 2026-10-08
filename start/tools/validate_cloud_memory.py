#!/usr/bin/env python3
"""Exercise real EGO cloud callbacks under an owned ROS master, without flight."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rosgraph,rospy
import sys
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header,String
p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('--mode',choices=['retain','replace'],required=True);p.add_argument('--ready-gate',action='store_true');a=p.parse_args()
root=Path(__file__).resolve().parents[2];out=Path(a.directory).resolve();out.mkdir(parents=True,exist_ok=True)
if rosgraph.is_master_online():raise SystemExit('An existing ROS master must be stopped first.')
keep=a.mode=='retain';launch=out/'map_test.launch';launch.write_text('<launch><include file="'+str(root/'src/drone_stack/launch/ego.launch')+'"/><param name="/ego_planner_node/grid_map/retain_cloud_obstacles" value="'+str(keep).lower()+'"/><param name="/ego_planner_node/grid_map/cloud_memory_start_topic" value="'+('/drone/flight_state' if a.ready_gate else '')+'"/><param name="/ego_planner_node/grid_map/require_observed_free" value="false"/><param name="/ego_planner_node/grid_map/cloud_body_pose_topic" value=""/></launch>\n')
env=dict(os.environ,ROS_HOSTNAME='127.0.0.1',ROS_MASTER_URI='http://127.0.0.1:11311');env.pop('ROS_IP',None)
sys.path.insert(0,str(root/'src/drone_stack/scripts'))
from run_core_acceptance import fingerprint
report={'mode':a.mode,'passed':False,'configuration_sha256':fingerprint(root),
        'limits':'Actual EGO ROS callback test with synthetic world-frame point clouds; no simulated aircraft.'};sim=None
try:
 with (out/'launch.log').open('w') as log:sim=subprocess.Popen(['roslaunch',str(launch)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 deadline=time.monotonic()+25
 while not rosgraph.is_master_online():
  if time.monotonic()>deadline or sim.poll() is not None:raise RuntimeError('ROS startup failed')
  time.sleep(.1)
 rospy.init_node('cloud_memory_contract',anonymous=True);data={}
 sub=rospy.Subscriber('/grid_map/occupancy_inflate_safety',PointCloud2,lambda m:data.update(map=m),queue_size=1,tcp_nodelay=True)
 cloud_pub=rospy.Publisher('/drone/cloud_fcu_world',PointCloud2,queue_size=1)
 odom_pub=rospy.Publisher('/mavros/local_position/odom',Odometry,queue_size=1)
 def odom(_):
  m=Odometry();m.header.stamp=rospy.Time.now();m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.position.z=1;m.pose.pose.orientation.w=1;odom_pub.publish(m)
 timer=rospy.Timer(rospy.Duration(.05),odom)
 def wait(condition):
  deadline=time.monotonic()+10
  while time.monotonic()<deadline:
   if condition():return
   time.sleep(.05)
  raise RuntimeError('Map condition timed out')
 wait(lambda:cloud_pub.get_num_connections()>0 and odom_pub.get_num_connections()>0)
 time.sleep(.3)
 if rospy.get_param('/ego_planner_node/grid_map/retain_cloud_obstacles')!=keep:raise RuntimeError('Retain parameter override failed')
 def contains(x):
  if 'map' not in data:return False
  return any(abs(px-x)<.12 and abs(py)<.12 and abs(pz-1)<.12 for px,py,pz in point_cloud2.read_points(data['map'],field_names=('x','y','z'),skip_nans=True))
 first=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='odom'),[[2,0,1]])
 cloud_pub.publish(first);wait(lambda:'map' in data and data['map'].header.stamp==first.header.stamp and contains(2))
 second=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='odom'),[[-2,0,1]])
 cloud_pub.publish(second);wait(lambda:data['map'].header.stamp==second.header.stamp and contains(-2))
 if a.ready_gate:
  report['before_ready_replaces_old_obstacle']=not contains(2)
  if contains(2):raise RuntimeError('Initialization cloud unexpectedly retained')
  state_pub=rospy.Publisher('/drone/flight_state',String,queue_size=1,latch=True)
  wait(lambda:state_pub.get_num_connections()>0)
  state_pub.publish(String(data='READY'));time.sleep(.3)
  first=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='odom'),[[2,0,1]])
  cloud_pub.publish(first);wait(lambda:data['map'].header.stamp==first.header.stamp and contains(2))
  report['ready_clears_initialization_obstacle']=not contains(-2)
  if contains(-2):raise RuntimeError('READY left initialization occupancy')
  state_pub.publish(String(data='HOLD'));time.sleep(.1)
  second=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='odom'),[[-2,0,1]])
  cloud_pub.publish(second);wait(lambda:data['map'].header.stamp==second.header.stamp and contains(-2))
  state_pub.publish(String(data='READY'));time.sleep(.3)
 old_present=contains(2);report['old_obstacle_present_after_unseen_scan']=old_present;report['new_obstacle_present']=contains(-2)
 if old_present!=keep:raise RuntimeError('Unseen obstacle retention differs from configured mode')
 cloud_pub.publish(first);time.sleep(.4)
 report['old_stamp_does_not_refresh']=data['map'].header.stamp==second.header.stamp
 empty=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='odom'),[]);cloud_pub.publish(empty);time.sleep(.4)
 report['empty_scan_does_not_refresh']=data['map'].header.stamp==second.header.stamp
 report['configuration_unchanged']=fingerprint(root)==report['configuration_sha256']
 report['passed']=report['configuration_unchanged'] and report['old_stamp_does_not_refresh'] and report['empty_scan_does_not_refresh'] and contains(-2) and contains(2)==keep
except Exception as exc:report['error']=str(exc)
finally:
 if sim is not None and sim.poll() is None:
  os.killpg(sim.pid,signal.SIGINT)
  try:sim.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGTERM);sim.wait(timeout=5)
 (out/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
raise SystemExit(0 if report['passed'] else 1)
