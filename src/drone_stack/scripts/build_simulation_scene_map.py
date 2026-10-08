#!/usr/bin/env python3
"""Create an explicitly synthetic navigation prior from Gazebo collision boxes."""
import argparse,hashlib,json,math,os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, ExtendedState
from tf.transformations import euler_matrix,quaternion_matrix,euler_from_quaternion


def sdf_pose(element):
    text=element.findtext('pose','0 0 0 0 0 0')
    v=list(map(float,text.split()))
    if len(v)!=6:raise ValueError('Invalid SDF pose')
    matrix=euler_matrix(*v[3:]);matrix[:3,3]=v[:3];return matrix


def pose_matrix(p):
    q=p.orientation;matrix=quaternion_matrix([q.x,q.y,q.z,q.w])
    matrix[:3,3]=[p.position.x,p.position.y,p.position.z];return matrix


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();rospy.init_node('build_simulation_scene_map',anonymous=True)
    if not rospy.get_param('/use_sim_time',False):raise RuntimeError('Scene map generator requires Gazebo simulation')
    state=rospy.wait_for_message('/mavros/state',State,timeout=8)
    landed=rospy.wait_for_message('/mavros/extended_state',ExtendedState,timeout=8)
    if not state.connected or state.armed or landed.landed_state!=ExtendedState.LANDED_STATE_ON_GROUND:
        raise RuntimeError('Generate the scene prior only while connected, grounded and disarmed')
    models=rospy.wait_for_message('/gazebo/model_states',ModelStates,timeout=8)
    odom=rospy.wait_for_message('/mavros/local_position/odom',Odometry,timeout=8)
    transforms={name:pose_matrix(p) for name,p in zip(models.name,models.pose)}
    if 'inspection_quad' not in transforms:raise ValueError('Missing inspection_quad in current scene')
    body=transforms['inspection_quad'];local=pose_matrix(odom.pose.pose)
    yaw=euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])[2]
    truth_yaw=math.atan2(body[1,0],body[0,0])
    map_from_source=euler_matrix(0,0,truth_yaw-yaw)
    map_from_source[:3,3]=body[:3,3]-map_from_source[:3,:3]@local[:3,3]
    source_from_map=np.linalg.inv(map_from_source)
    resolution=float(rospy.get_param('/ego_planner_node/grid_map/resolution',.1))
    size=np.array([rospy.get_param('/ego_planner_node/grid_map/map_size_'+a,v) for a,v in zip('xyz',[30.,30.,8.])],float)
    origin=np.array([-size[0]/2,-size[1]/2,rospy.get_param('/ego_planner_node/grid_map/ground_height',.5)])
    dims=np.ceil(size/resolution).astype(int);occupied=set();boxes=[]
    world_path=Path(args.world).resolve()
    for model in ET.parse(world_path).getroot().findall('world/model'):
        name=model.get('name')
        if model.findtext('static','false')!='true':continue
        world_from_model=transforms.get(name,sdf_pose(model))
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                box=collision.find('geometry/box')
                if box is None:raise ValueError('Unsupported collision geometry: '+name)
                half=np.array(list(map(float,box.findtext('size').split())))/2
                transform=source_from_map@world_from_model@sdf_pose(link)@sdf_pose(collision)
                axes=transform[:3,:3];center=transform[:3,3]
                if abs(axes[2,0])+abs(axes[2,1])+abs(axes[0,2])+abs(axes[1,2])>1e-6:raise ValueError('Only yaw-rotated scene boxes are currently supported')
                extent=np.abs(axes)@half
                lo=np.maximum(0,np.floor((center-extent-origin-1e-8)/resolution).astype(int))
                hi=np.minimum(dims-1,np.floor((center+extent-origin+1e-8)/resolution).astype(int))
                count=0
                for x in range(lo[0],hi[0]+1):
                    for y in range(lo[1],hi[1]+1):
                        xy=origin[:2]+(np.array([x,y])+.5)*resolution-center[:2]
                        # SAT on voxel XY axes and oriented obstacle XY axes.
                        if np.any(np.abs(xy)>extent[:2]+resolution*.5+1e-9):continue
                        if np.any(np.abs(axes[:2,:2].T@xy)>half[:2]+resolution*.5*np.sum(np.abs(axes[:2,:2]),axis=0)+1e-9):continue
                        for z in range(lo[2],hi[2]+1):
                            if abs(origin[2]+(z+.5)*resolution-center[2])>half[2]+resolution*.5+1e-9:continue
                            occupied.add(int(x*dims[1]*dims[2]+y*dims[2]+z));count+=1
                boxes.append({'model':name,'collision':collision.get('name'),'source_center':center.tolist(),'size':(half*2).tolist(),'candidate_occupied_voxels':count})
    if not occupied:raise ValueError('No scene obstacles generated')
    output=Path(args.output).resolve();output.parent.mkdir(parents=True,exist_ok=True)
    metadata=[*map_from_source[:3,3],math.degrees(truth_yaw-yaw)]
    header=['DRONE_GRID_V2','odom',resolution,*origin,*dims,len(occupied),*metadata]
    temporary=output.with_suffix(output.suffix+'.tmp')
    with temporary.open('w') as f:
        f.write(' '.join(map(str,header))+'\n')
        for address in sorted(occupied):f.write('%d 3 0\n'%address)
    os.replace(temporary,output)
    info={'source':'Gazebo collision geometry; synthetic prior, not a lidar scan or localization input','map_coordinates':'Gazebo world ENU','world':str(world_path),'world_sha256':hashlib.sha256(world_path.read_bytes()).hexdigest(),'map_sha256':hashlib.sha256(output.read_bytes()).hexdigest(),'resolution':resolution,'source_origin':origin.tolist(),'source_dims':dims.tolist(),'map_from_source_translation':metadata[:3],'map_from_source_yaw_deg':metadata[3],'occupied_voxels':len(occupied),'free_voxels':0,'boxes':boxes,'initial_body_world_xyz':body[:3,3].tolist(),'initial_body_world_yaw_deg':math.degrees(truth_yaw),'note':'Raw solid obstacle voxels only. Inflation and ceiling regenerated by map backend; real-time measured rays provide free space. Floor below navigation grid is excluded.'}
    output.with_suffix('.json').write_text(json.dumps(info,ensure_ascii=False,indent=2))
    print(json.dumps({'map':str(output),'occupied_voxels':len(occupied),'boxes':len(boxes),'bytes':output.stat().st_size},ensure_ascii=False))


if __name__=='__main__':main()
