#!/usr/bin/env python3
"""Ground-only map/session alignment; no changes to LIO, EKF or control ENU."""
import json
import hashlib
import math
import os
import threading
import time
from pathlib import Path
import numpy as np
import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, PoseStamped
from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from plan_env.srv import MapArchive, MapArchiveRequest
from drone_stack.srv import MapSession, MapSessionResponse, MapSessionRequest
from gazebo_msgs.msg import ModelStates


def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class MapSessionManager:
    def __init__(self):
        self.lock=threading.RLock();self.latest={};self.receipts={}
        self.ready=False;self.initialized=False;self.prepared=None
        self.map_context={}
        self.translation=np.zeros(3);self.yaw=0.;self.mode='WAITING'
        self.startup_map=rospy.get_param('~simulation_startup_map','')
        if rospy.get_param('~simulation_online_start',False):self.startup_map=''
        self.startup_loading=False;self.startup_retry_wall=0.
        self.boot_pending=bool(self.startup_map);self.boot_preview=None
        if self.startup_map and not rospy.get_param('/use_sim_time',False):
            raise ValueError('Automatic truth-assisted map alignment is simulation-only')
        self.ready_pub=rospy.Publisher('/drone/map_alignment_ready',Bool,queue_size=1,latch=True)
        self.status_pub=rospy.Publisher('/drone/map_session_status',String,queue_size=1,latch=True)
        self.pose_pub=rospy.Publisher('/drone/map_transform',PoseStamped,queue_size=1,latch=True)
        self.preview_pub=rospy.Publisher('/drone/prior_map_preview',PointCloud2,queue_size=1,latch=True)
        self.broadcaster=tf2_ros.TransformBroadcaster()
        self.archive=rospy.ServiceProxy('/drone/map_archive',MapArchive)
        for topic,kind,key in [('/mavros/state',State,'state'),('/mavros/extended_state',ExtendedState,'extended'),('/mavros/local_position/odom',Odometry,'odom'),('/drone/flight_state',String,'phase'),('/drone/flight_health',String,'health'),('/drone/goal_queue',RosPath,'queue')]:
            rospy.Subscriber(topic,kind,self.receive,callback_args=key,queue_size=1,tcp_nodelay=True)
        if self.startup_map:
            rospy.Subscriber('/gazebo/model_states',ModelStates,self.receive,callback_args='simulation_models',queue_size=1)
        rospy.Service('/drone/map_session',MapSession,self.service)
        if self.boot_pending:
            self.mode='WAITING_POSE'
            try:
                self.boot_preview,self.prepared=self.read_preview(self.startup_map)
                self.preview(self.boot_preview)
                self.publish('先点选初始位置并拖动朝向，确认后启动Gazebo；当前仿真尚未启动')
            except Exception as exc:self.publish('默认地图无法预览，请选择地图文件：'+str(exc))
        else:
            rospy.set_param('/drone/simulation_spawn_request',{'x':1.01,'y':.98,'z':.17,'yaw_deg':0.,'mode':'ONLINE'})
            self.publish('在线模式启动，等待地面定位就绪')
        rospy.Timer(rospy.Duration(.1),self.tick,reset=True)

    def receive(self,msg,key):
        with self.lock:self.latest[key]=msg;self.receipts[key]=time.monotonic()

    def ground_error(self):
        d=self.latest;s=d.get('state');e=d.get('extended');o=d.get('odom')
        if not s or not s.connected or s.armed or not e or e.landed_state!=ExtendedState.LANDED_STATE_ON_GROUND:
            return '地图坐标操作需要已确认落地且未解锁'
        if not o or any(time.monotonic()-self.receipts.get(k,0)>3 for k in ['state','extended','odom','health']):
            return '飞控/位姿/健康数据过期'
        if d.get('phase') is None or d['phase'].data!='READY' or d['health'].data!='HEALTHY':return '需要READY且定位健康'
        if d.get('queue') is not None and d['queue'].poses:return '请先清空目标队列'
        p=o.pose.pose.position;q=o.pose.pose.orientation
        if not all(math.isfinite(v) for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.025:return '初始位姿无效'
        return ''

    def publish(self,message):
        self.ready_pub.publish(Bool(self.ready))
        self.status_pub.publish(String(json.dumps({'mode':self.mode,'ready':self.ready,'message':message,'map_context':self.map_context},ensure_ascii=False)))
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=rospy.Time.now()
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=self.translation
        q=quaternion_from_euler(0,0,self.yaw)
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=q
        self.pose_pub.publish(pose)
        rospy.loginfo('Map session %s ready=%s: %s',self.mode,self.ready,message)

    def set_alignment(self,xyz,yaw):
        o=self.latest['odom'];p=o.pose.pose.position
        self.yaw=yaw-yaw_of(o.pose.pose.orientation)
        self.yaw=math.atan2(math.sin(self.yaw),math.cos(self.yaw))
        self.translation=np.array(xyz)-rotation(self.yaw)@np.array([p.x,p.y,p.z])
        self.initialized=True

    def set_map_context(self,path,expected):
        source=Path(path).resolve();digest=hashlib.sha256()
        self.map_context={}
        try:
            with source.open('rb') as f:
                for block in iter(lambda:f.read(1024*1024),b''):digest.update(block)
            info=source.stat()
            if (info.st_size,info.st_mtime_ns)!=expected:raise ValueError('文件已变化，请重新加载地图')
            self.map_context=dict(path=str(source),digest=digest.hexdigest(),size=info.st_size,mtime_ns=info.st_mtime_ns)
        except (OSError,ValueError) as exc:
            rospy.logwarn('Map region-library identity unavailable: %s',exc)

    def load_startup_map(self):
        """Use truth only for one grounded initial map alignment, never pose fusion."""
        try:
            result=self.service(MapSessionRequest(action='prepare',path=self.startup_map))
            if not result.success:raise RuntimeError(result.message)
            # Let health/state subscribers refresh after archive parsing.
            time.sleep(.2)
            with self.lock:
                reason=self.ground_error()
                if reason:raise RuntimeError(reason)
                states=self.latest.get('simulation_models')
                if states is None or time.monotonic()-self.receipts.get('simulation_models',0)>3 or 'inspection_quad' not in states.name:
                    raise RuntimeError('等待仿真初始机体位姿')
                pose=states.pose[states.name.index('inspection_quad')]
                spawn=rospy.get_param('/drone/simulation_spawn_request',{})
                if spawn.get('mode')=='PRIOR_NAV':
                    delta=np.array([pose.position.x-spawn['x'],pose.position.y-spawn['y'],pose.position.z-spawn['z']])
                    dyaw=yaw_of(pose.orientation)-math.radians(spawn['yaw_deg'])
                    if np.linalg.norm(delta)>.1 or abs(math.atan2(math.sin(dyaw),math.cos(dyaw)))>math.radians(5):
                        raise RuntimeError('Gazebo实际出生位姿与用户设置不符，禁止加载导航')
                request=MapSessionRequest(action='load',path=self.startup_map,x=pose.position.x,
                                          y=pose.position.y,z=pose.position.z,yaw_deg=math.degrees(yaw_of(pose.orientation)))
            result=self.service(request)
            if not result.success:raise RuntimeError(result.message)
        except Exception as exc:
            with self.lock:
                self.ready=False;self.publish('仿真地图自动加载尚未完成：'+str(exc))
                self.startup_retry_wall=time.monotonic()+5.
        finally:
            with self.lock:self.startup_loading=False

    def tick(self,_):
        with self.lock:
            if not self.boot_pending and not self.initialized and not self.ground_error():
                if self.startup_map:
                    if not self.startup_loading and time.monotonic()>=self.startup_retry_wall:
                        self.startup_loading=True
                        threading.Thread(target=self.load_startup_map,daemon=True).start()
                else:
                    self.set_alignment([0.,0.,0.],0.)
                    self.ready=True;self.mode='ONLINE'
                    self.publish('在线地图已将起始机体位置设为原点、起始机头设为+X')
            t=TransformStamped();t.header.stamp=rospy.Time.now();t.header.frame_id='map';t.child_frame_id='odom'
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=self.translation
            q=quaternion_from_euler(0,0,self.yaw)
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=q
            self.broadcaster.sendTransform(t)

    @staticmethod
    def read_preview(path):
        """Validate all records before previewing; no mutation of planner grid."""
        p=Path(path)
        if not p.is_absolute() or not p.is_file() or p.stat().st_size>256*1024*1024:raise ValueError('地图路径或文件大小无效')
        with p.open() as f:
            h=f.readline().split()
            if len(h) not in (10,14) or h[0] not in ('DRONE_GRID_V1','DRONE_GRID_V2') or h[1]!='odom':raise ValueError('地图格式或坐标系不兼容')
            if len(h)!=(14 if h[0]=='DRONE_GRID_V2' else 10):raise ValueError('地图坐标元数据不完整')
            res=float(h[2]);origin=np.array(list(map(float,h[3:6])));dims=np.array(list(map(int,h[6:9])));count=int(h[9])
            saved=np.array(list(map(float,h[10:13]))) if len(h)==14 else np.zeros(3)
            saved_yaw=math.radians(float(h[13])) if len(h)==14 else 0.
            if not math.isfinite(res) or res<=0 or not np.isfinite(origin).all() or not np.isfinite(saved).all() or not math.isfinite(saved_yaw) or any(dims<=0) or count<1 or count>int(np.prod(dims)):raise ValueError('地图元数据无效')
            occupied=[];seen=set()
            for _ in range(count):
                row=f.readline().split()
                if len(row)!=3:raise ValueError('地图记录不完整')
                address,evidence,free=map(int,row)
                if address in seen or not 0<=address<int(np.prod(dims)) or not 0<=evidence<=3 or free not in (0,1) or (evidence and free) or not(evidence or free):raise ValueError('地图体素记录无效')
                seen.add(address)
                if evidence:
                    occupied.append(address)
            if f.read().strip() or not occupied:raise ValueError('地图多余记录或没有障碍')
        addresses=np.asarray(occupied,dtype=np.int64)
        indices=np.column_stack((addresses//(dims[1]*dims[2]),addresses//dims[2]%dims[1],addresses%dims[2]))
        points=(origin+(indices+.5)*res)@rotation(saved_yaw).T+saved
        st=p.stat()
        return points,(str(p),st.st_size,st.st_mtime_ns)

    def preview(self,xyz):
        pts=np.asarray(xyz,dtype=np.float32).reshape(-1,3)
        msg=PointCloud2(height=1,width=len(pts),point_step=12,row_step=12*len(pts),is_dense=True,data=pts.tobytes())
        msg.header.frame_id='map';msg.header.stamp=rospy.Time.now()
        msg.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z'])]
        self.preview_pub.publish(msg)

    def boot_request(self,req):
        if req.action=='prepare':
            self.boot_preview,self.prepared=self.read_preview(req.path)
            self.startup_map=req.path;self.preview(self.boot_preview)
            self.publish('地图预览已就绪，请点选出生位置并拖动朝向，然后确认启动仿真')
            return MapSessionResponse(True,'预览完成；Gazebo尚未启动',False)
        if req.action in ('new','online'):
            self.startup_map='';self.prepared=None;self.boot_preview=None;self.preview([])
            request={'x':1.01,'y':.98,'z':.17,'yaw_deg':0.,'mode':'ONLINE'}
        elif req.action=='load':
            if not self.prepared or self.prepared[0]!=req.path:raise ValueError('请先打开此地图预览')
            st=Path(req.path).stat()
            if (req.path,st.st_size,st.st_mtime_ns)!=self.prepared:raise ValueError('地图已变化，请重新预览')
            if not all(math.isfinite(v) for v in [req.x,req.y,req.z,req.yaw_deg]) or max(abs(req.x),abs(req.y))>8 or abs(req.yaw_deg)>180:raise ValueError('出生位置/朝向无效；XY需在±8m内')
            if not .15<=req.z<=.20:raise ValueError('当前平地场景初始机体Z需为0.15～0.20m（推荐0.17m），不能从空中开始')
            if np.any(np.linalg.norm(self.boot_preview[:,:2]-[req.x,req.y],axis=1)<.55):raise ValueError('出生位置靠近地图障碍，请保留至少0.55m间距')
            request={'x':req.x,'y':req.y,'z':req.z,'yaw_deg':req.yaw_deg,'mode':'PRIOR_NAV','map':req.path}
        else:raise ValueError('仿真尚未启动；请设置初始位姿或选择在线模式')
        rospy.set_param('/drone/simulation_spawn_request',request)
        self.boot_pending=False;self.mode='STARTING_SIM'
        self.publish('初始位姿已提交，正在启动Gazebo/PX4和定位；就绪后加载地图')
        return MapSessionResponse(True,'已按初始位姿请求启动仿真，请等待定位就绪',False)

    def service(self,req):
        with self.lock:
            if self.boot_pending:
                try:return self.boot_request(req)
                except Exception as exc:return MapSessionResponse(False,str(exc),False)
            reason=self.ground_error()
            if reason:return MapSessionResponse(False,reason,self.ready)
            try:
                if req.action=='prepare':
                    points,fingerprint=self.read_preview(req.path)
                    self.prepared=fingerprint;self.ready=False;self.mode='ALIGNING'
                    self.preview(points);self.publish('预建地图已显示：点选初始位置并拖动朝向，确认后允许导航')
                    return MapSessionResponse(True,'地图预览就绪，请设置初始位姿',False)
                if req.action not in ('load','save','new','online'):raise ValueError('未知地图会话操作')
                if req.action=='load':
                    if self.initialized and rospy.has_param('/drone/simulation_spawn_request'):
                        raise ValueError('仿真出生位姿已固定；更改初始位姿需要重启并重新设置')
                    if not self.prepared or self.prepared[0]!=req.path:raise ValueError('请先打开此地图预览')
                    st=Path(req.path).stat()
                    if (req.path,st.st_size,st.st_mtime_ns)!=self.prepared:raise ValueError('地图文件发生变化，请重新预览')
                    if not all(math.isfinite(v) for v in [req.x,req.y,req.z,req.yaw_deg]) or max(abs(req.x),abs(req.y))>30 or abs(req.z)>5 or abs(req.yaw_deg)>180:raise ValueError('初始位姿超出允许范围')
                    old=(self.translation.copy(),self.yaw,self.initialized)
                    self.set_alignment([req.x,req.y,req.z],math.radians(req.yaw_deg))
                    offset=-rotation(-self.yaw)@self.translation
                    request=MapArchiveRequest(action='load',path=req.path,offset_x=offset[0],offset_y=offset[1],offset_z=offset[2],yaw_deg=math.degrees(-self.yaw),use_map_frame=True,clip_to_navigation_grid=bool(self.startup_map and self.startup_loading))
                    try:reply=self.archive(request)
                    except Exception:
                        self.translation,self.yaw,self.initialized=old;raise
                    if not reply.success:
                        self.translation,self.yaw,self.initialized=old
                        return MapSessionResponse(False,reply.message,self.ready)
                    self.ready=True;self.mode='PRIOR_NAV';self.prepared=None;self.preview([])
                    self.set_map_context(req.path,(st.st_size,st.st_mtime_ns))
                    self.publish('预建地图已对齐；初始位姿确认，导航已开放')
                    return MapSessionResponse(True,reply.message,True)
                if req.action=='save' and not self.ready:raise ValueError('初始位姿尚未确认，不能保存')
                request=MapArchiveRequest(action=req.action,path=req.path,use_map_frame=True,offset_x=self.translation[0],offset_y=self.translation[1],offset_z=self.translation[2],yaw_deg=math.degrees(self.yaw))
                reply=self.archive(request)
                if not reply.success:return MapSessionResponse(False,reply.message,self.ready)
                if req.action in ('new','online'):
                    self.set_alignment([0.,0.,0.],0.);self.ready=True;self.mode='ONLINE';self.prepared=None;self.preview([])
                    self.map_context={}
                self.publish(reply.message)
                return MapSessionResponse(True,reply.message,self.ready)
            except Exception as exc:
                rospy.logwarn('Map session request failed: %s',exc)
                return MapSessionResponse(False,str(exc),self.ready)


if __name__=='__main__':
    rospy.init_node('drone_map_session')
    MapSessionManager()
    rospy.spin()
