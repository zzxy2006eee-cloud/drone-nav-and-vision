#!/usr/bin/env python3
"""Static-map downward inspection. Never arms or takes off."""
import copy
from collections import deque
import json
import math
from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace
import uuid
import sys

# catkin's devel relay executes this source with a separate globals dictionary;
# helper imports must resolve to real modules beside the source, not relays.
sys.path.insert(0,str(Path(__file__).resolve().parent))

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String, Header
from std_srvs.srv import Trigger
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker, MarkerArray
from drone_stack.srv import (PlanInspection, PlanInspectionResponse, InspectionCommand,
    InspectionCommandResponse, ExecuteInspectionRoute, ExecuteInspectionRouteRequest, InspectionRegions, InspectionRegionsResponse)
from inspection_region_store import RegionStore, validate_region
from inspection_geometry import (inside, validate_polygon, strips, orient_strips, raster,
    footprint, automatic_spacing, line_points, split_free, forward_projection, route_window)
from safe_global_path import SafeGlobalPath, ReferenceSafetyCells
from voxel_map_client import VoxelMapClient


ACTIVE={'TRANSITING','SCANNING','TURNING','REPLANNING'}


class InspectionManager:
    def __init__(self):
        self.lock=threading.RLock();self.state='IDLE';self.plan=None;self.pending=None
        self.serial=0;self.task_id=0;self.progress=0.;self.previous_position=None
        self.phase='';self.health='UNKNOWN';self.map_mode='';self.aligned=False
        self.transform=None;self.odom=None;self.image=None;self.poses=deque(maxlen=300)
        self.camera_info=None
        self.map_cells=None;self.free=None;self.map_stamp=rospy.Time(0);self.epoch=0
        self.flight_error='';self.nav_stage='';self.health_reason=''
        self.error_wall=0.;self.started_wall=0.
        self.loaded_map_context={}
        self.ai_session='';self.ai_control=False;self.ai_seen_wall=0.;self.ai_instance='';self.ai_counter=-1
        self.ai_request='';self.ai_task_active=False
        self.issues={};self.record_failed=False;self.record_dir=None;self.last_frame=-1.
        self.started=rospy.Time(0);self.last_reference_wall=0.;self.last_status_wall=0.
        self.last_coverage_wall=0.;self.coverage_dirty=True
        self.bridge=CvBridge();self.io_queue=queue.Queue(maxsize=32)
        self.camera=rospy.get_param('~camera');self.surface=float(rospy.get_param('~surface_z_map',0.))
        self.res=float(rospy.get_param('~coverage_resolution_m',.1))
        self.horizon=float(rospy.get_param('~reference_horizon_m',12.))
        self.timeout=float(rospy.get_param('~planning_timeout_wall_s',30.))
        self.camera_timeout=float(rospy.get_param('~camera_timeout_s',.5))
        self.keyframe_hz=float(rospy.get_param('~keyframe_hz',1.))
        self.completion_ratio=float(rospy.get_param('~completion_ratio',.99))
        self.validate_config()
        root=Path(__file__).resolve().parents[3]
        record=Path(rospy.get_param('~record_root','start/inspection_records'))
        self.record_root=record if record.is_absolute() else root/record
        region_root=Path(rospy.get_param('~region_store_root','start/inspection_regions'))
        self.region_root=region_root if region_root.is_absolute() else root/region_root
        self.origin=[-15.,-15.,.5];self.map_res=rospy.get_param('/ego_planner_node/grid_map/resolution',.1)
        self.shape=[math.ceil(rospy.get_param('/ego_planner_node/grid_map/map_size_'+a,s)/self.map_res) for a,s in zip('xyz',[30.,30.,8.])]
        self.origin[:2]=[-self.shape[0]*self.map_res/2,-self.shape[1]*self.map_res/2]
        self.origin[2]=rospy.get_param('/ego_planner_node/grid_map/ground_height',.5)
        self.voxels=VoxelMapClient(self.origin,self.map_res,self.shape,self.on_map,self.invalidate_map)
        self.status_pub=rospy.Publisher('/drone/inspection/status',String,queue_size=1,latch=True)
        self.exclusive_pub=rospy.Publisher('/drone/inspection/exclusive',Bool,queue_size=1,latch=True)
        self.preview_pub=rospy.Publisher('/drone/inspection/markers',MarkerArray,queue_size=1,latch=True)
        self.coverage_pub=rospy.Publisher('/drone/inspection/coverage',MarkerArray,queue_size=1,latch=True)
        self.execute=rospy.ServiceProxy('/drone/execute_inspection_route',ExecuteInspectionRoute)
        self.hold=rospy.ServiceProxy('/drone/hold',Trigger)
        rospy.Subscriber('/mavros/local_position/odom',Odometry,self.on_odom,queue_size=1)
        rospy.Subscriber('/drone/map_transform',PoseStamped,self.on_transform,queue_size=1)
        rospy.Subscriber('/drone/map_session_status',String,self.on_map_session,queue_size=1)
        rospy.Subscriber('/drone/ai/status',String,self.on_ai_status,queue_size=1)
        rospy.Subscriber('/drone/map_alignment_ready',Bool,lambda m:setattr(self,'aligned',m.data),queue_size=1)
        for topic,key in [('flight_state','phase'),('flight_health','health'),('map_mode','map_mode'),
                          ('navigation_stage','nav_stage')]:
            rospy.Subscriber('/drone/'+topic,String,lambda m,k=key:setattr(self,k,m.data),queue_size=1)
        rospy.Subscriber('/drone/flight_error',String,self.on_flight_error,queue_size=1)
        rospy.Subscriber('/drone/flight_health_snapshot',String,self.on_health,queue_size=1)
        rospy.Subscriber('/drone/down/image_raw',Image,self.on_image,queue_size=1,buff_size=4*1024*1024)
        rospy.Subscriber('/drone/down/camera_info',CameraInfo,lambda m:setattr(self,'camera_info',m),queue_size=1)
        rospy.Service('/drone/inspection/plan',PlanInspection,self.plan_request)
        rospy.Service('/drone/inspection/command',InspectionCommand,self.command)
        rospy.Service('/drone/inspection/regions',InspectionRegions,self.regions)
        threading.Thread(target=self.writer,daemon=True).start()
        threading.Thread(target=self.loop,daemon=True).start()

    def validate_config(self):
        values=[self.surface,self.res,self.horizon,self.timeout,self.camera_timeout,self.keyframe_hz,self.completion_ratio]
        values += [float(self.camera[k]) for k in ('width','height','fx','fy','cx','cy')]
        if not all(math.isfinite(v) for v in values) or not (.05<=self.res<=.25 and 6<=self.horizon<=20 and
                1<=self.timeout<=120 and .1<=self.camera_timeout<=2 and .1<=self.keyframe_hz<=5 and .9<=self.completion_ratio<=1):
            raise ValueError('巡检配置参数非法')
        if min(float(self.camera[k]) for k in ('width','height','fx','fy'))<=0:raise ValueError('相机内参非法')
        self.optical=np.asarray(self.camera['optical_rotation_body'],dtype=float).reshape(3,3)
        self.camera_offset=np.asarray(self.camera['xyz_body'],dtype=float)
        if self.camera_offset.shape!=(3,) or not np.isfinite(self.camera_offset).all() or not np.allclose(self.optical.T@self.optical,np.eye(3),atol=1e-5) or np.linalg.det(self.optical)<.99:
            raise ValueError('相机安装外参非法')

    def on_health(self,m):
        try:self.health_reason=json.loads(m.data).get('reason','')
        except ValueError:pass

    def on_map_session(self,m):
        try:
            data=json.loads(m.data)
            with self.lock:self.loaded_map_context=data.get('map_context',{}) if data.get('mode')=='PRIOR_NAV' else {}
        except (ValueError,TypeError):pass

    def on_ai_status(self,m):
        try:
            data=json.loads(m.data);instance=data['instance_id'];counter=int(data['heartbeat_seq'])
            if instance==self.ai_instance and counter<=self.ai_counter:return
            self.ai_instance=instance;self.ai_counter=counter;self.ai_session=data.get('session_id','')
            self.ai_request=data.get('request_id','');self.ai_task_active=bool(data.get('busy') and data.get('state') in ('PARSING','EXECUTING'))
            self.ai_control=bool(data.get('control_authorized') and data.get('client_alive'));self.ai_seen_wall=time.monotonic()
        except (ValueError,KeyError,TypeError):pass

    def owner_valid(self,owner):
        if not owner:return True
        parts=owner.split('|',1)
        return bool(parts[0]=='ai:'+self.ai_session and len(parts)==2 and parts[1]==self.ai_request and
                    self.ai_control and self.ai_task_active and time.monotonic()-self.ai_seen_wall<=3.)

    def regions(self,req):
        try:
            with self.lock:
                if not self.static_ready():raise ValueError('已知区域仅支持已加载并对齐的预建地图')
                context=copy.deepcopy(self.loaded_map_context)
                if not context.get('digest'):raise ValueError('地图标识尚未就绪，请重新加载地图')
                source=Path(context['path']);stat=source.stat()
                if stat.st_size!=context['size'] or stat.st_mtime_ns!=context['mtime_ns']:
                    raise ValueError('预建地图文件已变化，请重新加载后使用区域库')
                data=json.loads(req.payload or '{}')
                if not isinstance(data,dict):raise ValueError('区域请求格式错误')
                if data.get('map_digest',context['digest'])!=context['digest']:raise ValueError('地图已切换，请刷新已知区域')
                if req.action not in ('list','save','update','delete'):raise ValueError('未知区域操作')
                if req.action!='list' and self.state in ACTIVE|{'PAUSED','PLANNING'}:
                    raise ValueError('执行/暂停期间禁止修改区域库，请先取消任务')
                if req.action in ('save','update'):
                    clean=validate_region(data)
                    points=np.column_stack([clean['polygon'],np.full(len(clean['polygon']),clean['altitude'])])
                    planner,_=self.planner(self.map_cells)
                    if any(not all(lo<=v<=hi for v,lo,hi in zip(p,planner.lower,planner.upper)) for p in self.to_odom(points)):
                        raise ValueError('区域或高度超出当前导航范围/天花板')
                store=RegionStore(self.region_root,context['digest'],context['path'])
            records=store.read() if req.action=='list' else store.change(req.action,data)
            with self.lock:
                if self.loaded_map_context.get('digest')!=context['digest']:
                    raise ValueError('地图已切换，请重新读取区域列表')
                self.resolve('REGIONS')
            return InspectionRegionsResponse(True,'已知区域列表已更新' if req.action=='list' else '已知区域操作完成',
                json.dumps(dict(map_digest=context['digest'],regions=records),ensure_ascii=False))
        except Exception as exc:
            self.issue('REGIONS','WARNING',str(exc),'核对地图、区域名称和保存目录后重试')
            return InspectionRegionsResponse(False,str(exc),'')

    def on_flight_error(self,m):
        self.flight_error=m.data;self.error_wall=time.monotonic()

    def on_transform(self,m):
        q=m.pose.orientation;p=m.pose.position
        r=quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
        with self.lock:self.transform=(r,np.array([p.x,p.y,p.z]))

    def on_odom(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        with self.lock:
            self.odom=m
            self.poses.append((m.header.stamp.to_sec(),np.array([p.x,p.y,p.z]),quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]))

    def on_image(self,m):
        with self.lock:self.image=m

    def on_map(self,occupied,free,stamp,epoch,revision):
        with self.lock:self.map_cells=occupied;self.free=free;self.map_stamp=stamp;self.epoch=epoch

    def invalidate_map(self):
        with self.lock:self.map_cells=None;self.free=None

    def static_ready(self):
        age=(rospy.Time.now()-self.map_stamp).to_sec()
        return self.map_mode=='PRIOR_NAV' and self.aligned and self.transform is not None and self.map_cells is not None and -.02<=age<=2.

    def map_position(self):
        p=self.odom.pose.pose.position;r,t=self.transform
        return r@np.array([p.x,p.y,p.z])+t

    def to_odom(self,points,transform=None):
        r,t=transform or self.transform
        return (np.asarray(points)-t)@r

    def issue(self,code,level,reason,action):
        with self.lock:
            old=self.issues.get(code);now=rospy.Time.now().to_sec()
            self.issues[code]=dict(code=code,level=level,reason=reason,action=action,
                first_time=old['first_time'] if old else now,last_time=now,count=old['count']+1 if old else 1,
                detail=(self.flight_error or self.health_reason) if code=='FLIGHT' else '')
            if old is None or any(old.get(k)!=self.issues[code][k] for k in ('reason','level','action')):
                self.record('event',self.issues[code])

    def resolve(self,code):
        with self.lock:
            old=self.issues.pop(code,None)
            if old:self.record('event',dict(old,resolved=True,resolved_time=rospy.Time.now().to_sec()))

    def record(self,kind,data):
        if self.record_dir is None or self.record_failed:return
        try:self.io_queue.put_nowait((self.record_dir,kind,data))
        except queue.Full:
            self.record_failed=True
            self.issue('RECORD','WARNING','记录队列已满，停止新增记录','飞行保护保持；检查磁盘与写入负载')

    def writer(self):
        while not rospy.is_shutdown():
            try:directory,kind,data=self.io_queue.get(timeout=.5)
            except queue.Empty:continue
            try:
                directory.mkdir(parents=True,exist_ok=True)
                if kind=='frame':
                    image,metadata=data
                    filename='down_%019d.jpg'%metadata['stamp_ns']
                    encoded=self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
                    if not cv2.imwrite(str(directory/filename),encoded,[cv2.IMWRITE_JPEG_QUALITY,85]):raise OSError('图像写入失败')
                    metadata=dict(metadata,image=filename)
                    with (directory/'frames.jsonl').open('a') as f:f.write(json.dumps(metadata,ensure_ascii=False)+'\n')
                elif kind in ('plan','summary'):
                    temporary=directory/(kind+'.json.tmp')
                    temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2));temporary.replace(directory/(kind+'.json'))
                else:
                    with (directory/(kind+'.jsonl')).open('a') as f:f.write(json.dumps(data,ensure_ascii=False)+'\n')
            except Exception as exc:
                with self.lock:
                    self.record_failed=True
                    self.issue('RECORD','WARNING','记录失败：'+str(exc),'停止新增记录；检查磁盘，飞行保护不变')
            finally:self.io_queue.task_done()

    def planner(self,occupied):
        p=SafeGlobalPath.__new__(SafeGlobalPath)
        p.origin=self.origin;p.res=self.map_res;p.shape=self.shape
        p.geometry=SimpleNamespace(map_origin=p.origin,map_resolution=p.res)
        p.lower=[-8.,-8.,.5];p.upper=[8.,8.,float(rospy.get_param('/drone/max_flight_height_m',2.5))-.2]
        p.step=.25;p.max_search_nodes=15000
        cells=(ReferenceSafetyCells(occupied),None)
        return p,cells

    def cancelled(self,serial,deadline):
        if serial!=self.serial or rospy.is_shutdown():raise RuntimeError('规划已取消')
        if time.monotonic()>deadline:raise RuntimeError('巡检规划超时，请减小区域或增加间距')

    def connector(self,p,cells,start,end,d0,d1,serial,deadline):
        if math.dist(start,end)<.03:return [tuple(start),tuple(end)]
        if d0 is not None:
            scale=max(.2,min(1.,math.dist(start,end)*.8))
            controls=[np.array(start),np.array(start)+d0*scale,np.array(end)-d1*scale,np.array(end)]
            arc=p.bezier(controls,cells)
            if arc:return arc
        result=None
        for candidate in p.astar_search(start,end,cells):
            self.cancelled(serial,deadline)
            if candidate is not None:result=candidate;break
            time.sleep(.001)
        return p.smooth_route(result,cells) if result else None

    def build_route(self,plan,start,occupied,serial,deadline=None):
        deadline=deadline or time.monotonic()+self.timeout;p,cells=self.planner(occupied)
        route=[tuple(start)];sections=[];kept=[];missed=[];direction=None
        for ident,(a,b) in enumerate(plan['lines_odom']):
            self.cancelled(serial,deadline)
            # Keep previously scanned strips out of a resumed/repaired task.
            if ident in plan.get('finished_lines',set()):continue
            fraction=plan.get('line_progress',{}).get(ident,0.)
            a=np.asarray(a)+(np.asarray(b)-a)*fraction
            parts=split_free(a[:2],b[:2],a[2],lambda u,v:p.segment_safe(u,v,cells))
            for part,(u,v) in enumerate(parts):
                tangent=np.array(v)-u;tangent/=np.linalg.norm(tangent)
                joining=self.connector(p,cells,route[-1],u,direction,tangent,serial,deadline)
                if not joining:missed.append(ident);continue
                sections.append(dict(kind='TRANSITING' if not kept else 'TURNING',line=ident,start=len(route)-1,end=len(route)+len(joining)-2))
                route.extend(joining[1:]);index=len(route)-1
                route.extend(line_points(u,v)[1:])
                sections.append(dict(kind='SCANNING',line=ident,start=index,end=len(route)-1))
                kept.append(ident);direction=tangent
            if not parts:missed.append(ident)
        if len(route)<2 or not kept:raise ValueError('没有可达扫描条带，请调整区域、高度或入口')
        if len(route)>100000:raise ValueError('路线过长，请增加条带间距')
        return route,sections,sorted(set(missed)),set(kept)

    def plan_request(self,req):
        with self.lock:
            try:
                if self.state in ACTIVE or self.state in ('PAUSED','PLANNING'):raise ValueError('请先取消当前巡检任务')
                if not self.static_ready() or self.odom is None:raise ValueError('巡检仅支持已对齐的预建地图，等待地图与位置就绪')
                if not self.owner_valid(req.owner):raise ValueError('AI巡检授权或会话连接失效')
                if req.region.header.frame_id!='map':raise ValueError('巡检区域必须在地图坐标系中')
                poly,area=validate_polygon([(p.x,p.y) for p in req.region.polygon.points])
                if not all(math.isfinite(x) for x in [req.altitude,req.spacing,req.overlap,req.speed,req.angle_deg]):raise ValueError('巡检参数包含非法数值')
                if not .1<=req.speed<=1 or not 0<=req.overlap<=.8 or not 0<=req.entry<=4:raise ValueError('巡检速度、重叠率或入口选择非法')
                if req.angle_deg!=-1 and not 0<=req.angle_deg<180:raise ValueError('方向需为-1自动或0～179度')
                auto,width=automatic_spacing(req.altitude,self.camera,self.surface,req.overlap)
                spacing=auto if req.auto_spacing else req.spacing
                if not .1<=spacing<=5:raise ValueError('条带间距需为0.1～5m')
                xyz=np.column_stack([poly,np.full(len(poly),req.altitude)])
                transformed=self.to_odom(xyz)
                planner,_=self.planner(self.map_cells)
                if any(not all(lo<=v<=hi for v,lo,hi in zip(point,planner.lower,planner.upper)) for point in transformed):raise ValueError('区域或巡检高度超出控制边界/天花板')
                xy,mask,_,_=raster(poly,self.res)
                self.serial+=1;serial=self.serial;self.state='PLANNING'
                self.issues={};self.record_dir=None;self.record_failed=False
                plan=dict(id=uuid.uuid4().hex[:12],polygon=poly,area=area,altitude=req.altitude,
                    spacing=spacing,overlap=req.overlap,speed=req.speed,auto_spacing=req.auto_spacing,width=width,
                    angle=req.angle_deg,entry=req.entry,xy=xy,region_mask=mask,covered=np.zeros(len(xy),dtype=bool),
                    transform=copy.deepcopy(self.transform),epoch=self.epoch,finished_lines=set(),owner=req.owner)
                self.pending=plan;self.plan=None
                occupied=self.map_cells;start=self.to_odom([self.map_position()])[0]
                start[2]=max(planner.lower[2],min(planner.upper[2],start[2]))
                threading.Thread(target=self.plan_worker,args=(plan,start,occupied,serial),daemon=True).start()
                return PlanInspectionResponse(True,'正在后台生成巡检预览',plan['id'])
            except Exception as exc:
                self.issue('PLAN','ERROR',str(exc),'调整区域或参数后重新生成')
                return PlanInspectionResponse(False,str(exc),'')

    def plan_worker(self,plan,start,occupied,serial):
        try:
            deadline=time.monotonic()+self.timeout
            angles=[math.radians(plan['angle'])] if plan['angle']>=0 else [math.radians(a) for a in range(0,180,15)]
            candidates=[]
            position=self.map_position()[:2]
            for angle in angles:
                lines=strips(plan['polygon'],plan['spacing'],angle)
                for choice in ([plan['entry']-1] if plan['entry'] else range(4)):
                    ordered=orient_strips(lines,choice)
                    if not ordered:continue
                    cost=sum(np.linalg.norm(b-a) for a,b in ordered)+np.linalg.norm(ordered[0][0]-position)
                    cost+=sum(np.linalg.norm(b[0]-a[1]) for a,b in zip(ordered,ordered[1:]))
                    candidates.append((cost,angle,choice,ordered))
            if not candidates:raise ValueError('区域不能生成有效扫描线')
            planner,cells=self.planner(occupied);selected=None
            # Compare reachable entrances, without running every full mission search.
            for _,angle,choice,lines in sorted(candidates,key=lambda v:v[0])[:8]:
                self.cancelled(serial,deadline)
                candidate=copy.copy(plan)
                candidate['lines_map']=[(tuple((*a,plan['altitude'])),tuple((*b,plan['altitude']))) for a,b in lines]
                candidate['lines_odom']=[tuple(self.to_odom([a,b],plan['transform'])) for a,b in candidate['lines_map']]
                try:
                    route,sections,missed,kept=self.build_route(candidate,start,occupied,serial,deadline)
                    selected=(candidate,route,sections,missed,kept,angle,choice);break
                except ValueError:continue
            if selected is None:raise ValueError('所有候选入口均不可达')
            plan,route,sections,missed,kept,angle,choice=selected
            xyz=self.to_odom(np.column_stack([plan['xy'],np.full(len(plan['xy']),plan['altitude'])]),plan['transform'])
            eligible=plan['region_mask'].copy()
            for i in np.flatnonzero(eligible):
                if i%64==0:self.cancelled(serial,deadline)
                if not planner.segment_safe(xyz[i],xyz[i],cells):eligible[i]=False
            if not eligible.any():raise ValueError('区域全部被安全障碍层排除')
            plan.update(route=route,sections=sections,missed=missed,kept=kept,eligible=eligible,
                        selected_angle=math.degrees(angle),selected_entry=choice+1)
            with self.lock:
                if serial!=self.serial:return
                self.plan=plan;self.pending=None;self.state='READY';self.progress=0.
                self.coverage_dirty=True
                self.resolve('PLAN');self.publish_preview()
        except Exception as exc:
            with self.lock:
                if serial==self.serial:
                    self.state='FAILED';self.pending=None
                    self.issue('PLAN','ERROR',str(exc),'缩小区域、调整高度/间距/入口后重新生成')

    def command(self,req):
        with self.lock:
            try:
                if req.action=='cancel':
                    current=(self.plan or self.pending or {}).get('id','')
                    if req.plan_id and req.plan_id!=current:raise ValueError('巡检计划已被替换，没有取消其它任务')
                    if req.owner and req.owner!=(self.plan or self.pending or {}).get('owner',''):
                        raise ValueError('巡检已由其它操作接管，没有取消该任务')
                    self.serial+=1
                    if self.state in ACTIVE:self.request_hold()
                    self.state='CANCELED';self.task_id=0;self.summary();self.clear_markers()
                    for code in list(self.issues):
                        if code not in ('RECORD','HOLD') and not (code=='FLIGHT' and self.phase in ('LANDING','DESCENDING','FAILSAFE')):
                            self.resolve(code)
                    return InspectionCommandResponse(True,'巡检已取消；飞行状态以顶部保护提示为准')
                if req.action=='pause':
                    if self.state not in ACTIVE:raise ValueError('当前没有执行中的巡检')
                    self.pause('MANUAL','INFO','操作员暂停巡检','点击继续恢复剩余扫描')
                    return InspectionCommandResponse(True,'巡检已暂停')
                if req.action not in ('start','resume'):raise ValueError('未知巡检操作')
                if self.plan is None or (req.plan_id and req.plan_id!=self.plan['id']):raise ValueError('巡检计划已失效，请重新生成')
                if self.state!=('READY' if req.action=='start' else 'PAUSED'):raise ValueError('当前巡检阶段不支持该操作')
                if not self.static_ready() or self.epoch!=self.plan['epoch'] or not self.same_transform():raise ValueError('地图或初始位姿已变化，请取消后重新生成')
                if self.phase!='HOLD' or self.health!='HEALTHY':raise ValueError('请先手动起飞并进入健康 HOLD')
                if not self.owner_valid(req.owner):raise ValueError('AI巡检执行授权或会话已失效')
                if not self.camera_valid():raise ValueError(self.camera_error())
                self.serial+=1;serial=self.serial;self.state='REPLANNING'
                self.plan['owner']=req.owner
                self.exclusive_pub.publish(Bool(True))
                self.resolve('COMMAND');self.resolve('CAMERA');self.resolve('FLIGHT');self.resolve('MANUAL');self.resolve('BLOCKED')
                self.resolve('AI')
                if self.record_dir is None:
                    self.record_dir=self.record_root/(time.strftime('%Y%m%d_%H%M%S')+'_'+self.plan['id'])
                    self.record('plan',self.plan_json());self.last_frame=-1.
                self.task_id=0
                start=self.to_odom([self.map_position()])[0]
                threading.Thread(target=self.execution_worker,args=(start,self.map_cells,serial),daemon=True).start()
                return InspectionCommandResponse(True,'正在从当前悬停位置准备巡检路线')
            except Exception as exc:
                self.issue('COMMAND','WARNING',str(exc),'核对任务、地图、相机和悬停状态后重试')
                return InspectionCommandResponse(False,str(exc))

    def same_transform(self):
        return self.transform is not None and all(np.allclose(a,b,atol=1e-5) for a,b in zip(self.transform,self.plan['transform']))

    def execution_worker(self,start,occupied,serial):
        try:
            route,sections,missed,kept=self.build_route(self.plan,start,occupied,serial)
            with self.lock:
                if serial!=self.serial:return
                self.plan.update(route=route,sections=sections,missed=sorted(set(self.plan['missed'])|set(missed)),kept=kept)
                ends={}
                for section in sections:
                    if section['kind']=='SCANNING':ends[section['line']]=max(ends.get(section['line'],0),section['end'])
                self.plan['line_ends']=ends
                self.progress=0.;self.previous_position=None;self.started=rospy.Time.now()
                self.started_wall=time.monotonic()
                self.publish_preview()
                projection,arc=forward_projection(start,route,0.,.35)
                window,continuous=route_window(route,projection,arc,self.horizon)
                reply=self.submit(window,continuous,0)
                if not reply.success:raise ValueError(reply.message)
                self.task_id=reply.task_id;self.state='TRANSITING';self.last_reference_wall=time.monotonic()
                self.plan['ever_executed']=True
                for code in ('ROUTE','BLOCKED','DEVIATION','INTERNAL'):self.resolve(code)
                self.record('route',dict(time=rospy.Time.now().to_sec(),points=route,sections=sections,missed=missed))
        except Exception as exc:
            with self.lock:
                if serial==self.serial:
                    self.task_id=0
                    if isinstance(exc,ValueError) and '没有可达扫描条带' in str(exc) and self.plan.get('ever_executed'):
                        self.request_hold();self.state='COMPLETED_PARTIAL'
                        self.issue('PARTIAL','WARNING','剩余扫描条带无法安全连接，巡检部分完成','查看漏扫区域并调整任务');self.summary()
                    else:
                        self.state='PAUSED'
                        self.issue('ROUTE','ERROR',str(exc),'当前保持HOLD；调整后继续或取消')

    def path_message(self,points):
        msg=RosPath(header=Header(stamp=rospy.Time.now(),frame_id='odom'))
        for xyz in points:
            pose=PoseStamped(header=msg.header);pose.pose.orientation.w=1.;pose.pose.position=Point(*map(float,xyz));msg.poses.append(pose)
        return msg

    def submit(self,points,continuous,task):
        return self.execute(ExecuteInspectionRouteRequest(route=self.path_message(points),speed=self.plan['speed'],task_id=task,continuous=continuous))

    def request_hold(self):
        try:
            reply=self.hold()
            if not reply.success:self.issue('HOLD','ERROR',reply.message,'飞行管理器保护仍负责降落；核对飞控状态')
            else:self.resolve('HOLD')
        except rospy.ServiceException as exc:self.issue('HOLD','ERROR','HOLD服务异常：'+str(exc),'检查管理器和飞控，必要时使用一键降落')

    def pause(self,code,level,reason,action):
        self.serial+=1;self.request_hold();self.task_id=0;self.state='PAUSED'
        self.issue(code,level,reason,action);self.summary()

    def flight_reason(self):
        raw=self.flight_error
        for fragment,message in [('Inspection manager heartbeat','巡检管理器心跳超时，导航已取消并悬停'),
                ('heartbeat','EGO心跳超时，导航已取消并悬停'),
                ('map became stale','障碍或空闲地图过期，导航已取消并悬停'),
                ('inside a known inflated obstacle','目标进入障碍膨胀层，导航已取消'),
                ('OFFBOARD','飞控退出OFFBOARD，飞行保护已触发'),
                ('Health degraded','定位或飞控估计持续异常，已进入HOLD'),
                ('Severe health','定位或飞控估计严重异常，已触发降落'),
                ('ACK timed out','局部规划启动确认超时，导航已取消')]:
            if fragment in raw:return message
        return '飞行管理器进入保护状态，请查看顶部状态及异常详情'

    def camera_valid(self):
        return not self.camera_error()

    def camera_error(self):
        m=self.image
        if m is None:return '未收到下视相机图像'
        age=(rospy.Time.now()-m.header.stamp).to_sec()
        if not -.02<=age<=self.camera_timeout:return '下视图像时间过期或超前（允许年龄%.2fs）'%self.camera_timeout
        if m.width!=int(self.camera['width']) or m.height!=int(self.camera['height']) or not m.data:return '下视图像尺寸与配置不符或图像为空'
        info=self.camera_info
        if info is None:return '未收到下视相机CameraInfo，请检查相机话题'
        calibrated=(info is not None and info.width==m.width and info.height==m.height and
                    all(math.isfinite(v) for v in info.K) and
                    abs(info.K[0]-float(self.camera['fx']))<=max(1.,float(self.camera['fx'])*.01) and
                    abs(info.K[4]-float(self.camera['fy']))<=max(1.,float(self.camera['fy'])*.01) and
                    abs(info.K[2]-float(self.camera['cx']))<=2. and abs(info.K[5]-float(self.camera['cy']))<=2.)
        return '' if calibrated else '下视CameraInfo内参与配置不符，请核对inspection.yaml'

    def capture(self,line):
        image=self.image;stamp=image.header.stamp.to_sec()
        if stamp<=getattr(self,'last_coverage_stamp',-1.):return
        pose=min(self.poses,key=lambda v:abs(v[0]-stamp)) if self.poses else None
        if pose is None or abs(pose[0]-stamp)>.15:return
        r,t=self.transform;_,position,rotation=pose
        map_body=r@position+t;map_rotation=r@rotation
        camera_position=map_body+map_rotation@self.camera_offset
        intrinsics=[float(self.camera[k]) for k in ('fx','fy','cx','cy','width','height')]
        visible=footprint(camera_position,map_rotation@self.optical,intrinsics,self.surface)
        if visible is None:return
        observed=inside(self.plan['xy'],visible)&self.plan['eligible']
        self.coverage_dirty |= bool(np.any(observed&~self.plan['covered']))
        self.plan['covered'] |= observed
        self.last_coverage_stamp=stamp
        if stamp-self.last_frame>=1./self.keyframe_hz:
            self.record('frame',(image,dict(stamp_ns=image.header.stamp.to_nsec(),pose_map=map_body.tolist(),
                rotation_map=map_rotation.tolist(),camera_footprint=visible.tolist(),line=line,state=self.state)))
            self.last_frame=stamp

    def loop(self):
        while not rospy.is_shutdown():
            try:
                with self.lock:self.tick()
            except Exception as exc:
                with self.lock:
                    if self.state in ACTIVE:self.pause('INTERNAL','ERROR','巡检管理异常：'+str(exc),'保持悬停；查看日志后继续或取消')
                    else:self.issue('INTERNAL','ERROR',str(exc),'查看巡检管理器日志')
                rospy.logerr_throttle(2.,'Inspection manager: %s',exc)
            time.sleep(.2)

    def tick(self):
        now=rospy.Time.now();wall=time.monotonic()
        if self.state in ACTIVE:
            if self.phase in ('LANDING','DESCENDING','FAILSAFE','DISCONNECTED'):
                self.serial+=1;self.state='FAILED';self.task_id=0
                if self.phase in ('LANDING','DESCENDING') and self.flight_error=='Operator requested landing' and self.health!='SEVERE':
                    self.state='CANCELED';self.issue('OPERATOR','INFO','操作员请求降落，巡检结束','继续执行降落，不恢复巡检')
                else:self.issue('FLIGHT','ERROR',self.flight_reason(),'保护降落/接管；不自动恢复')
                self.summary()
            elif not self.owner_valid(self.plan.get('owner','')):
                self.pause('AI','WARNING','AI执行授权或控制连接失效，巡检已暂停','可在巡检页手动继续或取消')
            elif not self.static_ready() or not self.same_transform() or self.epoch!=self.plan['epoch']:
                self.pause('MAP','ERROR','预建地图失效或坐标对齐改变','取消任务，重新对齐地图并生成')
            elif self.health in ('HOLD','SEVERE','UNKNOWN') or (self.task_id and self.phase=='HOLD' and (now-self.started).to_sec()>1.):
                end=self.to_odom([self.map_position()])[0]
                final=self.plan['route'][-1]
                if self.phase=='HOLD' and self.health=='HEALTHY' and math.dist(end,final)<.20 and self.progress>sum(math.dist(a,b) for a,b in zip(self.plan['route'],self.plan['route'][1:]))-.20:
                    self.finish()
                elif self.phase=='HOLD' and self.health=='HEALTHY' and self.error_wall<self.started_wall:
                    self.pause('MANUAL','INFO','操作员或外部控制进入HOLD，巡检已暂停','点击继续或取消')
                else:self.pause('FLIGHT','WARNING',self.flight_reason(),'健康恢复后点击继续')
            elif not self.camera_valid():
                self.pause('CAMERA','WARNING',self.camera_error(),'核对相机配置与图像；恢复后点击继续')
            elif self.task_id:
                self.resolve('CAMERA');self.resolve('FLIGHT')
                if self.health=='WARNING':self.issue('HEALTH','WARNING','飞行健康短暂下降','沿用现有1秒HOLD/2秒降落保护；暂停累计覆盖')
                else:self.resolve('HEALTH')
                position=self.to_odom([self.map_position()])[0]
                advance=max(.35,math.dist(position,self.previous_position)+.15) if self.previous_position is not None else .35
                projection,arc=forward_projection(position,self.plan['route'],self.progress,advance)
                if projection is None or projection[0]>.75:
                    self.pause('DEVIATION','WARNING','机体偏离巡检路线超过0.75m','悬停后从当前位置继续或取消')
                else:
                    self.progress=max(self.progress,float(projection[1]));self.previous_position=position
                    index=projection[2];section=next((s for s in self.plan['sections'] if s['start']<=index<s['end']),self.plan['sections'][-1])
                    self.state='TURNING' if self.nav_stage=='TURNING' else section['kind']
                    for line,end in self.plan['line_ends'].items():
                        if index>=end:self.plan['finished_lines'].add(line)
                    if self.state=='SCANNING':
                        a,b=map(np.asarray,self.plan['lines_odom'][section['line']]);d=b-a
                        along=float(np.clip(np.dot(position-a,d)/max(1e-9,np.dot(d,d)),0.,1.))
                        history=self.plan.setdefault('line_progress',{})
                        history[section['line']]=max(history.get(section['line'],0.),along)
                    if self.state=='SCANNING' and self.health=='HEALTHY':self.capture(section['line'])
                    self.record('trajectory',dict(time=now.to_sec(),position_map=self.map_position().tolist(),progress=self.progress,state=self.state))
                    if wall-self.last_reference_wall>=.5:
                        window,continuous=route_window(self.plan['route'],projection,arc,self.horizon)
                        planner,cells=self.planner(self.map_cells)
                        if len(window)>=2 and not planner.route_safe(window,cells):
                            self.pause('BLOCKED','WARNING','新障碍阻挡扫描路线，正在绕行并标记漏扫','重新连接后续可扫描条带')
                            self.state='REPLANNING';self.serial+=1;serial=self.serial
                            threading.Thread(target=self.execution_worker,args=(position,self.map_cells,serial),daemon=True).start()
                        elif len(window)>=2:
                            reply=self.submit(window,continuous,self.task_id)
                            if not reply.success:self.pause('ROUTE','WARNING',reply.message,'核对地图和健康后继续')
                            self.last_reference_wall=wall
        if self.state=='PAUSED' and self.camera_valid():self.resolve('CAMERA')
        if self.state=='READY' and self.static_ready():
            if self.camera_valid():self.resolve('CAMERA')
            else:self.issue('CAMERA','WARNING',self.camera_error(),'核对下视相机与配置后再开始')
        if wall-self.last_status_wall>=.5:
            self.publish_status()
            if wall-self.last_coverage_wall>=1.:
                self.publish_coverage();self.last_coverage_wall=wall
            self.last_status_wall=wall

    def metrics(self):
        if self.plan is None:return dict(covered_m2=0.,eligible_m2=0.,excluded_m2=0.,ratio=0.)
        p=self.plan;n=int(p['eligible'].sum());covered=int((p['covered']&p['eligible']).sum())
        return dict(covered_m2=covered*self.res**2,eligible_m2=n*self.res**2,
                    excluded_m2=int((p['region_mask']&~p['eligible']).sum())*self.res**2,ratio=covered/max(1,n))

    def finish(self):
        self.task_id=0;metrics=self.metrics()
        partial=bool(self.plan['missed']) or metrics['ratio']<self.completion_ratio
        self.state='COMPLETED_PARTIAL' if partial else 'COMPLETED'
        if partial:self.issue('PARTIAL','WARNING','巡检部分完成，仍有受阻或未覆盖区域','查看覆盖图和记录；重新圈选剩余区域')
        self.summary()

    def plan_json(self):
        p=self.plan
        return dict(plan_id=p['id'],polygon=p['polygon'].tolist(),altitude=p['altitude'],spacing=p['spacing'],
            overlap=p['overlap'],speed=p['speed'],auto_spacing=p['auto_spacing'],camera=self.camera,surface_z_map=self.surface,
            selected_angle=p['selected_angle'],selected_entry=p['selected_entry'],lines_map=p['lines_map'],
            map_path=self.loaded_map_context.get('path',rospy.get_param('/drone/prebuilt_map_path','')),
            map_digest=self.loaded_map_context.get('digest',''),map_epoch=p['epoch'],
            map_transform=[a.tolist() for a in p['transform']])

    def summary(self):
        if self.plan is not None:self.record('summary',dict(plan_id=self.plan['id'],state=self.state,metrics=self.metrics(),
            issues=list(self.issues.values()),missed_lines=self.plan['missed'],covered_cells=np.flatnonzero(self.plan['covered']).tolist(),
            coverage_resolution=self.res,coverage_xy=self.plan['xy'].tolist()))

    def publish_status(self):
        self.exclusive_pub.publish(Bool(self.state in ACTIVE or self.state=='PAUSED'))
        p=self.plan;self.status_pub.publish(String(json.dumps(dict(state=self.state,
            plan_id=p['id'] if p else self.pending['id'] if self.pending else '',available=self.static_ready(),
            stamp_ns=rospy.Time.now().to_nsec(),progress_m=self.progress,metrics=self.metrics(),
            issues=list(self.issues.values()),selected_angle=p['selected_angle'] if p else None,
            selected_entry=p['selected_entry'] if p else None,spacing=p['spacing'] if p else None,
            lines=len(p['lines_map']) if p else 0,record_dir=str(self.record_dir or ''),
            map_digest=self.loaded_map_context.get('digest',''),map_path=self.loaded_map_context.get('path','')),ensure_ascii=False)))

    def marker(self,ident,kind,color):
        m=Marker(header=Header(stamp=rospy.Time.now(),frame_id='map'),ns='inspection',id=ident,type=kind,action=Marker.ADD)
        m.pose.orientation.w=1.;m.color.r,m.color.g,m.color.b,m.color.a=color;m.scale.x=.035
        return m

    def publish_preview(self):
        p=self.plan;markers=MarkerArray()
        boundary=self.marker(0,Marker.LINE_STRIP,(.1,.8,1.,1.))
        boundary.points=[Point(x=float(x),y=float(y),z=p['altitude']) for x,y in list(p['polygon'])+[p['polygon'][0]]]
        markers.markers.append(boundary)
        lanes=self.marker(1,Marker.LINE_LIST,(0.,1.,1.,.65))
        for a,b in p['lines_map']:lanes.points.extend([Point(*map(float,a)),Point(*map(float,b))])
        markers.markers.append(lanes)
        route=self.marker(2,Marker.LINE_STRIP,(1.,.5,0.,.6))
        r,t=p['transform'];route.points=[Point(*map(float,r@np.array(v)+t)) for v in p['route']];markers.markers.append(route)
        entry=self.marker(3,Marker.SPHERE,(1.,.4,0.,1.));entry.scale.x=entry.scale.y=entry.scale.z=.15
        a=p['sections'][0]['end'];entry.pose.position=Point(*map(float,r@np.array(p['route'][a])+t));markers.markers.append(entry)
        self.preview_pub.publish(markers)

    def publish_coverage(self):
        if self.plan is None or not self.coverage_dirty:return
        p=self.plan;markers=MarkerArray()
        for ident,mask,color in [(0,p['covered']&p['eligible'],(.1,.9,.2,.4)),
                                  (1,p['eligible']&~p['covered'],(1.,.65,0.,.22)),
                                  (2,p['region_mask']&~p['eligible'],(1.,.1,.1,.35))]:
            m=self.marker(ident,Marker.CUBE_LIST,color);m.ns='inspection_coverage'
            m.scale.x=m.scale.y=self.res;m.scale.z=.01
            m.points=[Point(x=float(x),y=float(y),z=self.surface+.015) for x,y in p['xy'][mask]]
            markers.markers.append(m)
        self.coverage_pub.publish(markers)
        self.coverage_dirty=False

    def clear_markers(self):
        m=Marker(action=Marker.DELETEALL)
        self.preview_pub.publish(MarkerArray(markers=[m]));self.coverage_pub.publish(MarkerArray(markers=[m]))
        self.coverage_dirty=False


if __name__=='__main__':
    rospy.init_node('drone_inspection_manager')
    manager=InspectionManager()
    rospy.spin()
