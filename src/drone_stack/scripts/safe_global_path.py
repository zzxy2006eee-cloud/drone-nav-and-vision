#!/usr/bin/env python3
"""Authoritative, collision-checked smooth global ENU reference for EGO.

The same dense curve goes to EGO and Qt. Progress advances monotonically on
an immutable route version; map changes trigger a replacement from the vehicle.
Unknown space is allowed in the reference; execution checks observed free space.
"""
import heapq, math, threading, time, json
import os, sys
from types import SimpleNamespace
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float64
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ego_planner.msg import Bspline, GlobalRoute
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trajectory_guard import PackedCells, validate_curve, curve_geometry
from voxel_map_client import VoxelMapClient
from flight_manager import FlightManager

class ReferenceSafetyCells:
    """Conservative grid padding for EGO's existing 5 cm curve margin.

    At 10 cm voxel resolution, one neighbour cell guarantees that margin.
    Query lazily; never expand/convert the whole map again each scan.
    """
    def __init__(self,occupied):
        self.occupied=occupied;self.cache={}
    def __contains__(self,cell):
        cell=tuple(cell)
        if cell not in self.cache:
            x,y,z=cell
            self.cache[cell]=any((x+dx,y+dy,z+dz) in self.occupied
                                 for dx in (-1,0,1) for dy in (-1,0,1) for dz in (-1,0,1))
        return self.cache[cell]

class SafeGlobalPath:
    def __init__(self):
        self.lock=threading.RLock(); self.goal=None; self.task_sequence=0; self.odom=None; self.cells=None; self.free=None; self.free_stamp=rospy.Time(0); self.free_wall=0
        self.stage='WAITING'; self.curve_epoch=rospy.Time(0); self.search=None; self.search_generation=-1
        self.search_start=None; self.search_prefix=None; self.path_complete=False
        self.route_curve_id=None; self.last_publish_wall=0.; self.retry_wall=0.
        self.reference_route=[]; self.reference_progress=0.; self.progress_pose=None; self.route_version=0
        self.phase=''; self.curve=None; self.path=[]; self.map_stamp=rospy.Time(0); self.map_wall=0
        self.generation=0; self.last_clock=0; self.clock_wall=time.monotonic(); self.clock_fault=False
        self.origin=[-rospy.get_param('/ego_planner_node/grid_map/map_size_x',30.)/2,
                     -rospy.get_param('/ego_planner_node/grid_map/map_size_y',30.)/2,
                     rospy.get_param('/ego_planner_node/grid_map/ground_height',.5)]
        self.res=rospy.get_param('/ego_planner_node/grid_map/resolution',.1)
        self.shape=[math.ceil(rospy.get_param('/ego_planner_node/grid_map/map_size_'+a,s)/self.res) for a,s in zip('xyz',[30.,30.,8.])]
        self.lower=[-8.,-8.,.5]; self.upper=[8.,8.,float(rospy.get_param('/drone/max_flight_height_m',2.5))-.20]; self.step=.25
        self.max_search_nodes=int(rospy.get_param("~max_search_nodes",15000))
        if not 1000 <= self.max_search_nodes <= 60000:raise ValueError("Invalid global search node limit")
        self.geometry=SimpleNamespace(map_origin=self.origin,map_resolution=self.res)
        self.inspection_intent=None;self.inspection_applied_revision=-1
        self.pub=rospy.Publisher('/drone/global_path',Path,queue_size=1,latch=True)
        self.progress_pub=rospy.Publisher('/drone/global_route_progress',String,queue_size=1,latch=True)
        self.route_pub=rospy.Publisher('/drone/global_route',GlobalRoute,queue_size=1,latch=True)
        self.marker_pub=rospy.Publisher('/drone/global_path_markers',MarkerArray,queue_size=1,latch=True)
        self.status=rospy.Publisher('/drone/global_path_status',String,queue_size=1,latch=True)
        rospy.Subscriber('/drone/max_flight_height',Float64,self.on_ceiling,queue_size=1)
        rospy.Subscriber('/drone/navigation_goal',PoseStamped,self.on_goal,queue_size=1)
        rospy.Subscriber('/drone/inspection_route',GlobalRoute,self.on_inspection_route,queue_size=1)
        rospy.Subscriber('/drone/flight_state',String,self.on_phase,queue_size=5)
        rospy.Subscriber('/drone/navigation_stage',String,self.on_stage,queue_size=1)
        # The bridge exposes FCU numeric ENU coordinates as odom, exactly like
        # cloud_fcu_world. MAVROS's original local_position topic is labelled map.
        rospy.Subscriber('/drone/fcu/odom',Odometry,self.on_odom,queue_size=1,tcp_nodelay=True)
        self.voxel_client=VoxelMapClient(self.origin,self.res,self.shape,self.on_voxel_map,self.invalidate_voxel_map)
        threading.Thread(target=self.run,daemon=True).start()

    def on_ceiling(self,message):
        if not math.isfinite(message.data) or not .8<=message.data<=2.5:return
        with self.lock:
            height=message.data-.20
            if abs(self.upper[2]-height)<1e-8:return
            self.upper[2]=height;self.generation+=1;self.search=None
            self.reference_route=[];self.reference_progress=0.;self.progress_pose=None
            self.route_version+=1
            if self.goal is not None:self.clear('最高飞行高度已更新，重新计算受限路线')

    def clear(self,reason):
        self.path=[]; self.path_complete=False; self.publish([],reason)

    def publish(self,points,reason):
        msg=Path();msg.header.frame_id='odom';msg.header.stamp=rospy.Time.now()
        for v in points:
            p=PoseStamped();p.header=msg.header;p.pose.orientation.w=1
            p.pose.position.x,p.pose.position.y,p.pose.position.z=v;msg.poses.append(p)
        self.pub.publish(msg);self.status.publish(String(data=reason));self.last_publish_wall=time.monotonic()
        self.progress_pub.publish(String(data=json.dumps(dict(task_id=self.task_sequence,
            version=self.route_version,arc_m=self.reference_progress,remaining_points=len(points),
            stamp=msg.header.stamp.to_sec()))))
        intent=self.inspection_intent
        inspect=bool(intent is not None and intent.task_id==self.task_sequence)
        continuous=bool(inspect and intent.continuous)
        blocked=bool(inspect and self.goal is not None and
            (not points or math.dist(points[-1],self.goal)>=.02))
        if inspect and not points and self.odom is not None and self.goal is not None:
            p=self.odom.pose.pose.position
            remaining=sum(math.dist(a,b) for a,b in zip(self.reference_route,self.reference_route[1:]))-self.reference_progress
            blocked=not (self.reference_route and remaining<=.20 and math.dist((p.x,p.y,p.z),self.goal)<=.15 and
                         self.segment_safe((p.x,p.y,p.z),self.goal,(self.cells,self.free)))
        self.route_pub.publish(GlobalRoute(task_id=self.task_sequence,path=msg,
            inspection=inspect,continuous=continuous,revision=self.route_version,blocked=blocked))
        markers=MarkerArray()
        for ident in (0,1):
            marker=Marker();marker.header=msg.header;marker.ns='reference_route';marker.id=ident
            marker.type=Marker.LINE_LIST;marker.action=Marker.ADD if points else Marker.DELETE
            marker.pose.orientation.w=1;marker.scale.x=.055
            marker.color.r=1;marker.color.g=1;marker.color.a=1
            markers.markers.append(marker)
        arc=0.0
        for a,b in zip(points,points[1:]):
            length=math.dist(a,b)
            if length<1e-9:continue
            # Split long shortcut edges to show the known/unknown boundary.
            count=max(1,math.ceil(length/.05))
            for i in range(count):
                u=tuple(x+(y-x)*i/count for x,y in zip(a,b))
                v=tuple(x+(y-x)*(i+1)/count for x,y in zip(a,b))
                known=self.free is not None and all(c in self.free for c in FlightManager.segment_cells(self.geometry,u,v))
                # Carry dash phase across edges, including dense spline samples.
                midpoint=arc+length*(i+.5)/count
                if not known and midpoint% .30 >= .18:continue
                marker=markers.markers[0 if known else 1]
                for xyz in (u,v):marker.points.append(Point(x=xyz[0],y=xyz[1],z=xyz[2]+.06))
            arc+=length
        self.marker_pub.publish(markers)

    def on_inspection_route(self,m):
        if m.path.header.frame_id!='odom' or not m.inspection or not 2<=len(m.path.poses)<=2048:return
        with self.lock:self.inspection_intent=m

    def on_goal(self,m):
        if m.header.frame_id!='odom':return
        p=m.pose.position;v=(p.x,p.y,p.z)
        with self.lock:
            self.task_sequence=m.header.stamp.to_nsec()
            self.generation+=1;self.curve=None;self.search=None;self.route_curve_id=None
            self.reference_route=[];self.reference_progress=0.;self.progress_pose=None;self.route_version=0
            self.inspection_applied_revision=-1
            self.cells=None;self.free=None
            self.map_stamp=rospy.Time(0);self.free_stamp=rospy.Time(0)
            self.goal=v if all(math.isfinite(x) for x in v) else None
            self.clear('目标已更新，等待黄色参考路径')

    def on_phase(self,m):
        with self.lock:
            self.phase=m.data
            if self.phase!='NAVIGATING' and self.goal is not None:
                # Manager publishes NAVIGATING before acknowledging the goal;
                # latched old HOLD may arrive during startup, so no new GUI goal
                # can survive a confirmed non-navigation transition.
                self.goal=None;self.generation+=1;self.curve=None;self.search=None
                self.reference_route=[];self.reference_progress=0.;self.progress_pose=None
                self.cells=None;self.free=None
                self.clear('导航已结束')

    def on_stage(self,m):
        with self.lock:
            if m.data==self.stage:return
            self.stage=m.data
            if self.stage!='TRACKING':
                self.curve=None;self.curve_epoch=rospy.Time.now();self.route_curve_id=None
                # Same-task waiting/ACK transitions preserve A* progress.
                # Goal, ceiling and displacement changes invalidate it separately.

    def on_odom(self,m):
        if m.header.frame_id!='odom' or (rospy.Time.now()-m.header.stamp).to_sec()<-.02:return
        with self.lock:self.odom=m

    def on_curve(self,m):
        with self.lock:
            if (self.goal is not None and self.phase=='NAVIGATING' and self.stage=='TRACKING' and
                    m.start_time>=self.curve_epoch):self.curve=m

    def on_voxel_map(self,occupied,free,stamp,epoch,revision):
        with self.lock:
            if self.goal is None or stamp<self.map_stamp:return
            if self.cells is None or self.cells.occupied is not occupied:self.cells=ReferenceSafetyCells(occupied)
            self.free=free;self.map_stamp=stamp;self.free_stamp=stamp
            self.map_wall=time.monotonic();self.free_wall=self.map_wall

    def invalidate_voxel_map(self):
        with self.lock:
            self.cells=None;self.free=None;self.map_stamp=rospy.Time(0);self.free_stamp=rospy.Time(0)

    def segment_safe(self,a,b,cells):
        if not all(math.isfinite(v) and lo-1e-9<=v<=hi+1e-9 for p in (a,b) for v,lo,hi in zip(p,self.lower,self.upper)):return False
        occupied,free=cells
        if occupied is None:return False
        return not any(c in occupied for c in FlightManager.segment_cells(self.geometry,a,b))

    def route_safe(self,points,cells):
        return bool(points) and all(self.segment_safe(a,b,cells) for a,b in zip(points,points[1:])) and self.segment_safe(points[0],points[0],cells)

    def astar(self,start,goal,cells,deadline):
        # Compatibility wrapper; runtime advances the generator in bounded
        # CPU batches, retaining its open set across successive observations.
        for result in self.astar_search(start,goal,cells):
            if result is not None:return result or None
            if time.thread_time()>=deadline:return None
        return None

    def astar_search(self,start,goal,cells):
        batch=time.thread_time()
        if self.segment_safe(start,goal,cells):
            yield [start,goal];return
        def index(p):return tuple(int(math.floor((v-lo)/self.step)) for v,lo in zip(p,self.lower))
        def point(i):return tuple(lo+(v+.5)*self.step for v,lo in zip(i,self.lower))
        dims=tuple(int(round((hi-lo)/self.step)) for lo,hi in zip(self.lower,self.upper))
        root=index(start);queue=[];cost={};parent={}
        # Connect the continuous start to nearby lattice centres safely.
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                for dz in (-1,0,1):
                    i=(root[0]+dx,root[1]+dy,root[2]+dz)
                    if not all(0<=v<n for v,n in zip(i,dims)):continue
                    p=point(i)
                    if self.segment_safe(start,p,cells):
                        c=math.dist(start,p);cost[i]=c;parent[i]=None;heapq.heappush(queue,(c+math.dist(p,goal),c,i))
        expanded=0
        while queue and expanded<self.max_search_nodes:
            if time.thread_time()-batch>=.025:
                yield None
                batch=time.thread_time()
            _,c,i=heapq.heappop(queue)
            if c!=cost.get(i):continue
            p=point(i);expanded+=1
            if math.dist(p,goal)<self.step*2 and self.segment_safe(p,goal,cells):
                result=[goal,p];j=parent[i]
                while j is not None:result.append(point(j));j=parent[j]
                result.append(start);result.reverse()
                # Shortcut only when the entire closed-voxel segment is free.
                reduced=[result[0]];k=0
                while k<len(result)-1:
                    nxt=k+1
                    for j in range(len(result)-1,k,-1):
                        if time.thread_time()-batch>=.025:
                            yield None
                            batch=time.thread_time()
                        if self.segment_safe(result[k],result[j],cells):nxt=j;break
                    reduced.append(result[nxt]);k=nxt
                yield reduced;return
            for axis in range(3):
                for sign in (-1,1):
                    j=list(i);j[axis]+=sign;j=tuple(j)
                    if not all(0<=v<n for v,n in zip(j,dims)):continue
                    nc=c+self.step
                    if nc>=cost.get(j,float('inf')):continue
                    if not self.segment_safe(p,point(j),cells):continue
                    cost[j]=nc;parent[j]=i;heapq.heappush(queue,(nc+math.dist(point(j),goal),nc,j))
        yield []

    def prefix(self,start,curve,cells,now):
        if curve is None:return [start]
        age=now-curve.start_time.to_sec()
        if age<-.02:return [start]
        reason=validate_curve(curve,cells[0],self.origin,self.res,self.lower,self.upper,wall_budget=.5,cpu_budget=.04)
        if reason:return [start]
        try:
            pts=np.array([[p.x,p.y,p.z] for p in curve.pos_pts]);knots=np.array(curve.knots)
            begin=knots[3]+max(0.,age);end=knots[len(pts)]
            if begin>=end:return [start]
            f=curve_geometry(curve)[3];values=f(np.linspace(begin,end,min(256,max(2,int((end-begin)/.1)+1))))
            route=[start]+[tuple(p) for p in values]
            if math.dist(start,route[1])>.75 or not self.route_safe(route,cells):return [start]
            return route
        except (ValueError,TypeError):return [start]

    def safe_prefix(self,route,cells):
        """Retain only the connected prefix passing the latest occupied map."""
        if not route or not self.segment_safe(route[0],route[0],cells):return []
        result=[route[0]]
        for a,b in zip(route,route[1:]):
            if self.segment_safe(a,b,cells):result.append(b);continue
            # Cut a blocked long edge at its last confirmed free point.
            count=max(1,math.ceil(math.dist(a,b)/.05))
            for i in range(1,count+1):
                q=tuple(x+(y-x)*i/count for x,y in zip(a,b))
                if not self.segment_safe(result[-1],q,cells):break
                result.append(q)
            break
        return result

    def bezier(self,controls,cells):
        """Validate the entire cubic, then sample the SAME curve for EGO."""
        curve=SimpleNamespace(order=3,knots=[0.,0.,0.,0.,1.,1.,1.,1.],
                              pos_pts=[SimpleNamespace(x=v[0],y=v[1],z=v[2]) for v in controls])
        if validate_curve(curve,cells[0],self.origin,self.res,self.lower,self.upper,
                          cpu_budget=.025,wall_budget=.15):return None
        c=np.asarray(controls);length=sum(np.linalg.norm(b-a) for a,b in zip(c,c[1:]))
        count=max(8,min(256,int(math.ceil(length/.035))))
        # Tight final joins need angular sampling as well as distance
        # sampling. A short curve must not become an eight-segment zigzag.
        while True:
            t=np.linspace(0.,1.,count+1)[:,None];u=1.-t
            values=u**3*c[0]+3*u*u*t*c[1]+3*u*t*t*c[2]+t**3*c[3]
            d=np.diff(values,axis=0);lengths=np.linalg.norm(d,axis=1)
            if np.any(lengths<1e-9):return None
            directions=d/lengths[:,None]
            endpoints=np.array([c[1]-c[0],c[3]-c[2]])
            norm=np.linalg.norm(endpoints,axis=1)
            if np.any(norm<1e-9):return None
            sequence=np.vstack([endpoints[0]/norm[0],directions,endpoints[1]/norm[1]])
            dot=np.sum(sequence[:-1]*sequence[1:],axis=1)
            if np.all(dot>=math.cos(math.radians(4.))):break
            if count>=1024:return None
            count=min(1024,count*2)
        result=[tuple(v) for v in values]
        return result if self.route_safe(result,cells) else None

    def smooth_route(self,points,cells):
        """Round every bend with tangent-continuous cubic Bezier joins.

        Shrink the corner blend only if the continuous collision guard rejects
        it. Never publish a sharp fallback as an executable smooth reference.
        """
        clean=[]
        for p in points:
            if not clean or math.dist(p,clean[-1])>1e-6:clean.append(tuple(p))
        if len(clean)<2:return None
        result=[clean[0]]
        def line_to(end):
            start=result[-1];length=math.dist(start,end)
            count=max(1,int(math.ceil(length/.04)))
            result.extend(tuple(a+(b-a)*i/count for a,b in zip(start,end)) for i in range(1,count+1))
        for a,b,c in zip(clean,clean[1:],clean[2:]):
            a,b,c=np.asarray(a),np.asarray(b),np.asarray(c)
            incoming=b-a;outgoing=c-b;la=np.linalg.norm(incoming);lb=np.linalg.norm(outgoing)
            incoming/=la;outgoing/=lb;dot=float(np.dot(incoming,outgoing))
            if dot>.99999:continue
            if dot<-.95:return None  # A reversal needs a different spatial route.
            cut=min(.8,.42*la,.42*lb);arc=None
            while cut>=.015:
                entry=b-incoming*cut;exit=b+outgoing*cut
                controls=[entry,entry+(b-entry)*2/3,exit+(b-exit)*2/3,exit]
                arc=self.bezier(controls,cells)
                if arc is not None and self.segment_safe(result[-1],entry,cells):break
                arc=None;cut*=.5
            if arc is None:return None
            line_to(tuple(entry));result.extend(arc[1:])
        line_to(clean[-1])
        return result if len(result)<=2048 and self.route_safe(result,cells) else None

    def project_forward(self,start,route,progress=0.,advance_limit=float('inf')):
        if len(route)<2:return [start],progress
        points=np.asarray(route);lengths=np.linalg.norm(np.diff(points,axis=0),axis=1)
        arc=np.r_[0.,np.cumsum(lengths)];best=float('inf');selected=None
        for i,length in enumerate(lengths):
            if length<1e-9 or arc[i+1]<progress-1e-8 or arc[i]>advance_limit:continue
            d=points[i+1]-points[i]
            lo=max(0.,(progress-arc[i])/length);hi=min(1.,(advance_limit-arc[i])/length)
            if lo>hi:continue
            u=float(np.clip(np.dot(np.asarray(start)-points[i],d)/(length*length),lo,hi))
            q=points[i]+u*d;gap=float(np.linalg.norm(q-start));along=float(arc[i]+u*length)
            # At a shared endpoint choose the FORWARD segment, not the old
            # segment pointing back to a waypoint already passed.
            if gap<best-1e-8 or (abs(gap-best)<=1e-8 and (selected is None or along>=selected[0])):
                best=gap;selected=(along,i,tuple(q))
        if selected is None or best>.75:return [start],progress
        along,i,q=selected;tail=[q]+list(route[i+1:])
        while len(tail)>1 and math.dist(tail[0],tail[1])<1e-6:tail.pop(0)
        return tail,max(progress,along)

    def connect_reference(self,start,tail,cells):
        if len(tail)<2:return tail
        if math.dist(start,tail[0])<=.02:return tail
        # Smoothly rejoin ahead, rather than inserting a sharp vehicle-to-old-
        # waypoint chord. This connector is part of the authoritative reference.
        pts=np.asarray(tail);arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(pts,axis=0),axis=1))]
        tangent=pts[1]-pts[0];tangent/=np.linalg.norm(tangent)
        for distance in (.35,.6,1.):
            i=min(len(pts)-1,max(1,int(np.searchsorted(arc,distance))))
            end=pts[i];last=pts[i+1]-pts[i] if i+1<len(pts) else pts[i]-pts[i-1];last/=np.linalg.norm(last)
            scale=float(np.linalg.norm(end-start))/3
            controls=[np.asarray(start),np.asarray(start)+tangent*scale,end-last*scale,end]
            connector=self.bezier(controls,cells)
            if connector is not None:
                joined=connector+tail[i+1:]
                return joined
        return []

    def reanchor(self,start,route):
        # Compatibility helper; runtime also supplies persistent arc progress.
        tail,_=self.project_forward(start,route)
        return tail

    def run(self):
        while not rospy.is_shutdown():
            time.sleep(.1)
            now=rospy.Time.now().to_sec();wall=time.monotonic()
            with self.lock:
                if now<self.last_clock-.02:self.clock_fault=True
                if now>self.last_clock:self.clock_wall=wall
                self.last_clock=now
                goal=self.goal;odom=self.odom;cells=(self.cells,self.free);generation=self.generation
                if goal is None:continue
                if (self.clock_fault or wall-self.clock_wall>10 or self.cells is None or self.free is None or odom is None or
                    not -.02<=now-self.map_stamp.to_sec()<=2 or wall-self.map_wall>10 or
                    not -.02<=now-self.free_stamp.to_sec()<=2 or wall-self.free_wall>10 or
                    not -.02<=now-odom.header.stamp.to_sec()<=.7):
                    self.search=None;self.clear('地图、时钟或位姿失效，参考路径不可用');continue
                p=odom.pose.pose.position;start=(p.x,p.y,p.z)
                intent=self.inspection_intent
                inspection=bool(intent is not None and intent.task_id==self.task_sequence)
                if inspection:
                    if intent.revision==self.inspection_applied_revision and wall-self.last_publish_wall<.45:continue
                    if not -.02<=now-intent.path.header.stamp.to_sec()<=2.:
                        self.clear('巡检参考已过期，等待巡检管理器');continue
                    if intent.revision!=self.inspection_applied_revision:
                        self.reference_route=[(p.pose.position.x,p.pose.position.y,p.pose.position.z) for p in intent.path.poses]
                        self.reference_progress=0.;self.progress_pose=None
                        self.inspection_applied_revision=intent.revision
                        self.goal=self.reference_route[-1];goal=self.goal
                        self.route_version+=1
                    self.search=None
                reference=self.reference_route;progress=self.reference_progress;previous_pose=self.progress_pose
                complete=self.path_complete
                if not all(math.isfinite(v) for v in start):continue
            limit=progress+max(.35,math.dist(start,previous_pose)+.15) if previous_pose is not None else float('inf')
            tail,new_progress=self.project_forward(start,reference,progress,limit)
            anchored=self.connect_reference(start,tail,cells)
            retained=self.safe_prefix(anchored,cells)
            if inspection:
                with self.lock:
                    if generation!=self.generation:continue
                    self.reference_progress=new_progress;self.progress_pose=start
                    self.path=retained;self.path_complete=len(retained)>=2 and math.dist(retained[-1],goal)<.02
                    self.publish(retained if len(retained)>=2 else [],
                        '巡检条带控制参考' if self.path_complete else '巡检参考受阻，保留安全前段并等待绕行')
                continue
            still_complete=len(retained)>=2 and math.dist(retained[-1],goal)<.01
            with self.lock:
                if generation!=self.generation:continue
                self.reference_progress=new_progress;self.progress_pose=start
                if self.search is not None and (self.search_generation!=generation or math.dist(self.search_start,start)>.5):self.search=None
                if not still_complete and self.search is None and wall>=self.retry_wall:
                    self.search=self.astar_search(start,goal,cells);self.search_generation=generation;self.search_start=start
                search=self.search
            candidate=None
            if search is not None:
                try:result=next(search)
                except StopIteration:result=[]
                if result is not None:
                    with self.lock:
                        still_current=self.search is search
                        if still_current:self.search=None;self.retry_wall=wall+1.
                    if result and still_current:candidate=self.smooth_route(result,cells)
            with self.lock:
                if generation!=self.generation or self.goal is None:continue
                current=rospy.Time.now().to_sec();current_cells=(self.cells,self.free)
                if (self.cells is None or self.free is None or not -.02<=current-self.map_stamp.to_sec()<=2 or
                    not -.02<=current-self.free_stamp.to_sec()<=2):
                    self.clear('最新地图过期，参考路径不可用');continue
                route=self.safe_prefix(retained,current_cells)
                if candidate:
                    tail,candidate_progress=self.project_forward(start,candidate)
                    replacement=self.connect_reference(start,tail,current_cells)
                    if len(replacement)>=2 and self.route_safe(replacement,current_cells) and math.dist(replacement[-1],goal)<.01:
                        self.route_version+=1
                        self.reference_route=candidate;self.reference_progress=candidate_progress;self.progress_pose=start
                        route=replacement
                full=len(route)>=2 and math.dist(route[-1],goal)<.01
                if len(route)<2:route=[]
                lost_suffix=complete and not full
                self.path=route;self.path_complete=full
                if not route:self.clear('正在重规划安全平滑曲线，当前没有可显示的安全前段')
                elif candidate or lost_suffix or wall-self.last_publish_wall>=.5:
                    reason=('黄色平滑曲线：实线已观测自由，虚线待观测；同一曲线输入EGO' if full else
                            '地图出现障碍：保留安全曲线前段，重新规划完整曲线')
                    self.publish(route,reason)

if __name__=='__main__':
    rospy.init_node('drone_safe_global_path');SafeGlobalPath();rospy.spin()
