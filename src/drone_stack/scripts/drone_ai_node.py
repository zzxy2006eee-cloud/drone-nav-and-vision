#!/usr/bin/env python3
"""Text-only DeepSeek plan agent; sequential existing ROS services, no shell."""
import copy
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Point32
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, ExtendedState
from std_msgs.msg import String, Bool, Header
from std_srvs.srv import SetBool
from tf.transformations import quaternion_matrix
from drone_stack.srv import (AiText,AiTextResponse,AiGate,AiGateResponse,AiCancel,AiCancelResponse,
    ExecuteAiAction,ExecuteAiActionRequest,ExecuteAiGoal,ExecuteAiGoalRequest,CancelNavigation,
    PlanInspection,PlanInspectionRequest,InspectionCommand,InspectionCommandRequest,
    InspectionRegions,InspectionRegionsRequest)
from drone_ai_contract import request_plan,redact,service_reason


class Cancelled(Exception):pass


class AiNode:
    def __init__(self):
        self.lock=threading.RLock();self.dispatch_lock=threading.RLock()
        self.instance=uuid.uuid4().hex;self.counter=0;self.session='';self.parse=False;self.control=False
        self.clients={};self.epoch=0;self.busy=False;self.state='IDLE';self.answer='';self.error=''
        self.request_id='';self.plan={'steps':[]};self.step=-1;self.owned=None
        self.phase='';self.health='UNKNOWN';self.flight_authorized=False;self.manager_wall=0.
        self.phase_wall=0.
        self.odom=None;self.transform=None;self.fcu=State();self.extended=ExtendedState();self.inspection={};self.exclusive=False
        self.base_url=str(rospy.get_param('~base_url','https://api.deepseek.com'))
        if self.base_url.rstrip('/') not in ('https://api.deepseek.com','https://api.deepseek.com/v1'):raise ValueError('AI服务地址仅允许官方DeepSeek HTTPS端点')
        self.model=str(rospy.get_param('~model','deepseek-flash'))
        self.network_timeout=float(rospy.get_param('~request_timeout_wall_s',30.))
        self.sim_timeout=float(rospy.get_param('~step_timeout_sim_s',300.));self.wall_timeout=float(rospy.get_param('~step_timeout_wall_s',900.))
        self.client_timeout=float(rospy.get_param('~client_timeout_wall_s',3.))
        if not all(math.isfinite(v) for v in [self.network_timeout,self.sim_timeout,self.wall_timeout,self.client_timeout]) or not (
                5<=self.network_timeout<=120 and 30<=self.sim_timeout<=900 and 60<=self.wall_timeout<=3600 and 1<=self.client_timeout<=3):
            raise ValueError('AI超时配置非法')
        root=Path(__file__).resolve().parents[3];self.log_root=root/'start/logs/ai'
        path=Path(rospy.get_param('~api_key_file','start/private/deepseek.local'))
        path=path if path.is_absolute() else root/path
        try:self.key=os.environ.get('DEEPSEEK_API_KEY','') or json.loads(path.read_text()).get('api_key','')
        except (OSError,ValueError,AttributeError):self.key=''
        if not isinstance(self.key,str) or '\n' in self.key or '\r' in self.key:self.key=''
        self.key=self.key.strip()
        self.status_pub=rospy.Publisher('/drone/ai/status',String,queue_size=1,latch=True)
        self.event_pub=rospy.Publisher('/drone/ai/events',String,queue_size=20)
        rospy.Subscriber('/drone/ai/client_heartbeat',String,self.heartbeat,queue_size=1)
        rospy.Subscriber('/mavros/state',State,lambda m:setattr(self,'fcu',m),queue_size=1)
        rospy.Subscriber('/mavros/extended_state',ExtendedState,lambda m:setattr(self,'extended',m),queue_size=1)
        rospy.Subscriber('/mavros/local_position/odom',Odometry,lambda m:setattr(self,'odom',m),queue_size=1)
        rospy.Subscriber('/drone/map_transform',PoseStamped,self.map_transform,queue_size=1)
        rospy.Subscriber('/drone/manager_heartbeat',Header,lambda m:setattr(self,'manager_wall',time.monotonic()),queue_size=1)
        rospy.Subscriber('/drone/inspection/status',String,self.inspect_status,queue_size=1)
        rospy.Subscriber('/drone/inspection/exclusive',Bool,lambda m:setattr(self,'exclusive',m.data),queue_size=1)
        rospy.Subscriber('/drone/flight_state',String,self.on_phase,queue_size=1)
        for topic,key in [('flight_health','health')]:
            rospy.Subscriber('/drone/'+topic,String,lambda m,k=key:setattr(self,k,m.data),queue_size=1)
        rospy.Subscriber('/drone/authorized',Bool,lambda m:setattr(self,'flight_authorized',m.data),queue_size=1)
        self.flight=rospy.ServiceProxy('/drone/execute_ai_action',ExecuteAiAction)
        self.navigate=rospy.ServiceProxy('/drone/execute_ai_goal',ExecuteAiGoal)
        self.cancel_nav=rospy.ServiceProxy('/drone/cancel_navigation',CancelNavigation)
        self.region_service=rospy.ServiceProxy('/drone/inspection/regions',InspectionRegions)
        self.plan_inspection=rospy.ServiceProxy('/drone/inspection/plan',PlanInspection)
        self.inspect_command=rospy.ServiceProxy('/drone/inspection/command',InspectionCommand)
        self.set_flight_auth=rospy.ServiceProxy('/drone/set_authorized',SetBool)
        rospy.Service('/drone/ai/set_gate',AiGate,self.gate)
        rospy.Service('/drone/ai/submit_text',AiText,self.submit)
        rospy.Service('/drone/ai/cancel',AiCancel,self.cancel)
        threading.Thread(target=self.monitor,daemon=True).start()

    def heartbeat(self,m):
        try:
            data=json.loads(m.data);session=data['session_id']
            if not isinstance(session,str) or not 1<=len(session)<=64:return
            with self.lock:
                self.clients[session]=(time.monotonic(),data)
                if len(self.clients)>32:self.clients={k:v for k,v in self.clients.items() if time.monotonic()-v[0]<10}
        except (ValueError,KeyError,TypeError):pass

    def alive(self,session=None):
        return time.monotonic()-self.clients.get(session or self.session,(0.,{}))[0]<=self.client_timeout

    def map_transform(self,m):
        q=m.pose.orientation;p=m.pose.position
        self.transform=(quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3],np.array([p.x,p.y,p.z]))

    def on_phase(self,m):
        self.phase=m.data;self.phase_wall=time.monotonic()

    def inspect_status(self,m):
        try:self.inspection=json.loads(m.data)
        except ValueError:pass

    def event(self,message):
        record=dict(time=time.time(),request_id=self.request_id,state=self.state,step=self.step,message=redact(message))
        self.event_pub.publish(String(json.dumps(record,ensure_ascii=False)))
        try:
            self.log_root.mkdir(parents=True,exist_ok=True)
            with (self.log_root/'events.jsonl').open('a') as f:f.write(json.dumps(record,ensure_ascii=False)+'\n')
        except OSError:self.error='AI事件记录写入失败；任务状态仍在界面显示'

    def publish(self):
        with self.lock:
            self.counter+=1
            self.status_pub.publish(String(json.dumps(dict(instance_id=self.instance,heartbeat_seq=self.counter,
                session_id=self.session,client_alive=self.alive(),parse_authorized=self.parse,control_authorized=self.control,
                configured=bool(self.key),model=self.model,state=self.state,busy=self.busy,request_id=self.request_id,
                step=self.step,steps=self.plan.get('steps',[]),answer=self.answer,error=redact(self.error),
                client_sessions=[k for k in self.clients if self.alive(k)]),ensure_ascii=False)))

    def gate(self,req):
        stop=False
        with self.lock:
            if req.gate not in ('parse','control'):return AiGateResponse(False,'未知AI授权类型')
            if not self.alive(req.session_id):return AiGateResponse(False,'Qt会话心跳未就绪')
            if self.session and self.session!=req.session_id and self.alive() and (self.parse or self.control):
                return AiGateResponse(False,'另一Qt会话持有AI授权')
            self.session=req.session_id
            if req.gate=='parse':
                self.parse=req.enabled
                if not req.enabled:self.control=False;stop=True
            else:
                if req.enabled and not self.parse:return AiGateResponse(False,'请先允许云端解析')
                self.control=req.enabled;stop=not req.enabled
            if stop:
                stopped_epoch=self.epoch
                self.epoch+=1
                if self.busy:self.state='CANCELED';self.event('AI授权已撤销，停止后续步骤')
        self.publish()
        if stop:threading.Thread(target=self.cleanup,args=(stopped_epoch,),daemon=True).start()
        return AiGateResponse(True,'AI授权已更新，飞行授权与原安全门独立保留')

    def cancel(self,req):
        with self.lock:
            if req.session_id!=self.session:return AiCancelResponse(False,'不是当前AI控制会话')
            stopped_epoch=self.epoch;self.epoch+=1;self.state='CANCELED';self.error='';self.event('操作员停止AI任务')
        threading.Thread(target=self.cleanup,args=(stopped_epoch,),daemon=True).start();self.publish()
        return AiCancelResponse(True,'停止后续AI步骤；当前降落继续执行')

    def valid(self,epoch,execute=False):
        if rospy.is_shutdown() or epoch!=self.epoch or not self.parse or not self.alive() or (execute and not self.control):
            raise Cancelled('AI任务已停止或授权/Qt连接失效')

    def context(self):
        regions=[]
        if self.inspection.get('available'):
            reply=self.region_service(InspectionRegionsRequest(action='list',payload='{}'))
            if reply.success:regions=json.loads(reply.payload).get('regions',[])
        position=self.map_position() if self.odom is not None and self.transform is not None else None
        ui=self.clients.get(self.session,(0.,{}))[1]
        context=dict(phase=self.phase,health=self.health,armed=self.fcu.armed,flight_authorized=self.flight_authorized,
            map_position=position.tolist() if position is not None else None,takeoff_height_m=ui.get('takeoff_height_m',1.),
            map_digest=self.inspection.get('map_digest',''),
            regions=[dict(id=r['id'],name=r['name'],altitude=r['altitude']) for r in regions])
        return context,regions

    def submit(self,req):
        with self.lock:
            if req.session_id!=self.session or not self.parse or not self.alive():return AiTextResponse(False,'','请先允许当前Qt会话的云端解析')
            if self.busy:return AiTextResponse(False,'','已有AI任务；请先停止或等待完成')
            text=req.text.strip()
            if not text or len(text)>4000:return AiTextResponse(False,'','文字需为1～4000字符')
            if not self.key:return AiTextResponse(False,'','DeepSeek密钥未配置，请检查本机私有文件')
            self.busy=True;self.state='PARSING';self.error='';self.answer='';self.plan={'steps':[]};self.step=-1
            self.request_id=uuid.uuid4().hex[:12];self.epoch+=1;epoch=self.epoch;allowed=self.control
            self.event('用户文字：'+redact(text))
            self.publish()
            threading.Thread(target=self.worker,args=(text,epoch,allowed),daemon=True).start()
            return AiTextResponse(True,self.request_id,'文字已提交，正在解析完整计划')

    def worker(self,text,epoch,allowed):
        partial=False
        try:
            context,regions=self.context();self.valid(epoch)
            plan=request_plan(self.base_url,self.model,self.key,text,context,self.network_timeout)
            with self.lock:
                self.valid(epoch);self.plan=plan;self.answer=plan['answer'];self.event('计划已校验：'+json.dumps(plan['steps'],ensure_ascii=False))
                if not plan['steps']:self.state='NEEDS_INPUT';return
                if not allowed:self.state='PARSED';self.event('仅解析，未授权执行');return
            for index,step in enumerate(plan['steps']):
                self.valid(epoch,True)
                if step['tool'] not in ('status','land','hold','disarm') and self.inspection.get('map_digest','')!=context.get('map_digest',''):
                    raise ValueError('地图在AI解析后发生变化，请重新提交文字')
                with self.lock:
                    self.valid(epoch,True);self.step=index;self.state='EXECUTING';self.event('执行：'+step['description']);self.publish()
                partial |= self.execute_step(step,regions,epoch)
                with self.lock:
                    self.valid(epoch,True)
                    if self.owned is not None and self.owned[2]==epoch:self.owned=None
                    self.event('该步骤已由实际状态确认结束')
            with self.lock:
                self.valid(epoch,True);self.state='COMPLETED_PARTIAL' if partial else 'SUCCEEDED';self.event('AI计划结束')
        except Cancelled:
            if epoch==self.epoch:self.state='CANCELED'
            self.cleanup(epoch)
        except Exception as exc:
            if epoch==self.epoch:
                self.state='FAILED';self.error=service_reason(str(exc));self.event('AI任务停止：'+self.error)
            self.cleanup(epoch)
        finally:
            with self.lock:self.busy=False
            self.publish()

    def map_position(self):
        p=self.odom.pose.pose.position;r,t=self.transform
        return r@np.array([p.x,p.y,p.z])+t

    def owner_tag(self):
        return 'ai:'+self.session+'|'+self.request_id

    def wait(self,predicate,epoch,timeout=None):
        started=time.monotonic();sim=rospy.Time.now().to_sec()
        while True:
            self.valid(epoch,True)
            if predicate():return
            if self.owned is not None and self.owned[0] in ('nav','inspection','takeoff'):
                if not self.fcu.armed:raise ValueError('飞控已锁定，停止后续AI步骤')
                if self.phase in ('LANDING','DESCENDING'):raise ValueError('已进入降落，停止后续AI步骤')
            if time.monotonic()-started>self.wall_timeout or rospy.Time.now().to_sec()-sim>(timeout or self.sim_timeout):
                raise ValueError('等待实际任务结果超时，停止后续步骤')
            if self.phase in ('FAILSAFE','DISCONNECTED') or self.health=='SEVERE':raise ValueError('飞行严重异常，停止AI后续步骤')
            if time.monotonic()-self.manager_wall>3.:raise ValueError('飞行管理器心跳失效')
            time.sleep(.2)

    def dispatch(self,epoch,function):
        with self.dispatch_lock:
            self.valid(epoch,True)
            return function()

    def execute_step(self,step,regions,epoch):
        tool=step['tool'];args=step['arguments']
        if tool=='status':
            self.event('当前状态：%s，健康%s，解锁%s'%(self.phase,self.health,self.fcu.armed));return False
        if not self.fcu.connected or time.monotonic()-self.manager_wall>3.:raise ValueError('飞控或飞行管理器连接失效，未提交动作')
        if tool in ('arm','takeoff','land','hold','disarm'):
            if tool in ('arm','takeoff') and not self.flight_authorized:raise ValueError('请先在综合页授权飞行；AI授权不会代替飞行授权')
            if tool=='arm' and self.fcu.armed:return False
            if tool in ('land','disarm') and not self.fcu.armed and self.extended.landed_state==ExtendedState.LANDED_STATE_ON_GROUND:
                self.event('机体已落地且锁定，无需重复执行');return False
            def command():
                reply=self.flight(ExecuteAiActionRequest(session_id=self.session,request_id=self.request_id,action=tool,height_m=args.get('height_m',0.)))
                if not reply.success:raise ValueError(reply.message)
                self.owned=(tool,None,epoch,self.request_id);return reply
            self.dispatch(epoch,command)
            if tool=='arm':self.wait(lambda:self.fcu.armed and self.phase=='ARMED',epoch,90.)
            elif tool=='takeoff':self.wait(lambda:self.phase=='HOLD',epoch,90.)
            elif tool=='land':self.wait(lambda:self.extended.landed_state==ExtendedState.LANDED_STATE_ON_GROUND,epoch,120.)
            elif tool=='hold':self.wait(lambda:self.phase=='HOLD',epoch,30.)
            elif tool=='disarm':self.wait(lambda:not self.fcu.armed,epoch,30.)
            return False
        if tool in ('navigate_map','navigate_relative'):
            if self.phase!='HOLD' or self.exclusive:raise ValueError('点到点导航需要空闲HOLD；请先暂停/取消当前任务')
            if self.odom is None or self.transform is None:raise ValueError('缺少位置或地图对齐')
            if tool=='navigate_map':
                target=np.array([args['x'],args['y'],args.get('z',self.map_position()[2])]);r,t=self.transform;xyz=r.T@(target-t)
            else:
                q=self.odom.pose.pose.orientation;rotation=quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
                yaw=math.atan2(rotation[1,0],rotation[0,0]);c,s=math.cos(yaw),math.sin(yaw)
                f,l,u=[args.get(k,0.) for k in ('forward','left','up')];p=self.odom.pose.pose.position
                xyz=np.array([p.x+c*f-s*l,p.y+s*f+c*l,p.z+u])
            goal=PoseStamped();goal.header.frame_id='odom';goal.header.stamp=rospy.Time.now()
            goal.pose.orientation.w=1.;goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=xyz
            def command():
                reply=self.navigate(ExecuteAiGoalRequest(session_id=self.session,request_id=self.request_id,goal=goal))
                if not reply.success:raise ValueError(reply.message)
                self.owned=('nav',reply.task_id,epoch)
            before=self.phase_wall
            self.dispatch(epoch,command)
            def arrived():
                if self.phase=='NAVIGATING':return False
                if self.phase!='HOLD':return False
                p=self.odom.pose.pose.position;v=self.odom.twist.twist.linear
                if math.dist([p.x,p.y,p.z],xyz)>.15 or math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)>.15:
                    raise ValueError('导航已停止但未到达目标；停止后续步骤')
                return True
            self.wait(lambda:self.phase_wall>before and self.phase in ('NAVIGATING','HOLD'),epoch,30.)
            self.wait(arrived,epoch);return False
        if tool=='inspect_region':
            if self.phase!='HOLD' or self.exclusive:raise ValueError('开始巡检需要空闲健康HOLD')
            fresh=self.region_service(InspectionRegionsRequest(action='list',payload='{}'))
            if not fresh.success:raise ValueError(fresh.message)
            current=json.loads(fresh.payload).get('regions',[])
            expected=next((r for r in regions if args['region'] in (r['id'],r['name'])),None)
            region=next((r for r in current if expected and r['id']==expected['id']),None)
            if region is None:raise ValueError('已知区域已删除或地图切换，请重新提交文字')
            keys=('updated_at','name','polygon','altitude','spacing','overlap','speed','auto_spacing','angle_deg','entry')
            if any(region.get(k)!=expected.get(k) for k in keys):raise ValueError('已知区域在解析后被修改，请重新提交文字')
            req=PlanInspectionRequest();req.region.header.frame_id='map';req.region.header.stamp=rospy.Time.now()
            for x,y in region['polygon']:req.region.polygon.points.append(Point32(x=x,y=y,z=region['altitude']))
            for name in ('altitude','auto_spacing','spacing','overlap','speed','angle_deg','entry'):setattr(req,name,region[name])
            req.owner=self.owner_tag()
            def planning():
                reply=self.plan_inspection(req)
                if not reply.success:raise ValueError(reply.message)
                self.owned=('inspection',reply.plan_id,epoch,self.owner_tag());return reply.plan_id
            ident=self.dispatch(epoch,planning)
            def planned():
                if self.inspection.get('plan_id')!=ident:return False
                if self.inspection.get('state')=='FAILED':raise ValueError('巡检路线生成失败，查看巡检页异常')
                return self.inspection.get('state')=='READY'
            self.wait(planned,epoch,90.)
            self.inspection_control('start',ident,epoch)
            self.wait_inspection(ident,epoch)
            return self.inspection.get('state')=='COMPLETED_PARTIAL'
        ident=self.inspection.get('plan_id','')
        if not ident:raise ValueError('没有当前巡检任务')
        action={'pause_inspection':'pause','resume_inspection':'resume','cancel_inspection':'cancel'}[tool]
        before=self.inspection.get('stamp_ns',0)
        self.inspection_control(action,ident,epoch)
        if action=='resume':
            self.owned=('inspection',ident,epoch,self.owner_tag())
            self.wait(lambda:self.inspection.get('stamp_ns',0)>before,epoch,30.)
            self.wait_inspection(ident,epoch)
        return False

    def inspection_control(self,action,ident,epoch):
        def command():
            reply=self.inspect_command(InspectionCommandRequest(action=action,plan_id=ident,
                owner=self.owner_tag() if action in ('start','resume') else ''))
            if not reply.success:raise ValueError(reply.message)
        self.dispatch(epoch,command)

    def wait_inspection(self,ident,epoch):
        def complete():
            if self.inspection.get('plan_id')!=ident:raise ValueError('巡检计划被其它操作替换')
            state=self.inspection.get('state')
            if state in ('FAILED','CANCELED'):raise ValueError('巡检失败或已取消，停止后续步骤')
            if state=='PAUSED':raise ValueError('巡检已暂停；保留进度，请在巡检页查看原因并继续')
            return state in ('COMPLETED','COMPLETED_PARTIAL') and not self.exclusive and self.phase=='HOLD'
        self.wait(complete,epoch)

    def cleanup(self,epoch=None):
        with self.dispatch_lock:
            owned=self.owned
            if owned is None:return
            kind,ident,owner_epoch=owned[:3]
            if epoch is not None and epoch!=owner_epoch:return
            self.owned=None
            try:
                if kind=='nav':self.cancel_nav(ident)
                elif kind=='inspection':
                    # A fault-paused inspection retains its progress for the operator.
                    if self.inspection.get('state')!='PAUSED':self.inspect_command(InspectionCommandRequest(action='cancel',plan_id=ident,owner=owned[3]))
                elif kind in ('arm','takeoff'):
                    if not self.fcu.armed:self.set_flight_auth(False)
                    elif kind=='arm' and self.extended.landed_state==ExtendedState.LANDED_STATE_ON_GROUND:
                        self.flight(ExecuteAiActionRequest(session_id=self.session,request_id=owned[3],action='disarm'))
                    elif self.phase not in ('LANDING','DESCENDING'):self.flight(ExecuteAiActionRequest(session_id=self.session,request_id=owned[3],action='hold'))
            except rospy.ServiceException:
                self.control=False;self.error='AI停止请求通信失败，已撤销执行授权；飞行管理器独立保护仍生效'

    def monitor(self):
        while not rospy.is_shutdown():
            expired=False
            with self.lock:
                if (self.parse or self.control) and not self.alive():
                    stopped_epoch=self.epoch;self.parse=False;self.control=False;self.epoch+=1;self.state='CANCELED';self.error='Qt会话心跳失效，AI授权已撤销';expired=True
            self.publish()
            if expired:threading.Thread(target=self.cleanup,args=(stopped_epoch,),daemon=True).start()
            time.sleep(.5)


if __name__=='__main__':
    rospy.init_node('drone_ai')
    agent=AiNode();rospy.spin()
