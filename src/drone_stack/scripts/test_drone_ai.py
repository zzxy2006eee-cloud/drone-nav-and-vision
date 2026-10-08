#!/usr/bin/env python3
import json
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
from drone_ai_contract import validate_plan,request_plan,redact


REGIONS=[dict(id='1'*32,name='设备区')]
def plan(tool,args=None):return dict(steps=[dict(tool=tool,arguments=args or {},description=tool)],answer='准备执行')


class ContractTests(unittest.TestCase):
    def test_known_region_valid(self):self.assertEqual(validate_plan(plan('inspect_region',{'region':'设备区'}),REGIONS)['steps'][0]['tool'],'inspect_region')
    def test_unknown_region_rejected(self):
        with self.assertRaises(ValueError):validate_plan(plan('inspect_region',{'region':'不存在'}),REGIONS)
    def test_shell_action_rejected(self):
        with self.assertRaises(ValueError):validate_plan(plan('run_shell',{'cmd':'anything'}),REGIONS)
    def test_ned_or_unknown_coordinate_fields_rejected(self):
        with self.assertRaises(ValueError):validate_plan(plan('navigate_map',{'x':1,'y':2,'frame':'NED'}),REGIONS)
    def test_missing_coordinates_not_invented(self):
        with self.assertRaises(ValueError):validate_plan(plan('navigate_map',{'x':1}),REGIONS)
    def test_height_limit(self):
        with self.assertRaises(ValueError):validate_plan(plan('takeoff',{'height_m':10}),REGIONS)
    def test_nan_rejected(self):
        with self.assertRaises(ValueError):validate_plan(plan('navigate_map',{'x':float('nan'),'y':2}),REGIONS)
    def test_coordinate_boolean_rejected(self):
        with self.assertRaises(ValueError):validate_plan(plan('navigate_map',{'x':True,'y':2}),REGIONS)
    def test_optional_height_preserves_absence(self):self.assertNotIn('z',validate_plan(plan('navigate_map',{'x':1,'y':2}),REGIONS)['steps'][0]['arguments'])
    def test_secret_redacted(self):self.assertNotIn('sk-',redact('sk-exampleSecretString0123456789'))
    def test_cloud_function_call_contract(self):
        response=Mock(status_code=200);response.json.return_value={'choices':[dict(finish_reason='tool_calls',message=dict(tool_calls=[dict(function=dict(name='submit_plan',arguments=json.dumps(plan('status'))))]))]}
        post=Mock(return_value=response)
        result=request_plan('https://api.deepseek.com','deepseek-flash','dummy','查看状态',{'regions':REGIONS},post=post)
        self.assertEqual(result['steps'][0]['tool'],'status')
        self.assertEqual(post.call_args.kwargs['json']['tool_choice']['function']['name'],'submit_plan')
    def test_http_failure_does_not_echo_credentials(self):
        post=Mock(return_value=Mock(status_code=401))
        with self.assertRaises(ValueError) as err:request_plan('https://api.deepseek.com','deepseek-flash','private-test-value','status',{},post=post)
        self.assertNotIn('private-test-value',str(err.exception))
    def test_multiple_cloud_plans_rejected(self):
        response=Mock(status_code=200);call=dict(function=dict(name='submit_plan',arguments=json.dumps(plan('status'))))
        response.json.return_value={'choices':[dict(message=dict(tool_calls=[call,call]))]}
        with self.assertRaises(ValueError):request_plan('https://api.deepseek.com','deepseek-flash','dummy','status',{},post=Mock(return_value=response))


class AgentTests(unittest.TestCase):
    def node(self):
        from drone_ai_node import AiNode
        n=AiNode.__new__(AiNode);n.lock=threading.RLock();n.dispatch_lock=threading.RLock()
        n.epoch=1;n.parse=True;n.control=True;n.alive=Mock(return_value=True);n.context=Mock(return_value=({'regions':[]},[]))
        n.key='dummy';n.base_url='https://api.deepseek.com';n.model='deepseek-flash';n.network_timeout=30.
        n.state='PARSING';n.plan={'steps':[]};n.answer='';n.error='';n.busy=True;n.step=-1;n.owned=None
        n.inspection={}
        n.cleanup=Mock();n.publish=Mock();n.event=Mock();n.execute_step=Mock(return_value=False)
        return n
    def test_parse_only_cannot_execute(self):
        n=self.node()
        with patch('drone_ai_node.request_plan',return_value=plan('arm')):n.worker('解锁',1,False)
        self.assertEqual(n.state,'PARSED');n.execute_step.assert_not_called()
    def test_late_cloud_result_after_revoke_is_discarded(self):
        n=self.node()
        def late(*a):n.epoch=2;n.parse=False;n.state='CANCELED';return plan('arm')
        with patch('drone_ai_node.request_plan',side_effect=late):n.worker('解锁',1,True)
        self.assertEqual(n.plan,{'steps':[]});n.execute_step.assert_not_called();self.assertEqual(n.state,'CANCELED')
    def test_steps_are_sequential(self):
        n=self.node();p=dict(steps=[plan(t)['steps'][0] for t in ['arm','takeoff','land']],answer='按顺序')
        with patch('drone_ai_node.request_plan',return_value=p):n.worker('起飞后降落',1,True)
        self.assertEqual([c.args[0]['tool'] for c in n.execute_step.call_args_list],['arm','takeoff','land'])
        self.assertEqual(n.state,'SUCCEEDED')
    def test_failure_stops_later_steps(self):
        n=self.node();n.execute_step.side_effect=ValueError('安全门拒绝')
        p=dict(steps=[plan(t)['steps'][0] for t in ['arm','land']],answer='按顺序')
        with patch('drone_ai_node.request_plan',return_value=p):n.worker('解锁然后降落',1,True)
        self.assertEqual(n.execute_step.call_count,1);self.assertEqual(n.state,'FAILED')
    def test_cancel_navigation_identity_is_preserved(self):
        from drone_ai_node import AiNode
        n=self.node();n.owned=('nav',123,1);n.cancel_nav=Mock()
        AiNode.cleanup(n);n.cancel_nav.assert_called_once_with(123)
    def test_cancel_does_not_interrupt_landing(self):
        from drone_ai_node import AiNode
        n=self.node();n.owned=('land',None,1);n.flight=Mock()
        AiNode.cleanup(n);n.flight.assert_not_called()
    def test_revoke_reports_canceled_and_invalidates_old_epoch(self):
        from drone_ai_node import AiNode
        from drone_stack.srv import AiGateRequest
        n=self.node();n.session='session';old=n.epoch
        with patch('drone_ai_node.threading.Thread'):
            result=AiNode.gate(n,AiGateRequest(session_id='session',gate='control',enabled=False))
        self.assertTrue(result.success);self.assertEqual(n.state,'CANCELED');self.assertGreater(n.epoch,old)
        self.assertFalse(n.control)
    def test_late_cleanup_cannot_cancel_a_new_request(self):
        from drone_ai_node import AiNode
        n=self.node();n.owned=('nav',456,2);n.cancel_nav=Mock()
        AiNode.cleanup(n,1)
        n.cancel_nav.assert_not_called();self.assertEqual(n.owned,('nav',456,2))


class FlightGuardTests(unittest.TestCase):
    def node(self,phase):
        from flight_manager import FlightManager
        n=FlightManager.__new__(FlightManager);n.lock=threading.RLock()
        n.ai_flight_owned='takeoff';n.ai_flight_session='session';n.ai_flight_request='request';n.phase=phase
        n.hold=Mock();n.disarm=Mock();return n
    def test_stale_ground_flag_cannot_disarm_during_takeoff(self):
        from drone_stack.srv import ExecuteAiActionRequest
        n=self.node('TAKEOFF')
        result=n.execute_ai_action(ExecuteAiActionRequest(session_id='session',request_id='request',action='disarm'))
        self.assertFalse(result.success);n.disarm.assert_not_called()
    def test_stop_does_not_override_existing_landing(self):
        from drone_stack.srv import ExecuteAiActionRequest
        n=self.node('LANDING')
        result=n.execute_ai_action(ExecuteAiActionRequest(session_id='session',request_id='request',action='hold'))
        self.assertFalse(result.success);n.hold.assert_not_called()
    def test_inspection_owner_generation_is_checked(self):
        from inspection_manager import InspectionManager
        n=InspectionManager.__new__(InspectionManager);n.ai_session='session';n.ai_request='new';n.ai_control=True;n.ai_task_active=True;n.ai_seen_wall=time.monotonic()
        self.assertFalse(n.owner_valid('ai:session|old'));self.assertTrue(n.owner_valid('ai:session|new'));self.assertTrue(n.owner_valid(''))
    def test_late_inspection_cancel_cannot_cancel_manual_takeover(self):
        from inspection_manager import InspectionManager
        from drone_stack.srv import InspectionCommandRequest
        n=InspectionManager.__new__(InspectionManager);n.lock=threading.RLock();n.plan={'id':'plan','owner':''};n.pending=None;n.serial=0;n.issue=Mock();n.request_hold=Mock()
        reply=n.command(InspectionCommandRequest(action='cancel',plan_id='plan',owner='ai:session|old'))
        self.assertFalse(reply.success);self.assertEqual(n.serial,0);n.request_hold.assert_not_called()
    def test_cancel_other_navigation_is_rejected(self):
        from flight_manager import FlightManager
        from drone_stack.srv import CancelNavigationRequest
        n=FlightManager.__new__(FlightManager);n.lock=threading.RLock();n.route_sequence=42;n.phase='NAVIGATING';n.hold=Mock()
        self.assertFalse(n.cancel_navigation(CancelNavigationRequest(task_id=41)).success);n.hold.assert_not_called()
    def test_old_ai_request_cannot_submit_new_motion(self):
        from flight_manager import FlightManager
        from drone_stack.srv import ExecuteAiGoalRequest
        n=FlightManager.__new__(FlightManager);n.lock=threading.RLock();n.ai_valid=Mock(return_value=True);n.ai_request_id='new';n.ai_task_active=True;n.local_goal=Mock()
        result=n.execute_ai_goal(ExecuteAiGoalRequest(session_id='session',request_id='old'))
        self.assertFalse(result.success);n.local_goal.assert_not_called()


if __name__=='__main__':unittest.main(verbosity=2)
