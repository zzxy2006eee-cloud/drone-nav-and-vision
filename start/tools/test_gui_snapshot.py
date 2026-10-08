#!/usr/bin/env python3
import unittest
from unittest.mock import patch,MagicMock
import gui_snapshot


class GUIChecks(unittest.TestCase):
    def app(self,pid,children):
        app=MagicMock();app.name='drone_operator_gui';app.childCount=children
        app.get_process_id.return_value=pid;app.__iter__.return_value=[]
        return app

    def test_duplicate_registration_same_pid_uses_populated_tree(self):
        empty=self.app(123,0);populated=self.app(123,1)
        with patch.object(gui_snapshot.pyatspi.Registry,'getDesktop',return_value=[empty,populated]):
            self.assertEqual(len(gui_snapshot.widgets()),1)
            empty.getState.assert_not_called();populated.getState.assert_called_once()

    def test_different_processes_remain_rejected(self):
        with patch.object(gui_snapshot.pyatspi.Registry,'getDesktop',return_value=[self.app(123,1),self.app(456,1)]):
            with self.assertRaises(RuntimeError):gui_snapshot.widgets()

    def test_all_empty_registrations_remain_rejected(self):
        with patch.object(gui_snapshot.pyatspi.Registry,'getDesktop',return_value=[self.app(123,0),self.app(123,0)]):
            with self.assertRaises(RuntimeError):gui_snapshot.widgets()

    def buttons(self, land=True):
        return [{'name':name,'role':'push button','enabled':enabled,'sensitive':enabled,'showing':True}
                for name,enabled in [('PX4 解锁 ARM',False),('起飞',False),('悬停',False),
                                     ('取消目标',False),('一键降落',land)]]

    def test_protection_requires_disabled_flight_and_available_land(self):
        with patch.object(gui_snapshot,'widgets',return_value=self.buttons()):
            self.assertTrue(gui_snapshot.snapshot('protection')['passed'])

    def test_missing_button_is_not_a_pass(self):
        with patch.object(gui_snapshot,'widgets',return_value=self.buttons()[:-1]):
            self.assertFalse(gui_snapshot.snapshot('protection')['passed'])

    def test_enabled_takeoff_is_rejected(self):
        buttons=self.buttons();buttons[1]['enabled']=True
        with patch.object(gui_snapshot,'widgets',return_value=buttons):
            self.assertFalse(gui_snapshot.snapshot('protection')['passed'])

    def test_fcu_stale_requires_land_disabled(self):
        with patch.object(gui_snapshot,'widgets',return_value=self.buttons(False)):
            self.assertTrue(gui_snapshot.snapshot('fcu_stale')['passed'])
        with patch.object(gui_snapshot,'widgets',return_value=self.buttons(True)):
            self.assertFalse(gui_snapshot.snapshot('fcu_stale')['passed'])

    def test_qt_capitalized_press_action_is_exercised(self):
        action=MagicMock();action.nActions=1;action.getName.return_value='Press'
        action.doAction.return_value=True
        widget=MagicMock();widget.queryAction.return_value=action
        entries=[{'name':'一键降落','role':'push button','enabled':True,'object':widget}]
        with patch.object(gui_snapshot,'widgets',return_value=entries):
            self.assertTrue(gui_snapshot.click_land()['activated'])
        action.doAction.assert_called_once_with(0)

    def test_unknown_action_cannot_certify_click(self):
        action=MagicMock();action.nActions=1;action.getName.return_value='toggle'
        widget=MagicMock();widget.queryAction.return_value=action
        entries=[{'name':'一键降落','role':'push button','enabled':True,'object':widget}]
        with patch.object(gui_snapshot,'widgets',return_value=entries):
            with self.assertRaisesRegex(RuntimeError,'toggle'):gui_snapshot.click_land()
        action.doAction.assert_not_called()

    def test_no_accessibility_tree_cannot_certify_gui(self):
        with patch.object(gui_snapshot,'widgets',return_value=[]):
            r=gui_snapshot.snapshot('protection')
            self.assertFalse(r['accessible']);self.assertFalse(r['passed'])


if __name__=='__main__':unittest.main(verbosity=2)
