#!/usr/bin/env python3
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/drone_stack/scripts'))
import audit_offboard_loss as module


class NativeAudit(unittest.TestCase):
    def fixture(self, landing_us=11000000, disarmed=True, failsafe=True):
        status={'timestamp':np.array([10000000,landing_us,landing_us+1000000]),
                'nav_state_timestamp':np.array([10000000,landing_us,landing_us]),
                'nav_state':np.array([14,18,18]),'failsafe':np.array([False,failsafe,failsafe]),
                'arming_state':np.array([2,2,1 if disarmed else 2])}
        control={'timestamp':np.array([9900000,10000000])}
        return SimpleNamespace(data_list=[SimpleNamespace(name='vehicle_status',data=status),
                                         SimpleNamespace(name='offboard_control_mode',data=control)])

    def test_native_timing_and_disarming_pass(self):
        with patch.object(module,'ULog',return_value=self.fixture()):
            result=module.audit('sample.ulg')
        self.assertTrue(result['passed']);self.assertEqual(result['response_from_last_logged_control_s'],1)

    def test_late_response_fails_original_limit(self):
        with patch.object(module,'ULog',return_value=self.fixture(landing_us=11600000)):
            self.assertFalse(module.audit('sample.ulg')['passed'])

    def test_remains_armed_cannot_pass(self):
        with patch.object(module,'ULog',return_value=self.fixture(disarmed=False)):
            self.assertFalse(module.audit('sample.ulg')['passed'])

    def test_requested_land_without_native_failsafe_cannot_pass(self):
        with patch.object(module,'ULog',return_value=self.fixture(failsafe=False)):
            self.assertFalse(module.audit('sample.ulg')['passed'])

    def test_missing_native_topics_cannot_pass(self):
        with patch.object(module,'ULog',return_value=SimpleNamespace(data_list=[])):
            self.assertFalse(module.audit('sample.ulg')['passed'])


if __name__=='__main__':unittest.main(verbosity=2)
