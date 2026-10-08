#!/usr/bin/env python3
"""Offline regression orchestration tests; subprocesses are mocked."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('runner',Path(__file__).with_name('run_sim_regression.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class RunnerTests(unittest.TestCase):
    def run_case(self, outcomes):
        with tempfile.TemporaryDirectory() as path:
            with patch.object(module.subprocess,'run',side_effect=outcomes) as run:
                result=module.run_suite([['takeoff'],['hold_check','60']],Path(path),{})
                self.assertEqual(json.loads((Path(path)/'suite_result.json').read_text()),result)
                self.assertEqual(run.call_args[0][0][-1],'land')
            return result

    def test_all_success_passes(self):
        self.assertTrue(self.run_case([SimpleNamespace(returncode=0)]*3)['passed'])

    def test_early_failure_still_lands(self):
        result=self.run_case([SimpleNamespace(returncode=1),SimpleNamespace(returncode=0)])
        self.assertFalse(result['passed']);self.assertEqual(len(result['checks']),2)

    def test_flight_timeout_preserved_and_lands(self):
        result=self.run_case([subprocess.TimeoutExpired('flight',360),SimpleNamespace(returncode=0)])
        self.assertFalse(result['passed']);self.assertEqual(result['checks'][0]['exit_code'],124)

    def test_land_timeout_preserved(self):
        result=self.run_case([SimpleNamespace(returncode=0)]*2+[subprocess.TimeoutExpired('land',200)])
        self.assertFalse(result['passed']);self.assertEqual(result['checks'][-1]['exit_code'],124)

    def test_land_rejection_fails_suite(self):
        self.assertFalse(self.run_case([SimpleNamespace(returncode=0)]*2+[SimpleNamespace(returncode=1)])['passed'])

    def test_process_start_error_preserved_and_lands(self):
        result=self.run_case([OSError('cannot start'),SimpleNamespace(returncode=0)])
        self.assertFalse(result['passed']);self.assertEqual(result['checks'][0]['exit_code'],125)


if __name__=='__main__':unittest.main(verbosity=2)
