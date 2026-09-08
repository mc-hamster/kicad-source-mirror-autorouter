"""Regression tests for false parity claims, independent of routing geometry."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import compare_results


class ParityGateTests(unittest.TestCase):
    def result(self):
        data = dict.fromkeys(compare_results.METRICS, 0)
        data.update(dict.fromkeys(compare_results.NATIVE_DRC_METRICS, 0))
        data.update(input_board_sha256="fixture", strip_tracks=True,
                    input_files_sha256={".kicad_pcb": "fixture", ".kicad_dru": None},
                    validation_complete=True, baseline_kicad_unconnected=10,
                    kicad_unconnected=0, completion_percent=100)
        return data

    def compare(self, reference, native):
        with tempfile.TemporaryDirectory() as directory:
            paths = [Path(directory) / name for name in ("reference.json", "native.json")]
            for path, data in zip(paths, (reference, native)):
                path.write_text(json.dumps(data))
            return subprocess.run([sys.executable, compare_results.__file__, *map(str, paths)],
                                  capture_output=True, text=True).returncode

    def test_valid_host_measurements_pass(self):
        self.assertEqual(self.compare(self.result(), self.result()), 0)

    def test_skipped_drc_cannot_pass(self):
        for missing in (-1, float("nan"), None):
            with self.subTest(missing=missing):
                native = self.result()
                native["new_kicad_drc_errors"] = missing
                self.assertNotEqual(self.compare(self.result(), native), 0)

    def test_missing_validation_cannot_pass(self):
        native = self.result()
        del native["validation_complete"]
        self.assertNotEqual(self.compare(self.result(), native), 0)

    def test_reference_needs_actual_host_drc_measurements(self):
        reference = self.result()
        reference["kicad_drc_errors"] = -1
        self.assertNotEqual(self.compare(reference, self.result()), 0)

    def test_synthetic_success_does_not_hide_unconnected_copper(self):
        native = self.result()
        native.update(kicad_unconnected=1, complete=True, routed_connections=10000)
        self.assertNotEqual(self.compare(self.result(), native), 0)

    def test_synthetic_denominator_does_not_define_parity(self):
        native = self.result()
        native.update(total_connections=10000, routed_connections=10000, completion_percent=50)
        self.assertEqual(self.compare(self.result(), native), 0)

    def test_different_inputs_cannot_pass(self):
        native = self.result()
        native["input_board_sha256"] = "different"
        self.assertNotEqual(self.compare(self.result(), native), 0)

    def test_different_custom_rules_cannot_pass(self):
        native = self.result()
        native["input_files_sha256"][".kicad_dru"] = "changed"
        self.assertNotEqual(self.compare(self.result(), native), 0)

    def test_new_violation_fails_even_when_routed(self):
        native = self.result()
        native["new_kicad_drc_errors"] = 1
        self.assertNotEqual(self.compare(self.result(), native), 0)

    def test_different_via_constraints_cannot_pass(self):
        reference, native = self.result(), self.result()
        reference["routing_constraints"] = {"allow_vias": True}
        native["routing_constraints"] = {"allow_vias": False}
        self.assertNotEqual(self.compare(reference, native), 0)

    def test_matching_no_via_constraints_can_pass(self):
        reference, native = self.result(), self.result()
        reference["routing_constraints"] = native["routing_constraints"] = {"allow_vias": False}
        self.assertEqual(self.compare(reference, native), 0)

    def test_raw_worker_run_is_not_production_parity(self):
        native = self.result()
        native.update(host_validated=False, host_unconnected=-1)
        self.assertNotEqual(self.compare(self.result(), native), 0)

    def test_production_and_materialized_connectivity_must_agree(self):
        native = self.result()
        native.update(host_validated=True, host_unconnected=1)
        self.assertNotEqual(self.compare(self.result(), native), 0)
        native["host_unconnected"] = 0
        self.assertEqual(self.compare(self.result(), native), 0)
        native["host_new_drc_violations"] = 1
        self.assertNotEqual(self.compare(self.result(), native), 0)

    def test_extra_vias_still_fail_when_drc_is_clean(self):
        native = self.result()
        native["via_count"] = 1
        self.assertNotEqual(self.compare(self.result(), native), 0)


if __name__ == "__main__":
    unittest.main()
