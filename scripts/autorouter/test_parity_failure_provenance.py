import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import run_parity_case


class FailureProvenanceTest(unittest.TestCase):
    def test_native_timeout_preserves_completed_reference_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            board, binary, jar, output = (root / name for name in ("input.kicad_pcb", "native", "reference.jar", "out"))
            board.write_text("fixture")
            board.with_suffix(".kicad_pro").write_text("project rules")
            binary.write_text("binary identity")
            with zipfile.ZipFile(jar, "w") as archive:
                archive.writestr("META-INF/MANIFEST.MF", "Build-Revision: fixture-pin\n")

            def run(command, **kwargs):
                if "--export-dsn" in command:
                    Path(command[command.index("--export-dsn") + 1]).write_text("dsn")
                elif "-do" in command:
                    Path(command[command.index("-do") + 1]).write_text("ses")
                    kwargs["stdout"].write("Auto-routing stage completed: completed in 1.00 seconds\n")
                elif "--import-ses" in command:
                    Path(command[command.index("--out") + 1]).write_text(json.dumps({"via_count": 0, "runtime_ms": 0}))
                else:
                    raise subprocess.TimeoutExpired(command, 1)
                return subprocess.CompletedProcess(command, 0)

            argv = ["run_parity_case", "--board", str(board), "--binary", str(binary),
                    "--reference-jar", str(jar), "--reference-commit", "fixture-pin",
                    "--output-dir", str(output), "--strip-tracks", "--save-boards", "--no-vias",
                    "--timeout-seconds", "1"]
            with patch("sys.argv", argv), patch.object(run_parity_case.subprocess, "run", side_effect=run):
                with self.assertRaises(subprocess.TimeoutExpired):
                    run_parity_case.main()
            reference = json.loads((output / "reference.json").read_text())
            self.assertEqual(reference["routing_constraints"], {"allow_vias": False})
            self.assertEqual(reference["routing_core_ms"], 1000)
            self.assertTrue(reference["input_board_sha256"])
            self.assertEqual((output / "reference.kicad_pro").read_text(), "project rules")
            self.assertFalse((output / "native.json").exists())
            commands = json.loads((output / "provenance.json").read_text())["commands"]
            self.assertEqual(commands[-1]["status"], "timeout")
            self.assertEqual(commands[-1]["timeout_seconds"], 1)
            self.assertGreaterEqual(commands[-1]["wall_ms"], 0)
