import tempfile
import unittest
import zipfile
from pathlib import Path

from run_room_oracle import REFERENCE_REVISION, verify_reference, normalize_drill_records


class ReferenceIdentityTest(unittest.TestCase):
    def test_pinned_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            jar = Path(directory) / "oracle.jar"
            with zipfile.ZipFile(jar, "w") as archive:
                archive.writestr("META-INF/MANIFEST.MF", f"Build-Revision: {REFERENCE_REVISION}\r\n")
            self.assertEqual(verify_reference(jar), REFERENCE_REVISION)

    def test_wrong_or_missing_revision_rejected(self):
        for manifest in ("Build-Revision: another-build\n", "Manifest-Version: 1.0\n"):
            with self.subTest(manifest=manifest), tempfile.TemporaryDirectory() as directory:
                jar = Path(directory) / "oracle.jar"
                with zipfile.ZipFile(jar, "w") as archive:
                    archive.writestr("META-INF/MANIFEST.MF", manifest)
                with self.assertRaises(ValueError):
                    verify_reference(jar)

    def test_drill_oracle_filters_only_known_timestamped_warning(self):
        records = "\n".join(prefix + str(i) for prefix in
                            ("CUT ", "AREA ", "PAGES ", "PAGE_COST ", "DRILL_COST ")
                            for i in range(256)) + "\n"
        warning = "2026-09-08 08:02:59.364 WARN   PolylineArea. split_to_convex: dimension 2 for hole expected\n"
        self.assertEqual(normalize_drill_records(warning + records), records)
        for bad in (records.replace("CUT 0\n", ""), records.replace("CUT 0\n", "AREA 0\n"),
                    records + "unexpected diagnostic\n", records + warning.replace("WARN", "ERROR")):
            with self.subTest(bad=bad[-100:]), self.assertRaises(ValueError):
                normalize_drill_records(bad)
