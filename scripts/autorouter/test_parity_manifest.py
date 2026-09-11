import json
import subprocess
import tempfile
import unittest
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SCRIPT = HERE / "build_parity_manifest.py"


class ParityManifestTest(unittest.TestCase):
    def test_jdk_parser_keeps_nested_and_overloaded_methods(self):
        with tempfile.TemporaryDirectory() as directory_name:
            directory = Path(directory_name)
            source_root = directory / "src/main/java/app/freerouting"
            source = source_root / "autoroute/Sample.java"
            source.parent.mkdir(parents=True)
            source.write_text(
                """package app.freerouting.autoroute;
public class Sample {
  public Sample() {}
  public int route(int count) { return count; }
  public int route(String name) { return name.length(); }
  record Door(int index) { int section() { return index; } }
}
""",
                encoding="utf-8",
            )
            completed = subprocess.run(
                ["java", str(HERE / "JavaMethodManifest.java"), "--source-root",
                 str(source_root), str(source)],
                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True,
            )
            records = [json.loads(line) for line in completed.stdout.splitlines()]
            signatures = {record["signature"] for record in records
                          if record["record"] == "method"}
            self.assertIn("Sample()", signatures)
            self.assertIn("route(int)", signatures)
            self.assertIn("route(String)", signatures)
            self.assertIn("section()", signatures)
            owners = {record["owner"] for record in records}
            self.assertIn("app.freerouting.autoroute.Sample$Door", owners)

    def test_checked_in_manifest_has_complete_unique_method_inventory(self):
        manifest = json.loads(
            (ROOT / "autorouter-docs/parity-method-manifest.json").read_text(encoding="utf-8")
        )
        methods = manifest["methods"]
        self.assertEqual(manifest["summary"]["methods"], len(methods))
        keys = [(method["key"], method["source_path"], method["source_line"])
                for method in methods]
        self.assertEqual(len(keys), len(set(keys)))
        self.assertTrue(all(method["status"] in {
            "PORTED_EXACT", "PORTED_ADAPTED", "PARTIAL", "MISSING", "EXCLUDED"
        } for method in methods))
        self.assertTrue(all(method["source_line"] > 0 for method in methods))


if __name__ == "__main__":
    unittest.main()
