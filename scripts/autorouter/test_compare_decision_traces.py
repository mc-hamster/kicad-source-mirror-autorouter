import json
import subprocess
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name("compare_decision_traces.py")


def reference_net(net: int = 7, name: str = "Net-(Q2-B)") -> str:
    return (
        "2026-01-01 TRACE [NetIncompletes.<init>] net=" + str(net)
        + ", name=" + name + ", total_items_in_collection=2\n"
    )


def reference_line(section: int, bounds: str, net: int = 7) -> str:
    min_x, min_y, max_x, max_y = bounds.split(",")
    return (
        "2026-01-01 TRACE RAW_SECTION assign selected_section=" + str(section)
        + ", from_section=0, backtrack_section=0, add_costs=0, adjustment=NONE, "
        "roomRipped=false, expansionValue=1.25, sortingValue=2.5, "
        "door=ExpansionDoor/bounds=[(10,-40)..(30,-20)]/dim=1/sections=2, "
        "door_bounds=[(" + min_x + "," + min_y + ")..(" + max_x + "," + max_y + ")], "
        "from_door=TargetItemExpansionDoor/item=1/dim=2/sections=1, "
        "from_door_bounds=[(1,-4)..(3,-2)], net=" + str(net) + "\n"
    )


class DecisionTraceComparatorTest(unittest.TestCase):
    def run_compare(self, reference: str, records: list[dict], *arguments: str):
        with tempfile.TemporaryDirectory() as directory_name:
            directory = Path(directory_name)
            reference_path = directory / "reference.log"
            native_path = directory / "native.jsonl"
            reference_path.write_text(reference, encoding="utf-8")
            native_path.write_text("".join(json.dumps(record) + "\n" for record in records),
                                   encoding="utf-8")
            return subprocess.run(
                ["python3", str(SCRIPT), str(reference_path), str(native_path), *arguments],
                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            )

    def test_matches_stable_structure_and_transformed_geometry(self):
        native = [{
            "sequence": 0,
            "event": "RAW_SECTION_ASSIGN",
            "fields": {
                "net": "2", "net_name": "Net-(Q2-B)",
                "selected_section": "0", "from_section": "0",
                "backtrack_section": "0", "add_costs": "0", "adjustment": "NONE",
                "room_ripped": "false", "door_dimension": "1",
                "from_door_dimension": "2", "door_bounds": "1000,2000,3000,4000",
                "from_door_bounds": "100,200,300,400",
                "expansion_value": "999", "sorting_value": "1000",
            },
        }]
        completed = self.run_compare(reference_net() + reference_line(0, "10,-40,30,-20"), native,
                                     "--strict-context", "--geometry")
        self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)
        self.assertEqual("MATCH", json.loads(completed.stdout)["status"])

    def test_geometry_conversion_outward_rounds_half_source_units(self):
        native = [{
            "sequence": 0,
            "event": "RAW_SECTION_ASSIGN",
            "fields": {
                "net": "2", "net_name": "Net-(Q2-B)",
                "selected_section": "0", "from_section": "0",
                "backtrack_section": "0", "add_costs": "0", "adjustment": "NONE",
                "room_ripped": "false", "door_dimension": "1",
                "from_door_dimension": "2",
                # Exact native bounds are x=[1480489, 1480920] and
                # y=[651819.5, 652250] in source-coordinate units.
                "door_bounds": "148048900,65181950,148092000,65225000",
                "from_door_bounds": "100,200,300,400",
            },
        }]
        reference = reference_net() + reference_line(
            0, "1480489,-652250,1480920,-651819"
        )
        completed = self.run_compare(
            reference, native, "--strict-context", "--geometry"
        )
        self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)
        self.assertEqual("MATCH", json.loads(completed.stdout)["status"])

    def test_reports_first_structural_divergence(self):
        native = []
        for sequence, section in enumerate((0, 2)):
            native.append({
                "sequence": sequence, "event": "RAW_SECTION_ASSIGN",
                "fields": {"net": "2", "net_name": "Net-(Q2-B)",
                           "selected_section": str(section),
                           "add_costs": "0", "adjustment": "NONE",
                           "door_dimension": "1"},
            })
        reference = (reference_net() + reference_line(0, "10,-40,30,-20")
                     + reference_line(1, "10,-40,30,-20"))
        completed = self.run_compare(reference, native)
        self.assertEqual(1, completed.returncode)
        report = json.loads(completed.stdout)
        self.assertEqual("DIVERGED", report["status"])
        self.assertEqual(1, report["first_divergence"])

    def test_matches_same_net_name_when_numeric_ids_differ(self):
        native = [{
            "sequence": 0,
            "event": "RAW_SECTION_ASSIGN",
            "fields": {
                "net": "2", "net_name": "Net-(Q2-B)", "selected_section": "0",
                "add_costs": "0", "adjustment": "NONE", "door_dimension": "1",
            },
        }]
        completed = self.run_compare(
            reference_net(7, "Net-(Q2-B)") + reference_line(0, "10,-40,30,-20", 7),
            native,
        )
        self.assertEqual(0, completed.returncode, completed.stdout + completed.stderr)
        report = json.loads(completed.stdout)
        self.assertEqual("MATCH", report["status"])
        self.assertEqual("net_name", report["keys"][0])

    def test_costs_are_opt_in_and_scaled_to_reference_units(self):
        native = [{
            "sequence": 0,
            "event": "RAW_SECTION_ASSIGN",
            "fields": {
                "net": "2", "net_name": "Net-(Q2-B)", "selected_section": "0",
                "add_costs": "9570784", "adjustment": "NONE", "door_dimension": "1",
            },
        }]
        reference = reference_net() + reference_line(0, "10,-40,30,-20").replace(
            "add_costs=0", "add_costs=95507"
        )

        structural = self.run_compare(reference, native)
        self.assertEqual(0, structural.returncode, structural.stdout + structural.stderr)

        exact_cost = self.run_compare(reference, native, "--costs")
        self.assertEqual(1, exact_cost.returncode)
        exact_report = json.loads(exact_cost.stdout)
        self.assertEqual("95507", exact_report["reference_context"][0]["add_costs"])
        self.assertEqual("95707.84", exact_report["native_context"][0]["add_costs"])

        tolerant_cost = self.run_compare(
            reference, native, "--costs", "--cost-tolerance-reference-units", "201"
        )
        self.assertEqual(0, tolerant_cost.returncode,
                         tolerant_cost.stdout + tolerant_cost.stderr)


if __name__ == "__main__":
    unittest.main()
