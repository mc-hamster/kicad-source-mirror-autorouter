#!/usr/bin/env python3
"""Report the first stable room-door decision divergence.

The pinned Freerouting source already emits ``RAW_SECTION assign`` records at
TRACE level.  Native KiCad emits equivalent JSONL records when
``KICAD_AUTOROUTER_DECISION_TRACE`` is set.  Volatile floating costs and local
object IDs are excluded by default; ``--strict-context`` and ``--geometry``
progressively tighten the comparison after structural ordering agrees.  Net
names, rather than engine-local numeric net IDs, provide the stable identity:
KiCad legitimately renumbers nets when the parity harness strips and reloads
the board.
"""

from __future__ import annotations

import argparse
from decimal import Decimal, InvalidOperation
import json
import re
import sys
from pathlib import Path


REFERENCE_PREFIX = re.compile(
    r"RAW_SECTION assign selected_section=(?P<selected_section>\d+), "
    r"from_section=(?P<from_section>\d+), "
    r"backtrack_section=(?P<backtrack_section>\d+), "
    r"add_costs=(?P<add_costs>-?\d+), "
    r"adjustment=(?P<adjustment>[A-Z_]+), "
    r"roomRipped=(?P<room_ripped>true|false), .*?"
    r"door=(?P<door>.*?), door_bounds=(?P<door_bounds>\[.*?\]), "
    r"from_door=(?P<from_door>.*?), "
    r"from_door_bounds=(?P<from_door_bounds>\[.*?\]), net=(?P<net>\d+)"
)
BOUNDS = re.compile(
    r"\[\((?P<min_x>-?\d+),(?P<min_y>-?\d+)\)\.\."
    r"\((?P<max_x>-?\d+),(?P<max_y>-?\d+)\)\]"
)
DIMENSION = re.compile(r"/dim=(?P<dimension>-?\d+)(?:/|$)")
REFERENCE_NET = re.compile(
    r"\[NetIncompletes\.<init>\].*?net=(?P<net>\d+), "
    r"name=(?P<name>.*?), total_items_in_collection="
)


def normalize_bounds(value: str) -> str:
    match = BOUNDS.fullmatch(value)
    if not match:
        raise ValueError(f"invalid Freerouting bounds: {value}")
    return ",".join(match.group(name) for name in ("min_x", "min_y", "max_x", "max_y"))


def dimension(value: str) -> str:
    match = DIMENSION.search(value)
    return match.group("dimension") if match else "-1"


def reference_events(path: Path) -> list[dict[str, str]]:
    events = []
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    net_names = {}

    for line in lines:
        match = REFERENCE_NET.search(line)
        if match:
            net_names[match.group("net")] = match.group("name")

    for line_number, line in enumerate(lines, 1):
        if "RAW_SECTION assign" not in line:
            continue
        match = REFERENCE_PREFIX.search(line)
        if not match or not match.group("door").startswith("ExpansionDoor/"):
            continue
        fields = match.groupdict()
        events.append({
            "net": fields["net"],
            "net_name": net_names.get(fields["net"], f"<unmapped-net-{fields['net']}>"),
            "selected_section": fields["selected_section"],
            "from_section": fields["from_section"],
            "backtrack_section": fields["backtrack_section"],
            "add_costs": fields["add_costs"],
            "adjustment": fields["adjustment"],
            "room_ripped": fields["room_ripped"],
            "door_dimension": dimension(fields["door"]),
            "from_door_dimension": dimension(fields["from_door"]),
            "door_bounds": normalize_bounds(fields["door_bounds"]),
            "from_door_bounds": normalize_bounds(fields["from_door_bounds"]),
            "source_line": str(line_number),
        })
    return events


def native_events(path: Path) -> list[dict[str, str]]:
    events = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get("event") != "RAW_SECTION_ASSIGN":
                continue
            fields = record.get("fields")
            if not isinstance(fields, dict) or not all(isinstance(value, str)
                                                       for value in fields.values()):
                raise ValueError(f"invalid native decision record at {path}:{line_number}")
            event = dict(fields)
            event["source_line"] = str(line_number)
            events.append(event)
    return events


def transformed_bounds(value: str, scale: int, invert_y: bool) -> str:
    coordinates = [int(part) for part in value.split(",")]
    if len(coordinates) != 4 or scale <= 0:
        raise ValueError(f"invalid native bounds or scale: {value}, {scale}")
    min_x, min_y, max_x, max_y = coordinates

    # Freerouting's IntBox is an outward-rounded envelope.  Native decision
    # traces retain exact KiCad-IU rational bounds, including half-source-unit
    # values.  Python round() uses nearest-even and can therefore contract an
    # envelope by one source coordinate.  Preserve the source floor/ceil
    # contract explicitly; y reflection also exchanges the lower/upper roles.
    def floor_scaled(coordinate: int) -> int:
        return coordinate // scale

    def ceil_scaled(coordinate: int) -> int:
        return -((-coordinate) // scale)

    source_min_x = floor_scaled(min_x)
    source_max_x = ceil_scaled(max_x)
    if invert_y:
        source_min_y = floor_scaled(-max_y)
        source_max_y = ceil_scaled(-min_y)
    else:
        source_min_y = floor_scaled(min_y)
        source_max_y = ceil_scaled(max_y)
    return f"{source_min_x},{source_min_y},{source_max_x},{source_max_y}"


def transformed_cost(value: str, scale: int) -> str:
    if scale <= 0:
        raise ValueError(f"invalid native cost scale: {scale}")
    try:
        result = Decimal(value) / Decimal(scale)
    except InvalidOperation as error:
        raise ValueError(f"invalid native cost: {value}") from error
    return format(result.normalize(), "f")


def projected(event: dict[str, str], keys: list[str], *, native: bool,
              scale: int, invert_y: bool) -> dict[str, str]:
    result = {key: event.get(key, "<missing>") for key in keys}
    if native:
        for key in ("door_bounds", "from_door_bounds"):
            if key in result and result[key] not in ("", "<missing>"):
                result[key] = transformed_bounds(result[key], scale, invert_y)
        if "add_costs" in result and result["add_costs"] not in ("", "<missing>"):
            result["add_costs"] = transformed_cost(result["add_costs"], scale)
    return result


def events_match(reference: dict[str, str], native: dict[str, str],
                 cost_tolerance: Decimal) -> bool:
    if reference.keys() != native.keys():
        return False
    for key, reference_value in reference.items():
        native_value = native[key]
        if key != "add_costs":
            if reference_value != native_value:
                return False
            continue
        try:
            if abs(Decimal(reference_value) - Decimal(native_value)) > cost_tolerance:
                return False
        except InvalidOperation:
            return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference_log", type=Path)
    parser.add_argument("native_jsonl", type=Path)
    parser.add_argument("--net", type=int)
    parser.add_argument("--net-name")
    parser.add_argument("--strict-context", action="store_true")
    parser.add_argument("--geometry", action="store_true")
    parser.add_argument(
        "--costs", action="store_true",
        help="also compare add_costs after converting native IU to reference units",
    )
    parser.add_argument(
        "--cost-tolerance-reference-units", type=Decimal, default=Decimal(0),
        help="absolute add_costs tolerance used with --costs (default: 0)",
    )
    parser.add_argument("--native-units-per-reference-unit", type=int, default=100)
    parser.add_argument("--no-invert-native-y", action="store_true")
    parser.add_argument("--max-events", type=int, default=0)
    parser.add_argument("--context", type=int, default=2)
    parser.add_argument("--json-report", type=Path)
    args = parser.parse_args()

    reference = reference_events(args.reference_log)
    native = native_events(args.native_jsonl)
    if args.net is not None:
        reference = [event for event in reference if event["net"] == str(args.net)]
        native = [event for event in native if event["net"] == str(args.net)]
    if args.net_name is not None:
        reference = [event for event in reference if event["net_name"] == args.net_name]
        native = [event for event in native if event["net_name"] == args.net_name]
    if args.max_events > 0:
        reference = reference[:args.max_events]
        native = native[:args.max_events]

    if args.cost_tolerance_reference_units < 0:
        parser.error("--cost-tolerance-reference-units must be non-negative")

    # Cost is intentionally opt-in.  It includes geometry-dependent detour
    # ratios and is therefore useful for classifying numeric drift only after
    # the stable decision stream agrees.
    keys = ["net_name", "selected_section", "adjustment", "door_dimension"]
    if args.costs:
        keys.append("add_costs")
    if args.strict_context:
        keys += ["from_section", "backtrack_section", "room_ripped", "from_door_dimension"]
    if args.geometry:
        keys += ["door_bounds", "from_door_bounds"]
    reference_view = [projected(event, keys, native=False,
                                scale=args.native_units_per_reference_unit,
                                invert_y=not args.no_invert_native_y)
                      for event in reference]
    native_view = [projected(event, keys, native=True,
                             scale=args.native_units_per_reference_unit,
                             invert_y=not args.no_invert_native_y)
                   for event in native]

    mismatch = next((index for index, pair in enumerate(zip(reference_view, native_view))
                     if not events_match(pair[0], pair[1],
                                         args.cost_tolerance_reference_units)), None)
    if mismatch is None and len(reference_view) != len(native_view):
        mismatch = min(len(reference_view), len(native_view))
    report: dict[str, object] = {
        "status": "MATCH" if mismatch is None else "DIVERGED",
        "keys": keys,
        "reference_events": len(reference_view),
        "native_events": len(native_view),
        "first_divergence": mismatch,
    }
    if args.costs:
        report["cost_tolerance_reference_units"] = str(
            args.cost_tolerance_reference_units
        )
    if mismatch is not None:
        start = max(0, mismatch - max(0, args.context))
        end = mismatch + max(0, args.context) + 1
        report["reference_context"] = reference_view[start:end]
        report["native_context"] = native_view[start:end]
        report["reference_source_lines"] = [event["source_line"] for event in reference[start:end]]
        report["native_source_lines"] = [event["source_line"] for event in native[start:end]]

    serialized = json.dumps(report, indent=2) + "\n"
    if args.json_report:
        args.json_report.write_text(serialized, encoding="utf-8")
    print(serialized, end="")

    if not reference_view or not native_view:
        print("decision comparison has no comparable events", file=sys.stderr)
        return 2
    return 0 if mismatch is None else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"decision comparison failed: {error}", file=sys.stderr)
        raise SystemExit(2)
