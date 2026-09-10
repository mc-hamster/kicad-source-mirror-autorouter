#!/usr/bin/env python3
"""Check native golden room/drill/destination primitives against pinned Freerouting.

The JAR is QA-only. This tool never builds in, instruments, or modifies a
Freerouting checkout. Outputs are written to a fresh native build directory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import subprocess
import zipfile

REFERENCE_REVISION = "a11c0a42d1b3827e5126429c5c9820c4ab5bec7c"
ROOT = Path(__file__).resolve().parents[2]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_reference(jar: Path) -> str:
    with zipfile.ZipFile(jar) as archive:
        manifest = archive.read("META-INF/MANIFEST.MF").decode().replace("\r\n ", "").replace("\n ", "")
    revision = next((line.partition(": ")[2] for line in manifest.splitlines()
                     if line.startswith("Build-Revision: ")), None)
    if revision != REFERENCE_REVISION:
        raise ValueError(f"reference revision {revision!r} != source pin {REFERENCE_REVISION}")
    return revision


def normalize_drill_records(raw: str) -> str:
    prefixes = ("CUT ", "AREA ", "PAGES ", "PAGE_COST ", "DRILL_COST ")
    records = []
    warning = re.compile(r"\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+ WARN\s+"
                         r"PolylineArea\. split_to_convex: dimension 2 for hole expected")
    for line in raw.splitlines():
        if line.startswith(prefixes):
            records.append(line)
        elif not warning.fullmatch(line):
            raise ValueError(f"unexpected drill oracle diagnostic: {line}")
    if any(sum(line.startswith(prefix) for line in records) != 256 for prefix in prefixes):
        raise ValueError("incomplete drill oracle output")
    return "\n".join(records) + "\n"


def normalize_simplex_records(raw: str) -> str:
    records = []
    warning = re.compile(r"\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+ WARN\s+"
                         r"Simplex\.cutout_from only implemented for 2-dim simplex")
    for line in raw.splitlines():
        if line.startswith(("SIMPLEX ", "SCUT ", "SOFF ", "STILE ")):
            records.append(line)
        elif not warning.fullmatch(line):
            raise ValueError(f"unexpected simplex oracle diagnostic: {line}")
    if sum(line.startswith("SIMPLEX ") for line in records) != 512:
        raise ValueError("incomplete simplex primitive output")
    if sum(line.startswith("SCUT ") for line in records) != 256:
        raise ValueError("incomplete simplex cutout output")
    if sum(line.startswith("SOFF ") for line in records) != 192:
        raise ValueError("incomplete simplex offset output")
    if sum(line.startswith("STILE ") for line in records) != 256:
        raise ValueError("incomplete simplex tile-shape output")
    return "\n".join(records) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-jar", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--java", default="java")
    parser.add_argument("--javac", default="javac")
    parser.add_argument("--oracle", choices=("room", "drill", "destination", "contacts", "fanout", "convex", "simplex", "tile2", "tilex", "polyline", "polytransform", "segment", "float", "spring", "octagon", "room45", "room-general", "neighbours45", "neighbours-general", "door45", "door-general"), default="room")
    args = parser.parse_args()
    jar, output = args.reference_jar.resolve(), args.output_dir.resolve()
    verify_reference(jar)
    output.mkdir(parents=True, exist_ok=False)
    classes = output / "classes"
    classes.mkdir()
    drill = args.oracle == "drill"
    main_class = {"room": "RoomSearchOracle", "drill": "app.freerouting.autoroute.maze.DrillSearchOracle",
                  "destination": "DestinationDistanceOracle", "contacts": "NormalContactsOracle", "fanout": "FanoutOrderOracle", "convex": "ConvexGeometryOracle", "simplex": "SimplexGeometryOracle", "tile2": "TileShapeOracle", "tilex": "TileTransformOracle", "polyline": "PolylineGeometryOracle", "polytransform": "PolylineTransformOracle", "segment": "LineSegmentOracle", "float": "FloatGeometryOracle",
                  "spring": "SpringOverOracle", "octagon": "IntOctagonOracle",
                  "room45": "ShapeSearchTree45DegreeOracle",
                  "room-general": "app.freerouting.board.searchtree.ShapeSearchTreeOracle",
                  "neighbours45": "Sorted45DegreeRoomNeighboursOracle",
                  "neighbours-general": "SortedRoomNeighboursOracle",
                  "door45": "ExpansionDoorOracle",
                  "door-general": "ExpansionDoorGeneralOracle"}[args.oracle]
    source = ROOT / "scripts/autorouter" / (main_class.rsplit(".", 1)[-1] + ".java")
    expected = ROOT / f"qa/data/pcbnew/autorouter/{args.oracle}-search-a11c0a42.txt"
    sources = [source]
    if args.oracle in ("spring", "simplex", "tile2", "tilex", "polyline", "polytransform", "segment"):
        sources.append(ROOT / "scripts/autorouter/ConvexGeometryOracle.java")
    commands = [
        [args.javac, "-cp", str(jar), "-d", str(classes), *map(str, sources)],
        [args.java, "-cp", f"{jar}{os.pathsep}{classes}", main_class],
    ]
    metadata = {"reference_revision": REFERENCE_REVISION, "reference_jar_sha256": sha256(jar),
                "oracle_source_sha256": sha256(source), "expected_sha256": sha256(expected),
                "source_sha256": {str(path.relative_to(ROOT)): sha256(path) for path in sources},
                "commands": commands, "oracle": args.oracle}
    try:
        with (output / "compile.log").open("w") as log:
            subprocess.run(commands[0], stdout=log, stderr=subprocess.STDOUT, check=True, timeout=60)
        actual = output / "oracle.txt"
        with actual.open("w") as out, (output / "oracle-stderr.log").open("w") as err:
            subprocess.run(commands[1], stdout=out, stderr=err, check=True, timeout=60, cwd=output)
        if drill or args.oracle == "simplex":
            raw = actual.read_text()
            (output / "oracle-raw.txt").write_text(raw)
            # The source logs timestamped warnings for deliberately exercised
            # lower-dimensional inputs. Retain raw diagnostics and compare
            # only the declared deterministic oracle records.
            actual.write_text(normalize_drill_records(raw) if drill
                              else normalize_simplex_records(raw))
            metadata["normalization"] = ("only known lower-dimensional input warnings removed; "
                                         "all raw diagnostics retained in oracle-raw.txt")
        metadata["actual_sha256"] = sha256(actual)
        metadata["matches"] = actual.read_bytes() == expected.read_bytes()
        print(f"Pinned {args.oracle} oracle: " + ("PASS" if metadata["matches"] else "FAIL"))
        return 0 if metadata["matches"] else 1
    finally:
        (output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
