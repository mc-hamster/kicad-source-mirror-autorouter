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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-jar", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--java", default="java")
    parser.add_argument("--javac", default="javac")
    parser.add_argument("--oracle", choices=("room", "drill", "destination", "contacts", "fanout", "convex", "spring", "octagon", "room45", "neighbours45"), default="room")
    args = parser.parse_args()
    jar, output = args.reference_jar.resolve(), args.output_dir.resolve()
    verify_reference(jar)
    output.mkdir(parents=True, exist_ok=False)
    classes = output / "classes"
    classes.mkdir()
    drill = args.oracle == "drill"
    main_class = {"room": "RoomSearchOracle", "drill": "app.freerouting.autoroute.maze.DrillSearchOracle",
                  "destination": "DestinationDistanceOracle", "contacts": "NormalContactsOracle", "fanout": "FanoutOrderOracle", "convex": "ConvexGeometryOracle",
                  "spring": "SpringOverOracle", "octagon": "IntOctagonOracle",
                  "room45": "ShapeSearchTree45DegreeOracle",
                  "neighbours45": "Sorted45DegreeRoomNeighboursOracle"}[args.oracle]
    source = ROOT / "scripts/autorouter" / (main_class.rsplit(".", 1)[-1] + ".java")
    expected = ROOT / f"qa/data/pcbnew/autorouter/{args.oracle}-search-a11c0a42.txt"
    sources = [source]
    if args.oracle == "spring":
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
        if drill:
            raw = actual.read_text()
            (output / "oracle-raw.txt").write_text(raw)
            # PolylineArea logs a timestamped warning for degenerate holes.
            # Retain raw diagnostics; compare only declared oracle records.
            actual.write_text(normalize_drill_records(raw))
            metadata["normalization"] = "only known degenerate-hole warnings removed; all raw diagnostics retained in oracle-raw.txt"
        metadata["actual_sha256"] = sha256(actual)
        metadata["matches"] = actual.read_bytes() == expected.read_bytes()
        print(f"Pinned {args.oracle} oracle: " + ("PASS" if metadata["matches"] else "FAIL"))
        return 0 if metadata["matches"] else 1
    finally:
        (output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
