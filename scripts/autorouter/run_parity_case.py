#!/usr/bin/env python3
"""Route one identical in-memory input with native KiCad and pinned Freerouting.

Java/DSN/SES are QA dependencies only. Both outputs are evaluated by KiCad,
including zone refill and actual connectivity. Input files are never saved.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
import zipfile
from pathlib import Path


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reference_routing_ms(log: str) -> float | None:
    """Sum engine-reported fanout/routing stages, excluding JVM startup and I/O.

    Do not silently use process wall time when stage timing is unavailable.
    The pinned engine reports seconds rounded in its human-readable log.
    """
    times = {}
    for stage, seconds in re.findall(
            r"(Fanout|Auto-routing) stage completed:[^\n]*?completed in ([0-9.]+) seconds", log):
        if stage in times:
            return None  # Multiple jobs are not a single-board measurement.
        times[stage] = float(seconds)
    return sum(times.values()) * 1000 if "Auto-routing" in times else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--reference-jar", type=Path, required=True)
    parser.add_argument("--reference-commit", default="a11c0a42d1b3827e5126429c5c9820c4ab5bec7c")
    parser.add_argument("--binary", type=Path,
                        default=Path("build/autorouter/qa/tools/autorouter_parity/qa_autorouter_parity"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--java", default="java")
    parser.add_argument("--strip-tracks", action="store_true")
    parser.add_argument("--save-boards", action="store_true",
                        help="save normalized input and both materialized routing results")
    parser.add_argument("--max-passes", type=int, default=4)
    parser.add_argument("--max-iterations", type=int, default=8)
    parser.add_argument("--max-expanded-nodes", type=int, default=250000)
    parser.add_argument("--timeout-seconds", type=int, default=600)
    args = parser.parse_args()
    board, binary, jar = args.board.resolve(), args.binary.resolve(), args.reference_jar.resolve()
    output = args.output_dir.resolve()
    # A fresh directory prevents stale sessions/results from passing a failed run.
    output.mkdir(parents=True, exist_ok=False)
    original_hash = sha256(board)
    def input_hashes() -> dict[str, str | None]:
        return {suffix: sha256(path) if path.is_file() else None
                for suffix in (".kicad_pcb", ".kicad_pro", ".kicad_dru")
                for path in (board.with_suffix(suffix),)}
    original_inputs = input_hashes()
    with zipfile.ZipFile(jar) as archive:
        manifest = archive.read("META-INF/MANIFEST.MF").decode().replace("\r\n ", "")
    revision = next((line.split(": ", 1)[1] for line in manifest.splitlines()
                     if line.startswith("Build-Revision: ")), None)
    if revision != args.reference_commit:
        raise ValueError(f"reference JAR revision {revision!r} != pinned {args.reference_commit}")

    provenance = {"input_board_sha256": original_hash, "strip_tracks": args.strip_tracks,
                  "input_files_sha256": original_inputs,
                  "reference_commit": revision, "reference_jar_sha256": sha256(jar),
                  "native_binary_sha256": sha256(binary), "commands": []}

    def run(name: str, command: list[str], *, reference: bool = False) -> float:
        print(name, flush=True)
        measurement = {"stage": name, "argv": command}
        provenance["commands"].append(measurement)
        (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
        start = time.monotonic()
        with (output / f"{name}.log").open("w") as log:
            environment = None
            if reference:
                environment = {key: value for key, value in os.environ.items()
                               if not key.upper().startswith("FREEROUTING__")
                               and key not in ("JAVA_TOOL_OPTIONS", "JDK_JAVA_OPTIONS", "_JAVA_OPTIONS")}
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                           timeout=args.timeout_seconds, check=True, env=environment,
                           cwd=output if reference else None)
        measurement["wall_ms"] = (time.monotonic() - start) * 1000
        (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
        return measurement["wall_ms"]

    common = [str(binary), "--board", str(board)]
    if args.strip_tracks:
        common.append("--strip-tracks")
    dsn, ses = output / "input.dsn", output / "reference.ses"
    def save_board(name: str) -> list[str]:
        return ["--save-board", str(output / f"{name}.kicad_pcb")] if args.save_boards else []
    try:
        run("export", common + ["--export-dsn", str(dsn)] + save_board("unrouted"))
        reference_ms = run("reference-route", [args.java, "-jar", str(jar), "-de", str(dsn),
                 "-do", str(ses), "-mp", str(args.max_passes), "-mt", "1",
                 f"--user_data_path={output / 'reference-user-data'}",
                 "--usage_and_diagnostic_data.disable_analytics=true",
                 "--gui.enabled=false", "--router.optimizer.enabled=false",
                 "--router.automatic_neckdown=false"], reference=True)
        if not ses.is_file():
            raise RuntimeError("reference returned without producing a session")
        run("reference-validate", common + ["--import-ses", str(ses),
                                           "--out", str(output / "reference.json")]
                                           + save_board("reference"))
        run("native", common + ["--out", str(output / "native.json"), "--max-passes",
              str(args.max_passes), "--max-iterations", str(args.max_iterations),
              "--max-expanded-nodes", str(args.max_expanded_nodes), "--optimization-passes", "0"]
              + save_board("native"))
        for name in ("reference", "native"):
            path = output / f"{name}.json"
            data = json.loads(path.read_text())
            data.update({"input_board_sha256": original_hash, "strip_tracks": args.strip_tracks})
            data["input_files_sha256"] = original_inputs
            if name == "reference":
                data["runtime_ms"] = reference_ms
                data["routing_core_ms"] = reference_routing_ms(
                        (output / "reference-route.log").read_text())
            else:
                data["routing_core_ms"] = data["runtime_ms"]
            path.write_text(json.dumps(data, indent=2) + "\n")
        if args.save_boards:
            for name in ("unrouted", "reference", "native"):
                for suffix in (".kicad_pro", ".kicad_dru"):
                    companion = board.with_suffix(suffix)
                    if companion.is_file():
                        shutil.copyfile(companion, output / f"{name}{suffix}")
        comparator = Path(__file__).with_name("compare_results.py")
        return subprocess.run([sys.executable, str(comparator), str(output / "reference.json"),
                               str(output / "native.json")], check=False).returncode
    finally:
        if input_hashes() != original_inputs:
            raise RuntimeError("input board, project, or custom rules changed during parity run")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"parity run failed: {error}", file=sys.stderr)
        raise SystemExit(2)
