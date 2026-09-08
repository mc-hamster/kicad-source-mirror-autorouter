#!/usr/bin/env python3
"""Fetch three pinned public boards and benchmark both engines serially.

The manifest pins original bytes and the official release JAR. Downloads and
saved boards stay in an ignored build directory, not in the source corpus.
"""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import sys
import urllib.request


ROOT = Path(__file__).resolve().parents[2]


def download(url: str, checksum: str, destination: Path) -> None:
    request = urllib.request.Request(url, headers={"User-Agent": "KiCad-autorouter-QA"})
    with urllib.request.urlopen(request, timeout=60) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != checksum:
        raise ValueError(f"SHA-256 mismatch: {url}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--timeout-seconds", type=int, default=180)
    args = parser.parse_args()
    if args.repeats < 1 or args.timeout_seconds < 1:
        parser.error("repeats and timeout must be positive")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest_path = ROOT / "autorouter-docs/benchmarks/online-simple-boards.json"
    manifest = json.loads(manifest_path.read_text())
    (output / "sources.json").write_text(json.dumps(manifest, indent=2) + "\n")
    boards = {}
    for item in manifest["files"]:
        destination = output / "inputs" / item["case"] / Path(item["path"]).name
        download(item["url"], item["sha256"], destination)
        if destination.suffix == ".kicad_pcb":
            boards[item["case"]] = destination
    reference = manifest["reference"]
    jar = output / "reference" / "freerouting-2.3.0.jar"
    download(reference["url"], reference["sha256"], jar)
    measurements = []
    failed = False
    # No parallel engines: CPU contention would invalidate a small-board timing.
    for repeat in range(1, args.repeats + 1):
        for case, board in boards.items():
            run_dir = output / "runs" / case / f"repeat-{repeat}"
            run_dir.parent.mkdir(parents=True, exist_ok=True)
            command = [sys.executable, str(ROOT / "scripts/autorouter/run_parity_case.py"),
                       "--board", str(board), "--reference-jar", str(jar),
                       "--reference-commit", reference["commit"], "--output-dir", str(run_dir),
                       "--strip-tracks", "--save-boards", "--max-passes", "4",
                       "--max-iterations", "8", "--max-expanded-nodes", "250000",
                       "--timeout-seconds", str(args.timeout_seconds)]
            print(f"Running {case}, repeat {repeat}/{args.repeats}", flush=True)
            with (run_dir.parent / f"repeat-{repeat}.log").open("w") as log:
                result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            entry = {"case": case, "repeat": repeat, "exit_code": result.returncode,
                     "directory": str(run_dir), "command": command}
            for engine in ("reference", "native"):
                path = run_dir / f"{engine}.json"
                if path.is_file():
                    entry[engine] = json.loads(path.read_text())
            measurements.append(entry)
            failed |= result.returncode != 0
            (output / "measurements.json").write_text(json.dumps(measurements, indent=2) + "\n")
            print(f"  comparison exit code: {result.returncode}", flush=True)
    summary = {}
    for case in boards:
        summary[case] = {}
        for engine in ("reference", "native"):
            results = [entry[engine] for entry in measurements
                       if entry["case"] == case and engine in entry]
            times = [result["routing_core_ms"] for result in results
                     if result.get("routing_core_ms") is not None]
            summary[case][engine] = {
                "completed_measurements": len(results),
                "routing_core_ms_median": statistics.median(times) if times else None,
                "routing_core_ms_range": [min(times), max(times)] if times else None,
                "actual_unconnected": [result["kicad_unconnected"] for result in results],
                "new_drc": [result["new_kicad_drc_errors"] for result in results],
                "vias": [result["via_count"] for result in results],
                "track_length_mm": [result["track_length_mm"] for result in results],
            }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
