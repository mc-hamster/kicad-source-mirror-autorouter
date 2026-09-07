#!/usr/bin/env python3
"""Run the headless native autorouter over the documented regression corpus.

The script intentionally does not depend on PyYAML.  The corpus manifest is a
small, reviewable YAML file whose ``board:`` entries are sufficient for this
runner.  It writes one JSON result per case and a machine-readable index so a
later Freerouting run can be compared without changing the board files.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


def cases_from_manifest(manifest: Path) -> list[tuple[str, Path]]:
    """Read case ids and board paths from the deliberately simple manifest."""

    cases: list[tuple[str, Path]] = []
    current_id: str | None = None

    for raw_line in manifest.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if line.startswith("- id:"):
            current_id = line.split(":", 1)[1].strip()
        elif line.startswith("board:") and current_id:
            board = Path(line.split(":", 1)[1].strip())
            cases.append((current_id, board))
            current_id = None

    if not cases:
        raise ValueError(f"no corpus cases found in {manifest}")

    return cases


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("autorouter-docs/regression-corpus.yml"),
    )
    parser.add_argument(
        "--binary",
        type=Path,
        default=Path("build/autorouter/qa/tools/autorouter_parity/qa_autorouter_parity"),
    )
    parser.add_argument("--output-dir", type=Path, default=Path("build/autorouter/corpus"))
    parser.add_argument(
        "--root",
        type=Path,
        default=Path.cwd(),
        help="repository root used to resolve relative board paths and the binary",
    )
    parser.add_argument("--max-passes", type=int, default=4)
    parser.add_argument("--max-iterations", type=int, default=8)
    parser.add_argument("--optimization-passes", type=int, default=2)
    parser.add_argument("--max-expanded-nodes", type=int, default=250000)
    parser.add_argument(
        "--max-nets",
        type=int,
        default=0,
        help="limit each board to a stable prefix of nets for bounded smoke runs",
    )
    parser.add_argument(
        "--max-connections",
        type=int,
        default=0,
        help="limit each selected net to a bounded number of connection items",
    )
    parser.add_argument("--no-fanout", action="store_true")
    parser.add_argument("--all-connections", action="store_true")
    parser.add_argument("--no-live-drc", action="store_true")
    parser.add_argument(
        "--continue-on-error",
        action="store_true",
        help="write failures into the index and continue with later boards",
    )
    args = parser.parse_args()

    manifest = args.manifest.resolve()
    root = args.root.resolve()
    binary = args.binary if args.binary.is_absolute() else root / args.binary
    output_dir = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        cases = cases_from_manifest(manifest)
    except (OSError, ValueError) as error:
        print(error, file=sys.stderr)
        return 2

    index: dict[str, object] = {
        "manifest": str(manifest),
        "binary": str(binary),
        "cases": [],
    }
    failures = 0

    for case_id, relative_board in cases:
        board = relative_board if relative_board.is_absolute() else root / relative_board
        output = output_dir / f"{case_id}.json"
        command = [
            str(binary),
            "--board",
            str(board),
            "--out",
            str(output),
            "--max-passes",
            str(args.max_passes),
            "--max-iterations",
            str(args.max_iterations),
            "--optimization-passes",
            str(args.optimization_passes),
            "--max-expanded-nodes",
            str(args.max_expanded_nodes),
        ]
        if args.max_nets > 0:
            command.extend(["--max-nets", str(args.max_nets)])
        if args.max_connections > 0:
            command.extend(["--max-connections", str(args.max_connections)])
        if args.no_fanout:
            command.append("--no-fanout")
        if args.all_connections:
            command.append("--all-connections")
        if args.no_live_drc:
            command.append("--no-live-drc")

        entry: dict[str, object] = {
            "id": case_id,
            "board": str(relative_board),
            "output": str(output),
        }

        print(f"[{case_id}] {relative_board}", flush=True)
        if not board.exists():
            entry["error"] = f"board does not exist: {board}"
            failures += 1
        elif not binary.exists():
            entry["error"] = f"autorouter binary does not exist: {binary}"
            failures += 1
        else:
            completed = subprocess.run(command, cwd=root, check=False)
            entry["returncode"] = completed.returncode
            if output.exists():
                try:
                    entry["result"] = json.loads(output.read_text(encoding="utf-8"))
                except json.JSONDecodeError as error:
                    entry["error"] = f"invalid result JSON: {error}"

            if completed.returncode != 0:
                failures += 1
                if not args.continue_on_error:
                    index["cases"].append(entry)
                    break

        index["cases"].append(entry)

    index_path = output_dir / "index.json"
    index_path.write_text(json.dumps(index, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {index_path}")

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
