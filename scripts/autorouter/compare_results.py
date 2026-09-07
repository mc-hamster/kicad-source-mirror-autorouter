#!/usr/bin/env python3
"""Compare one Freerouting reference metric JSON with one native result JSON.

The runner is intentionally format-agnostic: a board harness can emit the
metric keys from autorouter-docs/regression-corpus.yml without depending on
the KiCad GUI.  A non-zero exit status makes parity regressions suitable for
CI or a local corpus run.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


METRICS = (
    "total_connections",
    "routed_connections",
    "unrouted_connections",
    "completion_percent",
    "drc_errors",
    "track_length_mm",
    "via_count",
    "runtime_ms",
    "ripups",
    "routing_passes",
    "optimization_passes",
)

# These values are native-host health gates.  A Freerouting result cannot have
# KiCad DRC counts, so they are checked only when the native JSON contains
# them.  Keeping the check here prevents a parity run from passing on route
# completeness while silently introducing new KiCad violations.
NATIVE_DRC_METRICS = (
    "baseline_kicad_drc_errors",
    "kicad_drc_errors",
    "new_kicad_drc_errors",
)


def load(path: Path) -> dict[str, object]:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def number(data: dict[str, object], key: str) -> float | None:
    value = data.get(key)
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("native", type=Path)
    parser.add_argument("--max-drc-increase", type=float, default=0)
    parser.add_argument("--min-completion-delta", type=float, default=0.0)
    parser.add_argument("--max-via-increase", type=float, default=0.0)
    parser.add_argument("--max-track-length-increase-percent", type=float, default=10.0)
    parser.add_argument(
        "--max-new-kicad-drc",
        type=float,
        default=0.0,
        help="maximum native DRC violations introduced over the input board",
    )
    args = parser.parse_args()

    reference = load(args.reference)
    native = load(args.native)
    failed = False

    print(f"{'metric':<24} {'reference':>14} {'native':>14} {'delta':>14}")
    print("-" * 70)
    for key in METRICS:
        expected = number(reference, key)
        actual = number(native, key)
        if expected is None or actual is None:
            print(f"{key:<24} {'missing':>14} {'missing':>14} {'n/a':>14}")
            failed = True
            continue

        delta = actual - expected
        print(f"{key:<24} {expected:>14.3f} {actual:>14.3f} {delta:>14.3f}")

        if key == "drc_errors" and delta > args.max_drc_increase:
            failed = True
        if key == "completion_percent" and delta < args.min_completion_delta:
            failed = True
        if key == "via_count" and delta > args.max_via_increase:
            failed = True
        if key == "track_length_mm":
            allowed = max(abs(expected) * args.max_track_length_increase_percent / 100.0, 0.001)
            if delta > allowed:
                failed = True

    for key in NATIVE_DRC_METRICS:
        actual = number(native, key)
        if actual is None:
            continue

        print(f"{key:<24} {'n/a':>14} {actual:>14.3f} {'n/a':>14}")
        if key == "new_kicad_drc_errors" and actual > args.max_new_kicad_drc:
            failed = True

    if failed:
        print("parity check: FAILED", file=sys.stderr)
        return 1

    print("parity check: PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
