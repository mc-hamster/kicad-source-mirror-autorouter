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
import math
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

# Both outputs are materialized in KiCad, including the imported reference
# session. Missing/skipped host validation must never be treated as zero.
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
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    result = float(value)
    return result if math.isfinite(result) and result >= 0 else None


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
    if (not reference.get("input_board_sha256")
            or reference.get("input_board_sha256") != native.get("input_board_sha256")
            or not reference.get("input_files_sha256")
            or reference.get("input_files_sha256") != native.get("input_files_sha256")
            or reference.get("strip_tracks") != native.get("strip_tracks")):
        print("input identity/normalization mismatch", file=sys.stderr)
        failed = True

    if reference.get("routing_constraints") != native.get("routing_constraints"):
        print("routing constraint mismatch", file=sys.stderr)
        failed = True

    # Synthetic fanout edges are implementation-specific work, not electrical
    # completion. Both results must be measured with KiCad on materialized boards.
    reference_remaining = number(reference, "kicad_unconnected")
    native_remaining = number(native, "kicad_unconnected")
    reference_initial = number(reference, "baseline_kicad_unconnected")
    native_initial = number(native, "baseline_kicad_unconnected")
    if (reference_remaining is None or native_remaining is None
            or reference_initial is None or native_initial is None
            or reference_initial != native_initial
            or native_remaining > reference_remaining):
        print("electrical connectivity check: FAILED (missing, incomparable, or regressed)",
              file=sys.stderr)
        failed = True
    elif (100.0 * (reference_remaining - native_remaining) / max(1.0, native_initial)
          < args.min_completion_delta):
        failed = True

    if reference.get("validation_complete") is not True or native.get("validation_complete") is not True:
        print("KiCad validation was not completed for both results", file=sys.stderr)
        failed = True

    # Historical measurements predate the production host session. When the
    # current harness advertises it, require that path and cross-check its
    # connectivity against independent materialization. A bounded/raw worker
    # run is diagnostic evidence, not production parity.
    if "host_validated" in native and (
            native.get("host_validated") is not True
            or number(native, "host_unconnected") != native_remaining
            or ("host_new_drc_violations" in native and (
                number(native, "host_new_drc_violations") is None
                or number(native, "host_new_drc_violations")
                != number(native, "new_kicad_drc_errors")))):
        print("production proposal validation missing or disagrees with KiCad", file=sys.stderr)
        failed = True

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
        # Worker percentages have different denominators. Actual electrical
        # completion is checked above, using the common KiCad connectivity model.
        if key == "via_count" and delta > args.max_via_increase:
            failed = True
        if key == "track_length_mm":
            allowed = max(abs(expected) * args.max_track_length_increase_percent / 100.0, 0.001)
            if delta > allowed:
                failed = True

    for key in NATIVE_DRC_METRICS:
        actual = number(native, key)
        expected = number(reference, key)
        if actual is None or expected is None:
            print(f"{key}: missing or invalid", file=sys.stderr)
            failed = True
            continue

        print(f"{key:<24} {expected:>14.3f} {actual:>14.3f} {actual - expected:>14.3f}")
        if key == "new_kicad_drc_errors" and actual > args.max_new_kicad_drc:
            failed = True

    if failed:
        print("parity check: FAILED", file=sys.stderr)
        return 1

    print("parity check: PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
