#!/usr/bin/env python3
"""Build the complete Freerouting class-and-method parity inventory.

By default the exact pinned source is fetched from GitHub into a temporary
directory.  ``--reference-root`` exists only for repeatable offline QA and
rejects a git checkout at any other revision.  The protected local reference
checkout is never a default and is never modified.
"""

from __future__ import annotations

import argparse
import collections
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "autorouter-docs/parity-classifications.json"
OUTPUT = ROOT / "autorouter-docs/parity-method-manifest.json"
EXTRACTOR = Path(__file__).with_name("JavaMethodManifest.java")
JAVA_ROOT = Path("src/main/java/app/freerouting")
VALID_STATUSES = {"PORTED_EXACT", "PORTED_ADAPTED", "PARTIAL", "MISSING", "EXCLUDED"}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, text=True, check=True, **kwargs)


def obtain_reference(config: dict[str, object], requested: Path | None,
                     temporary: Path) -> Path:
    commit = str(config["reference_commit"])
    if requested is not None:
        reference = requested.resolve()
        actual = run(["git", "-C", str(reference), "rev-parse", "HEAD"],
                     stdout=subprocess.PIPE).stdout.strip()
        if actual != commit:
            raise ValueError(f"reference checkout {actual} != pinned {commit}")
        return reference

    reference = temporary / "freerouting"
    run(["git", "init", "-q", str(reference)])
    run(["git", "-C", str(reference), "remote", "add", "origin",
         str(config["reference_repository"])])
    run(["git", "-C", str(reference), "fetch", "-q", "--depth=1", "origin", commit])
    run(["git", "-C", str(reference), "checkout", "-q", "--detach", "FETCH_HEAD"])
    return reference


def source_files(reference: Path, roots: list[str]) -> list[Path]:
    java_root = reference / JAVA_ROOT
    result: set[Path] = set()
    for entry in roots:
        path = java_root / entry
        if path.is_file() and path.suffix == ".java":
            result.add(path)
        elif path.is_dir():
            result.update(path.rglob("*.java"))
        else:
            raise FileNotFoundError(f"configured source scope does not exist: {path}")
    return sorted(result)


def extract(reference: Path, sources: list[Path], java: str) -> list[dict[str, object]]:
    command = [java, str(EXTRACTOR), "--source-root", str(reference / JAVA_ROOT)]
    command.extend(str(path) for path in sources)
    completed = run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    records = [json.loads(line) for line in completed.stdout.splitlines() if line.strip()]
    if not records:
        raise RuntimeError("JDK parser returned an empty source inventory")
    return records


def native_index() -> dict[str, list[str]]:
    result: dict[str, list[str]] = collections.defaultdict(list)
    native_root = ROOT / "pcbnew/autorouter"
    for path in sorted(native_root.rglob("*")):
        if path.suffix not in (".h", ".cpp"):
            continue
        result[path.stem].append(path.relative_to(ROOT).as_posix())
    return dict(result)


def classification(owner: str, native: dict[str, list[str]],
                   config: dict[str, object]) -> tuple[str, list[str], str]:
    exact = config["exact_classes"]
    adapted = config["adapted_classes"]
    aliases = config["native_aliases"]
    if owner in exact:
        item = exact[owner]
        return "PORTED_EXACT", item["native_files"], item["evidence"]
    if owner in adapted:
        item = adapted[owner]
        return "PORTED_ADAPTED", item["native_files"], item["reason"]
    if owner in aliases:
        return "PARTIAL", aliases[owner], "explicit native adaptation; completeness remains open"

    # Nested source helpers normally live in the top-level source-named C++
    # unit.  Preserve that association rather than declaring every nested
    # comparator/record to be a separate missing subsystem.
    top_level = owner.rsplit(".", 1)[-1].split("$", 1)[0]
    candidates = native.get(top_level, [])
    if candidates:
        return "PARTIAL", candidates, "source-named native unit exists; method parity is not proven"
    return "MISSING", [], "no classified native counterpart"


def build(reference: Path, config: dict[str, object], java: str) -> dict[str, object]:
    sources = source_files(reference, config["source_roots"])
    parsed = extract(reference, sources, java)
    native = native_index()
    classes: dict[str, dict[str, object]] = {}
    methods: list[dict[str, object]] = []
    overrides = config.get("method_overrides", {})

    for record in parsed:
        owner = str(record["owner"])
        status, native_files, note = classification(owner, native, config)
        if status not in VALID_STATUSES:
            raise ValueError(f"invalid status {status} for {owner}")
        if record["record"] == "class":
            classes[owner] = {
                "source_path": record["path"],
                "source_line": record["line"],
                "kind": record["kind"],
                "status": status,
                "native_files": native_files,
                "classification_note": note,
            }
            continue

        key = owner + "#" + str(record["signature"])
        method_status = str(overrides.get(key, status))
        if method_status not in VALID_STATUSES:
            raise ValueError(f"invalid method status {method_status} for {key}")
        methods.append({
            "key": key,
            "owner": owner,
            "name": record["name"],
            "signature": record["signature"],
            "constructor": record["constructor"],
            "source_path": record["path"],
            "source_line": record["line"],
            "status": method_status,
            "native_files": native_files,
        })

    # Compiler-generated default constructors are absent from the syntax tree;
    # this is a declared-method inventory, not a bytecode API inventory.
    classes_list = [dict(owner=owner, **value) for owner, value in sorted(classes.items())]
    methods.sort(key=lambda item: (item["source_path"], item["source_line"], item["key"]))
    status_counts = collections.Counter(item["status"] for item in methods)
    source_hashes = {
        path.relative_to(reference / JAVA_ROOT).as_posix(): sha256(path) for path in sources
    }
    return {
        "schema_version": 1,
        "reference": {
            "repository": config["reference_repository"],
            "commit": config["reference_commit"],
            "quality_baseline_commit": config["quality_baseline_commit"],
        },
        "scope": {
            "included": config["source_roots"],
            "excluded_application_roots": config["excluded_application_roots"],
            "rule": "Every declared type and method under included roots is classified.",
        },
        "summary": {
            "source_files": len(sources),
            "classes": len(classes_list),
            "methods": len(methods),
            "method_status_counts": dict(sorted(status_counts.items())),
        },
        "source_sha256": source_hashes,
        "classes": classes_list,
        "methods": methods,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-root", type=Path)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--java", default="java")
    parser.add_argument("--check", action="store_true",
                        help="fail when regenerated output differs from the checked-in manifest")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))

    with tempfile.TemporaryDirectory(prefix="freerouting-parity-manifest-") as temporary_name:
        reference = obtain_reference(config, args.reference_root, Path(temporary_name))
        manifest = build(reference, config, args.java)
    serialized = json.dumps(manifest, indent=2, sort_keys=False) + "\n"

    if args.check:
        if not args.output.is_file() or args.output.read_text(encoding="utf-8") != serialized:
            print(f"parity manifest is stale: regenerate {args.output}", file=sys.stderr)
            return 1
        print(f"parity manifest is current: {manifest['summary']}")
        return 0

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(serialized, encoding="utf-8")
    print(f"wrote {args.output}: {manifest['summary']}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"parity manifest failed: {error}", file=sys.stderr)
        raise SystemExit(2)
