# DAC2020 Freerouting A/B test runbook

This is the executable procedure for an agent testing KiCad's native
autorouter against Freerouting on the ten checked-in DAC2020 KiCad projects.
The corpus belongs to the sibling Freerouting checkout; it is read-only input,
not a source tree to modify.

## What is being tested

Use an untouched `*.unrouted.kicad_pcb` as the common input.  The test runner
exports that board once to DSN, routes the DSN with a pinned Freerouting JAR,
routes the equivalent native snapshot, materializes both results in KiCad, and
compares KiCad connectivity and DRC results.  It does not compare bytes in an
SES file or require the same trace geometry.

The corpus is documented at:

```text
/Users/jmcasler/Documents/GitHub/mchamster/freerouting/fixtures/Issue508-DAC2020/README.md
```

Start with one of the seven complete historical cases: `bm01`, `bm02`, `bm04`,
`bm07`, `bm08`, `bm09`, or `bm10`.  Use `bm02`, `bm07`, or `bm08` for a quick
smoke test.  `bm05`, `bm06`, and `bm11` are known-partial historical outputs;
they are diagnostic capability cases and must never be labeled a successful
completion baseline.

## Preconditions

From the KiCad checkout:

```sh
git status --short
git diff --check
test -x build/autorouter/qa/tools/autorouter_parity/qa_autorouter_parity || \
  cmake --build build/autorouter --target qa_autorouter_parity -j4
python3 scripts/autorouter/test_compare_results.py
```

The working tree may contain unrelated user work.  Preserve it: do not reset,
clean, stash, or write to either checkout.  Java 25 must be available.  The
output directory used below must be new; the runner refuses to reuse one.

Set the paths explicitly.  Do not use a mutable `freerouting-current.jar` for
an acceptance result.

```sh
export FREEROUTING_CHECKOUT=/Users/jmcasler/Documents/GitHub/mchamster/freerouting
export FREEROUTING_JAR="$FREEROUTING_CHECKOUT/scripts/benchmark/binaries/freerouting-2.3.0.jar"
export FREEROUTING_REF_SHA=0291acb4e601cc2f6e6f9627557c087adfe9b4f2
export DAC_BOARD="$FREEROUTING_CHECKOUT/fixtures/Issue508-DAC2020/DAC2020_bm02/bm2.unrouted.kicad_pcb"
test -f "$FREEROUTING_JAR" && test -f "$DAC_BOARD"
unzip -p "$FREEROUTING_JAR" META-INF/MANIFEST.MF | tr -d '\r' | \
  rg "Build-Revision: $FREEROUTING_REF_SHA"
```

The `Build-Revision` check is mandatory: `run_parity_case.py` rejects a JAR
whose manifest revision differs from `--reference-commit`.

## Run one complete-board A/B test

```sh
RUN_DIR="build/autorouter/dac2020-ab-bm02-$(date +%Y%m%d-%H%M%S)"

python3 scripts/autorouter/run_parity_case.py \
  --board "$DAC_BOARD" \
  --binary build/autorouter/qa/tools/autorouter_parity/qa_autorouter_parity \
  --reference-jar "$FREEROUTING_JAR" \
  --reference-commit "$FREEROUTING_REF_SHA" \
  --output-dir "$RUN_DIR" \
  --save-boards \
  --max-passes 4 \
  --max-iterations 8 \
  --max-expanded-nodes 250000 \
  --timeout-seconds 600
```

The process exit code is the initial gate.  A zero exit means the comparator
found no connectivity regression, both outputs completed host/KiCad validation,
the native output introduced no KiCad DRC reports, and it stayed within the
configured via and trace-length limits.  A non-zero exit is a test failure or
an incomparable run, never an invitation to weaken the gate silently.

Inspect and retain these artifacts for every run:

```sh
cat "$RUN_DIR/provenance.json"
cat "$RUN_DIR/reference.json"
cat "$RUN_DIR/native.json"
tail -n 100 "$RUN_DIR/reference-route.log"
```

`provenance.json` records input hashes, JAR hash/revision, native-binary hash,
effective commands, and routing constraints.  The runner also confirms that
the original PCB, project, and rules files were not modified.

## Pass criteria

A native result passes only when all of these are true:

1. Both measurements identify the same input board/project/rules hashes and
   routing constraints.
2. Both outputs completed KiCad validation.
3. Native KiCad-unconnected count is no greater than Freerouting's count.
4. Native newly introduced KiCad DRC count is zero.
5. The configured score gates (default: no more vias and no more than 10% extra
   track length) pass.

Trace shape, exact coordinate sequence, route ordering, and SES bytes are
non-gates.  Different but DRC-clean paths are valid because routing tie-breaks
and optimization can vary between implementations and releases.

## Corpus sweep

Run the complete cases serially, with a new output directory per board.  Begin
with the three small boards, then run the larger boards only after those are
clean.  Do not use `bm05`, `bm06`, or `bm11` as a zero-unrouted expectation.

```sh
for board in 02 07 08 01 04 09 10; do
  number=$((10#$board))
  board_path="$FREEROUTING_CHECKOUT/fixtures/Issue508-DAC2020/DAC2020_bm${board}/bm${number}.unrouted.kicad_pcb"
  out="build/autorouter/dac2020-ab-bm${board}-$(date +%Y%m%d-%H%M%S)"
  python3 scripts/autorouter/run_parity_case.py \
    --board "$board_path" \
    --binary build/autorouter/qa/tools/autorouter_parity/qa_autorouter_parity \
    --reference-jar "$FREEROUTING_JAR" \
    --reference-commit "$FREEROUTING_REF_SHA" \
    --output-dir "$out" --save-boards --timeout-seconds 1800 || exit $?
done
```

Run only one router process at a time; parallel routing changes memory pressure
and makes time/peak-memory measurements incomparable.  Record wall time and
peak heap as scorecard data, not as a correctness substitute.

## Historical v2.1.0 sessions

The corpus also includes frozen v2.1.0 SES files and logs.  Keep those as
compatibility evidence, but do not use them for the current-quality gate: they
were produced with different routing behavior and `bm05`, `bm06`, and `bm11`
are explicitly partial.  The v2.3.0 run above deliberately regenerates the
reference from the same exported DSN and pins its exact JAR revision.

If an historical session differs from a fresh v2.3.0 result, report the
difference as **version drift**.  It becomes a native regression only if the
native result regresses against the pinned v2.3.0 output on the same input and
settings.  Never overwrite the checked-in v2.1.0 files to make a comparison
pass.

## Failure triage

First classify the failure before editing routing code:

| Observation | Classification | Next action |
| --- | --- | --- |
| input hashes or constraints differ | invalid comparison | fix setup; rerun from untouched input |
| reference JAR revision mismatch | invalid reference | obtain/pin the intended JAR and SHA |
| native adds DRC errors | safety regression | inspect saved `native.kicad_pcb` and DRC metrics |
| native has more KiCad-unconnected items | completion regression | inspect the first failed net with `--include-net` |
| valid connectivity/DRC but score gate fails | quality regression | compare vias/length; do not call it parity |
| only geometry/SES bytes differ | expected topology drift | retain metrics and continue |

For a bounded net-level diagnosis, rerun the QA binary with `--include-net NET`
or a modest `--max-connections N`; treat that as diagnostic evidence only, then
repeat the full-board test before accepting a change.
