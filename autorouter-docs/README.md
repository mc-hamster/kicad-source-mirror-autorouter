# Native autorouter engineering notes

Latest core work: [active exact-octagonal ordinary multilayer search](OCTAGONAL-MULTILAYER-SEARCH.md).
This is still a partial implementation, not full Freerouting parity.

**Status: experimental; Freerouting parity has not been achieved.** See
[Complete parity checklist](PARITY-CLOSURE-CHECKLIST.md) for the current implementation audit,
actual port progress, and same-board KiCad validation. The mappings below must
not be interpreted as proof that the named upstream algorithms are implemented.

* [Complete parity checklist](PARITY-CLOSURE-CHECKLIST.md) — all sixteen remaining
  core/integration areas, production host validation/repair, and latest A/B evidence.
* [Exact-octagonal ordinary multilayer search](OCTAGONAL-MULTILAYER-SEARCH.md) —
  active room/door/drill-layer checkpoint, safety gates, and remaining fanout/free-drill gaps.
* [Java → C++ destination translation audit](DESTINATION-DISTANCE-PARITY.md) —
  latest method-level port, bit-exact Java oracle, plane start-set fix and remaining substitutions.
* [Active multilayer room/drill search](DRILL-SEARCH-PARITY.md) — previous core
  port slice, pinned Java drill oracle, same-board A/B results and failed gates.
* [Active room/door search](ROOM-DOOR-SEARCH.md) — rectangular no-via slice,
  pinned Java primitive oracles, real-board results and explicit remaining gaps.
* [Mutable copper rework](MUTABLE-COPPER-REWORK.md) — preceding contact-model,
  trace-interior-targeting and fanout-cleanup milestone.
* [Core parity review and changes](CORE-ROUTING-PARITY-REVIEW.md) — historical active-path audit,
  verified fixes, rejected performance experiment, and remaining replacement work.
* [Requirements](autorouter-requirements.txt) — product scope and acceptance criteria.
* [Technical discovery](technical-discovery.md) — KiCad/Freerouting seams and risk analysis.
* [UPSTREAM.md](UPSTREAM.md) — pinned source commit and filename-level mapping.
* [Freerouting Codex sync runbook](FREEROUTING-CODEX-SYNC.md) — direct-from-GitHub PR inventory,
  autorouter-only porting rules, ledger format, validation gates, and target-PR traceability.
* [DAC2020 Freerouting A/B test runbook](FREEROUTING-DAC2020-AB-RUNBOOK.md) — agent procedure
  for testing native routing against a pinned Freerouting reference on the ten KiCad corpus boards.
* [Regression corpus](regression-corpus.yml) — parity cases and required measurements.
* [Three small online-board benchmarks](BASIC-BOARD-BENCHMARK.md) — official
  Freerouting 2.3.0 versus native, repeatable downloads, timings, DRC, and saved PCBs.

Metric JSON emitted by a corpus harness can be compared with:

```text
python3 scripts/autorouter/compare_results.py reference.json native.json
```

The native headless corpus runner is built with the QA tools and can execute every case in the
manifest without opening pcbnew or modifying a board file:

```text
python3 scripts/autorouter/run_native_corpus.py \
  --binary build/autorouter/qa/tools/autorouter_parity/qa_autorouter_parity \
  --output-dir build/autorouter/corpus
```

`qa_autorouter_parity` also accepts `--include-net NET` for bounded diagnosis,
`--no-vias` for a constrained room-search run,
`--max-nets N` for a stable bounded smoke slice, `--max-connections N` for a bounded
multi-pad/plane-net diagnosis, and `--dump-snapshot` for inspecting the immutable adapter input.
The QA-only
`--export-dsn FILE` option exports the freshly loaded board for a pinned upstream Freerouting
run; DSN/SES conversion is not used by the editor autorouter itself.  A typical same-board
comparison starts with:

```text
build/autorouter/qa/tools/autorouter_parity/qa_autorouter_parity \
  --board qa/data/pcbnew/pns_regressions/boards/simple.kicad_pcb \
  --export-dsn build/autorouter/simple.dsn
java -jar /path/to/freerouting.jar -de build/autorouter/simple.dsn \
  -do build/autorouter/simple.ses -mp 1 -mt 0 --gui.enabled=false
```

Keep the reference checkout and its build outside this source tree.  The native harness can then
be run on the untouched `.kicad_pcb` and the resulting JSON compared with the reference metrics
recorded from the `.ses`/reference log.

Each result includes the worker-side rule count and, when live DRC is enabled, the baseline and
post-route KiCad DRC counts plus the newly introduced count.  Use the latter when the input board
already contains intentional DRC markers.  Run the pinned Freerouting reference separately on the
same untouched boards, export the same metric keys, and compare each pair with the comparator above.
The comparator accepts `--max-new-kicad-drc N` as an explicit native-host safety gate in addition
to its reference-result quality thresholds.

The native source intentionally mirrors the Freerouting routing package where practical:

```text
pcbnew/autorouter/
  maze/       # AutorouteEngine, maze cost/search/expansion/rip-up/cleanup classes
  expansion/  # Freerouting-shaped rooms, doors, neighbours and visibility graph
  drill/      # Snapshot-safe drill pages and via-expansion data
  path/       # Connection and found-path locator/inserter seams
  pipeline/   # BatchAutorouter, pass/thread/fanout/optimizer/pipeline classes
  board/      # KiCad adapter, private host session, mutable copper facade/search tree
  datastructures/ # Ported minimum-area tree
  geometry/planar/ # Rectangle and floating geometry primitives
  events/     # Immutable worker event values/listener aliases
```

The host session constructs an isolated native in-memory board copy on the editor
thread. The worker runs data-only routing plus private-board refill/DRC and bounded
repairs. No live editor geometry, view, or undo state is touched. Acceptance is
blocked unless host validation completed and no new design-rule violations were
found; an explicitly reviewed, safe partial route can still be accepted.

The remaining files at the root are KiCad UI, worker, proposal and preview integration.  They do
not enter the routing algorithm and therefore do not need a Freerouting GUI counterpart.  The
source tree intentionally includes corresponding Freerouting filenames even where KiCad's
immutable snapshot/transaction model replaces a mutable Java board event or thread pool.

Ordinary single-layer and multilayer attempts now use exact-octagonal
room/door geometry and 45-degree corridor location. Multilayer search adds the
source-shaped rectangular drill-page layer and exact full-stack host via
preflight; fanout temporarily retains the qualified rectangular room frontier.
The source destination estimate is translated and used by both frontiers; the
remaining legacy grid/visibility fallback has a separately named heuristic.
General convex free-drill regions, complete forced insertion/shove, source
first-drill fanout ordering, and optimizer equivalence are still incomplete. See the
[latest translation audit](DESTINATION-DISTANCE-PARITY.md): matching filenames
or isolated numeric methods does not establish whole-engine parity.

Drilled pads and existing vias are represented by separate hole obstacles.  The worker captures
KiCad's copper-to-hole and hole-to-hole constraints: new via/drill collisions remain blocked,
while same-net pad copper and already-existing same-net via copper use KiCad's legal
copper-sharing semantics.  This closes a common gap between snapshot collision checks and KiCad's
live post-commit DRC.
