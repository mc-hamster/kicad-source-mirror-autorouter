# Native autorouter engineering notes

* [Requirements](autorouter-requirements.txt) — product scope and acceptance criteria.
* [Technical discovery](technical-discovery.md) — KiCad/Freerouting seams and risk analysis.
* [UPSTREAM.md](UPSTREAM.md) — pinned source commit and filename-level mapping.
* [Freerouting Codex sync runbook](FREEROUTING-CODEX-SYNC.md) — direct-from-GitHub PR inventory,
  autorouter-only porting rules, ledger format, validation gates, and target-PR traceability.
* [Regression corpus](regression-corpus.yml) — parity cases and required measurements.

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
  board/      # KiCad adapter and RoutingBoard boundary
  events/     # Immutable worker event values/listener aliases
```

The remaining files at the root are KiCad UI, worker, proposal and preview integration.  They do
not enter the routing algorithm and therefore do not need a Freerouting GUI counterpart.  The
source tree intentionally includes corresponding Freerouting filenames even where KiCad's
immutable snapshot/transaction model replaces a mutable Java board event or thread pool.

The current native search combines deterministic layer-aware maze expansion with an obstacle
visibility graph and retry-dependent negotiated congestion.  The batch pipeline also has a
bounded best-state history, repeated negotiated passes, a Freerouting-style SMD fanout snapshot
stage, plane-via costs, and route cleanup.  The expansion-room classes are real synchronization
seams, not a claim that empirical parity has already been demonstrated.  The corpus manifest is
the release gate for that claim.

Drilled pads and existing vias are represented by separate hole obstacles.  The worker captures
KiCad's copper-to-hole and hole-to-hole constraints: new via/drill collisions remain blocked,
while same-net pad copper and already-existing same-net via copper use KiCad's legal
copper-sharing semantics.  This closes a common gap between snapshot collision checks and KiCad's
live post-commit DRC.
