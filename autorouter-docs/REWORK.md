# Native autorouter rework

The native autorouter has **not reached Freerouting parity**. The existing
grid/visibility search, sampled plane targets, and path-shortening optimizer
are experimental replacements, not faithful translations. Matching filenames
in `UPSTREAM.md` describe intended correspondence, not completed ports.

## Required replacement boundary

Keep the native action, worker lifecycle, preview, rejection, and `BOARD_COMMIT`
integration. Replace the algorithm behind that boundary with a private mutable
routing model that preserves upstream item connectivity, compensated convex
geometry, search-tree updates, room/door search, forced insertion, shove,
rip-up, and pull-tight behavior. An immutable KiCad input snapshot does not
require an immutable algorithm-internal board. Do not reduce these operations
to collision queries just to avoid a private routing model.

The reference checkout is read-only. The pinned commit is
`a11c0a42d1b3827e5126429c5c9820c4ab5bec7c`.

## Changes implemented in this rework

- Search accepts sets of start and destination pad terminals. Backtracking
  records the actual starting terminal and never appends unchecked copper to
  the nominal source. Ordinary batch tasks choose from disconnected terminals
  toward the selected connected component. This is a **pad-terminal subset**
  of upstream connected-item sets; trace/plane-region targets remain missing.
- The adapter captures retained pad components from KiCad's actual copper
  clusters. Ratsnest endpoints resolve to pads several tracks/vias away using
  the anchor's cluster, not only directly adjacent items. Pad identity uses an
  object-to-index mapping rather than matching positions. Components are not
  reused when existing copper is scheduled for replacement.
- Ordinary searches can use both original SMD pads and the landings of completed
  fanouts. Failed landings never seed a search. Explicit terminal sets cannot be
  bypassed by the experimental direct-fanout shortcut.
- Task completion is recomputed from surviving connection topology rather than
  route record counts. Duplicate routes and stale one-node logical completions
  no longer hide an unrouted branch or a ripped-up bridge. This removes the
  observed >100% progress bug; it does not replace host connectivity.
- Destination estimates consider all destination layer groups rather than
  forcing the selected pad or prioritizing a distant same-layer target.
- Rip-up removes conflicts identified by a found route, counts those removals
  against the configured budget, and no longer removes arbitrary unrelated
  successful routes after a failed search.
- Per-connection attempt settings are honoured. Dense-board searches no longer
  silently coarsen the initial grid or override the configured node budget.
  This restores truthful settings; it is not a performance/parity solution.
- Removed pad-endpoint exceptions that ignored foreign-obstacle collisions.
  New via drills cannot overlap existing drills just because nets match, and
  same-net proposed vias must also obey drill spacing.
- History prioritizes clearance correctness ahead of completion and then via
  count and length, using explicit ordered comparisons rather than magic weights.
- UI labels distinguish routing tasks from verified electrical completion.
- Added the direct `ShapeSearchTree90Degree.restrainShape` translation and the
  incomplete room's contained-shape state. Its 512 ordered outputs are generated
  by the pinned Java implementation and checked by the native tests. **This
  primitive is not yet wired into an active room search.** Tree traversal,
  complete-room construction, 45-degree/general convex shapes, and doors remain
  to be ported before replacing the experimental search.

## Independent acceptance measurements

`qa_autorouter_parity` materializes the proposal in a disposable KiCad board,
applies removals, refills zones, rebuilds connectivity, and runs KiCad DRC.

- `kicad_unconnected` is the actual remaining whole-board ratsnest count.
- `complete` requires zero actual unconnected items and zero new KiCad violations.
- `worker_complete` and worker task percentages are diagnostics only.
- `validation_complete: false` and negative/missing DRC counts cannot pass parity.
- New violations are compared by code, location, layer and involved item IDs,
  rather than subtracting category totals. Unconnected-item reports are measured
  separately by connectivity. These fingerprints can be conservative if a
  pre-existing violation's reported location changes; investigate such cases.
- Track length and via count are measured from the materialized board for both
  engines. Full-board comparisons must not use truncated net/connection slices.

Run a reproducible pair (the output directory must not already exist):

```sh
python3 scripts/autorouter/run_parity_case.py \
  --board qa/data/pcbnew/pns_regressions/boards/simple.kicad_pcb \
  --reference-jar /path/to/pinned/freerouting-current-executable.jar \
  --output-dir build/autorouter/parity/simple \
  --strip-tracks
```

The runner checks the JAR revision, records binary/input hashes and exact commands,
exports freshly refilled input, executes Java, imports its session into a separate
disposable board, and evaluates both outputs using KiCad. Java and DSN/SES are
test dependencies only. The production editor does not invoke them. Both runs
disable final optimization to isolate routing; equivalent via/trace costs,
neckdown behavior, and exported versus host constraints still require explicit
comparison before treating the pair as a final quality baseline.

The reference uses a fresh private user-data directory, one routing thread, and
an environment without inherited `FREEROUTING__*` or Java option overrides.
Analytics are disabled. Automatic neckdown is explicitly requested off, but this
is **not evidence that effective widths match**: the reference result below still
has minimum-width violations in KiCad. Resolve that discrepancy before claiming
matched design constraints. Input identity includes `.kicad_pro` and `.kicad_dru`
hashes (including absence), not just the PCB file. Older JSON outputs without
these identity/validation fields intentionally cannot pass the comparator.

`--strip-tracks` removes unlocked tracks/vias in memory for both engines. Running
an already routed corpus board without stripping it is not evidence of autorouter
completion ability. Input files are never saved.

## Verified continuation results (2026-09-07)

- Rebuilt the native PCB module and both QA executables. All **32**
  `NativeAutorouter` tests pass, including 512 Java-oracle room-restraint cases,
  retained-copper clusters, explicit fanout terminal sets, and bridge rip-up
  accounting. All **9** Python parity-gate tests pass. The entire KiCad QA suite
  has not been run in this continuation.
- Stripped `net_chains/chain_bridging_4pad.kicad_pcb`: both engines reach zero
  unconnected items with no new KiCad DRC violations, 4 mm track length, no vias.
  This is a small smoke fixture, not evidence of complex-board parity.
- Stripped `pns_regressions/boards/simple.kicad_pcb`: input has 36 actual missing
  connections. At four routing passes, four native attempts per request, and
  10,000 native expanded nodes per attempt, native leaves **one** actual missing
  connection, uses **42 vias**, and introduces **22 dangling-via DRC reports**.
  Reference reaches zero missing connections, uses 12 vias, and has 25 new KiCad
  reports (23 minimum-width, two dangling-via). **Parity fails.**
- At the smaller two-pass/two-attempt/1,000-node checkpoint, native improved from
  nine missing connections before the component fixes to two after them. Worker
  progress now agrees with that count on this fixture.
- A diagnostic with fanout disabled leaves four connections missing but creates
  no new KiCad violations. Disabling fanout is not the solution: it trades away
  completion and does not supply upstream's missing topology-aware insertion,
  fanout reuse and redundant-copper cleanup.

Fresh final artifacts live under `build/autorouter/rework/chain-isolated-final/`
and `build/autorouter/rework/simple-isolated-final/`. Earlier runs without isolated
reference settings are diagnostic history, not reproducible acceptance baselines.

## Next required work

1. Finish the upstream geometry/search-tree dependency port with differential
   primitive tests, preserving obstacle traversal and tie ordering.
2. Add a private mutable routing board with actual connected-item sets and
   topology invalidation on insertion/removal. Preserve filled island identity.
3. Port complete rooms, door sections, drill expansion, and maze backtracking;
   switch the active backend only when geometric search tests pass.
4. Port forced insertion, shove and pull-tight with atomic rollback. Replacing
   `MazeTraceShover` with another local path shortcut does not satisfy this step.
5. Port batch selection, fanout, rip-up and optimizer against those operations.
6. Demonstrate full-board parity on clean fixtures and the user's replacement
   test asset. The former stripped Arduino board was deleted by the user; older
   temporary DSN/SES files are not a verified replacement acceptance fixture.

Do not report the product fixed or the port complete while these steps remain.

## Verification

```sh
cmake --build build/autorouter --target qa_autorouter_parity qa_pcbnew -j4
build/autorouter/qa/tests/pcbnew/qa_pcbnew --run_test=NativeAutorouter
python3 -m unittest discover -s scripts/autorouter -p 'test_*.py'
git diff --check
```

To regenerate the room-restraint oracle, compile `scripts/autorouter/RoomRestraintOracle.java`
against the pinned executable JAR in a build directory, execute it there, and
retain the lines prefixed `CASE ` without that prefix in
`qa/data/pcbnew/autorouter/room-restraint-a11c0a42.txt`. Production native tests
read the checked-in outputs and do not require Java.
