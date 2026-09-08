# Java → C++ translation audit: destination costs — 2026-09-08

**The whole router is still not a faithful Freerouting port.** This pass replaces
one active algorithmic substitution with a direct translation, fixes a plane
start-set discrepancy, and removes misleading equivalence comments. File names
alone are not evidence of algorithm parity. The full remaining scope is in
[the acceptance checklist](PARITY-CLOSURE-CHECKLIST.md).

## Identity and what was inspected

- Native starting commit: `69f5d3156c` (the previously delivered drill/room work).
- Java source and method oracle:
  `a11c0a42d1b3827e5126429c5c9820c4ab5bec7c`.
- Independent board-quality baseline: official **Freerouting v2.3.0**,
  `2d4de019aa89e9fa3dc1dc44e09bf509760cafc1`.
- The protected Java checkout was read, not modified or built. The QA harness
  invokes a separately built, revision-checked JAR; production never invokes Java.
- Inspected the active single-/multilayer search, legacy heuristic/control,
  connected-set scheduling, private host repair, result inserter, shortening
  routine, and purported multithreaded optimizer against their Java counterparts.
  This is not a fresh line-by-line proof of every file in the two repositories.

The source pin and release baseline are intentionally different revisions.
Source-method agreement must not be confused with release-board quality.

## Concrete substitutions found and corrected

### 1. `DestinationDistance` was not the Java algorithm

There were **three** different estimates: the named grid-normalized fallback
class, a minimum-over-individual-targets single-layer estimate, and a minimum
trace-cost/one-via multilayer estimate. None implemented the reference class.

`maze/DestinationDistance.{h,cpp}` now translates the Java constructor, `join`,
both `calculate` overloads, and `calculateCheapDistance`:

- Separate component/solder/combined-inner bounding boxes; **union of boxes**,
  not minimum distance to individual terminals.
- Same min/max horizontal/vertical cost preparation, active-layer counts,
  branch order, and one-/two-/three-/four-layer estimates.
- Normal and cheap via costs, source empty-box sentinels and maximum-cost return.
- `FloatPoint.boundingBox` floor/ceil, including negative fractions, rather than
  rounding the query to a point. `IntBox.weightedDistance` includes its source
  overlapping-axis branches and weighted Euclidean distance.
- Inactive outer-layer cost fields stay zero, and the source's unit preferred
  inner-layer assumption is retained. They were **not** silently “improved” into
  a different lower bound. The oracle includes these unusual cases.
- Cheap queries use a const helper with an explicit via-cost argument, instead
  of temporarily mutating/restoring the Java member. The result and subsequent
  normal query are tested, not merely the method names.

Both room frontiers now call this class for their initial and subsequent
estimates. Page/drill/layer transitions use the same destination instance.
Destination boxes on inactive layers are joined as in Java; physical ordinals,
not arbitrary KiCad layer IDs, are passed to the estimator. The strictly
single-layer subproblem maps its sole layer to ordinal zero.

The old raster estimate is preserved, unchanged in algorithm, under the explicit
name `LegacyDestinationDistance`. It is not used by the room frontier. The
raster fallback itself still needs replacement; isolating it is not porting it.

### 2. Literal expressions still differed because of floating-point contraction

The initial mechanical translation matched the binary-fraction corpus, but
non-binary directional weights exposed **137 bit-level query differences**.
The first normalized mismatch was record **26**, point query **1**: a one-ULP
weighted-distance difference. This was numeric drift caused by C++ fused
multiply/add contraction, not a different Java branch or target selection.

The translated destination translation unit now disables contraction, including
its inline geometry. CMake scopes `-ffp-contract=off` (GCC/Clang) or `/fp:strict`
(MSVC) to that source; a Clang pragma also covers standalone oracle builds.
No global KiCad floating-point policy or quality threshold was changed.

On Apple Clang 21 / arm64, the same default-compiler before/after comparisons are:

| Query-record checkpoint | Before: unequal double results | After |
|---|---:|---:|
| First 64 records | 1 | 0 |
| All 7,712 records / 30,848 results | 137 | 0 |

The C++ test now checks the **double bit patterns**, not an epsilon that hides
these differences. Explicit `-ffp-contract=fast` was also diagnosed separately;
its different optimization policy produced 123 mismatches, also eliminated.
These are method/rounding checkpoints, **not** end-to-end routing-stream parity
or cross-platform test results. Other unported costs/geometry are not thereby
certified bit-equivalent.

### 3. Plane tasks discarded most of their connected start set

Java `AutorouteConnectionRouter.route` starts plane work from `connectedSet`.
Native batch code instead forced the synthetic fanout landing and its one
selected layer. With the correct destination estimate this produced three vias
in an existing one-via regression fixture.

Non-exact plane-target work now obtains the entire physical connected terminal set from
`RoutingBoard::Terminals`. The existing one-via assertion passes **without
weakening it**. Explicit post-refill island-anchor requests retain their exact
source: expanding those host-only requests to all sampled plane starts caused
bounded repair regressions, which were reproduced and fixed. Both original
host-refill/worker regression tests remain intact and pass.

This is only the **start-set** correction. Destination conduction regions are
still represented by native targets, not full reference `ConductionArea` search,
and source-equivalent plane early exits / whole-unconnected-set selection remain
open. The host-only exact-anchor adapter must not be presented as Java behavior.

## Explicit unit and geometry boundary

Java uses finite `IntBox.EMPTY` coordinates of ±33,554,432. KiCad IU coordinates
can exceed those values on an ordinary board; using the sentinel directly in
IU can create spurious low-cost empty destinations.

`RoomCostSpace.h` is a deliberately separate native adapter. It reduces bounds
by powers of ten until `5 * maxAbsCoordinate < Limits.CRIT_INT`, following the
reference loader's range-protection principle. Destination boxes enclose their
scaled bounds; query points use the translated floor/ceil rule. Via costs are
divided by the same scale, and estimates are multiplied back **before** entering
the room queue, whose g-costs remain in native geometric units.

This is **heuristic-only normalization**, not an exact port of Specctra import
or the whole geometric model. It preserves source finite sentinels without
patching individual estimate branches. Tests cover negative bounds, ordinary
large KiCad coordinates, empty-box rejection and signed-64-bit extreme bounds.
Native target seeds are still points/axis-aligned lines; Java normally joins
compensated item tree shapes. This remaining modeling difference matters.

## What the rollout still does differently

| Active area | Current implementation versus Java |
|---|---|
| Geometry / search tree | Source-tested rectangular room/tree/drill operations, but conservative centre-space rectangles; no complete 45-degree/general convex compensation, thin-room handling or incremental invalidation. |
| Maze | Rectangular room/door/page/drill queue first, then legacy grid/visibility when unsupported or rejected. Target identities/regions, occupation and backtracking are not established equivalent to an entire Java search. |
| Costs | Destination formulas now translated. UI mappings, fallback scoring and batch scoring remain different: native via defaults 500/50 versus source 50/5; direction penalty divided by ten, grid-scaled bend costs, and retry-scaled fallback via costs are not reference control preparation. Do not claim this closes the entire cost model. |
| Insertion | `FoundConnectionInserter::AppendEdge` emits track/via records. Java inserts forced traces/vias, handles partial failure, connects/splits traces and normalizes contacts. Those algorithms are still missing. |
| Shove / neckdown | `MazeTraceShover::Shorten` is line-of-sight cleanup. It does not recursively displace copper, spring over obstacles, or neck down at pins. Its old “equivalent” comment was corrected. |
| Mutable board / repair | Approximate worker contacts and immutable host group unions remain. Repair snapshots freeze earlier job-generated copper along with original copper; source-equivalent safe rip-up needs persistent ownership/contact/cache transactions. |
| Batch / fanout | Partial connected-set scheduling plus synthetic breakout landings, retries and negotiated congestion; not source item ordering, fanout, connection-chain rip-up, pass policy or best-board lifecycle. |
| Optimization | Guarded shortening/tail cleanup, not full pull-tight/via relocation. `BatchOptimizerMultiThreaded` merely delegates serially; its misleading counterpart comment was corrected. |
| Host integration | Private KiCad copy, refill/full DRC and safe acceptance gate are intentional. The GUI is not being ported. Full editor reject/accept/undo/redo and zone-display parity remain unverified. |

The next fundamental work is **exact geometry + mutable item/contact/search-tree
transactions + forced insertion**, followed by shove, full batch/fanout and
optimization. Copying more class names or tuning a timeout does not supply those
missing capabilities. The separate constrained 555 repair failure must also be
closed without making user-owned copper freely rippable.

## Reproducible differential test

`DestinationDistanceOracle.java` invokes the actual pinned Java class; it does
not reproduce the formulas in the test harness. The 7,712 records cover 1/2/3/4/6
layers, every active-layer bitmask, all component/solder/inner target categories,
multiple point/line/area joins, fractional/negative/large queries, asymmetric and
non-binary weights, zero/normal/cheap via costs, and normal-query restoration.

```sh
python3 scripts/autorouter/run_room_oracle.py --oracle destination \
  --reference-jar /path/to/pinned/freerouting-current-executable.jar \
  --output-dir build/autorouter/a-new-destination-oracle-directory
```

Fixture SHA-256:
`62c918e9f06ba5050700894db3302006f75333325d5b67e30548222758015e6d`.
Keep **all** destination, drill, room and room-restraint fixtures with the QA
changes; the previous commit had left the room/drill golden files untracked.
No files were staged automatically.

## Verification and board measurements

- Rebuilt the actual native KiCad PCB module, `qa_pcbnew` and `qa_autorouter_parity`.
- **331 native/DRC/zone/MatchProperties cases, 334,627 assertions passed.**
  All 71 native cases are included; the existing one-via and host repair
  assertions were not relaxed.
- **7,712 destination oracle records / 30,848 double results match Java bit-for-bit**;
  the existing 7,166 room and 1,280 drill oracle records regenerate unchanged.
- **21 Python tests passed.** ASan+UBSan: 600 data-only multilayer cycles,
  400 valid routes / 200 correctly blocked stacks, independent edge checks;
  `detect_leaks=0`. This is not full-host sanitization or a peak-memory benchmark.
- Full PCB suite: **2,395/2,396 cases pass (18 with warnings)**;
  520,691/520,693 assertions pass, 21 failed warnings. The same existing
  `MatchProperties/KeysAreCanonicalAndLabelsAreFriendly` two label assertions
  fail in the full order, while the combined suite above passes. Not a green
  full suite and not waived.
- The built bundle was identified by its PCB module path/hash. No GUI session
  was restarted or used to claim acceptance/undo/redo coverage.

Three repeats per board/mode, same stripped originals, zones and rules retained:
4 passes, 8 native attempts, 250,000 native expanded nodes, 180-second **process**
limit; optimization off in both engines, reference one thread. No-via mode also
disables fanout in both. Final timing runs did not compete with compilation or
full-suite/sanitizer runs. Values are **native / v2.3.0**; core times are medians,
not JVM startup or full host session times.

| Board / mode | Missing | New KiCad DRC | Vias | Length mm | Core ms | Gate (all 3 repeats) |
|---|---:|---:|---:|---:|---:|---|
| regulated-5v, default | 0 / 0 | 0 / 0 | 0 / 0 | 323.211 / 319.203 | 1730 / 410 | PASS (small-board quality only) |
| 555-astable, default | 0 / 0 | 0 / 10 | 11 / 7 | 96.1797 / 184.248 | 33 / 1810 | FAIL: extra vias |
| bjt-astable, default | 0 / 0 | 0 / 0 | 0 / 0 | 145.458 / 152.435 | 18 / 280 | PASS (small-board quality only) |
| regulated-5v, no-vias | 0 / 0 | 0 / 0 | 0 / 0 | 357.391 / 319.203 | 401 / 410 | FAIL: excess length |
| 555-astable, no-vias | timeout / 0 | timeout / 1 | timeout / 0 | timeout / 193.479 | timeout / 1150 | FAIL: native timeout |
| bjt-astable, no-vias | 0 / 0 | 0 / 0 | 0 / 0 | 147.318 / 152.435 | 17 / 290 | PASS (small-board quality only) |

Default native output counts and lengths are unchanged from the preceding drill
milestone. The translated estimate is a real method-parity improvement, **not a
claim that board quality or runtime parity is now solved**. The 555 still has
four extra native vias; constrained regulator length still exceeds the unchanged
+10% gate, and constrained 555 still times out. Reference DRC reports never
justify introducing native violations. No phantom metrics are assigned to a timeout.

All **33 saved final routed outputs** that exist were independently reloaded,
without selecting a matching net or generating new geometry. Their byte hashes,
actual connectivity, DRC category counts, via counts and lengths matched.
The separate smoke experiments are not counted as final repeated evidence.

Durable local evidence:
[`autorouter-test-assets/online-simple/translation-audit-2026-09-08/`](../autorouter-test-assets/online-simple/translation-audit-2026-09-08/README.md).
It includes saved A/B boards, raw logs, failed intermediate tests, before/after
numeric comparisons, exact source patch, compiler/binary/JAR identities, scripts
and a verified SHA-256 manifest. The three older milestone archives and original
asset manifest were independently checked unchanged; no older archived report was rewritten.
