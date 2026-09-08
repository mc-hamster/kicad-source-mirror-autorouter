# Core routing parity review — 2026-09-07

**Historical milestone:** the host-validation/plane-repair changes in the
[complete parity checklist](PARITY-CLOSURE-CHECKLIST.md) supersede the 555 and
pad-only connectivity status below. The architectural search/insertion gaps
remain open. Do not use these older measurements as the current build's results.

> Follow-up: [Mutable copper rework](MUTABLE-COPPER-REWORK.md) implements and
> integrates the private copper/contact foundation. The findings and A/B table
> below describe the preceding review; the follow-up supersedes its connectivity
> implementation status and dangling-via results. Room/door and shove gaps remain.

## Verdict

**The native router is not implemented the same way as Freerouting.** The main
problem is architectural, not an insufficient iteration limit. Matching class
names conceal an alternate grid/visibility router without the mutable routing
board, compensated free-space search, forced insertion and shove operations
that make the reference effective. This pass includes concrete production fixes,
but **does not establish core parity or solve the 555 completion failure**.

Code baseline: native `47392bdb76b56ce62a7a05e524d763496bea5f18`, plus the changes
in this working tree. Code reference: Freerouting
`a11c0a42d1b3827e5126429c5c9820c4ab5bec7c`. The protected reference checkout was
read only; no reference sources were edited and no build was run there.
Performance baseline remains the official **v2.3.0** executable, revision
`2d4de019aa89e9fa3dc1dc44e09bf509760cafc1`, not the development reference JAR.
Do not conflate current source behavior with release-baseline behavior.

Paths below are relative to `pcbnew/autorouter/` unless otherwise indicated.

## Active call paths

Native: `RoutingPipeline` → `BatchAutorouterThread` → `BatchAutorouter` →
`AutorouteEngine::AutorouteConnection` → `MazeSearchEngine::FindConnection`.
`MazeExpansionEngine::Neighbours` supplies eight coordinate offsets and layer
jumps. `ExpansionGraph::BuildLandmarks` adds capped sampled points, not rooms.
Successful paths enter occupancy as vectors of nodes. `buildGeometry` later
turns their edges into tracks/vias. The editor's `BOARD_COMMIT` is applied after
review; it is **not** a search-time board transaction.

Reference: connected **Item sets** enter
[`AutorouteEngine.autorouteConnection`](https://github.com/freerouting/freerouting/blob/a11c0a42d1b3827e5126429c5c9820c4ab5bec7c/src/main/java/app/freerouting/autoroute/maze/AutorouteEngine.java).
[`MazeSearchEngine`](https://github.com/freerouting/freerouting/blob/a11c0a42d1b3827e5126429c5c9820c4ab5bec7c/src/main/java/app/freerouting/autoroute/maze/MazeSearchEngine.java)
expands door sections, completes rooms against a compensated shape search tree,
handles drill pages/layer transitions and obstacle rooms, and returns a destination
door/section. The locator reconstructs geometry through those sections.
[`FoundConnectionInserter`](https://github.com/freerouting/freerouting/blob/a11c0a42d1b3827e5126429c5c9820c4ab5bec7c/src/main/java/app/freerouting/autoroute/path/FoundConnectionInserter.java)
uses `insertForcedTracePolyline`, via insertion, trace attachment and neckdown
logic on a mutable routing board. Those operations can fail independently of
finding a path; a coordinate vector is not sufficient evidence of insertion.

## Findings and required work

### P1 — The compensated room/door algorithm is not active

Evidence: `maze/MazeSearchEngine.cpp`, `maze/MazeExpansionEngine.cpp`,
`expansion/ExpansionGraph.cpp`. The frontier and backtracking maps are keyed by
coordinate/layer, not an expandable door section with entry geometry and rip-up
state. Sample limits and grid pitch change reachability. They are not upstream
room-completion parameters.

`CompleteFreeSpaceExpansionRoom` and the door classes do not participate in the
active search. `ShapeSearchTree90Degree::RestrainShape` has 512 Java-derived
ordered primitive checks, but no production search caller. The engine-owned
drill page array is not consumed by its search; landmark generation creates a
separate page array. Do not count any of these filenames as a completed engine.

Required: port convex geometry/compensation, mutable search-tree insertion and
removal, room completion, door/section construction and the actual priority
queue/backtrack behavior as a connected vertical slice. Validate active calls,
not just the presence of translated helper methods.

### P1 — Connectivity is a pad/request graph, not mutable connected copper

Evidence: `board/RoutingBoardInterface.h` exposes only `CreateSnapshot`;
`pipeline/BatchAutorouter.cpp` maintains a pad DSU;
`pipeline/AutorouteUnroutedReport.h` unions successful request endpoints.
Synthetic plane samples and fanout landings are not physical connected item
sets. Existing trace interiors are not arbitrary search start/destination
regions. Pour refill can invalidate a sampled target or disconnect an island.

The saved 555 case is direct counter-evidence: the worker reports 28/28 tasks,
but KiCad reports one remaining electrical connection and seven dangling vias.
This remains after the retained changes. No legitimate completion test may use
that worker percentage as the acceptance criterion.

Required: private mutable pad/trace/via/conduction-area items with geometric
contacts and connected-set traversal, plus final KiCad zone refill and actual
connectivity. Preserve identity of separate fill regions; do not union every
sample of a net or count a fanout landing as a finished electrical connection.

### P1 — Forced insertion, shove and pull-tight are missing

Evidence: `path/FoundConnectionInserter.cpp` appends straight segments/vias;
`maze/MazeTraceShover.cpp::Shorten` removes intermediate points within one route.
It never recursively displaces a foreign trace or via. The locator delegates to
search again and is not used to backtrace door geometry; its 45-degree and
any-angle variants are aliases. `BatchOptimizerMultiThreaded` calls the serial
path shortener.

Required: reference-equivalent forced insertion and rollback on the private
board, item-local shove/spring-over/neckdown operations, spatial-tree updates,
contact recomputation and pull-tight. Keep KiCad acceptance undo separate from
algorithm-internal speculative undo. Do not rename shortening to shoving.

### P1 — Via entry/exit was incorrectly treated as a manufactured padstack

**Fixed for the supported through-hole subset in this pass.**

Reference
[`AutorouteControl.rebuildViaInfo`](https://github.com/freerouting/freerouting/blob/a11c0a42d1b3827e5126429c5c9820c4ab5bec7c/src/main/java/app/freerouting/autoroute/maze/AutorouteControl.java)
builds masks and per-layer radii from the net's actual `ViaRule` padstacks. Native
code instead used the chosen entry/exit range, omitted inactive layers from
materialized vias, and inferred blind/buried type from that shortened range.
A connection between two inner layers could therefore produce an unsupported
buried via rather than a through-hole via.

Changes retained:

- `rules/ViaRule.h` centralizes the **one supported through-hole padstack**.
  Its physical span includes every stack layer, independent of trace enablement.
- `MazeSearchEngine::CanUseSegment` validates the whole physical span, rejects
  unknown/disabled trace endpoints and non-coincident layer transitions, and
  honors `allowVias`.
- General search and both direct fanout orientations use that same predicate.
- `BatchFanout` checks/reserves the full through-via span during planning.
- `BatchAutorouter` materializes full-stack vias in physical ordinal order.
- Occupancy drill spacing and rip-up conflict checks no longer forget the outer
  portions of a via whose search happened to enter/exit on inner layers.
- The adapter retains pad obstacles on physical layers disabled for trace routing.

This is not a general `ViaRule` port: multiple padstacks, blind/buried/microvia
selection, per-layer shapes and rule-specific constraints still need explicit
modeling. They must not be inferred from arbitrary layer jumps.

### P1 — Final host validation is a QA gate, not a production acceptance gate

Evidence: `AutorouterTool::acceptProposal` verifies the board timestamp and
commits the proposal; it does not execute the disposable-board refill + DRC +
connectivity procedure used by `qa_autorouter_parity`. The worker's collision
count is not full KiCad DRC. The UI warning to verify a result is not validation.

Required: evaluate the complete materialized proposal on a separate board before
labeling it complete or safe. Compare against baseline violations without
ignoring new categories; report partial results truthfully. Preserve the user's
explicit review/accept/reject and original board on failure/cancellation.

### P2 — Pad bounding boxes close legal channels

**Fixed for non-circular copper contours in this pass.** Previously every
non-circular pad became an axis-aligned rectangle, including rotated rectangles,
ovals, chamfered/rounded and custom pads. Search could reject free corners and
narrow diagonal channels that the source board actually permits.

`KicadBoardAdapter::addPads` now preserves actual per-layer copper. Circles
remain circles; ovals are exact capsules; rounded rectangles use a four-corner
core plus disk radius (or a capsule/circle for degenerate cores). Cardinal
rectangles retain the fast rectangle primitive. Other shapes copy polygon
outlines/holes with zero baked-in clearance and a conservative outward arc
approximation of at most 1 µm. Pair clearance is still applied separately.
Using many arc facets for ordinary rounded rectangles initially doubled the
regulator runtime; the exact core-plus-radius representation avoids that cost. This improves the snapshot geometry; it does **not** port
upstream compensated convex decomposition or its search tree. Slotted-hole
geometry and layer/item-specific rule coverage remain separate gaps.

### P2 — The cost model and frontier state are not equivalent

Evidence: `maze/AutorouteControl.cpp` normalizes length by grid pitch, adds a
constant per-edge direction penalty and retry-scaled via penalty.
`MazeSearchEngine.cpp` charges `bendCost` on every non-via edge, even straight
ones. Its coordinate/layer key cannot preserve independent turn-dependent entry
labels. The short final connection also bypasses some movement costs.

Reference uses per-axis weighted geometric distance, padstack-radius-scaled via
costs and, at the development reference, a direction-change bend test. The cost
model, lower bound, units, queue labels and final arrival must be ported together.
Changing one formula and raising node/time budgets is not an acceptable remedy.

An experiment in this pass corrected the angular bend test, preserved incoming
heading in search labels and queued exact goal arrivals. All 38 experimental
unit cases passed, but the regulator A/B run increased to **12.882 s** native
routing time versus the prior approximately 1.3 s, with no completion benefit.
The experiment was **reverted**, including its tests; those changes are not in
the delivered router. Evidence remains locally under
`build/autorouter/parity-review/final/`. Despite the old directory name, these
are **rejected-experiment results**, not measurements of the retained patch.
The existing cost mismatch remains an explicit open item, not a hidden fix.

## Replacement sequence and exit criteria

1. Preserve the host snapshot/worker/proposal boundary. Introduce a private
   mutable routing board with stable item identities, per-layer convex copper,
   contacts, compensated search tree and reversible operations. Do not mutate
   the editor board from the worker or hide Java execution behind a native label.
2. Wire true room completion, doors, sections and padstack-aware drill expansion
   into an active room-search path. Translate reference ordering/tie-breaks and
   unit conversion; compare normalized first-divergence traces on small fixtures.
3. Port geometric backtrace and forced insertion, including shove, neckdown,
   rollback and contact/tree updates. Route connected item sets, not only pad
   centers, and re-evaluate requests after every mutation/rip-up.
4. Replace synthetic fanout/plane batching and same-path shortening with the
   corresponding upstream item-level operations and optimizers.
5. Make independent materialized KiCad connectivity/DRC part of final acceptance.
   Keep full-reference DRC distinct from incomplete reference statistics.

For each slice require primitive regression checks, an active-path integration
case, cancellation/rollback tests, and repeated untruncated same-input runs
against official v2.3.0. Record resolved constraints and source/binary hashes.
No new DRC violations is the first gate; matching completion is the second.
Measure engine time separately from JVM startup, export, import and validation.
Do not present the current three tiny boards as sufficient coverage for dense
SMD routing, multilayer boards, plane islands, forced insertion or rip-up.

## Verification of the retained changes

Final measurements are recorded below. Local evidence and all saved routed
outputs are under `build/autorouter/parity-review/delivered/`.
`kept/` and `verified/` contain intermediate polygon-representation experiments,
not the final primitive-preserving geometry. The original permanent
A/B assets under `autorouter-test-assets/online-simple/` were not overwritten.

- Rebuilt `pcbnew` / `_pcbnew.kiface`, `qa_pcbnew` and `qa_autorouter_parity`
  with CMake/Ninja. Existing duplicate-library linker warnings remain.
- Before changes: 32 native cases, 9,331 assertions passed.
- Retained patch: **38 native cases, 9,382 assertions passed** (six new cases).
- Python parity/metrics tooling: **12 tests passed**. `git diff --check` passed.
- Full KiCad test suite and manual GUI routing were not run in this pass;
  the headless harness executes the production native routing pipeline.
- Three sequential, untruncated A/B repeats per board, official v2.3.0 reference,
  four passes, native eight attempts and 250,000 nodes per attempt, final
  optimization disabled on both sides. Final materialized boards were refilled
  and independently checked with KiCad connectivity/DRC.

| Board | Reference core median (range), s | Native core median (range), s | Actual missing, reference/native | New DRC, reference/native |
|---|---:|---:|---:|---:|
| Regulated 5 V | 0.420 (0.410–0.420) | 1.313 (1.303–1.330) | 0 / 0 | 0 / 0 |
| 555 astable | 1.990 (1.710–2.110) | 0.023 (0.022–0.023) | 0 / 1 | 10 / 7 |
| BJT astable | 0.290 (0.280–0.290) | 0.018 (0.017–0.018) | 0 / 0 | 0 / 0 |

Connectivity and DRC outcomes are identical across the three repeats. The 555
reference findings are nine minimum-track-width violations and one dangling
via; it is **not** a clean DRC reference result. Its native result remains
incomplete, so its short runtime cannot be presented as a speed win. The
regulator remains roughly three times slower than the reference core. Relative
to the earlier native 1.292 s median it is approximately unchanged at 1.313 s;
this is not a performance-parity claim.

A persistent, Git-ignored archive of all nine runs, including routed boards,
checksums and the native source patch, is saved at:

`autorouter-test-assets/online-simple/parity-review-2026-09-07/`

All **51 original saved-corpus files** still match their recorded SHA-256 values.
No original PCB or baseline result was overwritten, and no production routing
path invokes Java or exports DSN/SES. Those remain comparison-tool dependencies.
