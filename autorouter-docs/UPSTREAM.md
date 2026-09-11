# Freerouting upstream mapping

**Correction:** the table records intended responsibility/file correspondence.
The historical "port" labels were incorrect and have been removed. In
particular, the shover remains a bounded subset. Exact-octagonal room/door
search and exact free-drill regions are active for ordinary and fanout
single-layer/multilayer attempts. Production no longer contains the previous
grid/visibility fallback. Its scope is documented in the
[octagonal multilayer milestone](OCTAGONAL-MULTILAYER-SEARCH.md), the historical
[drill milestone](DRILL-SEARCH-PARITY.md), and the
[preceding room milestone](ROOM-DOOR-SEARCH.md).
[Complete parity checklist](PARITY-CLOSURE-CHECKLIST.md) and the historical
[Core parity review](CORE-ROUTING-PARITY-REVIEW.md)
record actual implementation and validation status.

This document is the synchronization contract for the native KiCad autorouter.  The KiCad
implementation keeps the Freerouting package layout where it is useful: `maze`, `pipeline`, and
the board-facing `board` adapter are separate directories, and the principal C++ filenames use
the corresponding Freerouting class names.

## Reference

| Field | Value |
|---|---|
| Repository | <https://github.com/freerouting/freerouting> |
| Reference commit | `a11c0a42d1b3827e5126429c5c9820c4ab5bec7c` |
| Reference checkout | `/Users/jmcasler/Documents/GitHub/mchamster/freerouting` (read-only) |
| KiCad source baseline | `b22bb49234` |
| Current native measurement base | `023fdd342cbc8c20c4837428f01b95e1ffa4b930` |
| Native port root | `pcbnew/autorouter/` |

The reference commit is deliberately recorded as a commit rather than a release tag.  Any
algorithmic synchronization must update the hash, the mapping below, and the parity corpus in the
same change.

## Routing-file mapping

| Freerouting source | KiCad native source | Classification / adaptation |
|---|---|---|
| `autoroute/maze/AutorouteControl.java` | `pcbnew/autorouter/maze/AutorouteControl.h/.cpp` | Substitute cost model; grid-normalized lengths, fixed per-edge direction/bend costs, and retry scaling differ from upstream. |
| `autoroute/maze/AutorouteEngine.java` | `pcbnew/autorouter/maze/AutorouteEngine.h/.cpp` | Per-snapshot engine owner; `AUTOROUTER_JOB` owns the native thread and the search owns attempt-local drill/room state. The unused wrapper drill array has been removed; persistent reuse is not implemented. |
| `autoroute/maze/DestinationDistance.java` | `pcbnew/autorouter/maze/DestinationDistance.h/.cpp` | Direct constructor/join/point/box/cheap-estimate translation, active in both room frontiers; 7,712 bit-tested Java records. `RoomCostSpace` is the explicit IU adapter. The grid substitute is now `LegacyDestinationDistance`, not this class. |
| `autoroute/maze/MazeSearchEngine.java` | `pcbnew/autorouter/maze/MazeSearchEngine.h/.cpp` | Active exact-octagonal and arbitrary-convex ordinary/fanout single-/multilayer room search. `MazeSearchEngine45Degree`, `MazeSearchEngine45DegreeMultilayer`, `MazeSearchEngineAnyAngle`, `MazeSearchEngineAnyAngleMultilayer`, `MazeSearchEngine90Degree`, `MazeSearchEngineMultilayer`, the room contexts, and `MazeSearchEngineRooms` isolate core state and the host adapter. Multilayer source rooms are queued through first-class two-dimensional target-item doors rather than null-door states. The grid/visibility fallback and its public switch have been removed. Exact room-door insertion/small-door and shove-adjustment queue semantics remain partial. |
| `autoroute/maze/MazeExpansionEngine.java` | `pcbnew/autorouter/maze/MazeExpansionEngine.h/.cpp` | Pinned-reference page/drill frontier cost operations and radius-scaled via cost. The legacy coordinate-neighbour generator has been removed. |
| `autoroute/maze/MazeListElement.java` | `pcbnew/autorouter/maze/MazeListElement.h` | Pinned f/g/door-ID/section ordering key, including equal-key suppression. Host target/room identity remains an adaptation. |
| `autoroute/maze/MazeSearchElement.java` | `pcbnew/autorouter/maze/MazeSearchElement.h` | Value backtracking record for immutable snapshot search. |
| `autoroute/maze/MazeFanoutDiagnostics.java` | `pcbnew/autorouter/maze/MazeFanoutDiagnostics.h` | Optional callback diagnostics with no GUI or logger dependency. |
| `autoroute/maze/MazeRipupResolver.java` | `pcbnew/autorouter/maze/MazeRipupResolver.h/.cpp` | Active obstacle-room cost translation. Uses exact mutable `Connection.get()` chains when route-to-item mapping is unambiguous; static/fork-split source routes still use connection-record reconstruction and partial-route rip-up is not ported. |
| `autoroute/maze/MazeTraceShover.java` | `pcbnew/autorouter/maze/MazeTraceShover.h/.cpp` | Visibility shortening and redundant-via cleanup used by the optimizer. |
| `autoroute/pipeline/BatchAutorouter.java` | `pcbnew/autorouter/pipeline/BatchAutorouter.h/.cpp` | Active source-order connected-item adapter. Normal routing and optimizer sub-passes retain partial work and use negotiated rip-up; `autoroutePassesForOptimizingItem` is translated for ordinary signal items. Native pad representatives and synthetic fanout/plane targets remain host adaptations. |
| `autoroute/pipeline/BatchAutorouterThread.java` | `pcbnew/autorouter/pipeline/BatchAutorouterThread.h/.cpp` | Algorithm-side worker entry; `AUTOROUTER_JOB` owns the wx-safe native thread. |
| `autoroute/pipeline/AutoroutePassRunner.java` | `pcbnew/autorouter/pipeline/BatchAutorouter.cpp` | The C++ worker is single-threaded per job; the board snapshot makes the pass boundary explicit. |
| `autoroute/pipeline/AutorouteAirlineCalculator.java` | `pcbnew/autorouter/pipeline/AutorouteAirlineCalculator.h/.cpp` | Computes topology lower bounds for metrics and future board scoring. |
| `autoroute/pipeline/BatchFanout.java` | `pcbnew/autorouter/pipeline/BatchFanout.h/.cpp` | Snapshot-side SMD fanout and plane-target classification; fanout escapes are synthetic worker pads and become ordinary KiCad vias/tracks at acceptance. |
| `autoroute/pipeline/BatchOptimizer.java` | `pcbnew/autorouter/pipeline/BatchOptimizer.h/.cpp`, `ReadSortedRouteItems.h/.cpp` | Active serial exact-item-chain transaction with fork expansion, dynamic source cursor, source score/cost phases, whole-board optimizer batch sub-passes, and the source pass-level minimum cumulative weighted-trace-length acceptance floor. The sub-passes may negotiate an unaffected movable signal connection away and rebatch it on the next pass. Synthetic fanout/plane routes stay on dedicated bounded candidates; complete `optChangedArea`, arbitrary via contacts and parallel scheduling remain open. |
| `autoroute/pipeline/BatchOptimizerMultiThreaded.java` | `pcbnew/autorouter/pipeline/BatchOptimizerMultiThreaded.h` | Serial delegate only; no reference candidate scheduling, parallelism or scoring implementation. |
| `autoroute/pipeline/OptimizeRouteTask.java` | `pcbnew/autorouter/pipeline/OptimizeRouteTask.h` | Data-only optimization candidate task; never deep-copies a live KiCad `BOARD`. |
| `autoroute/pipeline/RoutingPipeline.java` | `pcbnew/autorouter/pipeline/RoutingPipeline.h/.cpp` | Stable orchestration seam for future fanout and diagnostics. |
| `autoroute/path/Connection.java` | `pcbnew/autorouter/path/Connection.h/.cpp` plus `ROUTING_CONNECTION` | Direct reverse-ID normal-contact chain/fork traversal over mutable source-shaped polyline/via items, including terminal layers, item set, trace length and detour. Exact rational contact identity is retained without host rounding; non-integral virtual-piece traversal, curved contacts and source connection caching remain open. |
| `autoroute/path/FoundConnectionInserter.java` | `pcbnew/autorouter/path/FoundConnectionInserter.h/.cpp` | Active checked insertion. Same-layer/style items use the reference corner advance/extend/one-corner-rewind loop. Terminal neckdown now uses a source-coordinate Euclidean usable-prefix query, ignores only routable non-shove-fixed copper, applies both source safety tolerances, and reconstructs the normal/narrow transition with `calculateAdditionalCorner` before atomic preflight. Fixed spring-over, bounded recursive trace/via shove and whole-operation rollback are active. Per-span mutable item publication/combination, arbitrary contact graphs and general shove geometry remain incomplete. Host materialization remains in the KiCad adapter/session. |
| `autoroute/path/FoundConnectionLocator.java` | `pcbnew/autorouter/path/FoundConnectionLocator.h/.cpp` | Host entry seam; the active path is reconstructed from retained door-section parent state by the angle-specific locators. |
| `autoroute/path/FoundConnectionLocator45Degree.java` | `pcbnew/autorouter/path/FoundConnectionLocator45Degree.h/.cpp` | Active octagonal ordinary/fanout single- and multilayer corridor locator. Full source shrink/acute/thin-room and pin-exit behavior remain incomplete. |
| `autoroute/path/FoundConnectionLocatorAnyAngle.java` | `pcbnew/autorouter/path/FoundConnectionLocatorAnyAngle.h/.cpp` | Active exact-simplex locator with source left/right visibility-range closure and integral KiCad materialization. Finite target regions and normalized decision-stream proof remain open. |
| `autoroute/expansion/*.java` | `pcbnew/autorouter/expansion/*.h/.cpp` | Active ordinary and fanout single- and multilayer frontiers use exact octagonal and rational-simplex free-room completion, neighbour ordering, edge removal, free/obstacle gaps, doors, sections, target attachment and exact free-drill decomposition. The 45-degree lifecycle reflects KiCad y-down input plus the complete original entry set into the source y-up side cycle, reflects edge flags/gaps back, and clips nominally unbounded removed supports to the actual board instead of applying the unscaled source `Limits.CRIT_INT` sentinel to finer KiCad IU. Concave polygons and holes are split into solid convex leaves. Those primitives are checked by pinned source oracles. Several finite-width/acute-corner locator semantics remain partial. `ExpansionGraph` is retained only for historical QA and is not referenced by production search. |
| `board/model/structure/BoardOutline.java` | `pcbnew/autorouter/board/model/structure/BoardOutline.h` | Active source-shaped routing-tree obstacle. Exact outer and hole contour segments share one item identity and stable shape order, are inserted before ordinary board items, use the source 10 um half width plus candidate-class compensation, and use the geometric contour plus 100 um only as the finite room-search bound. KiCad Edge.Cuts display stroke width does not alter routing geometry. |
| `board/searchtree/ShapeSearchTree45Degree.java` | `pcbnew/autorouter/board/searchtree/ShapeSearchTree45Degree.h/.cpp` | Source octagonal leaf filtering, completion, divide-large-room and recursive restraint are production-built, oracle-tested and selected by active ordinary and fanout single-/multilayer room frontiers. Drill-page free-region decomposition is exact octagonal. |
| `autoroute/drill/DrillPage.java` | `pcbnew/autorouter/drill/DrillPage.h/.cpp` | Lazy exact-octagonal free-drill cutouts with source piece order, source corner-average centre/pin-centre selection, net/policy cache and separate reset/invalidate semantics. |
| `autoroute/drill/DrillPageArray.java` | `pcbnew/autorouter/drill/DrillPageArray.h/.cpp` | Active, attempt-owned, resource-bounded row-major page array. Area-only overlap queries use the source bounding-page range. Fanout annulus candidates remain exact free-shape/drill/ViaRule candidates rather than path-grid samples. |
| `autoroute/drill/ExpansionDrill.java` | `pcbnew/autorouter/drill/ExpansionDrill.h` | Drill centroid/region, exact-octagonal rooms across the full physical stack for ordinary routing, per-layer occupation and source hash. Ordered through/blind/buried/microvia profiles retain layer-local circular copper diameter/clearance and materialize as custom KiCad via padstacks; non-circular source via-pad shapes have no ordinary `PCB_VIA` host representation. |
| `autoroute/AutorouteAttemptResult.java` | `AutorouterTypes.h` (`ROUTING_RESULT`) | Worker result, completion state, metrics, message and cancellation. |
| `autoroute/AutorouteAttemptState.java` | `ROUTER_PROGRESS` / `AUTOROUTER_JOB` | Progress state is polled by the native dialog; fanout connections are reported separately in `ROUTER_METRICS`. |
| `autoroute/BoardHistory*.java` | `BOARD_HISTORY` plus proposal-owned objects | Bounded duplicate-free worker snapshots use source-normalized scoring, restore counts/ranks, and a DRC-first native safety stratum; KiCad `BOARD_COMMIT` still owns editor undo and rejected proposals never enter the board. |
| `autoroute/pipeline/AutoroutePassRunner.java` | `AUTOROUTE_PASS_RUNNER` | Rebuilds the natural-order item-set work list each pass, including plane false-work suppression and exact-island host-repair handling. |
| `autoroute/pipeline/AutorouteBatchLoop.java` | `AUTOROUTE_BATCH_LOOP` | Implements pass-8/modulo-4 history cadence, 0.5-point local/global stagnation windows, restore reset, and one-time fanout recovery; job/thread time-limit orchestration remains host-owned. |
| `autoroute/ItemAutorouteInfo.java` | `BOARD_SNAPSHOT`, `ROUTING_PAD`, `ROUTING_NET` | Immutable worker-side topology. |
| `autoroute/ItemRouteResult.java` | `pcbnew/autorouter/ItemRouteResult.h` | Direct lexicographic incomplete/via/length result and improvement calculation. |
| `autoroute/ItemSelectionStrategy.java` | `BATCH_AUTOROUTER::orderNets` | Stable priority, pad-count and half-perimeter ordering. |
| `autoroute/PerformanceProfiler.java` | `ROUTER_METRICS` | Native elapsed time, expanded nodes, passes, rip-ups, vias and length. |
| `core/scoring/BoardStatistics.java` trace totals | `OptimizerWeightedTraceLength`, `BATCH_OPTIMIZER::Optimize` | Source mutable-trace width/clearance weighting, `SHOVE_FIXED` discount and pass-level minimum cumulative length acceptance contract. Whole-board source statistics outside the optimizer remain partial. |
| `autoroute/RoutingFailureLog.java` | `ROUTING_RESULT::message` | User-visible failure status; detailed diagnostics belong in the future parity logger. |
| `autoroute/events/*.java` | `pcbnew/autorouter/events/*.h` and `AUTOROUTER_JOB` progress callback | Immutable callback/event values replace Java event objects because wxWidgets and KiCad views are main-thread only. |

## Board, rules, geometry and transaction mapping

| Freerouting concept | KiCad equivalent | Native file |
|---|---|---|
| `board.facade.RoutingBoard` | Partial mutable copper/contact graph with R-tree queries, source-order exact rational normal contacts, integral polyline junction splitting and transactions; non-integral piece materialization, forced insertion and room mutation are still incomplete | `board/facade/RoutingBoard.h/.cpp`; input capture remains in `KicadBoardAdapter` |
| `RoutingBoardSearchFacade` / shape search tree | Snapshot obstacles and keepouts plus first-class exact geometric `BoardOutline` contour/hole entries | `board/KicadBoardAdapter.cpp`, `board/model/structure/BoardOutline.h`, `maze/MazeSearchEngine.cpp` |
| `RoutingBoardOperations` | Live edits use `BOARD_COMMIT` on the editor thread; a private `BOARD` is used by host validation on the worker. Neither implements reference forced insertion | `AutorouterTool.cpp`, `board/KicadRoutingSession.cpp` |
| `RoutingBoardUndoFacade` / board history | `BOARD_COMMIT` | KiCad-provided API; one accepted proposal is one commit |
| `board.trace.PolylineTrace*` | `PCB_TRACK` segments | `KicadBoardAdapter::CreatePreviewItems` |
| `autoroute.maze.MazeTraceShover`, `board.optimize.TraceShover` / tighteners | Active expansion-time source trace-side shove plus bounded recursive spring-over over source-shaped box, circle, capsule and convex support contours. Physical obstacle TileShapes receive the source two-half-clearance `Simplex.enlarge` sequence and one-coordinate wrap margin before checked mutation. Concave/holed contour mutation, direction-specific trace substitution, and full tightener mutation remain open. | `maze/MazeTraceShover.cpp`, `board/optimize/TraceShover.cpp`, `board/optimize/TraceTightener.cpp` |
| `board.model.items.Pin`, `core.library.Padstack` trace exits | Layer-local exact pin contour indices, normalized integral exit directions, centre-to-border lengths, package aspect policy, direct pin-to-drill nearest-exit bias, endpoint direction/length checks, source offset-convex entrance search and shortest-border reconnection, followed by a strictly inserted `SHOVE_FIXED` centre stub. Offset and rotated shapes are retained; a rational-only replacement fails closed because KiCad copper vertices are integral. | `board/model/items/Pin.h`, `board/KicadBoardAdapter.cpp`, multilayer maze frontiers and `board/optimize/TraceTightener.cpp` |
| `board/optimize/ViaOptimizer.java` | `pcbnew/autorouter/board/optimize/ViaOptimizer.h/.cpp` | Active weighted two-trace and bounded plane/fanout via-location candidates. Every replacement is strictly checked; arbitrary source item contact mutation and recursion remain partial. |
| planar `IntPoint`, `RationalPoint`, `Line`, `IntBox`, `IntOctagon`, `Simplex`, `Polyline` | Source-named data-only types plus KiCad adapter conversion | `geometry/planar/`; exact rational lines, closed-segment/polyline containment, bounded simplex/polyline operations and core octagon operations are present. `IntOctagon` and 45-degree restraint have 4,096 direct source-oracle records. Full `TileShape`, polygon/circle offsets and cutout/projection APIs remain open. |
| `rules.Net`, `NetClass`, clearance matrix | `NETCLASS`, `BOARD_DESIGN_SETTINGS`, pair/layer cache and exact host-resolved obstacle-item/net/layer clearance matrix | `board/KicadBoardAdapter.cpp`, `AutorouterTypes.h`; explicit actual-item rules are captured before detaching the worker snapshot and remain authoritative even when they lower a netclass default. Future-route position/geometry conditions remain acceptance-time KiCad DRC constraints. |
| `core.library.Padstack`, `rules.ViaInfo` / `ViaRule` | Ordered through/blind/buried/microvia choices with complete physical span, attach-SMD policy, and layer-local circular copper diameter/clearance. Exact search, shove, DRC and custom KiCad padstack materialization share one retained profile. Arbitrary non-circular Freerouting via-pad shapes cannot be represented by ordinary KiCad `PCB_VIA` objects and remain fail-closed. | `AutorouterTypes.h`, `rules/ViaRule.h`, `MazeSearchEngine.cpp`, `RoutingBoard.cpp`, `DesignRulesChecker.cpp`, `KicadBoardAdapter.cpp` |
| `drc.DesignRulesChecker` | Worker pair/layer and contextual obstacle clearance checks plus the KiCad DRC engine | `drc/DesignRulesChecker.cpp` consumes the detached matrix; `KicadRoutingSession` refills/validates private proposals before acceptance. QA independently materializes and checks them again. Host rules conditional on future route geometry still require this final KiCad gate. |
| `ConductionArea` / plane behavior | Region-aware copper graph, synthetic targets and exact ratsnest-region anchors, followed by host refill/repair | `KicadBoardAdapter`, `board/facade/RoutingBoard`, `KicadRoutingSession`; dynamic full-region plane search and reference via optimization remain missing |

## KiCad editor integration mapping

| Concern | Source |
|---|---|
| Action | `pcbnew/tools/pcb_actions.h/.cpp`, `PCB_ACTIONS::autorouteBoard` |
| Menu | `pcbnew/menubar_pcb_editor.cpp`, Route menu |
| Tool lifecycle | `pcbnew/autorouter/AutorouterTool.h/.cpp`, registered by `pcb_edit_frame.cpp` |
| Settings | `DialogAutorouter.h/.cpp` |
| Worker/cancel/progress | `AutorouterJob.h/.cpp`, `AutorouterDialogs.h/.cpp` |
| Proposal visualization | `AutorouterPreviewItem.h/.cpp`, `KIGFX::VIEW_GROUP` |
| Accept/reject | `AUTOROUTER_TOOL`, `BOARD_COMMIT`, `ROUTER_TRANSIENT` |

## Intentional host differences

1. Production never uses DSN/SES. The host session clones the board with native
   in-memory KiCad serialization, preserving unsaved geometry and net settings.
   Data-only snapshots feed the core; ordinary tracks/vias are created at acceptance.
2. KiCad views, wxWidgets UI and the live editor board are never touched from
   the worker thread. Refill and DRC run against a session-owned private board.
3. The preview is a `VIEW_ITEM`, not a `BOARD_ITEM`; this is what makes Reject exact and keeps the
   pre-run board out of the undo stack.
4. The adapter converts host coordinates/rules, but this is not a reason to omit the private
   compensated convex-shape model and mutable search tree required by the core port.
5. PNS is not used as a global routing engine.  It remains available for manual routing; reusing
   its collision primitives is acceptable only when parity tests show no behavioral change.
6. Freerouting's per-net `attachSmdAllowed` flag is exposed as the native
   `AUTOROUTER_SETTINGS::allowViaInSmdPad` control.  It defaults to false and the search refuses
   to transition layers at an SMD pad centre, so the native fanout stage produces an escaped via
   rather than silently creating via-in-pad geometry.
7. KiCad's live DRC permits same-net tracks to share existing same-net copper, including the
   copper around an already-placed via drill.  The adapter therefore retains existing via holes as
   hard obstacles for foreign nets and for new via-to-drill checks, but the worker does not create a
   false same-net track violation for existing routed-via UUIDs.

## Synchronization procedure

1. Diff `autoroute/maze`, `autoroute/path`, `autoroute/expansion`, `autoroute/drill`, and
   `autoroute/pipeline` between the recorded commit and the candidate upstream commit.
2. Locate the corresponding C++ file using the tables above; do not copy GUI or Specctra I/O
   changes into the routing layer.
3. Update the adapter only for KiCad ownership, geometry, rule, or transaction differences.
4. Run the pure algorithm tests and the regression corpus against both implementations.
5. Record the new commit and every intentional divergence in this file.

## Rectangular core milestone mapping (2026-09-08)

| Reference source | Native file | Scope |
|---|---|---|
| `datastructures/MinAreaTree.java` | `datastructures/MinAreaTree.h/.cpp` | Insertion ties, removal and dynamically pruned traversal; rectangle entries with stable handles. |
| `board/searchtree/ShapeSearchTree90Degree.java` | Same relative path, `.h/.cpp` | Complete-shape traversal/ignore semantics plus restraint; caller supplies expanded shapes. |
| `board/searchtree/ShapeSearchTree45Degree.java` | Same relative path, `.h/.cpp` | Exact octagonal broad/narrow tree, completion and restraint. Built, oracle-tested and active for ordinary and fanout single-/multilayer room search; drill-page free regions remain exact octagons. |
| `autoroute/expansion/SortedOrthogonalRoomNeighbours.java` | `expansion/SortedOrthogonalRoomNeighbours.h/.cpp` | Free-room ordering and gaps; obstacle-room branch not ported. |
| `autoroute/expansion/ExpansionDoor.java` | `expansion/ExpansionDoor.h/.cpp` | Exact octagonal overlap dimension, line/gravity sections and narrow-door behavior match 2,048 pinned source records and feed active single-layer search. The explicit host section-budget guard remains. |
| `autoroute/maze/AutorouteEngine.java`, `MazeSearchEngine.java` | `maze/MazeSearchEngine45Degree.h/.cpp`, `MazeSearchEngine45DegreeMultilayer.cpp`, `RoomSearchContext45Degree.h`, and existing 90-degree/multilayer files | Active exact-octagonal ordinary and fanout single-/multilayer lifecycle and frontier with paid obstacle rooms, exact drill regions, first-drill fanout termination and source-shaped queue state. The rectangular single-layer and grid/visibility fallbacks remain transitional. |
| `autoroute/maze/MazeListElement.java` | `maze/MazeListElement.h` | Exact ordering key; full backtracking state resides in the rectangular search. |
| `autoroute/expansion/TargetItemExpansionDoor.java` | `expansion/TargetItemExpansionDoor.h/.cpp` | First-class one-sided, dimension-two start doors now feed every multilayer frontier with source-compatible wrapped identity and room-clipped tree bounds. Exact integral point/axis/oblique trace connection-shape intersection selects the room attachment. Diagonal start traces use bounded exact lattice samples around all active supports instead of an AABB. Exact non-box tree contours and rational host endpoints remain open. |
| `autoroute/path/FoundConnectionLocator*.java` | `path/FoundConnectionLocator45Degree.h/.cpp`, `path/FoundConnectionLocatorAnyAngle.h/.cpp` | Active rectangular, octagonal and exact-general corridor construction. Direct pin-to-drill expansion uses the source nearest legal exit; post-route rectangular pin checks and shove-fixed exit stubs are active through the tightener. Arbitrary offset-shape border walks and remaining acute/thin-room details remain open. |
| `geometry/planar/IntBox.java`, `FloatPoint.java` | `geometry/planar/IntBox.h`, `FloatLine.h` | Required rectangle/rounding/weighted-distance primitives only. |

Additional drill milestone mapping:

| Reference source | Native file | Scope |
|---|---|---|
| `geometry/planar/IntBox.java` cutout; `PolylineArea.java` split-to-convex | `geometry/planar/IntBox.h`, `PolylineArea.h` | Rectangular cutout order/perimeter ties and sequential hole decomposition, with cancellation/resource failure. Not general polygon decomposition. |
| `autoroute/drill/DrillPage*`, `ExpansionDrill` | `drill/` | Active page/region/physical-room/layer state, not sampled landmarks. |
| `autoroute/maze/MazeExpansionEngine`, `MazeSearchEngine` | `maze/MazeExpansionEngine.h`, `MazeSearchEngineMultilayer.cpp` | One room/page/drill-layer queue with reference page/drill cost primitives. Heuristic, target IDs and geometry adapter remain partial. |
| `board/facade/RoutingBoard.removeTraceTails` and normal-contact splitting | `board/facade/RoutingBoard.cpp`, `pipeline/BatchOptimizer.cpp` | Native exact-junction trimming of generated trace tails/overlapping ends, preserving real pad/plane groups. Not the complete source item-chain/normalization algorithm. |

The Java executable is used only by the QA oracle sources in
`scripts/autorouter/` and A/B QA; it is not linked, launched, or required by
the native editor. The octagon goldens were generated from a clean GitHub
checkout of `a11c0a42` (JAR SHA-256
`1804b9a8e8fd1bcb5ee29b4249c709c2bf27bdc55547dc488bf6e8e380c39325`),
not from the protected local reference checkout.

## Direct destination translation — 2026-09-08

See [the current audit](DESTINATION-DISTANCE-PARITY.md) for exact source-method
coverage, floating-point contraction diagnostics, native IU adaptation, and
remaining active substitutions. `geometry/planar/FloatLine.h` currently hosts
`FLOAT_POINT::BoundingBox`; `geometry/planar/IntBox.h` contains the translated
weighted box distance. The source destination class itself preserves constructor,
join and calculation branch structure; the old heuristic was moved to
`LegacyDestinationDistance` to keep the reference filename unambiguous.

## Contact/insertion/fanout continuation

See [the implementation and remaining-gap report](CORE-CONTACT-INSERTION-PARITY.md)
for the same `a11c0a42` source pin. Added QA-only normal-contact and fanout-order
oracles; no Java or network router was introduced into production. Normal
contacts, integer splitting, checked atomic insertion and ordering are partial
core capabilities. Source-shaped pin neckdown and bounded forced shove are active;
general mutable item/contact shove and full batch parity remain incomplete.

### Exact geometry / fixed-obstacle contour insertion

`geometry/planar/{Point,Line,Polyline,Simplex}` and
`board/optimize/TraceShover` retain source filenames and package boundaries.
See [geometry/spring-over report](CONVEX-SPRING-OVER-PARITY.md) for the original
finite bounded geometry milestone, the 2,432-record Java oracle and endpoint-loss
guard. Later checkpoints added checked trace/via publication, terminal neckdown,
and source-shaped circle/capsule/general-convex spring-over; the report is
historical rather than the current capability boundary.
