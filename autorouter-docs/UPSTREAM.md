# Freerouting upstream mapping

**Correction:** the table records intended responsibility/file correspondence.
The historical "port" labels were incorrect and have been removed. In
particular, the shover still only shortens paths. Rectangular room/door/drill
search is now active for single-layer and multilayer attempts, with a remaining
grid/visibility fallback. Its scope is documented in the
[drill milestone](DRILL-SEARCH-PARITY.md) and
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
| Current native measurement base | `47392bdb76b56ce62a7a05e524d763496bea5f18` plus the archived working-tree patch |
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
| `autoroute/maze/MazeSearchEngine.java` | `pcbnew/autorouter/maze/MazeSearchEngine.h/.cpp` | Hybrid boundary: guarded rectangular single/multilayer room search first, experimental grid/visibility fallback, exact snapshot collision predicates. `MazeSearchEngine90Degree`, `MazeSearchEngineMultilayer`, `RoomSearchContext` and `MazeSearchEngineRooms` isolate core state and the host adapter. |
| `autoroute/maze/MazeExpansionEngine.java` | `pcbnew/autorouter/maze/MazeExpansionEngine.h/.cpp` | Pinned-reference page/drill frontier cost operations and radius-scaled via cost. Legacy coordinate neighbours remain only for fallback. |
| `autoroute/maze/MazeListElement.java` | `pcbnew/autorouter/maze/MazeListElement.h` | Pinned f/g/door-ID/section ordering key, including equal-key suppression. Host target/room identity remains an adaptation. |
| `autoroute/maze/MazeSearchElement.java` | `pcbnew/autorouter/maze/MazeSearchElement.h` | Value backtracking record for immutable snapshot search. |
| `autoroute/maze/MazeFanoutDiagnostics.java` | `pcbnew/autorouter/maze/MazeFanoutDiagnostics.h` | Optional callback diagnostics with no GUI or logger dependency. |
| `autoroute/maze/MazeRipupResolver.java` | `pcbnew/autorouter/maze/MazeRipupResolver.h/.cpp` | Active obstacle-room cost translation. Uses exact mutable `Connection.get()` chains when route-to-item mapping is unambiguous; static/fork-split source routes still use connection-record reconstruction and partial-route rip-up is not ported. |
| `autoroute/maze/MazeTraceShover.java` | `pcbnew/autorouter/maze/MazeTraceShover.h/.cpp` | Visibility shortening and redundant-via cleanup used by the optimizer. |
| `autoroute/pipeline/BatchAutorouter.java` | `pcbnew/autorouter/pipeline/BatchAutorouter.h/.cpp` | Substitute pad-graph batching, retries and result materialization; not a port of the connected-item routing pipeline. |
| `autoroute/pipeline/BatchAutorouterThread.java` | `pcbnew/autorouter/pipeline/BatchAutorouterThread.h/.cpp` | Algorithm-side worker entry; `AUTOROUTER_JOB` owns the wx-safe native thread. |
| `autoroute/pipeline/AutoroutePassRunner.java` | `pcbnew/autorouter/pipeline/BatchAutorouter.cpp` | The C++ worker is single-threaded per job; the board snapshot makes the pass boundary explicit. |
| `autoroute/pipeline/AutorouteAirlineCalculator.java` | `pcbnew/autorouter/pipeline/AutorouteAirlineCalculator.h/.cpp` | Computes topology lower bounds for metrics and future board scoring. |
| `autoroute/pipeline/BatchFanout.java` | `pcbnew/autorouter/pipeline/BatchFanout.h/.cpp` | Snapshot-side SMD fanout and plane-target classification; fanout escapes are synthetic worker pads and become ordinary KiCad vias/tracks at acceptance. |
| `autoroute/pipeline/BatchOptimizer.java` | `pcbnew/autorouter/pipeline/BatchOptimizer.h/.cpp`, `ReadSortedRouteItems.h/.cpp` | Active serial whole-connection reroute/pull-tight transaction with contact guards, tail cleanup, source score thresholds, increased-to-normal and per-trace rip-up cost scaling, and odd/even preferred-direction variation. `ReadSortedRouteItems` performs the source's mutable full-board rescan, strict x/y/layer cursor and via-first tie behavior after every item attempt. Native records still prevent exact item-chain removal and safe optimizer rip-up. |
| `autoroute/pipeline/BatchOptimizerMultiThreaded.java` | `pcbnew/autorouter/pipeline/BatchOptimizerMultiThreaded.h` | Serial delegate only; no reference candidate scheduling, parallelism or scoring implementation. |
| `autoroute/pipeline/OptimizeRouteTask.java` | `pcbnew/autorouter/pipeline/OptimizeRouteTask.h` | Data-only optimization candidate task; never deep-copies a live KiCad `BOARD`. |
| `autoroute/pipeline/RoutingPipeline.java` | `pcbnew/autorouter/pipeline/RoutingPipeline.h/.cpp` | Stable orchestration seam for future fanout and diagnostics. |
| `autoroute/path/Connection.java` | `pcbnew/autorouter/path/Connection.h/.cpp` plus `ROUTING_CONNECTION` | Direct reverse-ID normal-contact chain/fork traversal over mutable source-shaped polyline/via items, including terminal layers, item set, trace length and detour. Curved/rational contacts and source connection caching remain open. |
| `autoroute/path/FoundConnectionInserter.java` | `pcbnew/autorouter/path/FoundConnectionInserter.h/.cpp` | Active checked insertion. Same-layer/style items use the reference corner advance/extend/one-corner-rewind loop; terminal neckdown, fixed spring-over, bounded recursive trace/via shove and whole-operation rollback are active. Exact sampled partial-segment progress, per-span mutable item combination/normalization, arbitrary contact graphs and general geometry remain incomplete. Host materialization remains in the KiCad adapter/session. |
| `autoroute/path/FoundConnectionLocator.java` | `pcbnew/autorouter/path/FoundConnectionLocator.h/.cpp` | Unwired wrapper that invokes search again; not the upstream door-section backtrace locator. |
| `autoroute/path/FoundConnectionLocator45Degree.java` | `pcbnew/autorouter/path/FoundConnectionLocator45Degree.h/.cpp` | Active rectangular 90/45-degree corridor locator; composed with through-drill transitions. Full convex/acute/thin-room and pin-exit behavior remain unported. |
| `autoroute/path/FoundConnectionLocatorAnyAngle.java` | `pcbnew/autorouter/path/FoundConnectionLocatorAnyAngle.h` | Alias of the generic locator, not an any-angle locator implementation. |
| `autoroute/expansion/*.java` | `pcbnew/autorouter/expansion/*.h/.cpp` plus `ExpansionGraph` | Orthogonal neighbours, free-room lifecycle and door sections now run in the rectangular slice. Other angle/obstacle classes remain unported; rectangular drill-room linkage is now active. ExpansionGraph is still the separate legacy visibility graph. |
| `autoroute/drill/DrillPage.java` | `pcbnew/autorouter/drill/DrillPage.h/.cpp` | Lazy rectangular free-drill cutouts with source piece order, centroid/pin-centre selection, net/policy cache and separate reset/invalidate semantics. |
| `autoroute/drill/DrillPageArray.java` | `pcbnew/autorouter/drill/DrillPageArray.h/.cpp` | Active, attempt-owned, resource-bounded row-major page array. Area-only overlap queries use the source bounding-page range. ExpansionGraph still has legacy landmark sampling for fallback. |
| `autoroute/drill/ExpansionDrill.java` | `pcbnew/autorouter/drill/ExpansionDrill.h` | Drill centroid/region, rooms across the full physical stack, per-layer occupation and source hash. Through transitions materialize as ordinary `PCB_VIA`; general padstacks are not implemented. |
| `autoroute/AutorouteAttemptResult.java` | `AutorouterTypes.h` (`ROUTING_RESULT`) | Worker result, completion state, metrics, message and cancellation. |
| `autoroute/AutorouteAttemptState.java` | `ROUTER_PROGRESS` / `AUTOROUTER_JOB` | Progress state is polled by the native dialog; fanout connections are reported separately in `ROUTER_METRICS`. |
| `autoroute/BoardHistory*.java` | `BOARD_HISTORY` plus proposal-owned objects | Bounded duplicate-free worker snapshots use source-normalized scoring, restore counts/ranks, and a DRC-first native safety stratum; KiCad `BOARD_COMMIT` still owns editor undo and rejected proposals never enter the board. |
| `autoroute/pipeline/AutoroutePassRunner.java` | `AUTOROUTE_PASS_RUNNER` | Rebuilds the natural-order item-set work list each pass, including plane false-work suppression and exact-island host-repair handling. |
| `autoroute/pipeline/AutorouteBatchLoop.java` | `AUTOROUTE_BATCH_LOOP` | Implements pass-8/modulo-4 history cadence, 0.5-point local/global stagnation windows, restore reset, and one-time fanout recovery; job/thread time-limit orchestration remains host-owned. |
| `autoroute/ItemAutorouteInfo.java` | `BOARD_SNAPSHOT`, `ROUTING_PAD`, `ROUTING_NET` | Immutable worker-side topology. |
| `autoroute/ItemRouteResult.java` | `ROUTING_CONNECTION` | Per-connection result. |
| `autoroute/ItemSelectionStrategy.java` | `BATCH_AUTOROUTER::orderNets` | Stable priority, pad-count and half-perimeter ordering. |
| `autoroute/PerformanceProfiler.java` | `ROUTER_METRICS` | Native elapsed time, expanded nodes, passes, rip-ups, vias and length. |
| `autoroute/RoutingFailureLog.java` | `ROUTING_RESULT::message` | User-visible failure status; detailed diagnostics belong in the future parity logger. |
| `autoroute/events/*.java` | `pcbnew/autorouter/events/*.h` and `AUTOROUTER_JOB` progress callback | Immutable callback/event values replace Java event objects because wxWidgets and KiCad views are main-thread only. |

## Board, rules, geometry and transaction mapping

| Freerouting concept | KiCad equivalent | Native file |
|---|---|---|
| `board.facade.RoutingBoard` | Partial mutable copper/contact graph with R-tree queries, source-order normal contacts, polyline items/junction splitting and transactions; forced insertion and room mutation are still incomplete | `board/facade/RoutingBoard.h/.cpp`; input capture remains in `KicadBoardAdapter` |
| `RoutingBoardSearchFacade` / shape search tree | Snapshot obstacles, outline and keepouts | `board/KicadBoardAdapter.cpp`, `maze/MazeSearchEngine.cpp` |
| `RoutingBoardOperations` | Live edits use `BOARD_COMMIT` on the editor thread; a private `BOARD` is used by host validation on the worker. Neither implements reference forced insertion | `AutorouterTool.cpp`, `board/KicadRoutingSession.cpp` |
| `RoutingBoardUndoFacade` / board history | `BOARD_COMMIT` | KiCad-provided API; one accepted proposal is one commit |
| `board.trace.PolylineTrace*` | `PCB_TRACK` segments | `KicadBoardAdapter::CreatePreviewItems` |
| `autoroute.maze.MazeTraceShover`, `board.optimize.TraceShover` / tighteners | **Not ported.** The similarly named native class only shortens visible paths | `maze/MazeTraceShover.cpp` is legacy cleanup, not reference `checkShoveTraceLine` or recursive trace displacement |
| `board/optimize/ViaOptimizer.java` | `pcbnew/autorouter/board/optimize/ViaOptimizer.h/.cpp` | Active weighted two-trace and bounded plane/fanout via-location candidates. Every replacement is strictly checked; arbitrary source item contact mutation and recursion remain partial. |
| planar `IntPoint`, `IntBox`, `TileShape`, polygon geometry | `VECTOR2I`, `BOX2I`, `SHAPE_LINE_CHAIN`, `SHAPE_POLY_SET` | Adapter conversion; all worker coordinates are integer KiCad IU |
| `rules.Net`, `NetClass`, clearance matrix | `NETCLASS`, `BOARD_DESIGN_SETTINGS`, pad/track own clearance, layer settings | `board/KicadBoardAdapter.cpp` |
| `rules.ViaRule` / via padstacks | Supported through-hole-only subset; entry/exit layers do not define physical span | `rules/ViaRule.h`, `MazeSearchEngine::CanUseSegment`, `BatchAutorouter::buildGeometry` |
| `drc.DesignRulesChecker` | KiCad DRC engine and adapter collision checks | `KicadRoutingSession` refills/validates private proposals before acceptance. QA independently materializes and checks them again. Raw worker checks alone cannot validate a proposal |
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
| `autoroute/expansion/SortedOrthogonalRoomNeighbours.java` | `expansion/SortedOrthogonalRoomNeighbours.h/.cpp` | Free-room ordering and gaps; obstacle-room branch not ported. |
| `autoroute/expansion/ExpansionDoor.java` | `expansion/ExpansionDoor.h/.cpp` | Rectangle overlap/line sections, narrow-door handling, explicit host section-budget guard. |
| `autoroute/maze/AutorouteEngine.java`, `MazeSearchEngine.java` | `maze/MazeSearchEngine90Degree.h/.cpp` | Active free-room lifecycle/frontier; shared lifecycle now resides in `RoomSearchContext.h`, with the through-drill frontier in `MazeSearchEngineMultilayer.cpp`. No shove/rip-up rooms. |
| `autoroute/maze/MazeListElement.java` | `maze/MazeListElement.h` | Exact ordering key; full backtracking state resides in the rectangular search. |
| `autoroute/expansion/TargetItemExpansionDoor.java` | `expansion/TargetItemExpansionDoor.h/.cpp` | Exact integral point/axis/oblique trace intersection with reached rectangular rooms. Diagonal start traces use bounded exact lattice samples around all compensated orthogonal cuts instead of an AABB; general finite-width item shapes and rational endpoints remain open. |
| `autoroute/path/FoundConnectionLocator*.java` | `path/FoundConnectionLocator45Degree.h/.cpp` | Reference corner construction and rectangular corridor subset; no full 45/general geometry. |
| `geometry/planar/IntBox.java`, `FloatPoint.java` | `geometry/planar/IntBox.h`, `FloatLine.h` | Required rectangle/rounding/weighted-distance primitives only. |

Additional drill milestone mapping:

| Reference source | Native file | Scope |
|---|---|---|
| `geometry/planar/IntBox.java` cutout; `PolylineArea.java` split-to-convex | `geometry/planar/IntBox.h`, `PolylineArea.h` | Rectangular cutout order/perimeter ties and sequential hole decomposition, with cancellation/resource failure. Not general polygon decomposition. |
| `autoroute/drill/DrillPage*`, `ExpansionDrill` | `drill/` | Active page/region/physical-room/layer state, not sampled landmarks. |
| `autoroute/maze/MazeExpansionEngine`, `MazeSearchEngine` | `maze/MazeExpansionEngine.h`, `MazeSearchEngineMultilayer.cpp` | One room/page/drill-layer queue with reference page/drill cost primitives. Heuristic, target IDs and geometry adapter remain partial. |
| `board/facade/RoutingBoard.removeTraceTails` and normal-contact splitting | `board/facade/RoutingBoard.cpp`, `pipeline/BatchOptimizer.cpp` | Native exact-junction trimming of generated trace tails/overlapping ends, preserving real pad/plane groups. Not the complete source item-chain/normalization algorithm. |

The Java executable is used only by `scripts/autorouter/{Room,Drill}SearchOracle.java`
and A/B QA; it is not linked, launched, or required by the native editor.

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
core capabilities, not completed forced shove, neckdown or full batch parity.

### Exact geometry / fixed-obstacle contour insertion

`geometry/planar/{Point,Line,Polyline,Simplex}` and
`board/optimize/TraceShover` retain source filenames and package boundaries.
See [geometry/spring-over report](CONVEX-SPRING-OVER-PARITY.md) for the finite
bounded geometry and production orthogonal/rectangle limits, the 2,432-record
Java oracle, endpoint-loss guard and remaining full-port dependencies. This is
not `TraceShover.check/insert`, neckdown, or general forced insertion.
