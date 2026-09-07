# Freerouting upstream mapping

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
| Native port root | `pcbnew/autorouter/` |

The reference commit is deliberately recorded as a commit rather than a release tag.  Any
algorithmic synchronization must update the hash, the mapping below, and the parity corpus in the
same change.

## Routing-file mapping

| Freerouting source | KiCad native source | Classification / adaptation |
|---|---|---|
| `autoroute/maze/AutorouteControl.java` | `pcbnew/autorouter/maze/AutorouteControl.h/.cpp` | Port with KiCad settings values; retains via, trace, direction and congestion cost responsibilities. |
| `autoroute/maze/AutorouteEngine.java` | `pcbnew/autorouter/maze/AutorouteEngine.h/.cpp` | Per-snapshot engine owner; `AUTOROUTER_JOB` owns the native thread and the engine owns drill/search state. |
| `autoroute/maze/DestinationDistance.java` | `pcbnew/autorouter/maze/DestinationDistance.h/.cpp` | Layer-aware lower-bound destination estimate used by the maze frontier. |
| `autoroute/maze/MazeSearchEngine.java` | `pcbnew/autorouter/maze/MazeSearchEngine.h/.cpp` | Port with a KiCad-IU, layer-aware search frontier and adapter collision queries, including copper-to-hole and hole-to-hole drill spacing. |
| `autoroute/maze/MazeExpansionEngine.java` | `pcbnew/autorouter/maze/MazeExpansionEngine.h/.cpp` | Owns deterministic orthogonal/diagonal/layer neighbor expansion so search-order changes have a dedicated synchronization point. |
| `autoroute/maze/MazeListElement.java` | `pcbnew/autorouter/maze/MazeListElement.h` | Stable value frontier record retaining expansion/sorting/sequence ordering. |
| `autoroute/maze/MazeSearchElement.java` | `pcbnew/autorouter/maze/MazeSearchElement.h` | Value backtracking record for immutable snapshot search. |
| `autoroute/maze/MazeFanoutDiagnostics.java` | `pcbnew/autorouter/maze/MazeFanoutDiagnostics.h` | Optional callback diagnostics with no GUI or logger dependency. |
| `autoroute/maze/MazeRipupResolver.java` | `pcbnew/autorouter/maze/MazeRipupResolver.h/.cpp` | Port of deterministic victim selection. |
| `autoroute/maze/MazeTraceShover.java` | `pcbnew/autorouter/maze/MazeTraceShover.h/.cpp` | Visibility shortening and redundant-via cleanup used by the optimizer. |
| `autoroute/pipeline/BatchAutorouter.java` | `pcbnew/autorouter/pipeline/BatchAutorouter.h/.cpp` | Port of net ordering, pass sequencing, retries, rip-up and result materialization. |
| `autoroute/pipeline/BatchAutorouterThread.java` | `pcbnew/autorouter/pipeline/BatchAutorouterThread.h/.cpp` | Algorithm-side worker entry; `AUTOROUTER_JOB` owns the wx-safe native thread. |
| `autoroute/pipeline/AutoroutePassRunner.java` | `pcbnew/autorouter/pipeline/BatchAutorouter.cpp` | The C++ worker is single-threaded per job; the board snapshot makes the pass boundary explicit. |
| `autoroute/pipeline/AutorouteAirlineCalculator.java` | `pcbnew/autorouter/pipeline/AutorouteAirlineCalculator.h/.cpp` | Computes topology lower bounds for metrics and future board scoring. |
| `autoroute/pipeline/BatchFanout.java` | `pcbnew/autorouter/pipeline/BatchFanout.h/.cpp` | Snapshot-side SMD fanout and plane-target classification; fanout escapes are synthetic worker pads and become ordinary KiCad vias/tracks at acceptance. |
| `autoroute/pipeline/BatchOptimizer.java` | `pcbnew/autorouter/pipeline/BatchOptimizer.h/.cpp` | Port of cleanup responsibility; KiCad route objects are emitted only after optimization. |
| `autoroute/pipeline/BatchOptimizerMultiThreaded.java` | `pcbnew/autorouter/pipeline/BatchOptimizerMultiThreaded.h` | Snapshot-safe optimizer wrapper; candidate parallelism is deferred until occupancy copies are independent. |
| `autoroute/pipeline/OptimizeRouteTask.java` | `pcbnew/autorouter/pipeline/OptimizeRouteTask.h` | Data-only optimization candidate task; never deep-copies a live KiCad `BOARD`. |
| `autoroute/pipeline/RoutingPipeline.java` | `pcbnew/autorouter/pipeline/RoutingPipeline.h/.cpp` | Stable orchestration seam for future fanout and diagnostics. |
| `autoroute/path/Connection.java` | `pcbnew/autorouter/path/Connection.h/.cpp` plus `ROUTING_CONNECTION` | Data-only value wrapper; path nodes retain layer transitions. |
| `autoroute/path/FoundConnectionInserter.java` | `BatchAutorouter.cpp` and `KicadBoardAdapter.cpp` | KiCad transaction creates ordinary tracks/vias after proposal acceptance. |
| `autoroute/path/FoundConnectionLocator.java` | `pcbnew/autorouter/path/FoundConnectionLocator.h/.cpp` | Endpoint and path reconstruction responsibility; delegates to the maze engine. |
| `autoroute/path/FoundConnectionLocator45Degree.java` | `pcbnew/autorouter/path/FoundConnectionLocator45Degree.h` | 45-degree angle-policy synchronization seam. |
| `autoroute/path/FoundConnectionLocatorAnyAngle.java` | `pcbnew/autorouter/path/FoundConnectionLocatorAnyAngle.h` | Any-angle policy synchronization seam. |
| `autoroute/expansion/*.java` | `pcbnew/autorouter/expansion/*.h/.cpp` plus `ExpansionGraph` | Room/door value model and visibility landmarks; the search retains a complete adaptive cell fallback until room-expansion parity is validated. |
| `autoroute/drill/DrillPage.java` | `pcbnew/autorouter/drill/DrillPage.h/.cpp` | Snapshot-safe spatial page for via candidates. |
| `autoroute/drill/DrillPageArray.java` | `pcbnew/autorouter/drill/DrillPageArray.h/.cpp` | Page index used by `AutorouteEngine` for deterministic via landmarks. |
| `autoroute/drill/ExpansionDrill.java` | `pcbnew/autorouter/drill/ExpansionDrill.h` | Data-only via transition record; transitions materialize as ordinary `PCB_VIA`. |
| `autoroute/AutorouteAttemptResult.java` | `AutorouterTypes.h` (`ROUTING_RESULT`) | Worker result, completion state, metrics, message and cancellation. |
| `autoroute/AutorouteAttemptState.java` | `ROUTER_PROGRESS` / `AUTOROUTER_JOB` | Progress state is polled by the native dialog; fanout connections are reported separately in `ROUTER_METRICS`. |
| `autoroute/BoardHistory*.java` | `BOARD_HISTORY` plus proposal-owned objects | Bounded worker-side connection snapshots retain the best negotiated/routed state; KiCad `BOARD_COMMIT` still owns editor undo and rejected proposals never enter the board. |
| `autoroute/ItemAutorouteInfo.java` | `BOARD_SNAPSHOT`, `ROUTING_PAD`, `ROUTING_NET` | Immutable worker-side topology. |
| `autoroute/ItemRouteResult.java` | `ROUTING_CONNECTION` | Per-connection result. |
| `autoroute/ItemSelectionStrategy.java` | `BATCH_AUTOROUTER::orderNets` | Stable priority, pad-count and half-perimeter ordering. |
| `autoroute/PerformanceProfiler.java` | `ROUTER_METRICS` | Native elapsed time, expanded nodes, passes, rip-ups, vias and length. |
| `autoroute/RoutingFailureLog.java` | `ROUTING_RESULT::message` | User-visible failure status; detailed diagnostics belong in the future parity logger. |
| `autoroute/events/*.java` | `pcbnew/autorouter/events/*.h` and `AUTOROUTER_JOB` progress callback | Immutable callback/event values replace Java event objects because wxWidgets and KiCad views are main-thread only. |

## Board, rules, geometry and transaction mapping

| Freerouting concept | KiCad equivalent | Native file |
|---|---|---|
| `board.facade.RoutingBoard` | `KICAD_BOARD_ADAPTER` + `ROUTING_BOARD_INTERFACE` | `board/KicadBoardAdapter.h/.cpp`, `board/RoutingBoardInterface.h` |
| `RoutingBoardSearchFacade` / shape search tree | Snapshot obstacles, outline and keepouts | `board/KicadBoardAdapter.cpp`, `maze/MazeSearchEngine.cpp` |
| `RoutingBoardOperations` | `BOARD`, `PCB_TRACK`, `PCB_VIA`, `BOARD_COMMIT` | KiCad-provided APIs; used only on the editor thread |
| `RoutingBoardUndoFacade` / board history | `BOARD_COMMIT` | KiCad-provided API; one accepted proposal is one commit |
| `board.trace.PolylineTrace*` | `PCB_TRACK` segments | `KicadBoardAdapter::CreatePreviewItems` |
| `board.optimize.TraceShover` / tighteners | `MAZE_TRACE_SHOVER` called by `BatchOptimizer` | `maze/MazeTraceShover.cpp`; future direct ports must preserve the file mapping |
| `board.optimize.ViaOptimizer` | `ROUTING_VIA` cleanup seam | `pipeline/BatchOptimizer.cpp` and adapter |
| planar `IntPoint`, `IntBox`, `TileShape`, polygon geometry | `VECTOR2I`, `BOX2I`, `SHAPE_LINE_CHAIN`, `SHAPE_POLY_SET` | Adapter conversion; all worker coordinates are integer KiCad IU |
| `rules.Net`, `NetClass`, clearance matrix | `NETCLASS`, `BOARD_DESIGN_SETTINGS`, pad/track own clearance, layer settings | `board/KicadBoardAdapter.cpp` |
| `drc.DesignRulesChecker` | KiCad DRC engine and adapter collision checks | `drc/` APIs remain authoritative; the worker mirrors snapshot-safe clearance, hole, outline, keepout, and route-pair checks before commit, while the parity harness runs live KiCad DRC on a fresh board |
| `ConductionArea` / plane behavior | Filled KiCad zones plus synthetic worker landing pads | `KicadBoardAdapter` adds a small deterministic set of targets per filled same-net zone/layer and keeps the filled polygon traversable only for that net; isolated island/connectivity semantics still require parity coverage |

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

1. The board is never serialized through DSN/SES.  The adapter copies only the immutable inputs
   required by the worker and creates ordinary KiCad objects at acceptance.
2. wxWidgets and KiCad views are never touched from the worker thread.
3. The preview is a `VIEW_ITEM`, not a `BOARD_ITEM`; this is what makes Reject exact and keeps the
   pre-run board out of the undo stack.
4. KiCad's integer geometry and rule APIs replace Freerouting's planar model.  The adapter is the
   only place where that conversion is allowed.
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
