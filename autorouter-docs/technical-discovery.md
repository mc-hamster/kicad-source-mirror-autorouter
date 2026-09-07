# Native autorouter technical discovery

This is the first engineering deliverable for the native full-board autorouter.  It records the
current source-level seams before the port is expanded and is intentionally kept next to the
requirements and upstream mapping.

## KiCad source modules

### Board and geometry

* `pcbnew/board.h/.cpp` owns board-level item containers, UUID lookup, enabled layers, board
  outline and connectivity.
* `pcbnew/pcb_track.h/.cpp` owns `PCB_TRACK` and `PCB_VIA`; their constructors accept a board
  parent, and their layer, width, net, drill and layer-pair setters are sufficient to create
  accepted autoroute output.
* `include/board_item.h`, `pcbnew/board_connected_item.h`, `pcbnew/pad.h`, `pcbnew/zone.h`, and
  `pcbnew/pcb_board_outline.h` expose item geometry and layer sets.
* `libs/kimath/include/math/box2.h`, `libs/kimath/include/math/vector2d.h`,
  `include/geometry/shape_line_chain.h`, and `include/geometry/shape_poly_set.h` provide the
  integer geometry used by the adapter.
* `pcbnew/connectivity/connectivity_data.h` and `pcbnew/ratsnest/ratsnest_data.h` expose the
  current ratsnest.  The adapter uses the ratsnest edges when `routeOnlyUnconnected` is enabled,
  so a completely connected net is not routed again.

### Rules and DRC

* `include/board_design_settings.h` provides copper-to-edge and maximum clearance values.
* `pcbnew/netclass.h` supplies effective track width, via diameter/drill and priority.
* `BOARD_CONNECTED_ITEM::GetOwnClearance`, pad bounding boxes, drilled pad/via holes, rule areas,
  filled-zone polygons, and existing track geometry are copied into `ROUTING_OBSTACLE` records.
* KiCad's DRC implementation is under `pcbnew/drc/` and remains the final authority after
  acceptance.  The worker captures pair-specific copper clearances and the board's
  copper-to-hole and hole-to-hole constraints on the editor thread, then uses conservative
  clearance-expanded collision checks without touching a live DRC engine on a worker thread.
  `qa_autorouter_parity` also attaches a proposal to a freshly loaded board and runs the actual
  KiCad DRC providers, reporting baseline versus newly introduced errors.  Rule types that cannot
  be represented by the immutable snapshot remain a parity-corpus risk and must be covered before
  declaring the MVP complete.

### PNS investigation

* `pcbnew/router/router_tool.cpp` and `pcbnew/router/pns_kicad_iface.cpp` are optimized for an
  interactive state machine and a current mouse route, not a global net scheduler.
* PNS has excellent preview, collision and ordinary-board-item commit code, but no stable public
  API that accepts thousands of independent global routes without carrying interactive state.
* The native implementation therefore preserves Freerouting-derived global ordering, retry,
  congestion and rip-up behavior.  It reuses the same KiCad `BOARD`, geometry and commit
  infrastructure instead of routing through PNS.  This is an intentional parity decision, not an
  omission.

### Transactions and UI

* `pcbnew/board_commit.h/.cpp` stages `Add` and `Remove` changes, updates the view/connectivity,
  and creates one undo entry on `Push`.
* `include/view/view_group.h` is non-owning; `AutorouterPreviewItem` objects are owned by the
  autorouter tool and are added only to a select-overlay group.
* `include/eda_item_flags.h` provides `ROUTER_TRANSIENT`.  Preview board items carry that flag
  until acceptance and are never added to `BOARD` before the commit.
* `pcbnew/tools/pcb_actions.*`, `pcbnew/menubar_pcb_editor.cpp`, `pcbnew/pcb_edit_frame.cpp`,
  and `pcbnew/tools/pcb_tool_base.h` are the native action/tool seams.

## Freerouting routing dependencies

The exact package/file mapping is maintained in [UPSTREAM.md](UPSTREAM.md).  The minimum engine
surface is:

1. `autoroute/maze`: net-specific cost state, layer transitions, search expansion, destination
   detection and rip-up selection.
2. `autoroute/expansion`, `autoroute/path`, and `autoroute/drill`: geometry-aware free-space
   expansion, backtracking, trace/via construction and cleanup.
3. `autoroute/pipeline`: net selection, repeated passes, board scoring/history, optimizer and
   cancellation.
4. `board/facade`, `board/searchtree`, `board/trace`, `board/optimize`, `rules`, `drc`, and
   `geometry/planar`: dependencies that must either be adapted to KiCad or ported without
   changing algorithmic semantics.

GUI (`gui/`), REST/API (`api/`), and Specctra file I/O (`io/specctra/`) are not engine
dependencies.  Freerouting's GUI events are replaced by the thread-safe KiCad job snapshot;
Freerouting's file I/O is intentionally not brought into the native path.

## Risk analysis

| Risk | Impact | Mitigation |
|---|---|---|
| KiCad and Freerouting use different geometry primitives | Search ordering and corner legality can diverge | Keep all conversion in `KicadBoardAdapter`; use integer conservative clearance expansion and parity boards. |
| Clearance matrices are net-pair and layer specific | Over-conservative routes or DRC failures if omitted | Preserve own clearances in obstacles; add pair-aware adapter queries before parity sign-off. |
| Existing copper can be partially connected | Duplicate or unnecessary routes | Convert KiCad ratsnest edges to explicit snapshot connection pairs. |
| PNS interactive state is not a batch API | Global behavior would silently become repeated local routing | Keep PNS out of the global scheduler; use it only after measured parity. |
| Very fine grids create too many search nodes | Runtime and memory regressions | Bound `maxExpandedNodes`, report it, and add coarse-grid/geometry-event search parity tests. |
| Copper pours and plane nets have special semantics | Connectivity and via decisions can differ | The adapter exposes filled same-net zone/layer targets to the worker and leaves foreign-zone polygons blocking; isolated islands, thermal connectivity and plane-specific optimization remain corpus risks. |
| Worker thread and editor mutation race | Corrupt board or stale UUIDs | Immutable snapshot, no KiCad board access in worker, model reset cancels and joins before cleanup. |
| Rejected proposal accidentally enters undo/connectivity | Violates exact reject | Preview is view-only; only `Accept` invokes one `BOARD_COMMIT`. |

## Recommendation

The source boundaries are suitable for a faithful, maintainable C++ port: the algorithm has the
same principal `maze`/`expansion`/`drill`/`path`/`pipeline` seams as Freerouting, while KiCad owns
rules, geometry, board objects, DRC and undo.  The current engine has a deterministic
layer-aware maze frontier, visibility landmarks, negotiated occupancy, repeated rip-up/retry
passes, bounded best-state history, an SMD fanout snapshot stage, and route cleanup.  The room/door
and drill classes are now explicit synchronization points; the remaining parity work is to
replace the adaptive-cell fallback with the full upstream room expansion behavior where corpus
measurements show a quality difference, not to replace it with a second unrelated router.  The
regression corpus is the release gate for declaring Freerouting parity.
