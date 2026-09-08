# Mutable copper rework — 2026-09-07

> Superseded for host completion: [Parity closure checklist](PARITY-CLOSURE-CHECKLIST.md)
> records the production refill/repair implementation and closure of the 555 failure.
> Measurements below are the historical pre-repair milestone.

## Status: production foundation implemented; core parity NOT achieved

This follow-up replaces the native batch router's logical endpoint unions with
worker-owned copper/contact state. It is **not** a port of the room/door search,
forced insertion, shove, or pull-tight algorithms. The grid/visibility frontier
is still active. Do not label this milestone “Freerouting parity”.

Reference source: `a11c0a42d1b3827e5126429c5c9820c4ab5bec7c`.
Performance reference: official Freerouting v2.3.0,
`2d4de019aa89e9fa3dc1dc44e09bf509760cafc1`.
The reference checkout was read only, with no builds or edits in that directory.

## Implemented and connected to the production pipeline

- `board/facade/RoutingBoard.{h,cpp}` owns pad, retained copper, newly inserted
  trace/via edges and separate filled-region items. Stable item IDs, per-layer
  KiMath R-trees and same-net geometric contacts provide connected components.
  Layer-crossing conductivity comes from physical multilayer items, not route
  endpoint labels. Copper widths match proposal materialization.
- Physical pad UUIDs bind actual copper contours to pad terminals. Retained
  vias use their per-layer circular copper, not square bounding boxes. Filled
  region holes and inactive-layer copper remain part of contact geometry.
- Synthetic fanout landings have no copper; they resolve against inserted
  geometry only. A synthetic plane target resolves against a real conduction
  region, never a route merely ending at its coordinate.
- `ROUTING_OCCUPANCY` updates this board on add, remove, clear, and checkpoint
  restore. `BatchAutorouter` rebuilds its routing-order DSU from physical
  components and uses physical contacts for task skipping and completion.
  `AutorouteUnroutedReport` is no longer the production completion authority.
- Connected trace segments and via points are search terminals. Destination
  edges project onto segment interiors, and backtracking ends there rather
  than appending an unchecked jump to a representative pad. Start segments
  seed endpoints and projections toward destinations. This remains **sampled
  start geometry**, not the upstream free-space start-region expansion.
- Shortening is rejected if it disconnects a pre-existing pad component.
  Final cleanup maps synthetic fanout requests back to real pads and removes
  redundant fanout copper/via tails only while preserving real-pad/plane
  components. Cleanup runs even when optional optimization is disabled:
  unused fanout copper is a correctness issue, not a quality preference.
- A private `TRANSACTION` supports speculative copper edits and rollback,
  rebuilding pointer-bearing indexes and invalidating component caches. It is
  tested but **not yet the production forced-insertion transaction**; actual
  shove and insertion failure recovery still have to be ported.
- QA DRC details now include involved item UUIDs. This identified the 555
  failure as a connection between two regions of the same zone UUID
  `690af904-7ff2-4750-a4b8-fc6399ef1cee`, not an unfinished signal-net pad pair.

The contact model uses KiCad-coordinate shape collision, not an exact port of
upstream `Item.getNormalContacts()` and trace normalization. Approximate custom
pad/arc contours retain the adapter's existing outward geometry tolerance.
Immutable input connectivity groups cover retained host items not modeled by
exact shapes. These are explicit limitations, not proven decision parity.

## Tests and acceptance

Native suite: **45 cases / 9,418 assertions passed** (38 cases before this step).
Python parity harness: **12 tests passed**. Both `qa_pcbnew` and
`qa_autorouter_parity` build; their dependency rebuild includes the PCB editor
module. This is not a claim that the entire KiCad test suite was run.

New regressions cover:

1. Logical endpoint labels cannot bridge missing copper.
2. T-junction contact, rip-up, transaction rollback/commit and revision invalidation.
3. Synthetic fanouts require a real via; malformed later edges are rejected
   without partial insertion.
4. Separate same-net regions and polygon holes remain disconnected.
5. The **active pipeline** routes to a retained trace interior.
6. Shortening cannot detach a branch.
7. A route cannot manufacture a plane by reaching a virtual target coordinate.

Three pre-existing plane tests now contain actual plane copper rather than
virtual points. The SMD fanout test checks both the two planned fanouts and
physical connectivity after cleanup, with zero unused vias.

Full A/B results are recorded below after repeated runs. Both engines receive
identically stripped input; both outputs undergo KiCad refill, connectivity
and DRC. Reference JVM startup is excluded from engine timing. Existing local
fixtures and earlier A/B evidence are not overwritten.

## Next implementation work (not completed here)

1. Active compensated room completion, door sections and frontier/backtrack
   state, consuming the mutable item/index model. The current point projection
   is not a substitute for this port.
2. Forced trace/via insertion, splitting/normalizing trace contacts, local shove
   and rollback across all occupancy/search caches. Preserve electrical
   contacts during pull-tight, not just path endpoints.
3. Refill-aware plane connectivity and repair scheduling. The initial region
   can be cut by new foreign copper. Net/request counts cannot detect a new
   zone-only island after refill, including a net with zero initial requests.
   The editor needs host validation before declaring electrical completion.
4. General via rules/padstacks and exact contact semantics, then deterministic
   decision-stream comparisons at multiple bounded routing checkpoints.

Safety gate remains zero new host DRC violations, no completion regression
against v2.3.0, and repeated full-board connectivity checks. Passing unit tests
or a worker “100%” is not that gate.

## Repeated A/B outcome

Three serial runs per board, four passes / eight attempts / 250,000-node budget;
optional optimization disabled in both engines. Median routing-core time:

| Fixture | Native | Freerouting v2.3.0 | Native missing | Reference missing | Native new DRC | Reference new DRC |
|---|---:|---:|---:|---:|---:|---:|
| Regulated 5 V | 1.028 s | 0.410 s | 0 | 0 | 0 | 0 |
| 555 astable | 0.039 s | 1.890 s | **1** | 0 | 0 | 10 |
| BJT astable | 0.019 s | 0.280 s | 0 | 0 | 0 | 0 |

Missing-connection and DRC counts were identical in all three repeats.
“New DRC” excludes missing connections, which have their own columns. Reference
555 violations are nine minimum-width violations and one dangling via under
this board's KiCad rules. Native's earlier seven dangling vias are eliminated;
it now exports eight vias and 85.7163 mm of tracks, but **still has a disconnected
zone region after refill**. This is a failed completion-parity gate, not a win
based on the shorter runtime. Worker task completion still reads 28/28 because
the GND net had zero initial routing requests before new traces split its zone.

The regulator improved from the preceding 1.313 s native median to 1.028 s,
but remains slower than the reference. These are small-board measurements, not
proof of large-board performance or full geometric parity.

Raw output: `build/autorouter/core-rework/verified/`.
Durable local archive (all stripped inputs and both routed boards per repeat,
logs, source patch and SHA-256 manifest):
`autorouter-test-assets/online-simple/mutable-copper-2026-09-07/`.
The assets remain local/ignored; their upstream redistribution licenses have not
been established. Earlier original fixtures and comparison archives are intact.
