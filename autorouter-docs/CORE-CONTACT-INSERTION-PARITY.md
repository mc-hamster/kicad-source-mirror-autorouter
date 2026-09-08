# Core contacts, insertion and fanout — 2026-09-08

**This is a partial core implementation, not a faithful full port yet.** In
particular, checked insertion is not forced insertion, and normal-contact
splitting is not a shove engine. The completion/quality failures on the saved
boards must not be disguised by passing primitive tests.

Source pin: Freerouting `a11c0a42d1b3827e5126429c5c9820c4ab5bec7c`.
Board-quality baseline: official v2.3.0,
`2d4de019aa89e9fa3dc1dc44e09bf509760cafc1`.
The reference checkout remains read-only; Java runs only from the separate
revision-checked QA JAR. Production is still entirely native.

The native checkout started at `69f5d3156c` with the prior destination translation
uncommitted. During this pass an external commit advanced HEAD to `bc29a0567b`,
including most of the then-current changes. No commits or staging were performed
by this pass. Evidence must record the working-tree patch and new golden files,
not assume either commit alone reproduces the tested binary.

## Implemented and active

### Normal contacts are distinct from physical copper overlap

`board/model/items/NormalContacts.h` translates the predicates from Java
`Trace`, `DrillItem`, and `ConductionArea`:

- Trace/trace: common endpoints on a common layer, not arbitrary copper overlap.
- Trace/pin/via: endpoint equals the pin/via centre **exactly**, with a common layer.
- Pin/via pairs: coincident centres, common layer.
- Trace/area: an endpoint inside the area; drill/area: centre inside the area.
- Area/area overlap is not a normal contact.
- Net sharing is required for contact sets. As in Java, `normalContactPoint`
  itself does not filter nets, returns no point for areas, and returns no unique
  point for traces with both endpoints in common.

`RoutingBoard` keeps this graph separately from its physical connectivity graph.
`GetNormalContacts`, `NormalContactPoint`, `NormalConnectedSet`, `RouteItems` and
`PadItem` expose stable worker identities rather than raw indexed pointers.
Membership agreement is not traversal-order parity: Java `Item.compareTo`
orders newer IDs first (`other.id - id`); native graph containers still use
ascending IDs. Reference ID allocation/iteration and connection-chain traversal
must be translated before using this graph as a source-equivalent batch/shove
scheduler. The current two-item oracle verifies membership/contact points, not
that multi-item traversal order.

Existing host pad-group unions remain physical evidence only; they do not create
invented normal contacts. The batch's physical completion test has **not** been
silently replaced by the stricter normal-contact graph while target attachment
and pin exit restrictions are still incomplete.

`geometry/planar/ContactGeometry.h` implements exact straight-segment contact
predicates using wide integer intermediates, not epsilon tests. Integer line
crossings and collinear endpoints split worker traces; rational crossings are
explicitly **not rounded into integer junctions**. Polygon membership preserves
the source integer-point rule: outer boundaries and hole boundaries are included,
hole interiors are excluded. This is not the full rational/convex geometry port.

New routes split both their own straight segments and contacted straight copper
in the worker graph. The original ID stays on the first piece; extra pieces get
new IDs. Generated pieces stay owned by their original route for removal.
Retained host traces are split **virtually**: their actual KiCad object/UUID and
geometry remain untouched, and virtual retained pieces are never emitted as new
routing output. Arc tessellations with multiple shapes are not mislabeled as a
single straight trace. Merge/normalization, exact rational contacts, fixed-state
chains, all pin exit restrictions and general item mutation remain open.

The same exact predicates now back the optimizer's `TraceJunctions`. Its old
`SEG::SquaredDistance(...) == 0` check was not exact: KiMath rounds squared
distances, so a point 0.316 IU off a line could be mistaken for a junction.
Ordering now avoids overflowing squared integer norms as well.

### Checked insertion and rip-up are one transaction

The active batch no longer deletes victims and then blindly appends a candidate.
It calls `AutorouteEngine::InsertConnection` → `FoundConnectionInserter::Insert`:

1. Validate the candidate and take a worker transaction.
2. Remove only specified generated route victims. Retained host copper and
   synthetic fanout bridges are not made freely rippable.
3. Check every new trace/via edge against the post-rip-up state.
4. Add copper, split exact contacts, and update route records/congestion usage.
5. Commit together, or restore everything on failure/cancellation/exception.

The result distinguishes inserted, blocked, cancelled and invalid candidates,
including the failed edge index. Cancellation after copper insertion still
rolls back. Route-result vectors and rip-up counters change only after success.

Two additional correctness problems were closed:

- Strict insertion checks cannot inherit the maze's negotiated-crossing flag.
- A new trace endpoint is checked at **trace radius**, not the radius of its
  existing pad. Rechecking the whole pad envelope incorrectly rejected valid
  escapes in both split-plane repair tests.

The rollback snapshot includes original item IDs, route ownership, both contact
graphs, usage maps and a spatial index pointing at the saved item storage. The
rollback destructor swaps saved state without allocating/reindexing during
exception unwinding. It restores the ID allocator but advances the board revision
to invalidate derived connectivity. Current room frontiers are per-attempt;
this does **not** establish incremental persistent room/drill cache parity.
Copying/indexing snapshots has a cost; a source-style incremental undo journal is
still desirable before scaling recursive shove to dense boards.

This API rejects a blocked route atomically. It does **not** yet implement Java's
`insertForcedTracePolyline` partial-progress search, recursive displacement,
spring-over, neckdown, or alternative forced-via mask search.

### Reference component/pin fanout ordering and real pass bounds

The adapter now preserves component ordinals and distinct package-pad indices
(even for duplicate displayed pad numbers). A disabled copper routing layer no
longer misclassifies a through-hole pad as SMD.

`BatchFanout::OrderedPins` implements the source component/pin order:

- Components with more net-assigned SMD pins first; component ID breaks ties.
- Outer-first (default), inner-first, nearest same-net pin, densest surroundings
  within 20 mm, and pin-index fallback orders; package index breaks pin ties.
- Pin-centroid, nearest-net and density populations follow their respective
  source rules. Synthetic and plane sample targets are not physical package pins.

Both landing planning and active fanout attempts use this global order rather
than net order. Graph rewriting occurs once after planning, preserving every
source-to-landing bridge when pins from different nets interleave.

`maxFanoutPasses` now bounds actual routing passes; previously it bounded radial
landing sampling. Sampling has its own explicit native-adapter setting.
The pass loop stops on no routed pins, repeated routed/via-count outcomes,
unchanged route geometry, cancellation, or the pass limit. It schedules one pin
at a time and rechecks existing connectivity.

This is **not full reference fanout**: landing geometry remains a synthetic
sampling adapter, protected bridges differ from source connection-chain rip-up,
time-per-pin/escalation, forced breakout and auto/micro-neckdown remain missing.
Source-equivalent board hashes, item masks and all fanout settings are not proven.
Component identity follows the host footprint traversal; package export/import
reordering and user-filtered populations require additional end-to-end proof.

## Differential and regression proof

- `NormalContactsOracle.java` calls actual pinned item methods on 2,048 two-item
  boards, covering both directions, nets, layers, endpoint ambiguity, copper
  overlap without normal contact, areas and holes. It does not copy the predicates.
- `FanoutOrderOracle.java` instantiates actual pinned `BatchFanout` and reads its
  sorted components/pins for 128 mixed boards × 5 modes = **640 orders**.
  Both engines receive identical coordinates and physical units. An initial
  harness incorrectly scaled only its output coordinates: floating-point
  centroid/tie decisions are not scale invariant. That evidence is retained;
  agreement here is explicitly for the same coordinate domain, not proof of
  complete DSN/native coordinate normalization.
- With units aligned, record **176** still differed because of C++ floating
  contraction. Scoped no-contraction compilation fixes that ordering divergence;
  the oracle uses exact sequence comparisons, no relaxed tie tolerances.
- Native regressions cover generated and retained T-junctions, nested rollback,
  stable/reused IDs, pad-edge physical contact versus normal contact, exact and
  half-integer crossings, extreme-coordinate arithmetic, sub-IU false junctions,
  late blocked edges, invalid transitions, cancellation before/during/after
  insertion, stale negotiated permissions, package metadata and actual global
  landing order.
- A separate ASan/UBSan run recompiles the worker board and performs 1,000
  crossing/splitting/nested-transaction cycles. Linked KiMath/host libraries are
  not sanitizer-rebuilt, leak detection is disabled, and this is not a full-app
  sanitizer or large-board peak-memory claim.

Reproduce the pinned oracles with `scripts/autorouter/run_room_oracle.py`, using
`--oracle contacts` or `--oracle fanout`, an independently built pinned JAR, and
a fresh output directory. Golden fixtures remain required repository files even
if an external commit leaves them untracked.

## Still missing — requested features and additional blockers

| Required core area | Remaining work; do not confuse the implemented subset with completion |
|---|---|
| Full geometry/contact semantics | Rational points/lines, polyline line-intersection identity, convex decomposition, octagons/simplexes, offsets/cutouts/compensation, thin/acute rooms, 45/any-angle restrictions, curved/custom pad geometry, complete normal-contact and pin-exit semantics, split/combine/remove chains. Physical KiMath contours and immutable host groups remain adaptations. |
| Forced trace/via insertion | Translate `RoutingBoard.insertForcedTracePolyline`, `connectToTrace` and normalization, `ForcedPadRouter`, and `ForcedViaInserter.checkLayer/check/insert`. Preserve partial insertion results, changed-area tracking, complete padstack masks and entry-side selection. Atomic candidate preflight alone is not these algorithms. |
| Shove/spring-over | Translate both `autoroute/maze/MazeTraceShover` (obstacle-room/door expansion) and `board/optimize/TraceShover` (recursive check/insert/spring-over), plus drill-item movement and fixed-state/clearance-class restrictions. A displacement must preserve every affected contact and invalidate spatial/room/drill data. `Shorten` still does none of this. |
| Neckdown | Port `Pin.getTraceNeckdownHalfwidth`, pin max-width/clearance distance gates, `checkTraceSegment` partial lengths, source corner construction and `FoundConnectionInserter.tryNeckDown`/fanout micro-neckdown. **The native connection/output model currently assumes one net width**: variable-width pieces must survive search, insertion, occupancy, output, host minimum-width checking and undo. Adding a smaller-width heuristic or lowering DRC limits would be incorrect. |
| Batch/fanout | Full reference connected/unconnected item-set ordering/direction, plane early exits, pass costs/time limits, connection-chain rip-up, best-board checkpoints and cycle detection. Fanout needs real constrained via landing/forced breakout, pin exit restrictions, source pass settings, and dense/BGA fixtures. The new ordering/pass slice is only part of this. |
| Rules/vias | Actual clearance-class compensation, per-layer/item width and clearance, contextual host rules, several via rules/padstacks, blind/buried/microvia masks and inactive-layer copper/drill restrictions. Match source cost preparation; native UI/fallback cost mappings still differ. |
| Search/location | Full convex/45 search trees, obstacle expansion rooms, true item-region target doors, shove-aware maze state, alternative padstack transitions, any-angle/thin-room backtracking and end-to-end decision streams. The rectangular room/drill frontier still falls back to a different grid/visibility router. |
| Ownership/planes | Job-created copper becomes frozen together with user copper across host refill/repair snapshots. Preserve original/job UUID ownership and mutable contacts across repair; do not enable blanket existing-copper rip-up. Sampled plane regions, disconnected pours and plane/fanout via optimization remain incomplete. |
| Optimization | Reference 90/45/any-angle pull-tight, via movement/removal, changed-area optimization, rerouting selection/scoring, termination and genuine multithreaded optimizer behaviour. Current guarded cleanup/serial delegation is not a port. |
| Integration and evidence | Native preview/reject/accept/one-step undo/redo with zones and unsaved rules, cancellation/ownership lifecycle, unsupported-input signalling, dense/multilayer/locked-copper corpus, deterministic item/pass checkpoints, completion/full DRC, quality, runtime and peak memory. No GUI parity claim follows from a headless build. |

The [16-area acceptance checklist](PARITY-CLOSURE-CHECKLIST.md) remains the complete
acceptance scope. None of its full-port rows can be marked done by this milestone.
The next inseparable slice is variable-width/fixed-state item mutation plus
convex/polyline geometry and source forced insertion; recursive shove/neckdown
must be built on that, not substituted with another reroute heuristic.

## Validation and A/B evidence

Final verification and saved-board measurements are recorded below after the
final source build. Existing archives and the three original board assets are
not overwritten.

### Final verification

- Custom PCB editor module/application and both QA executables built successfully.
- Native: **81 cases / 346,812 assertions passed**.
- Native + DRC + zone + MatchProperties: **341 cases / 349,158 assertions passed**.
- Full PCB suite: **2,405 / 2,406 cases pass (including 19 warning cases)**;
  the existing order-dependent `MatchProperties/KeysAreCanonicalAndLabelsAreFriendly`
  label assertions still fail. 535,221 / 535,223 assertions pass; 22 failed warnings.
  The full suite is **not green**. The same label case passes in the focused suite.
- Five pinned Java oracles regenerate unchanged: contacts 2,048; fanout 640;
  destination 7,712; room 7,166; drill 1,280 records.
- Python harness/comparator tests: **21 pass**.
- ASan/UBSan: **1,000 contact/split/nested-rollback cycles pass** in the documented
  worker-board scope, with leak detection disabled.
- No GUI acceptance, undo/redo, or large-board peak-memory claim.

### Saved-board A/B results

The final binary was run once per configuration. Two preceding repetitions per
configuration were also run before the last sub-IU junction predicate correction;
all reported output counts/lengths agree. Timing is shown only for the final
binary, not pooled across different builds. An earlier six-case smoke run is
retained separately. No completed output is invented for a timeout.

Both engines use the same stripped inputs, four routing passes, optimizers off,
reference single-threaded, native eight attempts / 250,000 expanded-node limit,
and a 180-second per-process timeout. No-via runs disable fanout in both engines.
Full independent KiCad refill/connectivity/DRC checks remain mandatory.

Values below are **native / v2.3.0 reference**.

| Configuration | Board | Missing | New DRC | Vias | Length mm | Core ms | Gate |
|---|---|---:|---:|---:|---:|---:|---|
| default | regulated-5v | 0 / 0 | 0 / 0 | 0 / 0 | 323.211 / 319.203 | 1727 / 450.0 | Pass |
| default | 555-astable | 0 / 0 | 0 / 10 | 11 / 7 | 96.1797 / 184.248 | 67 / 2090.0 | Fail |
| default | bjt-astable | 0 / 0 | 0 / 0 | 0 / 0 | 145.458 / 152.435 | 28 / 290.0 | Pass |
| no-vias | regulated-5v | 0 / 0 | 0 / 0 | 0 / 0 | 357.391 / 319.203 | 422 / 430.0 | Fail |
| no-vias | 555-astable | timeout / 0 | timeout / 1 | timeout / 0 | timeout / 193.479 | timeout / 1120.0 | Fail |
| no-vias | bjt-astable | 0 / 0 | 0 / 0 | 0 / 0 | 147.318 / 152.435 | 32 / 350.0 | Pass |

Quality parity remains failed: default 555 has four extra native vias, no-via
regulator is about 12% longer, and no-via 555 times out. Default native outputs
are complete with zero new host DRC, but this pass does not fix those quality
failures. Reference DRC failures do not justify weakening native checks.

All **33 available outputs** from the final and preceding repetitions were
independently reloaded with routing disabled; connectivity, full DRC, vias and
length agree, with no generated copper or input-file changes.

Durable local evidence is in
`autorouter-test-assets/online-simple/contact-insertion-2026-09-08/`:
`results/` contains the six final pairs; `experiments/pre-exact-junction/` contains
the twelve preceding pairs; `experiments/smoke/` holds initial diagnosis. All
include saved normalized/reference/native boards where a run completed, logs,
settings, hashes and provenance. `native-source.patch` includes the required
untracked golden files. `native-build-provenance.json` and `sha256.json` identify
the source state, artifacts and archive. Original assets and every older
parity archive were hash-verified unchanged.
