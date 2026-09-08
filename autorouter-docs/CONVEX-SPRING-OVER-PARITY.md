# Exact line geometry and recursive fixed-obstacle spring-over

**Status: a connected, tested port slice—not full routing parity.**

This pass adds actual recursive contour replacement to insertion. It does not
implement movable-copper shove, partial-progress forced insertion, neckdown, or
complete batch/fanout/optimizer behaviour. Those remain required, not optional.
The protected Java checkout is read-only; Java is used only by QA oracles and A/B
benchmarks, never by the production router.

## Source and implementation boundary

Method baseline: Freerouting source/JAR revision
`a11c0a42d1b3827e5126429c5c9820c4ab5bec7c`.
Board-quality baseline: released **v2.3.0**,
`2d4de019aa89e9fa3dc1dc44e09bf509760cafc1`.

Native files retain the corresponding source package paths:

| Native file under `pcbnew/autorouter/` | Implemented semantics | Deliberately not claimed |
|---|---|---|
| `geometry/planar/Point.h` | Arbitrary-precision finite homogeneous rational coordinates, exact comparison/equality, checked integral conversion | Java class identity, projective infinity, point IDs, complete Point/RationalPoint operations |
| `geometry/planar/Line.h` | Directed integer support lines, arbitrary-precision intersections and side predicates | Complete Line/Direction/Vector API, offsets or floating fast-path equivalence |
| `geometry/planar/Polyline.{h,cpp}` | Support-line representation; rational corners; consecutive parallel/overlap filtering, direction normalization, combine/reverse, approximate length | General swept offsets, pull-tight, splitting/cleaning of all host trace types |
| `geometry/planar/Simplex.{h,cpp}` | Validated bounded full-dimensional convex borders; exact containment, closed intersection; source `LineSegment.borderIntersections`, `TileShape.entrancePoints` and polyline cutout ordering | Unbounded/degenerate Simplex construction, redundant half-plane elimination, convex decomposition, shape–shape cutout/intersection, octagon/offset API |
| `board/optimize/TraceShover.{h,cpp}` | Source recursive counterclockwise contour substitution, nested-obstacle selection, 20-level bound, reversed clockwise attempt, shortest-result selection including clockwise tie | `TraceShover.check/insert`, movable trace/via displacement, general board-item eligibility, connected-pin acid-trap handling |
| `maze/MazeSearchEngine.cpp` | Production adapter for **orthogonal paths around fixed rectangular snapshot obstacles**, preserving layer transitions and endpoints | General-angle swept geometry, curved/holed/convex obstacle clearance compensation, source item allocation/order |
| `path/FoundConnectionInserter.{h,cpp}` | On failed preflight, try spring-over inside the same transaction, strictly recheck all resulting edges, commit or roll back | `RoutingBoard.insertForcedTracePolyline` partial-progress return contract, recursive forced-pad/via insertion, neckdown |
| `pipeline/BatchAutorouter.cpp` | Publish the actually inserted replacement path, not the stale search proposal | Reference batch item selection or full fanout policy |

The new line kernel is also used by active exact-junction intersection checks.
Rational intersections are no longer discarded by the planar kernel; the host
junction adapter still requires integral trace vertices and refuses to round a
rational crossing into a false endpoint contact. This is a remaining item-model
boundary, not evidence that rational contacts are fully ported.

## Production safety and mutation lifecycle

1. Snapshot the worker board, contacts, route records and congestion usage.
2. Remove only the explicitly selected generated-route victims speculatively.
3. Preflight the original path. If blocked, ask the fixed-obstacle spring-over
   adapter for a replacement. Unsupported geometry produces no replacement.
4. Require identical endpoints, integral corners, valid original layer changes,
   and strict clearance/outline/drill/occupancy checks on **every** new edge.
5. Add the replacement and commit once. Batch storage and output emission receive
   the replacement nodes, not the original proposal. On failure or cancellation,
   restore earlier routes, item IDs, contact graph and usage.

Cancellation is latched through nested checks. Tests inject cancellation at every
poll boundary, including after rip-up, inside recursion and after addition. The
failed-edge diagnostic continues to identify the **original** input edge even
when the rejected replacement has a different number of segments.

No original host copper is made movable by this pass. The host-session full DRC,
refill/connectivity validation and acceptance rules remain unchanged.

## Differential testing and two important reference findings

`ConvexGeometryOracle.java` calls the actual Java Line, Polyline, TileShape and
Simplex methods. Its 2,048 records include 1,024 line intersections and 1,024
convex entrance/cutout cases, with negative coordinates, rational vertices,
nonorthogonal support lines and ordered boundary results. The fixture describes
Java's canonical border ordering; **Simplex border sorting is not being tested**.

`SpringOverOracle.java` builds 384 real Java boards and invokes the actual
`TraceShover.springOverObstacles`. Cases cover both directions, nested and
intersecting obstacles, looping paths, blocked endpoints and no-op paths.
The same coordinates, half-widths and source-returned clearance are used on both
sides. There is no copied Java comparator or synthetic expected-route generator.

All **2,432 records agree**. That statement is scoped to these methods/inputs,
not to arbitrary shapes or complete routing runs.

### Boundary touches and source search-tree clearance

The first comparison exposed a difference between **closed obstacle collision**
and **entering the convex interior**. A trace that touches a query boundary can
be an obstacle candidate even when `entrancePoints` returns no interior crossing.
Both predicates are implemented separately.

The pinned class-0 reference fixture also exposes a search-tree inconsistency:
`clearanceValue(..., true)` adds its 16-source-unit safety margin, whereas the
broad-phase bounds use the clearance matrix's unpadded maximum. With class 0 and
zero configured clearance, this broad phase only admits the original half-width
query. The wrapping contour is expanded by half-width + 16 + 1. The oracle
models those two different shapes explicitly, which closed the first mismatch;
it does not quietly equate the two checks. Production uses the host-resolved
clearance and a one-IU contour margin, **not this class-0 broad-phase omission**.
These numbers are not silently interpreted as millimetres or transplanted from
Java's DSN coordinate scale.

### Reference looping-path endpoint loss

One pinned example returns a path starting at `(2,25)` instead of the original
`(-100,-10)` after contour replacement/cutout of a looping path. The Java output
is retained in the fixture and matched by the core translation. The production
adapter's `HasSameEndpoints` guard rejects that result. A dedicated regression
checks the actual endpoint-loss example against the exact guard. Matching a
reference bug is not permission to accept an electrically incomplete route.

## What still blocks a faithful full port

The previous [complete acceptance checklist](PARITY-CLOSURE-CHECKLIST.md) remains
open. In implementation order:

1. Full finite/unbounded/degenerate convex geometry, IntOctagon and offset/swept
   polyline shapes; exact room/door/drill search using those shapes. Current maze
   geometry is still rectangular, with a grid/visibility fallback.
2. Mutable item-level ownership, source fixed states, source reverse-ID contact
   order/chain traversal, full split/combine normalization and rational contacts.
   Worker-generated copper must remain distinguishable and mutable across host
   refill/repair snapshots instead of being frozen as original copper.
3. Source `ForcedPadRouter`, `ForcedViaInserter`, `TraceShover.check/insert`, drill
   movement and `RoutingBoard.insertForcedTracePolyline`: recursive trace/via
   displacement, per-type recursion limits, partial progress, changed-area
   normalization, and rollback/invalidation of all affected search caches.
4. Width and clearance profiles that survive search, insertion, occupancy,
   optimization, materialization and host minimum-width checking. Only then port
   pin exit constraints, `tryNeckDown`, fanout micro-neckdown and forced entry.
   `ROUTING_CONNECTION` still has one implied net width; narrowing a search edge
   alone would emit the wrong copper and is not a valid implementation.
5. Full batch item/net order, plane direction/stopping, source pass costs/time
   limits, connection-chain rip-up, cycle detection and best-board restoration.
   Fanout sorting is tested, but radial synthetic landing planning is not the
   reference forced breakout algorithm.
6. 90/45/any-angle pull-tight, movable vias, plane/fanout optimization and actual
   parallel optimizer semantics; complete context-sensitive host rules and
   acceptance/reject/undo/redo/refill GUI lifecycle tests.

Names matching Java files do not close these items. Each needs method-level
oracles plus physical completion/full DRC on repeated real-board checkpoints.

## Reproduce

From the native repository (never the protected reference checkout):

```sh
cmake --build build/autorouter --target pcbnew pcbnew_kiface qa_pcbnew qa_autorouter_parity -j 6
build/autorouter/qa/tests/pcbnew/qa_pcbnew --run_test=NativeAutorouter --report_level=short
python3 scripts/autorouter/run_room_oracle.py --oracle convex \
  --reference-jar /tmp/freerouting-reference/build/libs/freerouting-current-executable.jar \
  --java /opt/homebrew/opt/openjdk@25/bin/java \
  --javac /opt/homebrew/opt/openjdk@25/bin/javac --output-dir /tmp/new-convex-check
# Repeat with --oracle spring and a different fresh output directory.
python3 -m unittest discover -s scripts/autorouter -p 'test_*.py'
```

The oracle runner refuses a wrong JAR revision and records source, fixture and
JAR SHA-256 hashes. It does not overwrite existing output directories or update
golden data to make a failed comparison pass.

## Validation from this pass

- Before edits: 81 native tests / 346,812 assertions passed.
- Built `pcbnew`, the actual bundled `_pcbnew.kiface`, `qa_pcbnew` and the parity
  runner. Building is not a claim of GUI acceptance testing.
- Focused native/DRC/zone/MatchProperties selection: **347 cases / 351,765
  assertions passed**, including 87 native autorouter cases.
- Java oracles: **2,432 exact normalized records matched**.
- Standalone ASan + UBSan build of all three new geometry/spring-over translation
  units: all 2,432 records passed without sanitizer findings. Leak detection was
  disabled; this is not a whole-application sanitizer or peak-memory result.
- Python harness: **21 tests passed**.

Full-suite and fresh three-board A/B results are recorded below when completed.
