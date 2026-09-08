# Active rectangular room/door routing — 2026-09-08

**Later milestone:** [Multilayer room/drill search](DRILL-SEARCH-PARITY.md) extends
this earlier snapshot. The default-multilayer limitations and measured results
below are historical; the archived evidence remains unchanged.

**Milestone, not full Freerouting parity.** This closes the previous situation
where the room/door classes were mostly unused names. It does **not** establish
that arbitrary two-layer boards now use the reference algorithm or meet its
quality/runtime. The [complete checklist](PARITY-CLOSURE-CHECKLIST.md) remains
open, including the previously failing 555 via-count gate.

## Actual production path

`MAZE_SEARCH_ENGINE::FindConnection` tries the new path when via insertion is
disabled or only one routing layer is enabled:

1. `MazeSearchEngineRooms.cpp` builds trace-centre free-space rectangles from the
   snapshot, pair/layer clearances, current copper, full physical via spans and
   board-edge margin. Shapes are rebuilt for each attempt; retained/new copper
   and rip-up removals therefore cannot leave a stale tree.
2. `MinAreaTree` preserves reference insertion ties and second-first traversal.
   `ShapeSearchTree90Degree::CompleteShape` shrinks its traversal query while
   restraining rooms, preserving reference cut/ignore ordering.
3. Free/incomplete/complete rooms, touching neighbours, edge enlargement, gap
   creation and overlap/line doors form an active search space. Neighbours are
   completed before outgoing doors are expanded, because completion mutates the
   door list. Four board-boundary restraints participate in neighbour ordering.
4. Frontier state belongs to a **door section**, with entry geometry and
   backtracking. Ordering is `f, g, door ID, section`, preserving first insertion
   for equal keys. Distance is reference weighted **Euclidean**, not Manhattan.
   Bend detection uses the reference normalized cross-product threshold.
5. `FoundConnectionLocator45Degree` is no longer an alias or another search.
   It reconstructs the selected rectangular corridor using nearest section /
   overlap entries and the reference 90/45-degree corner construction. The host
   requests 45-degree corners. Already compensated rooms are not shrunk twice.
6. Every resulting edge passes the existing exact snapshot collision predicate
   before becoming a proposal. The existing private KiCad session then refills,
   checks actual connectivity and full DRC, and repairs or rejects as before.

There is no Java subprocess, DSN export, SES import or disk intermediate in this
production path. Those mechanisms remain QA-only.

For unsupported geometry or a failed room search, the existing grid/visibility
search remains the fallback, sharing the expansion budget and cancellation.
Multi-layer jobs with vias enabled keep the existing search: greedily accepting
a same-layer route without comparing drill/via alternatives would introduce a
new routing-policy error. Full drill/frontier integration must precede promotion.

## Precisely what remains non-equivalent

- The host rectangles conservatively enclose curved/diagonal/polygonal obstacles.
  They are not the reference's compensated octagonal/general-convex shapes.
  A negative room result is therefore **not** proof that no physical route exists.
- Target regions are points/axis-aligned trace intervals; diagonal traces use
  endpoint seeds. These are not the full item-shape and conduction-area doors.
  Host-local terminal IDs preserve distinct source items with the same electrical
  owner, but are not upstream Java item IDs for full-stream comparison.
- The frontier omits obstacle-room traversal, shove, rip-up approach state,
  thin-room/acute-corner handling, neckdown and all drill/layer transitions.
- UI direction/bend-unit mapping and legacy batch route-score compatibility are
  explicit adaptations. The reference `AutorouteControl` settings/defaults,
  compensation and radius-scaled via costs still require a coordinated port.
  The post-location choice between eligible layers also retains a weighted
  rectilinear length proxy; it is not an end-to-end reference cost comparison.
- The locator implements the **rectangular subset** only, not all 45-degree
  room shapes, contact splitting, forced insertion, pull-tight or via optimization.
- Trees are rebuilt, not incrementally updated/reused with reference invalidation.
  Batch/fanout ordering, connected-item normalization and optimizer parity remain
  open. Faster geometry primitives cannot compensate for those missing algorithms.
- Per-door section allocation is bounded by the remaining caller budget. It
  refuses over-budget work rather than silently coarsening doors. Exhaustion is
  not success. Java's unbounded allocation is not reproduced.

## Differential validation

Pinned source executable revision:
`a11c0a42d1b3827e5126429c5c9820c4ab5bec7c`.
Quality baseline: official Freerouting v2.3.0,
`2d4de019aa89e9fa3dc1dc44e09bf509760cafc1`.

`RoomSearchOracle.java` invokes the **actual reference methods**, including private
methods through reflection; it does not recreate their expected outputs in Java.
The committed fixture covers 256 cases each of:

- mutable tree insertion/removal/reinsertion and traversal;
- completion, ignore-object/overlap handling, touching-neighbour order and gaps;
- door sections, including narrow and overlap doors;
- 90/45-degree corner construction;
- weighted distance;
- frontier comparison, including true equality/deduplication.

The pre-existing 512 single-obstacle restraint cases also remain enabled.
A real discrepancy exposed by this oracle was fixed: `IntBox.overlaps` uses
strict separating-edge comparisons. Replacing it with “intersection has area”
incorrectly ignores zero-width restraints lying **inside** a room.

Reproduce without writing to the reference checkout:

```sh
python3 scripts/autorouter/run_room_oracle.py \
  --reference-jar /path/to/source-pin-executable.jar \
  --output-dir build/autorouter/new-room-oracle-run
```

The wrapper requires the exact source revision, creates a fresh output directory,
checks the fixture byte-for-byte, and records JAR/source/output hashes and commands.
No fixture is silently regenerated or overwritten.

Native tests additionally exercise active production dispatch, a sub-grid
corridor, mutable-copper insertion/removal, multiple sources, trace-interior
attachment, deterministic repeats, cancellation, budgets, illegal door shapes,
angle/corner construction and 300 independently certified rectangular layouts.
The grid in that last test is only an independent feasibility certificate; it
is never called by the room search under test.

## Real-board A/B protocol

Default constraints and the additional **no-vias** constraint are separate cases.
All three saved online boards are stripped identically, with both optimizers off,
four passes, eight native retries, 250,000 native expanded nodes and a 180-second
process timeout. No gates or budgets were relaxed to make the new slice pass.

`run_parity_case.py --no-vias` disables vias **and fanout in both engines**, records
that constraint, and rejects nonzero via counts on stripped inputs. The comparator
rejects mismatched constraints. Reference `--router.allowed_via_types=false` was
verified against the pinned release's serialized settings field, not inferred
from a similarly named option. Both outputs receive independent KiCad refill/DRC.

Use `KICAD_AUTOROUTER_DEBUG=1` for per-attempt `ROOM_SEARCH` room/door/section counts,
expanded work and elapsed time. `accepted=1` means the *individual path* passed
snapshot legality, not that the complete board passed the host acceptance gate.

The first midpoint-locator experiment is retained separately: it routed the
regulator safely but was too long, and the no-via 555 timed out in legacy fallback.
That experiment prompted the active corridor locator and queue corrections above.
Passing primitive oracles is not used to conceal these board-level failures.

## Final measured results

Three repeats per board **in each mode**, eighteen A/B pairs total. Values are
**native / Freerouting v2.3.0**. Core times are medians in milliseconds; completed
outputs had identical counts and lengths across the three repeats.

### Default, vias permitted — legacy search regression gate

| Board | Actual missing | New KiCad DRC | Vias | Length, mm | Core time, ms | Strict gate |
|---|---:|---:|---:|---:|---:|---|
| 5 V regulator | 0 / 0 | 0 / 0 | 0 / 0 | 323.371 / 319.203 | 1,025 / 400 | Pass, 3/3 |
| 555 astable | 0 / 0 | 0 / 10 | 10 / 7 | 96.297 / 184.248 | 284 / 1,910 | **Fail, 3/3: +3 vias** |
| BJT astable | 0 / 0 | 0 / 0 | 0 / 0 | 147.897 / 152.435 | 19 / 270 | Pass, 3/3 |

These routes match the previous milestone's physical results. This is a
regression check of the hybrid dispatcher, **not** evidence that the new
room/drill engine is routing ordinary multilayer jobs. The reference 555 reports
nine minimum-width violations and one dangling via; native has zero total DRC.
Reference violations do not authorize native violations.

### No vias or fanout — active room search with legacy fallback

| Board | Actual missing | New KiCad DRC | Vias | Length, mm | Core time, ms | Strict gate |
|---|---:|---:|---:|---:|---:|---|
| 5 V regulator | 0 / 0 | 0 / 0 | 0 / 0 | 357.391 / 319.203 | 394 / 410 | **Fail, 3/3: +12.0% length** |
| 555 astable | unknown / 0 | unknown / 1 | unknown / 0 | unknown / 193.479 | timeout / 1,120 | **Fail, 3/3: native 180 s timeout** |
| BJT astable | 0 / 0 | 0 / 0 | 0 / 0 | 147.633 / 152.435 | 15 / 280 | Pass, 3/3 |

No-via debug logs show actual accepted room-search paths, but also fallback
attempts. They are not pure room-only board benchmarks. No-via runs enabled
diagnostics; default runs did not. Native core time includes initial/repair
routing but excludes copying, refill and DRC. Reference times come from logged
engine stages, rounded to hundredths of a second; process times include JVM
startup and are retained separately. These small runs do not establish memory,
scaling or optimizer parity. Neither the zero-extra-vias nor +10% length gate
was changed.

The 555 **can** route without vias in the reference. Its one host report is a
thermal-spoke violation, not an unrouted connection. Our initial no-via routing
finishes in approximately 0.4 s, but refill exposes **three** disconnections.
One repair path succeeds immediately; a subsequent exact-island request spends
repeated 250,000-node searches in the legacy fallback until process timeout.
There is no saved, host-validated native output for these three failures. An
initial worker task count must not be substituted for final physical completion.

### Repair ownership gap identified, not yet resolved

`KicadRoutingSession` takes a fresh adapter snapshot after applying and refilling
the first proposal. Repair deliberately sets `allowRipupExisting=false` to
protect user copper. However, that snapshot then treats **the current job's own
earlier tracks/vias** as fixed existing obstacles too. The batch occupancy no
longer owns those connections, so `allowRipupRouted` cannot move them. This is a
real lifecycle limitation compared with a continuously mutable routing board.
Its causal contribution to the particular failing 555 target still needs a
controlled reroute experiment; the timeout alone does not prove it.

The proper follow-up is job-owned copper identity across refill/re-snapshot,
including connected-set invalidation and transactional removal/reconstruction.
Use the session's original UUID set to distinguish original copper; test mixed
original/generated traces, vias, rejected repairs and rollback. **Do not** fix
this by globally enabling existing-copper removal, erasing plane islands, or
relaxing DRC. General-convex target geometry and reference rip-up/shove are still
needed as well; additive repair is not a substitute.

## Verification

- PCB editor module, `qa_pcbnew` and `qa_autorouter_parity` rebuilt successfully.
- Native autorouter: **60 cases / 46,117 assertions passed**.
- Combined native/DRC/zone suites: **294 cases / 48,290 assertions passed**.
- Pinned Java oracle regenerated into a fresh native output directory and
  matched all **7,166 fixture lines byte-for-byte**. Fixture SHA-256:
  `9443580e3d4917cdefb48254c1888d3f7d472f70b1254ffcee738e4d04f75e93`.
- Data-only room/tree harness: **600 ASan+UBSan cycles passed**, alternating
  90/45-degree routes, cancellation and tree insertion/removal. This is not a
  full-editor sanitizer run; leak detection was disabled on macOS.
- Python comparator/oracle/provenance tests: **20 passed**. A mocked native
  timeout verifies the completed reference retains its measurements/companions
  and the harness does not fabricate a native result.
- **33 saved routed outputs** independently reloaded/refilled/checked; physical
  metrics matched, no new geometry was generated and file hashes stayed intact.
  This includes the three reference-only no-via 555 outputs, not nonexistent
  native timeout outputs.
- Full PCB suite: **2,384/2,385 cases passed (19 with warnings)**;
  **234,526/234,528 assertions passed**. The previously recorded
  `MatchProperties/KeysAreCanonicalAndLabelsAreFriendly` case still fails two
  label assertions in the full run. Native plus MatchProperties passes
  **86 cases / 46,290 assertions**. The full suite is **not green**; no test
  or assertion was removed or relaxed to conceal the failure.
- No GUI acceptance/undo exercise was performed in this milestone. The rebuilt
  module and shared host-session QA path do not establish that UI gate.

The A/B harness was subsequently fixed to persist each completed engine's
annotated metrics immediately and record failed/timed-out stages. Previously a
native timeout left raw reference JSON without its core-time annotation. The
table's reference-only 555 median is therefore explicitly derived from the
three retained logs (1,120 / 1,440 / 1,080 ms), not inserted into the historical
run files. A successful additional BJT no-via pair verifies the updated harness;
it is kept separate from the three-repeat measurements.

## Durable local evidence and reproduction

New evidence, separate from every earlier baseline:
[`room-search-2026-09-08/`](/Users/jmcasler/Documents/GitHub/mchamster/kicad-source-mirror-autorouter/autorouter-test-assets/online-simple/room-search-2026-09-08/README.md).
It retains the eighteen final A/B pairs (including failed runs), all available
normalized/routed PCBs, DSN/SES files, reload checks, oracle output, sanitizer
harness, build/test logs, exact A/B driver, source snapshot and SHA-256 manifest.
The intermediate midpoint-locator experiments are labelled separately and are
not measurements of the final code. The board assets remain local/Git-ignored.

Run from the native repository, with a fresh output directory:

```sh
python3 scripts/autorouter/run_parity_case.py \
  --board 'autorouter-test-assets/online-simple/555-astable/original/555 Astable multivibrator.kicad_pcb' \
  --reference-jar build/autorouter/online-simple-stable/reference/freerouting-2.3.0.jar \
  --reference-commit 2d4de019aa89e9fa3dc1dc44e09bf509760cafc1 \
  --output-dir build/autorouter/next-room-555-comparison \
  --strip-tracks --save-boards --no-vias --max-passes 4 --max-iterations 8 \
  --max-expanded-nodes 250000 --timeout-seconds 180
```

Expected current no-via 555 outcome is timeout/failure, not a passing baseline.
Remove `--no-vias` for the separate default test (+3 native vias, also a failed
strict quality gate). Keep both cases until the corresponding gaps are closed.
