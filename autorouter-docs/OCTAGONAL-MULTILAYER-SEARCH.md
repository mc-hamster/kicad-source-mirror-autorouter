# Exact-octagonal ordinary multilayer search — 2026-09-10

**Status: active, safe checkpoint; not full Freerouting parity.**

Source behavior is pinned to Freerouting commit
`a11c0a42d1b3827e5126429c5c9820c4ab5bec7c`. Quality comparisons remain pinned
to Freerouting v2.3.0, `2d4de019aa89e9fa3dc1dc44e09bf509760cafc1`.
The protected checkout was not modified or built.

## Production path now active

`MazeSearchEngine45DegreeMultilayer.cpp` extends the source-derived 45-degree
room lifecycle across the physical copper stack:

- each active layer uses exact compensated `IntOctagon` fixed and paid-rip-up
  obstacle envelopes;
- start and target traces attach to reached octagonal rooms with the exact
  integral support-line clip;
- one best-first queue carries room-door sections, drill pages, drill entry,
  layer exit and target states;
- room IDs are shared across the layer contexts, while layer IDs retain KiCad's
  physical stack order;
- backtracking preserves exact octagonal room and door corridors and emits
  colocated layer-transition nodes at the selected drill;
- every drill candidate is checked by the host snapshot on every layer in its
  manufactured padstack before the route can be accepted;
- pending queue entries are bounded separately from the append-only parent
  chain, preventing a dense door from exhausting the state store before its
  best sections can be expanded;
- the adapter removes only redundant collinear corridor nodes before insertion;
  this cannot remove an existing branch contact because the proposal has not
  entered the mutable board yet.

The data-only DRC rectangle predicate now uses the same conservative octagonal
offset as maze preflight. The previous square AABB offset rejected valid corner
routes even though its corners lie outside KiCad's round clearance. The
octagon circumscribes that round offset, so the change removes a false positive
without reducing the required copper clearance.

## Deliberate transitional boundaries

- Freerouting's `DrillPage` is rectangular, and native page enumeration remains
  an `IntBox` grid. Exact 45-degree obstacle cutouts now retain `IntOctagon`
  regions through drill selection; see
  [the drill-region checkpoint](OCTAGONAL-DRILL-REGIONS.md). Arbitrary-angle
  `Simplex` cutouts remain open.
- Ordinary multilayer routing selects the exact-octagonal frontier. Fanout stays
  on the qualified rectangular frontier because enabling the new queue for
  first-drill termination regressed the 555 smoke from eight to eleven vias and
  required host repair. That failed experiment is not shipped as parity.
- The rectangular single-layer fallback and broad grid/visibility fallback
  remain available for unsupported or rejected paths. General-convex layers do
  not collapse to bounding rectangles.
- Full locator room shrinking, thin/acute-room correction, pin exits and source
  first-drill ordering are still open.

## Verification

The native autorouter suite passes **178 cases / 661,115 assertions**. Focused
coverage includes an exact diamond obstacle, one required physical layer
transition, exact room/door/drill metrics, colocated via nodes, a disabled-layer
obstacle, dense-door queue pressure, off-grid rectangle-corner routing and
independent final DRC.

Production 555 smoke (`--strip-tracks --max-passes 4 --max-iterations 8
--max-expanded-nodes 250000`):

| Metric | Native checkpoint |
|---|---:|
| Routed connections | 21 / 21 |
| Host unconnected | 0 |
| New KiCad DRC violations | 0 |
| Host repair passes | 0 |
| Vias | 8 |
| Track length | 117.876 mm |
| Expanded nodes | 10,788 |
| Routing passes / rip-ups | 4 / 2 |

The saved local evidence is under
`build/autorouter/online-simple-stable/native-room45multi-20260910-110016/`.
Freerouting v2.3.0 still wins the via-quality gate with seven vias, so this is
not a parity pass.

## Next dependency-ordered work

1. Port general-convex free-drill regions and exact drill-to-room expansion.
2. Match source first-drill fanout queue seeding, ownership, ordering and
   termination before switching production fanout to the octagonal frontier.
3. Port the complete 45-degree locator shrink/acute/thin-room and pin-exit
   control flow against normalized source decision streams.
4. Remove each rectangular/visibility fallback only after completion, full
   host DRC, quality, runtime and bounded-memory corpus gates pass.
