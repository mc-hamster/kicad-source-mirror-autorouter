# Exact 45-degree drill free regions — 2026-09-10

**Status: active geometry checkpoint; not full Freerouting parity.**

Routing-source behavior is pinned to Freerouting commit
`a11c0a42d1b3827e5126429c5c9820c4ab5bec7c`. The protected checkout was not
modified or built.

## Implemented source slice

The native `DrillPage` path no longer reduces an exact 45-degree obstacle to
its axis-aligned bounding box. It now preserves the compensated `IntOctagon`
from the shape tree through:

- immediate-previous obstacle containment suppression;
- intersection with the rectangular `DrillPage`;
- sequential `PolylineArea` hole subtraction;
- the source's specialised `IntBox`-minus-`IntOctagon` decomposition;
- the source's ordered eight-piece `IntOctagon`-minus-`IntOctagon`
  decomposition and circumference-reducing divider switches;
- strict SMD pin-centre containment;
- arithmetic corner-centre selection; and
- nearest-point approach costing in `MazeExpansionEngine`.

`ExpansionDrill::freeShape` is consequently an exact octagonal convex region,
not a `ROUTER_BOX`. Empty and lower-dimensional cutout pieces are discarded by
`PolylineArea`, matching the source layer at which that filtering occurs.
Box-shaped intermediate pieces retain source double-dispatch semantics for
later holes.

This geometry is active in both multilayer room frontiers. Final manufactured
padstack legality remains independently checked on every physical layer by the
snapshot callback; exact free-region search does not bypass KiCad DRC.

## Verification

- `qa_pcbnew` and `qa_autorouter_parity` build successfully.
- Native autorouter suite: **179 cases / 672,692 assertions passed**.
- The expanded pinned-Java `IntOctagon` oracle now compares both octagon and
  specialised box cutout dispatch for 2,048 deterministic shape pairs,
  including every ordered empty/degenerate piece. Its 2,048-record fixture
  SHA-256 is
  `1e4128207a6cefe178b4fceca4c97e1f8fc56262925d7ee604fa05f40e3201ac`.
- New direct coverage proves that a candidate inside a diamond's AABB but
  outside the diamond remains available, a pin in that region is selected,
  the obstacle interior is removed, and nearest-point projection reaches the
  diagonal support rather than the AABB.
- The exact multilayer diamond fixture still routes with one colocated layer
  transition and never enters the compensated obstacle interior. Equality on
  a compensated support line remains legal, as in the source; production
  obstacle compensation retains its separate one-IU host-safety margin.

The 555 production smoke remains complete and DRC-clean but changed from eight
to nine vias and required one host repair pass: 21/21 connections, 124.171 mm,
12,396 expanded nodes, and 2,203 ms worker time. Evidence is under
`build/autorouter/online-simple-stable/native-octdrill-20260910-111632/`.
Freerouting v2.3.0 uses seven vias, so the quality gate remains failed and this
regression is not hidden by the safety pass.

## Remaining dependency

This closes exact 45-degree drill regions only. Arbitrary-angle `Simplex`
shape intersection/cutout, rational convex region centres, and unrestricted-
angle drill-room search are still missing. Fanout still uses a synthetic
landing adapter and the qualified rectangular first-drill frontier; source
item-set fanout and exact queue ordering must replace it before the exact
frontier can be enabled there safely.
