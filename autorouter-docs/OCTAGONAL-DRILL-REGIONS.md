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

This geometry is active in both multilayer room frontiers. The frontier now
selects and validates the first legal ordered `ViaRule` entry when each drill
transition is queued, carries that complete manufactured span/type through
backtracking, and uses source-equivalent per-layer maximum via radii when
building drill obstacles. Layers outside a blind/buried/microvia padstack no
longer make that transition behave like a through via. Final manufactured
padstack legality remains independently checked before insertion; exact
free-region search does not bypass KiCad DRC.

## Verification

- `qa_pcbnew` and `qa_autorouter_parity` build successfully.
- Native autorouter suite: **180 cases / 703,297 assertions passed**.
- The expanded pinned-Java `IntOctagon` oracle now compares both octagon and
  specialised box cutout dispatch, all eight directional `borderPoint`
  operations, and stable distance-sorted `nearestBorderProjections` for 2,048
  deterministic shape pairs, including every ordered empty/degenerate cutout
  piece. The direct test contains **164,658 assertions**. Its 2,048-record
  fixture SHA-256 is
  `57deb7d5b6eb237fa5aa54699468ccaeef8cd94fd0a5dd77214c48e30e926773`.
- New direct coverage proves that a candidate inside a diamond's AABB but
  outside the diamond remains available, a pin in that region is selected,
  the obstacle interior is removed, and nearest-point projection reaches the
  diagonal support rather than the AABB.
- The exact multilayer diamond fixture still routes with one colocated layer
  transition and never enters the compensated obstacle interior. Equality on
  a compensated support line remains legal, as in the source; production
  obstacle compensation retains its separate one-IU host-safety margin.

With exact drill regions available, the same octagonal room/drill frontier is
also active for fanout.  The 555 production smoke is complete and DRC-clean
without host repair: 22/22 worker connections, seven vias, 125.205 mm, 17,740
expanded nodes, and 2,407 ms worker time. Evidence is under
`build/autorouter/online-simple-stable/native-exact-fanout-20260910-112916/`.
Freerouting v2.3.0 also uses seven vias on this fixture. Track length and
decision-stream parity remain separate gates and are not implied by this via
count match.

The post-padstack checkpoint repeats the same smoke without host repair at
22/22 connections, seven vias, 125.205 mm and 17,740 expanded nodes. Evidence
is under
`build/autorouter/online-simple-stable/native-via-transition-style-20260910-115420/`.

## Remaining dependency

This closes exact 45-degree drill regions and activates them for both ordinary
routing and fanout. Transition-time ViaRule selection and layer-span obstacle
filtering are active, but the snapshot still represents each padstack with one
uniform diameter rather than its complete layer-local shapes and clearance
class. Arbitrary-angle `Simplex` shape intersection/cutout, rational convex
region centres, and unrestricted-angle drill-room search are still missing.
Fanout still uses a planning-only synthetic landing adapter; the search itself
now consumes the source-shaped dynamic item sets and exact first-drill queue,
but the provisional landing chooser must still be removed.
