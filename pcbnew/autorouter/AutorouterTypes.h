/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * This program source code file is part of KiCad, a free EDA application.
 *
 * This file contains the data-only boundary between the Freerouting-derived
 * algorithm and the KiCad board adapter.  Keeping this boundary free of BOARD
 * and wxWidgets objects is intentional: a routing job can be executed on a
 * worker thread from an immutable board snapshot and can be regression tested
 * without opening pcbnew.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>


namespace KICAD_AUTOROUTER
{

/** KiCad's Specctra exporter writes `(resolution um 10)`, so one coordinate
 * in the pinned Freerouting engine is 0.1 micrometre, or 100 KiCad IU. */
inline constexpr double FREEROUTING_COORDINATE_UNIT_IU = 100.0;
inline constexpr double FREEROUTING_TRACE_WIDTH_TOLERANCE_IU =
        2.0 * FREEROUTING_COORDINATE_UNIT_IU;

/** A board coordinate used by the routing engine (KiCad IU, not floating point). */
struct ROUTER_POINT
{
    std::int64_t x = 0;
    std::int64_t y = 0;

    bool operator==( const ROUTER_POINT& aOther ) const
    {
        return x == aOther.x && y == aOther.y;
    }

    bool operator!=( const ROUTER_POINT& aOther ) const { return !( *this == aOther ); }
};


struct ROUTER_BOX
{
    std::int64_t minX = 0;
    std::int64_t minY = 0;
    std::int64_t maxX = 0;
    std::int64_t maxY = 0;

    bool Contains( const ROUTER_POINT& aPoint ) const
    {
        return aPoint.x >= minX && aPoint.x <= maxX && aPoint.y >= minY && aPoint.y <= maxY;
    }
};


enum class ROUTER_OBSTACLE_KIND
{
    RECTANGLE,
    SEGMENT,
    POLYGON
};


/** A layer preference corresponding to Freerouting's preferred direction/cost settings. */
struct ROUTER_LAYER_SETTINGS
{
    int  layerId = -1;
    bool enabled = true;
    // 0 = any, 1 = horizontal, 2 = vertical.
    int  preferredDirection = 0;
    int  directionCost = 20;
    // KiCad layer IDs are not ordered by physical stack position.  Keep the
    // copper ordinal explicitly so via spans never rely on enum arithmetic.
    int  layerOrdinal = -1;

    // Direct counterparts of RouterSettings.scoring's per-layer arrays.
    // NaN retains compatibility with older data-only callers, which set only
    // directionCost (in tenths above the global traceLengthCost).
    double preferredDirectionTraceCost = std::numeric_limits<double>::quiet_NaN();
    double undesiredDirectionTraceCost = std::numeric_limits<double>::quiet_NaN();

    std::pair<double, double> TraceCosts( double aFallbackPreferred ) const
    {
        const double preferred = std::isfinite( preferredDirectionTraceCost )
                                         ? std::max( 0.0, preferredDirectionTraceCost )
                                         : std::max( 0.0, aFallbackPreferred );
        const double undesired = std::isfinite( undesiredDirectionTraceCost )
                                         ? std::max( 0.0, undesiredDirectionTraceCost )
                                         : preferred + std::max( 0, directionCost ) / 10.0;

        if( preferredDirection == 0 )
            return { preferred, preferred };

        return { preferredDirection == 2 ? undesired : preferred,
                 preferredDirection == 1 ? undesired : preferred };
    }
};


/**
 * User-visible routing controls.
 *
 * Values intentionally mirror the names and relative meaning used by
 * Freerouting's AutorouteControl and routing settings.  KiCad's board rules
 * remain authoritative for widths, clearances, via dimensions, and keepouts.
 */
enum class FANOUT_PIN_ORDER { OUTER_FIRST, INNER_FIRST, CLOSEST_ON_NET, DENSEST_FIRST, PIN_INDEX };


/** Host via kind retained across the worker boundary.
 *
 * Freerouting selects an ordered ViaInfo/Padstack entry while routing.  KiCad
 * additionally needs the manufactured via kind when that selected padstack is
 * materialized.  AUTO is used by older data-only callers and is inferred from
 * the selected physical span at the adapter boundary.
 */
enum class ROUTER_VIA_TYPE
{
    AUTO,
    THROUGH,
    BLIND_BURIED,
    MICROVIA
};

struct AUTOROUTER_SETTINGS
{
    std::vector<ROUTER_LAYER_SETTINGS> layers;

    int  gridStepIU = 500000;          // 0.5 mm default search pitch
    int  viaCost = 50;
    // Freerouting uses a separate, cheaper via cost when the destination is
    // a copper plane.  Keep this independent so plane nets prefer a short
    // escape to their plane over a long surface route.
    int  planeViaCost = 5;
    int  traceLengthCost = 1;
    int  congestionCost = 100;
    int  bendCost = 10;
    // BoardHistory/stagnation scoring weights.  These are distinct from the
    // maze costs above and match Freerouting's DefaultSettings values.
    double unroutedNetPenalty = 5000000.0;
    double clearanceViolationPenalty = 1000000.0;
    double bendPenalty = 10.0;
    // Optional whole-connection retry width.  Zero disables it.  This is
    // distinct from a pin-local neckdown: Freerouting retries an otherwise
    // failed connection with this width before giving up the routing item.
    std::int64_t neckWidthIU = 0;
    // Base cost for the first rip-up/reroute pass.  The batch loop scales it
    // by the pass number, matching BatchAutorouter.startRipupCosts.
    int  startRipupCost = 100;
    // Legacy native setting retained for project/UI compatibility.  A
    // Freerouting routing item is searched exactly once in each batch pass;
    // difficulty is increased by the next pass, not by an inner retry loop.
    // Do not use this value to multiply maze searches.
    int  maxIterations = 8;
    // Pinned Freerouting v2.3.0 caps. Both stages have independent
    // convergence guards and normally finish far below these safety limits.
    int  maxPasses = 9999;
    int  optimizationPasses = 100;
    int  maxOptimizationItems = 0;     // 0 = no item limit
    // Freerouting's optimizer removes a complete connection and gives the
    // batch router a small, independent retry budget before comparing the
    // candidate lexicographically (incompletes, vias, then trace length).
    int  maxOptimizationAutoroutePasses = 6;
    int  maxOptimizationConsecutiveFailures = 50;
    // Freerouting stops before a pass when the remaining theoretical score
    // gain, or after a pass when its actual relative gain, is below 1%.
    // Zero disables only the percentage threshold; an unchanged pass still
    // converges normally.
    double optimizationImprovementThreshold = 0.01;
    // The first optimizer phase protects existing copper with higher rip-up
    // prices.  These values are retained independently even while the native
    // item-level rip-up subset remains fail-closed for unsupported contacts.
    int    optimizationAdditionalRipupCostFactorAtStart = 10;
    double optimizationTraceRipupCostFactor = 0.6;
    int  maxRipups = 128;
    int  maxExpandedNodes = 250000;
    // Pinned Freerouting v2.3.0 defaults. Normal convergence guards stop the
    // fanout stage before this cap when no additional pins are escaped.
    int  maxFanoutPasses = 20;
    int  maxFanoutItems = 0;
    std::int64_t maxFanoutMillisecondsPerPin = 10000;
    // Zero is the source default: no independent whole-fanout deadline.
    std::int64_t fanoutTimeoutMilliseconds = 0;
    bool fanoutRipupAllowed = true;
    // These are KiCad IU equivalents of Freerouting's default 2.5 mm and
    // 4.5 mm fanout escape envelope.  The geometric pad exit requirement is
    // still enforced as a stricter lower bound where necessary.
    std::int64_t fanoutMinEscapeLengthIU = 2500000;
    std::int64_t fanoutMaxEscapeLengthIU = 4500000;
    // RoutingBoard.fanout combines the net's via rule with the board rule
    // when this policy is enabled.  The worker has an explicit captured list
    // of board via dimensions so it can make the same fallback without
    // consulting a live BOARD from its background thread.
    bool fanoutFallbackToBoardVias = true;
    // Landing sampling is still a native adapter, NOT a count of routing passes.
    int  fanoutLandingSearchSteps = 20;
    FANOUT_PIN_ORDER fanoutPinOrder = FANOUT_PIN_ORDER::OUTER_FIRST;
    // Global congestion/rip-up bias.  Positive values make the search more
    // conservative around already reserved cells; negative values tolerate
    // denser intermediate packing before a rip-up is attempted.
    int  routingPriority = 0;

    bool allowVias = true;
    // Freerouting's attachSmdAllowed policy.  Keep the conservative default:
    // a fanout may not place a via directly under a single-layer SMD pad.
    // The search must first escape the pad on its source layer.
    bool allowViaInSmdPad = false;
    bool allowRipupExisting = false;
    bool allowRipupRouted = true;
    bool enableFanout = true;
    bool stopAfterFirstComplete = false;
    bool optimizeAfterComplete = true;
    bool routeOnlyUnconnected = true;

    std::vector<std::string> includeNetClasses;
    std::vector<std::string> excludeNetClasses;
    std::vector<std::string> includeNets;
    std::vector<std::string> excludeNets;
    // Freerouting enables negotiated rip-up on every ordinary routing pass,
    // including pass one.  Kept as an explicit switch for focused strict
    // search tests and host-repair policy overrides.
    bool allowRipupOnFirstIteration = true;
};


struct ROUTING_PAD
{
    int                    netCode = 0;
    ROUTER_POINT           position;
    std::vector<int>       layers;
    std::string            netClass;
    int                    netClassPriority = 0;
    std::int64_t           radius = 0;
    std::int64_t           clearance = 0;
    std::int64_t           trackWidth = 0;
    bool                    isPlaneTarget = false;
    // Set by the KiCad adapter for a single-layer SMD pad.  Keeping this
    // explicit avoids inferring package technology in data-only tests.
    bool                    isSmd = false;
    // Synthetic target created by BatchFanout for a short pad-to-via escape.
    bool                    isFanoutTarget = false;
    // Plane fanout uses one synthetic point for two routing stages.  The
    // first stage must terminate on the source layer, while the following
    // plane connection must start on the via's destination layer.  These
    // fields keep that connection-local intent in the data-only snapshot.
    int                     fanoutSourceLayer = -1;
    int                     fanoutTargetLayer = -1;
    // Real-pad index for a synthetic fanout landing.  This lets the batch
    // loop fall back to the original pad if the pre-pass cannot legally place
    // the synthetic escape in the current congestion state.
    std::size_t              fanoutSourcePadIndex = std::numeric_limits<std::size_t>::max();
    // Physical host identity; synthetic routing terminals deliberately have none.
    std::string              sourceId;
    bool                     isExactTarget = false;
    // Component ordinal and package-pad index, not net/ratsnest order.
    // -1 identifies data-only input without host package metadata.
    int                      componentId = -1;
    int                      pinIndex = -1;
    // The source router asks a Padstack for its geometry on the active layer
    // when deciding whether a trace must neck down at a pin.  A single
    // maximum radius is insufficient for asymmetric or multi-layer pads, so
    // keep that layer-local information in the worker snapshot.
    struct LAYER_GEOMETRY
    {
        /** Source Pin.TraceExitRestriction translated without an angle.
         *
         * Freerouting Direction is an equivalence class of integral vectors.
         * Preserve that representation so arbitrary component rotations do
         * not acquire floating-point equality rules at the worker boundary.
         */
        struct TRACE_EXIT_RESTRICTION
        {
            ROUTER_POINT direction;
            double       minLength = 0;

            bool operator==( const TRACE_EXIT_RESTRICTION& aOther ) const
            {
                return direction == aOther.direction && minLength == aOther.minLength;
            }
        };

        int          layer = -1;
        std::int64_t minWidth = 0;
        std::int64_t maxWidth = 0;
        std::int64_t clearance = 0;
        // Bounding box of the source ShapeSearchTree shape: physical pad
        // copper enlarged by ClearanceMatrix.clearanceCompensationValue().
        // An inverted box means that a data-only caller supplied only the
        // legacy width metadata.
        ROUTER_BOX   treeBounds{ 1, 1, 0, 0 };
        // Indices of the exact ROUTING_OBSTACLE contours which form this
        // physical layer of the padstack.  One custom pad can have several
        // disjoint outlines; a bounding box alone cannot represent it.
        std::vector<std::size_t> copperShapeIndices;
        std::vector<TRACE_EXIT_RESTRICTION> traceExitRestrictions;

        bool operator==( const LAYER_GEOMETRY& aOther ) const
        {
            return layer == aOther.layer && minWidth == aOther.minWidth
                   && maxWidth == aOther.maxWidth && clearance == aOther.clearance
                   && treeBounds.minX == aOther.treeBounds.minX
                   && treeBounds.minY == aOther.treeBounds.minY
                   && treeBounds.maxX == aOther.treeBounds.maxX
                   && treeBounds.maxY == aOther.treeBounds.maxY
                   && copperShapeIndices == aOther.copperShapeIndices
                   && traceExitRestrictions == aOther.traceExitRestrictions;
        }
    };

    std::vector<LAYER_GEOMETRY> layerGeometry;
    // Synthetic fanout-only metadata.  The real board's net rule remains
    // untouched for the ordinary batch stage; a selected board-via fallback
    // is attached to the synthetic landing and applied only to the fanout
    // connection that terminates there.
    std::int64_t fanoutViaDiameter = 0;
    std::int64_t fanoutViaDrill = 0;
    std::int64_t fanoutMinEscapeLength = 0;
    std::int64_t fanoutMaxEscapeLength = 0;
    // The synthetic landing is only electrically meaningful when its source
    // SMD pad can actually reach it on the source layer.  BatchFanout keeps a
    // bounded, preflighted local escape here (including the real pad centre
    // and the landing point) when a straight stub is blocked.  The maze
    // consumes this exact path before it considers a board-wide fallback, so
    // an arbitrary free via point never becomes a dangling synthetic target.
    // It is deliberately connection-local metadata, not a replacement for
    // Freerouting's item-set fanout search.
    std::vector<ROUTER_POINT> fanoutEscapePath;
    // Complete selected padstack metadata.  Diameter/drill are retained above
    // for compatibility with existing data-only callers; these fields prevent
    // the fanout pre-pass from silently widening a blind/buried or microvia to
    // the whole board stack before maze insertion and KiCad materialization.
    std::vector<int> fanoutViaLayers;
    bool             fanoutViaAttachSmdAllowed = false;
    ROUTER_VIA_TYPE  fanoutViaType = ROUTER_VIA_TYPE::AUTO;
};


/** One member of a search's electrical start/destination set. */
struct ROUTING_TERMINAL
{
    ROUTING_PAD pad;
    std::size_t padIndex = std::numeric_limits<std::size_t>::max();
    // A connected trace is a target region, not just its endpoints. Its start
    // is pad.position; padIndex identifies a real pad in that copper component.
    std::optional<ROUTER_POINT> segmentEnd;
    // Exact ConductionArea connection region. Pins and vias deliberately
    // remain point targets, matching DrillItem.getTraceConnectionShape().
    // The detached snapshot owns this immutable value through shared storage.
    std::shared_ptr<const struct ROUTING_OBSTACLE> connectionArea;
};


/** Freerouting board/model/structure/FixedState.
 *
 * Declaration order is semantic: USER_FIXED and SYSTEM_FIXED are not
 * routable/deletable, while SHOVE_FIXED may still be split, joined and
 * removed but is excluded from pull-tight/shove movement.
 */
enum class ROUTER_FIXED_STATE
{
    UNFIXED,
    SHOVE_FIXED,
    USER_FIXED,
    SYSTEM_FIXED
};


/** Item-context clearance resolved by KiCad before the worker starts.
 *
 * Freerouting stores a clearance class on each board item and resolves the
 * class pair by layer.  KiCad rules can additionally depend on the actual
 * item, footprint, net, or another host-only property.  The adapter resolves
 * that complete pair while the live BOARD_ITEM is available and carries the
 * resulting matrix entry with each detached obstacle.
 */
struct ROUTING_CONTEXTUAL_CLEARANCE
{
    int          candidateNetCode = 0;
    int          layer = -1;
    std::int64_t clearance = 0;

    bool operator==( const ROUTING_CONTEXTUAL_CLEARANCE& aOther ) const
    {
        return candidateNetCode == aOther.candidateNetCode && layer == aOther.layer
               && clearance == aOther.clearance;
    }
};


struct ROUTING_OBSTACLE
{
    ROUTER_OBSTACLE_KIND   kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    int                    netCode = 0;
    std::vector<int>       layers;
    ROUTER_POINT           start;
    ROUTER_POINT           end;
    ROUTER_BOX             box;
    std::vector<ROUTER_POINT> polygon;
    std::int64_t           radius = 0;
    bool                    blocksTracks = true;
    bool                    blocksVias = true;
    bool                    isExistingRoute = false;
    std::string            boardItemId;
    // Copper radius and rule clearance are separate.  Pair checks combine
    // this value with the candidate net's clearance once, rather than
    // inflating both geometries with the same netclass value.
    std::int64_t            clearance = 0;
    // Polygonal copper/keepout holes are legal space and must not be treated
    // as part of the enclosing obstacle.  Kept at the end to preserve the
    // convenient aggregate-initializer layout used by data-only tests.
    std::vector<std::vector<ROUTER_POINT>> polygonHoles;
    // Rule areas are obstacles even when KiCad associates them with a net.
    // Copper belonging to the active net remains traversable, so the worker
    // must distinguish the two cases rather than inferring it from netCode.
    bool                    isKeepout = false;
    // A drilled pad/via hole is a separate manufacturing obstacle.  It does
    // not replace the copper obstacle for the same item: tracks use the
    // copper-to-hole rule while vias also have to satisfy hole-to-hole.
    bool                    isHole = false;
    // True only for a straight, unlocked existing BOARD_ITEM that the adapter
    // captured with enough geometry to use as a *candidate* for the bounded
    // forced-shove path.  This is deliberately not an assertion that every
    // contact of the item is movable: the batch layer reconstructs a
    // fail-closed trace/via neighbourhood before it permits a mutation.
    bool                    isMovable = false;
    // A whole-net reroute may retain unsupported source copper solely as a
    // collision surface.  Such a copy must block maze/fanout/DRC geometry,
    // but it must never become a ROUTING_BOARD item or satisfy an electrical
    // ratsnest connection before the host item is explicitly regenerated.
    bool                    isCollisionOnly = false;
    // True on the canonical removable-source record when a collision-only
    // copy was appended to BOARD_SNAPSHOT::obstacles.  It prevents the
    // internal proposal DRC from counting the same physical BOARD_ITEM twice;
    // the record remains available for exact UUID removal on acceptance.
    bool                    isMirroredToObstacleModel = false;
    // The item already exists on the private KiCad proposal board, but was
    // created by an earlier autorouter stage rather than by the user.  This
    // distinction is essential during post-refill repair: user copper remains
    // protected, while job-owned copper stays electrically represented and
    // may be ripped up/rerouted like any other generated route.  Kept at the
    // end to preserve the aggregate-initializer layout used by tests.
    bool                    isAutorouterOwned = false;
    // Source ShapeSearchTree45Degree treats a rectangular DrillItem shape
    // specially: it offsets the IntBox as another IntBox before converting
    // it to an octagon.  ObstacleArea rectangles instead use the normal
    // chamfered octagonal offset.  Preserve that source type distinction
    // after the KiCad adapter has detached the geometry from PAD.
    bool                    isPad = false;
    // Existing KiCad copper normally corresponds to a protected DSN wire.
    // The adapter sets job-owned copper to UNFIXED and locked copper to
    // SYSTEM_FIXED explicitly.  Keeping this independent from `isMovable`
    // prevents host replacement policy from changing source item topology.
    ROUTER_FIXED_STATE      fixedState = ROUTER_FIXED_STATE::USER_FIXED;
    // Present only when KiCad has explicit clearance rules.  An entry is the
    // authoritative live DRC result for this exact source BOARD_ITEM against
    // new copper of candidateNetCode on layer; it may be lower than either
    // netclass default because custom rules are priority ordered.
    std::vector<ROUTING_CONTEXTUAL_CLEARANCE> contextualClearances;
};


inline std::optional<std::int64_t> ContextualObstacleClearance(
        const ROUTING_OBSTACLE& aObstacle, int aCandidateNetCode, int aLayer )
{
    std::optional<std::int64_t> result;
    for( const ROUTING_CONTEXTUAL_CLEARANCE& rule : aObstacle.contextualClearances )
    {
        if( rule.candidateNetCode != aCandidateNetCode
            || ( rule.layer >= 0 && aLayer >= 0 && rule.layer != aLayer ) )
        {
            continue;
        }

        const std::int64_t clearance = std::max<std::int64_t>( 0, rule.clearance );
        result = result ? std::max( *result, clearance ) : clearance;
    }
    return result;
}


/** One ordered entry in a net's Freerouting-style ViaRule.
 *
 * layers declares the complete padstack span (or just its two endpoints); an
 * empty list means the complete physical copper stack.  A profile may serve a
 * smaller search transition whenever its manufactured span contains both
 * transition layers.  The emitted via still occupies the profile's complete
 * span, matching FoundConnectionInserter.insertVia().
 */
struct ROUTING_VIA_PROFILE
{
    std::int64_t          diameter = 0;
    std::int64_t          drill = 0;
    std::vector<int>      layers;
    bool                  attachSmdAllowed = false;
    ROUTER_VIA_TYPE       type = ROUTER_VIA_TYPE::AUTO;

    bool operator==( const ROUTING_VIA_PROFILE& aOther ) const
    {
        return diameter == aOther.diameter && drill == aOther.drill
               && layers == aOther.layers
               && attachSmdAllowed == aOther.attachSmdAllowed && type == aOther.type;
    }
};


struct ROUTING_NET
{
    int                    netCode = 0;
    std::string            name;
    std::string            netClass;
    int                    netClassPriority = 0;
    std::int64_t           clearance = 0;
    std::int64_t           viaDiameter = 0;
    std::int64_t           viaDrill = 0;
    std::vector<std::size_t> padIndices;
    // Synthetic landing points inside filled same-net zones.  They model
    // Freerouting's plane target without putting a KiCad zone object on the
    // worker thread or treating the zone as a second pad.
    std::vector<std::size_t> planeTargetIndices;
    // Ratsnest edges are retained rather than reducing every net to a
    // source-to-all-pads chain.  This preserves already-connected topology
    // when routeOnlyUnconnected is enabled.
    std::vector<std::pair<std::size_t, std::size_t>> connections;
    // Set by the adapter after include/exclude filters and enabled layers are
    // applied.  Board metadata for excluded nets is still retained so those
    // nets remain obstacles and participate in pair-clearance resolution.
    bool routable = false;
    // Physical pad components in retained input copper, captured from KiCad's
    // connectivity clusters (not inferred from ratsnest pairs or positions).
    // Empty when existing copper is scheduled for replacement.
    std::vector<std::vector<std::size_t>> connectedPadGroups;
    // Ordered like rules.ViaRule.  When non-empty these are the only legal
    // new-via alternatives for this net; selection never invents a legacy
    // through via after every declared profile has failed.
    std::vector<ROUTING_VIA_PROFILE> viaProfiles;
};


/** A manufacturable through-via dimension captured from the KiCad board. */
struct ROUTING_VIA_DIMENSION
{
    std::int64_t diameter = 0;
    std::int64_t drill = 0;

    bool operator==( const ROUTING_VIA_DIMENSION& aOther ) const
    {
        return diameter == aOther.diameter && drill == aOther.drill;
    }
};


/** A layer-aware pair clearance captured from KiCad's DRC rule resolver. */
struct ROUTING_CLEARANCE_RULE
{
    int          firstNetCode = 0;
    int          secondNetCode = 0;
    int          layer = -1;
    std::int64_t clearance = 0;
};


/** Immutable worker-thread input. */
struct BOARD_SNAPSHOT
{
    ROUTER_BOX             bounds;
    std::vector<ROUTER_POINT> boardOutline;
    std::vector<std::vector<ROUTER_POINT>> boardHoles;
    std::int64_t           edgeClearance = 0;
    std::vector<ROUTING_PAD> pads;
    std::vector<ROUTING_OBSTACLE> obstacles;
    // Existing copper intentionally omitted from obstacles when the user
    // enables whole-board rip-up.  Keep a copy of those records so the
    // proposal can remove exactly the original BOARD_ITEMs on acceptance.
    std::vector<ROUTING_OBSTACLE> removableExistingRoutes;
    std::vector<ROUTING_NET> nets;
    // The host's board-wide via presets.  A net normally uses its resolved
    // netclass profile; fanout may additionally use these profiles when its
    // explicit Freerouting-compatible fallback policy is enabled.
    std::vector<ROUTING_VIA_DIMENSION> boardViaDimensions;
    std::vector<ROUTING_CLEARANCE_RULE> clearanceRules;
    // Manufacturing clearances are distinct from copper-to-copper netclass
    // clearances.  They are captured on the editor thread so the worker can
    // reject via/drill collisions without consulting a live DRC engine.
    std::int64_t holeClearance = 0;
    std::int64_t holeToHoleClearance = 0;
    // The board-wide lower bound is captured explicitly because a pin's
    // geometric neckdown width is not automatically a legal KiCad track
    // width. More-specific DRC constraints remain an acceptance-time gate.
    std::int64_t minimumTrackWidth = 0;
    // KiCad's modification counter captured on the editor thread.  It is not
    // used by the worker algorithm; the tool uses it to refuse acceptance if
    // the live board changed while a proposal was being reviewed.
    int sourceBoardTimestamp = 0;
    // Separate physical filled regions. Never union different islands by net name.
    std::vector<ROUTING_OBSTACLE> conductionAreas;
};


struct ROUTER_NODE
{
    ROUTER_POINT           point;
    int                    layer = -1;

    bool operator==( const ROUTER_NODE& aOther ) const
    {
        return point == aOther.point && layer == aOther.layer;
    }

    bool operator!=( const ROUTER_NODE& aOther ) const { return !( *this == aOther ); }
};


/**
 * Manufacturing style for one edge of a routed connection.
 *
 * A zero value means "inherit the net's resolved default".  Existing tests
 * and routes which predate per-edge styles therefore retain their exact
 * behaviour, while a forced insertion can represent the short narrow part of
 * a neckdown (or a via with a non-default mask) without changing every other
 * edge of the net.
 */
struct ROUTING_EDGE_STYLE
{
    std::int64_t     trackWidth = 0;
    std::int64_t     clearance = 0;
    std::int64_t     viaDiameter = 0;
    std::int64_t     viaDrill = 0;
    std::vector<int> viaLayers;
    ROUTER_VIA_TYPE  viaType = ROUTER_VIA_TYPE::AUTO;
    // A same-layer run changes source PolylineTrace identity at a fixed-state
    // boundary even when width and clearance are unchanged.
    ROUTER_FIXED_STATE fixedState = ROUTER_FIXED_STATE::UNFIXED;

    bool operator==( const ROUTING_EDGE_STYLE& aOther ) const
    {
        return trackWidth == aOther.trackWidth && clearance == aOther.clearance
               && viaDiameter == aOther.viaDiameter && viaDrill == aOther.viaDrill
               && viaLayers == aOther.viaLayers && viaType == aOther.viaType
               && fixedState == aOther.fixedState;
    }

    bool operator!=( const ROUTING_EDGE_STYLE& aOther ) const
    {
        return !( *this == aOther );
    }
};


/** A route produced for one electrical connection. */
struct ROUTING_CONNECTION
{
    int                    netCode = 0;
    std::vector<ROUTER_NODE> nodes;
    double                 cost = 0.0;
    bool                   complete = false;
    // Snapshot pad identities allow the batch loop to grow a connected set
    // for multi-pad nets and to preserve which route terminates at a plane.
    std::size_t fromPadIndex = std::numeric_limits<std::size_t>::max();
    std::size_t toPadIndex = std::numeric_limits<std::size_t>::max();
    bool        isPlaneConnection = false;
    bool        isFanoutConnection = false;
    // Existing BOARD_ITEM copper can be present in the worker occupancy as a
    // static collision participant while a full-net reroute is planned.  It
    // must not become electrical worker copper or be emitted a second time
    // unless a checked forced shove actually replaces it.  The source UUIDs
    // remain attached to a replacement so proposal acceptance can atomically
    // delete the exact original BOARD_ITEMs.
    bool                    isExistingBoardRoute = false;
    bool                    isShoveMovable = true;
    std::vector<std::string> sourceBoardItemIds;
    // Empty means every edge inherits the resolved net defaults.  Otherwise
    // there is precisely one style for each pair of consecutive nodes.
    std::vector<ROUTING_EDGE_STYLE> edgeStyles;
    // See ROUTING_OBSTACLE::isAutorouterOwned.  This connection has a host
    // UUID because an earlier stage materialized it on the private proposal
    // board, but it is not protected source copper.  Kept last so existing
    // aggregate initializers retain their field mapping.
    bool                    isAutorouterOwned = false;
};


/** One static worker route promoted into proposal copper without changing its
 * centreline.  A source via move uses this for each contacted source trace:
 * KiCad acceptance removes the old BOARD_ITEM and re-emits the exact trace
 * together with the short bridge to the moved via. */
struct ROUTING_CONNECTION_REPLACEMENT
{
    ROUTING_CONNECTION original;
    ROUTING_CONNECTION replacement;
};


/** Atomic result of moving a drill item with its source-style trace contacts.
 *
 * Freerouting's DrillItem.moveBy() moves the via and inserts one bridge trace
 * for each unique contacted-trace style.  The native worker cannot mutate
 * host BOARD_ITEMs in place, so the unchanged contacted traces are promoted
 * from static occupancy records and emitted as proposal geometry instead.
 */
struct ROUTING_VIA_SHOVE_PLAN
{
    ROUTING_CONNECTION replacement;
    std::vector<ROUTING_CONNECTION_REPLACEMENT> materializedContacts;
    std::vector<ROUTING_CONNECTION> bridges;
};


inline bool HasValidEdgeStyles( const ROUTING_CONNECTION& aConnection )
{
    return aConnection.edgeStyles.empty()
           || aConnection.edgeStyles.size() + 1 == aConnection.nodes.size();
}


inline const ROUTING_EDGE_STYLE& EdgeStyle( const ROUTING_CONNECTION& aConnection,
                                            std::size_t aEdgeIndex )
{
    static const ROUTING_EDGE_STYLE inherited;

    if( aConnection.edgeStyles.empty() )
        return inherited;

    return aConnection.edgeStyles.at( aEdgeIndex );
}


inline void EnsureEdgeStyles( ROUTING_CONNECTION& aConnection )
{
    const std::size_t edgeCount = aConnection.nodes.empty() ? 0 : aConnection.nodes.size() - 1;

    if( aConnection.edgeStyles.empty() )
    {
        aConnection.edgeStyles.resize( edgeCount );
        return;
    }

    if( aConnection.edgeStyles.size() != edgeCount )
        aConnection.edgeStyles.resize( edgeCount );
}


inline bool SameRouteGeometry( const ROUTING_CONNECTION& aLeft,
                               const ROUTING_CONNECTION& aRight )
{
    if( aLeft.netCode != aRight.netCode || aLeft.nodes != aRight.nodes
        || aLeft.edgeStyles != aRight.edgeStyles )
    {
        return false;
    }

    // Normal worker routes historically have no physical identity, so retain
    // their geometry-only comparison.  Static source BOARD_ITEM routes do:
    // two coincident tracks/vias must not be treated as one victim merely
    // because their copper geometry is identical.
    if( aLeft.sourceBoardItemIds.empty() && aRight.sourceBoardItemIds.empty() )
        return true;

    return aLeft.sourceBoardItemIds == aRight.sourceBoardItemIds;
}


/** True when replacing the given contiguous edge run by one edge cannot lose
 * a width, clearance, or via-mask decision. */
inline bool CanCollapseRouteEdges( const ROUTING_CONNECTION& aConnection,
                                   std::size_t aFirstEdge, std::size_t aLastEdge )
{
    if( aFirstEdge > aLastEdge || aLastEdge + 1 >= aConnection.nodes.size()
        || !HasValidEdgeStyles( aConnection ) )
    {
        return false;
    }

    const ROUTING_EDGE_STYLE& first = EdgeStyle( aConnection, aFirstEdge );

    for( std::size_t edge = aFirstEdge + 1; edge <= aLastEdge; ++edge )
        if( EdgeStyle( aConnection, edge ) != first )
            return false;

    return true;
}


/** Replace nodes (firstNode, lastNode) by one direct edge. Callers must first
 * prove the replacement geometry is legal. */
inline bool CollapseRouteNodes( ROUTING_CONNECTION& aConnection, std::size_t aFirstNode,
                                std::size_t aLastNode )
{
    if( aFirstNode >= aLastNode || aLastNode >= aConnection.nodes.size()
        || !CanCollapseRouteEdges( aConnection, aFirstNode, aLastNode - 1 ) )
    {
        return false;
    }

    if( !aConnection.edgeStyles.empty() && aLastNode > aFirstNode + 1 )
    {
        aConnection.edgeStyles.erase(
                aConnection.edgeStyles.begin()
                        + static_cast<std::ptrdiff_t>( aFirstNode + 1 ),
                aConnection.edgeStyles.begin() + static_cast<std::ptrdiff_t>( aLastNode ) );
    }

    aConnection.nodes.erase( aConnection.nodes.begin()
                                     + static_cast<std::ptrdiff_t>( aFirstNode + 1 ),
                             aConnection.nodes.begin()
                                     + static_cast<std::ptrdiff_t>( aLastNode ) );
    return true;
}


inline bool RemoveRouteEndpoint( ROUTING_CONNECTION& aConnection, bool aFront )
{
    if( aConnection.nodes.empty() || !HasValidEdgeStyles( aConnection ) )
        return false;

    if( aConnection.nodes.size() == 1 )
    {
        aConnection.nodes.clear();
        aConnection.edgeStyles.clear();
        return true;
    }

    if( !aConnection.edgeStyles.empty() )
    {
        if( aFront )
            aConnection.edgeStyles.erase( aConnection.edgeStyles.begin() );
        else
            aConnection.edgeStyles.pop_back();
    }

    if( aFront )
        aConnection.nodes.erase( aConnection.nodes.begin() );
    else
        aConnection.nodes.pop_back();

    return true;
}


inline void ClearRouteGeometry( ROUTING_CONNECTION& aConnection )
{
    aConnection.nodes.clear();
    aConnection.edgeStyles.clear();
}


struct ROUTING_SEGMENT
{
    int                    netCode = 0;
    int                    layer = -1;
    ROUTER_POINT           start;
    ROUTER_POINT           end;
    std::int64_t           width = 0;
    // Per-edge clearance is routing metadata, not a KiCad track property.
    // It preserves the clearance class resolved by a forced/neckdown edge so
    // worker-side conflict checks and final proposal DRC use the same rule
    // that admitted the connection.
    std::int64_t           clearance = 0;
};


struct ROUTING_VIA
{
    int                    netCode = 0;
    ROUTER_POINT           position;
    int                    topLayer = -1;
    int                    bottomLayer = -1;
    std::int64_t           diameter = 0;
    std::int64_t           drill = 0;
    // Explicit layers touched by the via.  This is needed for inner-layer
    // spans because KiCad's layer enum values are not a physical ordering.
    std::vector<int> layers;
    // See ROUTING_SEGMENT::clearance.  This applies to the via's copper
    // annulus; hole-to-hole clearance remains a distinct board rule.
    std::int64_t           clearance = 0;
    ROUTER_VIA_TYPE        type = ROUTER_VIA_TYPE::AUTO;
};


struct ROUTER_METRICS
{
    int                    totalConnections = 0;
    int                    routedConnections = 0;
    int                    unroutedConnections = 0;
    int                    passes = 0;
    int                    retries = 0;
    int                    ripups = 0;
    int                    optimizationPasses = 0;
    int                    expandedNodes = 0;
    int                    segmentCount = 0;
    int                    bendCount = 0;
    int                    viaCount = 0;
    int                    fanoutConnections = 0;
    int                    drcViolations = 0;
    double                 routedLengthIU = 0.0;
    double                 airlineLengthIU = 0.0;
    double                 completionPercent = 0.0;
    std::int64_t           elapsedMilliseconds = 0;
};


struct ROUTER_PROGRESS
{
    int                    pass = 0;
    int                    maxPasses = 0;
    int                    routedConnections = 0;
    int                    totalConnections = 0;
    int                    ripups = 0;
    int                    retries = 0;
    int                    expandedNodes = 0;
    std::int64_t           elapsedMilliseconds = 0;
    std::string            stage;
};


struct ROUTING_RESULT
{
    std::vector<ROUTING_CONNECTION> connections;
    std::vector<ROUTING_SEGMENT> segments;
    std::vector<ROUTING_VIA> vias;
    std::vector<std::string> removedBoardItemIds;
    std::vector<int> unroutedNetCodes;
    ROUTER_METRICS metrics;
    bool complete = false;
    bool cancelled = false;
    // A fanout timeout is not a job cancellation.  The main batch stage is
    // still allowed to finish the board, and the UI can report that the
    // fallback path was used instead of implying a hung worker.
    bool fanoutTimedOut = false;
    std::string message;
    bool hostValidated = false;
    int hostUnconnected = -1;
    int hostNewDrcViolations = -1;
    int hostRepairPasses = 0;
    std::int64_t hostValidationMilliseconds = 0;

    // A partial route may be accepted deliberately, but an unchecked or
    // design-rule-violating proposal must never be committed to the editor.
    bool CanAcceptProposal() const
    {
        return hostValidated && !cancelled && hostUnconnected >= 0
                && hostNewDrcViolations == 0 && metrics.drcViolations == 0;
    }
};


using ROUTER_CANCEL_CALLBACK = std::function<bool()>;
using ROUTER_SEARCH_PROGRESS_CALLBACK = std::function<void( int )>;
using ROUTER_PROGRESS_CALLBACK = std::function<void( const ROUTER_PROGRESS& )>;

} // namespace KICAD_AUTOROUTER
