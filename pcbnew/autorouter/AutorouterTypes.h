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
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>


namespace KICAD_AUTOROUTER
{

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
};


/**
 * User-visible routing controls.
 *
 * Values intentionally mirror the names and relative meaning used by
 * Freerouting's AutorouteControl and routing settings.  KiCad's board rules
 * remain authoritative for widths, clearances, via dimensions, and keepouts.
 */
enum class FANOUT_PIN_ORDER { OUTER_FIRST, INNER_FIRST, CLOSEST_ON_NET, DENSEST_FIRST, PIN_INDEX };

struct AUTOROUTER_SETTINGS
{
    std::vector<ROUTER_LAYER_SETTINGS> layers;

    int  gridStepIU = 500000;          // 0.5 mm default search pitch
    int  viaCost = 500;
    // Freerouting uses a separate, cheaper via cost when the destination is
    // a copper plane.  Keep this independent so plane nets prefer a short
    // escape to their plane over a long surface route.
    int  planeViaCost = 50;
    int  traceLengthCost = 1;
    int  congestionCost = 100;
    int  bendCost = 10;
    // Base cost for the first rip-up/reroute pass.  The batch loop scales it
    // by the pass number, matching BatchAutorouter.startRipupCosts.
    int  startRipupCost = 100;
    int  maxIterations = 8;
    int  maxPasses = 4;
    int  optimizationPasses = 2;
    int  maxOptimizationItems = 0;     // 0 = no item limit
    int  maxRipups = 128;
    int  maxExpandedNodes = 250000;
    int  maxFanoutPasses = 20;
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
};


/** One member of a search's electrical start/destination set. */
struct ROUTING_TERMINAL
{
    ROUTING_PAD pad;
    std::size_t padIndex = std::numeric_limits<std::size_t>::max();
    // A connected trace is a target region, not just its endpoints. Its start
    // is pad.position; padIndex identifies a real pad in that copper component.
    std::optional<ROUTER_POINT> segmentEnd;
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
    std::vector<ROUTING_CLEARANCE_RULE> clearanceRules;
    // Manufacturing clearances are distinct from copper-to-copper netclass
    // clearances.  They are captured on the editor thread so the worker can
    // reject via/drill collisions without consulting a live DRC engine.
    std::int64_t holeClearance = 0;
    std::int64_t holeToHoleClearance = 0;
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
};


struct ROUTING_SEGMENT
{
    int                    netCode = 0;
    int                    layer = -1;
    ROUTER_POINT           start;
    ROUTER_POINT           end;
    std::int64_t           width = 0;
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
