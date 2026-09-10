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
 * Freerouting equivalent: autoroute/maze/MazeSearchEngine.java and
 * autoroute/maze/MazeExpansionEngine.java.
 *
 * Single-layer and multilayer paths first try the rectangular room/drill slice.
 * Unsupported or rejected proposals still use the experimental grid/visibility engine;
 * neither the combined implementation nor its cost model is at upstream parity.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "../board/facade/RoutingBoard.h"
#include "AutorouteControl.h"
#include "LegacyDestinationDistance.h"
#include "MazeExpansionEngine.h"
#include "MazeSearchEngine90Degree.h"


namespace KICAD_AUTOROUTER
{

struct ROUTER_CELL_KEY
{
    std::int64_t x = 0;
    std::int64_t y = 0;
    int          layer = -1;

    bool operator==( const ROUTER_CELL_KEY& aOther ) const
    {
        return x == aOther.x && y == aOther.y && layer == aOther.layer;
    }
};


struct ROUTER_CELL_HASH
{
    std::size_t operator()( const ROUTER_CELL_KEY& aKey ) const noexcept;
};


/** Negotiated-congestion usage map shared by all routes in a pass. */
class ROUTING_OCCUPANCY
{
public:
    explicit ROUTING_OCCUPANCY( std::int64_t aGridStep ) :
            m_gridStep( std::max<std::int64_t>( 1, aGridStep ) )
    {
    }

    void InitializeBoard( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings );
    ROUTING_BOARD* Board() const { return m_board.get(); }

    void Add( const ROUTING_CONNECTION& aConnection );
    /**
     * Add retained host copper to the transient collision/congestion model
     * without turning it into electrical worker copper.  Whole-net reroute
     * tasks must still route every requested ratsnest edge; treating source
     * tracks as connected here would let the proposal delete copper it never
     * regenerated.  A later checked shove removes this static record and
     * inserts its replacement with Add().
     */
    void AddStatic( const ROUTING_CONNECTION& aConnection );
    void Remove( const ROUTING_CONNECTION& aConnection );
    void Clear();
    int  Usage( const ROUTER_CELL_KEY& aCell, int aNetCode ) const;
    int  SegmentUsage( const ROUTER_NODE& aStart, const ROUTER_NODE& aEnd,
                       int aNetCode ) const;
    int  ProximityUsage( const ROUTER_CELL_KEY& aCell, int aNetCode ) const;
    std::vector<ROUTING_CONNECTION> ConflictingConnections(
            const ROUTING_CONNECTION& aConnection ) const;
    const std::vector<ROUTING_CONNECTION>& Connections() const { return m_connections; }

    /** One speculative edit spans copper IDs/contacts AND route/usage maps. */
    class TRANSACTION
    {
    public:
        explicit TRANSACTION( ROUTING_OCCUPANCY& aOccupancy );
        ~TRANSACTION();
        TRANSACTION( const TRANSACTION& ) = delete;
        TRANSACTION& operator=( const TRANSACTION& ) = delete;
        void Commit();
    private:
        struct STATE;
        ROUTING_OCCUPANCY& m_occupancy;
        std::unique_ptr<STATE> m_before;
    };

    std::vector<ROUTER_CELL_KEY> CellsForSegment( const ROUTER_NODE& aStart,
                                                  const ROUTER_NODE& aEnd ) const;

private:
    std::unique_ptr<ROUTING_BOARD> m_board;
    std::int64_t m_gridStep;
    std::unordered_map<ROUTER_CELL_KEY, std::map<int, int>, ROUTER_CELL_HASH> m_usage;
    std::vector<ROUTING_CONNECTION> m_connections;
};


class MAZE_SEARCH_ENGINE
{
public:
    /** Layer-local pad information needed by FoundConnectionInserter's
     * source-derived neckdown fallback.  The worker never dereferences a
     * live PAD while routing. */
    struct PIN_ENTRY_STYLE
    {
        ROUTING_EDGE_STYLE style;
        std::int64_t       maxPadWidth = 0;
        std::int64_t       clearance = 0;
    };

    /**
     * Build a search engine for an immutable board snapshot.  Fanout can
     * select a different legal ViaRule entry for each SMD pin of one net;
     * aViaOverride is deliberately scoped to one net and one engine so that
     * the search, conflict predicate, and strict insertion all see the same
     * padstack.  Ordinary batch routing leaves both optional arguments at
     * their defaults.
     */
    MAZE_SEARCH_ENGINE( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
                        ROUTING_OCCUPANCY& aOccupancy, int aViaOverrideNetCode = 0,
                        std::optional<ROUTING_VIA_PROFILE> aViaOverride = std::nullopt,
                        int aTrackWidthOverrideNetCode = 0,
                        std::optional<std::int64_t> aTrackWidthOverride = std::nullopt );

    std::optional<ROUTING_CONNECTION> FindConnection( const ROUTING_PAD& aStart,
                                                       const ROUTING_PAD& aTarget,
                                                       int aRetry,
                                                       int& aExpandedNodes,
                                                       const ROUTER_CANCEL_CALLBACK& aCancel,
                                                       const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress = {},
                                                       const std::vector<ROUTING_TERMINAL>& aStarts = {},
                                                       const std::vector<ROUTING_TERMINAL>& aTargets = {},
                                                       bool aAllowLegacyFallback = true ) const;

    // Retry searches may temporarily cross committed routes.  Resolve those
    // crossings with the same netclass, layer-span, copper, and drill rules
    // used by the search engine before the batch layer removes a victim.
    std::vector<ROUTING_CONNECTION> FindConflictingConnections(
            const ROUTING_CONNECTION& aCandidate ) const;

    bool CanUseSegment( int aNetCode, const ROUTER_NODE& aStart, const ROUTER_NODE& aEnd,
                        bool aForVia = false,
                        const ROUTING_EDGE_STYLE* aStyle = nullptr ) const;
    /** Strict insertion check: never inherits a negotiated-search ripup flag.
     * Endpoint copper is the new trace width, not the already-existing pad size.
     */
    bool CanInsertSegment( int aNetCode, const ROUTER_NODE& aStart, const ROUTER_NODE& aEnd,
                           const ROUTING_EDGE_STYLE* aStyle = nullptr ) const;
    /** Select the first legal ViaInfo in the net's ordered ViaRule.
     *
     * The returned style contains the complete manufactured padstack span,
     * not merely the two layers requested by the maze transition.  No legacy
     * fallback is considered when the net declares at least one profile.
     */
    std::optional<ROUTING_EDGE_STYLE> SelectViaStyle(
            int aNetCode, const ROUTER_NODE& aStart, const ROUTER_NODE& aEnd,
            bool aAttachesToSmd = false ) const;
    std::int64_t ResolveTrackWidth( int aNetCode,
                                    const ROUTING_EDGE_STYLE& aStyle ) const;
    std::optional<PIN_ENTRY_STYLE> PinEntryStyle( std::size_t aPadIndex,
                                                  const ROUTER_NODE& aNode,
                                                  int aNetCode,
                                                  std::int64_t aNormalTrackWidth ) const;
    /** Fixed rectangular obstacle/orthogonal spring-over adapter. Unsupported
     * geometry returns no proposal; all returned edges need strict preflight.
     */
    std::optional<ROUTING_CONNECTION> SpringOverConnection(
            const ROUTING_CONNECTION& aConnection, const ROUTER_CANCEL_CALLBACK& aCancel ) const;
    /**
     * Spring a generated connection over transient worker copper.  This is
     * used by checked forced insertion to move a mutable generated trace
     * around the incoming path without turning that copper into a permanent
     * board-snapshot obstacle.  It intentionally fails closed for unsupported
     * geometry and does not make original host copper movable.
     */
    std::optional<ROUTING_CONNECTION> SpringOverConnection(
            const ROUTING_CONNECTION& aConnection,
            const std::vector<ROUTING_CONNECTION>& aTransientObstacles,
            const ROUTER_CANCEL_CALLBACK& aCancel ) const;
    /**
     * Relocate one mutable via away from transient worker copper.  A complete
     * isolated source via can be translated directly; generated worker
     * trace-via-trace chains also rebuild their two adjacent legs.  Static
     * source contacts are retained separately by ShoveViaConnectionPlan.
     * This is a
     * checked forced-insertion primitive, not a claim that arbitrary host
     * vias or every source DrillItemMover contact graph are movable.
     */
    std::optional<ROUTING_CONNECTION> ShoveViaConnection(
            const ROUTING_CONNECTION& aConnection,
            const std::vector<ROUTING_CONNECTION>& aTransientObstacles,
            const ROUTER_CANCEL_CALLBACK& aCancel,
            const std::vector<ROUTING_CONNECTION>& aStaticDrillObstacles = {},
            const std::function<bool( const ROUTING_CONNECTION& )>& aPlacementFilter = {} ) const;
    /** Build the atomic worker equivalent of DrillItem.moveBy().  In addition
     * to the moved drill it preserves every supported static trace contact by
     * materialising that trace and adding one old-centre-to-new-centre bridge
     * per contacted layer.  aContactCandidates is an explicit live/static
     * route view because collision-only source copper is intentionally absent
     * from ROUTING_BOARD's electrical item graph. */
    std::optional<ROUTING_VIA_SHOVE_PLAN> ShoveViaConnectionPlan(
            const ROUTING_CONNECTION& aConnection,
            const std::vector<ROUTING_CONNECTION>& aTransientObstacles,
            const std::vector<ROUTING_CONNECTION>& aContactCandidates,
            const ROUTER_CANCEL_CALLBACK& aCancel ) const;
    const ROOM_SEARCH_METRICS& LastRoomSearchMetrics() const { return m_roomMetrics; }

private:
    std::vector<SHAPE_TREE_ENTRY> roomObstacles( int aNet, int aLayer, bool aForVia,
                                               bool aSkipGeneralConvex,
                                               const ROUTER_CANCEL_CALLBACK& aCancel,
                                               const std::vector<ROOM_RIPUP_OBSTACLE>*
                                                       aRipupObstacles = nullptr ) const;
    std::vector<ROOM_RIPUP_OBSTACLE> roomRipupObstacles(
            int aNet, int aLayer, bool aForVia, int aRetry, bool aFanout,
            const ROUTER_CANCEL_CALLBACK& aCancel ) const;
    /** The rectangular room/frontier cannot faithfully represent an arbitrary
     * convex contour. A layer which contains one stays on the exact visibility
     * fallback; other physical layers may still use rooms.  Drill candidates
     * on that mixed-stack path are checked against the real convex contour
     * before they are accepted. */
    bool hasGeneralConvexRoomGeometry( int aNet, int aLayer ) const;
    std::optional<ROUTING_CONNECTION> findMultilayerRoomConnection(
            const std::vector<ROUTING_TERMINAL>& aStarts,
            const std::vector<ROUTING_TERMINAL>& aTargets, int aRetry, int& aExpanded,
            const ROUTER_CANCEL_CALLBACK& aCancel,
            const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress,
            const ROUTING_PAD* aFanoutTarget = nullptr ) const;
    std::optional<ROUTING_CONNECTION> findRoomConnection(
            const std::vector<ROUTING_TERMINAL>& aStarts,
            const std::vector<ROUTING_TERMINAL>& aTargets, int aRetry, int& aExpanded,
            const ROUTER_CANCEL_CALLBACK& aCancel,
            const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress ) const;
    mutable ROOM_SEARCH_METRICS m_roomMetrics;
    // Negotiated attempts keep movable copper in the tree as explicit
    // ObstacleExpansionRooms.  Free rooms still route around it; entering one
    // pays the source-shaped pass/detour/fanout-protection cost.  Final exact
    // conflict discovery and transactional insertion remain authoritative.
    mutable bool m_useRoutableObstacleRooms = false;

    struct OPEN_NODE
    {
        ROUTER_NODE node;
        double      g = 0.0;
        double      f = 0.0;
        std::size_t sequence = 0;
    };

    struct OPEN_NODE_COMPARE
    {
        bool operator()( const OPEN_NODE& aLeft, const OPEN_NODE& aRight ) const
        {
            if( aLeft.f != aRight.f )
                return aLeft.f > aRight.f;

            return aLeft.sequence > aRight.sequence;
        }
    };

    struct NODE_KEY_HASH
    {
        std::size_t operator()( const ROUTER_NODE& aNode ) const noexcept;
    };

    bool isLayerEnabled( int aLayer ) const;
    bool isPureSmdNet( int aNetCode ) const;
    int  layerOrdinal( int aLayer ) const;
    bool isOnPadLayer( const ROUTING_PAD& aPad, int aLayer ) const;
    bool isPointAllowed( const ROUTER_POINT& aPoint, int aLayer, int aNetCode,
                         bool aForVia, std::int64_t aEndpointRadius = -1,
                         std::int64_t aDrillRadius = -1,
                         std::int64_t aEdgeClearance = 0 ) const;
    bool isSegmentAllowed( const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd, int aLayer,
                           int aNetCode, bool aForVia, std::int64_t aStartRadius = -1,
                           std::int64_t aEndRadius = -1,
                           std::int64_t aSegmentRadius = -1,
                           std::int64_t aEdgeClearance = 0 ) const;
    // A frontier node has already passed the point legality check when it is
    // expanded.  Avoid rechecking that same point for every outgoing edge;
    // the destination is still checked with the complete endpoint radius.
    bool isSegmentAllowedFromKnownStart( const ROUTER_POINT& aStart,
                                         const ROUTER_POINT& aEnd, int aLayer, int aNetCode,
                                         bool aForVia, std::int64_t aEndRadius = -1,
                                         std::int64_t aSegmentRadius = -1,
                                         std::int64_t aEdgeClearance = 0 ) const;
    bool isInsideBoard( const ROUTER_POINT& aPoint, std::int64_t aMargin ) const;
    bool isInsideOutline( const ROUTER_POINT& aPoint ) const;
    bool isInsidePolygon( const ROUTER_POINT& aPoint,
                          const std::vector<ROUTER_POINT>& aPolygon ) const;
    bool isNearBoardEdge( const ROUTER_POINT& aPoint, std::int64_t aMargin ) const;
    /** Static collision-only source copper is deliberately absent from the
     * worker board's electrical graph.  Normal occupancy checks still cover
     * static drill items that remain live, but a forced-insertion transaction
     * temporarily removes every initial victim.  Retain the same-net
     * hole-to-hole rule for those removed source vias while selecting a new
     * via location. */
    bool hasStaticViaDrillClearance(
            const ROUTING_CONNECTION& aCandidate,
            const std::vector<ROUTING_CONNECTION>& aStaticDrillObstacles ) const;
    /** DrillItemMover permits trace *and* ConductionArea normal contacts. A
     * source via may therefore move while touching a filled same-net area,
     * but the immutable native proposal must keep its translated annulus in
     * every area it originally contacted; otherwise the worker could claim a
     * routed connection that the host refill disconnects. */
    bool preservesConductionAreaContacts( const ROUTING_CONNECTION& aOriginal,
                                          const ROUTING_CONNECTION& aReplacement ) const;
    std::int64_t netTrackRadius( int aNetCode ) const;
    std::int64_t netViaRadius( int aNetCode ) const;
    std::int64_t netViaDrillRadius( int aNetCode ) const;
    std::int64_t netClearance( int aNetCode ) const;
    std::int64_t endpointRadius( int aNetCode, const ROUTER_POINT& aPoint ) const;
    std::int64_t pairClearance( int aFirstNetCode, int aSecondNetCode,
                                int aLayer = -1 ) const;
    std::int64_t edgePairClearance( int aFirstNetCode, int aSecondNetCode, int aLayer,
                                    std::int64_t aFirstEdgeClearance = 0,
                                    std::int64_t aSecondEdgeClearance = 0 ) const;
    /**
     * Exact worker-route collision predicate shared by normal occupancy
     * conflict discovery and speculative forced-via placement.  A via mover
     * cannot rely on CanInsertSegment() alone: its candidate route may be
     * clear of immutable board geometry while still landing on the incoming
     * transient trace that is about to be committed.
     */
    bool connectionsConflict( const ROUTING_CONNECTION& aCandidate,
                             const ROUTING_CONNECTION& aExisting ) const;
    std::int64_t obstacleExpansionRadius( const ROUTING_OBSTACLE& aObstacle, int aNetCode,
                                          int aLayer, bool aForVia,
                                          std::int64_t aCandidateRadius = -1,
                                          std::int64_t aCandidateDrillRadius = -1,
                                          std::int64_t aCandidateEdgeClearance = 0 ) const;
    const std::vector<std::size_t>& obstacleIndices( int aLayer ) const;
    void collectObstacleIndices( int aLayer, const ROUTER_BOX& aQuery,
                                 std::vector<std::size_t>& aResult ) const;

    std::vector<ROUTER_NODE> neighbours( const ROUTER_NODE& aNode ) const;
    std::vector<ROUTER_NODE> adaptiveNeighbours(
            const ROUTER_NODE& aNode, const ROUTING_PAD& aTarget,
            const std::vector<ROUTER_NODE>& aLandmarks, int aNetCode ) const;
    std::vector<ROUTER_NODE> buildLandmarks( const ROUTING_PAD& aStart,
                                             const ROUTING_PAD& aTarget,
                                             int aNetCode ) const;
    double heuristic( const ROUTER_NODE& aNode, const ROUTER_POINT& aTarget,
                      int aTargetLayer, const AUTOROUTE_CONTROL& aControl ) const;
    bool canFinish( const ROUTER_NODE& aNode, const ROUTING_PAD& aTarget,
                    int aNetCode ) const;
    bool assignViaStyles( ROUTING_CONNECTION& aConnection ) const;

private:
    const BOARD_SNAPSHOT&     m_board;
    const AUTOROUTER_SETTINGS& m_settings;
    ROUTING_OCCUPANCY&        m_occupancy;
    mutable std::int64_t       m_activeGridStep;
    // Negotiated-congestion retries may temporarily cross committed copper;
    // the batch layer removes the specific conflicting connections from the
    // occupancy map when the candidate is accepted.
    mutable bool m_allowRipupOccupancy = false;
    mutable LEGACY_DESTINATION_DISTANCE m_legacyDestinationDistance;
    std::unordered_map<int, std::vector<std::size_t>> m_obstaclesByLayer;
    std::unordered_map<ROUTER_CELL_KEY, std::vector<std::size_t>, ROUTER_CELL_HASH>
            m_obstaclesBySpatialCell;
    // Visibility landmarks use the same coarse spatial partition as
    // obstacles.  The expansion graph is deliberately board-wide, but a
    // frontier element should only sort landmarks in the nearby rooms; a
    // full scan of all 2048 landmarks for every A* node turns a large board
    // into an accidental quadratic search.
    std::unordered_map<ROUTER_CELL_KEY, std::vector<std::size_t>, ROUTER_CELL_HASH>
            m_landmarksBySpatialCell;
    std::unordered_map<int, std::vector<std::size_t>> m_largeObstaclesByLayer;
    std::int64_t m_obstacleBucketSize = 1;
    std::int64_t m_maxObstacleSearchInflation = 0;
    std::vector<ROUTER_BOX> m_obstacleBounds;
    // A visibility query can visit several spatial buckets containing the
    // same obstacle.  Rebuilding and sorting a temporary candidate vector on
    // every A* edge dominated large-board routing.  These marks let the query
    // collect each obstacle once without allocating or sorting that vector.
    mutable std::vector<std::uint32_t> m_obstacleQueryMarks;
    mutable std::uint32_t              m_obstacleQueryGeneration = 0;
    // Opt-in diagnostics only.  These counters make it possible to tell
    // whether a difficult search is spending its time in spatial candidate
    // collection or in the exact geometry predicates.
    mutable std::uint64_t m_debugObstacleQueries = 0;
    mutable std::uint64_t m_debugObstacleCandidates = 0;
    mutable std::uint64_t m_debugPointChecks = 0;
    mutable std::uint64_t m_debugSegmentChecks = 0;
    std::unordered_map<int, std::int64_t> m_trackRadii;
    std::unordered_map<int, std::int64_t> m_viaRadii;
    std::unordered_map<int, std::int64_t> m_viaDrillRadii;
    std::unordered_map<int, std::int64_t> m_netClearances;
    int m_viaOverrideNetCode = 0;
    std::optional<ROUTING_VIA_PROFILE> m_viaOverride;
    // Pad centers are queried for almost every visibility candidate when
    // distinguishing same-net holes from foreign drills.  Cache the result by
    // net and position instead of rescanning every net's pad list per query.
    std::unordered_map<ROUTER_CELL_KEY, std::int64_t, ROUTER_CELL_HASH> m_endpointRadii;
    // Obstacle corners, board boundaries, pad locations and drill-page
    // centres are independent of the connection being routed.  Keep one
    // immutable visibility-landmark set per search engine instead of
    // rebuilding it for every pad pair on a large board.
    std::vector<ROUTER_NODE> m_baseLandmarks;
    mutable std::map<std::tuple<int, int, int>, std::int64_t> m_pairClearances;
};

} // namespace KICAD_AUTOROUTER
