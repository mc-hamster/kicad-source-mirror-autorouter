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
    MAZE_SEARCH_ENGINE( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
                        ROUTING_OCCUPANCY& aOccupancy );

    std::optional<ROUTING_CONNECTION> FindConnection( const ROUTING_PAD& aStart,
                                                       const ROUTING_PAD& aTarget,
                                                       int aRetry,
                                                       int& aExpandedNodes,
                                                       const ROUTER_CANCEL_CALLBACK& aCancel,
                                                       const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress = {},
                                                       const std::vector<ROUTING_TERMINAL>& aStarts = {},
                                                       const std::vector<ROUTING_TERMINAL>& aTargets = {} ) const;

    // Retry searches may temporarily cross committed routes.  Resolve those
    // crossings with the same netclass, layer-span, copper, and drill rules
    // used by the search engine before the batch layer removes a victim.
    std::vector<ROUTING_CONNECTION> FindConflictingConnections(
            const ROUTING_CONNECTION& aCandidate ) const;

    bool CanUseSegment( int aNetCode, const ROUTER_NODE& aStart, const ROUTER_NODE& aEnd,
                        bool aForVia = false ) const;
    /** Strict insertion check: never inherits a negotiated-search ripup flag.
     * Endpoint copper is the new trace width, not the already-existing pad size.
     */
    bool CanInsertSegment( int aNetCode, const ROUTER_NODE& aStart, const ROUTER_NODE& aEnd ) const;
    const ROOM_SEARCH_METRICS& LastRoomSearchMetrics() const { return m_roomMetrics; }

private:
    std::vector<SHAPE_TREE_ENTRY> roomObstacles( int aNet, int aLayer, bool aForVia,
                                               const ROUTER_CANCEL_CALLBACK& aCancel ) const;
    std::optional<ROUTING_CONNECTION> findMultilayerRoomConnection(
            const std::vector<ROUTING_TERMINAL>& aStarts,
            const std::vector<ROUTING_TERMINAL>& aTargets, int aRetry, int& aExpanded,
            const ROUTER_CANCEL_CALLBACK& aCancel,
            const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress ) const;
    std::optional<ROUTING_CONNECTION> findRoomConnection(
            const std::vector<ROUTING_TERMINAL>& aStarts,
            const std::vector<ROUTING_TERMINAL>& aTargets, int aRetry, int& aExpanded,
            const ROUTER_CANCEL_CALLBACK& aCancel,
            const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress ) const;
    mutable ROOM_SEARCH_METRICS m_roomMetrics;

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
    int  layerOrdinal( int aLayer ) const;
    bool isOnPadLayer( const ROUTING_PAD& aPad, int aLayer ) const;
    bool isPointAllowed( const ROUTER_POINT& aPoint, int aLayer, int aNetCode,
                         bool aForVia, std::int64_t aEndpointRadius = -1 ) const;
    bool isSegmentAllowed( const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd, int aLayer,
                           int aNetCode, bool aForVia, std::int64_t aStartRadius = -1,
                           std::int64_t aEndRadius = -1 ) const;
    // A frontier node has already passed the point legality check when it is
    // expanded.  Avoid rechecking that same point for every outgoing edge;
    // the destination is still checked with the complete endpoint radius.
    bool isSegmentAllowedFromKnownStart( const ROUTER_POINT& aStart,
                                         const ROUTER_POINT& aEnd, int aLayer, int aNetCode,
                                         bool aForVia, std::int64_t aEndRadius = -1 ) const;
    bool isInsideBoard( const ROUTER_POINT& aPoint, std::int64_t aMargin ) const;
    bool isInsideOutline( const ROUTER_POINT& aPoint ) const;
    bool isInsidePolygon( const ROUTER_POINT& aPoint,
                          const std::vector<ROUTER_POINT>& aPolygon ) const;
    bool isNearBoardEdge( const ROUTER_POINT& aPoint, std::int64_t aMargin ) const;
    std::int64_t netTrackRadius( int aNetCode ) const;
    std::int64_t netViaRadius( int aNetCode ) const;
    std::int64_t netViaDrillRadius( int aNetCode ) const;
    std::int64_t netClearance( int aNetCode ) const;
    std::int64_t endpointRadius( int aNetCode, const ROUTER_POINT& aPoint ) const;
    std::int64_t pairClearance( int aFirstNetCode, int aSecondNetCode,
                                int aLayer = -1 ) const;
    std::int64_t obstacleExpansionRadius( const ROUTING_OBSTACLE& aObstacle, int aNetCode,
                                          int aLayer, bool aForVia,
                                          std::int64_t aCandidateRadius = -1 ) const;
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
