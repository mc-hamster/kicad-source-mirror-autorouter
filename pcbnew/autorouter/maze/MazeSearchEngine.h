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
 * This is a direct, data-oriented translation of the same responsibilities:
 * expand a layer-aware search frontier, score trace/via/direction/congestion
 * costs, and return a complete connection path without touching the host PCB.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "AutorouteControl.h"
#include "DestinationDistance.h"
#include "MazeExpansionEngine.h"


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

    void Add( const ROUTING_CONNECTION& aConnection );
    void Remove( const ROUTING_CONNECTION& aConnection );
    void Clear();
    int  Usage( const ROUTER_CELL_KEY& aCell, int aNetCode ) const;
    int  ProximityUsage( const ROUTER_CELL_KEY& aCell, int aNetCode ) const;
    const std::vector<ROUTING_CONNECTION>& Connections() const { return m_connections; }

    std::vector<ROUTER_CELL_KEY> CellsForSegment( const ROUTER_NODE& aStart,
                                                  const ROUTER_NODE& aEnd ) const;

private:
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
                                                       const ROUTER_CANCEL_CALLBACK& aCancel ) const;

    bool CanUseSegment( int aNetCode, const ROUTER_NODE& aStart, const ROUTER_NODE& aEnd,
                        bool aForVia = false ) const;

private:
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
    mutable DESTINATION_DISTANCE m_destinationDistance;
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
    std::unordered_map<int, std::int64_t> m_trackRadii;
    std::unordered_map<int, std::int64_t> m_viaRadii;
    std::unordered_map<int, std::int64_t> m_viaDrillRadii;
    std::unordered_map<int, std::int64_t> m_netClearances;
    // Obstacle corners, board boundaries, pad locations and drill-page
    // centres are independent of the connection being routed.  Keep one
    // immutable visibility-landmark set per search engine instead of
    // rebuilding it for every pad pair on a large board.
    std::vector<ROUTER_NODE> m_baseLandmarks;
    mutable std::map<std::tuple<int, int, int>, std::int64_t> m_pairClearances;
};

} // namespace KICAD_AUTOROUTER
