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
 */

#include "MazeSearchEngine.h"
#include "../rules/ViaRule.h"

#include "../AutorouterDebug.h"
#include "../expansion/ExpansionGraph.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>


namespace KICAD_AUTOROUTER
{

namespace
{

std::int64_t squaredDistance( const ROUTER_POINT& aLeft, const ROUTER_POINT& aRight )
{
    const long double dx = static_cast<long double>( aLeft.x ) - aRight.x;
    const long double dy = static_cast<long double>( aLeft.y ) - aRight.y;
    const long double value = dx * dx + dy * dy;

    return value >= static_cast<long double>( std::numeric_limits<std::int64_t>::max() )
                   ? std::numeric_limits<std::int64_t>::max()
                   : static_cast<std::int64_t>( value );
}


// Project onto existing copper, not onto the representative pad's centre.
ROUTER_POINT terminalPoint( const ROUTING_TERMINAL& aTerminal, const ROUTER_POINT& aFrom )
{
    if( !aTerminal.segmentEnd )
        return aTerminal.pad.position;
    const auto& start = aTerminal.pad.position;
    const auto& end = *aTerminal.segmentEnd;
    const long double dx = static_cast<long double>( end.x ) - start.x;
    const long double dy = static_cast<long double>( end.y ) - start.y;
    const long double lengthSquared = dx * dx + dy * dy;
    const long double t = lengthSquared == 0 ? 0 : std::clamp(
            ( ( static_cast<long double>( aFrom.x ) - start.x ) * dx
              + ( static_cast<long double>( aFrom.y ) - start.y ) * dy ) / lengthSquared,
            0.0L, 1.0L );
    return { std::llround( start.x + t * dx ), std::llround( start.y + t * dy ) };
}


double distance( const ROUTER_POINT& aLeft, const ROUTER_POINT& aRight )
{
    return std::sqrt( static_cast<double>( squaredDistance( aLeft, aRight ) ) );
}


bool rangesOverlap( std::int64_t aMinA, std::int64_t aMaxA, std::int64_t aMinB,
                    std::int64_t aMaxB )
{
    if( aMinA > aMaxA )
        std::swap( aMinA, aMaxA );

    if( aMinB > aMaxB )
        std::swap( aMinB, aMaxB );

    return aMinA <= aMaxB && aMinB <= aMaxA;
}


int orientation( const ROUTER_POINT& a, const ROUTER_POINT& b, const ROUTER_POINT& c )
{
    const long double value = static_cast<long double>( b.x - a.x ) * ( c.y - a.y )
                              - static_cast<long double>( b.y - a.y ) * ( c.x - a.x );

    if( value > 0 )
        return 1;
    if( value < 0 )
        return -1;
    return 0;
}


bool onSegment( const ROUTER_POINT& a, const ROUTER_POINT& b, const ROUTER_POINT& p )
{
    return orientation( a, b, p ) == 0 && rangesOverlap( a.x, b.x, p.x, p.x )
           && rangesOverlap( a.y, b.y, p.y, p.y );
}


bool segmentsIntersect( const ROUTER_POINT& a, const ROUTER_POINT& b, const ROUTER_POINT& c,
                        const ROUTER_POINT& d )
{
    const int o1 = orientation( a, b, c );
    const int o2 = orientation( a, b, d );
    const int o3 = orientation( c, d, a );
    const int o4 = orientation( c, d, b );

    if( o1 != o2 && o3 != o4 )
        return true;

    return ( o1 == 0 && onSegment( a, b, c ) ) || ( o2 == 0 && onSegment( a, b, d ) )
           || ( o3 == 0 && onSegment( c, d, a ) ) || ( o4 == 0 && onSegment( c, d, b ) );
}


double pointToSegmentDistance( const ROUTER_POINT& aPoint, const ROUTER_POINT& aStart,
                               const ROUTER_POINT& aEnd )
{
    const long double dx = static_cast<long double>( aEnd.x ) - aStart.x;
    const long double dy = static_cast<long double>( aEnd.y ) - aStart.y;
    const long double lengthSquared = dx * dx + dy * dy;

    if( lengthSquared <= 0.0L )
        return distance( aPoint, aStart );

    const long double t = std::clamp(
            ( static_cast<long double>( aPoint.x ) - aStart.x ) * dx
                    + ( static_cast<long double>( aPoint.y ) - aStart.y ) * dy,
            0.0L, lengthSquared )
                          / lengthSquared;
    const long double x = static_cast<long double>( aStart.x ) + t * dx;
    const long double y = static_cast<long double>( aStart.y ) + t * dy;
    const long double px = static_cast<long double>( aPoint.x ) - x;
    const long double py = static_cast<long double>( aPoint.y ) - y;

    return std::sqrt( static_cast<double>( px * px + py * py ) );
}


bool pointInOrOnPolygon( const ROUTER_POINT& aPoint, const std::vector<ROUTER_POINT>& aPolygon )
{
    if( aPolygon.size() < 3 )
        return false;

    bool inside = false;

    for( std::size_t i = 0, j = aPolygon.size() - 1; i < aPolygon.size(); j = i++ )
    {
        const ROUTER_POINT& a = aPolygon[i];
        const ROUTER_POINT& b = aPolygon[j];

        if( onSegment( a, b, aPoint ) )
            return true;

        const bool intersects = ( ( a.y > aPoint.y ) != ( b.y > aPoint.y ) )
                                 && ( static_cast<long double>( b.x - a.x )
                                              * ( aPoint.y - a.y )
                                      / static_cast<long double>( b.y - a.y )
                                      + a.x
                                      > aPoint.x );

        if( intersects )
            inside = !inside;
    }

    return inside;
}


bool pointNearPolygon( const ROUTER_POINT& aPoint,
                       const std::vector<ROUTER_POINT>& aPolygon,
                       std::int64_t aRadius )
{
    if( aPolygon.empty() )
        return false;

    for( std::size_t i = 0; i < aPolygon.size(); ++i )
    {
        const ROUTER_POINT& start = aPolygon[i];
        const ROUTER_POINT& end = aPolygon[( i + 1 ) % aPolygon.size()];

        // A clearance-expanded boundary is legal at exact tangency.  KiCad's
        // DRC treats the required clearance as a minimum separation, so only
        // a strict penetration is a collision here.  This matters for a pad
        // whose copper exactly fits inside a polygon hole.
        if( pointToSegmentDistance( aPoint, start, end ) < aRadius )
            return true;
    }

    return false;
}


bool pointInPolygonWithHoles( const ROUTER_POINT& aPoint,
                              const std::vector<ROUTER_POINT>& aPolygon,
                              const std::vector<std::vector<ROUTER_POINT>>& aHoles )
{
    if( !pointInOrOnPolygon( aPoint, aPolygon ) )
        return false;

    return std::none_of( aHoles.begin(), aHoles.end(),
                         [&]( const std::vector<ROUTER_POINT>& aHole )
                         {
                             return pointInOrOnPolygon( aPoint, aHole );
                         } );
}


bool pointNearPolygonWithHoles( const ROUTER_POINT& aPoint,
                                const std::vector<ROUTER_POINT>& aPolygon,
                                const std::vector<std::vector<ROUTER_POINT>>& aHoles,
                                std::int64_t aRadius )
{
    if( pointNearPolygon( aPoint, aPolygon, aRadius ) )
        return true;

    return std::any_of( aHoles.begin(), aHoles.end(),
                        [&]( const std::vector<ROUTER_POINT>& aHole )
                        {
                            return pointNearPolygon( aPoint, aHole, aRadius );
                        } );
}


ROUTER_BOX obstacleBounds( const ROUTING_OBSTACLE& aObstacle )
{
    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
        return aObstacle.box;

    ROUTER_BOX result;
    bool       initialized = false;

    auto include = [&]( const ROUTER_POINT& aPoint )
    {
        if( !initialized )
        {
            result = { aPoint.x, aPoint.y, aPoint.x, aPoint.y };
            initialized = true;
            return;
        }

        result.minX = std::min( result.minX, aPoint.x );
        result.minY = std::min( result.minY, aPoint.y );
        result.maxX = std::max( result.maxX, aPoint.x );
        result.maxY = std::max( result.maxY, aPoint.y );
    };

    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
    {
        include( aObstacle.start );
        include( aObstacle.end );
    }
    else
    {
        for( const ROUTER_POINT& point : aObstacle.polygon )
            include( point );
    }

    return initialized ? result : ROUTER_BOX{};
}


bool boxesOverlap( const ROUTER_BOX& aLeft, const ROUTER_BOX& aRight, std::int64_t aInflation )
{
    return aLeft.maxX >= aRight.minX - aInflation
           && aRight.maxX >= aLeft.minX - aInflation
           && aLeft.maxY >= aRight.minY - aInflation
           && aRight.maxY >= aLeft.minY - aInflation;
}


bool boxContainsInflated( const ROUTER_BOX& aBox, const ROUTER_POINT& aPoint,
                          std::int64_t aInflation )
{
    return aPoint.x >= aBox.minX - aInflation && aPoint.x <= aBox.maxX + aInflation
           && aPoint.y >= aBox.minY - aInflation && aPoint.y <= aBox.maxY + aInflation;
}


std::int64_t floorDivide( std::int64_t aValue, std::int64_t aDivisor )
{
    if( aDivisor <= 0 )
        return aValue;

    if( aValue >= 0 )
        return aValue / aDivisor;

    return -( ( -aValue + aDivisor - 1 ) / aDivisor );
}


std::int64_t saturatedAdd( std::int64_t aLeft, std::int64_t aRight )
{
    if( aRight > 0 && aLeft > std::numeric_limits<std::int64_t>::max() - aRight )
        return std::numeric_limits<std::int64_t>::max();

    if( aRight < 0 && aLeft < std::numeric_limits<std::int64_t>::min() - aRight )
        return std::numeric_limits<std::int64_t>::min();

    return aLeft + aRight;
}


bool segmentsWithinClearance( const ROUTER_POINT& aFirstStart,
                              const ROUTER_POINT& aFirstEnd,
                              const ROUTER_POINT& aSecondStart,
                              const ROUTER_POINT& aSecondEnd,
                              double aClearance )
{
    if( segmentsIntersect( aFirstStart, aFirstEnd, aSecondStart, aSecondEnd ) )
        return true;

    return pointToSegmentDistance( aFirstStart, aSecondStart, aSecondEnd ) <= aClearance
           || pointToSegmentDistance( aFirstEnd, aSecondStart, aSecondEnd ) <= aClearance
           || pointToSegmentDistance( aSecondStart, aFirstStart, aFirstEnd ) <= aClearance
           || pointToSegmentDistance( aSecondEnd, aFirstStart, aFirstEnd ) <= aClearance;
}


} // namespace


std::size_t ROUTER_CELL_HASH::operator()( const ROUTER_CELL_KEY& aKey ) const noexcept
{
    std::size_t result = std::hash<std::int64_t>{}( aKey.x );
    result ^= std::hash<std::int64_t>{}( aKey.y ) + 0x9e3779b9 + ( result << 6 ) + ( result >> 2 );
    result ^= std::hash<int>{}( aKey.layer ) + 0x9e3779b9 + ( result << 6 ) + ( result >> 2 );
    return result;
}


std::vector<ROUTER_CELL_KEY> ROUTING_OCCUPANCY::CellsForSegment( const ROUTER_NODE& aStart,
                                                                 const ROUTER_NODE& aEnd ) const
{
    const double length = distance( aStart.point, aEnd.point );
    const int count = std::max( 1, static_cast<int>( std::ceil( length / m_gridStep ) ) );
    std::vector<ROUTER_CELL_KEY> cells;
    cells.reserve( static_cast<std::size_t>( count ) + 1 );

    const auto snap = [this]( std::int64_t aValue )
    {
        return static_cast<std::int64_t>(
                std::llround( static_cast<double>( aValue ) / m_gridStep ) * m_gridStep );
    };

    for( int i = 0; i <= count; ++i )
    {
        const double ratio = static_cast<double>( i ) / count;
        ROUTER_CELL_KEY cell;
        cell.x = snap( static_cast<std::int64_t>( std::llround(
                aStart.point.x + ( aEnd.point.x - aStart.point.x ) * ratio ) ) );
        cell.y = snap( static_cast<std::int64_t>( std::llround(
                aStart.point.y + ( aEnd.point.y - aStart.point.y ) * ratio ) ) );
        cell.layer = aStart.layer;

        if( cells.empty() || !( cells.back() == cell ) )
            cells.push_back( cell );
    }

    return cells;
}


void ROUTING_OCCUPANCY::InitializeBoard( const BOARD_SNAPSHOT& aBoard,
                                           const AUTOROUTER_SETTINGS& aSettings )
{
    auto board = std::make_unique<ROUTING_BOARD>( aBoard, aSettings );
    for( const auto& connection : m_connections )
        board->AddRoute( connection );
    m_board = std::move( board );
}


struct ROUTING_OCCUPANCY::TRANSACTION::STATE
{
    decltype( ROUTING_OCCUPANCY::m_usage ) usage;
    std::vector<ROUTING_CONNECTION> connections;
    std::unique_ptr<ROUTING_BOARD::TRANSACTION> board;
};

ROUTING_OCCUPANCY::TRANSACTION::TRANSACTION( ROUTING_OCCUPANCY& occupancy ) :
        m_occupancy( occupancy ), m_before( std::make_unique<STATE>() )
{
    m_before->usage = occupancy.m_usage;
    m_before->connections = occupancy.m_connections;
    if( occupancy.m_board )
        m_before->board = std::make_unique<ROUTING_BOARD::TRANSACTION>( *occupancy.m_board );
}

ROUTING_OCCUPANCY::TRANSACTION::~TRANSACTION()
{
    if( m_before )
    {
        m_occupancy.m_usage.swap( m_before->usage );
        m_occupancy.m_connections.swap( m_before->connections );
        // STATE destruction rolls back the pointer-bearing copper index too.
    }
}

void ROUTING_OCCUPANCY::TRANSACTION::Commit()
{
    if( !m_before )
        return;
    if( m_before->board )
        m_before->board->Commit();
    m_before.reset();
}

void ROUTING_OCCUPANCY::Add( const ROUTING_CONNECTION& aConnection )
{
    if( !aConnection.complete )
        throw std::invalid_argument( "Cannot insert an incomplete routing connection" );
    TRANSACTION transaction( *this );
    if( m_board )
        m_board->AddRoute( aConnection );
    m_connections.push_back( aConnection );

    for( std::size_t i = 1; i < aConnection.nodes.size(); ++i )
    {
        if( aConnection.nodes[i - 1].layer != aConnection.nodes[i].layer )
            continue;

        for( const ROUTER_CELL_KEY& cell : CellsForSegment( aConnection.nodes[i - 1],
                                                             aConnection.nodes[i] ) )
        {
            ++m_usage[cell][aConnection.netCode];
        }
    }
    transaction.Commit();
}


void ROUTING_OCCUPANCY::Remove( const ROUTING_CONNECTION& aConnection )
{
    auto connectionIt = std::find_if(
            m_connections.begin(), m_connections.end(),
            [&]( const ROUTING_CONNECTION& aExisting )
            {
                return aExisting.netCode == aConnection.netCode
                       && aExisting.nodes == aConnection.nodes;
            } );

    if( connectionIt == m_connections.end() )
        return;

    // Copy first: callers may pass a reference into Connections().
    const ROUTING_CONNECTION removed = *connectionIt;
    if( m_board )
        m_board->RemoveRoute( removed );
    m_connections.erase( connectionIt );

    for( std::size_t i = 1; i < removed.nodes.size(); ++i )
    {
        if( removed.nodes[i - 1].layer != removed.nodes[i].layer )
            continue;

        for( const ROUTER_CELL_KEY& cell : CellsForSegment( removed.nodes[i - 1],
                                                             removed.nodes[i] ) )
        {
            auto usageIt = m_usage.find( cell );

            if( usageIt == m_usage.end() )
                continue;

            auto netIt = usageIt->second.find( removed.netCode );

            if( netIt == usageIt->second.end() )
                continue;

            if( --netIt->second <= 0 )
                usageIt->second.erase( netIt );

            if( usageIt->second.empty() )
                m_usage.erase( usageIt );
        }
    }
}


void ROUTING_OCCUPANCY::Clear()
{
    if( m_board )
        m_board->ClearRoutes();
    m_usage.clear();
    m_connections.clear();
}


int ROUTING_OCCUPANCY::Usage( const ROUTER_CELL_KEY& aCell, int aNetCode ) const
{
    auto usageIt = m_usage.find( aCell );

    if( usageIt == m_usage.end() )
        return 0;

    int total = 0;

    for( const auto& [netCode, count] : usageIt->second )
    {
        if( netCode != aNetCode )
            total += count;
    }

    return total;
}


int ROUTING_OCCUPANCY::SegmentUsage( const ROUTER_NODE& aStart, const ROUTER_NODE& aEnd,
                                     int aNetCode ) const
{
    int maximum = 0;

    for( const ROUTER_CELL_KEY& cell : CellsForSegment( aStart, aEnd ) )
        maximum = std::max( maximum, Usage( cell, aNetCode ) );

    return maximum;
}


int ROUTING_OCCUPANCY::ProximityUsage( const ROUTER_CELL_KEY& aCell, int aNetCode ) const
{
    int total = 0;
    const auto snap = [this]( std::int64_t aValue )
    {
        return static_cast<std::int64_t>(
                std::llround( static_cast<double>( aValue ) / m_gridStep ) * m_gridStep );
    };
    const ROUTER_CELL_KEY center{ snap( aCell.x ), snap( aCell.y ), aCell.layer };

    // Keep exact overlap a hard conflict, but expose nearby reservations to the
    // cost model.  This gives the search a negotiated-congestion signal without
    // allowing the committed result to contain intersecting foreign-net routes.
    for( int dx = -1; dx <= 1; ++dx )
    {
        for( int dy = -1; dy <= 1; ++dy )
        {
            total += Usage( { center.x + dx * m_gridStep, center.y + dy * m_gridStep,
                               center.layer },
                            aNetCode );
        }
    }

    return total;
}


std::vector<ROUTING_CONNECTION> ROUTING_OCCUPANCY::ConflictingConnections(
        const ROUTING_CONNECTION& aConnection ) const
{
    std::vector<ROUTING_CONNECTION> result;
    std::unordered_set<ROUTER_CELL_KEY, ROUTER_CELL_HASH> candidateCells;

    const auto snap = [this]( std::int64_t aValue )
    {
        return static_cast<std::int64_t>(
                std::llround( static_cast<double>( aValue ) / m_gridStep ) * m_gridStep );
    };

    const auto addNode = [&]( std::unordered_set<ROUTER_CELL_KEY, ROUTER_CELL_HASH>& aCells,
                              const ROUTER_NODE& aNode )
    {
        if( aNode.layer >= 0 )
            aCells.insert( { snap( aNode.point.x ), snap( aNode.point.y ), aNode.layer } );
    };

    const auto collectCells = [&]( const ROUTING_CONNECTION& aRoute,
                                   std::unordered_set<ROUTER_CELL_KEY, ROUTER_CELL_HASH>& aCells )
    {
        for( const ROUTER_NODE& node : aRoute.nodes )
            addNode( aCells, node );

        for( std::size_t index = 1; index < aRoute.nodes.size(); ++index )
        {
            const ROUTER_NODE& start = aRoute.nodes[index - 1];
            const ROUTER_NODE& end = aRoute.nodes[index];

            if( start.layer == end.layer )
            {
                for( const ROUTER_CELL_KEY& cell : CellsForSegment( start, end ) )
                    aCells.insert( cell );
            }
        }
    };

    collectCells( aConnection, candidateCells );

    for( const ROUTING_CONNECTION& existing : m_connections )
    {
        if( existing.netCode == aConnection.netCode )
            continue;

        std::unordered_set<ROUTER_CELL_KEY, ROUTER_CELL_HASH> existingCells;
        collectCells( existing, existingCells );

        const bool conflicts = std::any_of(
                candidateCells.begin(), candidateCells.end(),
                [&]( const ROUTER_CELL_KEY& cell )
                {
                    return existingCells.contains( cell );
                } );

        if( conflicts )
            result.push_back( existing );
    }

    return result;
}


std::size_t MAZE_SEARCH_ENGINE::NODE_KEY_HASH::operator()( const ROUTER_NODE& aNode ) const noexcept
{
    ROUTER_CELL_HASH cellHash;
    return cellHash( { aNode.point.x, aNode.point.y, aNode.layer } );
}


MAZE_SEARCH_ENGINE::MAZE_SEARCH_ENGINE( const BOARD_SNAPSHOT& aBoard,
                                        const AUTOROUTER_SETTINGS& aSettings,
                                        ROUTING_OCCUPANCY& aOccupancy ) :
        m_board( aBoard ),
        m_settings( aSettings ),
        m_occupancy( aOccupancy ),
        m_activeGridStep( std::max<std::int64_t>( 1, aSettings.gridStepIU ) )
{
    // A search engine is constructed for one immutable board and one
    // connection at a time.  Cache the net dimensions and obstacle bounds
    // here rather than walking every pad/net/rule for every visibility probe.
    // This is especially important on real boards where a single A* route
    // evaluates thousands of event-to-landmark segments.
    for( const ROUTING_NET& net : m_board.nets )
    {
        std::int64_t width = 0;
        for( std::size_t padIndex : net.padIndices )
        {
            if( padIndex < m_board.pads.size() )
                width = std::max( width, m_board.pads[padIndex].trackWidth );
        }

        m_trackRadii[net.netCode] = std::max<std::int64_t>( 1,
                                                            ( width > 0 ? width : 150000 ) / 2 );
        m_viaRadii[net.netCode] = std::max<std::int64_t>( 1,
                                                          net.viaDiameter > 0
                                                                  ? net.viaDiameter / 2
                                                                  : 300000 );
        m_viaDrillRadii[net.netCode] = std::max<std::int64_t>(
                1, net.viaDrill > 0 ? net.viaDrill / 2 : 150000 );
        m_netClearances[net.netCode] = std::max<std::int64_t>( 0, net.clearance );

        for( std::size_t padIndex : net.padIndices )
        {
            if( padIndex >= m_board.pads.size() )
                continue;

            const ROUTING_PAD& pad = m_board.pads[padIndex];
            const ROUTER_CELL_KEY key{ pad.position.x, pad.position.y, net.netCode };
            const std::int64_t radius = pad.radius + pad.clearance;
            auto [radiusIt, inserted] = m_endpointRadii.emplace( key, radius );
            if( !inserted )
                radiusIt->second = std::max( radiusIt->second, radius );
        }
    }

    // The old implementation walked every obstacle on a layer for every
    // visibility probe.  That is acceptable for a small synthetic board but
    // turns a few thousand pads into quadratic work on a real MCU board.  A
    // coarse, immutable spatial index keeps the exact geometry predicates
    // below unchanged while making the candidate set local.  The bucket size
    // is deliberately independent from the retry-refined search grid: retry
    // changes must not invalidate the index or change search semantics.
    m_obstacleBucketSize = std::max<std::int64_t>(
            2000000, std::max<std::int64_t>( 1, m_settings.gridStepIU ) * 4 );

    std::int64_t maximumCandidateRadius = 0;
    std::int64_t maximumDrillRadius = 0;
    std::int64_t maximumNetClearance = 0;

    for( const auto& [netCode, radius] : m_trackRadii )
    {
        (void) netCode;
        maximumCandidateRadius = std::max( maximumCandidateRadius, radius );
    }

    for( const auto& [netCode, radius] : m_viaRadii )
    {
        (void) netCode;
        maximumCandidateRadius = std::max( maximumCandidateRadius, radius );
    }

    for( const auto& [netCode, radius] : m_viaDrillRadii )
    {
        (void) netCode;
        maximumDrillRadius = std::max( maximumDrillRadius, radius );
    }

    for( const auto& [netCode, clearance] : m_netClearances )
    {
        (void) netCode;
        maximumNetClearance = std::max( maximumNetClearance, clearance );
    }

    for( const ROUTING_CLEARANCE_RULE& rule : m_board.clearanceRules )
        maximumNetClearance = std::max( maximumNetClearance, rule.clearance );

    for( const ROUTING_PAD& pad : m_board.pads )
    {
        maximumCandidateRadius = std::max( maximumCandidateRadius,
                                           pad.radius + pad.clearance );
    }

    m_obstacleBounds.reserve( m_board.obstacles.size() );
    for( const ROUTING_OBSTACLE& obstacle : m_board.obstacles )
        m_obstacleBounds.push_back( obstacleBounds( obstacle ) );

    m_obstacleQueryMarks.assign( m_board.obstacles.size(), 0 );

    std::int64_t maximumObstacleInflation = 0;
    for( const ROUTING_OBSTACLE& obstacle : m_board.obstacles )
    {
        // Segment and point obstacles store their copper/drill radius outside
        // the bounding box.  Include it in the spatial-query margin or an
        // obstacle that straddles a bucket boundary could be omitted before
        // the exact collision predicate gets a chance to inspect it.
        maximumObstacleInflation = std::max( maximumObstacleInflation,
                                             obstacle.radius + obstacle.clearance );
    }

    // This is only an index query margin.  The exact per-obstacle expansion
    // is still calculated by obstacleExpansionRadius() before an obstacle is
    // accepted or rejected.  Include both copper and manufacturing terms so
    // the coarse index can never hide a valid collision candidate.
    m_maxObstacleSearchInflation = std::max<std::int64_t>(
            0, m_board.edgeClearance + maximumCandidateRadius + maximumDrillRadius
                       + maximumNetClearance + maximumObstacleInflation
                       + m_board.holeClearance + m_board.holeToHoleClearance );

    for( std::size_t index = 0; index < m_board.obstacles.size(); ++index )
    {
        std::vector<int> obstacleLayers = m_board.obstacles[index].layers;
        if( obstacleLayers.empty() )
        {
            for( const ROUTER_LAYER_SETTINGS& layer : m_settings.layers )
                obstacleLayers.push_back( layer.layerId );
        }

        for( int layer : obstacleLayers )
        {
            auto& indices = m_obstaclesByLayer[layer];
            if( std::find( indices.begin(), indices.end(), index ) == indices.end() )
                indices.push_back( index );
        }

        const ROUTER_BOX& bounds = m_obstacleBounds[index];
        const std::int64_t firstX = floorDivide( bounds.minX, m_obstacleBucketSize );
        const std::int64_t lastX = floorDivide( bounds.maxX, m_obstacleBucketSize );
        const std::int64_t firstY = floorDivide( bounds.minY, m_obstacleBucketSize );
        const std::int64_t lastY = floorDivide( bounds.maxY, m_obstacleBucketSize );

        const long double cellCount = static_cast<long double>( lastX - firstX + 1 )
                                      * static_cast<long double>( lastY - firstY + 1 );

        // Large pours/keepouts are cheaper and safer in a layer fallback
        // list than in millions of bucket entries.  Small obstacles, which
        // dominate real boards, remain local to their spatial cells.
        constexpr long double maxIndexedCells = 4096.0L;
        if( cellCount > maxIndexedCells || cellCount <= 0.0L )
        {
            for( int layer : obstacleLayers )
                m_largeObstaclesByLayer[layer].push_back( index );
            continue;
        }

        for( std::int64_t cellX = firstX; cellX <= lastX; ++cellX )
        {
            for( std::int64_t cellY = firstY; cellY <= lastY; ++cellY )
            {
                for( int layer : obstacleLayers )
                {
                    auto& bucket = m_obstaclesBySpatialCell[{ cellX, cellY, layer }];
                    if( std::find( bucket.begin(), bucket.end(), index ) == bucket.end() )
                        bucket.push_back( index );
                }
            }
        }
    }

    // Build the board-wide visibility landmarks once.  The exact net-pair
    // clearance is still checked by isSegmentAllowed(); using the largest
    // snapshot clearance for the landmark margin only makes this broad phase
    // conservative and avoids a full obstacle/pad walk for every connection.
    std::int64_t maximumTrackRadius = 1;
    std::int64_t maximumViaRadius = 1;
    for( const auto& [netCode, radius] : m_trackRadii )
    {
        (void) netCode;
        maximumTrackRadius = std::max( maximumTrackRadius, radius );
    }
    for( const auto& [netCode, radius] : m_viaRadii )
    {
        (void) netCode;
        maximumViaRadius = std::max( maximumViaRadius, radius );
    }

    ROUTING_PAD emptyTerminal;
    maximumTrackRadius = saturatedAdd( maximumTrackRadius, maximumNetClearance );
    m_baseLandmarks = EXPANSION_GRAPH::BuildLandmarks(
            m_board, m_settings, emptyTerminal, emptyTerminal, maximumTrackRadius,
            maximumViaRadius, 0 );

    // Keep the board-wide landmark set, but index it spatially so the maze
    // frontier can inspect nearby room features without scanning/sorting the
    // complete set for every expanded node.  The bucket size is intentionally
    // the same coarse size used by the obstacle index: it is independent of
    // retry refinement and therefore cannot change the route's tie-breaking
    // semantics when the active grid gets finer.
    for( std::size_t index = 0; index < m_baseLandmarks.size(); ++index )
    {
        const ROUTER_NODE& landmark = m_baseLandmarks[index];
        const ROUTER_CELL_KEY key{ floorDivide( landmark.point.x, m_obstacleBucketSize ),
                                   floorDivide( landmark.point.y, m_obstacleBucketSize ),
                                   landmark.layer };
        m_landmarksBySpatialCell[key].push_back( index );
    }

    if( autorouterDebugEnabled() )
    {
        std::ostringstream message;
        message << "search index ready pads=" << m_board.pads.size()
                << " obstacles=" << m_board.obstacles.size()
                << " layers=" << m_settings.layers.size()
                << " landmarks=" << m_baseLandmarks.size()
                << " obstacleBuckets=" << m_obstaclesBySpatialCell.size()
                << " bucketSizeIU=" << m_obstacleBucketSize;
        autorouterDebugLog( message.str() );
    }
}


bool MAZE_SEARCH_ENGINE::isLayerEnabled( int aLayer ) const
{
    return std::any_of( m_settings.layers.begin(), m_settings.layers.end(),
                        [aLayer]( const ROUTER_LAYER_SETTINGS& aLayerSetting )
                        {
                            return aLayerSetting.layerId == aLayer && aLayerSetting.enabled;
                        } );
}


int MAZE_SEARCH_ENGINE::layerOrdinal( int aLayer ) const
{
    auto it = std::find_if( m_settings.layers.begin(), m_settings.layers.end(),
                            [aLayer]( const ROUTER_LAYER_SETTINGS& aSetting )
                            {
                                return aSetting.layerId == aLayer;
                            } );

    if( it == m_settings.layers.end() )
        return aLayer;

    if( it->layerOrdinal >= 0 )
        return it->layerOrdinal;

    // Data-only tests often use 0, 1, 2 for layer IDs and omit the physical
    // ordinal.  The settings vector is the documented fallback ordering.
    return static_cast<int>( std::distance( m_settings.layers.begin(), it ) );
}


bool MAZE_SEARCH_ENGINE::isOnPadLayer( const ROUTING_PAD& aPad, int aLayer ) const
{
    return std::find( aPad.layers.begin(), aPad.layers.end(), aLayer ) != aPad.layers.end();
}


bool MAZE_SEARCH_ENGINE::isInsidePolygon( const ROUTER_POINT& aPoint,
                                          const std::vector<ROUTER_POINT>& aPolygon ) const
{
    return pointInOrOnPolygon( aPoint, aPolygon );
}


bool MAZE_SEARCH_ENGINE::isInsideOutline( const ROUTER_POINT& aPoint ) const
{
    if( m_board.boardOutline.empty() )
        return true;

    if( !isInsidePolygon( aPoint, m_board.boardOutline ) )
        return false;

    return std::none_of( m_board.boardHoles.begin(), m_board.boardHoles.end(),
                         [&]( const std::vector<ROUTER_POINT>& aHole )
                         {
                             return isInsidePolygon( aPoint, aHole );
                         } );
}


bool MAZE_SEARCH_ENGINE::isNearBoardEdge( const ROUTER_POINT& aPoint, std::int64_t aMargin ) const
{
    if( aMargin <= 0 )
        return false;

    // Some imported or in-progress boards do not have a polygonal outline,
    // but the adapter still provides the board bounding box.  Treat that box
    // as the conservative outline instead of silently allowing a track or via
    // to be placed on the edge.  This also keeps the worker and the final
    // snapshot DRC checker consistent for outline-less boards.
    if( m_board.boardOutline.size() < 3 )
    {
        return aPoint.x <= m_board.bounds.minX + aMargin
               || aPoint.x >= m_board.bounds.maxX - aMargin
               || aPoint.y <= m_board.bounds.minY + aMargin
               || aPoint.y >= m_board.bounds.maxY - aMargin;
    }

    auto nearPolygon = [&]( const std::vector<ROUTER_POINT>& aPolygon )
    {
        for( std::size_t i = 0; i < aPolygon.size(); ++i )
        {
            const ROUTER_POINT& start = aPolygon[i];
            const ROUTER_POINT& end = aPolygon[( i + 1 ) % aPolygon.size()];

            if( pointToSegmentDistance( aPoint, start, end ) < aMargin )
                return true;
        }

        return false;
    };

    if( nearPolygon( m_board.boardOutline ) )
        return true;

    for( const std::vector<ROUTER_POINT>& hole : m_board.boardHoles )
    {
        if( nearPolygon( hole ) )
            return true;
    }

    return false;
}


bool MAZE_SEARCH_ENGINE::isInsideBoard( const ROUTER_POINT& aPoint, std::int64_t aMargin ) const
{
    if( !m_board.bounds.Contains( aPoint ) || !isInsideOutline( aPoint ) )
        return false;

    return !isNearBoardEdge( aPoint, aMargin );
}


std::int64_t MAZE_SEARCH_ENGINE::netTrackRadius( int aNetCode ) const
{
    const auto it = m_trackRadii.find( aNetCode );
    return it == m_trackRadii.end() ? 75000 : it->second;
}


std::int64_t MAZE_SEARCH_ENGINE::netViaRadius( int aNetCode ) const
{
    const auto it = m_viaRadii.find( aNetCode );
    return it == m_viaRadii.end() ? 300000 : it->second;
}


std::int64_t MAZE_SEARCH_ENGINE::netViaDrillRadius( int aNetCode ) const
{
    const auto it = m_viaDrillRadii.find( aNetCode );
    return it == m_viaDrillRadii.end() ? 150000 : it->second;
}


std::int64_t MAZE_SEARCH_ENGINE::netClearance( int aNetCode ) const
{
    const auto it = m_netClearances.find( aNetCode );
    return it == m_netClearances.end() ? 0 : it->second;
}


std::int64_t MAZE_SEARCH_ENGINE::endpointRadius( int aNetCode,
                                                 const ROUTER_POINT& aPoint ) const
{
    const auto it = m_endpointRadii.find( { aPoint.x, aPoint.y, aNetCode } );
    return it == m_endpointRadii.end() ? -1 : it->second;
}


std::int64_t MAZE_SEARCH_ENGINE::pairClearance( int aFirstNetCode,
                                                int aSecondNetCode, int aLayer ) const
{
    if( aFirstNetCode == aSecondNetCode )
        return 0;

    const int first = std::min( aFirstNetCode, aSecondNetCode );
    const int second = std::max( aFirstNetCode, aSecondNetCode );
    const std::tuple<int, int, int> key{ first, second, aLayer };

    if( const auto cached = m_pairClearances.find( key ); cached != m_pairClearances.end() )
        return cached->second;

    std::int64_t result = std::max( netClearance( aFirstNetCode ),
                                    netClearance( aSecondNetCode ) );

    for( const ROUTING_CLEARANCE_RULE& rule : m_board.clearanceRules )
    {
        const bool samePair =
                ( rule.firstNetCode == aFirstNetCode && rule.secondNetCode == aSecondNetCode )
                || ( rule.firstNetCode == aSecondNetCode
                     && rule.secondNetCode == aFirstNetCode );

        if( samePair && ( rule.layer < 0 || aLayer < 0 || rule.layer == aLayer ) )
        {
            result = std::max( result, std::max<std::int64_t>( 0, rule.clearance ) );
        }
    }

    m_pairClearances.emplace( key, result );
    return result;
}


std::int64_t MAZE_SEARCH_ENGINE::obstacleExpansionRadius(
        const ROUTING_OBSTACLE& aObstacle, int aNetCode, int aLayer, bool aForVia,
        std::int64_t aCandidateRadius ) const
{
    if( !aObstacle.isHole )
    {
        const std::int64_t candidateRadius = aCandidateRadius >= 0
                                                      ? aCandidateRadius
                                                      : aForVia ? netViaRadius( aNetCode )
                                                                : netTrackRadius( aNetCode );
        const std::int64_t clearance = aObstacle.netCode != 0
                                                && aObstacle.netCode != aNetCode
                                        ? std::max( pairClearance( aNetCode, aObstacle.netCode,
                                                                   aLayer ),
                                                    aObstacle.clearance )
                                        : std::max( netClearance( aNetCode ),
                                                    aObstacle.clearance );
        return aObstacle.radius + candidateRadius + clearance;
    }

    // A drilled hole has two independent clearance constraints when the
    // candidate is a via: its copper annulus must clear the hole and its
    // mechanical drill must clear the other drill.  Taking the larger
    // expanded radius is conservative and avoids sending a live DRC query to
    // the worker thread.
    const std::int64_t copperRadius = aCandidateRadius >= 0
                                              ? aCandidateRadius
                                              : netViaRadius( aNetCode );
    const std::int64_t copperExpansion = copperRadius + m_board.holeClearance;
    const std::int64_t drillExpansion = aForVia
                                                ? netViaDrillRadius( aNetCode )
                                                          + m_board.holeToHoleClearance
                                                : ( aCandidateRadius >= 0
                                                            ? aCandidateRadius
                                                            : netTrackRadius( aNetCode ) )
                                                          + m_board.holeClearance;
    return aObstacle.radius + std::max( copperExpansion, drillExpansion );
}


const std::vector<std::size_t>& MAZE_SEARCH_ENGINE::obstacleIndices( int aLayer ) const
{
    static const std::vector<std::size_t> empty;
    const auto it = m_obstaclesByLayer.find( aLayer );
    return it == m_obstaclesByLayer.end() ? empty : it->second;
}


void MAZE_SEARCH_ENGINE::collectObstacleIndices( int aLayer, const ROUTER_BOX& aQuery,
                                                 std::vector<std::size_t>& aResult ) const
{
    aResult.clear();
    const bool debug = autorouterDebugEnabled();

    if( debug )
        ++m_debugObstacleQueries;

    if( m_obstacleQueryMarks.size() != m_board.obstacles.size() )
        m_obstacleQueryMarks.assign( m_board.obstacles.size(), 0 );

    ++m_obstacleQueryGeneration;
    if( m_obstacleQueryGeneration == 0 )
    {
        std::fill( m_obstacleQueryMarks.begin(), m_obstacleQueryMarks.end(), 0 );
        m_obstacleQueryGeneration = 1;
    }

    const auto appendUnique = [&]( const std::vector<std::size_t>& aIndices )
    {
        for( std::size_t index : aIndices )
        {
            if( index >= m_obstacleQueryMarks.size()
                || m_obstacleQueryMarks[index] == m_obstacleQueryGeneration )
            {
                continue;
            }

            m_obstacleQueryMarks[index] = m_obstacleQueryGeneration;
            aResult.push_back( index );

            if( debug )
                ++m_debugObstacleCandidates;
        }
    };

    const auto allOnLayer = m_obstaclesByLayer.find( aLayer );
    if( allOnLayer == m_obstaclesByLayer.end() )
        return;

    const auto largeOnLayer = m_largeObstaclesByLayer.find( aLayer );
    if( largeOnLayer != m_largeObstaclesByLayer.end() )
        appendUnique( largeOnLayer->second );

    const std::int64_t margin = m_maxObstacleSearchInflation;
    const std::int64_t firstX = floorDivide( saturatedAdd( aQuery.minX, -margin ),
                                             m_obstacleBucketSize );
    const std::int64_t lastX = floorDivide( saturatedAdd( aQuery.maxX, margin ),
                                            m_obstacleBucketSize );
    const std::int64_t firstY = floorDivide( saturatedAdd( aQuery.minY, -margin ),
                                             m_obstacleBucketSize );
    const std::int64_t lastY = floorDivide( saturatedAdd( aQuery.maxY, margin ),
                                            m_obstacleBucketSize );

    const long double cellCount = static_cast<long double>( lastX - firstX + 1 )
                                  * static_cast<long double>( lastY - firstY + 1 );

    // A long visibility segment or a very large clearance region can span
    // most of the index.  Falling back to the layer list is bounded and
    // avoids turning a candidate query into an integer-overflow loop.
    constexpr long double maxQueryCells = 1024.0L;
    if( cellCount > maxQueryCells || cellCount <= 0.0L )
    {
        appendUnique( allOnLayer->second );
        return;
    }

    for( std::int64_t cellX = firstX; cellX <= lastX; ++cellX )
    {
        for( std::int64_t cellY = firstY; cellY <= lastY; ++cellY )
        {
            const auto bucket = m_obstaclesBySpatialCell.find( { cellX, cellY, aLayer } );
            if( bucket == m_obstaclesBySpatialCell.end() )
                continue;

            appendUnique( bucket->second );
        }
    }
}


bool MAZE_SEARCH_ENGINE::isPointAllowed( const ROUTER_POINT& aPoint, int aLayer, int aNetCode,
                                         bool aForVia,
                                         std::int64_t aEndpointRadius ) const
{
    if( autorouterDebugEnabled() )
        ++m_debugPointChecks;

    if( aForVia )
    {
        // Inactive routing layers still carry the copper/drill of a through-via.
        // Only trace entry/exit is gated by layer enablement.
        const auto layerIt = std::find_if(
                m_settings.layers.begin(), m_settings.layers.end(),
                [aLayer]( const ROUTER_LAYER_SETTINGS& aSetting )
                {
                    return aSetting.layerId == aLayer;
                } );

        if( layerIt == m_settings.layers.end() )
        {
            return false;
        }
    }
    else if( !isLayerEnabled( aLayer ) )
    {
        return false;
    }

    const std::int64_t geometryRadius = aEndpointRadius >= 0
                                                ? aEndpointRadius
                                                : ( aForVia ? netViaRadius( aNetCode )
                                                            : netTrackRadius( aNetCode ) );
    const std::int64_t margin = m_board.edgeClearance + geometryRadius;

    if( !isInsideBoard( aPoint, margin ) )
        return false;

    std::vector<std::size_t> candidateObstacles;
    collectObstacleIndices( aLayer, { aPoint.x, aPoint.y, aPoint.x, aPoint.y },
                             candidateObstacles );

    for( std::size_t obstacleIndex : candidateObstacles )
    {
        const ROUTING_OBSTACLE& obstacle = m_board.obstacles[obstacleIndex];
        const bool sameNetPadHole =
                obstacle.isHole && obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT
                && endpointRadius( aNetCode, obstacle.start ) >= 0;
        const bool ownPadHole = sameNetPadHole && obstacle.start == aPoint;
        // KiCad permits same-net copper to pass over the drill of an already
        // placed same-net via.  Keep the drill as a real obstacle for foreign
        // nets and for new vias (hole-to-hole clearance), but do not make an
        // existing routed via strand an artificial wall for same-net tracks.
        const bool existingSameNetViaHole =
                !aForVia && obstacle.isHole && obstacle.isExistingRoute
                && !obstacle.boardItemId.empty();
        if( obstacle.netCode != 0 && obstacle.netCode == aNetCode && !obstacle.isKeepout
            && ( !obstacle.isHole || ( !aForVia && ownPadHole ) || existingSameNetViaHole ) )
            continue;

        if( aForVia && !obstacle.blocksVias )
            continue;

        if( !aForVia && !obstacle.blocksTracks )
            continue;

        const std::int64_t radius = obstacleExpansionRadius(
                obstacle, aNetCode, aLayer, aForVia, geometryRadius );

        // Visibility expansion probes many obstacles that are far away from
        // the candidate point.  Reject those with an integer bounding-box
        // test before invoking the more expensive segment/polygon geometry.
        if( obstacle.kind != ROUTER_OBSTACLE_KIND::RECTANGLE
            && !boxContainsInflated( m_obstacleBounds[obstacleIndex], aPoint, radius ) )
        {
            continue;
        }

        if( obstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
        {
            ROUTER_BOX box = obstacle.box;
            box.minX -= radius;
            box.minY -= radius;
            box.maxX += radius;
            box.maxY += radius;

            if( box.Contains( aPoint ) )
                return false;
        }
        else if( obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
        {
            if( pointToSegmentDistance( aPoint, obstacle.start, obstacle.end ) <= radius )
                return false;
        }
        else if( pointInPolygonWithHoles( aPoint, obstacle.polygon, obstacle.polygonHoles )
                 || pointNearPolygonWithHoles( aPoint, obstacle.polygon,
                                               obstacle.polygonHoles, radius ) )
        {
            return false;
        }
    }

    // A normal track point is always reached through a segment predicate,
    // which performs the exact occupancy intersection test below.  Repeating
    // the same full-route scan from isPointAllowed for every A* endpoint made
    // dense boards spend most of their time checking the same committed
    // segments twice.  Standalone via locations still need the point-level
    // test because a via occupies a stack without an intervening track edge.
    // Same-net copper sharing never waives spacing between distinct drills.
    // This check also applies during negotiated retries, since a net cannot
    // rip up its own supporting copper. Exact reuse of a worker via is
    // materialized as one via by the insertion stage.
    if( aForVia )
    {
        for( const ROUTING_CONNECTION& connection : m_occupancy.Connections() )
        {
            if( connection.netCode != aNetCode )
                continue;
            for( std::size_t index = 1; index < connection.nodes.size(); ++index )
            {
                const ROUTER_NODE& first = connection.nodes[index - 1];
                const ROUTER_NODE& second = connection.nodes[index];
                if( first.layer == second.layer || first.point == aPoint )
                    continue;
                if( distance( aPoint, first.point ) < 2 * netViaDrillRadius( aNetCode )
                                                        + m_board.holeToHoleClearance )
                    return false;
            }
        }
    }

    if( aForVia && !m_allowRipupOccupancy )
    {
        for( const ROUTING_CONNECTION& connection : m_occupancy.Connections() )
        {
            if( connection.netCode == aNetCode )
                continue;

            const std::int64_t currentRadius = aForVia ? netViaRadius( aNetCode )
                                                        : netTrackRadius( aNetCode );

            for( std::size_t i = 1; i < connection.nodes.size(); ++i )
            {
                const ROUTER_NODE& previous = connection.nodes[i - 1];
                const ROUTER_NODE& current = connection.nodes[i];

                if( previous.layer == current.layer )
                {
                    if( previous.layer != aLayer )
                        continue;

                    if( pointToSegmentDistance( aPoint, previous.point, current.point )
                        <= currentRadius + netTrackRadius( connection.netCode )
                                   + pairClearance( aNetCode, connection.netCode, aLayer ) )
                    {
                        return false;
                    }
                }
                else if( distance( aPoint, previous.point )
                         <= ( aForVia
                                      ? std::max( currentRadius + netViaRadius( connection.netCode )
                                                          + pairClearance( aNetCode,
                                                                           connection.netCode,
                                                                           aLayer ),
                                                  netViaDrillRadius( aNetCode )
                                                          + netViaDrillRadius( connection.netCode )
                                                          + m_board.holeToHoleClearance )
                                      : currentRadius + netViaRadius( connection.netCode )
                                                + pairClearance( aNetCode, connection.netCode,
                                                                 aLayer ) ) )
                {
                    return false;
                }
            }
        }
    }

    return true;
}


bool MAZE_SEARCH_ENGINE::isSegmentAllowed( const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
                                            int aLayer, int aNetCode, bool aForVia,
                                            std::int64_t aStartRadius,
                                            std::int64_t aEndRadius ) const
{
    if( !isPointAllowed( aStart, aLayer, aNetCode, aForVia, aStartRadius ) )
        return false;

    return isSegmentAllowedFromKnownStart( aStart, aEnd, aLayer, aNetCode, aForVia,
                                           aEndRadius );
}


bool MAZE_SEARCH_ENGINE::isSegmentAllowedFromKnownStart(
        const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd, int aLayer, int aNetCode,
        bool aForVia, std::int64_t aEndRadius ) const
{
    if( autorouterDebugEnabled() )
        ++m_debugSegmentChecks;

    if( !isPointAllowed( aEnd, aLayer, aNetCode, aForVia, aEndRadius ) )
        return false;

    const std::int64_t defaultRadius = aForVia ? netViaRadius( aNetCode )
                                                : netTrackRadius( aNetCode );

    // Endpoints are checked above.  For the segment itself, a straight line
    // can leave a concave outline (or graze a board hole) between legal
    // endpoints.  Segment-to-segment distance is both exact for this case and
    // substantially cheaper than sampling every half-grid cell of a long
    // visibility edge.
    const double boundaryClearance = static_cast<double>( m_board.edgeClearance
                                                          + defaultRadius );
    const auto nearBoundary = [&]( const std::vector<ROUTER_POINT>& aPolygon )
    {
        for( std::size_t i = 0; i < aPolygon.size(); ++i )
        {
            if( segmentsWithinClearance( aStart, aEnd, aPolygon[i],
                                          aPolygon[( i + 1 ) % aPolygon.size()],
                                          boundaryClearance ) )
            {
                return true;
            }
        }

        return false;
    };

    if( !m_board.boardOutline.empty() && nearBoundary( m_board.boardOutline ) )
        return false;

    for( const std::vector<ROUTER_POINT>& hole : m_board.boardHoles )
    {
        if( nearBoundary( hole ) )
            return false;
    }

    const ROUTER_BOX routeBounds{ std::min( aStart.x, aEnd.x ),
                                  std::min( aStart.y, aEnd.y ),
                                  std::max( aStart.x, aEnd.x ),
                                  std::max( aStart.y, aEnd.y ) };
    std::vector<std::size_t> candidateObstacles;
    collectObstacleIndices( aLayer, routeBounds, candidateObstacles );

    for( std::size_t obstacleIndex : candidateObstacles )
    {
        const ROUTING_OBSTACLE& obstacle = m_board.obstacles[obstacleIndex];
        // A same-net through-hole pad is an electrical connection surface, so
        // a trace may pass through its centre.  A same-net drill that is not
        // represented by a pad remains a manufacturing obstacle.
        const bool ownPadHole = endpointRadius( aNetCode, obstacle.start ) >= 0
                                && obstacle.isHole
                                && obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT
                                && ( obstacle.start == aStart || obstacle.start == aEnd );
        const bool existingSameNetViaHole =
                !aForVia && obstacle.isHole && obstacle.isExistingRoute
                && !obstacle.boardItemId.empty();
        if( obstacle.netCode != 0 && obstacle.netCode == aNetCode && !obstacle.isKeepout
            && ( !obstacle.isHole || ( !aForVia && ownPadHole ) || existingSameNetViaHole ) )
            continue;

        if( aForVia ? !obstacle.blocksVias : !obstacle.blocksTracks )
            continue;

        const std::int64_t radius =
                obstacleExpansionRadius( obstacle, aNetCode, aLayer, aForVia );

        if( !boxesOverlap( routeBounds, m_obstacleBounds[obstacleIndex], radius ) )
            continue;

        if( obstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
        {
            ROUTER_BOX box = obstacle.box;
            box.minX -= radius;
            box.minY -= radius;
            box.maxX += radius;
            box.maxY += radius;

            const ROUTER_POINT topLeft{ box.minX, box.minY };
            const ROUTER_POINT topRight{ box.maxX, box.minY };
            const ROUTER_POINT bottomRight{ box.maxX, box.maxY };
            const ROUTER_POINT bottomLeft{ box.minX, box.maxY };
            const bool startInside = box.Contains( aStart );
            const bool endInside = box.Contains( aEnd );
            if( startInside || endInside
                || segmentsIntersect( aStart, aEnd, topLeft, topRight )
                || segmentsIntersect( aStart, aEnd, topRight, bottomRight )
                || segmentsIntersect( aStart, aEnd, bottomRight, bottomLeft )
                || segmentsIntersect( aStart, aEnd, bottomLeft, topLeft ) )
            {
                return false;
            }
        }
        else if( obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
        {
            const bool startNear = pointToSegmentDistance( aStart, obstacle.start,
                                                            obstacle.end ) <= radius;
            const bool endNear = pointToSegmentDistance( aEnd, obstacle.start, obstacle.end )
                                 <= radius;
            if( startNear || endNear
                || segmentsIntersect( aStart, aEnd, obstacle.start, obstacle.end )
                || pointToSegmentDistance( obstacle.start, aStart, aEnd ) <= radius
                || pointToSegmentDistance( obstacle.end, aStart, aEnd ) <= radius )
            {
                return false;
            }
        }
        else
        {
            if( pointInPolygonWithHoles( aStart, obstacle.polygon, obstacle.polygonHoles )
                || pointInPolygonWithHoles( aEnd, obstacle.polygon, obstacle.polygonHoles ) )
            {
                return false;
            }

            // A polygon hole is legal free space.  Testing its boundary as a
            // solid obstacle prevents a route whose endpoints are both inside
            // a pad/keepout opening from traversing that opening.  The outer
            // contour remains a hard boundary; samples below detect a segment
            // that crosses solid material between two holes.
            const auto intersectsOuterPolygon =
                    [&]( const std::vector<ROUTER_POINT>& aPolygon )
            {
                for( std::size_t i = 0; i < aPolygon.size(); ++i )
                {
                    const ROUTER_POINT& start = aPolygon[i];
                    const ROUTER_POINT& end = aPolygon[( i + 1 ) % aPolygon.size()];

                    if( segmentsIntersect( aStart, aEnd, start, end )
                        || pointToSegmentDistance( aStart, start, end ) <= radius
                        || pointToSegmentDistance( aEnd, start, end ) <= radius
                        || pointToSegmentDistance( start, aStart, aEnd ) <= radius
                        || pointToSegmentDistance( end, aStart, aEnd ) <= radius )
                    {
                        return true;
                    }
                }

                return false;
            };

            if( intersectsOuterPolygon( obstacle.polygon ) )
            {
                return false;
            }

            const std::int64_t sampleStep = std::max<std::int64_t>( 1, m_activeGridStep / 2 );
            const int polygonSampleCount = std::min(
                    10000,
                    std::max( 1, static_cast<int>( std::ceil(
                                             distance( aStart, aEnd ) / sampleStep ) ) ) );

            for( int index = 0; index <= polygonSampleCount; ++index )
            {
                const double ratio = static_cast<double>( index ) / polygonSampleCount;
                const ROUTER_POINT sample{
                    static_cast<std::int64_t>(
                            std::llround( aStart.x + ( aEnd.x - aStart.x ) * ratio ) ),
                    static_cast<std::int64_t>(
                            std::llround( aStart.y + ( aEnd.y - aStart.y ) * ratio ) ) };

                if( pointInPolygonWithHoles( sample, obstacle.polygon, obstacle.polygonHoles )
                    || pointNearPolygonWithHoles( sample, obstacle.polygon,
                                                   obstacle.polygonHoles, radius ) )
                {
                    return false;
                }
            }
        }
    }

    if( !m_allowRipupOccupancy )
    {
        for( const ROUTER_CELL_KEY& cell : m_occupancy.CellsForSegment( { aStart, aLayer },
                                                                          { aEnd, aLayer } ) )
        {
            if( m_occupancy.Usage( cell, aNetCode ) > 0 )
                return false;
        }

        const std::int64_t currentRadius = aForVia ? netViaRadius( aNetCode )
                                                    : netTrackRadius( aNetCode );

        for( const ROUTING_CONNECTION& connection : m_occupancy.Connections() )
        {
            if( connection.netCode == aNetCode )
                continue;

            for( std::size_t i = 1; i < connection.nodes.size(); ++i )
            {
                const ROUTER_NODE& previous = connection.nodes[i - 1];
                const ROUTER_NODE& current = connection.nodes[i];
                const bool otherIsVia = previous.layer != current.layer;
                const std::int64_t copperClearance =
                        currentRadius
                        + ( otherIsVia ? netViaRadius( connection.netCode )
                                       : netTrackRadius( connection.netCode ) )
                        + pairClearance( aNetCode, connection.netCode, aLayer );
                const std::int64_t clearance =
                        aForVia && otherIsVia
                                ? std::max( copperClearance,
                                            netViaDrillRadius( aNetCode )
                                                    + netViaDrillRadius( connection.netCode )
                                                    + m_board.holeToHoleClearance )
                                : copperClearance;

                if( previous.layer == current.layer )
                {
                    if( previous.layer != aLayer )
                        continue;

                    if( segmentsIntersect( aStart, aEnd, previous.point, current.point )
                        || pointToSegmentDistance( aStart, previous.point, current.point )
                                   <= clearance
                        || pointToSegmentDistance( aEnd, previous.point, current.point )
                                   <= clearance
                        || pointToSegmentDistance( previous.point, aStart, aEnd ) <= clearance
                        || pointToSegmentDistance( current.point, aStart, aEnd ) <= clearance )
                    {
                        return false;
                    }
                }
                else if( pointToSegmentDistance( previous.point, aStart, aEnd ) <= clearance )
                {
                    return false;
                }
            }
        }
    }

    return true;
}


bool MAZE_SEARCH_ENGINE::CanUseSegment( int aNetCode, const ROUTER_NODE& aStart,
                                        const ROUTER_NODE& aEnd, bool aForVia ) const
{
    if( aStart.layer != aEnd.layer )
    {
        if( aStart.point != aEnd.point
            || !VIA_RULE::AllowsTransition( m_settings, aStart.layer, aEnd.layer ) )
            return false;

        for( const auto& layer : m_settings.layers )
        {
            if( !isPointAllowed( aStart.point, layer.layerId, aNetCode, true ) )
                return false;
        }

        return true;
    }

    return isSegmentAllowed( aStart.point, aEnd.point, aStart.layer, aNetCode, aForVia,
                             endpointRadius( aNetCode, aStart.point ),
                             endpointRadius( aNetCode, aEnd.point ) );
}


bool MAZE_SEARCH_ENGINE::CanInsertSegment( int net, const ROUTER_NODE& start,
                                            const ROUTER_NODE& end ) const
{
    struct RESTORE
    {
        bool& flag;
        bool value;
        ~RESTORE() { flag = value; }
    } restore{ m_allowRipupOccupancy, m_allowRipupOccupancy };
    m_allowRipupOccupancy = false;
    if( start.layer != end.layer )
        return CanUseSegment( net, start, end );
    const auto radius = netTrackRadius( net );
    return isSegmentAllowed( start.point, end.point, start.layer, net, false, radius, radius );
}


std::vector<ROUTING_CONNECTION> MAZE_SEARCH_ENGINE::FindConflictingConnections(
        const ROUTING_CONNECTION& aCandidate ) const
{
    std::vector<ROUTING_CONNECTION> result;


    const auto edgeConflicts = [&]( const ROUTER_NODE& aLeftStart,
                                    const ROUTER_NODE& aLeftEnd,
                                    int aLeftNetCode,
                                    const ROUTER_NODE& aRightStart,
                                    const ROUTER_NODE& aRightEnd,
                                    int aRightNetCode )
    {
        const bool leftVia = aLeftStart.layer != aLeftEnd.layer;
        const bool rightVia = aRightStart.layer != aRightEnd.layer;

        if( !leftVia && !rightVia )
        {
            if( aLeftStart.layer != aRightStart.layer )
                return false;

            const std::int64_t clearance = netTrackRadius( aLeftNetCode )
                                           + netTrackRadius( aRightNetCode )
                                           + pairClearance( aLeftNetCode, aRightNetCode,
                                                            aLeftStart.layer );
            return segmentsWithinClearance( aLeftStart.point, aLeftEnd.point,
                                            aRightStart.point, aRightEnd.point,
                                            static_cast<double>( clearance ) );
        }

        if( leftVia && rightVia )
        {
            const std::int64_t copperClearance = netViaRadius( aLeftNetCode )
                                                 + netViaRadius( aRightNetCode )
                                                 + pairClearance( aLeftNetCode, aRightNetCode );
            const std::int64_t drillClearance = netViaDrillRadius( aLeftNetCode )
                                                + netViaDrillRadius( aRightNetCode )
                                                + m_board.holeToHoleClearance;
            return distance( aLeftStart.point, aRightStart.point )
                   <= static_cast<double>( std::max( copperClearance, drillClearance ) );
        }

        const ROUTER_NODE& viaStart = leftVia ? aLeftStart : aRightStart;
        const int viaNetCode = leftVia ? aLeftNetCode : aRightNetCode;
        const ROUTER_NODE& trackStart = leftVia ? aRightStart : aLeftStart;
        const ROUTER_NODE& trackEnd = leftVia ? aRightEnd : aLeftEnd;
        const int trackNetCode = leftVia ? aRightNetCode : aLeftNetCode;

        const std::int64_t clearance = netViaRadius( viaNetCode )
                                       + netTrackRadius( trackNetCode )
                                       + pairClearance( viaNetCode, trackNetCode,
                                                        trackStart.layer );
        return pointToSegmentDistance( viaStart.point, trackStart.point, trackEnd.point )
               <= static_cast<double>( clearance );
    };

    for( const ROUTING_CONNECTION& existing : m_occupancy.Connections() )
    {
        if( existing.netCode == aCandidate.netCode )
            continue;

        bool conflicts = false;
        for( std::size_t left = 1; left < aCandidate.nodes.size() && !conflicts; ++left )
        {
            for( std::size_t right = 1; right < existing.nodes.size(); ++right )
            {
                if( edgeConflicts( aCandidate.nodes[left - 1], aCandidate.nodes[left],
                                   aCandidate.netCode, existing.nodes[right - 1],
                                   existing.nodes[right], existing.netCode ) )
                {
                    conflicts = true;
                    break;
                }
            }
        }

        if( conflicts )
            result.push_back( existing );
    }

    return result;
}


std::vector<ROUTER_NODE> MAZE_SEARCH_ENGINE::neighbours( const ROUTER_NODE& aNode ) const
{
    return MAZE_EXPANSION_ENGINE::Neighbours( aNode, m_activeGridStep,
                                              m_settings.layers, m_settings.allowVias );
}


std::vector<ROUTER_NODE> MAZE_SEARCH_ENGINE::buildLandmarks( const ROUTING_PAD& aStart,
                                                             const ROUTING_PAD& aTarget,
                                                             int aNetCode ) const
{
    std::vector<ROUTER_NODE> result;
    result.reserve( m_baseLandmarks.size() + aStart.layers.size() + aTarget.layers.size() );

    auto add = [&]( const ROUTER_POINT& aPoint, int aLayer )
    {
        if( aLayer < 0 )
            return;

        const ROUTER_NODE node{ aPoint, aLayer };
        if( std::find( result.begin(), result.end(), node ) == result.end() )
            result.push_back( node );
    };

    // Terminal nodes must be present even when the bounded board-wide set is
    // full.  This is also what makes a clear-board route take the direct
    // visibility edge rather than falling back to the raster grid.
    for( int layer : aStart.layers )
        add( aStart.position, layer );
    for( int layer : aTarget.layers )
        add( aTarget.position, layer );

    // A plane fanout target is deliberately represented on the destination
    // layer, but its via must be reached from the source layer first.  Add a
    // source-layer landmark at the same physical point so the search can make
    // one legal visibility move followed by a via transition there.  This is
    // the native equivalent of Freerouting's escaped SMD fanout stub.
    if( aTarget.isFanoutTarget && aTarget.fanoutSourceLayer >= 0 )
        add( aTarget.position, aTarget.fanoutSourceLayer );

    // Keep the room/door visibility set large enough for a full board while
    // leaving the per-frontier visibility cap below as the runtime bound.
    // Freerouting's expansion graph is board-wide; a 768-entry prefix is too
    // spatially biased on dense boards with many pads and copper fragments.
    constexpr std::size_t maxLandmarks = 2048;
    for( const ROUTER_NODE& landmark : m_baseLandmarks )
    {
        if( result.size() >= maxLandmarks )
            break;

        if( std::find( result.begin(), result.end(), landmark ) == result.end() )
            result.push_back( landmark );
    }

    (void) aNetCode;
    return result;
}

std::vector<ROUTER_NODE> MAZE_SEARCH_ENGINE::adaptiveNeighbours(
        const ROUTER_NODE& aNode, const ROUTING_PAD& aTarget,
        const std::vector<ROUTER_NODE>& aLandmarks, int aNetCode ) const
{
    std::vector<ROUTER_NODE> result = neighbours( aNode );
    std::vector<std::pair<long double, ROUTER_NODE>> candidates;
    candidates.reserve( std::min<std::size_t>( aLandmarks.size(), 128 ) );

    auto collectNearbyLandmarks = [&]( int aCellRadius )
    {
        const std::int64_t centerX = floorDivide( aNode.point.x, m_obstacleBucketSize );
        const std::int64_t centerY = floorDivide( aNode.point.y, m_obstacleBucketSize );

        for( int offsetX = -aCellRadius; offsetX <= aCellRadius; ++offsetX )
        {
            for( int offsetY = -aCellRadius; offsetY <= aCellRadius; ++offsetY )
            {
                const ROUTER_CELL_KEY key{ centerX + offsetX, centerY + offsetY,
                                           aNode.layer };
                const auto bucket = m_landmarksBySpatialCell.find( key );
                if( bucket == m_landmarksBySpatialCell.end() )
                    continue;

                for( std::size_t landmarkIndex : bucket->second )
                {
                    if( landmarkIndex >= m_baseLandmarks.size() )
                        continue;

                    const ROUTER_NODE& landmark = m_baseLandmarks[landmarkIndex];
                    if( landmark == aNode )
                        continue;

                    const long double dx = static_cast<long double>( landmark.point.x )
                                           - aNode.point.x;
                    const long double dy = static_cast<long double>( landmark.point.y )
                                           - aNode.point.y;
                    const long double distanceSquared = dx * dx + dy * dy;

                    if( distanceSquared != 0.0L )
                        candidates.emplace_back( distanceSquared, landmark );
                }
            }
        }
    };

    // A nearby room normally contains all useful escape doors.  If a
    // frontier node is in a sparse part of the graph, widen the lookup once
    // rather than falling back to an O(landmarks) scan.  The exact target is
    // still considered below, so long clear-board routes do not depend on
    // the bucket radius.
    collectNearbyLandmarks( 2 );
    if( candidates.size() < 8 )
        collectNearbyLandmarks( 8 );

    auto compareCandidates = []( const auto& aLeft, const auto& aRight )
    {
        if( aLeft.first != aRight.first )
            return aLeft.first < aRight.first;
        if( aLeft.second.point.x != aRight.second.point.x )
            return aLeft.second.point.x < aRight.second.point.x;
        if( aLeft.second.point.y != aRight.second.point.y )
            return aLeft.second.point.y < aRight.second.point.y;
        return aLeft.second.layer < aRight.second.layer;
    };

    // Freerouting's room graph presents only nearby door sections to a
    // frontier element.  The native visibility fallback has the same bounded
    // responsibility: keep enough nearest landmarks to escape a local room,
    // but do not sort and test every board landmark for every A* cell.  The
    // regular grid neighbours remain the complete fallback when no landmark
    // in this bounded set is visible.
    // Dense boards tend to have thousands of filled-zone fragments.  A room
    // frontier still needs a few escape landmarks, but testing the complete
    // visibility fan against every such fragment is disproportionate once
    // the orthogonal grid already supplies the local fallback.  Keep the
    // richer set for ordinary boards and use a bounded dense-board set so a
    // difficult connection remains cancellable in the editor.
    const bool denseBoard = m_board.obstacles.size() > 2000 || m_board.pads.size() > 300;
    const std::size_t maxVisibilityCandidates = denseBoard ? 16 : 128;
    if( candidates.size() > maxVisibilityCandidates )
    {
        std::nth_element( candidates.begin(),
                          candidates.begin()
                                  + static_cast<std::ptrdiff_t>( maxVisibilityCandidates ),
                          candidates.end(), compareCandidates );
        candidates.resize( maxVisibilityCandidates );
    }

    std::stable_sort( candidates.begin(), candidates.end(), compareCandidates );

    const std::size_t maxVisibleLandmarks = denseBoard ? 2 : 24;
    std::size_t visible = 0;

    for( const auto& [unusedDistance, candidate] : candidates )
    {
        (void) unusedDistance;

        if( visible >= maxVisibleLandmarks )
            break;

        // The endpoint is a copper pad, not a moving piece of track.  Its
        // own copper radius is already represented by the same-net obstacle
        // that is skipped below; inflating a foreign obstacle by the whole
        // pad radius here can incorrectly make a legal SMD escape have no
        // legal start point.  Only the routed trace width belongs in this
        // visibility probe.
        const std::int64_t candidateRadius = candidate.point == aTarget.position
                                                     ? netTrackRadius( aNetCode )
                                                     : -1;
        if( !isSegmentAllowedFromKnownStart( aNode.point, candidate.point, aNode.layer,
                                             aNetCode, false, candidateRadius ) )
            continue;

        if( std::find( result.begin(), result.end(), candidate ) == result.end() )
        {
            result.push_back( candidate );
            ++visible;
        }
    }

    // The exact target is a special landmark even when it is farther than the
    // local visibility radius.  This fast path preserves straight traces on
    // clear boards and removes an unnecessary dependence on raster alignment.
    for( int layer : { aNode.layer } )
    {
        const ROUTER_NODE target{ aTarget.position, layer };
        if( target != aNode
            && isSegmentAllowedFromKnownStart( aNode.point, target.point, layer, aNetCode,
                                               false, netTrackRadius( aNetCode ) )
            && std::find( result.begin(), result.end(), target ) == result.end() )
        {
            result.push_back( target );
        }
    }

    return result;
}


double MAZE_SEARCH_ENGINE::heuristic( const ROUTER_NODE& aNode, const ROUTER_POINT& aTarget,
                                      int aTargetLayer, const AUTOROUTE_CONTROL& aControl ) const
{
    // All destination shapes contribute to the lower bound. A distance to
    // one arbitrarily selected pad can overestimate the cost to another
    // member of the destination set and bias the frontier away from it.
    (void) aTarget;
    (void) aTargetLayer;
    (void) aControl;
    return m_legacyDestinationDistance.Calculate( aNode.point, aNode.layer );
}


bool MAZE_SEARCH_ENGINE::canFinish( const ROUTER_NODE& aNode, const ROUTING_PAD& aTarget,
                                    int aNetCode ) const
{
    if( !isOnPadLayer( aTarget, aNode.layer ) )
        return false;

    const double maxFinalDistance = std::max<double>( m_activeGridStep * 1.5, 1.0 );

    return distance( aNode.point, aTarget.position ) <= maxFinalDistance
           && isSegmentAllowedFromKnownStart( aNode.point, aTarget.position, aNode.layer,
                                              aNetCode, false, netTrackRadius( aNetCode ) );
}


std::optional<ROUTING_CONNECTION>
MAZE_SEARCH_ENGINE::FindConnection( const ROUTING_PAD& aStart, const ROUTING_PAD& aTarget,
                                    int aRetry, int& aExpandedNodes,
                                    const ROUTER_CANCEL_CALLBACK& aCancel,
                                    const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress,
                                    const std::vector<ROUTING_TERMINAL>& aStarts,
                                    const std::vector<ROUTING_TERMINAL>& aTargets ) const
{
    aExpandedNodes = 0;
    m_roomMetrics = {};
    const std::vector<ROUTING_TERMINAL> starts = aStarts.empty()
            ? std::vector<ROUTING_TERMINAL>{ { aStart } } : aStarts;
    const std::vector<ROUTING_TERMINAL> targets = aTargets.empty()
            ? std::vector<ROUTING_TERMINAL>{ { aTarget } } : aTargets;
    for( const auto& terminal : starts )
        if( terminal.pad.netCode != aStart.netCode )
            return std::nullopt;
    for( const auto& terminal : targets )
        if( terminal.pad.netCode != aStart.netCode )
            return std::nullopt;
    m_allowRipupOccupancy = m_settings.allowRipupRouted && aRetry > 0;
    const bool debug = autorouterDebugEnabled();
    m_debugObstacleQueries = 0;
    m_debugObstacleCandidates = 0;
    m_debugPointChecks = 0;
    m_debugSegmentChecks = 0;

    const auto searchStarted = std::chrono::steady_clock::now();

    if( debug )
    {
        std::ostringstream message;
        message << "BEGIN search net=" << aStart.netCode << " start=(" << aStart.position.x
                << ',' << aStart.position.y << ") target=(" << aTarget.position.x << ','
                << aTarget.position.y << ") retry=" << aRetry
                << " maxExpanded=" << m_settings.maxExpandedNodes;
        autorouterDebugLog( message.str() );
    }

    const std::int64_t baseStep = std::max<std::int64_t>( 1, m_settings.gridStepIU );
    const int refinement = std::min( 3, std::max( 0, aRetry ) );
    const std::int64_t refinedStep = std::max<std::int64_t>( 1, baseStep >> refinement );
    m_activeGridStep = std::min( baseStep, std::max<std::int64_t>( 50000, refinedStep ) );

    // Do not silently substitute a smaller budget on dense boards. The
    // caller's explicit budget is the limit reported in diagnostics/UI.
    const int effectiveMaxExpandedNodes = std::max( 0, m_settings.maxExpandedNodes );
    m_legacyDestinationDistance.Configure( m_settings, targets.front().pad );
    for( const auto& terminal : targets )
    {
        const ROUTER_POINT& point = terminal.pad.position;
        const auto end = terminal.segmentEnd.value_or( point );
        for( int layer : terminal.pad.layers )
            m_legacyDestinationDistance.Join( { std::min( point.x, end.x ), std::min( point.y, end.y ),
                                         std::max( point.x, end.x ), std::max( point.y, end.y ) }, layer );
    }
    AUTOROUTE_CONTROL control( m_settings, aStart.netCode, aRetry,
                               aTarget.isPlaneTarget );

    // Plane fanout has an intentionally simple first-stage topology: a short
    // source-layer stub ending at the synthetic landing point, followed by a
    // via at that point.  Try that direct visibility construction before the
    // general maze frontier.  It both preserves the Freerouting fanout shape
    // and avoids spending the whole expansion budget rediscovering a path that
    // the visibility graph already describes.  If any exact snapshot rule
    // rejects the stub or via, the normal search remains the fallback.
    if( aStarts.empty() && aTargets.empty()
        && aTarget.isFanoutTarget && aTarget.fanoutSourceLayer >= 0
        && aTarget.fanoutTargetLayer >= 0
        && isOnPadLayer( aStart, aTarget.fanoutSourceLayer )
        && isOnPadLayer( aTarget, aTarget.fanoutTargetLayer ) )
    {
        const ROUTER_NODE source{ aStart.position, aTarget.fanoutSourceLayer };
        const ROUTER_NODE landing{ aTarget.position, aTarget.fanoutSourceLayer };
        const ROUTER_NODE destination{ aTarget.position, aTarget.fanoutTargetLayer };
        const bool viaAllowed = CanUseSegment( aStart.netCode, landing, destination, true );

        const bool stubAllowed = isSegmentAllowed(
                source.point, landing.point, source.layer, aStart.netCode, false,
                netTrackRadius( aStart.netCode ), -1 );
        if( viaAllowed && stubAllowed )
        {
            ROUTING_CONNECTION direct;
            direct.netCode = aStart.netCode;
            direct.complete = true;
            direct.cost = control.TraceCost( distance( source.point, landing.point ) )
                          + control.ViaCost();
            direct.nodes = { source, landing, destination };

            if( debug )
                autorouterDebugLog( "END search via direct fanout path" );

            return direct;
        }
    }

    // The batch net walk may reach an escaped fanout landing from the rest of
    // the net before it reaches the original SMD pad.  In that orientation the
    // synthetic landing is the search source rather than the target.  Treat
    // the same two-stage fanout as reversible so a short, already-certified
    // fanout stub is not sent through the full-board maze search (where the
    // layer transition at the landing can consume the entire expansion
    // budget).
    if( aStarts.empty() && aTargets.empty()
        && aStart.isFanoutTarget && aStart.fanoutSourceLayer >= 0
        && aStart.fanoutTargetLayer >= 0
        && isOnPadLayer( aStart, aStart.fanoutTargetLayer )
        && isOnPadLayer( aTarget, aStart.fanoutSourceLayer ) )
    {
        const ROUTER_NODE source{ aStart.position, aStart.fanoutTargetLayer };
        const ROUTER_NODE landing{ aStart.position, aStart.fanoutSourceLayer };
        const ROUTER_NODE destination{ aTarget.position, aStart.fanoutSourceLayer };
        const bool viaAllowed = CanUseSegment( aStart.netCode, source, landing, true );

        const bool stubAllowed = isSegmentAllowed(
                landing.point, destination.point, destination.layer, aStart.netCode, false,
                -1, netTrackRadius( aStart.netCode ) );
        if( viaAllowed && stubAllowed )
        {
            ROUTING_CONNECTION direct;
            direct.netCode = aStart.netCode;
            direct.complete = true;
            direct.cost = control.TraceCost( distance( landing.point, destination.point ) )
                          + control.ViaCost();
            direct.nodes = { source, landing, destination };

            if( debug )
                autorouterDebugLog( "END search via reversed direct fanout path" );

            return direct;
        }
    }

    // The rectangular room frontier now compares same-layer routes and
    // through-drill alternatives in one queue. Unsupported/rejected geometry
    // retains the legacy fallback with the same work budget and cancellation.
    const auto enabledLayers = std::count_if( m_settings.layers.begin(), m_settings.layers.end(),
                                             []( const auto& layer ) { return layer.enabled; } );
    const auto roomPath = !m_settings.allowVias || enabledLayers == 1
            ? findRoomConnection( starts, targets, aRetry, aExpandedNodes, aCancel, aProgress )
            : findMultilayerRoomConnection( starts, targets, aRetry, aExpandedNodes, aCancel, aProgress );
    if( roomPath )
        return roomPath;
    if( aCancel && aCancel() )
        return std::nullopt;

    std::priority_queue<OPEN_NODE, std::vector<OPEN_NODE>, OPEN_NODE_COMPARE> open;
    std::unordered_map<ROUTER_NODE, double, NODE_KEY_HASH> bestCost;
    std::unordered_map<ROUTER_NODE, ROUTER_NODE, NODE_KEY_HASH> cameFrom;
    const std::vector<ROUTER_NODE> landmarks = buildLandmarks( aStart, aTarget,
                                                                 aStart.netCode );
    std::size_t sequence = 0;
    std::unordered_map<ROUTER_NODE, std::size_t, NODE_KEY_HASH> startOwners;

    for( const ROUTING_TERMINAL& terminal : starts )
    {
        std::vector<ROUTER_POINT> seeds{ terminal.pad.position };
        if( terminal.segmentEnd )
        {
            seeds.push_back( *terminal.segmentEnd );
            for( const auto& target : targets )
            {
                seeds.push_back( terminalPoint( terminal, target.pad.position ) );
                if( target.segmentEnd )
                    seeds.push_back( terminalPoint( terminal, *target.segmentEnd ) );
            }
        }
        for( const auto& point : seeds )
        {
            for( const ROUTER_LAYER_SETTINGS& layer : m_settings.layers )
            {
                if( !layer.enabled || !isOnPadLayer( terminal.pad, layer.layerId )
                    || !isPointAllowed( point, layer.layerId, aStart.netCode, false,
                                       netTrackRadius( aStart.netCode ) ) )
                    continue;
                ROUTER_NODE start{ point, layer.layerId };
                if( bestCost.contains( start ) )
                    continue;
                const double h = heuristic( start, aTarget.position, layer.layerId, control );
                open.push( { start, 0.0, h, sequence++ } );
                bestCost[start] = 0.0;
                startOwners[start] = terminal.padIndex;
            }
        }
    }

    auto logSearchState = [&]( const char* aState )
    {
        if( !debug )
            return;

        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - searchStarted )
                                      .count();
        std::ostringstream message;
        message << aState << " search net=" << aStart.netCode << " expanded=" << aExpandedNodes
                << " open=" << open.size() << " best=" << bestCost.size()
                << " obstacleQueries=" << m_debugObstacleQueries
                << " obstacleCandidates=" << m_debugObstacleCandidates
                << " pointChecks=" << m_debugPointChecks
                << " segmentChecks=" << m_debugSegmentChecks << " elapsed=" << elapsed << " ms";
        autorouterDebugLog( message.str() );
    };

    if( open.empty() )
        logSearchState( "END search no legal start" );

    auto nextDiagnostic = searchStarted + std::chrono::seconds( 2 );

    while( !open.empty() )
    {
        if( aCancel && aCancel() )
        {
            logSearchState( "END search cancelled" );
            return std::nullopt;
        }

        if( debug && std::chrono::steady_clock::now() >= nextDiagnostic )
        {
            logSearchState( "PROGRESS" );
            nextDiagnostic = std::chrono::steady_clock::now() + std::chrono::seconds( 2 );
        }

        OPEN_NODE current = open.top();
        open.pop();

        auto bestIt = bestCost.find( current.node );

        if( bestIt == bestCost.end() || current.g > bestIt->second )
            continue;

        if( aExpandedNodes >= effectiveMaxExpandedNodes )
        {
            logSearchState( "END search expansion limit" );
            return std::nullopt;
        }
        ++aExpandedNodes;

        // A difficult connection may expand tens of thousands of nodes before
        // it either succeeds or exhausts its budget.  Report at a bounded
        // cadence so the KiCad progress dialog does not appear frozen,
        // without taking the job mutex on every A* expansion.
        if( aProgress && ( aExpandedNodes & 511 ) == 0 )
            aProgress( aExpandedNodes );

        const auto destination = std::find_if( targets.begin(), targets.end(),
                [&]( const ROUTING_TERMINAL& terminal )
                {
                    ROUTING_PAD target = terminal.pad;
                    target.position = terminalPoint( terminal, current.node.point );
                    return canFinish( current.node, target, aStart.netCode );
                } );
        if( destination != targets.end() )
        {
            ROUTING_CONNECTION result;
            result.netCode = aStart.netCode;
            result.complete = true;
            result.isPlaneConnection = destination->pad.isPlaneTarget;
            result.toPadIndex = destination->padIndex;
            const ROUTER_POINT finish = terminalPoint( *destination, current.node.point );
            result.cost = current.g + control.TraceCost( distance( current.node.point, finish ) );
            result.nodes.push_back( { finish, current.node.layer } );

            ROUTER_NODE cursor = current.node;

            while( true )
            {
                result.nodes.push_back( cursor );
                auto parentIt = cameFrom.find( cursor );

                if( parentIt == cameFrom.end() )
                    break;

                cursor = parentIt->second;
            }

            std::reverse( result.nodes.begin(), result.nodes.end() );

            result.fromPadIndex = startOwners.at( result.nodes.front() );

            // Remove duplicate layer-transition nodes and collinear points.
            std::vector<ROUTER_NODE> simplified;
            simplified.reserve( result.nodes.size() );

            for( const ROUTER_NODE& node : result.nodes )
            {
                if( simplified.empty() || node != simplified.back() )
                    simplified.push_back( node );

                while( simplified.size() >= 3 )
                {
                    const ROUTER_NODE& first = simplified[simplified.size() - 3];
                    const ROUTER_NODE& middle = simplified[simplified.size() - 2];
                    const ROUTER_NODE& last = simplified[simplified.size() - 1];

                    if( first.layer == middle.layer && middle.layer == last.layer
                        && ( middle.point.x - first.point.x ) * ( last.point.y - middle.point.y )
                                   == ( middle.point.y - first.point.y )
                                              * ( last.point.x - middle.point.x )
                        && isSegmentAllowed( first.point, last.point, first.layer, aStart.netCode,
                                             false,
                                             netTrackRadius( aStart.netCode ),
                                             netTrackRadius( aStart.netCode ) ) )
                    {
                        simplified.erase( simplified.end() - 2 );
                    }
                    else
                    {
                        break;
                    }
                }
            }

            result.nodes = std::move( simplified );

            logSearchState( "END search route found" );
            return result;
        }

        auto nextNodes = adaptiveNeighbours( current.node, aTarget, landmarks, aStart.netCode );
        for( const ROUTING_TERMINAL& terminal : targets )
        {
            if( !isOnPadLayer( terminal.pad, current.node.layer ) )
                continue;
            const ROUTER_NODE node{ terminalPoint( terminal, current.node.point ), current.node.layer };
            if( node != current.node
                && std::find( nextNodes.begin(), nextNodes.end(), node ) == nextNodes.end() )
                nextNodes.push_back( node );
        }
        for( const ROUTER_NODE& next : nextNodes )
        {
            const bool via = next.layer != current.node.layer;

            if( via )
            {
                // Match Freerouting's conservative fanout policy: a via is
                // not allowed to start in a single-layer SMD pad unless the
                // user explicitly enabled via-in-pad.  Without this guard
                // the cheaper layer-transition edge wins immediately and
                // produces a via at the pad centre instead of an escaped
                // fanout stub.
                if( !m_settings.allowViaInSmdPad
                    && std::any_of( starts.begin(), starts.end(), [&]( const auto& terminal )
                       { return terminal.pad.isSmd && terminal.pad.position == current.node.point; } ) )
                {
                    continue;
                }

                if( !CanUseSegment( aStart.netCode, current.node, next, true ) )
                    continue;
            }
            else if( !isSegmentAllowedFromKnownStart( current.node.point, next.point, next.layer,
                                                      aStart.netCode, false ) )
            {
                continue;
            }

            const double length = distance( current.node.point, next.point );
            const int usage = via ? 0
                                   : m_occupancy.SegmentUsage( current.node, next,
                                                                aStart.netCode );
            const double bend = ( !via && current.node.point != next.point )
                                        ? static_cast<double>( m_settings.bendCost )
                                        : 0.0;
            const double moveCost = via ? control.ViaCost()
                                        : control.TraceCost( length )
                                                  + control.CongestionCost( usage )
                                                  + control.DirectionCost( next.layer,
                                                                           current.node.point,
                                                                           next.point )
                                                  + bend;
            const double newCost = current.g + moveCost;

            auto nextBestIt = bestCost.find( next );

            if( nextBestIt != bestCost.end() && nextBestIt->second <= newCost )
                continue;

            bestCost[next] = newCost;
            cameFrom[next] = current.node;
            open.push( { next, newCost,
                         newCost + heuristic( next, aTarget.position, next.layer, control ),
                         sequence++ } );
        }
    }

    logSearchState( "END search exhausted" );
    return std::nullopt;
}

} // namespace KICAD_AUTOROUTER
