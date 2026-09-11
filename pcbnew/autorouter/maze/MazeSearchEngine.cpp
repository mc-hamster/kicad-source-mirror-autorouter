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
#include "../board/optimize/TraceShover.h"
#include "../geometry/planar/ContactGeometry.h"
#include "../geometry/planar/Simplex.h"
#include "../rules/ViaRule.h"

#include "../AutorouterDebug.h"
#include "../expansion/ExpansionGraph.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <tuple>
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


// Project onto an exact integral point of the existing copper centreline, not
// merely near that line and not onto the representative pad's centre.  KiCad
// stores integral coordinates and requires an actual centreline junction for
// connectivity.  Rounding x/y independently can produce a point a fraction
// of an IU off an oblique trace, which later appears as a dangling route.
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
    const std::int64_t integralDx = end.x - start.x;
    const std::int64_t integralDy = end.y - start.y;
    const std::int64_t latticeSteps = std::gcd( std::llabs( integralDx ),
                                                std::llabs( integralDy ) );
    if( latticeSteps == 0 )
        return start;

    const std::int64_t step = std::clamp<std::int64_t>(
            std::llround( t * static_cast<long double>( latticeSteps ) ),
            0, latticeSteps );
    return { start.x + integralDx / latticeSteps * step,
             start.y + integralDy / latticeSteps * step };
}


double distance( const ROUTER_POINT& aLeft, const ROUTER_POINT& aRight )
{
    return std::sqrt( static_cast<double>( squaredDistance( aLeft, aRight ) ) );
}


std::optional<PLANAR::SIMPLEX> exactConvexClearanceShape(
        const ROUTING_OBSTACLE& aObstacle, std::int64_t aRadius )
{
    // Source TileShape routing works directly on convex support lines.  Keep
    // the rational representation for a non-rounded, hole-free snapshot
    // contour rather than falling back to floating samples or its enclosing
    // rectangle. Rounded and holed contours retain their dedicated generic
    // geometry paths until their arc/decomposition semantics are ported.
    if( aObstacle.kind != ROUTER_OBSTACLE_KIND::POLYGON || aObstacle.radius != 0
        || !aObstacle.polygonHoles.empty() )
    {
        return {};
    }

    return PLANAR::SIMPLEX::FromConvexPolygon( aObstacle.polygon, aRadius );
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
    // Cast before subtracting.  Board coordinates are signed 64-bit IU, so
    // subtracting in the integral domain can overflow before the otherwise
    // exact long-double orientation predicate sees the operands.
    const long double value = ( static_cast<long double>( b.x ) - a.x )
                              * ( static_cast<long double>( c.y ) - a.y )
                              - ( static_cast<long double>( b.y ) - a.y )
                                        * ( static_cast<long double>( c.x ) - a.x );

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
                                 && ( ( static_cast<long double>( b.x ) - a.x )
                                              * ( static_cast<long double>( aPoint.y ) - a.y )
                                      / ( static_cast<long double>( b.y ) - a.y )
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


bool segmentIntersectsPolygonWithHoles( const ROUTER_POINT& aStart,
                                        const ROUTER_POINT& aEnd,
                                        const ROUTING_OBSTACLE& aObstacle,
                                        std::int64_t aRadius )
{
    // A generic polygon is not safe to validate by sampling along the
    // candidate trace.  In particular, a chord joining two arms of a concave
    // *hole* can pass through solid material while both sampled endpoints
    // remain legal.  The minimum distance of two finite line segments occurs
    // at an intersection or at one of their endpoints, so test every outer
    // and hole boundary exactly enough for the worker's integer geometry.
    // Together with solid-area endpoint classification this covers every
    // possible transition between outside, copper, and a hole without a
    // grid-dependent blind spot.
    if( pointInPolygonWithHoles( aStart, aObstacle.polygon, aObstacle.polygonHoles )
        || pointInPolygonWithHoles( aEnd, aObstacle.polygon, aObstacle.polygonHoles ) )
    {
        return true;
    }

    const double clearance = static_cast<double>( std::max<std::int64_t>( 0, aRadius ) );
    const auto hitsBoundary = [&]( const std::vector<ROUTER_POINT>& aContour )
    {
        if( aContour.size() < 2 )
            return false;

        for( std::size_t index = 0; index < aContour.size(); ++index )
            if( segmentsWithinClearance( aStart, aEnd, aContour[index],
                                         aContour[( index + 1 ) % aContour.size()], clearance ) )
            {
                return true;
            }

        return false;
    };

    if( hitsBoundary( aObstacle.polygon ) )
        return true;

    return std::any_of( aObstacle.polygonHoles.begin(), aObstacle.polygonHoles.end(),
                        hitsBoundary );
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
        if( !connection.isExistingBoardRoute || connection.isAutorouterOwned )
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


void ROUTING_OCCUPANCY::AddStatic( const ROUTING_CONNECTION& aConnection )
{
    if( !aConnection.complete || !aConnection.isExistingBoardRoute
        || !HasValidEdgeStyles( aConnection ) )
    {
        throw std::invalid_argument( "Cannot insert an invalid static board routing connection" );
    }

    // This deliberately mirrors Add's congestion accounting but does not
    // call ROUTING_BOARD::AddRoute.  Static source copper must block a
    // proposed trace and participate in conflict discovery, yet a full-net
    // reroute may not use it to satisfy CountMissing() before that source
    // copper is regenerated (or explicitly preserved) in the proposal.
    TRANSACTION transaction( *this );
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
                return SameRouteGeometry( aExisting, aConnection );
            } );

    if( connectionIt == m_connections.end() )
        return;

    // Copy first: callers may pass a reference into Connections().
    const ROUTING_CONNECTION removed = *connectionIt;
    if( m_board && ( !removed.isExistingBoardRoute || removed.isAutorouterOwned ) )
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


bool ROUTING_OCCUPANCY::RemoveItems( const ROUTING_BOARD::ITEM_ID_SET& aItems )
{
    if( !m_board || aItems.empty() )
        return false;

    TRANSACTION transaction( *this );
    if( !m_board->RemoveItems( aItems ) )
        return false;

    std::vector<ROUTING_CONNECTION> normalized;
    normalized.reserve( m_connections.size() );
    for( const ROUTING_CONNECTION& connection : m_connections )
    {
        // AddStatic deliberately has no ROUTING_BOARD item. Preserve those
        // fixed source records while replacing every mutable compound route
        // by the exact item routes which survived removal.
        if( connection.isExistingBoardRoute && !connection.isAutorouterOwned )
            normalized.push_back( connection );
    }
    for( ROUTING_CONNECTION connection : m_board->ItemRoutes() )
        normalized.push_back( std::move( connection ) );
    m_connections = std::move( normalized );

    m_usage.clear();
    for( const ROUTING_CONNECTION& connection : m_connections )
    {
        for( std::size_t index = 1; index < connection.nodes.size(); ++index )
        {
            if( connection.nodes[index - 1].layer != connection.nodes[index].layer )
                continue;
            for( const ROUTER_CELL_KEY& cell : CellsForSegment( connection.nodes[index - 1],
                                                                 connection.nodes[index] ) )
            {
                ++m_usage[cell][connection.netCode];
            }
        }
    }

    transaction.Commit();
    return true;
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
                                        ROUTING_OCCUPANCY& aOccupancy,
                                        int aViaOverrideNetCode,
                                        std::optional<ROUTING_VIA_PROFILE> aViaOverride,
                                        int aTrackWidthOverrideNetCode,
                                        std::optional<std::int64_t> aTrackWidthOverride ) :
        m_board( aBoard ),
        m_settings( aSettings ),
        m_occupancy( aOccupancy ),
        m_activeGridStep( std::max<std::int64_t>( 1, aSettings.gridStepIU ) ),
        m_viaOverrideNetCode( aViaOverrideNetCode ),
        m_viaOverride( aViaOverride )
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
        std::int64_t maximumViaDiameter = net.viaDiameter;
        std::int64_t maximumViaDrill = net.viaDrill;
        for( const ROUTING_VIA_PROFILE& profile : net.viaProfiles )
        {
            maximumViaDiameter = std::max( maximumViaDiameter, profile.diameter );
            maximumViaDrill = std::max( maximumViaDrill, profile.drill );
        }
        m_viaRadii[net.netCode] = std::max<std::int64_t>(
                1, maximumViaDiameter > 0 ? maximumViaDiameter / 2 : 300000 );
        m_viaDrillRadii[net.netCode] = std::max<std::int64_t>(
                1, maximumViaDrill > 0 ? maximumViaDrill / 2 : 150000 );
        m_netClearances[net.netCode] = std::max<std::int64_t>( 0, net.clearance );

        for( std::size_t padIndex : net.padIndices )
        {
            if( padIndex >= m_board.pads.size() )
                continue;

            const ROUTING_PAD& pad = m_board.pads[padIndex];
            const ROUTER_CELL_KEY key{ pad.position.x, pad.position.y, net.netCode };
            // A terminal is an already-existing copper item; the maze is
            // validating only the new trace which leaves or enters it.  Using
            // the pad's largest bounding radius here models that entire pad a
            // second time as circular candidate copper.  On a legal fine-
            // pitch package this can reject every pad centre against its
            // adjacent pin before the first trace segment is considered.
            // Freerouting starts from the compensated target-item door and
            // applies the trace half-width to the routed edge.  Preserve that
            // semantic while retaining the endpoint map as the stable signal
            // used for same-net pad-hole contact handling.
            const std::int64_t radius = m_trackRadii[net.netCode];
            auto [radiusIt, inserted] = m_endpointRadii.emplace( key, radius );
            if( !inserted )
                radiusIt->second = std::max( radiusIt->second, radius );
        }
    }

    // RoutingBoard.fanout() can select a different entry from its combined
    // net/board ViaRule for each pin.  A single net-wide mutation makes the
    // first pin's profile leak into every later fanout task: search then
    // rejects a narrow landing using the earlier large annulus, or explores
    // it at the wrong cost before the final per-edge style corrects it.
    // Keep this override local to the fanout task's engine.  All resulting
    // probes, conflict checks, and strict insertion therefore use exactly
    // the profile carried by that synthetic landing.
    if( aViaOverride && aViaOverrideNetCode > 0 && aViaOverride->diameter > 0
        && aViaOverride->drill > 0 )
    {
        m_viaRadii[aViaOverrideNetCode] = std::max<std::int64_t>(
                1, aViaOverride->diameter / 2 );
        m_viaDrillRadii[aViaOverrideNetCode] = std::max<std::int64_t>(
                1, aViaOverride->drill / 2 );
    }

    // AutorouteConnectionRouter's optional necked retry owns a separate
    // immutable engine.  Narrow its local track radius before any spatial
    // index padding or clearance test is derived.  The caller makes output
    // trace styles explicit, so this changes neither other nets nor the final
    // KiCad width of an unrelated ordinary route.
    if( aTrackWidthOverride && aTrackWidthOverrideNetCode > 0
        && *aTrackWidthOverride > 0 )
    {
        m_trackRadii[aTrackWidthOverrideNetCode] = std::max<std::int64_t>(
                1, *aTrackWidthOverride / 2 );
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


bool MAZE_SEARCH_ENGINE::isPureSmdNet( int aNetCode ) const
{
    bool found = false;
    for( const ROUTING_PAD& pad : m_board.pads )
    {
        if( pad.netCode != aNetCode || pad.isFanoutTarget || pad.isPlaneTarget )
            continue;
        found = true;
        if( !pad.isSmd || pad.layers.size() != 1 )
            return false;
    }

    // This mirrors AutorouteControl.isPureSmdNet's Item test: after a net has
    // acquired trace/via items it is no longer the all-Pin special case.
    if( std::any_of( m_occupancy.Connections().begin(), m_occupancy.Connections().end(),
                     [aNetCode]( const ROUTING_CONNECTION& aConnection )
                     { return aConnection.netCode == aNetCode; } ) )
    {
        return false;
    }

    return found;
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


std::int64_t MAZE_SEARCH_ENGINE::ResolveTrackWidth( int aNetCode,
                                                     const ROUTING_EDGE_STYLE& aStyle ) const
{
    return aStyle.trackWidth > 0 ? aStyle.trackWidth : 2 * netTrackRadius( aNetCode );
}


std::optional<MAZE_SEARCH_ENGINE::PIN_ENTRY_STYLE> MAZE_SEARCH_ENGINE::PinEntryStyle(
        std::size_t aPadIndex, const ROUTER_NODE& aNode, int aNetCode,
        std::int64_t aNormalTrackWidth ) const
{
    if( aPadIndex >= m_board.pads.size() || aNormalTrackWidth <= 0 )
        return {};

    const ROUTING_PAD& pad = m_board.pads[aPadIndex];

    if( pad.netCode != aNetCode || pad.position != aNode.point
        || std::find( pad.layers.begin(), pad.layers.end(), aNode.layer ) == pad.layers.end() )
    {
        return {};
    }

    const auto geometry = std::find_if( pad.layerGeometry.begin(), pad.layerGeometry.end(),
                                        [&]( const ROUTING_PAD::LAYER_GEOMETRY& aGeometry )
                                        {
                                            return aGeometry.layer == aNode.layer;
                                        } );

    if( geometry == pad.layerGeometry.end() || geometry->minWidth <= 0
        || geometry->maxWidth <= 0 )
    {
        return {};
    }

    // Pin.getTraceNeckdownHalfwidth() in the pinned Java source is exactly
    // floor(max(minWidth / 2 - 1, 1)); preserve that integer behavior before
    // applying KiCad's global manufacturing lower bound.
    const std::int64_t halfWidth = std::max<std::int64_t>( geometry->minWidth / 2 - 1, 1 );
    const std::int64_t neckdownWidth = std::max<std::int64_t>(
            2 * halfWidth, m_board.minimumTrackWidth );

    PIN_ENTRY_STYLE result;
    result.style.trackWidth = neckdownWidth;
    result.maxPadWidth = geometry->maxWidth;
    // The source uses the trace/pin clearance matrix at this point.  The
    // adapter preserves the pad-side resolved clearance; the net value is a
    // conservative fallback when an input did not carry per-pad metadata.
    result.clearance = std::max( geometry->clearance, netClearance( aNetCode ) );
    return result;
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


std::int64_t MAZE_SEARCH_ENGINE::edgePairClearance(
        int aFirstNetCode, int aSecondNetCode, int aLayer,
        std::int64_t aFirstEdgeClearance, std::int64_t aSecondEdgeClearance ) const
{
    // The source stores a clearance *class* on every routed trace/via.  The
    // immutable native snapshot has already resolved that class to a
    // distance, so it is an additional minimum on the normal net-pair rule,
    // not a second copy of both netclass values.  Same-net copper remains an
    // electrical connection surface and deliberately has no copper spacing
    // requirement here (drill spacing is checked separately).
    if( aFirstNetCode == aSecondNetCode )
        return 0;

    return std::max( { pairClearance( aFirstNetCode, aSecondNetCode, aLayer ),
                       std::max<std::int64_t>( 0, aFirstEdgeClearance ),
                       std::max<std::int64_t>( 0, aSecondEdgeClearance ) } );
}


std::int64_t MAZE_SEARCH_ENGINE::obstacleExpansionRadius(
        const ROUTING_OBSTACLE& aObstacle, int aNetCode, int aLayer, bool aForVia,
        std::int64_t aCandidateRadius, std::int64_t aCandidateDrillRadius,
        std::int64_t aCandidateEdgeClearance ) const
{
    if( !aObstacle.isHole )
    {
        const std::int64_t candidateRadius = aCandidateRadius >= 0
                                                      ? aCandidateRadius
                                                      : aForVia ? netViaRadius( aNetCode )
                                                                : netTrackRadius( aNetCode );
        const std::int64_t styleClearance = std::max<std::int64_t>( 0,
                                                                     aCandidateEdgeClearance );
        const std::int64_t clearance = aObstacle.netCode != 0
                                                && aObstacle.netCode != aNetCode
                                        ? std::max( edgePairClearance( aNetCode,
                                                                      aObstacle.netCode, aLayer,
                                                                      styleClearance ),
                                                    aObstacle.clearance )
                                        : std::max( { netClearance( aNetCode ),
                                                      aObstacle.clearance,
                                                      styleClearance } );
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
                                                ? ( aCandidateDrillRadius >= 0
                                                            ? aCandidateDrillRadius
                                                            : netViaDrillRadius( aNetCode ) )
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
                                         std::int64_t aEndpointRadius,
                                         std::int64_t aDrillRadius,
                                         std::int64_t aEdgeClearance ) const
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
    const std::int64_t drillRadius = aForVia
            ? ( aDrillRadius >= 0 ? aDrillRadius : netViaDrillRadius( aNetCode ) ) : 0;
    const std::int64_t margin = m_board.edgeClearance + geometryRadius;

    if( !isInsideBoard( aPoint, margin ) )
    {
        if( autorouterDebugEnabled() && m_debugPointChecks == 1 )
        {
            autorouterDebugLog( "start point rejected by board outline point=("
                                + std::to_string( aPoint.x ) + ","
                                + std::to_string( aPoint.y ) + ",L"
                                + std::to_string( aLayer ) + ") margin="
                                + std::to_string( margin ) );
        }
        return false;
    }

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
                obstacle, aNetCode, aLayer, aForVia, geometryRadius, drillRadius,
                aEdgeClearance );

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
            {
                if( autorouterDebugEnabled() && m_debugPointChecks == 1 )
                    autorouterDebugLog( "start point rejected by rectangle obstacle="
                                        + std::to_string( obstacleIndex ) + " net="
                                        + std::to_string( obstacle.netCode ) + " point=("
                                        + std::to_string( aPoint.x ) + ","
                                        + std::to_string( aPoint.y ) + ",L"
                                        + std::to_string( aLayer ) + ")" );
                return false;
            }
        }
        else if( obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
        {
            if( pointToSegmentDistance( aPoint, obstacle.start, obstacle.end ) <= radius )
            {
                if( autorouterDebugEnabled() && m_debugPointChecks == 1 )
                    autorouterDebugLog( "start point rejected by segment obstacle="
                                        + std::to_string( obstacleIndex ) + " net="
                                        + std::to_string( obstacle.netCode ) + " radius="
                                        + std::to_string( radius ) + " obstacle=("
                                        + std::to_string( obstacle.start.x ) + ","
                                        + std::to_string( obstacle.start.y ) + ")->("
                                        + std::to_string( obstacle.end.x ) + ","
                                        + std::to_string( obstacle.end.y ) + ") point=("
                                        + std::to_string( aPoint.x ) + ","
                                        + std::to_string( aPoint.y ) + ",L"
                                        + std::to_string( aLayer ) + ")" );
                return false;
            }
        }
        else if( const auto simplex = exactConvexClearanceShape( obstacle, radius ); simplex )
        {
            if( simplex->Contains( PLANAR::POINT( aPoint ) ) )
            {
                if( autorouterDebugEnabled() && m_debugPointChecks == 1 )
                    autorouterDebugLog( "start point rejected by convex obstacle="
                                        + std::to_string( obstacleIndex ) + " net="
                                        + std::to_string( obstacle.netCode ) + " point=("
                                        + std::to_string( aPoint.x ) + ","
                                        + std::to_string( aPoint.y ) + ",L"
                                        + std::to_string( aLayer ) + ")" );
                return false;
            }
        }
        else if( pointInPolygonWithHoles( aPoint, obstacle.polygon, obstacle.polygonHoles )
                 || pointNearPolygonWithHoles( aPoint, obstacle.polygon,
                                               obstacle.polygonHoles, radius ) )
        {
            if( autorouterDebugEnabled() && m_debugPointChecks == 1 )
                autorouterDebugLog( "start point rejected by polygon obstacle="
                                    + std::to_string( obstacleIndex ) + " net="
                                    + std::to_string( obstacle.netCode ) + " point=("
                                    + std::to_string( aPoint.x ) + ","
                                    + std::to_string( aPoint.y ) + ",L"
                                    + std::to_string( aLayer ) + ")" );
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
                const ROUTING_EDGE_STYLE& otherStyle = EdgeStyle( connection, index - 1 );
                if( !VIA_RULE::SpansLayer( m_settings, first.layer, second.layer, otherStyle,
                                           aLayer ) )
                {
                    continue;
                }
                const std::int64_t otherDrillRadius = otherStyle.viaDrill > 0
                        ? otherStyle.viaDrill / 2 : netViaDrillRadius( connection.netCode );
                if( distance( aPoint, first.point ) < drillRadius + otherDrillRadius
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

            const std::int64_t currentRadius = geometryRadius;

            for( std::size_t i = 1; i < connection.nodes.size(); ++i )
            {
                const ROUTER_NODE& previous = connection.nodes[i - 1];
                const ROUTER_NODE& current = connection.nodes[i];
                const ROUTING_EDGE_STYLE& otherStyle = EdgeStyle( connection, i - 1 );
                const std::int64_t otherTrackRadius = otherStyle.trackWidth > 0
                        ? otherStyle.trackWidth / 2 : netTrackRadius( connection.netCode );
                const std::int64_t otherViaRadius = otherStyle.viaDiameter > 0
                        ? otherStyle.viaDiameter / 2 : netViaRadius( connection.netCode );
                const std::int64_t otherViaDrillRadius = otherStyle.viaDrill > 0
                        ? otherStyle.viaDrill / 2 : netViaDrillRadius( connection.netCode );

                if( previous.layer == current.layer )
                {
                    if( previous.layer != aLayer )
                        continue;

                    if( pointToSegmentDistance( aPoint, previous.point, current.point )
                        <= currentRadius + otherTrackRadius
                                   + edgePairClearance( aNetCode, connection.netCode, aLayer,
                                                        aEdgeClearance, otherStyle.clearance ) )
                    {
                        return false;
                    }
                }
                else if( distance( aPoint, previous.point )
                         <= std::max( currentRadius + otherViaRadius
                                              + edgePairClearance( aNetCode, connection.netCode,
                                                                   aLayer, aEdgeClearance,
                                                                   otherStyle.clearance ),
                                      drillRadius + otherViaDrillRadius
                                              + m_board.holeToHoleClearance )
                    && VIA_RULE::SpansLayer( m_settings, previous.layer, current.layer,
                                              otherStyle, aLayer ) )
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
                                            std::int64_t aEndRadius,
                                            std::int64_t aSegmentRadius,
                                            std::int64_t aEdgeClearance ) const
{
    if( !isPointAllowed( aStart, aLayer, aNetCode, aForVia, aStartRadius, -1,
                         aEdgeClearance ) )
        return false;

    return isSegmentAllowedFromKnownStart( aStart, aEnd, aLayer, aNetCode, aForVia,
                                           aEndRadius, aSegmentRadius, aEdgeClearance );
}


bool MAZE_SEARCH_ENGINE::isSegmentAllowedFromKnownStart(
        const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd, int aLayer, int aNetCode,
        bool aForVia, std::int64_t aEndRadius, std::int64_t aSegmentRadius,
        std::int64_t aEdgeClearance ) const
{
    if( autorouterDebugEnabled() )
        ++m_debugSegmentChecks;

    if( !isPointAllowed( aEnd, aLayer, aNetCode, aForVia, aEndRadius, -1,
                         aEdgeClearance ) )
        return false;

    const std::int64_t defaultRadius = aForVia ? netViaRadius( aNetCode )
                                                : netTrackRadius( aNetCode );
    const std::int64_t segmentRadius = aSegmentRadius >= 0 ? aSegmentRadius : defaultRadius;

    // Endpoints are checked above.  For the segment itself, a straight line
    // can leave a concave outline (or graze a board hole) between legal
    // endpoints.  Segment-to-segment distance is both exact for this case and
    // substantially cheaper than sampling every half-grid cell of a long
    // visibility edge.
    const double boundaryClearance = static_cast<double>( m_board.edgeClearance
                                                          + segmentRadius );
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

        const std::int64_t radius = obstacleExpansionRadius( obstacle, aNetCode, aLayer, aForVia,
                                                               segmentRadius, -1,
                                                               aEdgeClearance );

        if( !boxesOverlap( routeBounds, m_obstacleBounds[obstacleIndex], radius ) )
            continue;

        if( obstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
        {
            // Freerouting's 45-degree compensated tree uses the eight
            // orthogonal/diagonal support lines of an IntOctagon here.  A
            // square bounding-box enlargement is safe but materially too
            // conservative: it rejects legal 45-degree traces around a
            // rectangular corner and invalidates paths produced by the exact
            // room frontier.  The support octagon circumscribes the true
            // round offset, so accepting its exterior remains DRC-safe.
            const PLANAR::INT_OCTAGON clearanceShape =
                    PLANAR::INT_OCTAGON::FromBox( obstacle.box ).Offset( radius );
            const auto simplex = clearanceShape.ToSimplex();
            const PLANAR::POLYLINE path = PLANAR::POLYLINE::FromPoints(
                    { aStart, aEnd } );
            if( simplex && ( ( !path.Empty() && simplex->IntersectsSegment( path, 1 ) )
                             || ( path.Empty()
                                  && simplex->Contains( PLANAR::POINT( aStart ) ) ) ) )
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
            if( const auto simplex = exactConvexClearanceShape( obstacle, radius ); simplex )
            {
                try
                {
                    const PLANAR::POLYLINE path = PLANAR::POLYLINE::FromPoints(
                            { aStart, aEnd } );
                    if( !path.Empty() && simplex->IntersectsSegment( path, 1 ) )
                        return false;
                    if( path.Empty() && simplex->Contains( PLANAR::POINT( aStart ) ) )
                        return false;
                    continue;
                }
                catch( const std::exception& )
                {
                    // A degenerate/overflowing support line must not be
                    // rounded into a legal route. Fall back to the existing
                    // conservative polygon predicate below.
                }
            }

            if( segmentIntersectsPolygonWithHoles( aStart, aEnd, obstacle, radius ) )
                return false;
        }
    }

    if( !m_allowRipupOccupancy )
    {
        // Occupancy cells are deliberately coarse (one routing-grid sample
        // per segment) and feed negotiated-congestion cost only.  They are
        // not a clearance representation: two correctly separated tracks
        // can occupy the same cell.  In particular, treating them as a hard
        // legality veto made a transactionally shoved trace impossible to
        // reinsert beside the incoming route.  The exact copper/via scan
        // below is the authoritative foreign-net collision check.
        const std::int64_t currentRadius = segmentRadius;

        for( const ROUTING_CONNECTION& connection : m_occupancy.Connections() )
        {
            if( connection.netCode == aNetCode )
                continue;

            for( std::size_t i = 1; i < connection.nodes.size(); ++i )
            {
                const ROUTER_NODE& previous = connection.nodes[i - 1];
                const ROUTER_NODE& current = connection.nodes[i];
                const bool otherIsVia = previous.layer != current.layer;
                const ROUTING_EDGE_STYLE& otherStyle = EdgeStyle( connection, i - 1 );
                const std::int64_t otherCopperRadius = otherIsVia
                        ? ( otherStyle.viaDiameter > 0 ? otherStyle.viaDiameter / 2
                                                       : netViaRadius( connection.netCode ) )
                        : ( otherStyle.trackWidth > 0 ? otherStyle.trackWidth / 2
                                                       : netTrackRadius( connection.netCode ) );
                const std::int64_t otherDrillRadius = otherStyle.viaDrill > 0
                        ? otherStyle.viaDrill / 2 : netViaDrillRadius( connection.netCode );
                const std::int64_t copperClearance =
                        currentRadius
                        + otherCopperRadius
                        + edgePairClearance( aNetCode, connection.netCode, aLayer,
                                             aEdgeClearance, otherStyle.clearance );
                const std::int64_t clearance =
                        aForVia && otherIsVia
                                ? std::max( copperClearance,
                                            netViaDrillRadius( aNetCode )
                                                    + otherDrillRadius
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
                                        const ROUTER_NODE& aEnd, bool aForVia,
                                        const ROUTING_EDGE_STYLE* aStyle ) const
{
    const std::int64_t styleRadius = aForVia
            ? ( aStyle && aStyle->viaDiameter > 0 ? aStyle->viaDiameter / 2
                                                   : netViaRadius( aNetCode ) )
            : ( aStyle && aStyle->trackWidth > 0 ? aStyle->trackWidth / 2
                                                  : netTrackRadius( aNetCode ) );
    const std::int64_t styleClearance =
            aStyle ? std::max<std::int64_t>( 0, aStyle->clearance ) : 0;

    if( aStart.layer != aEnd.layer )
    {
        if( aStart.point != aEnd.point
            || !VIA_RULE::AllowsTransition( m_settings, aStart.layer, aEnd.layer, aStyle ) )
            return false;

        const std::vector<int> viaLayers = VIA_RULE::LayersFor( m_settings, aStart.layer,
                                                                 aEnd.layer, aStyle );
        const std::int64_t drillRadius = aStyle && aStyle->viaDrill > 0
                ? aStyle->viaDrill / 2 : netViaDrillRadius( aNetCode );
        for( int layer : viaLayers )
        {
            if( !isPointAllowed( aStart.point, layer, aNetCode, true, styleRadius, drillRadius,
                                 styleClearance ) )
                return false;
        }

        return true;
    }

    if( aStyle )
    {
        return isSegmentAllowed( aStart.point, aEnd.point, aStart.layer, aNetCode, aForVia,
                                 styleRadius, styleRadius, styleRadius, styleClearance );
    }

    return isSegmentAllowed( aStart.point, aEnd.point, aStart.layer, aNetCode, aForVia,
                             endpointRadius( aNetCode, aStart.point ),
                             endpointRadius( aNetCode, aEnd.point ) );
}


bool MAZE_SEARCH_ENGINE::CanInsertSegment( int net, const ROUTER_NODE& start,
                                            const ROUTER_NODE& end,
                                            const ROUTING_EDGE_STYLE* style ) const
{
    struct RESTORE
    {
        bool& flag;
        bool value;
        ~RESTORE() { flag = value; }
    } restore{ m_allowRipupOccupancy, m_allowRipupOccupancy };
    m_allowRipupOccupancy = false;
    if( start.layer != end.layer )
        return CanUseSegment( net, start, end, true, style );
    const auto radius = style && style->trackWidth > 0 ? style->trackWidth / 2
                                                        : netTrackRadius( net );
    const auto clearance = style ? std::max<std::int64_t>( 0, style->clearance ) : 0;
    return isSegmentAllowed( start.point, end.point, start.layer, net, false, radius, radius,
                             radius, clearance );
}


std::optional<ROUTING_EDGE_STYLE> MAZE_SEARCH_ENGINE::SelectViaStyle(
        int aNetCode, const ROUTER_NODE& aStart, const ROUTER_NODE& aEnd,
        bool aAttachesToSmd ) const
{
    if( aStart.layer == aEnd.layer || aStart.point != aEnd.point )
        return std::nullopt;

    const ROUTING_NET* net = nullptr;
    for( const ROUTING_NET& candidate : m_board.nets )
    {
        if( candidate.netCode == aNetCode )
        {
            net = &candidate;
            break;
        }
    }

    const auto tryStyle = [&]( ROUTING_EDGE_STYLE aStyle,
                               bool aAttachSmdAllowed )
            -> std::optional<ROUTING_EDGE_STYLE>
    {
        if( aAttachesToSmd && !m_settings.allowViaInSmdPad && !aAttachSmdAllowed )
            return std::nullopt;

        std::vector<int> span = VIA_RULE::LayersFor( m_settings, aStart.layer,
                                                     aEnd.layer, &aStyle );
        if( span.empty() )
            return std::nullopt;

        // Canonicalize the complete manufactured span now.  Reconstruction
        // and proposal emission must not shrink it to the maze edge's layers.
        aStyle.viaLayers = std::move( span );
        if( aStyle.viaType == ROUTER_VIA_TYPE::AUTO )
        {
            aStyle.viaType = aStyle.viaLayers == VIA_RULE::ThroughLayers( m_settings )
                                     ? ROUTER_VIA_TYPE::THROUGH
                                     : ROUTER_VIA_TYPE::BLIND_BURIED;
        }

        if( !CanUseSegment( aNetCode, aStart, aEnd, true, &aStyle ) )
            return std::nullopt;

        return aStyle;
    };

    // A fanout task has already selected one entry from the combined
    // net/board ViaRule.  Keep that decision local and exclusive so another
    // profile cannot leak in between preflight and forced insertion.
    if( m_viaOverride && m_viaOverrideNetCode == aNetCode )
    {
        ROUTING_EDGE_STYLE style;
        style.viaDiameter = m_viaOverride->diameter;
        style.viaDrill = m_viaOverride->drill;
        style.viaLayers = m_viaOverride->layers;
        style.viaType = m_viaOverride->type;
        return tryStyle( std::move( style ), m_viaOverride->attachSmdAllowed );
    }

    if( net && !net->viaProfiles.empty() )
    {
        for( const ROUTING_VIA_PROFILE& profile : net->viaProfiles )
        {
            ROUTING_EDGE_STYLE style;
            style.viaDiameter = profile.diameter > 0 ? profile.diameter : net->viaDiameter;
            style.viaDrill = profile.drill > 0 ? profile.drill : net->viaDrill;
            style.viaLayers = profile.layers;
            style.viaType = profile.type;
            if( style.viaDiameter <= 0 || style.viaDrill <= 0 )
                continue;

            if( auto selected = tryStyle( std::move( style ), profile.attachSmdAllowed ) )
                return selected;
        }

        // An explicit ViaRule is authoritative.  Falling back to an invented
        // through via here would violate the same rule that rejected every
        // declared ViaInfo above.
        return std::nullopt;
    }

    ROUTING_EDGE_STYLE legacy;
    legacy.viaDiameter = net && net->viaDiameter > 0
                                 ? net->viaDiameter : 2 * netViaRadius( aNetCode );
    legacy.viaDrill = net && net->viaDrill > 0
                              ? net->viaDrill : 2 * netViaDrillRadius( aNetCode );
    legacy.viaType = ROUTER_VIA_TYPE::THROUGH;
    return tryStyle( std::move( legacy ), false );
}


bool MAZE_SEARCH_ENGINE::assignViaStyles( ROUTING_CONNECTION& aConnection ) const
{
    if( !HasValidEdgeStyles( aConnection ) )
        return false;

    const bool hasVia = std::adjacent_find(
            aConnection.nodes.begin(), aConnection.nodes.end(),
            []( const ROUTER_NODE& aLeft, const ROUTER_NODE& aRight )
            { return aLeft.layer != aRight.layer; } ) != aConnection.nodes.end();
    if( !hasVia )
        return true;

    EnsureEdgeStyles( aConnection );
    for( std::size_t edge = 1; edge < aConnection.nodes.size(); ++edge )
    {
        const ROUTER_NODE& from = aConnection.nodes[edge - 1];
        const ROUTER_NODE& to = aConnection.nodes[edge];
        if( from.layer == to.layer )
            continue;

        ROUTING_EDGE_STYLE& current = aConnection.edgeStyles[edge - 1];
        const bool explicitPadstack = current.viaDiameter > 0 || current.viaDrill > 0
                                      || !current.viaLayers.empty()
                                      || current.viaType != ROUTER_VIA_TYPE::AUTO;
        if( explicitPadstack )
        {
            std::vector<int> span = VIA_RULE::LayersFor( m_settings, from.layer, to.layer,
                                                         &current );
            if( span.empty() )
                return false;
            current.viaLayers = std::move( span );
            if( current.viaType == ROUTER_VIA_TYPE::AUTO )
            {
                current.viaType = current.viaLayers == VIA_RULE::ThroughLayers( m_settings )
                                          ? ROUTER_VIA_TYPE::THROUGH
                                          : ROUTER_VIA_TYPE::BLIND_BURIED;
            }
            if( !CanUseSegment( aConnection.netCode, from, to, true, &current ) )
                return false;
            continue;
        }

        const bool attachesToSmd = std::any_of(
                m_board.pads.begin(), m_board.pads.end(), [&]( const ROUTING_PAD& aPad )
                {
                    return aPad.netCode == aConnection.netCode && aPad.isSmd
                           && aPad.position == from.point
                           && ( isOnPadLayer( aPad, from.layer )
                                || isOnPadLayer( aPad, to.layer ) );
                } );
        auto selected = SelectViaStyle( aConnection.netCode, from, to, attachesToSmd );
        if( !selected )
            return false;
        selected->trackWidth = current.trackWidth;
        selected->clearance = current.clearance;
        current = std::move( *selected );
    }

    return true;
}


bool MAZE_SEARCH_ENGINE::hasStaticViaDrillClearance(
        const ROUTING_CONNECTION& aCandidate,
        const std::vector<ROUTING_CONNECTION>& aStaticDrillObstacles ) const
{
    // ROUTING_OCCUPANCY::AddStatic intentionally keeps source copper out of
    // the worker board's connectivity graph.  `isPointAllowed` nevertheless
    // sees every static route that is still resident in occupancy.  A forced
    // insertion removes its full initial conflict set before asking this
    // helper to choose a shove location, however, so a same-net source via
    // in that removed set would otherwise escape the regular hole-to-hole
    // check.  Compare only via drill geometry here: same-net copper sharing
    // is electrical connectivity, while two distinct drills still require
    // manufacturing spacing.
    if( !HasValidEdgeStyles( aCandidate ) )
        return false;

    const auto viaDrillRadius = [&]( int aNetCode, const ROUTING_EDGE_STYLE& aStyle )
    {
        return aStyle.viaDrill > 0 ? aStyle.viaDrill / 2
                                   : netViaDrillRadius( aNetCode );
    };
    const auto replacesSameSourceItem = [&]( const ROUTING_CONNECTION& aStaticRoute )
    {
        // The moved route retains its source UUID solely so proposal
        // acceptance removes the old BOARD_ITEM.  That old drill is the one
        // item whose former centre must not veto its own replacement.
        return !aCandidate.sourceBoardItemIds.empty()
               && aCandidate.sourceBoardItemIds == aStaticRoute.sourceBoardItemIds;
    };

    for( std::size_t candidateEdge = 1; candidateEdge < aCandidate.nodes.size();
         ++candidateEdge )
    {
        const ROUTER_NODE& candidateStart = aCandidate.nodes[candidateEdge - 1];
        const ROUTER_NODE& candidateEnd = aCandidate.nodes[candidateEdge];
        if( candidateStart.layer == candidateEnd.layer
            || candidateStart.point != candidateEnd.point )
        {
            continue;
        }

        const ROUTING_EDGE_STYLE& candidateStyle = EdgeStyle( aCandidate, candidateEdge - 1 );
        const std::vector<int> candidateLayers = VIA_RULE::LayersFor(
                m_settings, candidateStart.layer, candidateEnd.layer, &candidateStyle );
        if( candidateLayers.empty() )
            return false;

        const std::int64_t candidateDrill = viaDrillRadius( aCandidate.netCode,
                                                             candidateStyle );
        for( const ROUTING_CONNECTION& staticRoute : aStaticDrillObstacles )
        {
            if( !staticRoute.isExistingBoardRoute || staticRoute.netCode != aCandidate.netCode
                || !HasValidEdgeStyles( staticRoute ) || replacesSameSourceItem( staticRoute ) )
            {
                continue;
            }

            for( std::size_t staticEdge = 1; staticEdge < staticRoute.nodes.size(); ++staticEdge )
            {
                const ROUTER_NODE& staticStart = staticRoute.nodes[staticEdge - 1];
                const ROUTER_NODE& staticEnd = staticRoute.nodes[staticEdge];
                if( staticStart.layer == staticEnd.layer || staticStart.point != staticEnd.point )
                    continue;

                const ROUTING_EDGE_STYLE& staticStyle = EdgeStyle( staticRoute, staticEdge - 1 );
                const bool sharedCopperLayer = std::any_of(
                        candidateLayers.begin(), candidateLayers.end(), [&]( int aLayer )
                        {
                            return VIA_RULE::SpansLayer( m_settings, staticStart.layer,
                                                        staticEnd.layer, staticStyle, aLayer );
                        } );
                if( !sharedCopperLayer )
                    continue;

                const std::int64_t requiredClearance =
                        candidateDrill + viaDrillRadius( staticRoute.netCode, staticStyle )
                        + m_board.holeToHoleClearance;
                if( distance( candidateStart.point, staticStart.point )
                    < static_cast<double>( requiredClearance ) )
                {
                    return false;
                }
            }
        }
    }

    return true;
}


bool MAZE_SEARCH_ENGINE::preservesConductionAreaContacts(
        const ROUTING_CONNECTION& aOriginal,
        const ROUTING_CONNECTION& aReplacement ) const
{
    if( !HasValidEdgeStyles( aOriginal ) || !HasValidEdgeStyles( aReplacement ) )
        return false;

    const auto viaRadius = [&]( int aNetCode, const ROUTING_EDGE_STYLE& aStyle )
    {
        return aStyle.viaDiameter > 0 ? aStyle.viaDiameter / 2 : netViaRadius( aNetCode );
    };
    const auto appliesOnLayer = []( const ROUTING_OBSTACLE& aArea, int aLayer )
    {
        return aArea.layers.empty()
               || std::find( aArea.layers.begin(), aArea.layers.end(), aLayer )
                          != aArea.layers.end();
    };
    const auto annulusTouchesArea = [&]( const ROUTING_OBSTACLE& aArea,
                                         const ROUTER_POINT& aCentre,
                                         std::int64_t aRadius )
    {
        if( aArea.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
        {
            ROUTER_BOX expanded = aArea.box;
            expanded.minX = saturatedAdd( expanded.minX, -aRadius );
            expanded.minY = saturatedAdd( expanded.minY, -aRadius );
            expanded.maxX = saturatedAdd( expanded.maxX, aRadius );
            expanded.maxY = saturatedAdd( expanded.maxY, aRadius );
            return expanded.Contains( aCentre );
        }

        if( aArea.kind != ROUTER_OBSTACLE_KIND::POLYGON )
            return false;

        // A via whose centre is inside a filled region clearly contacts it.
        // If its centre is just outside a boundary (or in a thermal void),
        // its annulus can still be a normal contact; retain that exact
        // straight-contour case as well.  This matches the worker's polygon
        // clearance predicates without pretending curved zone outlines have
        // been reconstructed here.
        return CONTACT_GEOMETRY::ContainsArea( aArea, aCentre )
               || pointNearPolygonWithHoles( aCentre, aArea.polygon,
                                             aArea.polygonHoles, aRadius );
    };

    const auto replacementTouchesArea = [&]( const ROUTING_OBSTACLE& aArea, int aLayer )
    {
        for( std::size_t edge = 1; edge < aReplacement.nodes.size(); ++edge )
        {
            const ROUTER_NODE& first = aReplacement.nodes[edge - 1];
            const ROUTER_NODE& second = aReplacement.nodes[edge];
            if( first.layer == second.layer || first.point != second.point )
                continue;

            const ROUTING_EDGE_STYLE& style = EdgeStyle( aReplacement, edge - 1 );
            if( !VIA_RULE::SpansLayer( m_settings, first.layer, second.layer, style, aLayer ) )
                continue;

            if( annulusTouchesArea( aArea, first.point,
                                    viaRadius( aReplacement.netCode, style ) ) )
            {
                return true;
            }
        }

        return false;
    };

    for( std::size_t edge = 1; edge < aOriginal.nodes.size(); ++edge )
    {
        const ROUTER_NODE& first = aOriginal.nodes[edge - 1];
        const ROUTER_NODE& second = aOriginal.nodes[edge];
        if( first.layer == second.layer || first.point != second.point )
            continue;

        const ROUTING_EDGE_STYLE& style = EdgeStyle( aOriginal, edge - 1 );
        const std::vector<int> layers = VIA_RULE::LayersFor(
                m_settings, first.layer, second.layer, &style );
        if( layers.empty() )
            return false;

        for( int layer : layers )
        {
            for( const ROUTING_OBSTACLE& area : m_board.conductionAreas )
            {
                if( area.netCode != aOriginal.netCode || !appliesOnLayer( area, layer )
                    || !annulusTouchesArea( area, first.point,
                                            viaRadius( aOriginal.netCode, style ) ) )
                {
                    continue;
                }

                if( !replacementTouchesArea( area, layer ) )
                    return false;
            }
        }
    }

    return true;
}


std::optional<ROUTING_CONNECTION> MAZE_SEARCH_ENGINE::SpringOverConnection(
        const ROUTING_CONNECTION& connection, const ROUTER_CANCEL_CALLBACK& cancel ) const
{
    return SpringOverConnection( connection, {}, cancel );
}


std::optional<ROUTING_CONNECTION> MAZE_SEARCH_ENGINE::SpringOverConnection(
        const ROUTING_CONNECTION& connection,
        const std::vector<ROUTING_CONNECTION>& transientObstacles,
        const ROUTER_CANCEL_CALLBACK& cancel ) const
{
    if( connection.nodes.size() < 2 || !HasValidEdgeStyles( connection ) ) return {};
    ROUTING_CONNECTION result = connection;
    result.nodes.clear();
    result.edgeStyles.clear();

    const auto appendNode = [&]( const ROUTER_NODE& aNode,
                                 const ROUTING_EDGE_STYLE* aStyle ) -> bool
    {
        if( result.nodes.empty() )
        {
            result.nodes.push_back( aNode );
            return true;
        }

        if( !aStyle )
            return false;

        result.nodes.push_back( aNode );
        if( !connection.edgeStyles.empty() )
            result.edgeStyles.push_back( *aStyle );
        return true;
    };

    // A `ROUTING_CONNECTION` may contain a short terminal neckdown followed
    // by normal-width copper on the same layer.  TraceShover operates on one
    // trace style at a time; treating that entire layer run as an all-or-
    // nothing polyline rejected a legal spring-over of the ordinary-width
    // middle merely because an untouched terminal edge had a different
    // width.  Iterate edges, splitting only at a via or a style boundary,
    // while retaining the shared node between adjacent runs.
    if( !appendNode( connection.nodes.front(), nullptr ) )
        return {};

    std::size_t edge = 0;
    while( edge + 1 < connection.nodes.size() )
    {
        if( cancel && cancel() ) return {};

        // A via is already a single style-preserving edge. It is not a
        // TraceShover contour, so carry it verbatim and resume on its exit
        // layer.
        if( connection.nodes[edge].layer != connection.nodes[edge + 1].layer )
        {
            if( !appendNode( connection.nodes[edge + 1], &EdgeStyle( connection, edge ) ) )
                return {};
            ++edge;
            continue;
        }

        const int layer = connection.nodes[edge].layer;
        const ROUTING_EDGE_STYLE& style = EdgeStyle( connection, edge );
        std::size_t lastEdge = edge;
        std::vector<ROUTER_POINT> points{ connection.nodes[edge].point };
        while( lastEdge + 1 < connection.nodes.size()
               && connection.nodes[lastEdge].layer == connection.nodes[lastEdge + 1].layer
               && EdgeStyle( connection, lastEdge ) == style )
        {
            // POLYLINE preserves arbitrary support lines exactly. A later
            // IntegralCorners check rejects any non-integral contour result;
            // never round a general-angle forced route into a false contact.
            points.push_back( connection.nodes[lastEdge + 1].point );
            ++lastEdge;
        }

        std::vector<TRACE_SHOVER::OBSTACLE> obstacles;
        for( std::size_t i = m_board.obstacles.size(); i > 0; --i )
        {
            if( cancel && cancel() ) return {};
            const auto& obstacle = m_board.obstacles[i - 1];
            if( !obstacle.blocksTracks || obstacle.isHole
                || ( obstacle.netCode == connection.netCode && !obstacle.isKeepout )
                || ( !obstacle.layers.empty()
                     && std::find( obstacle.layers.begin(), obstacle.layers.end(), layer )
                                == obstacle.layers.end() )
                || !obstacle.polygonHoles.empty() )
            {
                continue;
            }
            const auto radius = obstacleExpansionRadius(
                    obstacle, connection.netCode, layer, false,
                    style.trackWidth > 0 ? style.trackWidth / 2
                                         : netTrackRadius( connection.netCode ),
                    -1, style.clearance );
            // Host rule evaluation already supplies clearance. No Java
            // class-0 broad-phase omission or hardcoded source-unit margin.
            if( obstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
            {
                // A rounded rectangle is not an exact box. Keep it for
                // the general shape path below once an offset model for
                // its arcs exists; do not silently square its corners.
                if( obstacle.radius != 0 )
                    continue;
                const ROUTER_BOX box = obstacle.box;
                if( box.minX >= box.maxX || box.minY >= box.maxY )
                    continue;
                const auto expanded = [&]( std::int64_t aExtra )
                {
                    return PLANAR::SIMPLEX::Box(
                            { box.minX - radius - aExtra, box.minY - radius - aExtra,
                              box.maxX + radius + aExtra, box.maxY + radius + aExtra } );
                };
                obstacles.push_back( { i, box, expanded( 0 ), expanded( 1 ) } );
            }
            else if( obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
            {
                // KiCad represents circular and oval pads, vias and straight
                // copper as a centre segment swept by its physical radius.
                // The source TraceShover receives their compensated TileShape
                // from the search tree, so excluding native SEGMENT snapshots
                // made forced insertion give up on the most common fixed
                // obstacles even though CanInsertSegment() could see them.
                //
                // Construct the same kind of convex support contour here: the
                // L-infinity sweep contains the circular clearance envelope,
                // retains exact integer/rational support intersections, and is
                // checked again against KiCad's real capsule geometry before
                // publication. A point segment (circle/via) needs a Box because
                // FromExpandedSegment intentionally rejects zero-length input.
                if( radius <= 0 || radius == std::numeric_limits<std::int64_t>::max() )
                    continue;

                const auto expandedSegment = [&]( std::int64_t aRadius )
                        -> std::optional<PLANAR::SIMPLEX>
                {
                    if( aRadius <= 0 || aRadius == std::numeric_limits<std::int64_t>::max() )
                        return {};

                    if( obstacle.start != obstacle.end )
                    {
                        return PLANAR::SIMPLEX::FromExpandedSegment(
                                obstacle.start, obstacle.end, aRadius );
                    }

                    const ROUTER_BOX box{
                            saturatedAdd( obstacle.start.x, -aRadius ),
                            saturatedAdd( obstacle.start.y, -aRadius ),
                            saturatedAdd( obstacle.start.x, aRadius ),
                            saturatedAdd( obstacle.start.y, aRadius ) };
                    if( box.minX >= box.maxX || box.minY >= box.maxY )
                        return {};

                    try
                    {
                        return PLANAR::SIMPLEX::Box( box );
                    }
                    catch( const std::exception& )
                    {
                        // A malformed/overflowing support contour must never
                        // be rounded into a plausible forced route.
                        return {};
                    }
                };

                const auto check = expandedSegment( radius );
                const auto offset = expandedSegment( saturatedAdd( radius, 1 ) );
                if( !check || !offset )
                    continue;

                const std::int64_t physicalRadius = std::max<std::int64_t>( 0, obstacle.radius );
                const ROUTER_BOX physicalBounds{
                        saturatedAdd( std::min( obstacle.start.x, obstacle.end.x ), -physicalRadius ),
                        saturatedAdd( std::min( obstacle.start.y, obstacle.end.y ), -physicalRadius ),
                        saturatedAdd( std::max( obstacle.start.x, obstacle.end.x ), physicalRadius ),
                        saturatedAdd( std::max( obstacle.start.y, obstacle.end.y ), physicalRadius ) };
                obstacles.push_back( { i, physicalBounds, *check, *offset } );
            }
            else if( obstacle.kind == ROUTER_OBSTACLE_KIND::POLYGON )
            {
                // A source TileShape is convex at this operation. Accept
                // only an explicitly convex snapshot contour and use an
                // L-infinity offset, which conservatively contains the
                // circular copper-clearance offset. Concave/holed shapes
                // stay fail-closed until their decomposition is ported.
                const auto check = PLANAR::SIMPLEX::FromConvexPolygon( obstacle.polygon,
                                                                         radius );
                const auto offset = radius == std::numeric_limits<std::int64_t>::max()
                        ? std::optional<PLANAR::SIMPLEX>{}
                        : PLANAR::SIMPLEX::FromConvexPolygon( obstacle.polygon,
                                                               radius + 1 );
                if( !check || !offset )
                    continue;
                obstacles.push_back( { i, m_obstacleBounds[i - 1], *check, *offset } );
            }
        }

        // Generated routes are mutable worker objects, not immutable
        // snapshot obstacles.  During forced insertion they are supplied
        // here explicitly so the same recursive contour logic can move a
        // trace around the incoming copper before it is committed.  Keep
        // this representation intentionally conservative: arbitrary
        // swept/curved shapes return no proposal elsewhere rather than a
        // bounding-box route that could violate clearance.
        const std::int64_t movingRadius = style.trackWidth > 0
                ? style.trackWidth / 2 : netTrackRadius( connection.netCode );
        for( std::size_t routeIndex = 0; routeIndex < transientObstacles.size(); ++routeIndex )
        {
            const ROUTING_CONNECTION& transient = transientObstacles[routeIndex];
            if( transient.netCode == connection.netCode
                || !HasValidEdgeStyles( transient ) )
            {
                continue;
            }

            for( std::size_t transientEdge = 1; transientEdge < transient.nodes.size();
                 ++transientEdge )
            {
                if( cancel && cancel() ) return {};
                const ROUTER_NODE& first = transient.nodes[transientEdge - 1];
                const ROUTER_NODE& second = transient.nodes[transientEdge];
                const ROUTING_EDGE_STYLE& transientStyle =
                        EdgeStyle( transient, transientEdge - 1 );
                const bool via = first.layer != second.layer;
                const std::uint64_t id = ( std::uint64_t{ 1 } << 63 )
                                         + ( routeIndex << 20 ) + transientEdge;
                ROUTER_BOX raw;
                std::int64_t expansion = 0;

                if( !via )
                {
                    if( first.layer != layer )
                        continue;

                    const std::int64_t transientRadius = transientStyle.trackWidth > 0
                            ? transientStyle.trackWidth / 2
                            : netTrackRadius( transient.netCode );
                    expansion = movingRadius + transientRadius
                                + edgePairClearance( connection.netCode, transient.netCode,
                                                     layer, style.clearance,
                                                     transientStyle.clearance );
                    raw = { std::min( first.point.x, second.point.x ),
                            std::min( first.point.y, second.point.y ),
                            std::max( first.point.x, second.point.x ),
                            std::max( first.point.y, second.point.y ) };
                }
                else
                {
                    if( !VIA_RULE::SpansLayer( m_settings, first.layer, second.layer,
                                               transientStyle, layer ) )
                    {
                        continue;
                    }

                    const std::int64_t transientRadius = transientStyle.viaDiameter > 0
                            ? transientStyle.viaDiameter / 2
                            : netViaRadius( transient.netCode );
                    expansion = movingRadius + transientRadius
                                + edgePairClearance( connection.netCode, transient.netCode,
                                                     layer, style.clearance,
                                                     transientStyle.clearance );
                    raw = { first.point.x, first.point.y, first.point.x, first.point.y };
                }

                // Both width radii are non-negative and the source's
                // closed collision semantics require a nonzero wrapped
                // shape even for a point via.
                expansion = std::max<std::int64_t>( 1, expansion );
                if( via )
                {
                    const ROUTER_BOX check{ raw.minX - expansion, raw.minY - expansion,
                                            raw.maxX + expansion, raw.maxY + expansion };
                    const ROUTER_BOX offset{ check.minX - 1, check.minY - 1,
                                             check.maxX + 1, check.maxY + 1 };
                    obstacles.push_back( { id, raw, PLANAR::SIMPLEX::Box( check ),
                                           PLANAR::SIMPLEX::Box( offset ) } );
                }
                else
                {
                    // A trace is a swept segment, not the rectangle that
                    // encloses its two endpoints.  Using that enclosing
                    // box made a forced shove treat the empty diagonal
                    // wedges beside a trace as copper and could reject a
                    // legal recursive move before strict insertion had a
                    // chance to prove it.  The convex L-infinity sweep
                    // keeps integer input vertices and source-style exact
                    // support intersections; malformed/overflowing input
                    // fails closed by declining this spring-over move.
                    const auto check = PLANAR::SIMPLEX::FromExpandedSegment(
                            first.point, second.point, expansion );
                    const auto offset = PLANAR::SIMPLEX::FromExpandedSegment(
                            first.point, second.point, saturatedAdd( expansion, 1 ) );
                    if( !check || !offset )
                        continue;
                    obstacles.push_back( { id, raw, *check, *offset } );
                }
            }
        }

        const auto path = PLANAR::POLYLINE::FromPoints( points );
        if( path.Empty() )
            return {};
        auto wrapped = TRACE_SHOVER::SpringOverObstacles( path, obstacles, cancel );
        if( wrapped.cancelled || !wrapped.polyline ) return {};
        // The pinned spring-over method can lose an endpoint on a looping
        // input. Preserve its oracle output, but NEVER accept that mutation.
        if( !wrapped.polyline->HasSameEndpoints( path ) ) return {};
        const auto corners = wrapped.polyline->IntegralCorners();
        if( !corners ) return {};

        for( std::size_t corner = 1; corner < corners->size(); ++corner )
            if( !appendNode( { corners->at( corner ), layer }, &style ) )
                return {};
        edge = lastEdge;
    }
    if( SameRouteGeometry( result, connection ) ) return {};
    // Preserve endpoint identities, via transitions and metadata. Cost belongs
    // to the original search; geometric quality is evaluated from actual nodes.
    return result;
}


std::optional<ROUTING_CONNECTION> MAZE_SEARCH_ENGINE::ShoveViaConnection(
        const ROUTING_CONNECTION& aConnection,
        const std::vector<ROUTING_CONNECTION>& aTransientObstacles,
        const ROUTER_CANCEL_CALLBACK& aCancel,
        const std::vector<ROUTING_CONNECTION>& aStaticDrillObstacles,
        const std::function<bool( const ROUTING_CONNECTION& )>& aPlacementFilter ) const
{
    // DrillItemMover can translate an unfixed drill item together with every
    // trace that contacts it.  The worker has no mutable item graph, so retain
    // the subsets whose replacement copper can be reconstructed exactly:
    //
    // - an isolated via is simply translated; and
    // - a via with one trace tail (or a generated terminal) on each side has
    //   those two legs rebuilt as direct/orthogonal doglegs.
    //
    // The static-host reconstruction marks only genuinely isolated source
    // vias eligible for the first case.  A branch, host pad contact, plane
    // contact, synthetic fanout terminal, or unsupported contact graph still
    // fails closed until it has the source item's complete normal-contact
    // mutation semantics.  Every returned edge is subsequently validated
    // against the transactional occupancy.
    if( aConnection.nodes.size() < 2 || aTransientObstacles.empty()
        || !HasValidEdgeStyles( aConnection ) )
    {
        return {};
    }

    const auto traceRadius = [&]( int aNetCode, const ROUTING_EDGE_STYLE& aStyle )
    {
        return aStyle.trackWidth > 0 ? aStyle.trackWidth / 2 : netTrackRadius( aNetCode );
    };
    const auto viaRadius = [&]( int aNetCode, const ROUTING_EDGE_STYLE& aStyle )
    {
        return aStyle.viaDiameter > 0 ? aStyle.viaDiameter / 2 : netViaRadius( aNetCode );
    };
    const auto viaDrillRadius = [&]( int aNetCode, const ROUTING_EDGE_STYLE& aStyle )
    {
        return aStyle.viaDrill > 0 ? aStyle.viaDrill / 2
                                   : netViaDrillRadius( aNetCode );
    };
    const auto doglegs = []( const ROUTER_NODE& aFrom, const ROUTER_POINT& aTo, int aLayer )
    {
        std::vector<std::vector<ROUTER_NODE>> result;

        if( aFrom.point == aTo )
        {
            result.emplace_back();
            return result;
        }

        // A moved DrillItem keeps the exact shape of each attached trace
        // whenever that straight leg remains legal.  Restricting every
        // non-axis-aligned relocation to two Manhattan doglegs discards a
        // valid general-angle shove merely because both artificial bend
        // corners touch nearby copper.  Try the direct leg first; strict
        // insertion below remains the authority for clearance, layer, and
        // board-edge legality.  The orthogonal alternatives are retained for
        // 90-degree routing modes and for obstacles that block the diagonal.
        result.push_back( { { aTo, aLayer } } );

        if( aFrom.point.x == aTo.x || aFrom.point.y == aTo.y )
            return result;

        result.push_back( { { { aTo.x, aFrom.point.y }, aLayer }, { aTo, aLayer } } );
        result.push_back( { { { aFrom.point.x, aTo.y }, aLayer }, { aTo, aLayer } } );
        return result;
    };

    const auto closestPointOnSegment = []( const ROUTER_POINT& aPoint,
                                           const ROUTER_POINT& aStart,
                                           const ROUTER_POINT& aEnd )
    {
        const long double dx = static_cast<long double>( aEnd.x ) - aStart.x;
        const long double dy = static_cast<long double>( aEnd.y ) - aStart.y;
        const long double lengthSquared = dx * dx + dy * dy;
        const long double factor = lengthSquared <= 0.0L
                ? 0.0L
                : std::clamp(
                          ( ( static_cast<long double>( aPoint.x ) - aStart.x ) * dx
                            + ( static_cast<long double>( aPoint.y ) - aStart.y ) * dy )
                                  / lengthSquared,
                          0.0L, 1.0L );
        return std::pair{ static_cast<long double>( aStart.x ) + factor * dx,
                          static_cast<long double>( aStart.y ) + factor * dy };
    };

    const auto roundedPoint = []( long double aX, long double aY )
            -> std::optional<ROUTER_POINT>
    {
        constexpr long double minimum = static_cast<long double>(
                std::numeric_limits<std::int64_t>::min() );
        constexpr long double maximum = static_cast<long double>(
                std::numeric_limits<std::int64_t>::max() );
        if( !std::isfinite( aX ) || !std::isfinite( aY ) || aX <= minimum || aX >= maximum
            || aY <= minimum || aY >= maximum )
        {
            return {};
        }

        return ROUTER_POINT{ static_cast<std::int64_t>( std::llround( aX ) ),
                             static_cast<std::int64_t>( std::llround( aY ) ) };
    };

    ROUTING_CONNECTION source = aConnection;
    EnsureEdgeStyles( source );

    const auto clearsTransientCopper = [&]( const ROUTING_CONNECTION& aCandidate )
    {
        // The source DrillItemMover checks the translated drill against the
        // complete temporary search-tree state before it commits the move.
        // In the immutable worker the incoming route is held separately from
        // board obstacles, so CanInsertSegment() cannot see it.  Reject a
        // locally legal via candidate if any transient foreign route still
        // overlaps it; otherwise the first nearest projection can poison the
        // whole shove even though a later projection is clear.
        return std::none_of( aTransientObstacles.begin(), aTransientObstacles.end(),
                             [&]( const ROUTING_CONNECTION& aObstacle )
                             { return connectionsConflict( aCandidate, aObstacle ); } );
    };

    const auto terminalAnchor = [&]( std::size_t aPadIndex, const ROUTER_NODE& aEndpoint )
            -> std::optional<ROUTER_NODE>
    {
        if( aPadIndex >= m_board.pads.size() )
            return {};

        const ROUTING_PAD& pad = m_board.pads[aPadIndex];
        // DrillItemMover only moves a via when every normal contact is a
        // trace or a conduction area.  A KiCad PAD is a fixed host item even
        // when its centre happens to coincide with the generated via, so it
        // must not become a movable terminal anchor just because the worker
        // snapshot represents both with ROUTING_PAD coordinates.  Data-only
        // generated terminals intentionally have no sourceId and remain the
        // limited testable subset supported by this reconstruction.
        if( pad.netCode != source.netCode || !pad.sourceId.empty() || pad.isFanoutTarget
            || pad.isPlaneTarget || !isOnPadLayer( pad, aEndpoint.layer )
            || pad.position != aEndpoint.point )
        {
            return {};
        }

        return ROUTER_NODE{ pad.position, aEndpoint.layer };
    };

    for( std::size_t viaIndex = 1; viaIndex < source.nodes.size(); ++viaIndex )
    {
        if( aCancel && aCancel() )
            return {};

        const ROUTER_NODE& viaStart = source.nodes[viaIndex - 1];
        const ROUTER_NODE& viaEnd = source.nodes[viaIndex];

        if( viaStart.layer == viaEnd.layer || viaStart.point != viaEnd.point )
        {
            continue;
        }

        // A source via with no normal contacts is a legal DrillItemMover
        // case: moveBy() translates the drill and adds no bridge traces.  It
        // is deliberately recognized before the two-sided reconstruction
        // below, which needs endpoints for both attachment legs.
        const bool standaloneVia = viaIndex == 1 && source.nodes.size() == 2;
        const bool hasBeforeTrace = viaIndex >= 2
                                    && source.nodes[viaIndex - 2].layer == viaStart.layer;
        const bool hasAfterTrace = viaIndex + 1 < source.nodes.size()
                                   && source.nodes[viaIndex + 1].layer == viaEnd.layer;

        std::optional<ROUTER_NODE> before;
        std::optional<ROUTER_NODE> after;
        ROUTING_EDGE_STYLE beforeStyle;
        ROUTING_EDGE_STYLE afterStyle;

        if( hasBeforeTrace )
        {
            before = source.nodes[viaIndex - 2];
            beforeStyle = source.edgeStyles[viaIndex - 2];
        }
        else if( viaIndex == 1 )
        {
            before = terminalAnchor( source.fromPadIndex, viaStart );
        }

        if( hasAfterTrace )
        {
            after = source.nodes[viaIndex + 1];
            afterStyle = source.edgeStyles[viaIndex];
        }
        else if( viaIndex + 1 == source.nodes.size() )
        {
            after = terminalAnchor( source.toPadIndex, viaEnd );
        }

        if( !standaloneVia
            && ( !before || !after || before->layer != viaStart.layer
                 || after->layer != viaEnd.layer ) )
            continue;

        const ROUTING_EDGE_STYLE& viaStyle = source.edgeStyles[viaIndex - 1];
        const std::vector<int> viaLayers = VIA_RULE::LayersFor(
                m_settings, viaStart.layer, viaEnd.layer, &viaStyle );
        if( viaLayers.empty() )
            continue;

        std::vector<ROUTER_POINT> candidates;
        const auto addCandidate = [&]( const ROUTER_POINT& aPoint )
        {
            if( aPoint != viaStart.point
                && std::find( candidates.begin(), candidates.end(), aPoint ) == candidates.end() )
            {
                candidates.push_back( aPoint );
            }
        };

        const auto addOffsetCandidate = [&]( long double aOriginX, long double aOriginY,
                                             long double aDirectionX, long double aDirectionY,
                                             std::int64_t aDistance )
        {
            const long double length = std::hypotl( aDirectionX, aDirectionY );
            if( length <= 0.0L || aDistance <= 0 )
                return;

            if( const auto point = roundedPoint(
                        aOriginX + aDirectionX * aDistance / length,
                        aOriginY + aDirectionY * aDistance / length ) )
            {
                addCandidate( *point );
            }
        };

        for( const ROUTING_CONNECTION& obstacleRoute : aTransientObstacles )
        {
            if( aCancel && aCancel() )
                return {};
            if( obstacleRoute.netCode == source.netCode || !HasValidEdgeStyles( obstacleRoute ) )
                continue;

            for( std::size_t edge = 1; edge < obstacleRoute.nodes.size(); ++edge )
            {
                const ROUTER_NODE& first = obstacleRoute.nodes[edge - 1];
                const ROUTER_NODE& second = obstacleRoute.nodes[edge];
                const ROUTING_EDGE_STYLE& obstacleStyle =
                        EdgeStyle( obstacleRoute, edge - 1 );
                const bool obstacleVia = first.layer != second.layer;
                std::int64_t clearance = 0;
                ROUTER_BOX bounds;

                if( obstacleVia )
                {
                    bool sharesLayer = false;
                    std::int64_t pair = 0;
                    for( int layer : viaLayers )
                    {
                        if( !VIA_RULE::SpansLayer( m_settings, first.layer, second.layer,
                                                   obstacleStyle, layer ) )
                        {
                            continue;
                        }

                        sharesLayer = true;
                        pair = std::max(
                                pair, edgePairClearance( source.netCode,
                                                         obstacleRoute.netCode, layer,
                                                         viaStyle.clearance,
                                                         obstacleStyle.clearance ) );
                    }

                    if( !sharesLayer )
                        continue;

                    clearance = std::max(
                            viaRadius( source.netCode, viaStyle )
                                    + viaRadius( obstacleRoute.netCode, obstacleStyle ) + pair,
                            viaDrillRadius( source.netCode, viaStyle )
                                    + viaDrillRadius( obstacleRoute.netCode, obstacleStyle )
                                    + m_board.holeToHoleClearance );
                    bounds = { first.point.x, first.point.y, first.point.x, first.point.y };

                    if( distance( viaStart.point, first.point ) > clearance )
                        continue;

                    // DrillItemMover asks its exact shape for the nearest
                    // outside locations.  A via is circular in the worker
                    // model, so its radial escape is exact; when the two
                    // centres coincide try the compass and diagonal choices
                    // rather than arbitrarily biasing the move to a box side.
                    const std::int64_t margin = std::max<std::int64_t>(
                            1, saturatedAdd( clearance, 2 ) );
                    const long double deltaX = static_cast<long double>( viaStart.point.x )
                                               - first.point.x;
                    const long double deltaY = static_cast<long double>( viaStart.point.y )
                                               - first.point.y;
                    if( std::hypotl( deltaX, deltaY ) > 0.0L )
                    {
                        addOffsetCandidate( first.point.x, first.point.y, deltaX, deltaY,
                                            margin );
                    }
                    else
                    {
                        for( const auto& direction :
                             std::array<std::array<int, 2>, 8>{
                                     std::array<int, 2>{ 1, 0 },
                                     std::array<int, 2>{ -1, 0 },
                                     std::array<int, 2>{ 0, 1 },
                                     std::array<int, 2>{ 0, -1 },
                                     std::array<int, 2>{ 1, 1 },
                                     std::array<int, 2>{ 1, -1 },
                                     std::array<int, 2>{ -1, 1 },
                                     std::array<int, 2>{ -1, -1 } } )
                        {
                            addOffsetCandidate( first.point.x, first.point.y, direction[0],
                                                direction[1], margin );
                        }
                    }
                }
                else
                {
                    if( !VIA_RULE::SpansLayer( m_settings, viaStart.layer, viaEnd.layer,
                                               viaStyle, first.layer ) )
                    {
                        continue;
                    }

                    clearance = viaRadius( source.netCode, viaStyle )
                                + traceRadius( obstacleRoute.netCode, obstacleStyle )
                                + edgePairClearance( source.netCode,
                                                     obstacleRoute.netCode, first.layer,
                                                     viaStyle.clearance,
                                                     obstacleStyle.clearance );
                    bounds = { std::min( first.point.x, second.point.x ),
                               std::min( first.point.y, second.point.y ),
                               std::max( first.point.x, second.point.x ),
                               std::max( first.point.y, second.point.y ) };

                    if( pointToSegmentDistance( viaStart.point, first.point, second.point )
                        > clearance )
                    {
                        continue;
                    }

                    // Box-edge candidates alone are not equivalent to
                    // DrillItemMover.tryShoveViaPoints for a diagonal trace:
                    // its nearest legal via location lies on the normal of
                    // the compensated segment.  Preserve that geometric
                    // candidate before adding broad box projections below.
                    const std::int64_t margin = std::max<std::int64_t>(
                            1, saturatedAdd( clearance, 2 ) );
                    const auto [closestX, closestY] =
                            closestPointOnSegment( viaStart.point, first.point, second.point );
                    const long double deltaX = static_cast<long double>( viaStart.point.x )
                                               - closestX;
                    const long double deltaY = static_cast<long double>( viaStart.point.y )
                                               - closestY;
                    const long double tangentX = static_cast<long double>( second.point.x )
                                                - first.point.x;
                    const long double tangentY = static_cast<long double>( second.point.y )
                                                - first.point.y;

                    if( std::hypotl( deltaX, deltaY ) > 0.0L )
                        addOffsetCandidate( closestX, closestY, deltaX, deltaY, margin );

                    // At a centre-line intersection the radial direction is
                    // undefined.  The two compensated trace normals are the
                    // source's nearest relative outside locations; retain
                    // both so a fixed item on one side does not force a
                    // needless rip-up.
                    addOffsetCandidate( closestX, closestY, -tangentY, tangentX, margin );
                    addOffsetCandidate( closestX, closestY, tangentY, -tangentX, margin );
                }

                const ROUTER_POINT projected{
                        std::clamp( viaStart.point.x, bounds.minX, bounds.maxX ),
                        std::clamp( viaStart.point.y, bounds.minY, bounds.maxY ) };
                const std::int64_t margin = std::max<std::int64_t>( 1, clearance + 1 );
                addCandidate( { saturatedAdd( bounds.minX, -margin ), projected.y } );
                addCandidate( { saturatedAdd( bounds.maxX, margin ), projected.y } );
                addCandidate( { projected.x, saturatedAdd( bounds.minY, -margin ) } );
                addCandidate( { projected.x, saturatedAdd( bounds.maxY, margin ) } );
            }
        }

        std::sort( candidates.begin(), candidates.end(), [&]( const ROUTER_POINT& aLeft,
                                                               const ROUTER_POINT& aRight )
        {
            const std::int64_t leftDistance = squaredDistance( viaStart.point, aLeft );
            const std::int64_t rightDistance = squaredDistance( viaStart.point, aRight );
            if( leftDistance != rightDistance )
                return leftDistance < rightDistance;
            if( aLeft.x != aRight.x )
                return aLeft.x < aRight.x;
            return aLeft.y < aRight.y;
        } );

        // DrillItemMover tries a bounded set of nearest projections. Keeping
        // the worker bound explicit prevents a large conflict fan-out from
        // turning one forced insertion into unbounded dogleg enumeration.
        constexpr std::size_t maxViaCandidates = 20;
        if( candidates.size() > maxViaCandidates )
            candidates.resize( maxViaCandidates );

        for( const ROUTER_POINT& candidate : candidates )
        {
            if( aCancel && aCancel() )
                return {};

            if( standaloneVia )
            {
                ROUTING_CONNECTION moved = source;
                moved.nodes = { { candidate, viaStart.layer }, { candidate, viaEnd.layer } };
                moved.edgeStyles = { viaStyle };

                const bool insertable = CanInsertSegment(
                        moved.netCode, moved.nodes.front(), moved.nodes.back(),
                        &moved.edgeStyles.front() );
                const bool clearsTransient = insertable && clearsTransientCopper( moved );
                const bool clearsDrills = clearsTransient
                        && hasStaticViaDrillClearance( moved, aStaticDrillObstacles );
                const bool preservesAreas = clearsDrills
                        && preservesConductionAreaContacts( source, moved );
                const bool passesPlacement = preservesAreas
                        && ( !aPlacementFilter || aPlacementFilter( moved ) );
                if( passesPlacement )
                {
                    return moved;
                }
                if( autorouterDebugEnabled() )
                {
                    std::ostringstream message;
                    message << "VIA_SHOVE_REJECTED from=(" << viaStart.point.x << ','
                            << viaStart.point.y << ") candidate=(" << candidate.x << ','
                            << candidate.y << ") insertable=" << insertable
                            << " transient=" << clearsTransient << " drills=" << clearsDrills
                            << " areas=" << preservesAreas << " placement="
                            << passesPlacement;
                    autorouterDebugLog( message.str() );
                }

                continue;
            }

            const auto beforeOptions = doglegs( *before, candidate, viaStart.layer );
            const auto afterOptions = doglegs( { candidate, viaEnd.layer }, after->point,
                                               viaEnd.layer );

            for( const auto& beforePath : beforeOptions )
            {
                for( const auto& afterPath : afterOptions )
                {
                    ROUTING_CONNECTION moved = source;
                    moved.nodes.clear();
                    moved.edgeStyles.clear();
                    const auto append = [&]( const ROUTER_NODE& aNode,
                                             const ROUTING_EDGE_STYLE* aStyle )
                    {
                        if( moved.nodes.empty() )
                        {
                            moved.nodes.push_back( aNode );
                            return true;
                        }

                        if( moved.nodes.back() == aNode )
                            return true;
                        if( !aStyle )
                            return false;
                        moved.nodes.push_back( aNode );
                        moved.edgeStyles.push_back( *aStyle );
                        return true;
                    };

                    if( !append( source.nodes.front(), nullptr ) )
                        continue;
                    bool valid = true;
                    const std::size_t prefixEnd = hasBeforeTrace ? viaIndex - 2 : viaIndex - 1;
                    for( std::size_t node = 1; node <= prefixEnd && valid; ++node )
                        valid = append( source.nodes[node], &source.edgeStyles[node - 1] );
                    for( const ROUTER_NODE& node : beforePath )
                        valid = valid && append( node, &beforeStyle );
                    valid = valid && append( { candidate, viaEnd.layer }, &viaStyle );
                    for( const ROUTER_NODE& node : afterPath )
                        valid = valid && append( node, &afterStyle );
                    const std::size_t suffixBegin = hasAfterTrace ? viaIndex + 2 : viaIndex + 1;
                    for( std::size_t node = suffixBegin; node < source.nodes.size() && valid;
                         ++node )
                    {
                        valid = append( source.nodes[node], &source.edgeStyles[node - 1] );
                    }

                    if( !valid || !HasValidEdgeStyles( moved ) || SameRouteGeometry( moved, source ) )
                        continue;

                    for( std::size_t edge = 1; edge < moved.nodes.size() && valid; ++edge )
                    {
                        valid = CanInsertSegment( moved.netCode, moved.nodes[edge - 1],
                                                  moved.nodes[edge], &moved.edgeStyles[edge - 1] );
                    }

                    if( valid && clearsTransientCopper( moved )
                        && hasStaticViaDrillClearance( moved, aStaticDrillObstacles )
                        && preservesConductionAreaContacts( source, moved )
                        && ( !aPlacementFilter || aPlacementFilter( moved ) ) )
                        return moved;
                }
            }
        }
    }

    return {};
}


std::optional<ROUTING_VIA_SHOVE_PLAN> MAZE_SEARCH_ENGINE::ShoveViaConnectionPlan(
        const ROUTING_CONNECTION& aConnection,
        const std::vector<ROUTING_CONNECTION>& aTransientObstacles,
        const std::vector<ROUTING_CONNECTION>& aContactCandidates,
        const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    // DrillItem.moveBy() preserves every normal trace contact by adding a
    // short old-centre-to-new-centre trace on that trace's layer/style.  The
    // path-only worker has to make the equivalent edit explicit: a retained
    // source trace is materialised into proposal copper, then a bridge joins
    // it to the translated via.
    //
    // Placement and contact mutation cannot be separated. DrillItemMover.check
    // validates the *translated drill plus its temporary board state* before
    // it accepts a candidate. In this worker a bridge is not implicit inside
    // the board item graph; returning the first via position and only later
    // discovering that its bridge crosses the incoming trace discarded an
    // otherwise legal later projection. Build and validate the complete plan
    // while ShoveViaConnection still enumerates ordered candidate centres.
    const auto isDirectStaticVia = []( const ROUTING_CONNECTION& aRoute )
    {
        return aRoute.isExistingBoardRoute && aRoute.sourceBoardItemIds.size() == 1
               && aRoute.nodes.size() == 2
               && aRoute.nodes.front().point == aRoute.nodes.back().point
               && aRoute.nodes.front().layer != aRoute.nodes.back().layer
               && HasValidEdgeStyles( aRoute ) && aRoute.edgeStyles.size() == 1;
    };

    // Generated drills and reconstructed trace-via-trace worker routes carry
    // every replacement edge in one connection, so their normal candidate
    // validation in ShoveViaConnection is already complete. Only an isolated
    // static board via has independent source-contact bridges to account for.
    if( !isDirectStaticVia( aConnection ) )
    {
        auto replacement = ShoveViaConnection( aConnection, aTransientObstacles, aCancel,
                                               aContactCandidates );
        if( !replacement )
            return {};

        ROUTING_VIA_SHOVE_PLAN result;
        result.replacement = std::move( *replacement );
        return result;
    }

    const ROUTING_EDGE_STYLE& viaStyle = aConnection.edgeStyles.front();
    const std::vector<int> viaLayers = VIA_RULE::LayersFor(
            m_settings, aConnection.nodes.front().layer, aConnection.nodes.back().layer,
            &viaStyle );
    if( viaLayers.empty() )
        return {};

    const ROUTER_POINT oldCenter = aConnection.nodes.front().point;
    const auto isViaLayer = [&]( int aLayer )
    {
        return std::find( viaLayers.begin(), viaLayers.end(), aLayer ) != viaLayers.end();
    };
    const auto hasNormalTraceEndpoint = [&]( const ROUTER_POINT& aStart,
                                              const ROUTER_POINT& aEnd )
    {
        // DrillItem.getNormalContacts() accepts a Trace only when the drill
        // centre equals its first or last corner. A through trace has no
        // TraceInfo and must not receive an invented bridge.
        return oldCenter == aStart || oldCenter == aEnd;
    };

    struct NORMAL_CONTACT
    {
        ROUTING_CONNECTION_REPLACEMENT materialization;
        int                            layer = -1;
        ROUTING_EDGE_STYLE             bridgeStyle;
    };

    std::vector<NORMAL_CONTACT> contacts;
    for( const ROUTING_CONNECTION& contact : aContactCandidates )
    {
        if( aCancel && aCancel() )
            return {};
        if( SameRouteGeometry( contact, aConnection ) || !contact.isExistingBoardRoute
            || contact.sourceBoardItemIds.empty() || contact.netCode != aConnection.netCode
            || contact.nodes.size() != 2
            || contact.nodes.front().layer != contact.nodes.back().layer
            || !HasValidEdgeStyles( contact ) || contact.edgeStyles.size() != 1 )
        {
            continue;
        }

        const int layer = contact.nodes.front().layer;
        if( !isViaLayer( layer )
            || !hasNormalTraceEndpoint( contact.nodes.front().point,
                                        contact.nodes.back().point ) )
        {
            continue;
        }

        // The caller combines live occupancy and its removed initial victims.
        // Materialize a source BOARD_ITEM just once even when it appears in
        // both views.
        if( std::any_of( contacts.begin(), contacts.end(),
                         [&]( const NORMAL_CONTACT& aExisting )
                         {
                             return SameRouteGeometry( aExisting.materialization.original,
                                                       contact );
                         } ) )
        {
            continue;
        }

        ROUTING_CONNECTION materialized = contact;
        materialized.isExistingBoardRoute = false;
        materialized.isAutorouterOwned = false;
        materialized.isShoveMovable = true;
        ROUTING_EDGE_STYLE bridgeStyle = contact.edgeStyles.front();
        bridgeStyle.trackWidth = ResolveTrackWidth( contact.netCode, bridgeStyle );
        contacts.push_back( { { contact, std::move( materialized ) }, layer,
                              std::move( bridgeStyle ) } );
    }

    const auto makePlan = [&]( const ROUTING_CONNECTION& aReplacement )
            -> std::optional<ROUTING_VIA_SHOVE_PLAN>
    {
        if( !isDirectStaticVia( aReplacement )
            || aReplacement.nodes.front().layer != aConnection.nodes.front().layer
            || aReplacement.nodes.back().layer != aConnection.nodes.back().layer
            || aReplacement.nodes.front().point != aReplacement.nodes.back().point
            || aReplacement.nodes.front().point == oldCenter )
        {
            return {};
        }

        ROUTING_VIA_SHOVE_PLAN result;
        result.replacement = aReplacement;
        const ROUTER_POINT newCenter = aReplacement.nodes.front().point;

        // DrillItem.TraceInfo.compareTo() keys its TreeSet by layer, not by
        // width or clearance class. Each contact stays materialized, but the
        // first stable trace style on a layer creates the sole bridge.
        std::set<int> bridgedLayers;
        for( const NORMAL_CONTACT& contact : contacts )
        {
            result.materializedContacts.push_back( contact.materialization );
            if( !bridgedLayers.insert( contact.layer ).second )
                continue;

            ROUTING_CONNECTION bridge;
            bridge.netCode = aConnection.netCode;
            bridge.complete = true;
            bridge.isShoveMovable = true;
            bridge.nodes = { { oldCenter, contact.layer }, { newCenter, contact.layer } };
            bridge.edgeStyles = { contact.bridgeStyle };
            result.bridges.push_back( std::move( bridge ) );
        }

        return result;
    };

    const auto routeFitsTemporaryBoard = [&]( const ROUTING_CONNECTION& aRoute )
    {
        if( !aRoute.complete || !HasValidEdgeStyles( aRoute ) )
            return false;

        for( std::size_t edge = 1; edge < aRoute.nodes.size(); ++edge )
        {
            if( !CanInsertSegment( aRoute.netCode, aRoute.nodes[edge - 1],
                                   aRoute.nodes[edge], &aRoute.edgeStyles[edge - 1] ) )
            {
                return false;
            }
        }

        return std::none_of( aTransientObstacles.begin(), aTransientObstacles.end(),
                             [&]( const ROUTING_CONNECTION& aObstacle )
                             { return connectionsConflict( aRoute, aObstacle ); } );
    };

    const auto planFitsTemporaryBoard = [&]( const ROUTING_CONNECTION& aReplacement )
    {
        if( aCancel && aCancel() )
            return false;

        const auto plan = makePlan( aReplacement );
        if( !plan || !routeFitsTemporaryBoard( plan->replacement ) )
            return false;

        for( const ROUTING_CONNECTION_REPLACEMENT& contact : plan->materializedContacts )
            if( !routeFitsTemporaryBoard( contact.replacement ) )
                return false;
        for( const ROUTING_CONNECTION& bridge : plan->bridges )
            if( !routeFitsTemporaryBoard( bridge ) )
                return false;
        return true;
    };

    auto replacement = ShoveViaConnection( aConnection, aTransientObstacles, aCancel,
                                           aContactCandidates, planFitsTemporaryBoard );
    if( !replacement )
        return {};

    return makePlan( *replacement );
}

bool MAZE_SEARCH_ENGINE::connectionsConflict( const ROUTING_CONNECTION& aCandidate,
                                              const ROUTING_CONNECTION& aExisting ) const
{
    if( aCandidate.netCode == aExisting.netCode || !HasValidEdgeStyles( aCandidate )
        || !HasValidEdgeStyles( aExisting ) )
    {
        return false;
    }

    const auto trackRadius = [&]( int aNetCode, const ROUTING_EDGE_STYLE& aStyle )
    {
        return aStyle.trackWidth > 0 ? aStyle.trackWidth / 2 : netTrackRadius( aNetCode );
    };
    const auto viaRadius = [&]( int aNetCode, const ROUTING_EDGE_STYLE& aStyle )
    {
        return aStyle.viaDiameter > 0 ? aStyle.viaDiameter / 2 : netViaRadius( aNetCode );
    };
    const auto viaDrillRadius = [&]( int aNetCode, const ROUTING_EDGE_STYLE& aStyle )
    {
        return aStyle.viaDrill > 0 ? aStyle.viaDrill / 2 : netViaDrillRadius( aNetCode );
    };
    const auto viaSpans = [&]( const ROUTER_NODE& aStart, const ROUTER_NODE& aEnd,
                               const ROUTING_EDGE_STYLE& aStyle, int aLayer )
    {
        return VIA_RULE::SpansLayer( m_settings, aStart.layer, aEnd.layer, aStyle, aLayer );
    };
    const auto edgeConflicts = [&]( const ROUTER_NODE& aLeftStart,
                                    const ROUTER_NODE& aLeftEnd,
                                    int aLeftNetCode,
                                    const ROUTING_EDGE_STYLE& aLeftStyle,
                                    const ROUTER_NODE& aRightStart,
                                    const ROUTER_NODE& aRightEnd,
                                    int aRightNetCode,
                                    const ROUTING_EDGE_STYLE& aRightStyle )
    {
        const bool leftVia = aLeftStart.layer != aLeftEnd.layer;
        const bool rightVia = aRightStart.layer != aRightEnd.layer;

        if( !leftVia && !rightVia )
        {
            if( aLeftStart.layer != aRightStart.layer )
                return false;

            const std::int64_t clearance = trackRadius( aLeftNetCode, aLeftStyle )
                                           + trackRadius( aRightNetCode, aRightStyle )
                                           + edgePairClearance( aLeftNetCode, aRightNetCode,
                                                                 aLeftStart.layer,
                                                                 aLeftStyle.clearance,
                                                                 aRightStyle.clearance );
            return segmentsWithinClearance( aLeftStart.point, aLeftEnd.point,
                                            aRightStart.point, aRightEnd.point,
                                            static_cast<double>( clearance ) );
        }

        if( leftVia && rightVia )
        {
            std::int64_t pair = 0;
            bool sharesLayer = false;
            for( const ROUTER_LAYER_SETTINGS& layer : m_settings.layers )
            {
                if( !viaSpans( aLeftStart, aLeftEnd, aLeftStyle, layer.layerId )
                    || !viaSpans( aRightStart, aRightEnd, aRightStyle, layer.layerId ) )
                {
                    continue;
                }

                sharesLayer = true;
                pair = std::max( pair, edgePairClearance( aLeftNetCode, aRightNetCode,
                                                           layer.layerId,
                                                           aLeftStyle.clearance,
                                                           aRightStyle.clearance ) );
            }

            if( !sharesLayer )
                return false;

            const std::int64_t copperClearance = viaRadius( aLeftNetCode, aLeftStyle )
                                                 + viaRadius( aRightNetCode, aRightStyle ) + pair;
            const std::int64_t drillClearance = viaDrillRadius( aLeftNetCode, aLeftStyle )
                                                + viaDrillRadius( aRightNetCode, aRightStyle )
                                                + m_board.holeToHoleClearance;
            return distance( aLeftStart.point, aRightStart.point )
                   <= static_cast<double>( std::max( copperClearance, drillClearance ) );
        }

        const ROUTER_NODE& viaStart = leftVia ? aLeftStart : aRightStart;
        const int viaNetCode = leftVia ? aLeftNetCode : aRightNetCode;
        const ROUTING_EDGE_STYLE& viaStyle = leftVia ? aLeftStyle : aRightStyle;
        const ROUTER_NODE& trackStart = leftVia ? aRightStart : aLeftStart;
        const ROUTER_NODE& trackEnd = leftVia ? aRightEnd : aLeftEnd;
        const int trackNetCode = leftVia ? aRightNetCode : aLeftNetCode;
        const ROUTING_EDGE_STYLE& trackStyle = leftVia ? aRightStyle : aLeftStyle;

        if( !viaSpans( viaStart, leftVia ? aLeftEnd : aRightEnd, viaStyle,
                        trackStart.layer ) )
        {
            return false;
        }

        const std::int64_t clearance = viaRadius( viaNetCode, viaStyle )
                                       + trackRadius( trackNetCode, trackStyle )
                                       + edgePairClearance( viaNetCode, trackNetCode,
                                                             trackStart.layer,
                                                             viaStyle.clearance,
                                                             trackStyle.clearance );
        return pointToSegmentDistance( viaStart.point, trackStart.point, trackEnd.point )
               <= static_cast<double>( clearance );
    };

    for( std::size_t left = 1; left < aCandidate.nodes.size(); ++left )
    {
        for( std::size_t right = 1; right < aExisting.nodes.size(); ++right )
        {
            if( edgeConflicts( aCandidate.nodes[left - 1], aCandidate.nodes[left],
                               aCandidate.netCode, EdgeStyle( aCandidate, left - 1 ),
                               aExisting.nodes[right - 1], aExisting.nodes[right],
                               aExisting.netCode, EdgeStyle( aExisting, right - 1 ) ) )
            {
                return true;
            }
        }
    }

    return false;
}


std::vector<ROUTING_CONNECTION> MAZE_SEARCH_ENGINE::FindConflictingConnections(
        const ROUTING_CONNECTION& aCandidate ) const
{
    std::vector<ROUTING_CONNECTION> result;

    for( const ROUTING_CONNECTION& existing : m_occupancy.Connections() )
    {
        if( connectionsConflict( aCandidate, existing ) )
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
    result.reserve( aStart.layers.size() + aTarget.layers.size() + 512 );

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

    // The immutable board-wide graph is indexed separately below. Add only
    // connection-local exact convex corners here; otherwise copying the
    // whole graph into this vector obscures the small set of per-net features
    // from adaptiveNeighbours and makes every frontier expansion rescan it.
    // A rectangle has an exact room representation already. For a strict
    // convex contour, however, its L-infinity offset preserves free wedges
    // that its axis-aligned bounding box would erase. A one-IU guard keeps
    // an integral support intersection outside the host's closed collision
    // boundary; rational intersections are deliberately not rounded.
    //
    // Do not stop at the first 512 corners in snapshot order. KiCad writes
    // zones/tracks in board-item order, not in relation to this connection;
    // on a dense board that made a late convex blocker invisible while an
    // unrelated footprint monopolized every dynamic landmark slot. Retain a
    // bounded reservoir ranked by distance to this connection's direct
    // corridor, then validate/support the best integral siblings below.
    constexpr std::size_t maxDynamicLandmarks = 512;
    constexpr std::size_t maxRawConvexLandmarks = 4096;
    constexpr std::size_t retainedRawConvexLandmarks = 2048;
    struct CONVEX_LANDMARK_CANDIDATE
    {
        long double corridorDistanceSquared = 0.0L;
        long double endpointDistanceSquared = 0.0L;
        ROUTER_NODE node;
    };
    std::vector<CONVEX_LANDMARK_CANDIDATE> convexCandidates;
    convexCandidates.reserve( retainedRawConvexLandmarks );
    const auto scoreConvexCandidate = [&]( const ROUTER_POINT& aPoint )
    {
        const long double startX = static_cast<long double>( aStart.position.x );
        const long double startY = static_cast<long double>( aStart.position.y );
        const long double endX = static_cast<long double>( aTarget.position.x );
        const long double endY = static_cast<long double>( aTarget.position.y );
        const long double pointX = static_cast<long double>( aPoint.x );
        const long double pointY = static_cast<long double>( aPoint.y );
        const long double directionX = endX - startX;
        const long double directionY = endY - startY;
        const long double lengthSquared = directionX * directionX + directionY * directionY;
        const long double startDistanceSquared = ( pointX - startX ) * ( pointX - startX )
                                               + ( pointY - startY ) * ( pointY - startY );
        const long double endDistanceSquared = ( pointX - endX ) * ( pointX - endX )
                                             + ( pointY - endY ) * ( pointY - endY );
        if( lengthSquared <= 0.0L )
            return std::pair{ startDistanceSquared, startDistanceSquared };

        const long double projection = std::clamp(
                ( ( pointX - startX ) * directionX + ( pointY - startY ) * directionY )
                        / lengthSquared,
                0.0L, 1.0L );
        const long double closestX = startX + projection * directionX;
        const long double closestY = startY + projection * directionY;
        const long double corridorDistanceSquared = ( pointX - closestX ) * ( pointX - closestX )
                                                   + ( pointY - closestY )
                                                             * ( pointY - closestY );
        return std::pair{ corridorDistanceSquared,
                          std::min( startDistanceSquared, endDistanceSquared ) };
    };
    const auto compareConvexCandidates = []( const CONVEX_LANDMARK_CANDIDATE& aLeft,
                                             const CONVEX_LANDMARK_CANDIDATE& aRight )
    {
        if( aLeft.corridorDistanceSquared != aRight.corridorDistanceSquared )
            return aLeft.corridorDistanceSquared < aRight.corridorDistanceSquared;
        if( aLeft.endpointDistanceSquared != aRight.endpointDistanceSquared )
            return aLeft.endpointDistanceSquared < aRight.endpointDistanceSquared;
        if( aLeft.node.point.x != aRight.node.point.x )
            return aLeft.node.point.x < aRight.node.point.x;
        if( aLeft.node.point.y != aRight.node.point.y )
            return aLeft.node.point.y < aRight.node.point.y;
        return aLeft.node.layer < aRight.node.layer;
    };
    const auto addConvexCandidate = [&]( const ROUTER_POINT& aPoint, int aLayer )
    {
        if( aLayer < 0 )
            return;

        const auto [corridorDistanceSquared, endpointDistanceSquared] =
                scoreConvexCandidate( aPoint );
        convexCandidates.push_back( { corridorDistanceSquared, endpointDistanceSquared,
                                      { aPoint, aLayer } } );
        if( convexCandidates.size() > maxRawConvexLandmarks )
        {
            std::nth_element( convexCandidates.begin(),
                              convexCandidates.begin()
                                      + static_cast<std::ptrdiff_t>( retainedRawConvexLandmarks ),
                              convexCandidates.end(), compareConvexCandidates );
            convexCandidates.resize( retainedRawConvexLandmarks );
        }
    };

    for( const ROUTING_OBSTACLE& obstacle : m_board.obstacles )
    {
        if( obstacle.kind != ROUTER_OBSTACLE_KIND::POLYGON || obstacle.isHole
            || !obstacle.blocksTracks || !obstacle.polygonHoles.empty()
            || obstacle.radius != 0
            || ( obstacle.netCode == aNetCode && !obstacle.isKeepout ) )
        {
            continue;
        }

        for( const ROUTER_LAYER_SETTINGS& layer : m_settings.layers )
        {
            if( !layer.enabled
                || ( !obstacle.layers.empty()
                     && std::find( obstacle.layers.begin(), obstacle.layers.end(), layer.layerId )
                                == obstacle.layers.end() ) )
            {
                continue;
            }

            const std::int64_t radius = obstacleExpansionRadius(
                    obstacle, aNetCode, layer.layerId, false,
                    netTrackRadius( aNetCode ) );
            if( radius == std::numeric_limits<std::int64_t>::max() )
                continue;

            const auto simplex = PLANAR::SIMPLEX::FromConvexPolygon(
                    obstacle.polygon, radius + 1 );
            if( !simplex )
                continue;

            for( std::size_t index = 0; index < simplex->Borders().size(); ++index )
            {
                // A source support-line intersection may be rational. KiCad
                // cannot emit fractional IU coordinates, but dropping the
                // corner altogether forces an otherwise exact convex route
                // back onto the coarse grid. Enumerate the at-most four
                // surrounding integer points and retain only candidates that
                // pass the full compensated point predicate. This keeps the
                // exact support geometry as the oracle and never rounds a
                // corner through copper merely to make it integral.
                const auto bounds = simplex->Corner( index ).SurroundingBox();
                if( !bounds )
                    continue;

                for( const std::int64_t x : { bounds->minX, bounds->maxX } )
                {
                    for( const std::int64_t y : { bounds->minY, bounds->maxY } )
                    {
                        const ROUTER_POINT candidate{ x, y };
                        addConvexCandidate( candidate, layer.layerId );
                    }
                }
            }
        }
    }

    std::stable_sort( convexCandidates.begin(), convexCandidates.end(), compareConvexCandidates );
    convexCandidates.erase(
            std::unique( convexCandidates.begin(), convexCandidates.end(),
                         []( const CONVEX_LANDMARK_CANDIDATE& aLeft,
                             const CONVEX_LANDMARK_CANDIDATE& aRight )
                         { return aLeft.node == aRight.node; } ),
            convexCandidates.end() );
    for( const CONVEX_LANDMARK_CANDIDATE& candidate : convexCandidates )
    {
        if( result.size() >= maxDynamicLandmarks )
            break;

        if( isPointAllowed( candidate.node.point, candidate.node.layer, aNetCode, false,
                            netTrackRadius( aNetCode ) ) )
        {
            add( candidate.node.point, candidate.node.layer );
        }
    }

    return result;
}

std::vector<ROUTER_NODE> MAZE_SEARCH_ENGINE::adaptiveNeighbours(
        const ROUTER_NODE& aNode, const ROUTING_PAD& aTarget,
        const std::vector<ROUTER_NODE>& aLandmarks, int aNetCode ) const
{
    std::vector<ROUTER_NODE> result = neighbours( aNode );
    std::vector<std::pair<long double, ROUTER_NODE>> nearbyCandidates;
    std::vector<std::pair<long double, ROUTER_NODE>> localCandidates;
    nearbyCandidates.reserve( 128 );
    localCandidates.reserve( std::min<std::size_t>( aLandmarks.size(), 128 ) );

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
                        nearbyCandidates.emplace_back( distanceSquared, landmark );
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
    if( nearbyCandidates.size() < 8 )
        collectNearbyLandmarks( 8 );

    // buildLandmarks intentionally contains only connection-local terminals
    // and exact convex support corners. Keep them in a distinct reserve until
    // the dense-board cap is applied below: a convex support corner is a
    // faithful escape door, not optional board-wide decoration. Letting
    // thousands of nearby pad/zone landmarks consume the cap first forces a
    // general-convex connection back onto the coarse grid even though its
    // exact support corner was already available.
    for( const ROUTER_NODE& landmark : aLandmarks )
    {
        if( landmark.layer != aNode.layer || landmark == aNode )
            continue;

        const long double dx = static_cast<long double>( landmark.point.x ) - aNode.point.x;
        const long double dy = static_cast<long double>( landmark.point.y ) - aNode.point.y;
        const long double distanceSquared = dx * dx + dy * dy;
        if( distanceSquared != 0.0L )
            localCandidates.emplace_back( distanceSquared, landmark );
    }

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
    const auto sortAndUnique = [&]( auto& aCandidates )
    {
        std::stable_sort( aCandidates.begin(), aCandidates.end(), compareCandidates );
        aCandidates.erase( std::unique( aCandidates.begin(), aCandidates.end(),
                                        []( const auto& aLeft, const auto& aRight )
                                        { return aLeft.second == aRight.second; } ),
                           aCandidates.end() );
    };
    const auto trimNearest = [&]( auto& aCandidates, std::size_t aMaximum )
    {
        if( aCandidates.size() <= aMaximum )
            return;

        std::nth_element( aCandidates.begin(),
                          aCandidates.begin() + static_cast<std::ptrdiff_t>( aMaximum ),
                          aCandidates.end(), compareCandidates );
        aCandidates.resize( aMaximum );
        std::stable_sort( aCandidates.begin(), aCandidates.end(), compareCandidates );
    };

    const bool denseBoard = m_board.obstacles.size() > 2000 || m_board.pads.size() > 300;
    sortAndUnique( localCandidates );
    const bool hasExtraLocalDoors = localCandidates.size() > 1;

    // The ordinary visibility graph stays deliberately small on dense
    // boards. Reserve a handful of slots for per-connection landmarks,
    // however; these include the exact offset corners of a non-rectangular
    // convex obstacle and are the only geometrically faithful door choices
    // when the rectangular room engine declines that layer.
    const std::size_t localReserve = denseBoard && hasExtraLocalDoors ? 24 : 0;
    const std::size_t maxVisibilityCandidates = denseBoard ? 24 : 128;
    trimNearest( localCandidates,
                 localReserve == 0 ? maxVisibilityCandidates : localReserve );

    trimNearest( nearbyCandidates, maxVisibilityCandidates - localCandidates.size() );

    std::vector<std::pair<long double, ROUTER_NODE>> candidates;
    candidates.reserve( localCandidates.size() + nearbyCandidates.size() );
    candidates.insert( candidates.end(), localCandidates.begin(), localCandidates.end() );
    candidates.insert( candidates.end(), nearbyCandidates.begin(), nearbyCandidates.end() );
    sortAndUnique( candidates );

    const std::size_t maxVisibleLandmarks = denseBoard
            ? ( hasExtraLocalDoors ? 24 : 2 ) : 24;
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
                                    const std::vector<ROUTING_TERMINAL>& aTargets,
                                    bool aAllowLegacyFallback ) const
{
    aExpandedNodes = 0;
    m_roomMetrics = {};
    ROUTING_TERMINAL defaultStart;
    defaultStart.pad = aStart;
    ROUTING_TERMINAL defaultTarget;
    defaultTarget.pad = aTarget;
    const std::vector<ROUTING_TERMINAL> starts = aStarts.empty()
            ? std::vector<ROUTING_TERMINAL>{ defaultStart } : aStarts;
    const std::vector<ROUTING_TERMINAL> targets = aTargets.empty()
            ? std::vector<ROUTING_TERMINAL>{ defaultTarget } : aTargets;
    // Explicit terminal sets normally describe a non-fanout caller's real
    // destinations.  BatchFanout is the exception: its synthetic control has
    // a valid source-pad identity and names a stop-at-first-drill search whose
    // actual item targets are supplied separately.
    const bool fanoutSearch = aTarget.isFanoutTarget
                              && aTarget.fanoutSourceLayer >= 0
                              && aTarget.fanoutTargetLayer >= 0
                              && ( aTargets.empty()
                                   || aTarget.fanoutSourcePadIndex < m_board.pads.size() );
    for( const auto& terminal : starts )
        if( terminal.pad.netCode != aStart.netCode )
            return std::nullopt;
    for( const auto& terminal : targets )
        if( terminal.pad.netCode != aStart.netCode )
            return std::nullopt;
    // Freerouting's AutorouteConnectionRouter enables rip-up on every
    // ordinary routing pass.  aRetry is zero-based in the native batch loop;
    // allowRipupOnFirstIteration therefore controls its pass-one equivalent.
    m_allowRipupOccupancy = m_settings.allowRipupRouted
                            && ( aRetry > 0 || m_settings.allowRipupOnFirstIteration );
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
                               aTarget.isPlaneTarget,
                               std::max( netTrackRadius( aStart.netCode ),
                                         netViaRadius( aStart.netCode ) ),
                               isPureSmdNet( aStart.netCode ) );

    // The rectangular room frontier now compares same-layer routes and
    // through-drill alternatives in one queue. A fanout attempt uses the same
    // room/drill frontier but terminates at its first layer transition, just
    // like AutorouteEngine with is_fanout. Production callers stop when this
    // translated room search rejects a route; they do not substitute the
    // older raster/visibility implementation.
    const auto enabledLayers = std::count_if( m_settings.layers.begin(), m_settings.layers.end(),
                                             []( const auto& layer ) { return layer.enabled; } );
    m_useRoutableObstacleRooms = m_allowRipupOccupancy;
    auto roomPath = !m_settings.allowVias || enabledLayers == 1
                            ? findRoomConnection( starts, targets, aRetry, aExpandedNodes,
                                                  aCancel, aProgress )
                            : findMultilayerRoomConnection( starts, targets, aRetry,
                                                            aExpandedNodes, aCancel, aProgress,
                                                            fanoutSearch ? &aTarget : nullptr );

    m_useRoutableObstacleRooms = false;

    if( roomPath )
        return roomPath;
    if( aCancel && aCancel() )
        return std::nullopt;
    if( !aAllowLegacyFallback )
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
                if( fanoutSearch && layer.layerId != aTarget.fanoutSourceLayer )
                    continue;
                if( fanoutSearch && aTarget.fanoutMaxEscapeLength > 0
                    && distance( point, aStart.position )
                               > static_cast<double>( aTarget.fanoutMaxEscapeLength ) )
                {
                    continue;
                }
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
        const auto fanoutDrillParent = fanoutSearch
                                               && current.node.layer
                                                          != aTarget.fanoutSourceLayer
                ? cameFrom.find( current.node )
                : cameFrom.end();
        const bool reachedFanoutDrill =
                fanoutDrillParent != cameFrom.end()
                && fanoutDrillParent->second.layer == aTarget.fanoutSourceLayer
                && fanoutDrillParent->second.point == current.node.point;

        // A fanout search terminates at its first drill even when the drill
        // happens to touch a destination item on the exit layer.  Checking
        // the ordinary destination first incorrectly appended the rest of a
        // plane/trace target and turned one fanout operation into a complete
        // net route.
        if( destination != targets.end() && !reachedFanoutDrill )
        {
            ROUTING_CONNECTION result;
            result.netCode = aStart.netCode;
            result.complete = true;
            result.isFanoutConnection = fanoutSearch;
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

            if( !assignViaStyles( result ) )
                continue;

            logSearchState( "END search route found" );
            return result;
        }

        if( reachedFanoutDrill )
        {
            ROUTING_CONNECTION result;
            result.netCode = aStart.netCode;
            result.complete = true;
            result.isFanoutConnection = true;
            result.cost = current.g;

            ROUTER_NODE cursor = current.node;
            while( true )
            {
                result.nodes.push_back( cursor );
                const auto previous = cameFrom.find( cursor );
                if( previous == cameFrom.end() )
                    break;
                cursor = previous->second;
            }
            std::reverse( result.nodes.begin(), result.nodes.end() );
            result.fromPadIndex = startOwners.at( result.nodes.front() );

            if( !assignViaStyles( result ) )
                continue;

            logSearchState( "END fanout search at first drill" );
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

            if( fanoutSearch && next.layer == aTarget.fanoutSourceLayer
                && aTarget.fanoutMaxEscapeLength > 0
                && distance( next.point, aStart.position )
                           > static_cast<double>( aTarget.fanoutMaxEscapeLength ) )
            {
                continue;
            }

            if( via )
            {
                if( fanoutSearch
                    && distance( current.node.point, aStart.position )
                               < static_cast<double>( std::max<std::int64_t>(
                                       0, aTarget.fanoutMinEscapeLength ) ) )
                {
                    continue;
                }
                // ViaInfo.attachSmdAllowed is profile-local.  A later entry
                // may legally attach even when an earlier one cannot.
                const bool attachesToSmd = std::any_of(
                        starts.begin(), starts.end(), [&]( const auto& terminal )
                        {
                            return terminal.pad.isSmd
                                   && terminal.pad.position == current.node.point;
                        } );
                if( !SelectViaStyle( aStart.netCode, current.node, next,
                                     attachesToSmd ) )
                    continue;
            }
            else if( !isSegmentAllowedFromKnownStart( current.node.point, next.point, next.layer,
                                                      aStart.netCode, false ) )
            {
                continue;
            }

            const int usage = via ? 0
                                   : m_occupancy.SegmentUsage( current.node, next,
                                                                aStart.netCode );
            const double bend = ( !via && current.node.point != next.point )
                                        ? static_cast<double>( m_settings.bendCost )
                                        : 0.0;
            const double moveCost = via ? control.ViaCost()
                                        : control.TraceCost( distance( current.node.point,
                                                                       next.point ) )
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
