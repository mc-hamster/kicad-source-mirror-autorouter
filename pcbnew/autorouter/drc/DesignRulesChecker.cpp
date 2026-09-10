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

#include "DesignRulesChecker.h"

#include "../AutorouterDebug.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <sstream>
#include <unordered_set>


namespace KICAD_AUTOROUTER
{

namespace
{

double squaredDistance( const ROUTER_POINT& aLeft, const ROUTER_POINT& aRight )
{
    const long double dx = static_cast<long double>( aLeft.x ) - aRight.x;
    const long double dy = static_cast<long double>( aLeft.y ) - aRight.y;
    return static_cast<double>( dx * dx + dy * dy );
}


double distance( const ROUTER_POINT& aLeft, const ROUTER_POINT& aRight )
{
    return std::sqrt( std::max( 0.0, squaredDistance( aLeft, aRight ) ) );
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


bool pointInPolygon( const ROUTER_POINT& aPoint,
                     const std::vector<ROUTER_POINT>& aPolygon )
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

        if( ( ( a.y > aPoint.y ) != ( b.y > aPoint.y ) )
            && static_cast<long double>( b.x - a.x ) * ( aPoint.y - a.y )
                               / static_cast<long double>( b.y - a.y )
                           + a.x
                                  > aPoint.x )
        {
            inside = !inside;
        }
    }

    return inside;
}


bool pointInPolygonWithHoles( const ROUTER_POINT& aPoint,
                              const ROUTING_OBSTACLE& aObstacle )
{
    if( !pointInPolygon( aPoint, aObstacle.polygon ) )
        return false;

    return std::none_of( aObstacle.polygonHoles.begin(), aObstacle.polygonHoles.end(),
                         [&]( const std::vector<ROUTER_POINT>& aHole )
                         {
                             return pointInPolygon( aPoint, aHole );
                         } );
}


bool pointNearPolygon( const ROUTER_POINT& aPoint,
                       const std::vector<ROUTER_POINT>& aPolygon, double aRadius )
{
    if( aPolygon.empty() )
        return false;

    for( std::size_t i = 0; i < aPolygon.size(); ++i )
    {
        if( pointToSegmentDistance( aPoint, aPolygon[i],
                                    aPolygon[( i + 1 ) % aPolygon.size()] )
            <= aRadius )
        {
            return true;
        }
    }

    return false;
}


bool layerContains( const std::vector<int>& aLayers, int aLayer )
{
    return aLayers.empty()
           || std::find( aLayers.begin(), aLayers.end(), aLayer ) != aLayers.end();
}


bool spansLayer( const ROUTING_VIA& aVia, int aLayer )
{
    if( !aVia.layers.empty() )
        return std::find( aVia.layers.begin(), aVia.layers.end(), aLayer ) != aVia.layers.end();

    const int top = std::min( aVia.topLayer, aVia.bottomLayer );
    const int bottom = std::max( aVia.topLayer, aVia.bottomLayer );
    return aLayer >= top && aLayer <= bottom;
}


std::vector<int> viaLayerSpan( const ROUTING_VIA& aVia )
{
    if( !aVia.layers.empty() )
        return aVia.layers;

    const int top = std::min( aVia.topLayer, aVia.bottomLayer );
    const int bottom = std::max( aVia.topLayer, aVia.bottomLayer );
    std::vector<int> result;
    for( int layer = top; layer <= bottom; ++layer )
        result.push_back( layer );
    return result;
}


const ROUTING_NET* findNet( const BOARD_SNAPSHOT& aBoard, int aNetCode )
{
    auto it = std::find_if( aBoard.nets.begin(), aBoard.nets.end(),
                            [aNetCode]( const ROUTING_NET& aNet )
                            {
                                return aNet.netCode == aNetCode;
                            } );
    return it == aBoard.nets.end() ? nullptr : &*it;
}


std::int64_t trackRadius( const BOARD_SNAPSHOT& aBoard, int aNetCode,
                          std::int64_t aWidth = 0 )
{
    constexpr std::int64_t defaultTrackWidth = 150000;
    const ROUTING_NET* net = findNet( aBoard, aNetCode );
    std::int64_t width = aWidth;
    if( width <= 0 && net )
    {
        for( std::size_t padIndex : net->padIndices )
        {
            if( padIndex < aBoard.pads.size() )
                width = std::max( width, aBoard.pads[padIndex].trackWidth );
        }
    }

    if( width <= 0 )
        width = defaultTrackWidth;

    return std::max<std::int64_t>( 1, width / 2 );
}


std::int64_t viaRadius( const BOARD_SNAPSHOT& aBoard, int aNetCode,
                        std::int64_t aDiameter )
{
    const ROUTING_NET* net = findNet( aBoard, aNetCode );
    const std::int64_t diameter = aDiameter > 0 ? aDiameter : ( net ? net->viaDiameter : 0 );
    return std::max<std::int64_t>( 1, diameter / 2 );
}


std::int64_t viaDrillRadius( const BOARD_SNAPSHOT& aBoard, int aNetCode,
                             std::int64_t aDrill )
{
    const ROUTING_NET* net = findNet( aBoard, aNetCode );
    const std::int64_t drill = aDrill > 0 ? aDrill : ( net ? net->viaDrill : 0 );
    return std::max<std::int64_t>( 1, drill / 2 );
}


std::int64_t netClearance( const BOARD_SNAPSHOT& aBoard, int aNetCode )
{
    const ROUTING_NET* net = findNet( aBoard, aNetCode );
    return net ? std::max<std::int64_t>( 0, net->clearance ) : 0;
}


std::int64_t pairClearance( const BOARD_SNAPSHOT& aBoard, int aFirstNetCode,
                            int aSecondNetCode, int aLayer = -1 )
{
    if( aFirstNetCode == aSecondNetCode )
        return 0;

    std::int64_t result = std::max( netClearance( aBoard, aFirstNetCode ),
                                    netClearance( aBoard, aSecondNetCode ) );

    for( const ROUTING_CLEARANCE_RULE& rule : aBoard.clearanceRules )
    {
        const bool samePair =
                ( rule.firstNetCode == aFirstNetCode && rule.secondNetCode == aSecondNetCode )
                || ( rule.firstNetCode == aSecondNetCode
                     && rule.secondNetCode == aFirstNetCode );

        if( samePair && ( rule.layer < 0 || aLayer < 0 || rule.layer == aLayer ) )
            result = std::max( result, std::max<std::int64_t>( 0, rule.clearance ) );
    }

    return result;
}


std::int64_t obstacleClearance( const BOARD_SNAPSHOT& aBoard, int aNetCode,
                                const ROUTING_OBSTACLE& aObstacle, int aLayer,
                                std::int64_t aEdgeClearance = 0 )
{
    const std::int64_t edgeClearance = std::max<std::int64_t>( 0, aEdgeClearance );
    if( aObstacle.netCode != 0 && aObstacle.netCode != aNetCode )
        return std::max( pairClearance( aBoard, aNetCode, aObstacle.netCode, aLayer ),
                         std::max( aObstacle.clearance, edgeClearance ) );

    return std::max( { netClearance( aBoard, aNetCode ), aObstacle.clearance,
                       edgeClearance } );
}


bool insideBoard( const BOARD_SNAPSHOT& aBoard, const ROUTER_POINT& aPoint,
                  std::int64_t aMargin )
{
    if( !aBoard.bounds.Contains( aPoint ) )
        return false;

    // The board outline is optional in the snapshot because KiCad permits
    // transient/imported boards without a closed edge polygon.  The bounding
    // box is still a valid conservative boundary in that case.
    if( aBoard.boardOutline.size() < 3 && aMargin > 0
        && ( aPoint.x <= aBoard.bounds.minX + aMargin
             || aPoint.x >= aBoard.bounds.maxX - aMargin
             || aPoint.y <= aBoard.bounds.minY + aMargin
             || aPoint.y >= aBoard.bounds.maxY - aMargin ) )
    {
        return false;
    }

    if( !aBoard.boardOutline.empty() && !pointInPolygon( aPoint, aBoard.boardOutline ) )
        return false;

    for( const std::vector<ROUTER_POINT>& hole : aBoard.boardHoles )
    {
        if( pointInPolygon( aPoint, hole ) )
            return false;
    }

    auto near = [&]( const std::vector<ROUTER_POINT>& aPolygon )
    {
        return pointNearPolygon( aPoint, aPolygon, static_cast<double>( aMargin ) );
    };

    if( aMargin > 0
        && ( near( aBoard.boardOutline )
             || std::any_of( aBoard.boardHoles.begin(), aBoard.boardHoles.end(), near ) ) )
    {
        return false;
    }

    return true;
}


std::int64_t endpointMargin( const BOARD_SNAPSHOT& aBoard, int aNetCode,
                             const ROUTER_POINT& aPoint, std::int64_t aTrackRadius )
{
    std::int64_t result = aTrackRadius;

    for( const ROUTING_NET& net : aBoard.nets )
    {
        if( net.netCode != aNetCode )
            continue;

        for( std::size_t padIndex : net.padIndices )
        {
            if( padIndex >= aBoard.pads.size() )
                continue;

            const ROUTING_PAD& pad = aBoard.pads[padIndex];
            if( pad.position == aPoint )
                result = std::max( result, pad.radius + pad.clearance );
        }
    }

    return result;
}


bool isPadEndpoint( const BOARD_SNAPSHOT& aBoard, int aNetCode,
                    const ROUTER_POINT& aPoint )
{
    const ROUTING_NET* net = findNet( aBoard, aNetCode );
    if( !net )
        return false;

    return std::any_of( net->padIndices.begin(), net->padIndices.end(),
                        [&]( std::size_t aPadIndex )
                        {
                            return aPadIndex < aBoard.pads.size()
                                   && aBoard.pads[aPadIndex].position == aPoint;
                        } );
}


bool segmentInsideBoard( const BOARD_SNAPSHOT& aBoard, const ROUTER_POINT& aStart,
                         const ROUTER_POINT& aEnd, int aNetCode, std::int64_t aTrackRadius,
                         std::int64_t aStartMargin, std::int64_t aEndMargin,
                         std::int64_t aSampleStep )
{
    const double length = distance( aStart, aEnd );
    const int count = std::max( 1, static_cast<int>(
                                         std::ceil( length / std::max<std::int64_t>( 1, aSampleStep ) ) ) );

    for( int i = 0; i <= count; ++i )
    {
        const double ratio = static_cast<double>( i ) / count;
        const ROUTER_POINT point{
            static_cast<std::int64_t>( std::llround( aStart.x + ( aEnd.x - aStart.x ) * ratio ) ),
            static_cast<std::int64_t>( std::llround( aStart.y + ( aEnd.y - aStart.y ) * ratio ) ) };

        const std::int64_t margin = i == 0 ? aStartMargin
                                           : i == count ? aEndMargin : aTrackRadius;
        if( !insideBoard( aBoard, point,
                          endpointMargin( aBoard, aNetCode, point, margin ) ) )
            return false;
    }

    return true;
}


bool segmentVsObstacle( const ROUTING_SEGMENT& aSegment, std::int64_t aRouteRadius,
                        const ROUTING_OBSTACLE& aObstacle, const BOARD_SNAPSHOT& aBoard )
{
    if( !layerContains( aObstacle.layers, aSegment.layer ) )
        return false;

    const bool routeIsViaProbe = aSegment.start == aSegment.end && aSegment.width == 0;
    const std::int64_t clearance = obstacleClearance( aBoard, aSegment.netCode, aObstacle,
                                                      aSegment.layer, aSegment.clearance );
    const std::int64_t expandedRadius =
            aObstacle.isHole
                    ? aObstacle.radius
                              + std::max( aRouteRadius + aBoard.holeClearance,
                                          routeIsViaProbe
                                                  ? viaDrillRadius( aBoard, aSegment.netCode, 0 )
                                                            + aBoard.holeToHoleClearance
                                                  : aRouteRadius + aBoard.holeClearance )
                    : aRouteRadius + aObstacle.radius + clearance;
    const double radius = static_cast<double>( expandedRadius );

    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
    {
        ROUTER_BOX box = aObstacle.box;
        const auto inflate = static_cast<std::int64_t>( std::ceil( radius ) );
        box.minX -= inflate;
        box.minY -= inflate;
        box.maxX += inflate;
        box.maxY += inflate;

        const ROUTER_POINT topLeft{ box.minX, box.minY };
        const ROUTER_POINT topRight{ box.maxX, box.minY };
        const ROUTER_POINT bottomRight{ box.maxX, box.maxY };
        const ROUTER_POINT bottomLeft{ box.minX, box.maxY };
        return box.Contains( aSegment.start ) || box.Contains( aSegment.end )
               || segmentsIntersect( aSegment.start, aSegment.end, topLeft, topRight )
               || segmentsIntersect( aSegment.start, aSegment.end, topRight, bottomRight )
               || segmentsIntersect( aSegment.start, aSegment.end, bottomRight, bottomLeft )
               || segmentsIntersect( aSegment.start, aSegment.end, bottomLeft, topLeft );
    }

    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
    {
        return segmentsIntersect( aSegment.start, aSegment.end, aObstacle.start, aObstacle.end )
               || pointToSegmentDistance( aSegment.start, aObstacle.start, aObstacle.end )
                              <= radius
               || pointToSegmentDistance( aSegment.end, aObstacle.start, aObstacle.end )
                              <= radius
               || pointToSegmentDistance( aObstacle.start, aSegment.start, aSegment.end )
                              <= radius
               || pointToSegmentDistance( aObstacle.end, aSegment.start, aSegment.end )
                              <= radius;
    }

    if( pointInPolygonWithHoles( aSegment.start, aObstacle )
        || pointInPolygonWithHoles( aSegment.end, aObstacle ) )
    {
        return true;
    }

    const auto intersectsOuterPolygon = [&]( const std::vector<ROUTER_POINT>& aPolygon )
    {
        for( std::size_t i = 0; i < aPolygon.size(); ++i )
        {
            const ROUTER_POINT& start = aPolygon[i];
            const ROUTER_POINT& end = aPolygon[( i + 1 ) % aPolygon.size()];

            if( segmentsIntersect( aSegment.start, aSegment.end, start, end )
                || pointToSegmentDistance( aSegment.start, start, end ) <= radius
                || pointToSegmentDistance( aSegment.end, start, end ) <= radius
                || pointToSegmentDistance( start, aSegment.start, aSegment.end ) <= radius
                || pointToSegmentDistance( end, aSegment.start, aSegment.end ) <= radius )
            {
                return true;
            }
        }

        return false;
    };

    if( intersectsOuterPolygon( aObstacle.polygon ) )
    {
        return true;
    }

    // Hole contours describe legal free space, not copper.  Sample the
    // segment to catch a route that crosses filled material between holes,
    // while allowing a segment that remains in a hole.  The outer-contour
    // test above catches long segments entering or leaving the obstacle.
    const double length = distance( aSegment.start, aSegment.end );
    const int sampleCount = std::min( 10000, std::max( 1, static_cast<int>(
            std::ceil( length / std::max( 1000.0, radius ) ) ) ) );

    for( int index = 0; index <= sampleCount; ++index )
    {
        const double ratio = static_cast<double>( index ) / sampleCount;
        const ROUTER_POINT sample{
            static_cast<std::int64_t>(
                    std::llround( aSegment.start.x
                                  + ( aSegment.end.x - aSegment.start.x ) * ratio ) ),
            static_cast<std::int64_t>(
                    std::llround( aSegment.start.y
                                  + ( aSegment.end.y - aSegment.start.y ) * ratio ) ) };

        if( pointInPolygonWithHoles( sample, aObstacle )
            || pointNearPolygon( sample, aObstacle.polygon, radius )
            || std::any_of( aObstacle.polygonHoles.begin(), aObstacle.polygonHoles.end(),
                            [&]( const std::vector<ROUTER_POINT>& aHole )
                            {
                                return pointNearPolygon( sample, aHole, radius );
                            } ) )
        {
            return true;
        }
    }

    return false;
}


bool segmentVsSegment( const ROUTING_SEGMENT& aLeft, std::int64_t aLeftRadius,
                       const ROUTING_SEGMENT& aRight, std::int64_t aRightRadius,
                       const BOARD_SNAPSHOT& aBoard )
{
    if( aLeft.layer != aRight.layer || aLeft.netCode == aRight.netCode )
        return false;

    const double clearance = static_cast<double>(
            aLeftRadius + aRightRadius
            + std::max( { pairClearance( aBoard, aLeft.netCode, aRight.netCode, aLeft.layer ),
                           std::max<std::int64_t>( 0, aLeft.clearance ),
                           std::max<std::int64_t>( 0, aRight.clearance ) } ) );
    return segmentsIntersect( aLeft.start, aLeft.end, aRight.start, aRight.end )
           || pointToSegmentDistance( aLeft.start, aRight.start, aRight.end ) <= clearance
           || pointToSegmentDistance( aLeft.end, aRight.start, aRight.end ) <= clearance
           || pointToSegmentDistance( aRight.start, aLeft.start, aLeft.end ) <= clearance
           || pointToSegmentDistance( aRight.end, aLeft.start, aLeft.end ) <= clearance;
}


bool segmentVsVia( const ROUTING_SEGMENT& aSegment, std::int64_t aSegmentRadius,
                   const ROUTING_VIA& aVia, const BOARD_SNAPSHOT& aBoard )
{
    if( aSegment.netCode == aVia.netCode || !spansLayer( aVia, aSegment.layer ) )
        return false;

    return pointToSegmentDistance( aVia.position, aSegment.start, aSegment.end )
           <= static_cast<double>( aSegmentRadius
                                   + viaRadius( aBoard, aVia.netCode, aVia.diameter )
                                   + std::max(
                                             { pairClearance( aBoard, aSegment.netCode,
                                                              aVia.netCode, aSegment.layer ),
                                               std::max<std::int64_t>( 0, aSegment.clearance ),
                                               std::max<std::int64_t>( 0, aVia.clearance ) } ) );
}


bool viaVsVia( const ROUTING_VIA& aLeft, const ROUTING_VIA& aRight,
               const BOARD_SNAPSHOT& aBoard )
{
    const std::vector<int> leftLayers = viaLayerSpan( aLeft );
    const std::vector<int> rightLayers = viaLayerSpan( aRight );
    std::vector<int> sharedLayers;

    for( int layer : leftLayers )
    {
        if( std::find( rightLayers.begin(), rightLayers.end(), layer ) != rightLayers.end()
            && std::find( sharedLayers.begin(), sharedLayers.end(), layer )
                       == sharedLayers.end() )
        {
            sharedLayers.push_back( layer );
        }
    }

    if( sharedLayers.empty() )
        return false;

    const double centerDistance = distance( aLeft.position, aRight.position );
    const double copperRadius = static_cast<double>(
            viaRadius( aBoard, aLeft.netCode, aLeft.diameter )
            + viaRadius( aBoard, aRight.netCode, aRight.diameter ) );
    const double drillRadius = static_cast<double>(
            viaDrillRadius( aBoard, aLeft.netCode, aLeft.drill )
            + viaDrillRadius( aBoard, aRight.netCode, aRight.drill )
            + aBoard.holeToHoleClearance );

    if( centerDistance < drillRadius )
        return true;
    if( aLeft.netCode == aRight.netCode )
        return false;

    // Net-pair rules may differ by copper layer.  Test every common layer
    // instead of using the first layer of one via as a proxy for the entire
    // span.
    return std::any_of( sharedLayers.begin(), sharedLayers.end(),
                        [&]( int aLayer )
                        {
                            return centerDistance
                                           <= copperRadius
                                                      + std::max(
                                                                { pairClearance(
                                                                          aBoard, aLeft.netCode,
                                                                          aRight.netCode, aLayer ),
                                                                  std::max<std::int64_t>(
                                                                          0, aLeft.clearance ),
                                                                  std::max<std::int64_t>(
                                                                          0, aRight.clearance ) } )
                                   || centerDistance < drillRadius;
                        } );
}


bool isRemoved( const std::unordered_set<std::string>& aRemoved, const std::string& aId )
{
    return !aId.empty() && aRemoved.contains( aId );
}

} // namespace


int DESIGN_RULES_CHECKER::CountViolations( const BOARD_SNAPSHOT& aBoard,
                                           const AUTOROUTER_SETTINGS& aSettings,
                                           const ROUTING_RESULT& aResult )
{
    const std::int64_t sampleStep = std::max<std::int64_t>( 1, aSettings.gridStepIU / 2 );
    std::unordered_set<std::string> removedIds( aResult.removedBoardItemIds.begin(),
                                                aResult.removedBoardItemIds.end() );
    int violations = 0;
    int segmentBoundaryViolations = 0;
    int segmentObstacleViolations = 0;
    int viaBoundaryViolations = 0;
    int viaObstacleViolations = 0;
    int segmentPairViolations = 0;
    int segmentViaViolations = 0;
    int viaPairViolations = 0;

    for( const ROUTING_SEGMENT& segment : aResult.segments )
    {
        const std::int64_t radius = trackRadius( aBoard, segment.netCode, segment.width );

        const std::int64_t startMargin = endpointMargin( aBoard, segment.netCode, segment.start,
                                                         radius );
        const std::int64_t endMargin = endpointMargin( aBoard, segment.netCode, segment.end,
                                                       radius );
        if( !segmentInsideBoard( aBoard, segment.start, segment.end, segment.netCode, radius,
                                 startMargin, endMargin, sampleStep ) )
        {
            ++violations;
            ++segmentBoundaryViolations;
        }

        for( const ROUTING_OBSTACLE& obstacle : aBoard.obstacles )
        {
            const bool ownPadHole =
                    obstacle.isHole && obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT
                    && isPadEndpoint( aBoard, segment.netCode, obstacle.start )
                    && ( obstacle.start == segment.start || obstacle.start == segment.end );
            const bool existingSameNetViaHole =
                    obstacle.isHole && obstacle.isExistingRoute
                    && !obstacle.boardItemId.empty();
            if( ( obstacle.netCode == segment.netCode && !obstacle.isKeepout
                  && ( !obstacle.isHole || ownPadHole || existingSameNetViaHole ) )
                || !obstacle.blocksTracks
                || isRemoved( removedIds, obstacle.boardItemId ) )
            {
                continue;
            }

            if( obstacle.isExistingRoute && isRemoved( removedIds, obstacle.boardItemId ) )
                continue;

            if( segmentVsObstacle( segment, radius, obstacle, aBoard ) )
            {
                ++violations;
                ++segmentObstacleViolations;
            }
        }

        for( const ROUTING_OBSTACLE& obstacle : aBoard.removableExistingRoutes )
        {
            // An unsupported source item is mirrored into `obstacles` as a
            // collision-only search shape during a whole-net reroute. Check
            // that physical copper once, not once through each snapshot
            // collection; the original record still owns UUID removal.
            if( obstacle.isMirroredToObstacleModel )
                continue;

            const bool ownPadHole =
                    obstacle.isHole && obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT
                    && isPadEndpoint( aBoard, segment.netCode, obstacle.start )
                    && ( obstacle.start == segment.start || obstacle.start == segment.end );
            const bool existingSameNetViaHole =
                    obstacle.isHole && obstacle.isExistingRoute
                    && !obstacle.boardItemId.empty();
            if( isRemoved( removedIds, obstacle.boardItemId ) || !obstacle.blocksTracks
                || ( obstacle.netCode == segment.netCode
                     && ( !obstacle.isHole || ownPadHole || existingSameNetViaHole ) ) )
                continue;

            if( segmentVsObstacle( segment, radius, obstacle, aBoard ) )
            {
                ++violations;
                ++segmentObstacleViolations;
            }
        }
    }

    for( const ROUTING_VIA& via : aResult.vias )
    {
        const std::int64_t radius = viaRadius( aBoard, via.netCode, via.diameter );

        for( int layer : viaLayerSpan( via ) )
        {
            if( !insideBoard( aBoard, via.position, radius ) )
            {
                ++violations;
                ++viaBoundaryViolations;
                break;
            }

            for( const ROUTING_OBSTACLE& obstacle : aBoard.obstacles )
            {
                // A proposed via creates a new drill. Same-net copper may
                // share existing copper, but a new drill cannot overlap an
                // existing pad/via drill, even at the same position.
                if( ( obstacle.netCode == via.netCode && !obstacle.isKeepout
                      && !obstacle.isHole )
                    || !obstacle.blocksVias
                    || isRemoved( removedIds, obstacle.boardItemId )
                    || !layerContains( obstacle.layers, layer ) )
                {
                    continue;
                }

                ROUTING_SEGMENT probe{ via.netCode, layer, via.position, via.position, 0,
                                       via.clearance };
                if( segmentVsObstacle( probe, radius, obstacle, aBoard ) )
                {
                    ++violations;
                    ++viaObstacleViolations;
                    if( autorouterDebugEnabled() )
                    {
                        std::ostringstream message;
                        message << "drc via obstacle net=" << via.netCode << " pos=("
                                << via.position.x << ',' << via.position.y << ") layer=" << layer
                                << " obstacleNet=" << obstacle.netCode << " kind="
                                << static_cast<int>( obstacle.kind ) << " hole=" << obstacle.isHole
                                << " existing=" << obstacle.isExistingRoute;
                        autorouterDebugLog( message.str() );
                    }
                    break;
                }
            }
        }

        for( const ROUTING_OBSTACLE& obstacle : aBoard.removableExistingRoutes )
        {
            if( obstacle.isMirroredToObstacleModel
                || isRemoved( removedIds, obstacle.boardItemId ) || !obstacle.blocksVias
                || ( obstacle.netCode == via.netCode && !obstacle.isKeepout
                     && !obstacle.isHole ) )
                continue;

            for( int layer : viaLayerSpan( via ) )
            {
                if( !layerContains( obstacle.layers, layer ) )
                    continue;

                ROUTING_SEGMENT probe{ via.netCode, layer, via.position, via.position, 0,
                                       via.clearance };
                if( segmentVsObstacle( probe, radius, obstacle, aBoard ) )
                {
                    ++violations;
                    ++viaObstacleViolations;
                    if( autorouterDebugEnabled() )
                    {
                        std::ostringstream message;
                        message << "drc removable via obstacle net=" << via.netCode << " pos=("
                                << via.position.x << ',' << via.position.y << ") layer=" << layer
                                << " obstacleNet=" << obstacle.netCode << " kind="
                                << static_cast<int>( obstacle.kind ) << " hole=" << obstacle.isHole
                                << " existing=" << obstacle.isExistingRoute;
                        autorouterDebugLog( message.str() );
                    }
                    break;
                }
            }
        }
    }

    for( std::size_t i = 0; i < aResult.segments.size(); ++i )
    {
        const ROUTING_SEGMENT& left = aResult.segments[i];
        const std::int64_t leftRadius = trackRadius( aBoard, left.netCode, left.width );

        for( std::size_t j = i + 1; j < aResult.segments.size(); ++j )
        {
            if( segmentVsSegment( left, leftRadius, aResult.segments[j],
                                  trackRadius( aBoard, aResult.segments[j].netCode,
                                               aResult.segments[j].width ), aBoard ) )
            {
                ++violations;
                ++segmentPairViolations;
            }
        }

        for( const ROUTING_VIA& via : aResult.vias )
        {
            if( segmentVsVia( left, leftRadius, via, aBoard ) )
            {
                ++violations;
                ++segmentViaViolations;
            }
        }
    }

    for( std::size_t i = 0; i < aResult.vias.size(); ++i )
    {
        for( std::size_t j = i + 1; j < aResult.vias.size(); ++j )
        {
            if( viaVsVia( aResult.vias[i], aResult.vias[j], aBoard ) )
            {
                ++violations;
                ++viaPairViolations;
            }
        }
    }

    if( autorouterDebugEnabled() && violations > 0 )
    {
        std::ostringstream message;
        message << "drc violations=" << violations << " segmentBoundary="
                << segmentBoundaryViolations << " segmentObstacle=" << segmentObstacleViolations
                << " viaBoundary=" << viaBoundaryViolations << " viaObstacle="
                << viaObstacleViolations << " segmentPair=" << segmentPairViolations
                << " segmentVia=" << segmentViaViolations << " viaPair=" << viaPairViolations;
        autorouterDebugLog( message.str() );
    }

    return violations;
}

} // namespace KICAD_AUTOROUTER
