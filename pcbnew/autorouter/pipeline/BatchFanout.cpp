/*
 * This program source code file is part of KiCad, a free EDA application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#include "BatchFanout.h"
#include "../rules/ViaRule.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <map>
#include <optional>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "../AutorouterDebug.h"

namespace KICAD_AUTOROUTER
{

namespace
{

std::size_t invalidIndex()
{
    return std::numeric_limits<std::size_t>::max();
}


int layerOrdinal( const AUTOROUTER_SETTINGS& aSettings, int aLayer )
{
    const auto it = std::find_if( aSettings.layers.begin(), aSettings.layers.end(),
                                  [aLayer]( const ROUTER_LAYER_SETTINGS& aSetting )
                                  {
                                      return aSetting.layerId == aLayer;
                                  } );

    if( it == aSettings.layers.end() || it->layerOrdinal < 0 )
        return it == aSettings.layers.end()
                       ? aLayer
                       : static_cast<int>( std::distance( aSettings.layers.begin(), it ) );

    return it->layerOrdinal;
}


std::vector<int> fanoutLayers( const AUTOROUTER_SETTINGS& aSettings, int aSourceLayer )
{
    std::vector<ROUTER_LAYER_SETTINGS> enabled;
    for( const ROUTER_LAYER_SETTINGS& layer : aSettings.layers )
    {
        if( layer.enabled && layer.layerId != aSourceLayer )
            enabled.push_back( layer );
    }

    if( enabled.empty() )
        return {};

    // Prefer a nearby trace landing layer. The manufactured through-via
    // still occupies the entire stack; this is not a partial-via mask.
    std::stable_sort( enabled.begin(), enabled.end(),
                      [&]( const ROUTER_LAYER_SETTINGS& aLeft,
                            const ROUTER_LAYER_SETTINGS& aRight )
                      {
                          const int leftDistance = std::abs(
                                  layerOrdinal( aSettings, aLeft.layerId )
                                  - layerOrdinal( aSettings, aSourceLayer ) );
                          const int rightDistance = std::abs(
                                  layerOrdinal( aSettings, aRight.layerId )
                                  - layerOrdinal( aSettings, aSourceLayer ) );
                          if( leftDistance != rightDistance )
                              return leftDistance < rightDistance;
                          return aLeft.layerId < aRight.layerId;
                      } );

    std::vector<int> result;
    result.reserve( enabled.size() );
    for( const ROUTER_LAYER_SETTINGS& layer : enabled )
        result.push_back( layer.layerId );
    return result;
}


ROUTER_BOX fanoutObstacleBounds( const ROUTING_OBSTACLE& aObstacle )
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
        }
        else
        {
            result.minX = std::min( result.minX, aPoint.x );
            result.minY = std::min( result.minY, aPoint.y );
            result.maxX = std::max( result.maxX, aPoint.x );
            result.maxY = std::max( result.maxY, aPoint.y );
        }
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


long double fanoutPointToSegmentDistance( const ROUTER_POINT& aPoint,
                                          const ROUTER_POINT& aStart,
                                          const ROUTER_POINT& aEnd )
{
    const long double dx = static_cast<long double>( aEnd.x ) - aStart.x;
    const long double dy = static_cast<long double>( aEnd.y ) - aStart.y;
    const long double lengthSquared = dx * dx + dy * dy;

    if( lengthSquared == 0.0L )
    {
        const long double px = static_cast<long double>( aPoint.x ) - aStart.x;
        const long double py = static_cast<long double>( aPoint.y ) - aStart.y;
        return std::sqrt( px * px + py * py );
    }

    const long double px = static_cast<long double>( aPoint.x ) - aStart.x;
    const long double py = static_cast<long double>( aPoint.y ) - aStart.y;
    const long double ratio = std::clamp( ( px * dx + py * dy ) / lengthSquared, 0.0L,
                                          1.0L );
    const long double closestX = aStart.x + ratio * dx;
    const long double closestY = aStart.y + ratio * dy;
    const long double distanceX = static_cast<long double>( aPoint.x ) - closestX;
    const long double distanceY = static_cast<long double>( aPoint.y ) - closestY;
    return std::sqrt( distanceX * distanceX + distanceY * distanceY );
}


long double fanoutCross( const ROUTER_POINT& a,
                         const ROUTER_POINT& b,
                         const ROUTER_POINT& c )
{
    // Cast before subtracting.  Fanout planning is allowed to inspect a
    // complete KiCad board, whose IU coordinates can lie near opposite int64
    // limits; subtracting first would overflow before this predicate can
    // protect the geometry.
    return ( static_cast<long double>( b.x ) - a.x )
                   * ( static_cast<long double>( c.y ) - a.y )
           - ( static_cast<long double>( b.y ) - a.y )
                     * ( static_cast<long double>( c.x ) - a.x );
}


bool fanoutOnSegment( const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
                      const ROUTER_POINT& aPoint )
{
    return aPoint.x >= std::min( aStart.x, aEnd.x )
           && aPoint.x <= std::max( aStart.x, aEnd.x )
           && aPoint.y >= std::min( aStart.y, aEnd.y )
           && aPoint.y <= std::max( aStart.y, aEnd.y )
           && std::abs( fanoutCross( aStart, aEnd, aPoint ) ) < 0.5L;
}


bool fanoutSegmentsIntersect( const ROUTER_POINT& aFirstStart,
                              const ROUTER_POINT& aFirstEnd,
                              const ROUTER_POINT& aSecondStart,
                              const ROUTER_POINT& aSecondEnd )
{
    const long double first = fanoutCross( aFirstStart, aFirstEnd, aSecondStart );
    const long double second = fanoutCross( aFirstStart, aFirstEnd, aSecondEnd );
    const long double third = fanoutCross( aSecondStart, aSecondEnd, aFirstStart );
    const long double fourth = fanoutCross( aSecondStart, aSecondEnd, aFirstEnd );

    const auto oppositeSigns = []( long double aLeft, long double aRight )
    {
        return ( aLeft < 0.0L && aRight > 0.0L ) || ( aLeft > 0.0L && aRight < 0.0L );
    };

    if( oppositeSigns( first, second ) && oppositeSigns( third, fourth ) )
        return true;

    return ( first == 0.0L && fanoutOnSegment( aFirstStart, aFirstEnd, aSecondStart ) )
           || ( second == 0.0L && fanoutOnSegment( aFirstStart, aFirstEnd, aSecondEnd ) )
           || ( third == 0.0L && fanoutOnSegment( aSecondStart, aSecondEnd, aFirstStart ) )
           || ( fourth == 0.0L && fanoutOnSegment( aSecondStart, aSecondEnd, aFirstEnd ) );
}


bool fanoutPointInPolygon( const ROUTER_POINT& aPoint,
                           const std::vector<ROUTER_POINT>& aPolygon )
{
    if( aPolygon.size() < 3 )
        return false;

    bool inside = false;
    for( std::size_t index = 0, previous = aPolygon.size() - 1; index < aPolygon.size();
         previous = index++ )
    {
        const ROUTER_POINT& current = aPolygon[index];
        const ROUTER_POINT& prior = aPolygon[previous];
        if( fanoutOnSegment( prior, current, aPoint ) )
            return true;

        const bool crosses = ( current.y > aPoint.y ) != ( prior.y > aPoint.y );
        if( crosses )
        {
            const long double intersectionX =
                    static_cast<long double>( prior.x )
                    + ( static_cast<long double>( current.x ) - prior.x )
                              * ( static_cast<long double>( aPoint.y ) - prior.y )
                              / ( static_cast<long double>( current.y ) - prior.y );
            if( aPoint.x < intersectionX )
                inside = !inside;
        }
    }

    return inside;
}


bool fanoutLayerApplies( const ROUTING_OBSTACLE& aObstacle, int aLayer )
{
    return aObstacle.layers.empty()
           || std::find( aObstacle.layers.begin(), aObstacle.layers.end(), aLayer )
                      != aObstacle.layers.end();
}


struct FANOUT_CLEARANCE_KEY
{
    int firstNetCode = 0;
    int secondNetCode = 0;
    int layer = -1;

    bool operator==( const FANOUT_CLEARANCE_KEY& aOther ) const
    {
        return firstNetCode == aOther.firstNetCode && secondNetCode == aOther.secondNetCode
               && layer == aOther.layer;
    }
};


struct FANOUT_CLEARANCE_KEY_HASH
{
    std::size_t operator()( const FANOUT_CLEARANCE_KEY& aKey ) const
    {
        std::size_t result = std::hash<int>{}( aKey.firstNetCode );
        result ^= std::hash<int>{}( aKey.secondNetCode ) + 0x9e3779b9 + ( result << 6 )
                  + ( result >> 2 );
        result ^= std::hash<int>{}( aKey.layer ) + 0x9e3779b9 + ( result << 6 )
                  + ( result >> 2 );
        return result;
    }
};


FANOUT_CLEARANCE_KEY fanoutClearanceKey( int aFirstNetCode, int aSecondNetCode, int aLayer )
{
    if( aFirstNetCode > aSecondNetCode )
        std::swap( aFirstNetCode, aSecondNetCode );

    return { aFirstNetCode, aSecondNetCode, aLayer };
}


struct FANOUT_CLEARANCE_CONTEXT
{
    std::unordered_map<int, std::int64_t> netClearances;
    std::unordered_map<FANOUT_CLEARANCE_KEY, std::int64_t, FANOUT_CLEARANCE_KEY_HASH>
            pairClearances;
    std::int64_t maximumClearance = 0;
    std::int64_t maximumObstacleInflation = 0;
};


std::vector<ROUTING_VIA_DIMENSION> fanoutViasForNet(
        const BOARD_SNAPSHOT& aBoard, int aNetCode,
        const AUTOROUTER_SETTINGS& aSettings )
{
    std::vector<ROUTING_VIA_DIMENSION> result;
    const auto net = std::find_if( aBoard.nets.begin(), aBoard.nets.end(),
                                   [aNetCode]( const ROUTING_NET& aCandidate )
                                   {
                                       return aCandidate.netCode == aNetCode;
                                   } );

    if( net == aBoard.nets.end() )
        return result;

    const auto append = [&]( ROUTING_VIA_DIMENSION aProfile )
    {
        if( aProfile.diameter <= 0 || aProfile.drill <= 0
            || std::find( result.begin(), result.end(), aProfile ) != result.end() )
        {
            return;
        }

        result.push_back( std::move( aProfile ) );
    };

    // A non-empty netclass via rule remains the first choice.  When enabled,
    // RoutingBoard.fanout() appends every board ViaRule alternative after the
    // net rule, not merely the first one.  Keeping that ordered candidate set
    // lets a small padstack escape a channel that rejects the default large
    // board via instead of declaring the pin impossible prematurely.
    if( net->viaDiameter > 0 )
    {
        append( { net->viaDiameter, net->viaDrill > 0 ? net->viaDrill : 300000 } );
    }

    if( !aSettings.fanoutFallbackToBoardVias )
        return result;

    // Board preset order is part of the rule: this is deliberately not a
    // smallest-via heuristic.  The search evaluates the first legal profile
    // in that declared order, matching the source ViaRule traversal.
    for( const ROUTING_VIA_DIMENSION& profile : aBoard.boardViaDimensions )
        append( profile );

    return result;
}


FANOUT_CLEARANCE_CONTEXT makeFanoutClearanceContext(
        const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings )
{
    FANOUT_CLEARANCE_CONTEXT result;
    result.netClearances.reserve( aBoard.nets.size() );
    result.pairClearances.reserve( aBoard.clearanceRules.size() );

    for( const ROUTING_NET& net : aBoard.nets )
    {
        auto [clearanceIt, inserted] = result.netClearances.emplace( net.netCode, net.clearance );
        if( !inserted )
            clearanceIt->second = std::max( clearanceIt->second, net.clearance );

        result.maximumClearance = std::max( result.maximumClearance, net.clearance );

    }

    for( const ROUTING_CLEARANCE_RULE& rule : aBoard.clearanceRules )
    {
        result.maximumClearance = std::max( result.maximumClearance, rule.clearance );
        const FANOUT_CLEARANCE_KEY key = fanoutClearanceKey(
                rule.firstNetCode, rule.secondNetCode, rule.layer < 0 ? -1 : rule.layer );
        auto [clearanceIt, inserted] = result.pairClearances.emplace( key, rule.clearance );
        if( !inserted )
            clearanceIt->second = std::max( clearanceIt->second, rule.clearance );
    }

    for( const ROUTING_OBSTACLE& obstacle : aBoard.obstacles )
    {
        result.maximumObstacleInflation = std::max(
                result.maximumObstacleInflation, obstacle.radius + obstacle.clearance );
    }

    return result;
}


std::int64_t fanoutPairClearance( const FANOUT_CLEARANCE_CONTEXT& aContext,
                                  const ROUTING_PAD& aPad, const ROUTING_OBSTACLE& aObstacle,
                                  int aLayer )
{
    if( aObstacle.netCode == 0 || aObstacle.netCode == aPad.netCode )
        return std::max( aPad.clearance, aObstacle.clearance );

    std::int64_t result = std::max( aPad.clearance, aObstacle.clearance );

    const FANOUT_CLEARANCE_KEY pairKey =
            fanoutClearanceKey( aPad.netCode, aObstacle.netCode, aLayer );
    const auto layerIt = aContext.pairClearances.find( pairKey );
    if( layerIt != aContext.pairClearances.end() )
        result = std::max( result, layerIt->second );

    const auto globalIt = aContext.pairClearances.find(
            fanoutClearanceKey( aPad.netCode, aObstacle.netCode, -1 ) );
    if( globalIt != aContext.pairClearances.end() )
        result = std::max( result, globalIt->second );

    const auto padNetIt = aContext.netClearances.find( aPad.netCode );
    if( padNetIt != aContext.netClearances.end() )
        result = std::max( result, padNetIt->second );

    const auto obstacleNetIt = aContext.netClearances.find( aObstacle.netCode );
    if( obstacleNetIt != aContext.netClearances.end() )
        result = std::max( result, obstacleNetIt->second );

    return result;
}


bool fanoutPointCollides( const ROUTING_OBSTACLE& aObstacle, const ROUTER_POINT& aPoint,
                          std::int64_t aRadius )
{
    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
    {
        ROUTER_BOX box = aObstacle.box;
        box.minX -= aRadius;
        box.minY -= aRadius;
        box.maxX += aRadius;
        box.maxY += aRadius;
        return box.Contains( aPoint );
    }

    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
        return fanoutPointToSegmentDistance( aPoint, aObstacle.start, aObstacle.end ) <= aRadius;

    // A polygon with holes represents solid outer-area minus every hole. A
    // landing inside a hole is legal only when it clears *all* solid
    // boundaries.  Returning after the containing hole misses a nearby outer
    // contour in a thin annular region, disagreeing with the main maze
    // engine's exact obstacle semantics.
    const auto nearContour = [&]( const std::vector<ROUTER_POINT>& aContour )
    {
        for( std::size_t index = 0; index < aContour.size(); ++index )
            if( fanoutPointToSegmentDistance(
                        aPoint, aContour[index], aContour[( index + 1 ) % aContour.size()] )
                <= static_cast<long double>( aRadius ) )
            {
                return true;
            }
        return false;
    };

    if( nearContour( aObstacle.polygon ) )
        return true;
    for( const auto& hole : aObstacle.polygonHoles )
        if( nearContour( hole ) )
            return true;

    if( !fanoutPointInPolygon( aPoint, aObstacle.polygon ) )
        return false;

    return std::none_of( aObstacle.polygonHoles.begin(), aObstacle.polygonHoles.end(),
                         [&]( const std::vector<ROUTER_POINT>& aHole )
                         { return fanoutPointInPolygon( aPoint, aHole ); } );
}


bool fanoutSegmentsWithinClearance( const ROUTER_POINT& aFirstStart,
                                    const ROUTER_POINT& aFirstEnd,
                                    const ROUTER_POINT& aSecondStart,
                                    const ROUTER_POINT& aSecondEnd,
                                    std::int64_t aRadius )
{
    if( fanoutSegmentsIntersect( aFirstStart, aFirstEnd, aSecondStart, aSecondEnd ) )
        return true;

    const long double clearance = std::max<std::int64_t>( 0, aRadius );
    // The minimum distance between finite segments occurs at an intersection
    // or at an endpoint.  This replaces the former sampled/outer-boundary
    // check so a direct escape cannot cross solid material between otherwise
    // legal endpoints.
    return fanoutPointToSegmentDistance( aFirstStart, aSecondStart, aSecondEnd ) <= clearance
           || fanoutPointToSegmentDistance( aFirstEnd, aSecondStart, aSecondEnd ) <= clearance
           || fanoutPointToSegmentDistance( aSecondStart, aFirstStart, aFirstEnd ) <= clearance
           || fanoutPointToSegmentDistance( aSecondEnd, aFirstStart, aFirstEnd ) <= clearance;
}


bool fanoutSegmentCollides( const ROUTING_OBSTACLE& aObstacle, const ROUTER_POINT& aStart,
                            const ROUTER_POINT& aEnd, std::int64_t aRadius )
{
    const ROUTER_BOX routeBounds{ std::min( aStart.x, aEnd.x ) - aRadius,
                                  std::min( aStart.y, aEnd.y ) - aRadius,
                                  std::max( aStart.x, aEnd.x ) + aRadius,
                                  std::max( aStart.y, aEnd.y ) + aRadius };
    const ROUTER_BOX obstacleBounds = fanoutObstacleBounds( aObstacle );
    if( routeBounds.maxX < obstacleBounds.minX || obstacleBounds.maxX < routeBounds.minX
        || routeBounds.maxY < obstacleBounds.minY || obstacleBounds.maxY < routeBounds.minY )
    {
        return false;
    }

    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
    {
        ROUTER_BOX box = aObstacle.box;
        box.minX -= aRadius;
        box.minY -= aRadius;
        box.maxX += aRadius;
        box.maxY += aRadius;
        const ROUTER_POINT topLeft{ box.minX, box.minY };
        const ROUTER_POINT topRight{ box.maxX, box.minY };
        const ROUTER_POINT bottomRight{ box.maxX, box.maxY };
        const ROUTER_POINT bottomLeft{ box.minX, box.maxY };
        return box.Contains( aStart ) || box.Contains( aEnd )
               || fanoutSegmentsIntersect( aStart, aEnd, topLeft, topRight )
               || fanoutSegmentsIntersect( aStart, aEnd, topRight, bottomRight )
               || fanoutSegmentsIntersect( aStart, aEnd, bottomRight, bottomLeft )
               || fanoutSegmentsIntersect( aStart, aEnd, bottomLeft, topLeft );
    }

    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
    {
        return fanoutSegmentsWithinClearance( aStart, aEnd, aObstacle.start, aObstacle.end,
                                              aRadius );
    }

    if( fanoutPointCollides( aObstacle, aStart, aRadius )
        || fanoutPointCollides( aObstacle, aEnd, aRadius ) )
    {
        return true;
    }

    // A direct fanout stub can start and end in separate arms of a concave
    // hole.  Looking only at the outer contour misses the solid chord between
    // those arms.  Every hole contour is a solid boundary too.
    const auto crossesContour = [&]( const std::vector<ROUTER_POINT>& aContour )
    {
        for( std::size_t index = 0; index < aContour.size(); ++index )
            if( fanoutSegmentsWithinClearance(
                        aStart, aEnd, aContour[index],
                        aContour[( index + 1 ) % aContour.size()], aRadius ) )
            {
                return true;
            }
        return false;
    };

    if( crossesContour( aObstacle.polygon ) )
        return true;
    for( const auto& hole : aObstacle.polygonHoles )
        if( crossesContour( hole ) )
            return true;

    return false;
}


bool fanoutPointInsideBoard( const BOARD_SNAPSHOT& aBoard, const ROUTER_POINT& aPoint,
                             std::int64_t aRadius )
{
    const long double radius = std::max<std::int64_t>( 0, aRadius );
    if( static_cast<long double>( aPoint.x ) < aBoard.bounds.minX + radius
        || static_cast<long double>( aPoint.x ) > aBoard.bounds.maxX - radius
        || static_cast<long double>( aPoint.y ) < aBoard.bounds.minY + radius
        || static_cast<long double>( aPoint.y ) > aBoard.bounds.maxY - radius )
    {
        return false;
    }

    if( !aBoard.boardOutline.empty() && !fanoutPointInPolygon( aPoint, aBoard.boardOutline ) )
        return false;

    if( std::any_of( aBoard.boardHoles.begin(), aBoard.boardHoles.end(),
                     [&]( const std::vector<ROUTER_POINT>& aHole )
                     {
                         return fanoutPointInPolygon( aPoint, aHole );
                     } ) )
    {
        return false;
    }

    // A rectangular board has already been handled by the bounds check. For
    // a shaped outline, however, a legal centre can still be closer than the
    // requested copper-to-edge margin to a concave edge or cutout. The main
    // maze treats these contours as real clearance boundaries; fanout must
    // not manufacture a landing that it later rejects.
    if( radius <= 0 )
        return true;

    const auto nearContour = [&]( const std::vector<ROUTER_POINT>& aContour )
    {
        for( std::size_t index = 0; index < aContour.size(); ++index )
            if( fanoutPointToSegmentDistance(
                        aPoint, aContour[index], aContour[( index + 1 ) % aContour.size()] )
                <= radius )
            {
                return true;
            }
        return false;
    };

    if( !aBoard.boardOutline.empty() && nearContour( aBoard.boardOutline ) )
        return false;

    return std::none_of( aBoard.boardHoles.begin(), aBoard.boardHoles.end(), nearContour );
}


bool fanoutTrackSegmentAllowed( const BOARD_SNAPSHOT& aBoard, const ROUTING_PAD& aPad,
                                const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
                                int aLayer,
                                const std::vector<std::size_t>& aNearbyObstacles,
                                const FANOUT_CLEARANCE_CONTEXT& aContext,
                                const ROUTER_CANCEL_CALLBACK& aCancel )
{
    if( aCancel && aCancel() )
        return false;

    const std::int64_t trackRadius = std::max<std::int64_t>( 1, aPad.trackWidth / 2 );
    const std::int64_t edgeMargin = aBoard.edgeClearance + trackRadius;
    if( !fanoutPointInsideBoard( aBoard, aStart, edgeMargin )
        || !fanoutPointInsideBoard( aBoard, aEnd, edgeMargin ) )
    {
        return false;
    }

    // For non-rectangular board outlines, two individually legal points can
    // still be separated by a concave exterior or an internal cutout. Test
    // the finite segment against every outline contour, not raster samples.
    const auto crossesBoardContour = [&]( const std::vector<ROUTER_POINT>& aContour )
    {
        for( std::size_t index = 0; index < aContour.size(); ++index )
            if( fanoutSegmentsWithinClearance(
                        aStart, aEnd, aContour[index],
                        aContour[( index + 1 ) % aContour.size()], edgeMargin ) )
            {
                return true;
            }
        return false;
    };

    if( !aBoard.boardOutline.empty() && crossesBoardContour( aBoard.boardOutline ) )
        return false;
    if( std::any_of( aBoard.boardHoles.begin(), aBoard.boardHoles.end(),
                     crossesBoardContour ) )
    {
        return false;
    }

    for( std::size_t obstacleIndex : aNearbyObstacles )
    {
        if( aCancel && aCancel() )
            return false;

        if( obstacleIndex >= aBoard.obstacles.size() )
            continue;

        const ROUTING_OBSTACLE& obstacle = aBoard.obstacles[obstacleIndex];
        if( !fanoutLayerApplies( obstacle, aLayer ) || !obstacle.blocksTracks )
            continue;

        const bool sameNetExistingViaHole = obstacle.isHole && obstacle.isExistingRoute
                                             && !obstacle.boardItemId.empty();
        const bool sameNetPadHole = obstacle.isHole
                                    && ( obstacle.start == aStart || obstacle.start == aEnd
                                         || obstacle.start == aPad.position );
        if( obstacle.netCode != 0 && obstacle.netCode == aPad.netCode && !obstacle.isKeepout
            && ( !obstacle.isHole || sameNetPadHole || sameNetExistingViaHole ) )
        {
            continue;
        }

        const std::int64_t radius = obstacle.isHole
                ? obstacle.radius + trackRadius + aBoard.holeClearance
                : obstacle.radius + trackRadius
                                            + fanoutPairClearance( aContext, aPad, obstacle,
                                                                   aLayer );
        if( fanoutSegmentCollides( obstacle, aStart, aEnd, radius ) )
            return false;
    }

    return true;
}


bool fanoutSegmentAllowed( const BOARD_SNAPSHOT& aBoard, const ROUTING_PAD& aPad,
                           const ROUTER_POINT& aPoint, int aSourceLayer, int aTargetLayer,
                           const ROUTING_VIA_DIMENSION& aVia,
                           const AUTOROUTER_SETTINGS& aSettings,
                           const std::vector<std::size_t>& aNearbyObstacles,
                           const FANOUT_CLEARANCE_CONTEXT& aContext,
                           const ROUTER_CANCEL_CALLBACK& aCancel, bool aCheckStub )
{
    if( aCancel && aCancel() )
        return false;

    const std::int64_t trackRadius = std::max<std::int64_t>( 1, aPad.trackWidth / 2 );
    const std::int64_t viaRadius = std::max<std::int64_t>( 1, aVia.diameter / 2 );
    const std::int64_t viaDrillRadius = std::max<std::int64_t>( 1, aVia.drill / 2 );

    const auto obstacleAllowed = [&]( const ROUTING_OBSTACLE& aObstacle, int aLayer,
                                      bool aForVia, std::int64_t aGeometryRadius )
    {
        if( !fanoutLayerApplies( aObstacle, aLayer ) )
            return true;

        const bool sameNetExistingViaHole =
                aObstacle.isHole && aObstacle.isExistingRoute
                && !aObstacle.boardItemId.empty();
        const bool sameNetPadHole = aObstacle.isHole && aObstacle.start == aPad.position;
        if( aObstacle.netCode != 0 && aObstacle.netCode == aPad.netCode
            && !aObstacle.isKeepout
            && ( !aObstacle.isHole
                 || ( !aForVia && ( sameNetPadHole || sameNetExistingViaHole ) ) ) )
        {
            return true;
        }

        if( aForVia ? !aObstacle.blocksVias : !aObstacle.blocksTracks )
            return true;

        // A via landing has two independent interactions with a drill: its
        // copper annulus observes copper-to-hole clearance while its own
        // drill observes hole-to-hole clearance.  This must match the maze
        // engine and final DRC rather than treating a hole as ordinary
        // foreign copper and accepting an electrically legal but
        // mechanically impossible fanout landing.
        const std::int64_t radius = aObstacle.isHole
                ? aObstacle.radius
                          + ( aForVia
                                      ? std::max( aGeometryRadius + aBoard.holeClearance,
                                                  viaDrillRadius
                                                          + aBoard.holeToHoleClearance )
                                      : aGeometryRadius + aBoard.holeClearance )
                : aObstacle.radius + aGeometryRadius
                                             + fanoutPairClearance( aContext, aPad, aObstacle,
                                                                    aLayer );
        return !fanoutPointCollides( aObstacle, aPoint, radius );
    };

    if( !fanoutPointInsideBoard( aBoard, aPoint,
                                 aBoard.edgeClearance + std::max( trackRadius, viaRadius ) ) )
    {
        return false;
    }

    if( aCheckStub && !fanoutTrackSegmentAllowed( aBoard, aPad, aPad.position, aPoint,
                                                   aSourceLayer, aNearbyObstacles, aContext,
                                                   aCancel ) )
        return false;

    if( !VIA_RULE::AllowsTransition( aSettings, aSourceLayer, aTargetLayer ) )
        return false;

    for( const ROUTER_LAYER_SETTINGS& layer : aSettings.layers )
    {
        for( std::size_t obstacleIndex : aNearbyObstacles )
        {
            if( aCancel && aCancel() )
                return false;

            if( obstacleIndex >= aBoard.obstacles.size() )
                continue;

            const ROUTING_OBSTACLE& obstacle = aBoard.obstacles[obstacleIndex];
            if( !obstacleAllowed( obstacle, layer.layerId, true, viaRadius ) )
            {
                return false;
            }
        }
    }

    return true;
}


std::optional<ROUTER_POINT> fanoutLandingPoint( const BOARD_SNAPSHOT& aBoard,
                                                const ROUTING_PAD& aPad,
                                                int aTargetLayer,
                                                const ROUTING_VIA_DIMENSION& aVia,
                                                const AUTOROUTER_SETTINGS& aSettings,
                                                const FANOUT_CLEARANCE_CONTEXT& aContext,
                                                const ROUTER_CANCEL_CALLBACK& aCancel,
                                                bool& aDirectStub,
                                                std::vector<ROUTER_POINT>& aEscapePath )
{
    aEscapePath.clear();
    aDirectStub = false;
    if( aCancel && aCancel() )
        return std::nullopt;

    // A fanout escape is a short straight stub, but a package edge is not
    // restricted to the eight compass directions.  Include the shallow
    // 1:2 and 2:1 directions as well; this is still deterministic while
    // avoiding a systematic failure for pins whose only free channel is
    // between the coarse cardinal/diagonal probes.
    std::vector<std::array<int, 2>> directions = {
        std::array<int, 2>{ 1, 0 },   std::array<int, 2>{ -1, 0 },
        std::array<int, 2>{ 0, 1 },   std::array<int, 2>{ 0, -1 },
        std::array<int, 2>{ 1, 1 },   std::array<int, 2>{ -1, 1 },
        std::array<int, 2>{ -1, -1 }, std::array<int, 2>{ 1, -1 },
        std::array<int, 2>{ 2, 1 },   std::array<int, 2>{ 2, -1 },
        std::array<int, 2>{ -2, 1 },  std::array<int, 2>{ -2, -1 },
        std::array<int, 2>{ 1, 2 },   std::array<int, 2>{ -1, 2 },
        std::array<int, 2>{ 1, -2 },  std::array<int, 2>{ -1, -2 } };

    // RoutingBoard.fanout() runs a real maze rather than constraining an
    // escape to the sixteen probes above. Preserve those probes first for
    // deterministic compatibility, then add a bounded set of primitive
    // 3:2/4:3/... directions. This captures narrow non-cardinal channels
    // without making each SMD pin an unbounded polar-grid search.
    constexpr int angularRefinement = 4;
    for( int x = -angularRefinement; x <= angularRefinement; ++x )
    {
        for( int y = -angularRefinement; y <= angularRefinement; ++y )
        {
            if( x == 0 && y == 0 )
                continue;

            const int absX = std::abs( x );
            const int absY = std::abs( y );
            if( std::gcd( absX, absY ) != 1 || std::max( absX, absY ) <= 2 )
                continue;

            const std::array<int, 2> direction{ x, y };
            if( std::find( directions.begin(), directions.end(), direction ) == directions.end() )
                directions.push_back( direction );
        }
    }

    std::vector<std::size_t> directionOrder( directions.size() );
    std::iota( directionOrder.begin(), directionOrder.end(), 0 );
    std::stable_sort( directionOrder.begin(), directionOrder.end(),
                      [&]( std::size_t aLeft, std::size_t aRight )
                      {
                          const auto room = [&]( std::size_t aIndex )
                          {
                              const auto& direction = directions[aIndex];
                              const std::int64_t xRoom = direction[0] < 0
                                                                 ? aPad.position.x
                                                                         - aBoard.bounds.minX
                                                                 : direction[0] > 0
                                                                         ? aBoard.bounds.maxX
                                                                                   - aPad.position.x
                                                                         : std::numeric_limits<
                                                                                           std::int64_t>::max();
                              const std::int64_t yRoom = direction[1] < 0
                                                                 ? aPad.position.y
                                                                         - aBoard.bounds.minY
                                                                 : direction[1] > 0
                                                                         ? aBoard.bounds.maxY
                                                                                   - aPad.position.y
                                                                         : std::numeric_limits<
                                                                                           std::int64_t>::max();
                              return std::min( xRoom, yRoom );
                          };
                          if( room( aLeft ) != room( aRight ) )
                              return room( aLeft ) > room( aRight );
                          return aLeft < aRight;
                      } );

    const std::int64_t geometricMinimum = std::max<std::int64_t>(
            1, aPad.radius + aPad.clearance + aPad.trackWidth / 2 );
    // Freerouting rejects a fanout drill that is too close to its source pin
    // and drops start-layer maze doors that stray beyond the escape envelope.
    // The synthetic landing planner is the native counterpart, so it must
    // apply both limits before it creates a graph edge.  The pad geometry is
    // an additional lower bound: asking for a 0.1 mm fanout must not put a
    // via inside the source copper merely because the user setting is small.
    const std::int64_t minimumEscape = std::max(
            geometricMinimum, std::max<std::int64_t>( 0, aSettings.fanoutMinEscapeLengthIU ) );
    const std::int64_t gridStep = std::max<std::int64_t>( 1, aSettings.gridStepIU );

    const std::int64_t configuredMaximum =
            std::max<std::int64_t>( 0, aSettings.fanoutMaxEscapeLengthIU );
    // MazeSearchEngine applies the upper and lower fanout bounds as two
    // independent predicates.  Do not "repair" an inconsistent user range
    // by widening its maximum to the minimum: Java rejects every drill in
    // that situation, whereas accepting it here would manufacture an escape
    // outside the requested maximum distance.
    if( configuredMaximum > 0 && configuredMaximum < minimumEscape )
        return std::nullopt;

    const std::int64_t maximumEscape = configuredMaximum > 0
            ? configuredMaximum
            : minimumEscape + 8 * gridStep;

    const std::int64_t viaRadius = std::max<std::int64_t>( 1, aVia.diameter / 2 );
    const std::int64_t maximumClearance = std::max( aPad.clearance,
                                                     aContext.maximumClearance );

    const std::int64_t maximumDistance = maximumEscape;
    const std::int64_t nearbyPadding = viaRadius + maximumClearance
                                       + aContext.maximumObstacleInflation + 2;
    const ROUTER_BOX nearbyBox{ aPad.position.x - maximumDistance - nearbyPadding,
                                aPad.position.y - maximumDistance - nearbyPadding,
                                aPad.position.x + maximumDistance + nearbyPadding,
                                aPad.position.y + maximumDistance + nearbyPadding };
    std::vector<std::size_t> nearbyObstacles;
    nearbyObstacles.reserve( 256 );
    for( std::size_t index = 0; index < aBoard.obstacles.size(); ++index )
    {
        if( aCancel && aCancel() )
            return std::nullopt;

        const ROUTER_BOX bounds = fanoutObstacleBounds( aBoard.obstacles[index] );
        if( bounds.maxX >= nearbyBox.minX && bounds.minX <= nearbyBox.maxX
            && bounds.maxY >= nearbyBox.minY && bounds.minY <= nearbyBox.maxY )
        {
            nearbyObstacles.push_back( index );
        }
    }

    // Try short, legal escape stubs first.  If the package is surrounded, the
    // increasing radii give the fanout a deterministic way to search for the
    // first open room without burdening the general maze frontier with a
    // synthetic pad-centre via attempt.
    const std::int64_t availableDistance = std::max<std::int64_t>(
            0, maximumEscape - minimumEscape );
    const int maximumDistanceExpansions = static_cast<int>( std::min<std::int64_t>(
            63, availableDistance / gridStep ) );
    const int maximumExpansions = std::min(
            std::clamp( aSettings.fanoutLandingSearchSteps, 1, 64 ) - 1,
            maximumDistanceExpansions );
    const auto findCandidate = [&]( bool aCheckStub ) -> std::optional<ROUTER_POINT>
    {
        for( int expansion = 0; expansion <= maximumExpansions; ++expansion )
        {
            if( aCancel && aCancel() )
                return std::nullopt;

            const std::int64_t distance = minimumEscape + expansion * gridStep;
            for( std::size_t directionIndex : directionOrder )
            {
                if( aCancel && aCancel() )
                    return std::nullopt;

                const auto& direction = directions[directionIndex];
                // `distance` is a physical fanout-envelope radius, not a
                // per-axis multiplier.  The earlier synthetic planner used
                // (dx * distance, dy * distance) directly, which made its
                // diagonal and 1:2 probes sqrt(2), sqrt(5), ... times too
                // long.  Most of those candidates were then rejected by the
                // maximum escape predicate, leaving a real fanout maze with
                // only four cardinal exits.  Normalize the deterministic
                // direction before converting back to integral KiCad IU so
                // every probe represents the same source-style escape radius.
                const long double directionLength = std::hypotl(
                        static_cast<long double>( direction[0] ),
                        static_cast<long double>( direction[1] ) );
                if( directionLength <= 0.0L )
                    continue;

                const long double candidateX = static_cast<long double>( aPad.position.x )
                        + static_cast<long double>( direction[0] ) * distance / directionLength;
                const long double candidateY = static_cast<long double>( aPad.position.y )
                        + static_cast<long double>( direction[1] ) * distance / directionLength;
                constexpr long double coordinateMinimum = static_cast<long double>(
                        std::numeric_limits<std::int64_t>::min() );
                constexpr long double coordinateMaximum = static_cast<long double>(
                        std::numeric_limits<std::int64_t>::max() );
                if( !std::isfinite( candidateX ) || !std::isfinite( candidateY )
                    || candidateX <= coordinateMinimum || candidateX >= coordinateMaximum
                    || candidateY <= coordinateMinimum || candidateY >= coordinateMaximum )
                {
                    continue;
                }

                ROUTER_POINT candidate{ static_cast<std::int64_t>( std::llround( candidateX ) ),
                                        static_cast<std::int64_t>( std::llround( candidateY ) ) };
                const long double candidateLength = std::sqrt(
                        ( static_cast<long double>( candidate.x ) - aPad.position.x )
                                * ( static_cast<long double>( candidate.x ) - aPad.position.x )
                        + ( static_cast<long double>( candidate.y ) - aPad.position.y )
                                * ( static_cast<long double>( candidate.y ) - aPad.position.y ) );
                if( candidateLength < static_cast<long double>( minimumEscape )
                    || candidateLength > static_cast<long double>( maximumEscape ) )
                    continue;
                if( fanoutSegmentAllowed( aBoard, aPad, candidate, aPad.layers.front(),
                                          aTargetLayer, aVia, aSettings, nearbyObstacles,
                                          aContext, aCancel, aCheckStub ) )
                {
                    return candidate;
                }
            }
        }
        return std::nullopt;
    };

    if( const std::optional<ROUTER_POINT> direct = findCandidate( true ) )
    {
        aDirectStub = true;
        aEscapePath = { aPad.position, *direct };
        return direct;
    }

    // RoutingBoard.fanout() is a real bounded maze search, not an arbitrary
    // reservation of an otherwise free via point.  The former native
    // fallback used findCandidate(false): it created a landing behind a
    // blocker and hoped that a later full-board search would discover an
    // escape.  On a dense board that can burn the full connection budget (or
    // leave a dangling synthetic target) before it proves the local breakout
    // impossible.  Run a deterministic, strictly bounded local grid walk
    // instead.  Every emitted edge is checked with the same finite-segment,
    // hole and clearance predicates as a direct stub; the maze consumes this
    // path verbatim before it falls back to global routing.
    struct LOCAL_NODE
    {
        int          x = 0;
        int          y = 0;
        ROUTER_POINT point;
        long double  cost = 0.0L;
        std::size_t  parent = invalidIndex();
    };
    struct LOCAL_OPEN
    {
        long double cost = 0.0L;
        int         x = 0;
        int         y = 0;
        std::size_t node = 0;
    };
    struct LOCAL_OPEN_COMPARE
    {
        bool operator()( const LOCAL_OPEN& aLeft, const LOCAL_OPEN& aRight ) const
        {
            if( aLeft.cost != aRight.cost )
                return aLeft.cost > aRight.cost;
            if( aLeft.x != aRight.x )
                return aLeft.x > aRight.x;
            if( aLeft.y != aRight.y )
                return aLeft.y > aRight.y;
            return aLeft.node > aRight.node;
        }
    };

    const auto pointAt = [&]( int aX, int aY ) -> std::optional<ROUTER_POINT>
    {
        const long double x = static_cast<long double>( aPad.position.x )
                              + static_cast<long double>( aX ) * gridStep;
        const long double y = static_cast<long double>( aPad.position.y )
                              + static_cast<long double>( aY ) * gridStep;
        constexpr long double coordinateMinimum = static_cast<long double>(
                std::numeric_limits<std::int64_t>::min() );
        constexpr long double coordinateMaximum = static_cast<long double>(
                std::numeric_limits<std::int64_t>::max() );
        if( !std::isfinite( x ) || !std::isfinite( y ) || x <= coordinateMinimum
            || x >= coordinateMaximum || y <= coordinateMinimum || y >= coordinateMaximum )
        {
            return std::nullopt;
        }
        return ROUTER_POINT{ static_cast<std::int64_t>( std::llround( x ) ),
                             static_cast<std::int64_t>( std::llround( y ) ) };
    };

    const int maximumGridRadius = static_cast<int>( std::clamp<std::int64_t>(
            ( maximumEscape + gridStep - 1 ) / gridStep, 1, 64 ) );
    // A default 4.5 mm / 0.5 mm fanout envelope contains only 361 grid
    // points.  Retain a hard cap for finer user grids so a bad SMD escape
    // cannot turn planning into a board-wide route attempt.
    const std::size_t maximumLocalNodes = static_cast<std::size_t>( std::clamp(
            std::max( 128, aSettings.fanoutLandingSearchSteps * 64 ), 128, 2048 ) );
    const std::array<std::array<int, 2>, 8> localDirections = {
            std::array<int, 2>{ 1, 0 },   std::array<int, 2>{ -1, 0 },
            std::array<int, 2>{ 0, 1 },   std::array<int, 2>{ 0, -1 },
            std::array<int, 2>{ 1, 1 },   std::array<int, 2>{ -1, 1 },
            std::array<int, 2>{ -1, -1 }, std::array<int, 2>{ 1, -1 } };

    std::vector<LOCAL_NODE> nodes;
    nodes.reserve( maximumLocalNodes );
    nodes.push_back( { 0, 0, aPad.position, 0.0L, invalidIndex() } );
    std::map<std::pair<int, int>, long double> best;
    best.emplace( std::pair{ 0, 0 }, 0.0L );
    std::priority_queue<LOCAL_OPEN, std::vector<LOCAL_OPEN>, LOCAL_OPEN_COMPARE> open;
    open.push( { 0.0L, 0, 0, 0 } );

    while( !open.empty() && nodes.size() <= maximumLocalNodes )
    {
        if( aCancel && aCancel() )
            return std::nullopt;

        const LOCAL_OPEN currentOpen = open.top();
        open.pop();
        const auto known = best.find( { currentOpen.x, currentOpen.y } );
        if( known == best.end() || known->second != currentOpen.cost
            || currentOpen.node >= nodes.size() )
        {
            continue;
        }

        const LOCAL_NODE& current = nodes[currentOpen.node];
        const long double sourceDistance = std::hypotl(
                static_cast<long double>( current.point.x ) - aPad.position.x,
                static_cast<long double>( current.point.y ) - aPad.position.y );
        if( currentOpen.node != 0
            && sourceDistance >= static_cast<long double>( minimumEscape )
            && sourceDistance <= static_cast<long double>( maximumEscape )
            && fanoutSegmentAllowed( aBoard, aPad, current.point, aPad.layers.front(),
                                     aTargetLayer, aVia, aSettings, nearbyObstacles, aContext,
                                     aCancel, false ) )
        {
            std::vector<ROUTER_POINT> reversePath;
            for( std::size_t index = currentOpen.node; index != invalidIndex();
                 index = nodes[index].parent )
            {
                reversePath.push_back( nodes[index].point );
            }
            std::reverse( reversePath.begin(), reversePath.end() );
            if( reversePath.size() >= 2 && reversePath.front() == aPad.position )
            {
                aEscapePath = std::move( reversePath );
                return aEscapePath.back();
            }
        }

        for( const auto& direction : localDirections )
        {
            if( nodes.size() >= maximumLocalNodes )
                break;

            const int nextX = current.x + direction[0];
            const int nextY = current.y + direction[1];
            if( std::abs( nextX ) > maximumGridRadius || std::abs( nextY ) > maximumGridRadius )
                continue;

            const std::optional<ROUTER_POINT> nextPoint = pointAt( nextX, nextY );
            if( !nextPoint || *nextPoint == current.point )
                continue;

            const long double nextSourceDistance = std::hypotl(
                    static_cast<long double>( nextPoint->x ) - aPad.position.x,
                    static_cast<long double>( nextPoint->y ) - aPad.position.y );
            if( nextSourceDistance > static_cast<long double>( maximumEscape ) )
                continue;

            if( !fanoutTrackSegmentAllowed( aBoard, aPad, current.point, *nextPoint,
                                             aPad.layers.front(), nearbyObstacles, aContext,
                                             aCancel ) )
            {
                continue;
            }

            const long double stepCost = std::hypotl(
                    static_cast<long double>( direction[0] ) * gridStep,
                    static_cast<long double>( direction[1] ) * gridStep );
            const long double nextCost = current.cost + stepCost;
            const std::pair<int, int> key{ nextX, nextY };
            const auto prior = best.find( key );
            if( prior != best.end() && prior->second <= nextCost )
                continue;

            best[key] = nextCost;
            const std::size_t nextNode = nodes.size();
            nodes.push_back( { nextX, nextY, *nextPoint, nextCost, currentOpen.node } );
            open.push( { nextCost, nextX, nextY, nextNode } );
        }
    }

    return std::nullopt;

}

} // namespace


std::vector<std::size_t> BATCH_FANOUT::OrderedPins(
        const BOARD_SNAPSHOT& board, FANOUT_PIN_ORDER order, const ROUTER_CANCEL_CALLBACK& cancel )
{
#if defined( __clang__ )
#pragma clang fp contract(off)
#endif
    struct COMPONENT
    {
        std::int64_t id;
        std::vector<std::size_t> pins;
    };
    std::map<std::int64_t, COMPONENT> components;
    std::int64_t maxComponent = 0;
    for( const auto& pad : board.pads )
        maxComponent = std::max<std::int64_t>( maxComponent, pad.componentId );
    for( std::size_t i = 0; i < board.pads.size(); ++i )
    {
        const auto& pad = board.pads[i];
        if( !pad.isSmd || pad.layers.size() != 1 || pad.netCode <= 0
            || pad.isFanoutTarget || pad.isPlaneTarget )
            continue;
        const auto id = pad.componentId >= 0 ? pad.componentId : maxComponent + 1 + i;
        components[id].id = id;
        components[id].pins.push_back( i );
    }
    std::vector<COMPONENT> sorted;
    for( auto& [id, component] : components )
        sorted.push_back( std::move( component ) );
    std::sort( sorted.begin(), sorted.end(), []( const auto& a, const auto& b )
    { return a.pins.size() != b.pins.size() ? a.pins.size() > b.pins.size() : a.id < b.id; } );
    std::vector<std::size_t> result;
    auto distance = []( ROUTER_POINT a, ROUTER_POINT b )
    {
        const double x = static_cast<double>( a.x ) - b.x, y = static_cast<double>( a.y ) - b.y;
        return std::sqrt( x * x + y * y );
    };
    for( auto& component : sorted )
    {
        if( cancel && cancel() )
            return {};
        double x = 0, y = 0;
        for( auto index : component.pins )
        {
            x += board.pads[index].position.x;
            y += board.pads[index].position.y;
        }
        x /= component.pins.size(); y /= component.pins.size();
        std::map<std::size_t, double> scores;
        for( auto index : component.pins )
        {
            const auto& pin = board.pads[index];
            const double dx = pin.position.x - x, dy = pin.position.y - y;
            double score = std::sqrt( dx * dx + dy * dy );
            if( order == FANOUT_PIN_ORDER::OUTER_FIRST )
                score = -score;
            else if( order == FANOUT_PIN_ORDER::PIN_INDEX )
                score = 0;
            else if( order == FANOUT_PIN_ORDER::CLOSEST_ON_NET || order == FANOUT_PIN_ORDER::DENSEST_FIRST )
            {
                score = order == FANOUT_PIN_ORDER::CLOSEST_ON_NET ? std::numeric_limits<double>::max() : 0;
                for( std::size_t j = 0; j < board.pads.size(); ++j )
                {
                    if( cancel && cancel() )
                        return {};
                    const auto& other = board.pads[j];
                    if( j == index || other.isFanoutTarget || other.isPlaneTarget )
                        continue;
                    const double d = distance( pin.position, other.position );
                    if( order == FANOUT_PIN_ORDER::CLOSEST_ON_NET && other.netCode == pin.netCode )
                        score = std::min( score, d );
                    // Source uses 20 mm and only net-assigned SMD pins here.
                    if( order == FANOUT_PIN_ORDER::DENSEST_FIRST && other.netCode > 0
                        && other.isSmd && other.layers.size() == 1 && d <= 20000000.0 )
                        score -= 1;
                }
            }
            scores[index] = score;
        }
        std::stable_sort( component.pins.begin(), component.pins.end(), [&]( auto a, auto b )
        {
            if( scores[a] != scores[b] )
                return scores[a] < scores[b];
            const auto pa = board.pads[a].pinIndex >= 0 ? board.pads[a].pinIndex : a;
            const auto pb = board.pads[b].pinIndex >= 0 ? board.pads[b].pinIndex : b;
            return pa < pb;
        } );
        result.insert( result.end(), component.pins.begin(), component.pins.end() );
    }
    return result;
}


BOARD_SNAPSHOT BATCH_FANOUT::PrepareSnapshot( const BOARD_SNAPSHOT& aBoard,
                                              const AUTOROUTER_SETTINGS& aSettings,
                                              const ROUTER_CANCEL_CALLBACK& aCancel )
{
    if( !aSettings.enableFanout || !aSettings.allowVias || aSettings.maxFanoutPasses <= 0
        || aSettings.layers.size() < 2 )
    {
        return aBoard;
    }

    BOARD_SNAPSHOT result = aBoard;
    const FANOUT_CLEARANCE_CONTEXT fanoutContext = makeFanoutClearanceContext( result, aSettings );
    // Landing-point selection happens before the worker occupancy map exists.
    // Keep a planning-only obstacle list so a later fanout cannot select a
    // via or escape stub through an earlier fanout.  These records are not
    // copied to the returned snapshot; the real connection geometry is what
    // owns the final collision model after the pre-pass.
    BOARD_SNAPSHOT planningBoard = result;
    std::vector<std::size_t> landingForPad( result.pads.size(), invalidIndex() );

    // Create one deterministic landing pad for each SMD endpoint that still
    // participates in the current connection graph.  Do not fan out every
    // pad in a net during a route-only-unconnected run: pads already joined
    // by existing copper are not part of the proposal and must remain
    // untouched.  Synthetic landing pads are not added to
    // ROUTING_NET::padIndices because that list describes real board pads and
    // is used for widths, ordering and reporting.
    const auto orderedPins = OrderedPins( aBoard, aSettings.fanoutPinOrder, aCancel );
    std::map<int, std::vector<std::size_t>> fanoutPadsByNet;
    for( auto orderedPin : orderedPins )
    {
        const auto netIt = std::find_if( result.nets.begin(), result.nets.end(), [&]( const auto& net )
        { return net.netCode == aBoard.pads[orderedPin].netCode; } );
        if( netIt == result.nets.end() )
            continue;
        auto& net = *netIt;
        if( aCancel && aCancel() )
            return result;

        // A single-layer SMD pad needs a legal source-layer escape when
        // via-in-pad is disabled.  If via-in-pad is explicitly allowed, keep
        // the original connection and let the maze place the transition at
        // the pad centre, matching Freerouting's attachSmdAllowed behavior.
        if( net.connections.empty() || aSettings.allowViaInSmdPad )
            continue;

        std::vector<std::size_t> connectionPads;
        for( const auto& [source, target] : net.connections )
        {
            if( aCancel && aCancel() )
                return result;

            if( source < result.pads.size() && !result.pads[source].isPlaneTarget
                && std::find( connectionPads.begin(), connectionPads.end(), source )
                           == connectionPads.end() )
            {
                connectionPads.push_back( source );
            }

            if( target < result.pads.size() && !result.pads[target].isPlaneTarget
                && std::find( connectionPads.begin(), connectionPads.end(), target )
                           == connectionPads.end() )
            {
                connectionPads.push_back( target );
            }
        }

        auto& fanoutPads = fanoutPadsByNet[net.netCode];
        for( std::size_t padIndex : { orderedPin } )
        {
            if( aCancel && aCancel() )
                return result;

            if( padIndex >= result.pads.size() )
                continue;

            if( std::find( connectionPads.begin(), connectionPads.end(), padIndex )
                == connectionPads.end() )
            {
                continue;
            }

            const ROUTING_PAD& pad = result.pads[padIndex];
            if( !pad.isSmd || pad.isPlaneTarget || pad.layers.size() != 1 )
                continue;

            // A source-layer SMD that already has a selected plane target on
            // the same layer does not need a fanout or via at all.  Keeping
            // the original pad-to-plane edge avoids manufacturing an unused
            // buried/blind via and matches Freerouting's short plane stub.
            if( !net.planeTargetIndices.empty()
                && std::any_of( net.connections.begin(), net.connections.end(),
                                [&]( const auto& connection )
                                {
                                    const std::size_t source = connection.first;
                                    const std::size_t target = connection.second;
                                    if( source != padIndex || target >= result.pads.size()
                                        || !result.pads[target].isPlaneTarget )
                                    {
                                        return false;
                                    }

                                    const auto& targetLayers = result.pads[target].layers;
                                    return std::any_of(
                                            pad.layers.begin(), pad.layers.end(),
                                            [&]( int layer )
                                            {
                                                return std::find( targetLayers.begin(),
                                                                  targetLayers.end(), layer )
                                                       != targetLayers.end();
                                            } );
                                } ) )
            {
                continue;
            }

            // Match RoutingBoard.fanout's ViaRule traversal.  Its maze can
            // evaluate every rule alternative, so a synthetic native escape
            // must try each netclass/board profile in the same declaration
            // order rather than making the first profile a hard gate.
            const std::vector<ROUTING_VIA_DIMENSION> fanoutVias =
                    fanoutViasForNet( result, net.netCode, aSettings );
            if( fanoutVias.empty() )
                continue;

            bool fanoutPlaced = false;
            for( const ROUTING_VIA_DIMENSION& fanoutVia : fanoutVias )
            {
                for( const int layer : fanoutLayers( aSettings, pad.layers.front() ) )
                {
                    if( aCancel && aCancel() )
                        return result;

                    // result.pads grows below; keep the source value independent
                    // of vector reallocation while the planning obstacles are
                    // appended.
                    const ROUTING_PAD sourcePad = pad;
                    ROUTING_PAD landing = pad;
                    bool directStub = false;
                    std::vector<ROUTER_POINT> escapePath;
                    const std::optional<ROUTER_POINT> landingPoint =
                            fanoutLandingPoint( planningBoard, pad, layer, fanoutVia, aSettings,
                                                fanoutContext, aCancel, directStub, escapePath );
                    if( !landingPoint )
                    {
                        if( autorouterDebugEnabled() )
                        {
                            std::ostringstream message;
                            message << "fanout no landing net=" << net.netCode << " pad="
                                    << padIndex << " source=(" << pad.position.x << ','
                                    << pad.position.y << ") targetLayer=" << layer
                                    << " via=(" << fanoutVia.diameter << ','
                                    << fanoutVia.drill << ") planningObstacles="
                                    << planningBoard.obstacles.size();
                            autorouterDebugLog( message.str() );
                        }
                        continue;
                    }

                    if( autorouterDebugEnabled() )
                    {
                        std::ostringstream message;
                        message << "fanout landing net=" << net.netCode << " pad=" << padIndex
                                << " source=(" << pad.position.x << ',' << pad.position.y
                                << ") targetLayer=" << layer << " target=(" << landingPoint->x
                                << ',' << landingPoint->y << ") via=(" << fanoutVia.diameter
                                << ',' << fanoutVia.drill << ") planningObstacles="
                                << planningBoard.obstacles.size() << " directStub=" << directStub;
                        autorouterDebugLog( message.str() );
                    }

                    landing.position = *landingPoint;
                    // A plane fanout landing is shared by two stages.  Expose both
                    // layers so the first connection can make the source-layer
                    // escape and place its via at the landing point; the following
                    // plane connection is then forced to start on the destination
                    // layer by BatchAutorouter.  Ordinary SMD fanout retains the
                    // destination-only layer set and therefore inserts its via as
                    // part of the pad-to-landing connection.
                    if( !net.planeTargetIndices.empty() )
                        landing.layers = { pad.layers.front(), layer };
                    else
                        landing.layers = { layer };
                    landing.isPlaneTarget = false;
                    landing.isSmd = false;
                    landing.isFanoutTarget = true;
                    // Keep both sides of the transition on every synthetic
                    // landing, not only on plane fanout landings.  The maze
                    // engine can then certify the short source-layer stub and
                    // place the via at this escaped point instead of exploring
                    // the whole board looking for an arbitrary via location.
                    landing.fanoutSourceLayer = pad.layers.front();
                    landing.fanoutTargetLayer = layer;
                    landing.fanoutSourcePadIndex = padIndex;
                    landing.fanoutViaDiameter = fanoutVia.diameter;
                    landing.fanoutViaDrill = fanoutVia.drill;
                    landing.fanoutMinEscapeLength = std::max(
                            std::max<std::int64_t>( 1, sourcePad.radius + sourcePad.clearance
                                                           + sourcePad.trackWidth / 2 ),
                            std::max<std::int64_t>( 0, aSettings.fanoutMinEscapeLengthIU ) );
                    landing.fanoutMaxEscapeLength = aSettings.fanoutMaxEscapeLengthIU > 0
                            ? aSettings.fanoutMaxEscapeLengthIU
                            : landing.fanoutMinEscapeLength
                                      + 8 * std::max<std::int64_t>( 1, aSettings.gridStepIU );
                    landing.fanoutEscapePath = std::move( escapePath );
                    // The synthetic endpoint represents a via landing, not a
                    // second copy of the entire SMD copper shape.  Keeping the
                    // source clearance while clearing the pad radius prevents the
                    // final via transition from being rejected merely because
                    // its endpoint inherited the pad's bounding radius.
                    landing.radius = 0;

                    const std::size_t landingIndex = result.pads.size();
                    result.pads.push_back( std::move( landing ) );
                    landingForPad.resize( result.pads.size(), invalidIndex() );
                    landingForPad[padIndex] = landingIndex;
                    fanoutPads.push_back( padIndex );

                    // Reserve both the via stack and its source-layer escape in
                    // the planning copy.  Same-net fanouts remain mergeable, but
                    // foreign nets must see the exact copper/clearance envelope
                    // while choosing their own deterministic landing.
                    const std::int64_t viaRadius =
                            std::max<std::int64_t>( 1, fanoutVia.diameter / 2 );

                    ROUTING_OBSTACLE viaObstacle;
                    viaObstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
                    viaObstacle.netCode = sourcePad.netCode;
                    viaObstacle.layers = VIA_RULE::ThroughLayers( aSettings );
                    viaObstacle.start = *landingPoint;
                    viaObstacle.end = *landingPoint;
                    viaObstacle.radius = viaRadius;
                    viaObstacle.blocksTracks = true;
                    viaObstacle.blocksVias = true;
                    planningBoard.obstacles.push_back( std::move( viaObstacle ) );

                    // Reserve the entire preflighted source-layer escape,
                    // not only a straight stub. A later pin must see a
                    // preceding bent fanout as real copper while selecting
                    // its own landing; otherwise the synthetic pre-pass can
                    // create mutually crossing breakout paths that the
                    // isolated fanout stage has to undo.
                    for( std::size_t pathIndex = 1;
                         pathIndex < result.pads.back().fanoutEscapePath.size(); ++pathIndex )
                    {
                        ROUTING_OBSTACLE stubObstacle;
                        stubObstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
                        stubObstacle.netCode = sourcePad.netCode;
                        stubObstacle.layers = { sourcePad.layers.front() };
                        stubObstacle.start = result.pads.back().fanoutEscapePath[pathIndex - 1];
                        stubObstacle.end = result.pads.back().fanoutEscapePath[pathIndex];
                        stubObstacle.radius =
                                std::max<std::int64_t>( 1, sourcePad.trackWidth / 2 );
                        stubObstacle.blocksTracks = true;
                        stubObstacle.blocksVias = true;
                        planningBoard.obstacles.push_back( std::move( stubObstacle ) );
                    }

                    fanoutPlaced = true;
                    break;
                }

                if( fanoutPlaced )
                    break;
            }
        }

    }

    // Rewrite the graph only after all globally ordered pin landings have
    // been planned. Earlier fanout bridges must not be remapped a second time.
    for( auto& net : result.nets )
    {
        const auto& fanoutPads = fanoutPadsByNet[net.netCode];
        if( fanoutPads.empty() )
            continue;

        const std::vector<std::pair<std::size_t, std::size_t>> originalConnections =
                net.connections;
        std::vector<std::pair<std::size_t, std::size_t>> fanoutConnections;
        fanoutConnections.reserve( fanoutPads.size() + originalConnections.size() );

        for( std::size_t padIndex : fanoutPads )
        {
            const std::size_t landingIndex = landingForPad[padIndex];
            if( landingIndex != invalidIndex() )
                fanoutConnections.emplace_back( padIndex, landingIndex );
        }

        for( const auto& [source, target] : originalConnections )
        {
            const std::size_t mappedSource = source < landingForPad.size()
                                                     && landingForPad[source] != invalidIndex()
                                             ? landingForPad[source]
                                             : source;
            const std::size_t mappedTarget = target < landingForPad.size()
                                                     && landingForPad[target] != invalidIndex()
                                             ? landingForPad[target]
                                             : target;

            if( mappedSource != mappedTarget )
                fanoutConnections.emplace_back( mappedSource, mappedTarget );
        }

        net.connections = std::move( fanoutConnections );
    }

    return result;
}

std::vector<std::size_t> BATCH_FANOUT::PlaneTargetsFor( const BOARD_SNAPSHOT&,
                                                        const ROUTING_NET& aNet )
{
    return aNet.planeTargetIndices;
}


bool BATCH_FANOUT::HasFanoutWork( const BOARD_SNAPSHOT& aBoard, const ROUTING_NET& aNet )
{
    for( std::size_t index : aNet.planeTargetIndices )
    {
        if( index < aBoard.pads.size() && aBoard.pads[index].isPlaneTarget )
            return true;
    }

    for( std::size_t index : aNet.padIndices )
    {
        if( index < aBoard.pads.size() && aBoard.pads[index].isSmd
            && aBoard.pads[index].layers.size() == 1 )
        {
            return true;
        }
    }

    return false;
}

} // namespace KICAD_AUTOROUTER
