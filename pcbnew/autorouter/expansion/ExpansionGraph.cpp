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

#include "ExpansionGraph.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <unordered_map>

#include "../drill/DrillPageArray.h"
#include "../geometry/planar/Simplex.h"

namespace KICAD_AUTOROUTER
{

namespace
{

bool isAxisAlignedRectangle( const std::vector<ROUTER_POINT>& aPolygon )
{
    if( aPolygon.size() != 4 )
        return false;

    std::vector<std::int64_t> x;
    std::vector<std::int64_t> y;
    x.reserve( aPolygon.size() );
    y.reserve( aPolygon.size() );

    for( const ROUTER_POINT& point : aPolygon )
    {
        x.push_back( point.x );
        y.push_back( point.y );
    }

    std::sort( x.begin(), x.end() );
    std::sort( y.begin(), y.end() );
    x.erase( std::unique( x.begin(), x.end() ), x.end() );
    y.erase( std::unique( y.begin(), y.end() ), y.end() );

    if( x.size() != 2 || y.size() != 2 )
        return false;

    return std::all_of( aPolygon.begin(), aPolygon.end(), [&]( const ROUTER_POINT& point )
    {
        return ( point.x == x.front() || point.x == x.back() )
               && ( point.y == y.front() || point.y == y.back() );
    } );
}

} // namespace


std::vector<ROUTER_NODE> EXPANSION_GRAPH::BuildLandmarks(
        const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
        const ROUTING_PAD& aStart, const ROUTING_PAD& aTarget, std::int64_t aTrackRadius,
        std::int64_t aViaRadius, int aNetCode )
{
    constexpr std::size_t maxLandmarks = 2048;
    std::vector<ROUTER_NODE> result;
    result.reserve( 96 );
    std::vector<ROUTER_NODE> obstacleLandmarks;
    obstacleLandmarks.reserve( std::min<std::size_t>( aBoard.obstacles.size() * 4, 32768 ) );
    // General convex support corners cannot share the broad phase's
    // one-feature-per-room sampling: for a rational corner, the first
    // floor/ceil realization may lie inside the offset shape while a sibling
    // realization is legal. Keep those candidates independently until they
    // are admitted below.
    std::vector<ROUTER_NODE> exactConvexLandmarks;
    exactConvexLandmarks.reserve( std::min<std::size_t>( aBoard.obstacles.size() * 8, 32768 ) );

    auto add = [&]( const ROUTER_POINT& aPoint, int aLayer )
    {
        if( aLayer < 0 || result.size() >= maxLandmarks )
            return;

        const ROUTER_NODE node{ aPoint, aLayer };
        if( std::find( result.begin(), result.end(), node ) == result.end() )
            result.push_back( node );
    };

    auto addForLayers = [&]( const ROUTER_POINT& aPoint, const std::vector<int>& aLayers )
    {
        for( int layer : aLayers )
            add( aPoint, layer );
    };

    // Do not stop collecting after the first few obstacles.  Large KiCad
    // boards often contain thousands of existing tracks, and a prefix of the
    // obstacle vector is spatially biased toward the first footprint.  Keep
    // the raw feature list here and sample it by board position below so the
    // visibility graph has escape points throughout the board.
    auto addObstacleForLayers = [&]( const ROUTER_POINT& aPoint,
                                     const std::vector<int>& aLayers )
    {
        for( int layer : aLayers )
            obstacleLandmarks.push_back( { aPoint, layer } );
    };

    auto addExactConvexForLayers = [&]( const ROUTER_POINT& aPoint,
                                        const std::vector<int>& aLayers )
    {
        for( int layer : aLayers )
            exactConvexLandmarks.push_back( { aPoint, layer } );
    };

    auto addPad = [&]( const ROUTING_PAD& aPad )
    {
        addForLayers( aPad.position, aPad.layers );
    };

    // The DRC resolver has already produced pair-specific values in the
    // snapshot.  Build the active-net lookup once per landmark build rather
    // than scanning the complete clearance-rule vector for every obstacle
    // corner.  Large real boards otherwise spend most of their route time in
    // this visibility-graph pre-pass.
    std::unordered_map<int, std::int64_t> netClearances;
    netClearances.reserve( aBoard.nets.size() );
    for( const ROUTING_NET& net : aBoard.nets )
        netClearances[net.netCode] = std::max<std::int64_t>( 0, net.clearance );

    std::unordered_map<int, std::int64_t> pairClearances;
    for( const ROUTING_CLEARANCE_RULE& rule : aBoard.clearanceRules )
    {
        int otherNetCode = 0;
        if( rule.firstNetCode == aNetCode )
            otherNetCode = rule.secondNetCode;
        else if( rule.secondNetCode == aNetCode )
            otherNetCode = rule.firstNetCode;

        if( otherNetCode != 0 && otherNetCode != aNetCode )
            pairClearances[otherNetCode] = std::max<std::int64_t>(
                    pairClearances[otherNetCode], rule.clearance );
    }

    auto clearanceFor = [&]( const ROUTING_OBSTACLE& aObstacle )
    {
        const std::int64_t activeClearance = netClearances.contains( aNetCode )
                                                      ? netClearances[aNetCode]
                                                      : 0;

        if( aObstacle.netCode != 0 && aObstacle.netCode != aNetCode )
        {
            const auto pair = pairClearances.find( aObstacle.netCode );
            if( pair != pairClearances.end() )
                return std::max( pair->second, aObstacle.clearance );

            const auto obstacleNet = netClearances.find( aObstacle.netCode );
            return std::max( { activeClearance, aObstacle.clearance,
                               obstacleNet == netClearances.end() ? 0 : obstacleNet->second } );
        }

        return std::max( activeClearance, aObstacle.clearance );
    };

    addPad( aStart );
    addPad( aTarget );

    for( const ROUTING_PAD& pad : aBoard.pads )
    {
        if( result.size() >= maxLandmarks )
            break;
        addPad( pad );
    }

    for( const ROUTING_OBSTACLE& obstacle : aBoard.obstacles )
    {
        const std::int64_t margin = obstacle.radius + aTrackRadius + clearanceFor( obstacle ) + 2;

        if( obstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
        {
            ROUTER_BOX expanded = obstacle.box;
            expanded.minX -= margin;
            expanded.minY -= margin;
            expanded.maxX += margin;
            expanded.maxY += margin;

            const ROUTER_POINT corners[] = {
                { expanded.minX, expanded.minY },
                { expanded.maxX, expanded.minY },
                { expanded.maxX, expanded.maxY },
                { expanded.minX, expanded.maxY },
                { ( expanded.minX + expanded.maxX ) / 2, expanded.minY },
                { expanded.maxX, ( expanded.minY + expanded.maxY ) / 2 },
                { ( expanded.minX + expanded.maxX ) / 2, expanded.maxY },
                { expanded.minX, ( expanded.minY + expanded.maxY ) / 2 }
            };

            for( const ROUTER_POINT& corner : corners )
                addObstacleForLayers( corner, obstacle.layers );
        }
        else if( obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
        {
            addObstacleForLayers( obstacle.start, obstacle.layers );
            addObstacleForLayers( obstacle.end, obstacle.layers );

            const long double dx = static_cast<long double>( obstacle.end.x )
                                   - obstacle.start.x;
            const long double dy = static_cast<long double>( obstacle.end.y )
                                   - obstacle.start.y;
            const long double length = std::sqrt( dx * dx + dy * dy );

            if( length > 0.0L )
            {
                for( int sign : { -1, 1 } )
                {
                    const auto offset = [&]( const ROUTER_POINT& aPoint )
                    {
                        return ROUTER_POINT{
                            aPoint.x + static_cast<std::int64_t>(
                                                std::llround( -dy / length * margin * sign ) ),
                            aPoint.y + static_cast<std::int64_t>(
                                                std::llround( dx / length * margin * sign ) ) };
                    };
                    addObstacleForLayers( offset( obstacle.start ), obstacle.layers );
                    addObstacleForLayers( offset( obstacle.end ), obstacle.layers );
                }
            }
        }
        else if( !obstacle.polygon.empty() )
        {
            // The rectangular room graph cannot represent a rotated or
            // general convex obstacle without closing real free space around
            // its corners.  Keep its broad-phase box landmarks for the
            // rectangular fallback, but also expose exact L-infinity-offset
            // support corners to the general visibility search.  The offset
            // is deliberately one IU wider than the exact copper clearance,
            // so every integral landmark remains strictly legal when the
            // route's exact predicate validates it.
            if( obstacle.polygonHoles.empty() && obstacle.radius == 0
                && !isAxisAlignedRectangle( obstacle.polygon ) )
            {
                const auto simplex = PLANAR::SIMPLEX::FromConvexPolygon(
                        obstacle.polygon, margin );

                if( simplex )
                {
                    for( std::size_t index = 0; index < simplex->Borders().size(); ++index )
                    {
                        // Convex support intersections are frequently
                        // rational (for example a 45-degree offset corner).
                        // Dropping those vertices from the board-wide
                        // visibility graph makes dense-board searches fall
                        // back to the coarse grid even though the exact
                        // convex predicates can route around the contour.
                        // A KiCad route must have integral IU coordinates,
                        // so retain its at-most-four surrounding candidates;
                        // MAZE_SEARCH_ENGINE is the sole authority that
                        // proves a candidate is outside the compensated
                        // contour before emitting an edge.  Never round one
                        // corner into a potentially illegal point here.
                        const auto bounds = simplex->Corner( index ).SurroundingBox();
                        if( !bounds )
                            continue;

                        for( const std::int64_t x : { bounds->minX, bounds->maxX } )
                            for( const std::int64_t y : { bounds->minY, bounds->maxY } )
                                addExactConvexForLayers( { x, y }, obstacle.layers );
                    }
                }
            }

            std::int64_t minX = obstacle.polygon.front().x;
            std::int64_t maxX = minX;
            std::int64_t minY = obstacle.polygon.front().y;
            std::int64_t maxY = minY;

            for( const ROUTER_POINT& vertex : obstacle.polygon )
            {
                minX = std::min( minX, vertex.x );
                maxX = std::max( maxX, vertex.x );
                minY = std::min( minY, vertex.y );
                maxY = std::max( maxY, vertex.y );
                addObstacleForLayers( vertex, obstacle.layers );
            }

            for( const ROUTER_POINT& corner : {
                     ROUTER_POINT{ minX - margin, minY - margin },
                     ROUTER_POINT{ maxX + margin, minY - margin },
                     ROUTER_POINT{ maxX + margin, maxY + margin },
                     ROUTER_POINT{ minX - margin, maxY + margin } } )
            {
                addObstacleForLayers( corner, obstacle.layers );
            }
        }
    }

    if( !exactConvexLandmarks.empty() && result.size() < maxLandmarks )
    {
        std::sort( exactConvexLandmarks.begin(), exactConvexLandmarks.end(),
                   []( const ROUTER_NODE& aLeft, const ROUTER_NODE& aRight )
                   {
                       if( aLeft.point.x != aRight.point.x )
                           return aLeft.point.x < aRight.point.x;
                       if( aLeft.point.y != aRight.point.y )
                           return aLeft.point.y < aRight.point.y;
                       return aLeft.layer < aRight.layer;
                   } );
        exactConvexLandmarks.erase(
                std::unique( exactConvexLandmarks.begin(), exactConvexLandmarks.end() ),
                exactConvexLandmarks.end() );

        // Keep a bounded but meaningful exact-geometry reserve even on a
        // board with thousands of pads.  The older vector filled with pad
        // centres first, so every general-angle obstacle vanished from the
        // base graph precisely on the dense boards that need it most.
        constexpr std::size_t maxExactConvexLandmarks = 512;
        const std::size_t selected = std::min( { exactConvexLandmarks.size(),
                                                 maxExactConvexLandmarks,
                                                 maxLandmarks } );
        const std::size_t ordinaryCapacity = maxLandmarks - selected;
        if( result.size() > ordinaryCapacity )
            result.resize( ordinaryCapacity );

        for( std::size_t index = 0; index < selected; ++index )
        {
            const std::size_t sourceIndex = selected == exactConvexLandmarks.size()
                                                    ? index
                                                    : index * exactConvexLandmarks.size() / selected;
            add( exactConvexLandmarks[sourceIndex].point,
                 exactConvexLandmarks[sourceIndex].layer );
        }
    }

    // Select at most one feature per coarse spatial bucket and physical
    // layer, then sample the sorted bucket list evenly when it is still too
    // large.  This preserves deterministic ordering while avoiding the old
    // "first 768 obstacles" bias that left far-side escape corridors out of
    // dense-board searches entirely.
    if( !obstacleLandmarks.empty() && result.size() < maxLandmarks )
    {
        const std::int64_t bucketSize = std::max<std::int64_t>(
                4000000, std::max<std::int64_t>( 1, aSettings.gridStepIU ) * 8 );
        auto bucketCoordinate = [bucketSize]( std::int64_t aCoordinate )
        {
            if( aCoordinate >= 0 )
                return aCoordinate / bucketSize;

            return -( ( -aCoordinate + bucketSize - 1 ) / bucketSize );
        };

        std::map<std::tuple<std::int64_t, std::int64_t, int>, ROUTER_NODE> buckets;
        for( const ROUTER_NODE& landmark : obstacleLandmarks )
        {
            buckets.try_emplace( { bucketCoordinate( landmark.point.x ),
                                   bucketCoordinate( landmark.point.y ), landmark.layer },
                                 landmark );
        }

        std::vector<ROUTER_NODE> sampled;
        sampled.reserve( buckets.size() );
        for( const auto& [unusedKey, landmark] : buckets )
        {
            (void) unusedKey;
            sampled.push_back( landmark );
        }

        const std::size_t capacity = maxLandmarks - result.size();
        const std::size_t selected = std::min( capacity, sampled.size() );
        for( std::size_t index = 0; index < selected; ++index )
        {
            const std::size_t sampledIndex = selected == sampled.size()
                                                     ? index
                                                     : ( index * sampled.size() ) / selected;
            add( sampled[sampledIndex].point, sampled[sampledIndex].layer );
        }
    }

    if( aBoard.boardOutline.size() >= 3 )
    {
        for( const ROUTER_POINT& point : aBoard.boardOutline )
        {
            for( const ROUTER_LAYER_SETTINGS& layer : aSettings.layers )
                add( point, layer.layerId );
        }
    }

    // DrillPageArray is the native counterpart of Freerouting's spatial via
    // pages.  Page centers are cheap, deterministic fallback visibility
    // landmarks; segment legality is still decided by MazeSearchEngine.
    const std::int64_t pageWidth = std::max<std::int64_t>( 1, aSettings.gridStepIU * 8 );
    DRILL_PAGE_ARRAY pages( aBoard.bounds, pageWidth );
    for( const ROUTER_POINT& center : pages.LandmarkCenters() )
    {
        for( const ROUTER_LAYER_SETTINGS& layer : aSettings.layers )
        {
            add( center, layer.layerId );
            if( result.size() >= maxLandmarks )
                break;
        }

        if( result.size() >= maxLandmarks )
            break;
    }

    (void) aViaRadius;
    return result;
}

} // namespace KICAD_AUTOROUTER
