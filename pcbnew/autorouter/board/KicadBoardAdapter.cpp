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

#include "KicadBoardAdapter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <unordered_map>

#include <board_design_settings.h>
#include <board_connected_item.h>
#include <connectivity/connectivity_data.h>
#include <connectivity/connectivity_algo.h>
#include <connectivity/connectivity_items.h>
#include <drc/drc_engine.h>
#include <footprint.h>
#include <geometry/shape_arc.h>
#include <geometry/shape_line_chain.h>
#include <geometry/shape_poly_set.h>
#include <layer_ids.h>
#include <netclass.h>
#include <pad.h>
#include <pcb_board_outline.h>
#include <pcb_track.h>
#include <project/net_settings.h>
#include <ratsnest/ratsnest_data.h>
#include <zone.h>
#include <trigo.h>


namespace KICAD_AUTOROUTER
{

namespace
{

std::string toStdString( const wxString& aString )
{
    return std::string( aString.ToUTF8() );
}


void appendLayers( std::vector<int>& aDestination, const LSET& aLayers )
{
    aLayers.RunOnLayers(
            [&]( PCB_LAYER_ID aLayer )
            {
                if( IsCopperLayer( aLayer ) )
                    aDestination.push_back( static_cast<int>( aLayer ) );
            } );
}


bool contains( const std::vector<std::string>& aValues, const std::string& aValue )
{
    return std::find( aValues.begin(), aValues.end(), aValue ) != aValues.end();
}


std::int64_t halfWidth( int aWidth )
{
    return std::max<std::int64_t>( 1, aWidth / 2 );
}


std::optional<VECTOR2I> findInteriorPoint( const SHAPE_POLY_SET& aPolygon )
{
    if( aPolygon.IsEmpty() )
        return std::nullopt;

    std::vector<VECTOR2I> candidates;
    const BOX2I bounds = aPolygon.BBox();
    candidates.push_back( bounds.GetCenter() );

    // The bounding-box center is ideal for rectangular pours, but it can land
    // in a concavity or a void.  Add the average of each outer contour and a
    // small deterministic grid of samples so irregular pours still receive a
    // usable landing point without a separate polygon triangulation pass.
    for( int outlineIndex = 0; outlineIndex < aPolygon.OutlineCount(); ++outlineIndex )
    {
        const SHAPE_LINE_CHAIN& outline = aPolygon.Outline( outlineIndex );

        if( outline.PointCount() == 0 )
            continue;

        std::int64_t sumX = 0;
        std::int64_t sumY = 0;

        for( const VECTOR2I& vertex : outline.CPoints() )
        {
            sumX += vertex.x;
            sumY += vertex.y;
        }

        candidates.emplace_back( static_cast<int>( sumX / outline.PointCount() ),
                                static_cast<int>( sumY / outline.PointCount() ) );
    }

    const int sampleCount = 17;
    for( int y = 0; y < sampleCount; ++y )
    {
        for( int x = 0; x < sampleCount; ++x )
        {
            const std::int64_t sampleX =
                    static_cast<std::int64_t>( bounds.GetLeft() )
                    + static_cast<std::int64_t>( bounds.GetWidth() ) * ( x + 1 )
                              / ( sampleCount + 1 );
            const std::int64_t sampleY =
                    static_cast<std::int64_t>( bounds.GetTop() )
                    + static_cast<std::int64_t>( bounds.GetHeight() ) * ( y + 1 )
                              / ( sampleCount + 1 );
            candidates.emplace_back( static_cast<int>( sampleX ), static_cast<int>( sampleY ) );
        }
    }

    auto pointToSegmentDistance = []( const VECTOR2I& aPoint, const VECTOR2I& aStart,
                                      const VECTOR2I& aEnd )
    {
        const long double dx = static_cast<long double>( aEnd.x ) - aStart.x;
        const long double dy = static_cast<long double>( aEnd.y ) - aStart.y;
        const long double lengthSquared = dx * dx + dy * dy;

        if( lengthSquared <= 0.0L )
        {
            const long double px = static_cast<long double>( aPoint.x ) - aStart.x;
            const long double py = static_cast<long double>( aPoint.y ) - aStart.y;
            return std::sqrt( static_cast<double>( px * px + py * py ) );
        }

        const long double projection = std::clamp(
                ( static_cast<long double>( aPoint.x ) - aStart.x ) * dx
                        + ( static_cast<long double>( aPoint.y ) - aStart.y ) * dy,
                0.0L, lengthSquared )
                                      / lengthSquared;
        const long double closestX = static_cast<long double>( aStart.x ) + projection * dx;
        const long double closestY = static_cast<long double>( aStart.y ) + projection * dy;
        const long double px = static_cast<long double>( aPoint.x ) - closestX;
        const long double py = static_cast<long double>( aPoint.y ) - closestY;
        return std::sqrt( static_cast<double>( px * px + py * py ) );
    };

    auto clearanceToBoundary = [&]( const VECTOR2I& aPoint )
    {
        double result = std::numeric_limits<double>::max();
        auto inspectChain = [&]( const SHAPE_LINE_CHAIN& aChain )
        {
            if( aChain.PointCount() < 2 )
                return;

            for( int index = 0; index < aChain.PointCount(); ++index )
            {
                result = std::min( result,
                                   pointToSegmentDistance(
                                           aPoint, aChain.CPoint( index ),
                                           aChain.CPoint( ( index + 1 ) % aChain.PointCount() ) ) );
            }
        };

        for( int outlineIndex = 0; outlineIndex < aPolygon.OutlineCount(); ++outlineIndex )
        {
            inspectChain( aPolygon.Outline( outlineIndex ) );
            for( int hole = 0; hole < aPolygon.HoleCount( outlineIndex ); ++hole )
                inspectChain( aPolygon.CHole( outlineIndex, hole ) );
        }

        return result;
    };

    std::optional<VECTOR2I> best;
    double                  bestClearance = -1.0;

    for( const VECTOR2I& candidate : candidates )
    {
        if( !aPolygon.PointInside( candidate ) || aPolygon.PointOnEdge( candidate ) )
            continue;

        const double clearance = clearanceToBoundary( candidate );
        if( !best || clearance > bestClearance
            || ( clearance == bestClearance
                 && ( candidate.x < best->x
                      || ( candidate.x == best->x && candidate.y < best->y ) ) ) )
        {
            best = candidate;
            bestClearance = clearance;
        }
    }

    if( best )
        return best;

    // A very narrow pour can have no strictly interior integer IU at the
    // chosen samples.  PointInside still guarantees that this fallback is on
    // the copper rather than in a hole; the worker treats same-net copper as
    // traversable and can therefore finish at the boundary safely.
    for( const VECTOR2I& candidate : candidates )
    {
        if( aPolygon.PointInside( candidate ) )
            return candidate;
    }

    return std::nullopt;
}


std::vector<VECTOR2I> findInteriorPoints( const SHAPE_POLY_SET& aPolygon )
{
    std::vector<VECTOR2I> result;

    // Prefer points in the aggregate filled polygon before considering the
    // individual outline/thermal islands.  SHAPE_POLY_SET does not promise
    // that Outline(0) is the largest usable copper island; choosing its first
    // centroid can therefore put a synthetic plane target on a thermal neck
    // or right against the board edge.  The aggregate helper tries the
    // bounding-box centre first and is the stable equivalent of Freerouting's
    // conduction-area interior target.
    if( const std::optional<VECTOR2I> interior = findInteriorPoint( aPolygon ) )
    {
        result.push_back( *interior );

        // A plane is an area, not a single point.  Keep the best central
        // landing and add a small deterministic set of interior alternatives
        // so distant pins can terminate at the nearest legal part of the
        // pour instead of routing all the way to one board-centre target.
        // The route-to-zone connection is still represented by ordinary
        // worker geometry; the filled polygon remains the authoritative host
        // connectivity source after acceptance.
        const BOX2I bounds = aPolygon.BBox();
        constexpr int sampleCount = 5;
        constexpr std::size_t maximumTargets = 9;

        for( int y = 1; y < sampleCount && result.size() < maximumTargets; ++y )
        {
            for( int x = 1; x < sampleCount && result.size() < maximumTargets; ++x )
            {
                const VECTOR2I candidate{
                    static_cast<int>( static_cast<std::int64_t>( bounds.GetLeft() )
                                      + static_cast<std::int64_t>( bounds.GetWidth() ) * x
                                                / sampleCount ),
                    static_cast<int>( static_cast<std::int64_t>( bounds.GetTop() )
                                      + static_cast<std::int64_t>( bounds.GetHeight() ) * y
                                                / sampleCount ) };

                if( aPolygon.PointInside( candidate ) && !aPolygon.PointOnEdge( candidate )
                    && std::find( result.begin(), result.end(), candidate ) == result.end() )
                {
                    result.push_back( candidate );
                }
            }
        }
    }

    for( int outlineIndex = 0; outlineIndex < aPolygon.OutlineCount(); ++outlineIndex )
    {
        const SHAPE_LINE_CHAIN& outline = aPolygon.Outline( outlineIndex );
        if( outline.PointCount() == 0 )
            continue;

        std::int64_t minX = outline.CPoint( 0 ).x;
        std::int64_t maxX = minX;
        std::int64_t minY = outline.CPoint( 0 ).y;
        std::int64_t maxY = minY;
        std::int64_t sumX = 0;
        std::int64_t sumY = 0;

        for( const VECTOR2I& point : outline.CPoints() )
        {
            minX = std::min<std::int64_t>( minX, point.x );
            maxX = std::max<std::int64_t>( maxX, point.x );
            minY = std::min<std::int64_t>( minY, point.y );
            maxY = std::max<std::int64_t>( maxY, point.y );
            sumX += point.x;
            sumY += point.y;
        }

        std::vector<VECTOR2I> candidates;
        candidates.emplace_back( static_cast<int>( sumX / outline.PointCount() ),
                                static_cast<int>( sumY / outline.PointCount() ) );
        candidates.emplace_back( static_cast<int>( ( minX + maxX ) / 2 ),
                                static_cast<int>( ( minY + maxY ) / 2 ) );

        constexpr int sampleCount = 7;
        for( int y = 1; y <= sampleCount; ++y )
        {
            for( int x = 1; x <= sampleCount; ++x )
            {
                candidates.emplace_back(
                        static_cast<int>( minX + ( maxX - minX ) * x / ( sampleCount + 1 ) ),
                        static_cast<int>( minY + ( maxY - minY ) * y / ( sampleCount + 1 ) ) );
            }
        }

        for( const VECTOR2I& candidate : candidates )
        {
            if( aPolygon.PointInside( candidate ) && !aPolygon.PointOnEdge( candidate )
                && std::find( result.begin(), result.end(), candidate ) == result.end() )
            {
                if( result.size() < 9 )
                    result.push_back( candidate );
                break;
            }
        }
    }

    return result;
}

} // namespace


ROUTER_POINT KICAD_BOARD_ADAPTER::point( const VECTOR2I& aPoint )
{
    return { aPoint.x, aPoint.y };
}


ROUTER_BOX KICAD_BOARD_ADAPTER::box( const BOX2I& aBox )
{
    return { aBox.GetLeft(), aBox.GetTop(), aBox.GetRight(), aBox.GetBottom() };
}


void KICAD_BOARD_ADAPTER::appendPolygon( std::vector<ROUTER_POINT>& aDestination,
                                         const SHAPE_LINE_CHAIN& aChain )
{
    for( const VECTOR2I& vertex : aChain.CPoints() )
        aDestination.push_back( point( vertex ) );
}


bool KICAD_BOARD_ADAPTER::netClassIncluded( const std::string& aNetClass,
                                            const AUTOROUTER_SETTINGS& aSettings ) const
{
    if( !aSettings.includeNetClasses.empty() && !contains( aSettings.includeNetClasses, aNetClass ) )
        return false;

    return !contains( aSettings.excludeNetClasses, aNetClass );
}


bool KICAD_BOARD_ADAPTER::netIncluded( const std::string& aNetName,
                                       const AUTOROUTER_SETTINGS& aSettings ) const
{
    if( !aSettings.includeNets.empty() && !contains( aSettings.includeNets, aNetName ) )
        return false;

    return !contains( aSettings.excludeNets, aNetName );
}


AUTOROUTER_SETTINGS KICAD_BOARD_ADAPTER::CreateDefaultSettings() const
{
    AUTOROUTER_SETTINGS settings;

    if( !m_board )
        return settings;

    m_board->GetEnabledLayers().RunOnLayers(
            [&]( PCB_LAYER_ID aLayer )
            {
                if( !IsCopperLayer( aLayer ) )
                    return;

                const int ordinal = static_cast<int>( CopperLayerToOrdinal( aLayer ) );
                settings.layers.push_back( { static_cast<int>( aLayer ), true,
                                             ordinal % 2 == 0 ? 1 : 2, 20, ordinal } );
            } );

    if( settings.layers.empty() )
    {
        settings.layers.push_back( { static_cast<int>( F_Cu ), true, 1, 20, 0 } );
        settings.layers.push_back( { static_cast<int>( B_Cu ), true, 2, 20,
                                     static_cast<int>( CopperLayerToOrdinal( B_Cu ) ) } );
    }

    return settings;
}


void KICAD_BOARD_ADAPTER::addBoardOutline( BOARD_SNAPSHOT& aSnapshot ) const
{
    if( !m_board )
        return;

    const_cast<BOARD*>( m_board )->UpdateBoardOutline();
    const PCB_BOARD_OUTLINE* outline = m_board->BoardOutline();

    if( !outline || !outline->HasOutline() )
        return;

    const SHAPE_POLY_SET& polygons = outline->GetOutline();

    if( polygons.OutlineCount() > 0 )
    {
        appendPolygon( aSnapshot.boardOutline, polygons.Outline( 0 ) );

        for( int hole = 0; hole < polygons.HoleCount( 0 ); ++hole )
        {
            std::vector<ROUTER_POINT> holePoints;
            appendPolygon( holePoints, polygons.CHole( 0, hole ) );
            aSnapshot.boardHoles.push_back( std::move( holePoints ) );
        }
    }
}


void KICAD_BOARD_ADAPTER::addNet( BOARD_SNAPSHOT& aSnapshot, int aNetCode,
                                  const wxString& aNetName, const wxString& aNetClass,
                                  int aPriority, std::int64_t aClearance,
                                  std::int64_t aViaDiameter, std::int64_t aViaDrill ) const
{
    if( std::find_if( aSnapshot.nets.begin(), aSnapshot.nets.end(),
                      [aNetCode]( const ROUTING_NET& aNet )
                      {
                          return aNet.netCode == aNetCode;
                      } )
        == aSnapshot.nets.end() )
    {
        ROUTING_NET net;
        net.netCode = aNetCode;
        net.name = toStdString( aNetName );
        net.netClass = toStdString( aNetClass );
        net.netClassPriority = aPriority;
        net.clearance = aClearance;
        net.viaDiameter = aViaDiameter;
        net.viaDrill = aViaDrill;
        aSnapshot.nets.push_back( std::move( net ) );
    }
}


void KICAD_BOARD_ADAPTER::addPads( BOARD_SNAPSHOT& aSnapshot,
                                   const AUTOROUTER_SETTINGS& aSettings ) const
{
    if( !m_board )
        return;

    std::map<const BOARD_CONNECTED_ITEM*, std::size_t> padIndices;
    for( PAD* pad : m_board->GetPads() )
    {
        if( !pad )
            continue;

        NETCLASS* netClass = pad->GetEffectiveNetClass();
        const std::string className = netClass ? toStdString( netClass->GetName() ) : std::string();
        const std::string netName = toStdString( pad->GetNetname() );

        std::vector<int> padLayers;
        appendLayers( padLayers, pad->GetLayerSet() );

        if( padLayers.empty() )
            continue;

        const int trackWidth = netClass ? netClass->GetTrackWidth() : -1;
        const int viaDiameter = netClass ? netClass->GetViaDiameter() : -1;
        const int viaDrill = netClass ? netClass->GetViaDrill() : -1;
        const std::int64_t width = trackWidth > 0 ? trackWidth : 150000;
        const std::int64_t diameter = viaDiameter > 0 ? viaDiameter : 600000;
        const std::int64_t drill = viaDrill > 0 ? viaDrill : 300000;
        std::int64_t       largestRadius = 0;

        // Keep rule metadata for filtered/foreign nets as well.  They are not
        // routed, but their pads, tracks and zones remain real obstacles and
        // KiCad's resolved pair clearance still depends on the foreign net's
        // netclass.
        if( pad->GetNetCode() > 0 )
        {
            addNet( aSnapshot, pad->GetNetCode(), pad->GetNetname(),
                    wxString::FromUTF8( className.c_str() ),
                    netClass ? netClass->GetPriority() : 0,
                    std::max( std::max( 0, m_board->GetDesignSettings().m_MinClearance ),
                              netClass ? std::max( 0, netClass->GetClearance() ) : 0 ),
                    diameter, drill );
        }

        // Keep copper on every physical layer, including layers disabled for
        // trace routing: through-vias must still clear those pads. Exclude
        // nonexistent layers reported by through-hole pad layer masks.
        std::vector<int> obstacleLayers = padLayers;
        obstacleLayers.erase(
                std::remove_if( obstacleLayers.begin(), obstacleLayers.end(),
                                [&]( int aLayer )
                                {
                                    return std::none_of(
                                            aSettings.layers.begin(), aSettings.layers.end(),
                                            [aLayer]( const ROUTER_LAYER_SETTINGS& aSetting )
                                            {
                                                return aSetting.layerId == aLayer;
                                            } );
                                } ),
                obstacleLayers.end() );

        // Keep an obstacle for a pad even when the settings filter has no
        // matching layer; this preserves snapshot safety for callers that
        // construct a deliberately minimal layer list.  Normal editor
        // settings always take the first branch above.
        if( obstacleLayers.empty() )
            obstacleLayers = padLayers;

        for( int layer : obstacleLayers )
        {
            const PCB_LAYER_ID layerId = static_cast<PCB_LAYER_ID>( layer );
            const BOX2I padBox = pad->GetBoundingBox( layerId );
            const int clearance = pad->GetOwnClearance( layerId );
            const VECTOR2I shapePos = pad->ShapePos( layerId );
            const VECTOR2I shapeSize = pad->GetSize( layerId );
            largestRadius = std::max<std::int64_t>(
                    largestRadius,
                    static_cast<std::int64_t>(
                            std::max( std::abs( padBox.GetRight() - padBox.GetLeft() ),
                                      std::abs( padBox.GetBottom() - padBox.GetTop() ) ) )
                            / 2 );

            ROUTING_OBSTACLE obstacle;
            obstacle.netCode = pad->GetNetCode();
            obstacle.boardItemId = pad->m_Uuid.AsString().ToStdString();
            obstacle.layers = { layer };

            obstacle.blocksTracks = true;
            obstacle.blocksVias = true;
            obstacle.clearance = std::max<std::int64_t>( 0, clearance );

            if( pad->GetShape( layerId ) == PAD_SHAPE::CIRCLE )
            {
                obstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
                obstacle.start = point( shapePos );
                obstacle.end = obstacle.start;
                obstacle.radius = std::max( shapeSize.x, shapeSize.y ) / 2;
                aSnapshot.obstacles.push_back( std::move( obstacle ) );
            }
            else if( pad->GetShape( layerId ) == PAD_SHAPE::OVAL )
            {
                // Match PAD::buildEffectiveShapes: an oval is an exact capsule,
                // not a many-edge polygon or its bounding rectangle.
                const VECTOR2I halfSize = shapeSize / 2;
                const int halfWidth = std::min( halfSize.x, halfSize.y );
                VECTOR2I halfLength( halfSize.x - halfWidth, halfSize.y - halfWidth );
                RotatePoint( halfLength, pad->GetOrientation() );
                obstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
                obstacle.start = point( shapePos - halfLength );
                obstacle.end = point( shapePos + halfLength );
                obstacle.radius = halfWidth;
                aSnapshot.obstacles.push_back( std::move( obstacle ) );
            }
            else if( pad->GetShape( layerId ) == PAD_SHAPE::RECTANGLE
                     && pad->GetOrientation().IsCardinal() )
            {
                // Only in this case is the AABB the actual copper rectangle.
                obstacle.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
                obstacle.box = box( padBox );
                aSnapshot.obstacles.push_back( std::move( obstacle ) );
            }
            else if( pad->GetShape( layerId ) == PAD_SHAPE::ROUNDRECT )
            {
                // A rounded rectangle is a rectangular core swept by a disk.
                // The worker already represents polygon + radius exactly;
                // avoid testing dozens of tessellated arc edges at every probe.
                const int radius = pad->GetRoundRectCornerRadius( layerId );
                const VECTOR2I halfCore = shapeSize / 2 - VECTOR2I( radius, radius );
                obstacle.radius = radius;
                if( halfCore.x <= 0 || halfCore.y <= 0 )
                {
                    VECTOR2I halfLength( std::max( 0, halfCore.x ),
                                         std::max( 0, halfCore.y ) );
                    RotatePoint( halfLength, pad->GetOrientation() );
                    obstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
                    obstacle.start = point( shapePos - halfLength );
                    obstacle.end = point( shapePos + halfLength );
                }
                else
                {
                    obstacle.kind = ROUTER_OBSTACLE_KIND::POLYGON;
                    for( VECTOR2I corner : { VECTOR2I( -halfCore.x, -halfCore.y ),
                                             VECTOR2I( halfCore.x, -halfCore.y ),
                                             VECTOR2I( halfCore.x, halfCore.y ),
                                             VECTOR2I( -halfCore.x, halfCore.y ) } )
                    {
                        RotatePoint( corner, pad->GetOrientation() );
                        obstacle.polygon.push_back( point( shapePos + corner ) );
                    }
                }
                aSnapshot.obstacles.push_back( std::move( obstacle ) );
            }
            else
            {
                // Preserve rotated, chamfered and custom copper
                // contours instead of closing legal channels with their AABB.
                // Clearance is applied by the pair resolver, not baked into
                // this polygon. Approximate arcs outward by at most 1 um.
                SHAPE_POLY_SET copper;
                pad->TransformShapeToPolygon( copper, layerId, 0, 1000, ERROR_OUTSIDE );
                for( int outline = 0; outline < copper.OutlineCount(); ++outline )
                {
                    ROUTING_OBSTACLE part = obstacle;
                    part.kind = ROUTER_OBSTACLE_KIND::POLYGON;
                    appendPolygon( part.polygon, copper.Outline( outline ) );
                    for( int hole = 0; hole < copper.HoleCount( outline ); ++hole )
                    {
                        std::vector<ROUTER_POINT> points;
                        appendPolygon( points, copper.CHole( outline, hole ) );
                        part.polygonHoles.push_back( std::move( points ) );
                    }
                    aSnapshot.obstacles.push_back( std::move( part ) );
                }
            }
        }

        if( pad->HasHole() )
        {
            const VECTOR2I padDrillSize = pad->GetDrillSize();
            const std::int64_t drillRadius =
                    std::max( std::abs( padDrillSize.x ), std::abs( padDrillSize.y ) ) / 2;

            if( drillRadius > 0 )
            {
                ROUTING_OBSTACLE hole;
                hole.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
                hole.netCode = pad->GetNetCode();
                hole.layers = padLayers;
                hole.start = point( pad->GetPosition() );
                hole.end = hole.start;
                hole.radius = drillRadius;
                hole.blocksTracks = true;
                hole.blocksVias = true;
                hole.isHole = true;
                aSnapshot.obstacles.push_back( std::move( hole ) );
            }
        }

        if( pad->GetNetCode() <= 0 || !netClassIncluded( className, aSettings )
            || !netIncluded( netName, aSettings ) )
        {
            continue;
        }

        std::vector<int> routeLayers = padLayers;
        routeLayers.erase( std::remove_if( routeLayers.begin(), routeLayers.end(),
                                      [&]( int aLayer )
                                      {
                                          return std::none_of(
                                                  aSettings.layers.begin(), aSettings.layers.end(),
                                                  [aLayer]( const ROUTER_LAYER_SETTINGS& aSetting )
                                                  {
                                                      return aSetting.enabled
                                                             && aSetting.layerId == aLayer;
                                                  } );
                                      } ),
                           routeLayers.end() );

        if( routeLayers.empty() )
            continue;

        ROUTING_PAD routingPad;
        routingPad.netCode = pad->GetNetCode();
        routingPad.sourceId = pad->m_Uuid.AsString().ToStdString();
        routingPad.position = point( pad->GetPosition() );
        routingPad.layers = routeLayers;
        routingPad.netClass = className;
        routingPad.netClassPriority = netClass ? netClass->GetPriority() : 0;
        routingPad.trackWidth = width;

        routingPad.radius = largestRadius;
        // KiCad's through-hole pad layer set spans the copper stack.  A
        // single-layer pad is therefore the stable host-side signal for
        // Freerouting's SMD fanout pre-pass.
        routingPad.isSmd = routeLayers.size() == 1;
        const PCB_LAYER_ID clearanceLayer = static_cast<PCB_LAYER_ID>( routeLayers.front() );
        routingPad.clearance =
                std::max<std::int64_t>( 0, pad->GetOwnClearance( clearanceLayer ) );
        const std::size_t padIndex = aSnapshot.pads.size();
        aSnapshot.pads.push_back( std::move( routingPad ) );
        padIndices.emplace( pad, padIndex );

        auto netIt = std::find_if( aSnapshot.nets.begin(), aSnapshot.nets.end(),
                                   [pad]( const ROUTING_NET& aNet )
                                   {
                                       return aNet.netCode == pad->GetNetCode();
                                   } );

        if( netIt != aSnapshot.nets.end() )
        {
            netIt->padIndices.push_back( padIndex );
            netIt->routable = true;
        }
    }

    // A filled same-net zone is a valid Freerouting plane target.  The worker
    // cannot query KiCad's live zone filler, so expose deterministic synthetic
    // landing pads at interior points while retaining the filled polygons as
    // same-net, traversable obstacles in addKeepouts().  This gives plane
    // nets the characteristic short-stub-to-plane behavior without copying a
    // zone into the proposed board geometry.
    for( ZONE* zone : m_board->Zones() )
    {
        if( !zone || zone->GetIsRuleArea() || zone->GetNetCode() <= 0 || !zone->IsFilled() )
            continue;

        auto netIt = std::find_if( aSnapshot.nets.begin(), aSnapshot.nets.end(),
                                   [zone]( const ROUTING_NET& aNet )
                                   {
                                       return aNet.netCode == zone->GetNetCode();
                                   } );

        if( netIt == aSnapshot.nets.end() )
            continue;

        zone->GetLayerSet().RunOnLayers(
                [&]( PCB_LAYER_ID aLayer )
                {
                    if( !IsCopperLayer( aLayer ) || !zone->HasFilledPolysForLayer( aLayer ) )
                        return;

                    const bool layerEnabled = std::any_of(
                            aSettings.layers.begin(), aSettings.layers.end(),
                            [aLayer]( const ROUTER_LAYER_SETTINGS& aSetting )
                            {
                                return aSetting.enabled && aSetting.layerId == aLayer;
                            } );

                    const std::shared_ptr<SHAPE_POLY_SET> filled =
                            zone->GetFilledPolysList( aLayer );

                    if( !filled )
                        return;

                    for( int region = 0; region < filled->OutlineCount(); ++region )
                    {
                        ROUTING_OBSTACLE area;
                        area.kind = ROUTER_OBSTACLE_KIND::POLYGON;
                        area.netCode = zone->GetNetCode();
                        area.layers = { static_cast<int>( aLayer ) };
                        appendPolygon( area.polygon, filled->Outline( region ) );
                        for( int hole = 0; hole < filled->HoleCount( region ); ++hole )
                        {
                            std::vector<ROUTER_POINT> points;
                            appendPolygon( points, filled->CHole( region, hole ) );
                            area.polygonHoles.push_back( std::move( points ) );
                        }
                        aSnapshot.conductionAreas.push_back( std::move( area ) );
                    }

                    // Disabled trace layers still carry physical plane contacts.
                    if( !layerEnabled )
                        return;

                    const std::vector<VECTOR2I> targets = findInteriorPoints( *filled );
                    if( targets.empty() )
                        return;

                    std::int64_t trackWidth = 150000;
                    if( !netIt->padIndices.empty() )
                    {
                        const std::size_t firstPad = netIt->padIndices.front();
                        if( firstPad < aSnapshot.pads.size()
                            && aSnapshot.pads[firstPad].trackWidth > 0 )
                        {
                            trackWidth = aSnapshot.pads[firstPad].trackWidth;
                        }
                    }

                    for( const VECTOR2I& target : targets )
                    {
                        ROUTING_PAD routingPad;
                        routingPad.netCode = netIt->netCode;
                        routingPad.position = point( target );
                        routingPad.layers = { static_cast<int>( aLayer ) };
                        routingPad.netClass = netIt->netClass;
                        routingPad.netClassPriority = netIt->netClassPriority;
                        routingPad.clearance = netIt->clearance;
                        routingPad.trackWidth = trackWidth;
                        routingPad.isPlaneTarget = true;

                        const std::size_t targetIndex = aSnapshot.pads.size();
                        aSnapshot.pads.push_back( std::move( routingPad ) );
                        netIt->planeTargetIndices.push_back( targetIndex );
                    }
                } );
    }

    const std::shared_ptr<CONNECTIVITY_DATA> connectivity = m_board->GetConnectivity();
    if( connectivity && !aSettings.allowRipupExisting )
    {
        // Search once for the whole snapshot. Directly adjacent pads are not
        // the connected set: tracks, vias and individual filled islands may
        // join distant terminals. Do not union by zone object or net code.
        const auto clusters = connectivity->GetConnectivityAlgo()->SearchClusters(
                CN_CONNECTIVITY_ALGO::CSM_CONNECTIVITY_CHECK );
        for( const auto& cluster : clusters )
        {
            std::vector<std::size_t> group;
            for( const CN_ITEM* item : *cluster )
            {
                if( !item->Valid() )
                    continue;
                const auto it = padIndices.find( item->Parent() );
                if( it != padIndices.end() )
                    group.push_back( it->second );
            }
            std::sort( group.begin(), group.end() );
            group.erase( std::unique( group.begin(), group.end() ), group.end() );
            if( group.size() < 2 )
                continue;
            const int netCode = aSnapshot.pads[group.front()].netCode;
            for( ROUTING_NET& net : aSnapshot.nets )
            {
                if( net.netCode == netCode )
                    net.connectedPadGroups.push_back( group );
            }
        }
    }

    for( ROUTING_NET& net : aSnapshot.nets )
    {
        if( net.padIndices.empty() )
            continue;

        auto endpointPad = [&]( const std::shared_ptr<const CN_ANCHOR>& aAnchor )
                -> std::optional<std::size_t>
        {
            if( !aAnchor )
                return std::nullopt;

            BOARD_CONNECTED_ITEM* parent = aAnchor->Parent();
            if( !parent )
                return std::nullopt;

            std::vector<PAD*> candidates;
            if( parent->Type() == PCB_PAD_T )
                candidates.push_back( static_cast<PAD*>( parent ) );
            else if( aAnchor->GetCluster() )
            {
                // The nearest pad can be several traces/vias away. The
                // anchor's cluster also preserves the particular zone island.
                for( const CN_ITEM* item : *aAnchor->GetCluster() )
                {
                    if( item->Valid() && item->Parent()->Type() == PCB_PAD_T )
                        candidates.push_back( static_cast<PAD*>( item->Parent() ) );
                }
            }

            if( candidates.empty() )
                return std::nullopt;

            PAD* best = nullptr;
            int  bestDistance = std::numeric_limits<int>::max();

            for( PAD* candidate : candidates )
            {
                if( !candidate || candidate->GetNetCode() != net.netCode
                    || !padIndices.contains( candidate ) )
                    continue;

                const int candidateDistance =
                        ( candidate->GetPosition() - aAnchor->Pos() ).EuclideanNorm();
                if( candidateDistance < bestDistance )
                {
                    best = candidate;
                    bestDistance = candidateDistance;
                }
            }

            if( !best )
                return std::nullopt;

            return padIndices.at( best );
        };

        if( net.padIndices.size() < 2 && net.planeTargetIndices.empty() )
            continue;

        // Allowing existing copper to be ripped up changes the operation from
        // a ratsnest completion into a full-net reroute.  Keeping only the
        // currently missing ratsnest edges while deleting the old copper
        // would leave the accepted proposal electrically incomplete.
        if( !aSettings.routeOnlyUnconnected || aSettings.allowRipupExisting )
        {
            for( std::size_t index = 1; index < net.padIndices.size(); ++index )
                net.connections.emplace_back( net.padIndices.front(), net.padIndices[index] );

            continue;
        }

        RN_NET* ratsnest = connectivity ? connectivity->GetRatsnestForNet( net.netCode ) : nullptr;


        if( !ratsnest )
        {
            // Boards created by importers and small unit-test boards may not
            // have a connectivity cache yet.  Preserve the useful behavior
            // of the batch router by falling back to a deterministic chain;
            // an existing, populated ratsnest remains authoritative.
            for( std::size_t index = 1; index < net.padIndices.size(); ++index )
                net.connections.emplace_back( net.padIndices.front(), net.padIndices[index] );

            continue;
        }

        auto endpointTerminal = [&]( const std::shared_ptr<const CN_ANCHOR>& anchor ) -> std::optional<std::size_t>
        {
            if( !anchor || !anchor->Parent() )
                return std::nullopt;
            if( auto* region = dynamic_cast<CN_ZONE_LAYER*>( anchor->Item() ) )
            {
                ROUTING_PAD terminal;
                terminal.netCode = net.netCode;
                terminal.netClass = net.netClass;
                terminal.trackWidth = net.padIndices.empty() ? 150000
                        : aSnapshot.pads[net.padIndices.front()].trackWidth;
                terminal.layers = { static_cast<int>( region->GetLayer() ) };
                terminal.isPlaneTarget = true;
                terminal.isExactTarget = true;
                VECTOR2I landing = anchor->Pos();
                // Move just inside this *specific* island, never toward an
                // arbitrary same-net fill sample (which may be another island).
                const int step = std::max<std::int64_t>( 1000, terminal.trackWidth / 2 );
                bool found = false;
                for( int radius : { step, 2 * step, 4 * step } )
                {
                    for( const VECTOR2I& direction : { VECTOR2I( 1, 0 ), VECTOR2I( 0, 1 ),
                            VECTOR2I( -1, 0 ), VECTOR2I( 0, -1 ), VECTOR2I( 1, 1 ),
                            VECTOR2I( -1, 1 ), VECTOR2I( -1, -1 ), VECTOR2I( 1, -1 ) } )
                    {
                        const VECTOR2I candidate = anchor->Pos() + direction * radius;
                        if( region->ContainsPoint( candidate )
                            && region->GetOutline().Distance( candidate, true ) > step + 1000 )
                        { landing = candidate; found = true; break; }
                    }
                    if( found )
                        break;
                }
                terminal.position = point( landing );
                const auto index = aSnapshot.pads.size();
                aSnapshot.pads.push_back( terminal );
                // Do not add to planeTargetIndices: that list would let a
                // failed search substitute an unrelated region of the net.
                return index;
            }
            return endpointPad( anchor );
        };

        std::set<std::pair<std::size_t, std::size_t>> uniqueConnections;
        for( const CN_EDGE& edge : ratsnest->GetEdges() )
        {
            const std::optional<std::size_t> source = endpointTerminal( edge.GetSourceNode() );
            const std::optional<std::size_t> target = endpointTerminal( edge.GetTargetNode() );


            if( !source || !target || *source == *target )
                continue;

            const auto connection = std::minmax( *source, *target );
            if( uniqueConnections.insert( connection ).second )
                net.connections.push_back( connection );
        }
    }
}


void KICAD_BOARD_ADAPTER::addExistingCopper( BOARD_SNAPSHOT& aSnapshot,
                                             const AUTOROUTER_SETTINGS& aSettings ) const
{
    if( !m_board )
        return;

    std::set<int> includedNets;
    for( const ROUTING_NET& net : aSnapshot.nets )
    {
        if( net.routable )
            includedNets.insert( net.netCode );
    }

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        if( !track )
            continue;

        const bool isIncludedNet = includedNets.contains( track->GetNetCode() );

        const std::string boardItemId = toStdString( track->m_Uuid.AsString() );
        const std::int64_t clearance =
                std::max( 0, track->GetOwnClearance( track->GetLayer() ) );
        std::vector<ROUTING_OBSTACLE> obstacles;

        auto makeObstacle = [&]()
        {
            ROUTING_OBSTACLE obstacle;
            obstacle.netCode = track->GetNetCode();
            obstacle.isExistingRoute = true;
            obstacle.boardItemId = boardItemId;
            appendLayers( obstacle.layers, track->GetLayerSet() );
            obstacle.blocksTracks = true;
            obstacle.blocksVias = true;
            obstacle.clearance = clearance;
            return obstacle;
        };

        if( track->Type() == PCB_VIA_T )
        {
            const PCB_VIA* via = static_cast<const PCB_VIA*>( track );
            for( PCB_LAYER_ID layer : via->GetLayerSet().Seq() )
            {
                ROUTING_OBSTACLE obstacle = makeObstacle();
                obstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
                obstacle.start = obstacle.end = point( via->GetPosition() );
                obstacle.radius = halfWidth( via->GetWidth( layer ) );
                obstacle.layers = { static_cast<int>( layer ) };
                obstacles.push_back( std::move( obstacle ) );
            }

            if( via->GetDrill() > 0 )
            {
                ROUTING_OBSTACLE hole = makeObstacle();
                hole.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
                hole.start = point( via->GetPosition() );
                hole.end = hole.start;
                hole.radius = via->GetDrill() / 2;
                hole.blocksTracks = true;
                hole.blocksVias = true;
                hole.isHole = true;
                obstacles.push_back( std::move( hole ) );
            }
        }
        else if( track->Type() == PCB_ARC_T )
        {
            // PCB_ARC is a real curved copper item.  Reducing it to its chord
            // leaves the middle of a bulging arc unprotected and can produce
            // a proposal that KiCad DRC rejects.  Convert the centerline to
            // a fine polyline and retain the UUID on every piece so a full
            // reroute removes the original arc atomically.
            const PCB_ARC* arc = static_cast<const PCB_ARC*>( track );
            const int       maxError = 1000; // one micrometre in KiCad IU
            int             actualError = 0;
            const SHAPE_ARC shape( arc->GetStart(), arc->GetMid(), arc->GetEnd(), 0 );
            const SHAPE_LINE_CHAIN polyline = shape.ConvertToPolyline( maxError, &actualError );
            const std::int64_t radius = halfWidth( track->GetWidth() )
                                        + std::max( 0, actualError );

            for( int index = 1; index < polyline.PointCount(); ++index )
            {
                if( polyline.CPoint( index - 1 ) == polyline.CPoint( index ) )
                    continue;

                ROUTING_OBSTACLE obstacle = makeObstacle();
                obstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
                obstacle.start = point( polyline.CPoint( index - 1 ) );
                obstacle.end = point( polyline.CPoint( index ) );
                obstacle.radius = radius;
                obstacles.push_back( std::move( obstacle ) );
            }

            if( obstacles.empty() )
            {
                ROUTING_OBSTACLE obstacle = makeObstacle();
                obstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
                obstacle.start = point( track->GetStart() );
                obstacle.end = point( track->GetEnd() );
                obstacle.radius = halfWidth( track->GetWidth() );
                obstacles.push_back( std::move( obstacle ) );
            }
        }
        else
        {
            ROUTING_OBSTACLE obstacle = makeObstacle();
            obstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
            obstacle.start = point( track->GetStart() );
            obstacle.end = point( track->GetEnd() );
            obstacle.radius = halfWidth( track->GetWidth() );
            obstacles.push_back( std::move( obstacle ) );
        }

        // A ratsnest-only run relies on the existing copper to represent the
        // already-connected portions of a net.  Do not remove that copper
        // from the collision model unless the user explicitly selected a
        // full-net reroute; otherwise accepting the proposal could delete
        // valid topology that was never regenerated.
        for( ROUTING_OBSTACLE& obstacle : obstacles )
        {
            if( aSettings.allowRipupExisting && isIncludedNet )
            {
                aSnapshot.removableExistingRoutes.push_back( std::move( obstacle ) );
            }
            else
            {
                aSnapshot.obstacles.push_back( std::move( obstacle ) );
            }
        }
    }
}


void KICAD_BOARD_ADAPTER::addKeepouts( BOARD_SNAPSHOT& aSnapshot ) const
{
    if( !m_board )
        return;

    for( ZONE* zone : m_board->Zones() )
    {
        if( !zone )
            continue;

        if( zone->GetIsRuleArea() )
        {
            if( !zone->GetDoNotAllowTracks() && !zone->GetDoNotAllowVias() )
                continue;

            const SHAPE_POLY_SET outline = zone->GetBoardOutline();

            for( int polygonIndex = 0; polygonIndex < outline.OutlineCount(); ++polygonIndex )
            {
                ROUTING_OBSTACLE obstacle;
                obstacle.kind = ROUTER_OBSTACLE_KIND::POLYGON;
                obstacle.netCode = zone->GetNetCode();
                appendLayers( obstacle.layers, zone->GetLayerSet() );
                appendPolygon( obstacle.polygon, outline.Outline( polygonIndex ) );
                for( int hole = 0; hole < outline.HoleCount( polygonIndex ); ++hole )
                {
                    std::vector<ROUTER_POINT> holePoints;
                    appendPolygon( holePoints, outline.CHole( polygonIndex, hole ) );
                    obstacle.polygonHoles.push_back( std::move( holePoints ) );
                }
                obstacle.blocksTracks = zone->GetDoNotAllowTracks();
                obstacle.blocksVias = zone->GetDoNotAllowVias();
                obstacle.isKeepout = true;
                obstacle.clearance = std::max( 0, zone->GetLocalClearance().value_or( 0 ) );
                aSnapshot.obstacles.push_back( std::move( obstacle ) );
            }

            continue;
        }

        // A copper pour is a conduction area, not a hard foreign-net
        // obstacle.  KiCad will refill the zone around the accepted tracks,
        // while the autorouter must be able to cross the pour and let the
        // resulting clearance void form normally.  Treating every filled
        // polygon as solid copper strands unrelated nets behind the plane
        // and also turns one large board into thousands of expensive polygon
        // obstacles.  Filled polygons are still used above as synthetic
        // same-net plane targets; only explicit rule areas are hard keepouts.
    }
}


void KICAD_BOARD_ADAPTER::addClearanceRules( BOARD_SNAPSHOT& aSnapshot,
                                             const AUTOROUTER_SETTINGS& aSettings ) const
{
    if( !m_board || aSnapshot.nets.size() < 2 )
        return;

    DRC_ENGINE* drcEngine = m_board->GetDesignSettings().m_DRCEngine.get();
    if( !drcEngine )
        return;

    // EvalClearanceBatch is the same resolver used by PNS and DRC.  Dummy
    // tracks are attached to the live board only as parents; they are never
    // inserted into the board container.  The result is copied into the
    // immutable snapshot before the worker starts, so custom rule conditions
    // do not get silently reduced to max(netclass clearance).
    std::vector<std::unique_ptr<PCB_TRACK>> dummyTracks;
    dummyTracks.reserve( aSnapshot.nets.size() );

    for( const ROUTING_NET& net : aSnapshot.nets )
    {
        auto track = std::make_unique<PCB_TRACK>( m_board );
        track->SetFlags( ROUTER_TRANSIENT );
        track->SetStart( { 0, 0 } );
        track->SetEnd( { 100, 0 } );
        track->SetWidth( 150000 );
        track->SetNetCode( net.netCode );
        dummyTracks.push_back( std::move( track ) );
    }

    std::vector<int> layers;
    for( const ROUTER_LAYER_SETTINGS& layer : aSettings.layers )
    {
        if( std::find( layers.begin(), layers.end(), layer.layerId ) == layers.end() )
            layers.push_back( layer.layerId );
    }

    if( layers.empty() )
        layers = { static_cast<int>( F_Cu ), static_cast<int>( B_Cu ) };

    // Clearance resolution is substantially more expensive than the rest of
    // snapshot capture because the DRC engine evaluates the complete rule
    // matcher.  Cache by the exact unordered net-code pair rather than only
    // by netclass: custom KiCad rules may select a net by name, footprint, or
    // another property that is not represented by the netclass matrix.
    // Keeping this cache exact preserves the live DRC resolver's semantics
    // while still avoiding duplicate lookups for the same pair/layer.
    using CLEARANCE_CACHE_KEY = std::tuple<int, int, int>;
    std::map<CLEARANCE_CACHE_KEY, int> clearanceCache;

    // Only a routable net and a net that contributes a real copper/drill
    // obstacle can affect the worker search.  Avoid resolving custom rules for
    // every pair of named-but-empty board nets; this is the difference between
    // a small constant amount of snapshot work and an O(N^2) DRC pass on
    // imported boards with many unused net definitions.
    std::set<int> relevantNetCodes;
    for( const ROUTING_NET& net : aSnapshot.nets )
    {
        if( !net.connections.empty() )
            relevantNetCodes.insert( net.netCode );
    }

    auto markObstacleNets = [&]( const std::vector<ROUTING_OBSTACLE>& aObstacles )
    {
        for( const ROUTING_OBSTACLE& obstacle : aObstacles )
        {
            if( obstacle.netCode != 0 )
                relevantNetCodes.insert( obstacle.netCode );
        }
    };

    markObstacleNets( aSnapshot.obstacles );
    markObstacleNets( aSnapshot.removableExistingRoutes );
    for( const ROUTING_PAD& pad : aSnapshot.pads )
    {
        if( pad.netCode != 0 )
            relevantNetCodes.insert( pad.netCode );
    }

    auto cacheKey = []( const ROUTING_NET& aFirst, const ROUTING_NET& aSecond,
                        int aLayer )
    {
        const auto codes = std::minmax( aFirst.netCode, aSecond.netCode );
        return CLEARANCE_CACHE_KEY{ codes.first, codes.second, aLayer };
    };

    for( std::size_t first = 0; first < aSnapshot.nets.size(); ++first )
    {
        const bool firstIsRouted = !aSnapshot.nets[first].connections.empty();

        for( std::size_t second = first + 1; second < aSnapshot.nets.size(); ++second )
        {
            if( !firstIsRouted && aSnapshot.nets[second].connections.empty() )
                continue;

            // The worker only creates copper for routable nets.  A pair of
            // foreign, non-routable nets cannot affect the legality of that
            // proposal, even though both may remain obstacles in the
            // snapshot.  Omitting those pairs preserves exact active-net vs
            // obstacle resolution and avoids quadratic DRC work when a
            // bounded corpus run selects only a small net prefix.
            if( !aSnapshot.nets[first].routable && !aSnapshot.nets[second].routable )
                continue;

            if( !relevantNetCodes.contains( aSnapshot.nets[first].netCode )
                && !relevantNetCodes.contains( aSnapshot.nets[second].netCode ) )
            {
                continue;
            }

            for( int layer : layers )
            {
                const CLEARANCE_CACHE_KEY key = cacheKey( aSnapshot.nets[first],
                                                          aSnapshot.nets[second], layer );
                auto cached = clearanceCache.find( key );

                if( cached == clearanceCache.end() )
                {
                    const PCB_LAYER_ID layerId = static_cast<PCB_LAYER_ID>( layer );
                    dummyTracks[first]->SetLayer( layerId );
                    dummyTracks[second]->SetLayer( layerId );

                    const DRC_CLEARANCE_BATCH batch = drcEngine->EvalClearanceBatch(
                            dummyTracks[first].get(), dummyTracks[second].get(), layerId );
                    cached = clearanceCache.emplace( key, std::max( 0, batch.clearance ) ).first;
                }

                aSnapshot.clearanceRules.push_back( { aSnapshot.nets[first].netCode,
                                                      aSnapshot.nets[second].netCode, layer,
                                                      cached->second } );
            }
        }
    }
}


std::shared_ptr<const BOARD_SNAPSHOT>
KICAD_BOARD_ADAPTER::CreateSnapshot( const AUTOROUTER_SETTINGS& aSettings ) const
{
    auto snapshot = std::make_shared<BOARD_SNAPSHOT>();

    if( !m_board )
        return snapshot;

    BOX2I bounds = m_board->GetBoardEdgesBoundingBox();

    if( bounds.GetWidth() <= 0 || bounds.GetHeight() <= 0 )
        bounds = m_board->GetBoundingBox();

    snapshot->bounds = box( bounds );
    snapshot->edgeClearance = m_board->GetDesignSettings().m_CopperEdgeClearance;
    snapshot->holeClearance = std::max<std::int64_t>(
            0, m_board->GetDesignSettings().m_HoleClearance );
    snapshot->holeToHoleClearance = std::max<std::int64_t>(
            0, m_board->GetDesignSettings().m_HoleToHoleMin );
    addBoardOutline( *snapshot );

    // Seed the snapshot with every named board net before collecting pads and
    // copper.  A net can be present only through an existing track, a filled
    // zone, or a filtered pad; retaining its netclass metadata is still
    // required to resolve pair-specific clearances for routes on other nets.
    for( NETINFO_ITEM* netInfo : m_board->GetNetInfo() )
    {
        if( !netInfo || netInfo->GetNetCode() <= 0 )
            continue;

        NETCLASS* netClass = netInfo->GetNetClass();
        addNet( *snapshot, netInfo->GetNetCode(), netInfo->GetNetname(),
                netClass ? netClass->GetName() : wxString(),
                netClass ? netClass->GetPriority() : 0,
                std::max( std::max( 0, m_board->GetDesignSettings().m_MinClearance ),
                          netClass ? std::max( 0, netClass->GetClearance() ) : 0 ),
                netClass ? std::max( 0, netClass->GetViaDiameter() ) : 0,
                netClass ? std::max( 0, netClass->GetViaDrill() ) : 0 );
    }

    addPads( *snapshot, aSettings );
    addExistingCopper( *snapshot, aSettings );
    addKeepouts( *snapshot );
    addClearanceRules( *snapshot, aSettings );

    // A custom DRC file can raise the manufacturing clearances above the
    // board defaults.  Capture the worst unconditional value so the worker
    // remains conservative after the live engine is no longer accessible.
    if( DRC_ENGINE* drcEngine = m_board->GetDesignSettings().m_DRCEngine.get() )
    {
        DRC_CONSTRAINT constraint;
        if( drcEngine->QueryWorstConstraint( HOLE_CLEARANCE_CONSTRAINT, constraint ) )
        {
            snapshot->holeClearance = std::max<std::int64_t>(
                    snapshot->holeClearance, constraint.GetValue().Min() );
        }

        if( drcEngine->QueryWorstConstraint( HOLE_TO_HOLE_CONSTRAINT, constraint ) )
        {
            snapshot->holeToHoleClearance = std::max<std::int64_t>(
                    snapshot->holeToHoleClearance, constraint.GetValue().Min() );
        }
    }
    snapshot->sourceBoardTimestamp = m_board->GetTimeStamp();

    return snapshot;
}


std::vector<std::unique_ptr<BOARD_ITEM>>
KICAD_BOARD_ADAPTER::CreatePreviewItems( const ROUTING_RESULT& aResult ) const
{
    std::vector<std::unique_ptr<BOARD_ITEM>> items;

    if( !m_board )
        return items;

    for( const ROUTING_SEGMENT& segment : aResult.segments )
    {
        auto track = std::make_unique<PCB_TRACK>( m_board );
        track->SetFlags( ROUTER_TRANSIENT );
        track->SetStart( { static_cast<int>( segment.start.x ), static_cast<int>( segment.start.y ) } );
        track->SetEnd( { static_cast<int>( segment.end.x ), static_cast<int>( segment.end.y ) } );
        track->SetLayer( static_cast<PCB_LAYER_ID>( segment.layer ) );
        track->SetWidth( static_cast<int>( segment.width ) );
        track->SetNetCode( segment.netCode );
        items.push_back( std::move( track ) );
    }

    for( const ROUTING_VIA& viaData : aResult.vias )
    {
        auto via = std::make_unique<PCB_VIA>( m_board );
        via->SetFlags( ROUTER_TRANSIENT );
        via->SetPosition( { static_cast<int>( viaData.position.x ),
                            static_cast<int>( viaData.position.y ) } );
        via->SetWidth( PADSTACK::TEMP_ALL_LAYERS, static_cast<int>( viaData.diameter ) );
        via->SetDrill( static_cast<int>( viaData.drill ) );

        const int topLayer = viaData.topLayer >= 0 ? viaData.topLayer : F_Cu;
        const int bottomLayer = viaData.bottomLayer >= 0 ? viaData.bottomLayer : B_Cu;
        const bool touchesFront = topLayer == static_cast<int>( F_Cu )
                                   || bottomLayer == static_cast<int>( F_Cu );
        const bool touchesBack = topLayer == static_cast<int>( B_Cu )
                                 || bottomLayer == static_cast<int>( B_Cu );

        // Freerouting primarily emits ordinary through vias, but preserving a
        // narrower transition is both legal KiCad and materially reduces
        // unnecessary copper obstacles on multilayer boards.
        via->SetViaType( touchesFront && touchesBack
                                 ? VIATYPE::THROUGH
                                 : ( touchesFront || touchesBack ? VIATYPE::BLIND
                                                                  : VIATYPE::BURIED ) );
        via->SetLayerPair( static_cast<PCB_LAYER_ID>( topLayer ),
                           static_cast<PCB_LAYER_ID>( bottomLayer ) );
        via->SetNetCode( viaData.netCode );
        items.push_back( std::move( via ) );
    }

    return items;
}


BOARD_ITEM* KICAD_BOARD_ADAPTER::FindBoardItem( const std::string& aUuid ) const
{
    if( !m_board || aUuid.empty() )
        return nullptr;

    return m_board->GetCachedItemById( KIID( aUuid ) );
}

} // namespace KICAD_AUTOROUTER
