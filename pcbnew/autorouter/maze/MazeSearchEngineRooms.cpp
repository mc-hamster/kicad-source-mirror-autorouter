/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * KiCad adapter for the rectangular free-room search slice. This boundary
 * deliberately does not pretend bounding boxes are exact octagonal/polygonal
 * Freerouting tree shapes. Rejected proposals use the legacy fallback.
 */
#include "MazeSearchEngine.h"
#include "../AutorouterDebug.h"
#include "MazeExpansionEngine.h"
#include "MazeRipupResolver.h"
#include "MazeSearchEngine45Degree.h"
#include "../path/Connection.h"
#include "../geometry/planar/ContactGeometry.h"
#include "../geometry/planar/Simplex.h"
#include "../rules/ViaRule.h"
#include <algorithm>
#include <sstream>

namespace KICAD_AUTOROUTER
{

namespace
{

using PLANAR::INT_OCTAGON;

INT_OCTAGON octagonalEnvelope( const std::vector<ROUTER_POINT>& aPoints,
                               std::int64_t aExpansion )
{
    if( aPoints.empty() )
        return INT_OCTAGON::Empty();

    std::int64_t left = aPoints.front().x;
    std::int64_t right = left;
    std::int64_t bottom = aPoints.front().y;
    std::int64_t top = bottom;
    std::int64_t upperLeft = aPoints.front().x - aPoints.front().y;
    std::int64_t lowerRight = upperLeft;
    std::int64_t lowerLeft = aPoints.front().x + aPoints.front().y;
    std::int64_t upperRight = lowerLeft;
    for( const ROUTER_POINT& point : aPoints )
    {
        left = std::min( left, point.x );
        right = std::max( right, point.x );
        bottom = std::min( bottom, point.y );
        top = std::max( top, point.y );
        upperLeft = std::min( upperLeft, point.x - point.y );
        lowerRight = std::max( lowerRight, point.x - point.y );
        lowerLeft = std::min( lowerLeft, point.x + point.y );
        upperRight = std::max( upperRight, point.x + point.y );
    }
    return INT_OCTAGON( left, bottom, right, top, upperLeft, lowerRight,
                        lowerLeft, upperRight ).Normalize().Offset( aExpansion );
}


INT_OCTAGON octagonalEnvelope( const ROUTING_OBSTACLE& aObstacle,
                               std::int64_t aExpansion )
{
    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
        return INT_OCTAGON::FromBox( aObstacle.box ).Offset( aExpansion );
    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
        return octagonalEnvelope( { aObstacle.start, aObstacle.end }, aExpansion );
    return octagonalEnvelope( aObstacle.polygon, aExpansion );
}

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


bool isGeneralConvexRoomObstacle( const ROUTING_OBSTACLE& aObstacle, int aNet,
                                  bool aForVia )
{
    if( aObstacle.kind != ROUTER_OBSTACLE_KIND::POLYGON || aObstacle.isHole
        || !( aForVia ? aObstacle.blocksVias : aObstacle.blocksTracks )
        || !aObstacle.polygonHoles.empty() || aObstacle.radius != 0
        || isAxisAlignedRectangle( aObstacle.polygon )
        || ( aObstacle.netCode == aNet && !aObstacle.isKeepout ) )
    {
        return false;
    }

    return PLANAR::SIMPLEX::FromConvexPolygon( aObstacle.polygon ).has_value();
}


bool areaTouchesVia( const ROUTING_OBSTACLE& aArea, ROUTER_POINT aCenter,
                     std::int64_t aViaRadius )
{
    if( aArea.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
    {
        ROUTER_BOX expanded = aArea.box;
        expanded.minX -= aViaRadius;
        expanded.minY -= aViaRadius;
        expanded.maxX += aViaRadius;
        expanded.maxY += aViaRadius;
        return expanded.Contains( aCenter );
    }

    if( aArea.kind != ROUTER_OBSTACLE_KIND::POLYGON )
        return false;

    if( CONTACT_GEOMETRY::ContainsArea( aArea, aCenter ) )
        return true;

    // The straight convex subset has an exact rational support-line test.
    // Non-convex areas and holes stay conservative: a centre outside their
    // copper is not invented as a normal contact merely from an AABB hit.
    if( aArea.polygonHoles.empty() )
        if( const auto expanded = PLANAR::SIMPLEX::FromConvexPolygon(
                    aArea.polygon, std::max<std::int64_t>( 0, aViaRadius ) ) )
            return expanded->Contains( PLANAR::POINT( aCenter ) );

    return false;
}


void removeGeneratedCollinearNodes( ROUTING_CONNECTION& aConnection,
                                    const MAZE_SEARCH_ENGINE& aSearch )
{
    // FoundConnectionLocator normalizes corners while it walks one complete
    // source backtrack chain.  The native multilayer adapter reconstructs a
    // same-layer chain one corridor step at a time so it must perform that
    // harmless normalization at the adapter boundary.  This is done before
    // insertion, where no branch contact can live on an intermediate node.
    for( std::size_t middle = 1; middle + 1 < aConnection.nodes.size(); )
    {
        const ROUTER_NODE& first = aConnection.nodes[middle - 1];
        const ROUTER_NODE& current = aConnection.nodes[middle];
        const ROUTER_NODE& last = aConnection.nodes[middle + 1];
        const long double firstDx = static_cast<long double>( current.point.x ) - first.point.x;
        const long double firstDy = static_cast<long double>( current.point.y ) - first.point.y;
        const long double lastDx = static_cast<long double>( last.point.x ) - current.point.x;
        const long double lastDy = static_cast<long double>( last.point.y ) - current.point.y;
        const bool forwardCollinear = firstDx * lastDy == firstDy * lastDx
                                      && firstDx * lastDx + firstDy * lastDy >= 0;
        const ROUTING_EDGE_STYLE* style = aConnection.edgeStyles.empty()
                                                  ? nullptr
                                                  : &aConnection.edgeStyles[middle - 1];
        if( first.layer == current.layer && current.layer == last.layer
            && forwardCollinear
            && CanCollapseRouteEdges( aConnection, middle - 1, middle )
            && aSearch.CanInsertSegment( aConnection.netCode, first, last, style )
            && CollapseRouteNodes( aConnection, middle - 1, middle + 1 ) )
        {
            if( middle > 1 )
                --middle;
            continue;
        }
        ++middle;
    }
}

} // namespace


bool MAZE_SEARCH_ENGINE::hasGeneralConvexRoomGeometry( int aNet, int aLayer ) const
{
    for( const std::size_t index : obstacleIndices( aLayer ) )
    {
        const ROUTING_OBSTACLE& obstacle = m_board.obstacles[index];
        if( isGeneralConvexRoomObstacle( obstacle, aNet, false ) )
            return true;
    }

    return false;
}


std::vector<SHAPE_TREE_ENTRY> MAZE_SEARCH_ENGINE::roomObstacles(
        int net, int aLayer, bool aForVia, bool aSkipGeneralConvex,
        const ROUTER_CANCEL_CALLBACK& aCancel,
        const std::vector<ROOM_RIPUP_OBSTACLE>* aRipupObstacles ) const
{
    const auto radius = aForVia ? netViaRadius( net ) : netTrackRadius( net );
    std::vector<SHAPE_TREE_ENTRY> entries;
    int id = 1;
    auto addOctagon = [&]( INT_OCTAGON shape )
    {
        shape = shape.Normalize();
        if( shape.Dimension() >= 0 )
            entries.push_back( { shape.BoundingBox(), id++, 0, aLayer, 0,
                                 false, true, shape } );
    };
    auto add = [&]( ROUTER_BOX box, std::int64_t expansion )
    {
        addOctagon( INT_OCTAGON::FromBox( box ).Offset( expansion ) );
    };
    auto addSegment = [&]( ROUTER_POINT start, ROUTER_POINT end,
                           std::int64_t expansion )
    {
        addOctagon( octagonalEnvelope( { start, end }, expansion ) );
    };
    for( auto index : obstacleIndices( aLayer ) )
    {
        if( aCancel && aCancel() )
            return {};
        const auto& obstacle = m_board.obstacles[index];
        if( !( aForVia ? obstacle.blocksVias : obstacle.blocksTracks ) )
            continue;
        if( obstacle.netCode == net && !obstacle.isKeepout )
        {
            const bool ownHole = obstacle.isHole && obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT
                                 && endpointRadius( net, obstacle.start ) >= 0;
            if( !obstacle.isHole || ( !aForVia && ( ownHole || obstacle.isExistingRoute ) ) )
                continue;
        }
        // The drill-page tree remains rectangular.  The 45-degree room tree
        // retains this exact eight-support envelope instead of treating a
        // diagonal convex contour as its axis-aligned bounding box.
        if( aSkipGeneralConvex && isGeneralConvexRoomObstacle( obstacle, net, aForVia ) )
            continue;
        // One extra IU makes the room boundary legal under the host's
        // inclusive collision predicates; do not apply clearance twice.
        addOctagon( octagonalEnvelope( obstacle, obstacleExpansionRadius(
                obstacle, net, aLayer, aForVia, radius ) + 1 ) );
    }
    // Every attempt sees current copper, including through-via copper on
    // intermediate layers. No stale per-net tree survives add/remove/rip-up.
    const auto& routes = m_occupancy.Connections();
    for( std::size_t routeIndex = 0; routeIndex < routes.size(); ++routeIndex )
    {
        if( aCancel && aCancel() )
            return {};

        const ROUTING_CONNECTION& connection = routes[routeIndex];

        if( connection.netCode == net )
        {
            if( aForVia )
                for( std::size_t i = 1; i < connection.nodes.size(); ++i )
                    if( connection.nodes[i - 1].layer != connection.nodes[i].layer )
                    {
                        const auto p = connection.nodes[i].point;
                        add( { p.x, p.y, p.x, p.y }, 2 * netViaDrillRadius( net )
                                + m_board.holeToHoleClearance + 1 );
                    }
            continue;
        }
        for( std::size_t i = 1; i < connection.nodes.size(); ++i )
        {
            const std::size_t edge = i - 1;
            const bool representedByRipupRoom = aRipupObstacles
                    && std::any_of( aRipupObstacles->begin(), aRipupObstacles->end(),
                                    [&]( const ROOM_RIPUP_OBSTACLE& aObstacle )
                                    {
                                        return aObstacle.connectionIndex == routeIndex
                                               && aObstacle.shape.shapeIndex
                                                          == static_cast<int>( edge )
                                               && aObstacle.shape.layer == aLayer;
                                    } );
            if( representedByRipupRoom )
                continue;

            const auto& from = connection.nodes[i - 1];
            const auto& to = connection.nodes[i];
            const ROUTING_EDGE_STYLE& style = EdgeStyle( connection, edge );
            std::int64_t expansion;
            if( from.layer == to.layer )
            {
                if( from.layer != aLayer )
                    continue;
                const std::int64_t otherRadius = style.trackWidth > 0
                        ? style.trackWidth / 2 : netTrackRadius( connection.netCode );
                expansion = radius + otherRadius
                            + edgePairClearance( net, connection.netCode, aLayer, 0,
                                                 style.clearance );
            }
            else
            {
                if( !VIA_RULE::SpansLayer( m_settings, from.layer, to.layer, style, aLayer ) )
                    continue;
                const std::int64_t otherViaRadius = style.viaDiameter > 0
                        ? style.viaDiameter / 2 : netViaRadius( connection.netCode );
                const std::int64_t otherDrillRadius = style.viaDrill > 0
                        ? style.viaDrill / 2 : netViaDrillRadius( connection.netCode );
                expansion = radius + std::max(
                        otherViaRadius
                                + edgePairClearance( net, connection.netCode, aLayer, 0,
                                                     style.clearance ),
                        otherDrillRadius + m_board.holeClearance );
                if( aForVia )
                    expansion = std::max( expansion, netViaDrillRadius( net )
                            + otherDrillRadius + m_board.holeToHoleClearance );
            }
            addSegment( from.point, to.point, expansion + 1 );
        }
    }
    if( aForVia && !m_settings.allowViaInSmdPad )
        for( const auto& pad : m_board.pads )
            if( pad.netCode == net && pad.isSmd && isOnPadLayer( pad, aLayer ) )
                add( { pad.position.x, pad.position.y, pad.position.x, pad.position.y },
                     std::max<std::int64_t>( 1, pad.radius ) + radius + 1 );
    return entries;
}


std::vector<ROOM_RIPUP_OBSTACLE> MAZE_SEARCH_ENGINE::roomRipupObstacles(
        int net, int aLayer, bool aForVia, int aRetry, bool aFanout,
        const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    std::vector<ROOM_RIPUP_OBSTACLE> result;
    if( !m_useRoutableObstacleRooms || aForVia )
        return result;

    const std::int64_t radius = netTrackRadius( net );
    MAZE_RIPUP_RESOLVER resolver;
    MAZE_RIPUP_RESOLVER::CONTEXT context;
    context.startRipupCosts = std::max( 0, m_settings.startRipupCost );
    context.ripupPassNo = std::max( 1, aRetry + 1 );
    context.ripupCosts = context.startRipupCosts * context.ripupPassNo;
    context.isFanout = aFanout;

    // Java seeds one deterministic Random with ripupCosts.  The native
    // rectangular adapter calculates item-room costs before queue expansion,
    // so derive the same nextDouble sequence in stable route/edge order.
    constexpr std::uint64_t multiplier = 0x5DEECE66DULL;
    constexpr std::uint64_t addend = 0xBULL;
    constexpr std::uint64_t mask = ( 1ULL << 48 ) - 1;
    std::uint64_t randomState =
            ( static_cast<std::uint64_t>( context.ripupCosts ) ^ multiplier ) & mask;
    auto nextBits = [&]( int aBits )
    {
        randomState = ( randomState * multiplier + addend ) & mask;
        return randomState >> ( 48 - aBits );
    };
    auto nextDouble = [&]()
    {
        const std::uint64_t high = nextBits( 26 );
        const std::uint64_t low = nextBits( 27 );
        return static_cast<double>( ( high << 27 ) + low )
               / static_cast<double>( 1ULL << 53 );
    };

    const auto& routes = m_occupancy.Connections();
    std::size_t nextItemGroup = 0;
    for( std::size_t routeIndex = 0; routeIndex < routes.size(); ++routeIndex )
    {
        if( aCancel && aCancel() )
            return {};

        const ROUTING_CONNECTION& connection = routes[routeIndex];
        std::vector<std::size_t> edgeGroups;
        std::vector<std::size_t> edgeItemOrdinals;
        edgeGroups.reserve( connection.nodes.empty() ? 0 : connection.nodes.size() - 1 );
        edgeItemOrdinals.reserve( edgeGroups.capacity() );
        std::size_t routeItemCount = 0;
        for( std::size_t edge = 0; edge + 1 < connection.nodes.size(); ++edge )
        {
            const bool continuesTrace = edge > 0
                    && connection.nodes[edge - 1].layer == connection.nodes[edge].layer
                    && connection.nodes[edge].layer == connection.nodes[edge + 1].layer
                    && EdgeStyle( connection, edge - 1 ).trackWidth
                               == EdgeStyle( connection, edge ).trackWidth
                    && EdgeStyle( connection, edge - 1 ).clearance
                               == EdgeStyle( connection, edge ).clearance;
            if( !continuesTrace )
            {
                ++nextItemGroup;
                ++routeItemCount;
            }
            edgeGroups.push_back( nextItemGroup - 1 );
            edgeItemOrdinals.push_back( routeItemCount - 1 );
        }
        const std::vector<ROUTING_BOARD::ITEM_ID> routeItems = m_occupancy.Board()
                ? m_occupancy.Board()->RouteItems( connection )
                : std::vector<ROUTING_BOARD::ITEM_ID>{};
        const bool movableRoute = connection.isShoveMovable
                                  && ( !connection.isExistingBoardRoute
                                       || connection.isAutorouterOwned
                                       || m_settings.allowRipupExisting );
        if( !movableRoute || connection.netCode == net || !connection.complete
            || !HasValidEdgeStyles( connection ) )
        {
            continue;
        }

        for( std::size_t edge = 0; edge + 1 < connection.nodes.size(); ++edge )
        {
            const ROUTER_NODE& from = connection.nodes[edge];
            const ROUTER_NODE& to = connection.nodes[edge + 1];
            const ROUTING_EDGE_STYLE& style = EdgeStyle( connection, edge );
            std::vector<std::int64_t> additionalViaTraceHalfWidths;
            bool unrippableNormalContact = false;
            if( from.layer != to.layer && from.point == to.point )
            {
                const std::vector<int> viaLayers = VIA_RULE::LayersFor(
                        m_settings, from.layer, to.layer, &style );
                const auto onViaLayer = [&]( int aCandidateLayer )
                {
                    return std::find( viaLayers.begin(), viaLayers.end(), aCandidateLayer )
                           != viaLayers.end();
                };
                const std::int64_t viaRadius = style.viaDiameter > 0
                        ? style.viaDiameter / 2 : netViaRadius( connection.netCode );

                // Via.getNormalContacts() rejects rip-up as soon as a pin or
                // conduction area is present; forced insertion may still
                // move the via transactionally while preserving that contact.
                for( const ROUTING_PAD& pad : m_board.pads )
                {
                    if( pad.netCode != connection.netCode || pad.position != from.point )
                        continue;
                    if( std::any_of( viaLayers.begin(), viaLayers.end(),
                                    [&]( int aViaLayer )
                                    { return isOnPadLayer( pad, aViaLayer ); } ) )
                    {
                        unrippableNormalContact = true;
                        break;
                    }
                }
                for( const ROUTING_OBSTACLE& area : m_board.conductionAreas )
                {
                    if( unrippableNormalContact || area.netCode != connection.netCode )
                        continue;
                    for( const int viaLayer : viaLayers )
                    {
                        if( ( area.layers.empty()
                              || std::find( area.layers.begin(), area.layers.end(), viaLayer )
                                         != area.layers.end() )
                            && areaTouchesVia( area, from.point, viaRadius ) )
                        {
                            unrippableNormalContact = true;
                            break;
                        }
                    }
                }

                // Snapshot reconstruction keeps an imported via and each
                // contacted trace as separate native connections. Rebuild
                // the source Via.getNormalContacts() width/fixed-state view.
                for( std::size_t contactRouteIndex = 0;
                     contactRouteIndex < routes.size() && !unrippableNormalContact;
                     ++contactRouteIndex )
                {
                    if( contactRouteIndex == routeIndex )
                        continue;
                    const ROUTING_CONNECTION& contact = routes[contactRouteIndex];
                    if( contact.netCode != connection.netCode
                        || !HasValidEdgeStyles( contact ) )
                    {
                        continue;
                    }
                    for( std::size_t contactEdge = 0;
                         contactEdge + 1 < contact.nodes.size(); ++contactEdge )
                    {
                        const ROUTER_NODE& contactFrom = contact.nodes[contactEdge];
                        const ROUTER_NODE& contactTo = contact.nodes[contactEdge + 1];
                        const ROUTING_EDGE_STYLE& contactStyle = EdgeStyle(
                                contact, contactEdge );
                        if( contactFrom.layer != contactTo.layer )
                        {
                            if( contactFrom.point == from.point
                                && contactTo.point == from.point )
                                unrippableNormalContact = true;
                            continue;
                        }
                        if( !onViaLayer( contactFrom.layer )
                            || ( contactFrom.point != from.point
                                 && contactTo.point != from.point ) )
                        {
                            continue;
                        }
                        if( !contact.isShoveMovable )
                        {
                            unrippableNormalContact = true;
                            break;
                        }
                        additionalViaTraceHalfWidths.push_back(
                                contactStyle.trackWidth > 0
                                        ? contactStyle.trackWidth / 2
                                        : netTrackRadius( contact.netCode ) );
                    }
                }
            }

            if( unrippableNormalContact )
                continue;

            std::int64_t expansion = 0;
            if( from.layer == to.layer )
            {
                if( from.layer != aLayer )
                    continue;
                const std::int64_t otherRadius = style.trackWidth > 0
                        ? style.trackWidth / 2 : netTrackRadius( connection.netCode );
                expansion = radius + otherRadius
                            + edgePairClearance( net, connection.netCode, aLayer, 0,
                                                 style.clearance );
            }
            else
            {
                if( !VIA_RULE::SpansLayer( m_settings, from.layer, to.layer, style, aLayer ) )
                    continue;
                const std::int64_t otherViaRadius = style.viaDiameter > 0
                        ? style.viaDiameter / 2 : netViaRadius( connection.netCode );
                const std::int64_t otherDrillRadius = style.viaDrill > 0
                        ? style.viaDrill / 2 : netViaDrillRadius( connection.netCode );
                expansion = radius + std::max(
                        otherViaRadius
                                + edgePairClearance( net, connection.netCode, aLayer, 0,
                                                     style.clearance ),
                        otherDrillRadius + m_board.holeClearance );
            }

            const INT_OCTAGON octagon = octagonalEnvelope(
                    { from.point, to.point }, expansion + 1 );
            SHAPE_TREE_ENTRY entry{ octagon.BoundingBox(), 0,
                                    static_cast<int>( edge ), aLayer,
                                    connection.netCode, false, true, octagon };
            std::optional<CONNECTION> topologyConnection;
            if( routeItems.size() == routeItemCount )
                topologyConnection = CONNECTION::Get(
                        *m_occupancy.Board(), routeItems[edgeItemOrdinals[edge]] );

            const int ripupCost = resolver.CheckRipup(
                    connection, edge, netTrackRadius( connection.netCode ), context,
                    nextDouble(), additionalViaTraceHalfWidths,
                    topologyConnection ? &*topologyConnection : nullptr );
            if( ripupCost >= 0 )
                result.push_back( { std::move( entry ), edgeGroups[edge], ripupCost,
                                    routeIndex } );
        }
    }
    return result;
}


std::optional<ROUTING_CONNECTION> MAZE_SEARCH_ENGINE::findRoomConnection(
        const std::vector<ROUTING_TERMINAL>& aStarts,
        const std::vector<ROUTING_TERMINAL>& aTargets, int aRetry, int& aExpanded,
        const ROUTER_CANCEL_CALLBACK& aCancel,
        const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress ) const
{
    const int net = aStarts.front().pad.netCode;

    const auto radius = netTrackRadius( net );
    const auto margin = m_board.edgeClearance + radius + 1;
    const ROUTER_BOX bounds{ m_board.bounds.minX + margin, m_board.bounds.minY + margin,
                             m_board.bounds.maxX - margin, m_board.bounds.maxY - margin };
    std::optional<ROUTING_CONNECTION> best;
    std::int64_t bestRipupCost = 0;
    const auto started = std::chrono::steady_clock::now();
    for( const auto& layer : m_settings.layers )
    {
        if( !layer.enabled || ( aCancel && aCancel() ) )
            continue;
        auto terminals = [&]( const auto& source )
        {
            std::vector<ROOM_TERMINAL> result;
            for( const auto& terminal : source )
            {
                if( !isOnPadLayer( terminal.pad, layer.layerId ) )
                    continue;
                const auto start = terminal.pad.position;
                const auto end = terminal.segmentEnd.value_or( start );
                result.push_back( { start, end, terminal.padIndex } );
            }
            return result;
        };
        const auto starts = terminals( aStarts );
        const auto targets = terminals( aTargets );
        if( starts.empty() || targets.empty() )
            continue;
        const auto ripupEntries = roomRipupObstacles(
                net, layer.layerId, false, aRetry, false, aCancel );
        const auto entries = roomObstacles( net, layer.layerId, false, false, aCancel,
                                            &ripupEntries );
        // Geometric per-axis costs for the isolated no-via frontier. The
        // existing dialog's direction penalty is mapped here, not substituted
        // into the legacy queue's incompatible grid-normalized heuristic.
        const double preferred = std::max( 1, m_settings.traceLengthCost );
        const double against = preferred + std::max( 0, layer.directionCost ) / 10.0;
        const double horizontal = layer.preferredDirection == 2 ? against : preferred;
        const double vertical = layer.preferredDirection == 1 ? against : preferred;
        auto path = MAZE_SEARCH_ENGINE_45_DEGREE::FindConnection(
                bounds, entries, layer.layerId, net, starts, targets, std::max<std::int64_t>( 1, radius ),
                horizontal, vertical, m_settings.maxExpandedNodes, aExpanded, m_roomMetrics,
                aCancel, aProgress,
                static_cast<double>( std::max( 0, m_settings.bendCost ) )
                        * std::max( 1, m_settings.gridStepIU ), ripupEntries );
        // Keep the established rectangular frontier as a bounded transition
        // fallback until the octagonal drill frontier is connected.  General
        // convex layers must not collapse back to bounding rectangles.
        if( !path && !hasGeneralConvexRoomGeometry( net, layer.layerId )
            && aExpanded < m_settings.maxExpandedNodes )
        {
            path = MAZE_SEARCH_ENGINE_90_DEGREE::FindConnection(
                    bounds, entries, layer.layerId, net, starts, targets,
                    std::max<std::int64_t>( 1, radius ), horizontal, vertical,
                    m_settings.maxExpandedNodes, aExpanded, m_roomMetrics,
                    aCancel, aProgress, false,
                    static_cast<double>( std::max( 0, m_settings.bendCost ) )
                            * std::max( 1, m_settings.gridStepIU ), ripupEntries );
        }
        if( !path )
            continue;
        ROUTING_CONNECTION found;
        found.netCode = net;
        found.fromPadIndex = path->startOwner;
        found.toPadIndex = path->targetOwner;
        found.complete = true;
        found.cost = path->ripupCost;
        for( const auto& point : path->points )
            found.nodes.push_back( { point, layer.layerId } );
        bool legal = !found.nodes.empty();
        // The first node is an already-existing source/target contact. It may
        // legitimately lie inside its pad or inside the board-edge centre
        // margin. Validate generated edges, not a synthetic zero-length edge
        // at that existing contact.
        for( std::size_t i = 1; i < found.nodes.size() && legal; ++i )
        {
            const auto& from = found.nodes[i - 1];
            const auto& to = found.nodes[i];
            legal = CanUseSegment( net, from, to );
            if( !legal && autorouterDebugEnabled() )
            {
                std::ostringstream message;
                message << "ROOM_PATH_REJECTED net=" << net << " edge="
                        << i - 1 << " from=(" << from.point.x << ','
                        << from.point.y << ",L" << from.layer << ") to=(" << to.point.x
                        << ',' << to.point.y << ",L" << to.layer << ") ripup_cost="
                        << path->ripupCost << " ripped_groups="
                        << path->rippedObstacleGroups.size();
                autorouterDebugLog( message.str() );
            }
            found.cost += std::abs( static_cast<double>( to.point.x ) - from.point.x ) * horizontal
                          + std::abs( static_cast<double>( to.point.y ) - from.point.y ) * vertical;
        }
        if( legal && ( !best || found.cost < best->cost ) )
        {
            bestRipupCost = path->ripupCost;
            best = std::move( found );
        }
    }
    m_roomMetrics.routed = best.has_value();
    if( autorouterDebugEnabled() )
    {
        std::ostringstream log;
        log << "ROOM_SEARCH net=" << net << " rooms=" << m_roomMetrics.rooms
            << " doors=" << m_roomMetrics.doors << " sections=" << m_roomMetrics.sections
            << " destination_queries=" << m_roomMetrics.destinationQueries
            << " expanded=" << aExpanded << " accepted=" << m_roomMetrics.routed
            << " elapsed_ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - started ).count();
        autorouterDebugLog( log.str() );
    }
    if( aCancel && aCancel() )
        return std::nullopt;
    if( best )
    {
        // The batch/rip-up layer still consumes legacy route-cost units. Do
        // not leak geometric IU costs from the new frontier into that score.
        AUTOROUTE_CONTROL control( m_settings, net, aRetry, false,
                                   std::max( netTrackRadius( net ), netViaRadius( net ) ),
                                   isPureSmdNet( net ) );
        best->cost = bestRipupCost;
        for( std::size_t i = 1; i < best->nodes.size(); ++i )
        {
            const auto& from = best->nodes[i - 1];
            const auto& to = best->nodes[i];
            best->cost += control.WeightedTraceCost( to.layer, from.point, to.point )
                    + control.CongestionCost( m_occupancy.SegmentUsage( from, to, net ) )
                    + ( from.point != to.point ? m_settings.bendCost : 0 );
        }
    }
    return best;
}
std::optional<ROUTING_CONNECTION> MAZE_SEARCH_ENGINE::findMultilayerRoomConnection(
        const std::vector<ROUTING_TERMINAL>& starts, const std::vector<ROUTING_TERMINAL>& targets,
        int retry, int& expanded, const ROUTER_CANCEL_CALLBACK& cancel,
        const ROUTER_SEARCH_PROGRESS_CALLBACK& progress,
        const ROUTING_PAD* fanoutTarget ) const
{
    const int net = starts.front().pad.netCode;

    const auto radius = netTrackRadius( net );
    const auto margin = m_board.edgeClearance + radius + 1;
    const ROUTER_BOX bounds{ m_board.bounds.minX + margin, m_board.bounds.minY + margin,
                             m_board.bounds.maxX - margin, m_board.bounds.maxY - margin };
    const auto physical = VIA_RULE::ThroughLayers( m_settings );
    std::vector<ROOM_LAYER> layers;
    ROOM_VIA_SETTINGS via;
    const auto viaMargin = m_board.edgeClearance + std::max( radius, netViaRadius( net ) ) + 1;
    via.bounds = { m_board.bounds.minX + viaMargin, m_board.bounds.minY + viaMargin,
                   m_board.bounds.maxX - viaMargin, m_board.bounds.maxY - viaMargin };
    via.pageWidth = std::max<std::int64_t>( 10000, 10 * netViaRadius( net ) );
    via.attachSmd = m_settings.allowViaInSmdPad;
    const auto netRule = std::find_if( m_board.nets.begin(), m_board.nets.end(),
                                      [net]( const ROUTING_NET& aNet )
                                      { return aNet.netCode == net; } );
    if( netRule != m_board.nets.end() )
    {
        via.attachSmd = via.attachSmd
                        || std::any_of( netRule->viaProfiles.begin(),
                                        netRule->viaProfiles.end(),
                                        []( const ROUTING_VIA_PROFILE& aProfile )
                                        { return aProfile.attachSmdAllowed; } );
    }
    if( via.attachSmd )
        for( const auto& pad : m_board.pads )
            if( pad.netCode == net && pad.isSmd && !pad.isFanoutTarget && !pad.isPlaneTarget )
                for( std::size_t ordinal = 0; ordinal < physical.size(); ++ordinal )
                    if( isOnPadLayer( pad, physical[ordinal] ) )
                        via.pins.push_back( { pad.position, static_cast<int>( ordinal ), true } );
    bool pureSmd = true, hasPads = false;
    for( const auto& pad : m_board.pads )
        if( pad.netCode == net && !pad.isPlaneTarget && !pad.isFanoutTarget )
        { hasPads = true; pureSmd = pureSmd && pad.isSmd; }
    const bool plane = std::any_of( targets.begin(), targets.end(),
                                    []( const auto& t ) { return t.pad.isPlaneTarget; } );
    via.normalCost = MAZE_EXPANSION_ENGINE::NormalViaCost( std::max( radius, netViaRadius( net ) ),
            std::max( 0, plane ? m_settings.planeViaCost : m_settings.viaCost ), hasPads && pureSmd );
    via.canDrill = [&]( ROUTER_POINT point )
    {
        for( int layer : physical )
            if( !isPointAllowed( point, layer, net, true ) )
                return false;
        return true;
    };
    if( fanoutTarget )
    {
        via.stopAtFirstDrill = true;
        via.fanoutSourceLayer = fanoutTarget->fanoutSourceLayer;
        via.fanoutMinDistance = std::max<std::int64_t>(
                0, fanoutTarget->fanoutMinEscapeLength );
        via.fanoutMaxDistance = std::max<std::int64_t>(
                0, fanoutTarget->fanoutMaxEscapeLength );
        via.fanoutCenter = starts.front().pad.position;
        if( fanoutTarget->fanoutSourcePadIndex < m_board.pads.size() )
            via.fanoutCenter = m_board.pads[fanoutTarget->fanoutSourcePadIndex].position;
    }
    const auto started = std::chrono::steady_clock::now();
    // Keep the production fanout gate on its already-qualified rectangular
    // drill frontier while the octagonal first-drill ordering is compared to
    // the source. Ordinary multilayer routing uses the exact frontier below.
    const bool exactFrontier = fanoutTarget == nullptr;
    int nextObstacleId = 1;
    for( int id : physical )
    {
        if( cancel && cancel() )
            return std::nullopt;
        const auto setting = std::find_if( m_settings.layers.begin(), m_settings.layers.end(),
                                           [id]( const auto& l ) { return l.layerId == id; } );
        ROOM_LAYER layer;
        layer.id = id;
        layer.active = setting->enabled
                       && ( exactFrontier || !hasGeneralConvexRoomGeometry( net, id ) );
        layer.bounds = bounds;
        if( layer.active )
        {
            layer.ripupObstacles = roomRipupObstacles(
                    net, id, false, retry, fanoutTarget != nullptr, cancel );
            layer.obstacles = roomObstacles( net, id, false, false, cancel,
                                             &layer.ripupObstacles );
        }
        const double preferred = std::max( 1, m_settings.traceLengthCost );
        const double against = preferred + std::max( 0, setting->directionCost ) / 10.0;
        layer.horizontalCost = setting->preferredDirection == 2 ? against : preferred;
        layer.verticalCost = setting->preferredDirection == 1 ? against : preferred;
        layer.bendCost = static_cast<double>( std::max( 0, m_settings.bendCost ) )
                         * std::max( 1, m_settings.gridStepIU );
        auto append = [&]( const auto& terminals, auto& output, bool aTarget )
        {
            for( const auto& t : terminals )
            {
                if( !isOnPadLayer( t.pad, id ) )
                    continue;
                const auto a = t.pad.position, b = t.segmentEnd.value_or( a );
                if( exactFrontier || aTarget || a.x == b.x || a.y == b.y )
                    output.push_back( { a, b, t.padIndex } );
                else
                {
                    output.push_back( { a, a, t.padIndex } );
                    output.push_back( { b, b, t.padIndex } );
                }
            }
        };
        append( starts, layer.starts, false );
        append( targets, layer.targets, true );
        for( auto obstacle : roomObstacles( net, id, true, true, cancel ) )
        {
            obstacle.objectId = nextObstacleId++;
            via.obstacles.push_back( obstacle );
        }
        layers.push_back( std::move( layer ) );
    }
    const auto path = exactFrontier
            ? MAZE_SEARCH_ENGINE_45_DEGREE::FindMultilayerConnection(
                      layers, net, std::max<std::int64_t>( 1, radius ), via,
                      m_settings.maxExpandedNodes, expanded, m_roomMetrics,
                      cancel, progress )
            : MAZE_SEARCH_ENGINE_90_DEGREE::FindMultilayerConnection(
                      layers, net, std::max<std::int64_t>( 1, radius ), via,
                      m_settings.maxExpandedNodes, expanded, m_roomMetrics,
                      cancel, progress );
    std::optional<ROUTING_CONNECTION> result;
    if( path && !path->nodes.empty() )
    {
        ROUTING_CONNECTION found;
        found.netCode = net; found.fromPadIndex = path->startOwner; found.toPadIndex = path->targetOwner;
        found.complete = true; found.nodes = path->nodes;
        found.cost = path->ripupCost;
        found.isFanoutConnection = fanoutTarget != nullptr;
        bool legal = assignViaStyles( found );
        if( legal )
            removeGeneratedCollinearNodes( found, *this );
        for( std::size_t i = 1; i < found.nodes.size() && legal; ++i )
        {
            const std::size_t edge = i - 1;
            const bool isVia = i > 0 && found.nodes[edge].layer != found.nodes[i].layer;
            const ROUTING_EDGE_STYLE* style = isVia ? &found.edgeStyles[edge] : nullptr;
            legal = CanUseSegment( net, found.nodes[edge], found.nodes[i], isVia, style );
            if( !legal && autorouterDebugEnabled() )
            {
                const auto& from = found.nodes[edge];
                const auto& to = found.nodes[i];
                std::ostringstream message;
                message << "ROOM_DRILL_PATH_REJECTED net=" << net << " edge=" << edge
                        << " from=(" << from.point.x << ',' << from.point.y << ",L"
                        << from.layer << ") to=(" << to.point.x << ',' << to.point.y
                        << ",L" << to.layer << ") via=" << isVia
                        << " ripup_cost=" << path->ripupCost
                        << " ripped_groups=" << path->rippedObstacleGroups.size();
                autorouterDebugLog( message.str() );
            }
        }
        if( legal && !( cancel && cancel() ) )
        {
            AUTOROUTE_CONTROL control( m_settings, net, retry, plane,
                                       std::max( radius, netViaRadius( net ) ),
                                       hasPads && pureSmd );
            for( std::size_t i = 1; i < found.nodes.size(); ++i )
            {
                const auto& a = found.nodes[i - 1]; const auto& b = found.nodes[i];
                found.cost += a.layer != b.layer ? control.ViaCost()
                        : control.WeightedTraceCost( b.layer, a.point, b.point )
                          + control.CongestionCost( m_occupancy.SegmentUsage( a, b, net ) )
                          + ( a.point != b.point ? m_settings.bendCost : 0 );
            }
            result = std::move( found );
        }
    }
    m_roomMetrics.routed = result.has_value();
    if( autorouterDebugEnabled() )
    {
        std::ostringstream log;
        log << "ROOM_DRILL_SEARCH net=" << net << " rooms=" << m_roomMetrics.rooms
            << " doors=" << m_roomMetrics.doors << " sections=" << m_roomMetrics.sections
            << " pages=" << m_roomMetrics.drillPages << " drills=" << m_roomMetrics.drills
            << " transitions=" << m_roomMetrics.layerTransitions << " expanded=" << expanded
            << " destination_queries=" << m_roomMetrics.destinationQueries
            << " accepted=" << m_roomMetrics.routed << " elapsed_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - started ).count();
        autorouterDebugLog( log.str() );
    }
    return result;
}
} // namespace KICAD_AUTOROUTER
