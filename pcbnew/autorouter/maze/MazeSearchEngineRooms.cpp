/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * KiCad adapter for the rectangular free-room search slice. This boundary
 * deliberately does not pretend bounding boxes are exact octagonal/polygonal
 * Freerouting tree shapes. Rejected proposals use the legacy fallback.
 */
#include "MazeSearchEngine.h"
#include "../AutorouterDebug.h"
#include "MazeExpansionEngine.h"
#include "../rules/ViaRule.h"
#include <sstream>

namespace KICAD_AUTOROUTER
{
std::vector<SHAPE_TREE_ENTRY> MAZE_SEARCH_ENGINE::roomObstacles(
        int net, int aLayer, bool aForVia, const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    const auto radius = aForVia ? netViaRadius( net ) : netTrackRadius( net );
    std::vector<SHAPE_TREE_ENTRY> entries;
    int id = 1;
    auto add = [&]( ROUTER_BOX box, std::int64_t expansion )
    {
        box.minX -= expansion;
        box.minY -= expansion;
        box.maxX += expansion;
        box.maxY += expansion;
        entries.push_back( { box, id++, 0, aLayer, 0, false, true } );
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
        // One extra IU makes the room boundary legal under the host's
        // inclusive collision predicates; do not apply clearance twice.
        add( m_obstacleBounds[index], obstacleExpansionRadius(
                obstacle, net, aLayer, aForVia, radius ) + 1 );
    }
    // Every attempt sees current copper, including through-via copper on
    // intermediate layers. No stale per-net tree survives add/remove/rip-up.
    for( const auto& connection : m_occupancy.Connections() )
    {
        if( aCancel && aCancel() )
            return {};
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
            const auto& from = connection.nodes[i - 1];
            const auto& to = connection.nodes[i];
            std::int64_t expansion;
            if( from.layer == to.layer )
            {
                if( from.layer != aLayer )
                    continue;
                expansion = radius + netTrackRadius( connection.netCode )
                            + pairClearance( net, connection.netCode, aLayer );
            }
            else
            {
                // This port manufactures through vias. Entry/exit trace
                // layers do not limit their copper on the physical stack.
                expansion = radius + std::max(
                        netViaRadius( connection.netCode ) + pairClearance( net, connection.netCode, aLayer ),
                        netViaDrillRadius( connection.netCode ) + m_board.holeClearance );
                if( aForVia )
                    expansion = std::max( expansion, netViaDrillRadius( net )
                            + netViaDrillRadius( connection.netCode ) + m_board.holeToHoleClearance );
            }
            add( { std::min( from.point.x, to.point.x ), std::min( from.point.y, to.point.y ),
                   std::max( from.point.x, to.point.x ), std::max( from.point.y, to.point.y ) }, expansion + 1 );
        }
    }
    if( aForVia && !m_settings.allowViaInSmdPad )
        for( const auto& pad : m_board.pads )
            if( pad.netCode == net && pad.isSmd && isOnPadLayer( pad, aLayer ) )
                add( { pad.position.x, pad.position.y, pad.position.x, pad.position.y },
                     std::max<std::int64_t>( 1, pad.radius ) + radius + 1 );
    return entries;
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
                // Axis-aligned trace-interior attachment is exact. A diagonal
                // trace cannot be replaced by its filled rectangular envelope.
                if( start.x == end.x || start.y == end.y )
                    result.push_back( { start, end, terminal.padIndex } );
                else
                {
                    result.push_back( { start, start, terminal.padIndex } );
                    result.push_back( { end, end, terminal.padIndex } );
                }
            }
            return result;
        };
        const auto starts = terminals( aStarts );
        const auto targets = terminals( aTargets );
        if( starts.empty() || targets.empty() )
            continue;
        const auto entries = roomObstacles( net, layer.layerId, false, aCancel );
        // Geometric per-axis costs for the isolated no-via frontier. The
        // existing dialog's direction penalty is mapped here, not substituted
        // into the legacy queue's incompatible grid-normalized heuristic.
        const double preferred = std::max( 1, m_settings.traceLengthCost );
        const double against = preferred + std::max( 0, layer.directionCost ) / 10.0;
        const double horizontal = layer.preferredDirection == 2 ? against : preferred;
        const double vertical = layer.preferredDirection == 1 ? against : preferred;
        const auto path = MAZE_SEARCH_ENGINE_90_DEGREE::FindConnection(
                bounds, entries, layer.layerId, net, starts, targets, std::max<std::int64_t>( 1, radius ),
                horizontal, vertical, m_settings.maxExpandedNodes, aExpanded, m_roomMetrics,
                aCancel, aProgress, false,
                static_cast<double>( std::max( 0, m_settings.bendCost ) )
                        * std::max( 1, m_settings.gridStepIU ) );
        if( !path )
            continue;
        ROUTING_CONNECTION found;
        found.netCode = net;
        found.fromPadIndex = path->startOwner;
        found.toPadIndex = path->targetOwner;
        found.complete = true;
        for( const auto& point : path->points )
            found.nodes.push_back( { point, layer.layerId } );
        bool legal = !found.nodes.empty();
        for( std::size_t i = 0; i < found.nodes.size() && legal; ++i )
        {
            const auto& from = found.nodes[i == 0 ? i : i - 1];
            const auto& to = found.nodes[i];
            legal = CanUseSegment( net, from, to );
            found.cost += std::abs( static_cast<double>( to.point.x ) - from.point.x ) * horizontal
                          + std::abs( static_cast<double>( to.point.y ) - from.point.y ) * vertical;
        }
        if( legal && ( !best || found.cost < best->cost ) )
            best = std::move( found );
    }
    m_roomMetrics.routed = best.has_value();
    if( autorouterDebugEnabled() )
    {
        std::ostringstream log;
        log << "ROOM_SEARCH net=" << net << " rooms=" << m_roomMetrics.rooms
            << " doors=" << m_roomMetrics.doors << " sections=" << m_roomMetrics.sections
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
        AUTOROUTE_CONTROL control( m_settings, net, aRetry );
        best->cost = 0;
        for( std::size_t i = 1; i < best->nodes.size(); ++i )
        {
            const auto& from = best->nodes[i - 1];
            const auto& to = best->nodes[i];
            best->cost += control.TraceCost( std::hypot(
                    static_cast<double>( to.point.x ) - from.point.x,
                    static_cast<double>( to.point.y ) - from.point.y ) )
                    + control.DirectionCost( to.layer, from.point, to.point )
                    + control.CongestionCost( m_occupancy.SegmentUsage( from, to, net ) )
                    + ( from.point != to.point ? m_settings.bendCost : 0 );
        }
    }
    return best;
}
std::optional<ROUTING_CONNECTION> MAZE_SEARCH_ENGINE::findMultilayerRoomConnection(
        const std::vector<ROUTING_TERMINAL>& starts, const std::vector<ROUTING_TERMINAL>& targets,
        int retry, int& expanded, const ROUTER_CANCEL_CALLBACK& cancel,
        const ROUTER_SEARCH_PROGRESS_CALLBACK& progress ) const
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
    const auto started = std::chrono::steady_clock::now();
    int nextObstacleId = 1;
    for( int id : physical )
    {
        if( cancel && cancel() )
            return std::nullopt;
        const auto setting = std::find_if( m_settings.layers.begin(), m_settings.layers.end(),
                                           [id]( const auto& l ) { return l.layerId == id; } );
        ROOM_LAYER layer;
        layer.id = id; layer.active = setting->enabled; layer.bounds = bounds;
        layer.obstacles = roomObstacles( net, id, false, cancel );
        const double preferred = std::max( 1, m_settings.traceLengthCost );
        const double against = preferred + std::max( 0, setting->directionCost ) / 10.0;
        layer.horizontalCost = setting->preferredDirection == 2 ? against : preferred;
        layer.verticalCost = setting->preferredDirection == 1 ? against : preferred;
        layer.bendCost = static_cast<double>( std::max( 0, m_settings.bendCost ) )
                         * std::max( 1, m_settings.gridStepIU );
        auto append = [&]( const auto& terminals, auto& output )
        {
            for( const auto& t : terminals )
            {
                if( !isOnPadLayer( t.pad, id ) )
                    continue;
                const auto a = t.pad.position, b = t.segmentEnd.value_or( a );
                if( a.x == b.x || a.y == b.y )
                    output.push_back( { a, b, t.padIndex } );
                else
                {
                    output.push_back( { a, a, t.padIndex } );
                    output.push_back( { b, b, t.padIndex } );
                }
            }
        };
        append( starts, layer.starts ); append( targets, layer.targets );
        for( auto obstacle : roomObstacles( net, id, true, cancel ) )
        {
            obstacle.objectId = nextObstacleId++;
            via.obstacles.push_back( obstacle );
        }
        layers.push_back( std::move( layer ) );
    }
    const auto path = MAZE_SEARCH_ENGINE_90_DEGREE::FindMultilayerConnection(
            layers, net, std::max<std::int64_t>( 1, radius ), via,
            m_settings.maxExpandedNodes, expanded, m_roomMetrics, cancel, progress );
    std::optional<ROUTING_CONNECTION> result;
    if( path && !path->nodes.empty() )
    {
        bool legal = true;
        for( std::size_t i = 0; i < path->nodes.size() && legal; ++i )
            legal = CanUseSegment( net, path->nodes[i == 0 ? 0 : i - 1], path->nodes[i] );
        if( legal && !( cancel && cancel() ) )
        {
            ROUTING_CONNECTION found;
            found.netCode = net; found.fromPadIndex = path->startOwner; found.toPadIndex = path->targetOwner;
            found.complete = true; found.nodes = path->nodes;
            AUTOROUTE_CONTROL control( m_settings, net, retry, plane );
            for( std::size_t i = 1; i < found.nodes.size(); ++i )
            {
                const auto& a = found.nodes[i - 1]; const auto& b = found.nodes[i];
                found.cost += a.layer != b.layer ? control.ViaCost()
                        : control.TraceCost( std::hypot( static_cast<double>( b.point.x ) - a.point.x,
                                                        static_cast<double>( b.point.y ) - a.point.y ) )
                          + control.DirectionCost( b.layer, a.point, b.point )
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
            << " accepted=" << m_roomMetrics.routed << " elapsed_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - started ).count();
        autorouterDebugLog( log.str() );
    }
    return result;
}
} // namespace KICAD_AUTOROUTER
