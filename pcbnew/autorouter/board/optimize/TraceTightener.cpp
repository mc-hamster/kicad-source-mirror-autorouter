/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Native worker counterpart of Freerouting board/optimize/TraceTightener.java
 * at a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#include "TraceTightener.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>

#include "ViaOptimizer.h"
#include "../facade/RoutingBoard.h"
#include "../model/items/Pin.h"
#include "../../geometry/planar/IntBox.h"
#include "../../maze/MazeSearchEngine.h"
#include "../../maze/MazeTraceShover.h"


namespace KICAD_AUTOROUTER
{
namespace
{

std::int64_t saturatedAdd( std::int64_t aLeft, std::int64_t aRight )
{
    if( aRight > 0 && aLeft > std::numeric_limits<std::int64_t>::max() - aRight )
        return std::numeric_limits<std::int64_t>::max();
    if( aRight < 0 && aLeft < std::numeric_limits<std::int64_t>::min() - aRight )
        return std::numeric_limits<std::int64_t>::min();
    return aLeft + aRight;
}


const ROUTING_NET* findNet( const BOARD_SNAPSHOT& aBoard, int aNetCode )
{
    const auto net = std::find_if( aBoard.nets.begin(), aBoard.nets.end(),
                                   [&]( const ROUTING_NET& aCandidate )
                                   { return aCandidate.netCode == aNetCode; } );
    return net == aBoard.nets.end() ? nullptr : &*net;
}


std::int64_t resolvedTrackWidth( const BOARD_SNAPSHOT& aBoard,
                                 const ROUTING_CONNECTION& aConnection,
                                 std::size_t aEdge )
{
    const ROUTING_EDGE_STYLE& style = EdgeStyle( aConnection, aEdge );
    if( style.trackWidth > 0 )
        return style.trackWidth;

    if( const ROUTING_NET* net = findNet( aBoard, aConnection.netCode ) )
    {
        for( std::size_t pad : net->padIndices )
            if( pad < aBoard.pads.size() && aBoard.pads[pad].trackWidth > 0 )
                return aBoard.pads[pad].trackWidth;
    }

    return 150000;
}


std::int64_t resolvedPinClearance( const BOARD_SNAPSHOT& aBoard,
                                   const ROUTING_CONNECTION& aConnection,
                                   std::size_t aEdge, const ROUTING_PAD& aPin,
                                   int aLayer )
{
    std::int64_t result = std::max<std::int64_t>(
            0, EdgeStyle( aConnection, aEdge ).clearance );
    if( const ROUTING_NET* net = findNet( aBoard, aConnection.netCode ) )
        result = std::max( result, net->clearance );
    if( const auto* geometry = PIN::LayerGeometry( aPin, aLayer ) )
        result = std::max( result, geometry->clearance );
    return result;
}


std::int64_t pinEdgeToTurnDistance( const BOARD_SNAPSHOT& aBoard )
{
    std::int64_t result = std::numeric_limits<std::int64_t>::max();
    for( const ROUTING_PAD& pad : aBoard.pads )
        if( pad.netCode > 0 && pad.trackWidth > 0 )
            result = std::min( result, pad.trackWidth / 2 );
    return result == std::numeric_limits<std::int64_t>::max() ? 0 : result;
}


int pinConnectionViolationCount( const BOARD_SNAPSHOT& aBoard,
                                 const ROUTING_CONNECTION& aConnection )
{
    if( aConnection.nodes.size() < 2 || !HasValidEdgeStyles( aConnection ) )
        return 0;

    int result = 0;
    const std::int64_t edgeToTurn = pinEdgeToTurnDistance( aBoard );
    const auto check = [&]( std::size_t aPadIndex, bool aAtStart )
    {
        if( aPadIndex >= aBoard.pads.size() )
            return;
        const std::size_t edge = aAtStart ? 0 : aConnection.nodes.size() - 2;
        const ROUTER_NODE& pinNode = aAtStart ? aConnection.nodes.front()
                                              : aConnection.nodes.back();
        const ROUTING_PAD& pin = aBoard.pads[aPadIndex];
        const std::int64_t width = resolvedTrackWidth( aBoard, aConnection, edge );
        const std::int64_t clearance = resolvedPinClearance(
                aBoard, aConnection, edge, pin, pinNode.layer );
        if( !PIN::CheckConnectionToPin( aConnection, pin, aAtStart, width,
                                        clearance, edgeToTurn ) )
        {
            ++result;
        }
    };
    check( aConnection.fromPadIndex, true );
    check( aConnection.toPadIndex, false );
    return result;
}


std::optional<ROUTING_CONNECTION> correctPinConnections(
        const BOARD_SNAPSHOT& aBoard, const ROUTING_CONNECTION& aConnection )
{
    ROUTING_CONNECTION result = aConnection;
    bool changed = false;
    const std::int64_t edgeToTurn = pinEdgeToTurnDistance( aBoard );
    const auto correct = [&]( std::size_t aPadIndex, bool aAtStart )
    {
        if( aPadIndex >= aBoard.pads.size() || result.nodes.size() < 2 )
            return;
        const std::size_t edge = aAtStart ? 0 : result.nodes.size() - 2;
        const ROUTER_NODE& pinNode = aAtStart ? result.nodes.front()
                                              : result.nodes.back();
        const ROUTING_PAD& pin = aBoard.pads[aPadIndex];
        const std::int64_t width = resolvedTrackWidth( aBoard, result, edge );
        const std::int64_t clearance = resolvedPinClearance(
                aBoard, result, edge, pin, pinNode.layer );
        changed = PIN::CorrectConnectionToPin(
                          result, pin, aAtStart, width, clearance, edgeToTurn )
                  || changed;
    };

    correct( result.fromPadIndex, true );
    correct( result.toPadIndex, false );
    return changed ? std::optional<ROUTING_CONNECTION>( std::move( result ) )
                   : std::nullopt;
}


std::int64_t resolvedViaDiameter( const BOARD_SNAPSHOT& aBoard,
                                  const ROUTING_CONNECTION& aConnection,
                                  std::size_t aEdge )
{
    const ROUTING_EDGE_STYLE& style = EdgeStyle( aConnection, aEdge );
    if( style.viaDiameter > 0 )
        return style.viaDiameter;
    if( const ROUTING_NET* net = findNet( aBoard, aConnection.netCode ) )
        return std::max<std::int64_t>( 0, net->viaDiameter );
    return 0;
}


std::vector<int> resolvedViaLayers( const AUTOROUTER_SETTINGS& aSettings,
                                    const ROUTING_CONNECTION& aConnection,
                                    std::size_t aEdge )
{
    const ROUTING_EDGE_STYLE& style = EdgeStyle( aConnection, aEdge );
    if( !style.viaLayers.empty() )
        return style.viaLayers;

    std::vector<int> result;
    for( const ROUTER_LAYER_SETTINGS& layer : aSettings.layers )
        if( layer.enabled )
            result.push_back( layer.layerId );
    if( result.empty() )
    {
        result.push_back( aConnection.nodes[aEdge].layer );
        result.push_back( aConnection.nodes[aEdge + 1].layer );
    }
    return result;
}


int routeViaCount( const ROUTING_CONNECTION& aConnection )
{
    int result = 0;
    for( std::size_t edge = 1; edge < aConnection.nodes.size(); ++edge )
        if( aConnection.nodes[edge - 1].layer != aConnection.nodes[edge].layer )
            ++result;
    return result;
}


double routeLength( const ROUTING_CONNECTION& aConnection )
{
    double result = 0.0;
    for( std::size_t edge = 1; edge < aConnection.nodes.size(); ++edge )
    {
        const ROUTER_NODE& first = aConnection.nodes[edge - 1];
        const ROUTER_NODE& second = aConnection.nodes[edge];
        if( first.layer == second.layer )
        {
            const long double dx = static_cast<long double>( second.point.x ) - first.point.x;
            const long double dy = static_cast<long double>( second.point.y ) - first.point.y;
            result += std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
        }
    }
    return result;
}


bool betterGeometry( const ROUTING_CONNECTION& aCandidate,
                     const ROUTING_CONNECTION& aCurrent )
{
    const int candidateVias = routeViaCount( aCandidate );
    const int currentVias = routeViaCount( aCurrent );
    if( candidateVias != currentVias )
        return candidateVias < currentVias;
    return routeLength( aCandidate ) + 1.0 < routeLength( aCurrent );
}


bool preservesPadGroups( const std::vector<std::vector<std::size_t>>& aGroups,
                         const ROUTING_BOARD& aBoard )
{
    for( const auto& group : aGroups )
    {
        if( group.empty() )
            continue;
        for( std::size_t pad : group )
            if( !aBoard.Connected( group.front(), pad ) )
                return false;
    }
    return true;
}


int missingCount( const BOARD_SNAPSHOT& aBoard, const ROUTING_BOARD& aRoutingBoard )
{
    int result = 0;
    for( const ROUTING_NET& net : aBoard.nets )
        result += aRoutingBoard.CountMissing( net );
    return result;
}


bool insertable( const ROUTING_CONNECTION& aCandidate, const MAZE_SEARCH_ENGINE& aSearch )
{
    if( !aCandidate.complete || aCandidate.nodes.size() < 2
        || !HasValidEdgeStyles( aCandidate ) )
    {
        return false;
    }

    for( std::size_t edge = 1; edge < aCandidate.nodes.size(); ++edge )
    {
        if( !aSearch.CanInsertSegment( aCandidate.netCode,
                                      aCandidate.nodes[edge - 1],
                                      aCandidate.nodes[edge],
                                      &EdgeStyle( aCandidate, edge - 1 ) ) )
        {
            return false;
        }
    }
    return true;
}


ROUTER_BOX edgeBox( ROUTER_POINT aFirst, ROUTER_POINT aSecond, std::int64_t aRadius )
{
    return { saturatedAdd( std::min( aFirst.x, aSecond.x ), -aRadius ),
             saturatedAdd( std::min( aFirst.y, aSecond.y ), -aRadius ),
             saturatedAdd( std::max( aFirst.x, aSecond.x ), aRadius ),
             saturatedAdd( std::max( aFirst.y, aSecond.y ), aRadius ) };
}


bool overlapsRegion( const ROUTING_CONNECTION& aConnection,
                     const PLANAR::INT_OCTAGON& aRegion,
                     int aLayer,
                     const BOARD_SNAPSHOT& aBoard )
{
    for( std::size_t edge = 1; edge < aConnection.nodes.size(); ++edge )
    {
        const ROUTER_NODE& first = aConnection.nodes[edge - 1];
        const ROUTER_NODE& second = aConnection.nodes[edge];
        if( first.layer == second.layer )
        {
            if( first.layer != aLayer )
                continue;
            const std::int64_t radius = std::max<std::int64_t>(
                    0, resolvedTrackWidth( aBoard, aConnection, edge - 1 ) / 2 );
            if( aRegion.Intersects( PLANAR::INT_OCTAGON::FromBox(
                        edgeBox( first.point, second.point, radius ) ) ) )
            {
                return true;
            }
        }
        else
        {
            const ROUTING_EDGE_STYLE& style = EdgeStyle( aConnection, edge - 1 );
            std::vector<int> layers = style.viaLayers;
            if( layers.empty() )
                layers = { first.layer, second.layer };
            if( std::find( layers.begin(), layers.end(), aLayer ) == layers.end() )
                continue;
            const std::int64_t radius = std::max<std::int64_t>(
                    0, resolvedViaDiameter( aBoard, aConnection, edge - 1 ) / 2 );
            if( aRegion.Intersects( PLANAR::INT_OCTAGON::FromBox(
                        edgeBox( first.point, first.point, radius ) ) ) )
            {
                return true;
            }
        }
    }
    return false;
}


double changedAreaOffset( const BOARD_SNAPSHOT& aBoard,
                          const std::vector<ROUTING_CONNECTION>& aConnections,
                          int aLayer )
{
    std::int64_t maximumClearance = 0;
    std::int64_t maximumHalfWidth = 0;
    for( const ROUTING_NET& net : aBoard.nets )
    {
        maximumClearance = std::max( maximumClearance, net.clearance );
        for( std::size_t pad : net.padIndices )
            if( pad < aBoard.pads.size() )
                maximumHalfWidth = std::max( maximumHalfWidth,
                                              aBoard.pads[pad].trackWidth / 2 );
    }
    for( const ROUTING_CONNECTION& connection : aConnections )
        for( std::size_t edge = 1; edge < connection.nodes.size(); ++edge )
            if( connection.nodes[edge - 1].layer == aLayer
                && connection.nodes[edge].layer == aLayer )
            {
                maximumHalfWidth = std::max(
                        maximumHalfWidth,
                        resolvedTrackWidth( aBoard, connection, edge - 1 ) / 2 );
                maximumClearance = std::max(
                        maximumClearance,
                        EdgeStyle( connection, edge - 1 ).clearance );
            }

    return 1.5 * static_cast<double>( maximumClearance + 2 * maximumHalfWidth );
}

} // namespace


int TRACE_TIGHTENER::LayerCount( const BOARD_SNAPSHOT& aBoard,
                                 const AUTOROUTER_SETTINGS& aSettings )
{
    int maximum = -1;
    for( const ROUTER_LAYER_SETTINGS& layer : aSettings.layers )
        maximum = std::max( maximum, layer.layerId );
    for( const ROUTING_PAD& pad : aBoard.pads )
        for( int layer : pad.layers )
            maximum = std::max( maximum, layer );
    for( const ROUTING_OBSTACLE& area : aBoard.conductionAreas )
        for( int layer : area.layers )
            maximum = std::max( maximum, layer );
    return maximum + 1;
}


void TRACE_TIGHTENER::MarkConnection( CHANGED_AREA& aChangedArea,
                                      const ROUTING_CONNECTION& aConnection,
                                      const BOARD_SNAPSHOT& aBoard,
                                      const AUTOROUTER_SETTINGS& aSettings )
{
    for( std::size_t edge = 1; edge < aConnection.nodes.size(); ++edge )
    {
        const ROUTER_NODE& first = aConnection.nodes[edge - 1];
        const ROUTER_NODE& second = aConnection.nodes[edge];
        if( first.layer == second.layer )
        {
            const std::int64_t radius = std::max<std::int64_t>(
                    0, resolvedTrackWidth( aBoard, aConnection, edge - 1 ) / 2 );
            aChangedArea.Join( PLANAR::INT_OCTAGON::FromBox(
                                       edgeBox( first.point, second.point, radius ) ),
                               first.layer );
        }
        else
        {
            const std::int64_t radius = std::max<std::int64_t>(
                    0, resolvedViaDiameter( aBoard, aConnection, edge - 1 ) / 2 );
            const auto viaShape = PLANAR::INT_OCTAGON::FromBox(
                    edgeBox( first.point, first.point, radius ) );
            for( int layer : resolvedViaLayers( aSettings, aConnection, edge - 1 ) )
                aChangedArea.Join( viaShape, layer );
        }
    }
}


bool TRACE_TIGHTENER::OptChangedArea(
        CHANGED_AREA& aChangedArea,
        std::vector<ROUTING_CONNECTION>& aConnections,
        int aOnlyNetCode,
        const ROUTER_CANCEL_CALLBACK& aCancel,
        int aTimeLimitMilliseconds ) const
{
    if( !m_occupancy.Board() )
        return false;

    const auto started = std::chrono::steady_clock::now();
    const auto stopRequested = [&]()
    {
        if( aCancel && aCancel() )
            return true;
        return aTimeLimitMilliseconds > 0
               && std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - started ).count()
                          >= aTimeLimitMilliseconds;
    };

    bool changedAny = false;
    bool somethingChanged = true;
    int fixedPointPasses = 0;
    while( somethingChanged && !stopRequested() && fixedPointPasses++ < 256 )
    {
        somethingChanged = false;
        for( int layer = 0; layer < aChangedArea.LayerCount(); ++layer )
        {
            PLANAR::INT_OCTAGON changedRegion = aChangedArea.GetArea( layer );
            if( changedRegion.IsEmpty() )
                continue;

            aChangedArea.SetEmpty( layer );
            changedRegion = changedRegion.Enlarge(
                    changedAreaOffset( m_board, m_occupancy.Connections(), layer ) );

            const std::vector<ROUTING_CONNECTION> overlapping = m_occupancy.Connections();
            for( const ROUTING_CONNECTION& snapshotRoute : overlapping )
            {
                if( stopRequested() )
                    return changedAny;
                if( aOnlyNetCode > 0 && snapshotRoute.netCode != aOnlyNetCode )
                    continue;
                if( snapshotRoute.isExistingBoardRoute && !snapshotRoute.isAutorouterOwned )
                    continue;
                if( !overlapsRegion( snapshotRoute, changedRegion, layer, m_board ) )
                    continue;

                const auto current = std::find_if(
                        m_occupancy.Connections().begin(), m_occupancy.Connections().end(),
                        [&]( const ROUTING_CONNECTION& aRoute )
                        { return SameRouteGeometry( aRoute, snapshotRoute ); } );
                if( current == m_occupancy.Connections().end() )
                    continue;

                const ROUTING_CONNECTION original = *current;
                const auto groups = m_occupancy.Board()->ConnectedPadGroups(
                        original.netCode );
                const int missingBefore = missingCount( m_board, *m_occupancy.Board() );
                const auto movableViaEdges = VIA_OPTIMIZER::MovableViaEdges(
                        original, m_board, *m_occupancy.Board() );

                ROUTING_OCCUPANCY::TRANSACTION transaction( m_occupancy );
                m_occupancy.Remove( original );
                MAZE_SEARCH_ENGINE search( m_board, m_settings, m_occupancy );

                std::vector<ROUTING_CONNECTION> candidates;
                const int originalPinViolations =
                        pinConnectionViolationCount( m_board, original );
                ROUTING_CONNECTION pulled = original;
                if( MAZE_TRACE_SHOVER::Shorten( pulled, search )
                    && betterGeometry( pulled, original ) )
                {
                    candidates.push_back( std::move( pulled ) );
                }

                if( auto corrected = correctPinConnections( m_board, original );
                    corrected
                    && pinConnectionViolationCount( m_board, *corrected )
                               < originalPinViolations )
                {
                    candidates.push_back( std::move( *corrected ) );
                }

                for( ROUTING_CONNECTION viaCandidate : VIA_OPTIMIZER::Candidates(
                             original, m_board, m_settings, *m_occupancy.Board(), search,
                             movableViaEdges, aCancel ) )
                {
                    MAZE_TRACE_SHOVER::Shorten( viaCandidate, search );
                    if( betterGeometry( viaCandidate, original ) )
                        candidates.push_back( std::move( viaCandidate ) );
                }

                const auto best = std::min_element(
                        candidates.begin(), candidates.end(),
                        [&]( const ROUTING_CONNECTION& aLeft,
                             const ROUTING_CONNECTION& aRight )
                        {
                            const int leftPinViolations =
                                    pinConnectionViolationCount( m_board, aLeft );
                            const int rightPinViolations =
                                    pinConnectionViolationCount( m_board, aRight );
                            if( leftPinViolations != rightPinViolations )
                                return leftPinViolations < rightPinViolations;
                            const int leftVias = routeViaCount( aLeft );
                            const int rightVias = routeViaCount( aRight );
                            return leftVias != rightVias ? leftVias < rightVias
                                                        : routeLength( aLeft )
                                                                  < routeLength( aRight );
                        } );
                if( best == candidates.end()
                    || ( pinConnectionViolationCount( m_board, *best )
                                 >= originalPinViolations
                         && !betterGeometry( *best, original ) )
                    || !insertable( *best, search ) )
                    continue;

                m_occupancy.Add( *best );
                if( missingCount( m_board, *m_occupancy.Board() ) > missingBefore
                    || !preservesPadGroups( groups, *m_occupancy.Board() ) )
                {
                    continue;
                }

                MarkConnection( aChangedArea, original, m_board, m_settings );
                MarkConnection( aChangedArea, *best, m_board, m_settings );
                transaction.Commit();
                aConnections = m_occupancy.Connections();
                somethingChanged = true;
                changedAny = true;
            }
        }
    }

    aConnections = m_occupancy.Connections();
    return changedAny;
}

} // namespace KICAD_AUTOROUTER
