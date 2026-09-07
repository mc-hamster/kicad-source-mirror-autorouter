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

#include "BatchAutorouter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <iterator>
#include <map>
#include <numeric>
#include <set>

#include "../drc/DesignRulesChecker.h"
#include "../path/FoundConnectionInserter.h"
#include "AutorouteAirlineCalculator.h"
#include "AutorouteConnectionRouter.h"
#include "AutorouteBatchLoop.h"
#include "BatchOptimizerMultiThreaded.h"
#include "AutorouteUnroutedReport.h"
#include "BatchFanout.h"
#include "../BoardHistory.h"


namespace KICAD_AUTOROUTER
{

namespace
{

double distance( const ROUTER_POINT& a, const ROUTER_POINT& b )
{
    const long double dx = static_cast<long double>( a.x ) - b.x;
    const long double dy = static_cast<long double>( a.y ) - b.y;
    return std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
}


std::int64_t viaValue( const ROUTING_NET& aNet, std::int64_t aFallback )
{
    return aNet.viaDiameter > 0 ? aNet.viaDiameter : aFallback;
}


int layerOrdinal( const AUTOROUTER_SETTINGS& aSettings, int aLayer )
{
    auto it = std::find_if( aSettings.layers.begin(), aSettings.layers.end(),
                            [aLayer]( const ROUTER_LAYER_SETTINGS& aSetting )
                            {
                                return aSetting.layerId == aLayer;
                            } );

    if( it == aSettings.layers.end() )
        return aLayer;

    return it->layerOrdinal >= 0
                   ? it->layerOrdinal
                   : static_cast<int>( std::distance( aSettings.layers.begin(), it ) );
}


std::vector<int> viaLayers( const AUTOROUTER_SETTINGS& aSettings, int aFirstLayer,
                            int aSecondLayer )
{
    const int firstOrdinal = layerOrdinal( aSettings, aFirstLayer );
    const int secondOrdinal = layerOrdinal( aSettings, aSecondLayer );
    const int low = std::min( firstOrdinal, secondOrdinal );
    const int high = std::max( firstOrdinal, secondOrdinal );

    std::vector<int> result;
    for( const ROUTER_LAYER_SETTINGS& layer : aSettings.layers )
    {
        if( !layer.enabled )
            continue;

        const int ordinal = layerOrdinal( aSettings, layer.layerId );
        if( ordinal >= low && ordinal <= high )
            result.push_back( layer.layerId );
    }

    if( result.empty() )
    {
        result.push_back( aFirstLayer );
        if( aSecondLayer != aFirstLayer )
            result.push_back( aSecondLayer );
    }

    std::stable_sort( result.begin(), result.end(),
                      [&]( int aLeft, int aRight )
                      {
                          return layerOrdinal( aSettings, aLeft )
                                 < layerOrdinal( aSettings, aRight );
                      } );
    result.erase( std::unique( result.begin(), result.end() ), result.end() );
    return result;
}

} // namespace


std::vector<BATCH_AUTOROUTER::NET_ORDER_ENTRY>
BATCH_AUTOROUTER::orderNets( const BOARD_SNAPSHOT& aBoard ) const
{
    std::vector<NET_ORDER_ENTRY> result;

    for( const ROUTING_NET& net : aBoard.nets )
    {
        if( !net.connections.empty() )
            result.push_back( { &net, netHalfPerimeter( aBoard, net ) } );
    }

    // Stable ordering is important for parity investigations and makes routing
    // results repeatable even when two nets have identical geometry.
    std::stable_sort( result.begin(), result.end(),
                      []( const NET_ORDER_ENTRY& aLeft, const NET_ORDER_ENTRY& aRight )
                      {
                          if( aLeft.net->netClassPriority != aRight.net->netClassPriority )
                          {
                              return aLeft.net->netClassPriority
                                     > aRight.net->netClassPriority;
                          }

                          if( aLeft.net->padIndices.size() != aRight.net->padIndices.size() )
                              return aLeft.net->padIndices.size() > aRight.net->padIndices.size();

                          if( aLeft.halfPerimeter != aRight.halfPerimeter )
                              return aLeft.halfPerimeter > aRight.halfPerimeter;

                          return aLeft.net->netCode < aRight.net->netCode;
                      } );

    return result;
}


const ROUTING_PAD* BATCH_AUTOROUTER::firstPad( const BOARD_SNAPSHOT& aBoard,
                                               const ROUTING_NET& aNet ) const
{
    if( aNet.padIndices.empty() )
        return nullptr;

    return &aBoard.pads[aNet.padIndices.front()];
}


double BATCH_AUTOROUTER::netHalfPerimeter( const BOARD_SNAPSHOT& aBoard,
                                           const ROUTING_NET& aNet ) const
{
    const ROUTING_PAD* first = firstPad( aBoard, aNet );

    if( !first )
        return 0.0;

    std::int64_t minX = first->position.x;
    std::int64_t maxX = first->position.x;
    std::int64_t minY = first->position.y;
    std::int64_t maxY = first->position.y;

    for( std::size_t index : aNet.padIndices )
    {
        const ROUTER_POINT& point = aBoard.pads[index].position;
        minX = std::min( minX, point.x );
        maxX = std::max( maxX, point.x );
        minY = std::min( minY, point.y );
        maxY = std::max( maxY, point.y );
    }

    return static_cast<double>( maxX - minX + maxY - minY );
}


bool BATCH_AUTOROUTER::isAlreadyRouted( const std::vector<ROUTING_CONNECTION>& aConnections,
                                        int aNetCode, std::size_t aExpectedConnections ) const
{
    const std::size_t routed = static_cast<std::size_t>( std::count_if(
            aConnections.begin(), aConnections.end(),
            [aNetCode]( const ROUTING_CONNECTION& aConnection )
            {
                return aConnection.netCode == aNetCode && aConnection.complete;
            } ) );

    return routed >= aExpectedConnections;
}


bool BATCH_AUTOROUTER::routeNet( const BOARD_SNAPSHOT& aBoard,
                                 const AUTOROUTER_SETTINGS& aSettings, const ROUTING_NET& aNet,
                                 int aRetry, ROUTING_OCCUPANCY& aOccupancy,
                                 const AUTOROUTE_ENGINE& aEngine,
                                 std::vector<ROUTING_CONNECTION>& aConnections,
                                 int& aExpandedNodes, const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    if( aNet.padIndices.empty() )
        return true;

    std::vector<ROUTING_CONNECTION> newConnections;
    if( aNet.connections.empty() )
        return true;

    // A failed pass may have left a partially routed version of this net in
    // the shared occupancy map.  Re-routing on top of those connections
    // would duplicate geometry and, more subtly, make the connected-set
    // orientation depend on the previous pass.  Freerouting retries an item
    // from a clean item state, so discard the old complete connections for
    // this net before beginning a fresh attempt.
    for( std::size_t index = aConnections.size(); index > 0; --index )
    {
        if( aConnections[index - 1].netCode != aNet.netCode )
            continue;

        aOccupancy.Remove( aConnections[index - 1] );
        aConnections.erase( aConnections.begin()
                            + static_cast<std::ptrdiff_t>( index - 1 ) );
    }

    std::map<std::size_t, std::size_t> parent;
    std::set<std::size_t> activePads;
    bool allConnectionsRouted = true;

    auto findRoot = [&]( std::size_t aIndex )
    {
        std::size_t root = aIndex;
        while( parent[root] != root )
            root = parent[root];

        while( parent[aIndex] != aIndex )
        {
            const std::size_t next = parent[aIndex];
            parent[aIndex] = root;
            aIndex = next;
        }

        return root;
    };

    auto unite = [&]( std::size_t aLeft, std::size_t aRight )
    {
        const std::size_t leftRoot = findRoot( aLeft );
        const std::size_t rightRoot = findRoot( aRight );
        if( leftRoot != rightRoot )
            parent[rightRoot] = leftRoot;
    };

    for( const auto& [sourceIndex, targetIndex] : aNet.connections )
    {
        if( aCancel && aCancel() )
            return false;

        if( sourceIndex >= aBoard.pads.size() || targetIndex >= aBoard.pads.size() )
            return false;

        std::size_t routeSourceIndex = sourceIndex;
        std::size_t routeTargetIndex = targetIndex;

        const bool targetIsPlane = aBoard.pads[targetIndex].isPlaneTarget;

        // Freerouting's connection router starts at the already-connected
        // component and expands towards the selected target.  Ratsnest edges
        // are not guaranteed to be listed in that order, so orient each edge
        // against the component built by earlier successful routes.
        if( !targetIsPlane )
        {
            parent.try_emplace( sourceIndex, sourceIndex );
            parent.try_emplace( targetIndex, targetIndex );

            const bool sourceConnected = activePads.contains( sourceIndex );
            const bool targetConnected = activePads.contains( targetIndex );

            if( !sourceConnected && targetConnected )
                std::swap( routeSourceIndex, routeTargetIndex );
            else if( sourceConnected && targetConnected
                     && findRoot( sourceIndex ) == findRoot( targetIndex ) )
            {
                ROUTING_CONNECTION alreadyConnected;
                alreadyConnected.netCode = aNet.netCode;
                alreadyConnected.complete = true;
                alreadyConnected.fromPadIndex = sourceIndex;
                alreadyConnected.toPadIndex = targetIndex;
                alreadyConnected.nodes.push_back(
                        { aBoard.pads[sourceIndex].position,
                          aBoard.pads[sourceIndex].layers.empty()
                                  ? -1
                                  : aBoard.pads[sourceIndex].layers.front() } );
                newConnections.push_back( std::move( alreadyConnected ) );
                continue;
            }
        }

        ROUTING_PAD source = aBoard.pads[routeSourceIndex];
        if( source.isFanoutTarget && source.fanoutTargetLayer >= 0 )
            source.layers = { source.fanoutTargetLayer };
        std::optional<ROUTING_CONNECTION> connection;
        ROUTING_PAD              target;

        // A filled plane is represented by several deterministic interior
        // targets.  The nearest target is normally best, but an existing
        // foreign copper corridor can make that particular interior point
        // inaccessible even though another island/target on the same plane
        // is reachable.  Try the remaining plane targets before declaring
        // the pad unroutable; Freerouting treats a conduction area as a
        // connected destination region rather than a single coordinate.
        std::vector<std::size_t> targetCandidates{ routeTargetIndex };
        if( targetIsPlane )
        {
            for( std::size_t candidate : aNet.planeTargetIndices )
            {
                if( candidate < aBoard.pads.size()
                    && std::find( targetCandidates.begin(), targetCandidates.end(), candidate )
                               == targetCandidates.end() )
                {
                    targetCandidates.push_back( candidate );
                }
            }
        }

        for( std::size_t candidate : targetCandidates )
        {
            target = aBoard.pads[candidate];
            connection.reset();

            for( int iteration = 0;
                 iteration < std::max( 1, aSettings.maxIterations ); ++iteration )
            {
                if( aCancel && aCancel() )
                    return false;

                int expanded = 0;
                connection = aEngine.AutorouteConnection( source, target, aRetry + iteration,
                                                           expanded, aCancel );
                aExpandedNodes += expanded;

                if( connection )
                {
                    routeTargetIndex = candidate;
                    break;
                }
            }

            if( connection )
                break;
        }

        if( !connection )
        {
            // A multi-pad/plane net is a set of independent connection items.
            // One blocked pad must not prevent the remaining items from being
            // attempted in the same pass: Freerouting carries successful
            // connections forward while retrying the failed item.  Returning
            // here made the first difficult fanout/plane stub discard the
            // opportunity to route every later pad on a large power net.
            allConnectionsRouted = false;
            continue;
        }

        connection->fromPadIndex = routeSourceIndex;
        connection->toPadIndex = routeTargetIndex;
        connection->isPlaneConnection = target.isPlaneTarget;
        connection->isFanoutConnection = target.isFanoutTarget && !source.isFanoutTarget;
        aOccupancy.Add( *connection );
        newConnections.push_back( std::move( *connection ) );

        if( !targetIsPlane )
        {
            activePads.insert( routeSourceIndex );
            activePads.insert( routeTargetIndex );
            unite( routeSourceIndex, routeTargetIndex );
        }
    }

    aConnections.insert( aConnections.end(),
                         std::make_move_iterator( newConnections.begin() ),
                         std::make_move_iterator( newConnections.end() ) );
    return allConnectionsRouted;
}


void BATCH_AUTOROUTER::buildGeometry( const BOARD_SNAPSHOT& aBoard,
                                       const AUTOROUTER_SETTINGS& aSettings,
                                       const std::vector<ROUTING_CONNECTION>& aConnections,
                                       ROUTING_RESULT& aResult ) const
{
    std::map<int, std::int64_t> netWidths;
    std::map<int, std::int64_t> netViaDiameters;
    std::map<int, std::int64_t> netViaDrills;

    for( const ROUTING_NET& net : aBoard.nets )
    {
        std::int64_t width = 0;

        for( std::size_t index : net.padIndices )
            width = std::max( width, aBoard.pads[index].trackWidth );

        netWidths[net.netCode] = width > 0 ? width : 150000;
        netViaDiameters[net.netCode] = viaValue( net, 600000 );
        netViaDrills[net.netCode] = net.viaDrill > 0 ? net.viaDrill : 300000;
    }

    for( const ROUTING_CONNECTION& connection : aConnections )
    {
        if( !connection.complete )
            continue;

        aResult.connections.push_back( connection );
        if( connection.isFanoutConnection )
            ++aResult.metrics.fanoutConnections;

        // The path layer owns route-to-geometry materialization.  KiCad board
        // objects are still not created here; the editor adapter does that
        // only after the proposal is accepted.
        for( std::size_t i = 1; i < connection.nodes.size(); ++i )
        {
            const ROUTER_NODE& previous = connection.nodes[i - 1];
            const ROUTER_NODE& current = connection.nodes[i];

            if( previous.layer == current.layer )
            {
                FOUND_CONNECTION_INSERTER::AppendEdge(
                        connection.netCode, previous, current, netWidths[connection.netCode],
                        netViaDiameters[connection.netCode], netViaDrills[connection.netCode], {},
                        aResult );
            }
            else
            {
                FOUND_CONNECTION_INSERTER::AppendEdge(
                        connection.netCode, previous, current, netWidths[connection.netCode],
                        netViaDiameters[connection.netCode], netViaDrills[connection.netCode],
                        viaLayers( aSettings, previous.layer, current.layer ), aResult );
            }
        }
    }

    // A plane fanout is represented by two logical connections that meet at
    // the same escaped landing point.  Each connection can describe the two
    // sides of that one physical via, so collapse exact same-net via records
    // before the proposal is handed to KiCad.  Keeping this normalization here
    // preserves the useful per-connection topology for rip-up while ensuring
    // the accepted board contains one ordinary via, not a co-located pair.
    std::vector<ROUTING_VIA> uniqueVias;
    uniqueVias.reserve( aResult.vias.size() );
    for( ROUTING_VIA& via : aResult.vias )
    {
        const bool duplicate = std::any_of(
                uniqueVias.begin(), uniqueVias.end(),
                [&]( const ROUTING_VIA& existing )
                {
                    return existing.netCode == via.netCode && existing.position == via.position
                           && existing.topLayer == via.topLayer
                           && existing.bottomLayer == via.bottomLayer
                           && existing.diameter == via.diameter && existing.drill == via.drill
                           && existing.layers == via.layers;
                } );
        if( !duplicate )
            uniqueVias.push_back( std::move( via ) );
    }
    aResult.vias = std::move( uniqueVias );

    std::set<std::string> removed;
    std::map<int, std::size_t> expectedConnections;
    std::map<int, std::size_t> routedConnections;

    for( const ROUTING_NET& net : aBoard.nets )
    {
        if( !net.connections.empty() )
            expectedConnections[net.netCode] = net.connections.size();
    }

    for( const ROUTING_CONNECTION& connection : aConnections )
    {
        if( connection.complete )
            ++routedConnections[connection.netCode];
    }

    if( aSettings.allowRipupExisting )
    {
        auto collectRemovable = [&]( const ROUTING_OBSTACLE& obstacle )
        {
            const auto expectedIt = expectedConnections.find( obstacle.netCode );
            const auto routedIt = routedConnections.find( obstacle.netCode );
            const bool netComplete = expectedIt != expectedConnections.end()
                                     && routedIt != routedConnections.end()
                                     && routedIt->second >= expectedIt->second;

            if( obstacle.isExistingRoute && netComplete && !obstacle.boardItemId.empty() )
            {
                removed.insert( obstacle.boardItemId );
            }
        };

        for( const ROUTING_OBSTACLE& obstacle : aBoard.obstacles )
            collectRemovable( obstacle );

        for( const ROUTING_OBSTACLE& obstacle : aBoard.removableExistingRoutes )
            collectRemovable( obstacle );
    }

    aResult.removedBoardItemIds.assign( removed.begin(), removed.end() );
    aResult.metrics.segmentCount = static_cast<int>( aResult.segments.size() );
    aResult.metrics.viaCount = static_cast<int>( aResult.vias.size() );

    for( const ROUTING_SEGMENT& segment : aResult.segments )
        aResult.metrics.routedLengthIU += distance( segment.start, segment.end );

    aResult.metrics.airlineLengthIU = AUTOROUTE_AIRLINE_CALCULATOR::TotalLength(
            aBoard, aResult.connections );
}


ROUTING_RESULT BATCH_AUTOROUTER::Run( const BOARD_SNAPSHOT& aBoard,
                                      const AUTOROUTER_SETTINGS& aSettings,
                                      const ROUTER_CANCEL_CALLBACK& aCancel,
                                      const ROUTER_PROGRESS_CALLBACK& aProgress ) const
{
    const auto startTime = std::chrono::steady_clock::now();
    // BatchFanout only changes the immutable worker snapshot.  The caller's
    // KiCad board remains untouched until the proposal is accepted.
    const BOARD_SNAPSHOT routedBoard = BATCH_FANOUT::PrepareSnapshot( aBoard, aSettings );
    const BOARD_SNAPSHOT& board = routedBoard;
    ROUTING_RESULT result;
    const std::vector<NET_ORDER_ENTRY> orderedNets = orderNets( board );
    const int totalConnections = std::accumulate(
            orderedNets.begin(), orderedNets.end(), 0,
            []( int aTotal, const NET_ORDER_ENTRY& aEntry )
            {
                return aTotal + static_cast<int>( aEntry.net->connections.size() );
            } );

    result.metrics.totalConnections = totalConnections;
    ROUTING_OCCUPANCY occupancy( aSettings.gridStepIU );
    // The search engine builds the immutable obstacle/spatial index once per
    // engine.  Reuse it for every connection in a pass; reconstructing it for
    // each net makes large boards spend most of their runtime re-indexing the
    // same pads and keepouts rather than expanding paths.
    AUTOROUTE_ENGINE routeEngine( board, aSettings, occupancy );
    MAZE_RIPUP_RESOLVER ripupResolver;
    std::vector<ROUTING_CONNECTION> connections;
    std::set<int> failedNets;
    int totalExpandedNodes = 0;
    int retries = 0;
    int ripups = 0;
    AUTOROUTE_BATCH_LOOP batchLoop;
    BOARD_HISTORY history;
    bool complete = orderedNets.empty();
    auto elapsedMilliseconds = [&]()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - startTime )
                .count();
    };

    auto routedConnectionCount = [&]()
    {
        return static_cast<int>( std::count_if(
                connections.begin(), connections.end(),
                []( const ROUTING_CONNECTION& aConnection )
                {
                    return aConnection.complete;
                } ) );
    };

    auto checkpoint = [&]()
    {
        ROUTING_RESULT candidate;
        candidate.metrics.totalConnections = totalConnections;
        candidate.metrics.routedConnections = routedConnectionCount();
        candidate.metrics.unroutedConnections = std::max(
                0, totalConnections - candidate.metrics.routedConnections );
        candidate.metrics.passes = result.metrics.passes;
        candidate.metrics.retries = retries;
        candidate.metrics.ripups = ripups;
        candidate.metrics.optimizationPasses = result.metrics.optimizationPasses;
        candidate.metrics.expandedNodes = totalExpandedNodes;
        buildGeometry( board, aSettings, connections, candidate );
        candidate.metrics.drcViolations =
                DESIGN_RULES_CHECKER::CountViolations( board, aSettings, candidate );
        candidate.complete = candidate.metrics.unroutedConnections == 0
                             && candidate.metrics.drcViolations == 0;
        history.Add( candidate );
    };

    auto restoreBestCheckpoint = [&]()
    {
        const std::optional<ROUTING_RESULT> best = history.Best();
        if( !best )
            return;

        connections = best->connections;
        occupancy.Clear();
        for( const ROUTING_CONNECTION& connection : connections )
        {
            if( connection.complete )
                occupancy.Add( connection );
        }
        result.metrics.routedConnections = routedConnectionCount();
    };

    for( int pass = 0; pass < std::max( 1, aSettings.maxPasses ); ++pass )
    {
        if( aCancel && aCancel() )
        {
            result.cancelled = true;
            result.message = "Autorouter cancelled";
            break;
        }

        ROUTER_PROGRESS progress;
        progress.pass = pass + 1;
        progress.maxPasses = std::max( 1, aSettings.maxPasses );
        progress.totalConnections = totalConnections;
        progress.routedConnections = result.metrics.routedConnections;
        progress.ripups = ripups;
        progress.retries = retries;
        progress.elapsedMilliseconds = elapsedMilliseconds();
        progress.stage = "Routing pass";

        if( aProgress )
            aProgress( progress );

        complete = true;

        for( const NET_ORDER_ENTRY& entry : orderedNets )
        {
            if( aCancel && aCancel() )
            {
                result.cancelled = true;
                result.message = "Autorouter cancelled";
                break;
            }

            const bool completeNet = isAlreadyRouted( connections, entry.net->netCode,
                                                      entry.net->connections.size() );
            const bool rerouteCompleteNet = pass > 0 && !aSettings.stopAfterFirstComplete
                                            && aSettings.allowRipupRouted;

            // Once the first pass has completed, Freerouting continues with
            // negotiated-congestion/optimization passes unless the user
            // explicitly requested an early stop.  Re-routing the complete
            // net is safe because checkpoints below retain the best board
            // state if a later attempt is worse.
            if( completeNet && !rerouteCompleteNet )
                continue;

            int expanded = 0;
            const bool routed = routeNet( board, aSettings, *entry.net, pass, occupancy,
                                          routeEngine,
                                          connections, expanded, aCancel );
            totalExpandedNodes += expanded;

            if( aCancel && aCancel() )
            {
                result.cancelled = true;
                result.message = "Autorouter cancelled";
                break;
            }

            if( !routed )
            {
                complete = false;
                failedNets.insert( entry.net->netCode );

                if( aSettings.allowRipupRouted && ripups < std::max( 0, aSettings.maxRipups ) )
                {
                    std::optional<std::size_t> victim =
                            ripupResolver.SelectVictim( connections, board.nets,
                                                       entry.net->netCode,
                                                       std::max( 1, aSettings.startRipupCost )
                                                               * ( pass + 1 ) );

                    if( victim )
                    {
                        const int victimNet = connections[*victim].netCode;

                        for( std::size_t i = connections.size(); i > 0; --i )
                        {
                            if( connections[i - 1].netCode != victimNet )
                                continue;

                            occupancy.Remove( connections[i - 1] );
                            connections.erase( connections.begin()
                                                       + static_cast<std::ptrdiff_t>( i - 1 ) );
                        }

                        failedNets.insert( victimNet );
                        ++ripups;
                    }
                }

                ++retries;
            }

            result.metrics.routedConnections = 0;
            for( const ROUTING_CONNECTION& connection : connections )
            {
                if( connection.complete )
                    ++result.metrics.routedConnections;
            }

            if( aProgress )
            {
                progress.routedConnections = result.metrics.routedConnections;
                progress.ripups = ripups;
                progress.retries = retries;
                progress.expandedNodes = totalExpandedNodes;
                progress.elapsedMilliseconds = elapsedMilliseconds();
                progress.stage = routed ? "Routing connections" : "Negotiating congestion";
                aProgress( progress );
            }
        }

        result.metrics.passes = pass + 1;

        if( result.cancelled )
            break;

        result.metrics.routedConnections = routedConnectionCount();
        checkpoint();

        if( complete && aSettings.stopAfterFirstComplete )
            break;

        if( batchLoop.Observe( result.metrics.routedConnections, ripups ) )
            break;

        if( !complete && failedNets.empty() )
            break;

        failedNets.clear();
    }

    result.metrics.passes = std::max( 1, result.metrics.passes );
    result.metrics.retries = retries;
    result.metrics.ripups = ripups;
    result.metrics.expandedNodes = totalExpandedNodes;

    if( !result.cancelled && aSettings.optimizeAfterComplete
        && !aSettings.stopAfterFirstComplete && !connections.empty() )
    {
        ROUTER_PROGRESS progress;
        progress.pass = result.metrics.passes;
        progress.maxPasses = std::max( 1, aSettings.maxPasses );
        progress.totalConnections = totalConnections;
        progress.routedConnections = result.metrics.routedConnections;
        progress.ripups = ripups;
        progress.retries = retries;
        progress.elapsedMilliseconds = elapsedMilliseconds();
        progress.stage = "Optimizing routes";

        if( aProgress )
            aProgress( progress );

        BATCH_OPTIMIZER_MULTI_THREADED optimizer( board, aSettings, occupancy );
        result.metrics.optimizationPasses = optimizer.Optimize( connections, aCancel );
        result.metrics.routedConnections = routedConnectionCount();

        if( aCancel && aCancel() )
        {
            result.cancelled = true;
            result.message = "Autorouter cancelled";
        }
    }

    if( !result.cancelled )
    {
        // Keep the strongest checkpoint, not merely the last negotiated pass.
        // This mirrors Freerouting's bounded BoardHistory and prevents a
        // failed late optimization from degrading an otherwise complete route.
        checkpoint();
        restoreBestCheckpoint();
        buildGeometry( board, aSettings, connections, result );
        result.metrics.routedConnections = routedConnectionCount();
        result.metrics.drcViolations =
                DESIGN_RULES_CHECKER::CountViolations( board, aSettings, result );
        result.metrics.unroutedConnections =
                std::max( 0, result.metrics.totalConnections - result.metrics.routedConnections );
        result.metrics.completionPercent = result.metrics.totalConnections > 0
                                                   ? 100.0 * result.metrics.routedConnections
                                                             / result.metrics.totalConnections
                                                   : 100.0;
        result.complete = result.metrics.unroutedConnections == 0
                          && result.metrics.drcViolations == 0;

        if( result.metrics.unroutedConnections != 0 )
        {
            result.message = "Autorouting finished with unrouted connections";
        }
        else if( result.metrics.drcViolations != 0 )
        {
            result.message = "Autorouting produced design-rule violations";
        }
        else
        {
            result.message = "Autorouting complete";
        }

        result.unroutedNetCodes = AUTOROUTE_UNROUTED_REPORT::Build( board.nets, connections );
    }

    const auto endTime = std::chrono::steady_clock::now();
    result.metrics.elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                  endTime - startTime )
                                                  .count();
    return result;
}

} // namespace KICAD_AUTOROUTER
