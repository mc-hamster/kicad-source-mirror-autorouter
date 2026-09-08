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
#include "../rules/ViaRule.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <iterator>
#include <map>
#include <numeric>
#include <set>
#include <sstream>

#include "../AutorouterDebug.h"
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


bool BATCH_AUTOROUTER::routeNet( const BOARD_SNAPSHOT& aBoard,
                                 const AUTOROUTER_SETTINGS& aSettings, const ROUTING_NET& aNet,
                                 int aRetry, ROUTING_OCCUPANCY& aOccupancy,
                                 const AUTOROUTE_ENGINE& aEngine,
                                 std::vector<ROUTING_CONNECTION>& aConnections,
                                 int& aExpandedNodes, int& aRipups, const ROUTER_CANCEL_CALLBACK& aCancel,
                                 const ROUTER_SEARCH_PROGRESS_CALLBACK& aSearchProgress ) const
{
    const auto netStarted = std::chrono::steady_clock::now();

    if( autorouterDebugEnabled() )
    {
        std::ostringstream message;
        message << "BEGIN net code=" << aNet.netCode << " pads=" << aNet.padIndices.size()
                << " connections=" << aNet.connections.size() << " retry=" << aRetry;
        autorouterDebugLog( message.str() );
    }

    if( aNet.padIndices.empty() )
        return true;

    std::vector<ROUTING_CONNECTION> newConnections;
    if( aNet.connections.empty() )
        return true;

    std::map<std::size_t, std::size_t> parent;
    for( std::size_t index : aNet.padIndices )
        parent.emplace( index, index );
    std::set<std::size_t> activePads;

    // Keep the successful part of a multi-pad net across negotiated-congestion
    // passes.  Freerouting retries the unresolved connection items while the
    // connected set and its already legal copper remain in place.  Throwing
    // away the whole net here made a large net (notably the Arduino board's
    // ground net) oscillate: every retry paid to rediscover hundreds of good
    // paths before it could make progress on the one blocked edge.

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

    // The disjoint-set is only a routing-order view of actual copper contacts.
    // Rebuild after every insertion/rip-up; endpoint labels are not connectivity.
    auto refreshContacts = [&]()
    {
        parent.clear();
        activePads.clear();
        for( std::size_t index : aNet.padIndices )
            parent[index] = index;
        for( const auto& [from, to] : aNet.connections )
        {
            parent.try_emplace( from, from );
            parent.try_emplace( to, to );
        }
        for( const auto& group : aOccupancy.Board()->ConnectedPadGroups( aNet.netCode ) )
        {
            for( std::size_t index : group )
            {
                parent.try_emplace( index, index );
                unite( group.front(), index );
                activePads.insert( index );
            }
        }
    };
    refreshContacts();

    // A ratsnest is an electrical graph, not a routing order.  On a large
    // multi-pad net (the Arduino board's ground net is a good example), the
    // first edge in that graph can be a long, highly-congested diagonal.  A
    // batch router should grow a connected tree from short legal edges first;
    // otherwise every retry can spend the full A* expansion budget on the
    // same pathological edge before any useful copper is committed.  Keep
    // the net ordering unchanged, but choose the next edge from this net
    // using the same connected-component information maintained below.
    std::vector<std::pair<std::size_t, std::size_t>> pendingConnections = aNet.connections;

    auto connectionDistance = [&]( const std::pair<std::size_t, std::size_t>& aConnection )
    {
        if( aConnection.first >= aBoard.pads.size() || aConnection.second >= aBoard.pads.size() )
            return std::numeric_limits<long double>::max();

        const ROUTER_POINT& first = aBoard.pads[aConnection.first].position;
        const ROUTER_POINT& second = aBoard.pads[aConnection.second].position;
        return std::abs( static_cast<long double>( first.x ) - second.x )
               + std::abs( static_cast<long double>( first.y ) - second.y );
    };

    while( !pendingConnections.empty() )
    {
        if( aCancel && aCancel() )
            return false;

        std::size_t selected = 0;
        int          selectedClass = std::numeric_limits<int>::max();
        long double  selectedDistance = std::numeric_limits<long double>::max();

        for( std::size_t index = 0; index < pendingConnections.size(); ++index )
        {
            const auto& [candidateSource, candidateTarget] = pendingConnections[index];
            if( candidateSource >= aBoard.pads.size() || candidateTarget >= aBoard.pads.size() )
                continue;

            const bool candidatePlane = aBoard.pads[candidateTarget].isPlaneTarget;
            const bool sourceConnected = activePads.contains( candidateSource );
            const bool targetConnected = activePads.contains( candidateTarget );

            // Class 0 grows the existing component (including a fanout stub
            // into a plane), class 1 starts the next shortest component, and
            // class 2 is a fallback for redundant/already-connected edges.
            const int candidateClass = candidatePlane
                                               ? sourceConnected ? 0 : 1
                                               : sourceConnected != targetConnected ? 0
                                                                                     : ( !sourceConnected
                                                                                                 && !targetConnected
                                                                                         ? 1
                                                                                         : 2 );
            const long double candidateDistance = connectionDistance(
                    pendingConnections[index] );

            if( candidateClass < selectedClass
                || ( candidateClass == selectedClass
                     && candidateDistance < selectedDistance ) )
            {
                selected = index;
                selectedClass = candidateClass;
                selectedDistance = candidateDistance;
            }
        }

        const auto [sourceIndex, targetIndex] = pendingConnections[selected];
        pendingConnections.erase( pendingConnections.begin()
                                  + static_cast<std::ptrdiff_t>( selected ) );

        if( sourceIndex >= aBoard.pads.size() || targetIndex >= aBoard.pads.size() )
            return false;

        if( aOccupancy.Board()->Connected( sourceIndex, targetIndex ) )
            continue;

        std::size_t routeSourceIndex = sourceIndex;
        std::size_t routeTargetIndex = targetIndex;

        const bool targetIsPlane = aBoard.pads[targetIndex].isPlaneTarget;

        // Select the connected component independently of ratsnest ordering.
        // For ordinary nets it becomes the destination set below, matching
        // upstream's unconnected-set -> connected-set search direction.
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
        if( targetIsPlane && !aBoard.pads[routeTargetIndex].isExactTarget )
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

            std::vector<ROUTING_TERMINAL> starts;
            std::vector<ROUTING_TERMINAL> destinations;
            // Upstream AutorouteConnectionRouter routes from the unconnected
            // set to the selected item's connected set for ordinary nets.
            // Preserve all legal pad terminals, not just the ratsnest pair.
            // Fanout and plane tasks retain their explicit layer transitions.
            const bool fanoutTask = target.isFanoutTarget
                                    && target.fanoutSourcePadIndex == routeSourceIndex;
            if( !fanoutTask && !targetIsPlane && aNet.planeTargetIndices.empty() )
            {
                const std::size_t connectedRoot = findRoot( routeSourceIndex );
                destinations = aOccupancy.Board()->Terminals( routeSourceIndex );
                std::set<std::size_t> visited{ connectedRoot };
                for( std::size_t index : aNet.padIndices )
                {
                    if( index >= aBoard.pads.size() || !visited.insert( findRoot( index ) ).second )
                        continue;
                    auto terminals = aOccupancy.Board()->Terminals( index );
                    starts.insert( starts.end(), terminals.begin(), terminals.end() );
                }
            }

            // Honour the explicit per-connection attempt budget. The room
            // engine will replace this experimental raster retry mechanism.
            const int attemptsThisPass = std::max( 1, aSettings.maxIterations );

            for( int iteration = 0; iteration < attemptsThisPass; ++iteration )
            {
                if( aCancel && aCancel() )
                    return false;

                int expanded = 0;
                const int expandedBeforeSearch = aExpandedNodes;
                const auto searchStarted = std::chrono::steady_clock::now();
                const ROUTER_SEARCH_PROGRESS_CALLBACK searchProgress =
                        [&]( int aSearchExpanded )
                {
                    if( aSearchProgress )
                        aSearchProgress( expandedBeforeSearch + expanded + aSearchExpanded );
                };
                connection = aEngine.AutorouteConnection( source, target, aRetry + iteration,
                                                           expanded, aCancel, searchProgress,
                                                           starts, destinations );
                aExpandedNodes += expanded;

                if( autorouterDebugEnabled() )
                {
                    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                  std::chrono::steady_clock::now()
                                                          - searchStarted )
                                                  .count();
                    std::ostringstream message;
                    message << "search result net=" << aNet.netCode << " sourcePad="
                            << routeSourceIndex << " targetPad=" << candidate
                            << " iteration=" << iteration << " found=" << connection.has_value()
                            << " expanded=" << expanded << " elapsed=" << elapsed << " ms";
                    autorouterDebugLog( message.str() );
                }

                if( connection )
                {
                    routeSourceIndex = connection->fromPadIndex < aBoard.pads.size()
                                               ? connection->fromPadIndex : routeSourceIndex;
                    routeTargetIndex = connection->toPadIndex < aBoard.pads.size()
                                               ? connection->toPadIndex : candidate;
                    source = aBoard.pads[routeSourceIndex];
                    target = aBoard.pads[routeTargetIndex];
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
            continue;
        }

        connection->fromPadIndex = routeSourceIndex;
        connection->toPadIndex = routeTargetIndex;
        connection->isPlaneConnection = target.isPlaneTarget;
        connection->isFanoutConnection = target.isFanoutTarget
                                        && target.fanoutSourcePadIndex == routeSourceIndex;

        // On a negotiated-congestion retry the maze search may cross a
        // committed route.  Remove only the concrete connections that the
        // candidate actually occupies; arbitrary net-wide rip-up loses much
        // more useful copper than Freerouting's item-level rip-up and causes
        // large multi-pad nets to oscillate.
        if( aSettings.allowRipupRouted )
        {
            const std::vector<ROUTING_CONNECTION> conflicts =
                    aEngine.FindConflictingConnections( *connection );

            // Fanout connections are the electrical bridge from an SMD pad
            // to its synthetic landing.  Removing one during ordinary
            // negotiated routing would leave later connections starting at a
            // landing with no copper back to the real pad.  Treat those
            // bridges as protected and let this edge be retried on a later
            // pass instead of accepting a disconnected proposal.
            if( conflicts.size() > static_cast<std::size_t>(
                        std::max( 0, aSettings.maxRipups - aRipups ) )
                || std::any_of( conflicts.begin(), conflicts.end(),
                             []( const ROUTING_CONNECTION& aConflict )
                             {
                                 return aConflict.isFanoutConnection;
                             } ) )
            {
                connection.reset();
                continue;
            }

            for( const ROUTING_CONNECTION& conflict : conflicts )
            {
                bool removed = false;

                for( std::size_t index = aConnections.size(); index > 0; --index )
                {
                    const ROUTING_CONNECTION& existing = aConnections[index - 1];
                    if( existing.netCode != conflict.netCode || existing.nodes != conflict.nodes )
                        continue;

                    aOccupancy.Remove( existing );
                    aConnections.erase( aConnections.begin()
                                       + static_cast<std::ptrdiff_t>( index - 1 ) );
                    removed = true;
                    ++aRipups;
                    break;
                }

                // A connection accepted earlier in this routeNet call has
                // not been moved into aConnections yet, but it is already
                // present in the shared occupancy map.  Remove it from the
                // pending batch too so the result and occupancy stay in sync.
                if( !removed )
                {
                    for( std::size_t index = newConnections.size(); index > 0; --index )
                    {
                        const ROUTING_CONNECTION& existing = newConnections[index - 1];
                        if( existing.netCode != conflict.netCode
                            || existing.nodes != conflict.nodes )
                        {
                            continue;
                        }

                        aOccupancy.Remove( existing );
                        newConnections.erase( newConnections.begin()
                                              + static_cast<std::ptrdiff_t>( index - 1 ) );
                        removed = true;
                        ++aRipups;
                        break;
                    }
                }
            }

            if( autorouterDebugEnabled() && !conflicts.empty() )
            {
                std::ostringstream message;
                message << "negotiated ripup net=" << aNet.netCode
                        << " conflicts=" << conflicts.size();
                autorouterDebugLog( message.str() );
            }
        }

        aOccupancy.Add( *connection );
        newConnections.push_back( std::move( *connection ) );

        refreshContacts();
    }

    const std::size_t newConnectionCount = newConnections.size();
    aConnections.insert( aConnections.end(),
                         std::make_move_iterator( newConnections.begin() ),
                         std::make_move_iterator( newConnections.end() ) );

    const bool allConnectionsRouted =
            aOccupancy.Board()->CountMissing( aNet ) == 0;

    if( autorouterDebugEnabled() )
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - netStarted )
                                      .count();
        std::ostringstream message;
        message << "END net code=" << aNet.netCode << " routed=" << allConnectionsRouted
                << " newConnections=" << newConnectionCount << " expanded=" << aExpandedNodes
                << " elapsed=" << elapsed << " ms";
        autorouterDebugLog( message.str() );
    }

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
                        VIA_RULE::ThroughLayers( aSettings ), aResult );
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
    std::set<int> completeNets;
    ROUTING_BOARD copper( aBoard, aSettings );
    for( const auto& connection : aConnections )
        copper.AddRoute( connection );
    for( const ROUTING_NET& net : aBoard.nets )
    {
        if( !net.connections.empty()
            && copper.CountMissing( net ) == 0 )
        {
            completeNets.insert( net.netCode );
        }
    }

    if( aSettings.allowRipupExisting )
    {
        auto collectRemovable = [&]( const ROUTING_OBSTACLE& obstacle )
        {
            if( obstacle.isExistingRoute && completeNets.contains( obstacle.netCode ) && !obstacle.boardItemId.empty() )
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
    AUTOROUTER_DEBUG_SCOPE runScope( "routing job" );
    ROUTING_RESULT result;

    if( autorouterDebugEnabled() )
    {
        const int snapshotConnections = std::accumulate(
                aBoard.nets.begin(), aBoard.nets.end(), 0,
                []( int aTotal, const ROUTING_NET& aNet )
                {
                    return aTotal + static_cast<int>( aNet.connections.size() );
                } );
        std::ostringstream message;
        message << "snapshot pads=" << aBoard.pads.size() << " obstacles="
                << aBoard.obstacles.size() << " nets=" << aBoard.nets.size()
                << " connections=" << snapshotConnections << " maxPasses=" << aSettings.maxPasses
                << " maxIterations=" << aSettings.maxIterations
                << " maxExpanded=" << aSettings.maxExpandedNodes;
        autorouterDebugLog( message.str() );
    }

    ROUTER_PROGRESS preparing;
    preparing.maxPasses = std::max( 1, aSettings.maxPasses );
    preparing.totalConnections = std::accumulate(
            aBoard.nets.begin(), aBoard.nets.end(), 0,
            []( int aTotal, const ROUTING_NET& aNet )
            {
                return aTotal + static_cast<int>( aNet.connections.size() );
            } );
    preparing.stage = "Preparing SMD fanout";
    preparing.elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                                            std::chrono::steady_clock::now() - startTime )
                                            .count();

    if( aProgress )
        aProgress( preparing );

    if( aCancel && aCancel() )
    {
        result.cancelled = true;
        result.message = "Autorouter cancelled";
        return result;
    }

    // BatchFanout only changes the immutable worker snapshot.  The caller's
    // KiCad board remains untouched until the proposal is accepted.
    const auto fanoutStarted = std::chrono::steady_clock::now();
    const BOARD_SNAPSHOT routedBoard = BATCH_FANOUT::PrepareSnapshot( aBoard, aSettings, aCancel );

    if( autorouterDebugEnabled() )
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - fanoutStarted )
                                      .count();
        std::ostringstream message;
        message << "fanout complete elapsed=" << elapsed << " ms pads="
                << routedBoard.pads.size() << " obstacles=" << routedBoard.obstacles.size();
        autorouterDebugLog( message.str() );
    }

    if( aCancel && aCancel() )
    {
        result.cancelled = true;
        result.message = "Autorouter cancelled";
        return result;
    }

    // Keep the fanout-expanded snapshot mutable until the fanout pre-pass has
    // decided which synthetic landings are actually usable.  A failed
    // synthetic escape must be removed from the graph and its ordinary
    // connections must point back at the real pad; otherwise the main router
    // can route successfully from an electrically disconnected landing.
    BOARD_SNAPSHOT        workingBoard = routedBoard;
    BOARD_SNAPSHOT&       board = workingBoard;
    const std::vector<NET_ORDER_ENTRY> orderedNets = orderNets( board );
    int totalConnections = std::accumulate(
            orderedNets.begin(), orderedNets.end(), 0,
            []( int aTotal, const NET_ORDER_ENTRY& aEntry )
            {
                return aTotal + static_cast<int>( aEntry.net->connections.size() );
            } );

    result.metrics.totalConnections = totalConnections;
    ROUTING_OCCUPANCY occupancy( aSettings.gridStepIU );
    occupancy.InitializeBoard( board, aSettings );
    // The search engine builds the immutable obstacle/spatial index once per
    // engine.  Reuse it for every connection in a pass; reconstructing it for
    // each net makes large boards spend most of their runtime re-indexing the
    // same pads and keepouts rather than expanding paths.
    AUTOROUTE_ENGINE routeEngine( board, aSettings, occupancy );
    autorouterDebugLog( "route engine constructed" );
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
        int missing = 0;
        for( const ROUTING_NET& net : board.nets )
            missing += occupancy.Board()->CountMissing( net );
        return totalConnections - missing;
    };

    // Freerouting's SMD fanout is a real pre-pass.  It escapes the pad to a
    // via landing before ordinary nets are allowed to reserve board space.
    // Treating the synthetic pad-to-landing edges as ordinary ratsnest edges
    // makes their legality depend on whichever unrelated net happened to be
    // routed first.  On the Arduino board that prevents otherwise trivial
    // escapes (the first retry then starts from an already blocked landing)
    // and leaves a large number of dangling vias behind.
    BOARD_SNAPSHOT fanoutBoard = board;
    int            fanoutConnectionTotal = 0;
    for( ROUTING_NET& net : fanoutBoard.nets )
    {
        std::vector<std::pair<std::size_t, std::size_t>> fanoutConnections;
        fanoutConnections.reserve( net.connections.size() );

        for( const auto& [source, target] : net.connections )
        {
            if( source < fanoutBoard.pads.size() && target < fanoutBoard.pads.size()
                && !fanoutBoard.pads[source].isFanoutTarget
                && fanoutBoard.pads[target].isFanoutTarget )
            {
                fanoutConnections.emplace_back( source, target );
            }
        }

        net.connections = std::move( fanoutConnections );
        fanoutConnectionTotal += static_cast<int>( net.connections.size() );
    }

    if( fanoutConnectionTotal > 0 )
    {
        const std::vector<NET_ORDER_ENTRY> orderedFanoutNets = orderNets( fanoutBoard );

        if( autorouterDebugEnabled() )
        {
            std::ostringstream message;
            message << "BEGIN fanout pre-pass connections=" << fanoutConnectionTotal;
            autorouterDebugLog( message.str() );
        }

        for( const NET_ORDER_ENTRY& entry : orderedFanoutNets )
        {
            if( aCancel && aCancel() )
            {
                result.cancelled = true;
                result.message = "Autorouter cancelled";
                break;
            }

            int expanded = 0;
            const bool routed = routeNet( fanoutBoard, aSettings, *entry.net, 1, occupancy,
                                          routeEngine, connections, expanded, ripups, aCancel, nullptr );
            totalExpandedNodes += expanded;

            if( !routed )
                failedNets.insert( entry.net->netCode );

            result.metrics.routedConnections = routedConnectionCount();

            if( aProgress )
            {
                ROUTER_PROGRESS progress;
                progress.pass = 0;
                progress.maxPasses = std::max( 1, aSettings.maxPasses );
                progress.totalConnections = totalConnections;
                progress.routedConnections = result.metrics.routedConnections;
                progress.expandedNodes = totalExpandedNodes;
                progress.elapsedMilliseconds = elapsedMilliseconds();
                progress.stage = "Routing SMD fanout";
                aProgress( progress );
            }
        }

        if( autorouterDebugEnabled() )
        {
            std::ostringstream message;
            message << "END fanout pre-pass routed=" << routedConnectionCount()
                    << " expanded=" << totalExpandedNodes;
            autorouterDebugLog( message.str() );
        }
    }

    // Any landing that did not survive the isolated fanout pre-pass must be
    // removed from the electrical graph.  Falling back to the original pad
    // keeps the ordinary connection routable and, more importantly, prevents
    // a successful-looking route from starting at a synthetic point that has
    // no copper back to its SMD pad.
    std::set<std::size_t> completedFanoutLandings;
    for( const ROUTING_CONNECTION& connection : connections )
    {
        if( !connection.complete )
            continue;

        for( std::size_t endpoint : { connection.fromPadIndex, connection.toPadIndex } )
        {
            if( endpoint < board.pads.size() && board.pads[endpoint].isFanoutTarget )
                completedFanoutLandings.insert( endpoint );
        }
    }

    std::map<std::size_t, std::size_t> failedFanoutLandings;
    for( std::size_t index = 0; index < board.pads.size(); ++index )
    {
        const ROUTING_PAD& pad = board.pads[index];
        if( pad.isFanoutTarget
            && pad.fanoutSourcePadIndex != std::numeric_limits<std::size_t>::max()
            && !completedFanoutLandings.contains( index ) )
        {
            failedFanoutLandings.emplace( index, pad.fanoutSourcePadIndex );
        }
    }

    if( !failedFanoutLandings.empty() )
    {
        for( ROUTING_NET& net : board.nets )
        {
            std::vector<std::pair<std::size_t, std::size_t>> normalized;
            normalized.reserve( net.connections.size() );

            for( const auto& [source, target] : net.connections )
            {
                if( source < board.pads.size() && target < board.pads.size()
                    && board.pads[target].isFanoutTarget
                    && !board.pads[source].isFanoutTarget
                    && failedFanoutLandings.contains( target ) )
                {
                    // This is the synthetic pad-to-landing edge itself.
                    continue;
                }

                const auto mapEndpoint = [&]( std::size_t endpoint )
                {
                    const auto it = failedFanoutLandings.find( endpoint );
                    return it == failedFanoutLandings.end() ? endpoint : it->second;
                };

                const std::size_t mappedSource = mapEndpoint( source );
                const std::size_t mappedTarget = mapEndpoint( target );
                if( mappedSource != mappedTarget )
                    normalized.emplace_back( mappedSource, mappedTarget );
            }

            net.connections = std::move( normalized );
        }

        totalConnections = std::accumulate(
                board.nets.begin(), board.nets.end(), 0,
                []( int aTotal, const ROUTING_NET& aNet )
                {
                    return aTotal + static_cast<int>( aNet.connections.size() );
                } );
        result.metrics.totalConnections = totalConnections;

        if( autorouterDebugEnabled() )
        {
            std::ostringstream message;
            message << "fanout fallback landings=" << failedFanoutLandings.size()
                    << " totalConnections=" << totalConnections;
            autorouterDebugLog( message.str() );
        }
    }

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

        if( autorouterDebugEnabled() )
        {
            std::ostringstream message;
            message << "BEGIN pass=" << pass + 1 << " of " << std::max( 1, aSettings.maxPasses )
                    << " totalConnections=" << totalConnections;
            autorouterDebugLog( message.str() );
        }

        complete = true;

        for( const NET_ORDER_ENTRY& entry : orderedNets )
        {
            if( aCancel && aCancel() )
            {
                result.cancelled = true;
                result.message = "Autorouter cancelled";
                break;
            }

            int expanded = 0;
            std::map<int, std::size_t> routedBefore;
            for( const ROUTING_CONNECTION& connection : connections )
            {
                if( connection.complete )
                    ++routedBefore[connection.netCode];
            }

            const bool completeNet =
                    occupancy.Board()->CountMissing( *entry.net ) == 0;
            const bool retryFailedNet = failedNets.contains( entry.net->netCode );

            // Keep successful nets stable while negotiated-congestion passes
            // revisit only nets that failed or were ripped up.  Re-routing
            // every complete net on every pass causes a dense board to churn
            // thousands of valid connections and can leave the final pass
            // with fewer routes than an earlier checkpoint.
            if( completeNet && ( pass == 0 || !retryFailedNet ) )
                continue;

            const bool routed = routeNet( board, aSettings, *entry.net, pass, occupancy,
                                          routeEngine,
                                          connections, expanded, ripups, aCancel,
                                          [&]( int aSearchExpanded )
            {
                if( !aProgress )
                    return;

                ROUTER_PROGRESS searchProgress;
                searchProgress.pass = pass + 1;
                searchProgress.maxPasses = std::max( 1, aSettings.maxPasses );
                searchProgress.totalConnections = totalConnections;
                searchProgress.routedConnections = result.metrics.routedConnections;
                searchProgress.ripups = ripups;
                searchProgress.retries = retries;
                searchProgress.expandedNodes = totalExpandedNodes + aSearchExpanded;
                searchProgress.elapsedMilliseconds = elapsedMilliseconds();
                searchProgress.stage = "Searching connection";
                aProgress( searchProgress );
            } );
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

                // Only a found path identifies the copper that must be
                // removed. Never sacrifice an unrelated completed route
                // merely because this search exhausted its budget.

                ++retries;
            }
            else
            {
                failedNets.erase( entry.net->netCode );
            }

            // A retry may legally cross an occupied route.  routeNet removes
            // the concrete conflicting connection after the candidate path is
            // accepted, but that victim is not the net currently being
            // processed.  Mark every net whose completed-connection count
            // dropped so a later negotiated-congestion pass can put the
            // ripped item back.  Without this bookkeeping, a successful
            // retry silently discarded earlier routes and the final board was
            // permanently worse than the best intermediate pass.
            std::map<int, std::size_t> routedAfter;
            for( const ROUTING_CONNECTION& connection : connections )
            {
                if( connection.complete )
                    ++routedAfter[connection.netCode];
            }

            for( const auto& [netCode, countBefore] : routedBefore )
            {
                const std::size_t countAfter = routedAfter[netCode];
                if( countAfter < countBefore )
                    failedNets.insert( netCode );
            }

            result.metrics.routedConnections = routedConnectionCount();

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

            if( autorouterDebugEnabled() )
            {
                std::ostringstream message;
                message << "connection batch net=" << entry.net->netCode << " routed=" << routed
                        << " totalRouted=" << result.metrics.routedConnections
                        << " expanded=" << expanded << " retries=" << retries
                        << " ripups=" << ripups;
                autorouterDebugLog( message.str() );
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
        // Fanout landings are temporary routing stages, not final electrical
        // terminals. Return requests to their real pads before normalizing
        // redundant via tails, otherwise virtual landings force unused vias.
        for( auto& net : board.nets )
            for( auto& [from, to] : net.connections )
                for( auto* index : { &from, &to } )
                    if( *index < board.pads.size() && board.pads[*index].isFanoutTarget )
                        *index = board.pads[*index].fanoutSourcePadIndex;
        BATCH_OPTIMIZER( board, aSettings, occupancy ).RemoveRedundantViaTails( connections, aCancel );
        if( aCancel && aCancel() )
        {
            result.cancelled = true;
            result.message = "Autorouter cancelled";
            return result;
        }
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
            result.message = "Routing tasks finished; verify KiCad connectivity and design rules";
        }

        for( const auto& net : board.nets )
            if( occupancy.Board()->CountMissing( net ) > 0 )
                result.unroutedNetCodes.push_back( net.netCode );
    }

    const auto endTime = std::chrono::steady_clock::now();
    result.metrics.elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                  endTime - startTime )
                                                  .count();
    return result;
}

} // namespace KICAD_AUTOROUTER
