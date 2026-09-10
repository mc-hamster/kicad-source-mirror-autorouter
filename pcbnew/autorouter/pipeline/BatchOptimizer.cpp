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

#include "BatchOptimizer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>

#include "../AutorouterDebug.h"
#include "../BoardHistory.h"
#include "../ItemRouteResult.h"
#include "../board/optimize/ViaOptimizer.h"
#include "../maze/MazeTraceShover.h"
#include "../path/FoundConnectionInserter.h"
#include "ReadSortedRouteItems.h"


namespace KICAD_AUTOROUTER
{
namespace
{
bool isProtectedSourceCopper( const ROUTING_CONNECTION& aConnection )
{
    return aConnection.isExistingBoardRoute && !aConnection.isAutorouterOwned;
}


void promoteChangedAutorouterCopper( ROUTING_CONNECTION& aConnection,
                                     const ROUTING_CONNECTION& aOriginal )
{
    if( aOriginal.isExistingBoardRoute && aOriginal.isAutorouterOwned
        && !SameRouteGeometry( aConnection, aOriginal ) )
    {
        aConnection.isExistingBoardRoute = false;
        aConnection.isAutorouterOwned = false;
        aConnection.isShoveMovable = true;
    }
}


int viaCount( const ROUTING_CONNECTION& aConnection )
{
    int result = 0;

    for( std::size_t index = 1; index < aConnection.nodes.size(); ++index )
        if( aConnection.nodes[index - 1].layer != aConnection.nodes[index].layer )
            ++result;

    return result;
}


double traceLength( const ROUTING_CONNECTION& aConnection )
{
    double result = 0.0;

    for( std::size_t index = 1; index < aConnection.nodes.size(); ++index )
    {
        const ROUTER_NODE& previous = aConnection.nodes[index - 1];
        const ROUTER_NODE& current = aConnection.nodes[index];

        if( previous.layer == current.layer )
        {
            const long double dx = static_cast<long double>( current.point.x )
                                   - previous.point.x;
            const long double dy = static_cast<long double>( current.point.y )
                                   - previous.point.y;
            result += std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
        }
    }

    return result;
}


struct ROUTE_QUALITY
{
    int    incomplete = 0;
    int    vias = 0;
    int    bends = 0;
    double length = 0.0;
};


int bendCount( const ROUTING_CONNECTION& aConnection )
{
    int result = 0;

    for( std::size_t node = 1; node + 1 < aConnection.nodes.size(); ++node )
    {
        if( aConnection.nodes[node - 1].layer != aConnection.nodes[node].layer
            || aConnection.nodes[node].layer != aConnection.nodes[node + 1].layer
            || EdgeStyle( aConnection, node - 1 ) != EdgeStyle( aConnection, node ) )
        {
            continue;
        }

        ++result;
    }

    return result;
}


ROUTE_QUALITY routeQuality( const BOARD_SNAPSHOT& aBoard,
                            const std::vector<ROUTING_CONNECTION>& aConnections,
                            const ROUTING_BOARD& aRoutingBoard,
                            std::size_t aOmit = std::numeric_limits<std::size_t>::max(),
                            const ROUTING_CONNECTION* aReplacement = nullptr )
{
    ROUTE_QUALITY result;

    for( const ROUTING_NET& net : aBoard.nets )
        result.incomplete += aRoutingBoard.CountMissing( net );

    for( std::size_t index = 0; index < aConnections.size(); ++index )
    {
        if( index == aOmit || !aConnections[index].complete )
            continue;

        result.vias += viaCount( aConnections[index] );
        result.bends += bendCount( aConnections[index] );
        result.length += traceLength( aConnections[index] );
    }

    if( aReplacement && aReplacement->complete )
    {
        result.vias += viaCount( *aReplacement );
        result.bends += bendCount( *aReplacement );
        result.length += traceLength( *aReplacement );
    }

    return result;
}


double optimizerScore( const BOARD_SNAPSHOT& aBoard, const ROUTE_QUALITY& aQuality,
                       const AUTOROUTER_SETTINGS& aSettings )
{
    const int maximumConnections = std::accumulate(
            aBoard.nets.begin(), aBoard.nets.end(), 0,
            []( int aTotal, const ROUTING_NET& aNet )
            { return aTotal + static_cast<int>( aNet.connections.size() ); } );

    ROUTING_RESULT snapshot;
    snapshot.metrics.totalConnections = maximumConnections;
    snapshot.metrics.unroutedConnections = aQuality.incomplete;
    snapshot.metrics.viaCount = aQuality.vias;
    snapshot.metrics.bendCount = aQuality.bends;
    snapshot.metrics.routedLengthIU = static_cast<std::int64_t>(
            std::llround( aQuality.length ) );
    // Every optimizer candidate is accepted only after strict insertion and
    // the outer pipeline performs the independent full proposal DRC.  The
    // Java pass-score call also omits its expensive full DRC here.
    snapshot.metrics.drcViolations = 0;
    return BOARD_HISTORY::NormalizedScore( snapshot, aSettings );
}


bool isImprovement( const ROUTE_QUALITY& aBefore, const ROUTE_QUALITY& aAfter )
{
    return ITEM_ROUTE_RESULT( 0, aBefore.vias, aAfter.vias, aBefore.length,
                              aAfter.length, aBefore.incomplete,
                              aAfter.incomplete ).Improved();
}


bool betterThan( const ROUTE_QUALITY& aLeft, const ROUTE_QUALITY& aRight )
{
    const ITEM_ROUTE_RESULT left( 0, 0, aLeft.vias, 0.0, aLeft.length, 0,
                                  aLeft.incomplete );
    const ITEM_ROUTE_RESULT right( 0, 0, aRight.vias, 0.0, aRight.length, 0,
                                   aRight.incomplete );
    return left.CompareTo( right ) < 0;
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


bool hasExternalInteriorContact( const ROUTING_CONNECTION& aConnection,
                                 const ROUTING_BOARD& aBoard )
{
    if( aConnection.nodes.size() < 2 )
        return false;

    const ROUTER_POINT front = aConnection.nodes.front().point;
    const ROUTER_POINT back = aConnection.nodes.back().point;
    for( std::size_t index = 1; index < aConnection.nodes.size(); ++index )
    {
        const ROUTER_NODE& first = aConnection.nodes[index - 1];
        const ROUTER_NODE& second = aConnection.nodes[index];
        if( first.layer != second.layer )
            continue;

        for( const ROUTER_POINT& junction :
             aBoard.TraceJunctions( aConnection.netCode, first, second ) )
        {
            // The two connection ends are intentionally free to move: the
            // reroute starts from their complete post-removal components.
            // Any other contact belongs to a branch/via that is outside this
            // ROUTING_CONNECTION. Moving only the trunk would strand that
            // item even when pad connectivity happened to remain satisfied.
            if( junction != front && junction != back )
                return true;
        }
    }

    return false;
}
}

void BATCH_OPTIMIZER::simplifyConnection( ROUTING_CONNECTION& aConnection,
                                          const MAZE_SEARCH_ENGINE& aSearch ) const
{
    MAZE_TRACE_SHOVER::Shorten( aConnection, aSearch );
}


void BATCH_OPTIMIZER::RemoveRedundantViaTails( std::vector<ROUTING_CONNECTION>& aConnections,
                                             const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    if( !m_occupancy.Board() )
        return;

    for( auto& connection : aConnections )
    {
        // A source BOARD_ITEM held in static occupancy has no worker copper
        // record to trim.  It is either retained unchanged or first promoted
        // to a proposal route by the checked forced-shove path.
        if( isProtectedSourceCopper( connection ) )
            continue;
        // Terminal via transitions may become redundant after a later trace
        // attaches on the source layer. Remove only if every real-pad/plane
        // contact component survives; synthetic fanout requests are not copper.
        auto groups = m_occupancy.Board()->ConnectedPadGroups( connection.netCode );
        for( auto& group : groups )
            std::erase_if( group, [&]( auto pad ) { return m_board.pads[pad].isFanoutTarget; } );
        // A fanout may be completely bypassed by subsequent same-layer
        // copper. Keeping its stub after deleting the via creates a dangling
        // track, so first try removing the whole redundant fanout item.
        if( connection.isFanoutConnection )
        {
            m_occupancy.Remove( connection );
            bool preservesContacts = true;
            for( const auto& group : groups )
                for( auto pad : group )
                    if( !m_occupancy.Board()->Connected( group.front(), pad ) )
                        preservesContacts = false;
            if( preservesContacts )
            {
                ClearRouteGeometry( connection );
                continue;
            }
            m_occupancy.Add( connection );
        }
        for( bool front : { true, false } )
        {
            if( aCancel && aCancel() )
                return;
            if( connection.nodes.size() < 2 )
                break;
            const auto& first = front ? connection.nodes[0]
                                      : connection.nodes[connection.nodes.size() - 2];
            const auto& second = front ? connection.nodes[1] : connection.nodes.back();
            if( first.layer == second.layer || first.point != second.point )
                continue;
            const ROUTING_CONNECTION original = connection;
            m_occupancy.Remove( original );
            if( !RemoveRouteEndpoint( connection, front ) )
            {
                connection = original;
                m_occupancy.Add( original );
                continue;
            }
            promoteChangedAutorouterCopper( connection, original );
            m_occupancy.Add( connection );
            bool preservesContacts = true;
            for( const auto& group : groups )
                for( auto pad : group )
                    if( !m_occupancy.Board()->Connected( group.front(), pad ) )
                        preservesContacts = false;
            if( !preservesContacts )
            {
                m_occupancy.Remove( connection );
                connection = original;
                m_occupancy.Add( connection );
            }
        }
    }
    removeTraceTails( aConnections, aCancel );
    std::erase_if( aConnections, []( const auto& route ) { return route.nodes.empty(); } );
}


void BATCH_OPTIMIZER::removeTraceTails( std::vector<ROUTING_CONNECTION>& connections,
                                      const ROUTER_CANCEL_CALLBACK& cancel ) const
{
    // Freerouting removeTraceTails operates on normalized trace items, split
    // at their contacts. Native paths can still span a via/T junction, so
    // deleting the entire last segment would also delete the useful trunk.
    // Trim only toward exact centre-line junctions and retain every real
    // pad/plane component. Existing host copper is never a deletion candidate.
    bool changed;
    do
    {
        changed = false;
        for( auto& connection : connections )
        {
            if( isProtectedSourceCopper( connection ) )
                continue;
            if( cancel && cancel() )
                return;
            auto groups = m_occupancy.Board()->ConnectedPadGroups( connection.netCode );
            for( auto& group : groups )
                std::erase_if( group, [&]( auto pad ) { return m_board.pads[pad].isFanoutTarget; } );
            std::int64_t width = 0;
            for( const auto& net : m_board.nets )
                if( net.netCode == connection.netCode )
                    for( auto index : net.padIndices )
                        width = std::max( width, m_board.pads[index].trackWidth );
            const auto radius = ( width > 0 ? width : 150000 ) / 2;
            for( bool front : { true, false } )
                while( connection.nodes.size() > 1 )
                {
                    if( cancel && cancel() )
                        return;
                    const auto from = front ? connection.nodes.front() : connection.nodes.back();
                    const auto to = front ? connection.nodes[1] : connection.nodes[connection.nodes.size() - 2];
                    if( from.layer != to.layer )
                        break;
                    const auto original = connection;
                    m_occupancy.Remove( original );
                    const bool endHasContact = m_occupancy.Board()->HasCopperAt(
                            connection.netCode, from, radius );
                    auto junctions = m_occupancy.Board()->TraceJunctions( connection.netCode, from, to );
                    std::erase( junctions, from.point );
                    if( connection.isFanoutConnection && autorouterDebugEnabled() )
                    {
                        std::ostringstream message;
                        message << "FANOUT_TAIL net=" << connection.netCode
                                << " front=" << front << " from=(" << from.point.x << ','
                                << from.point.y << ",L" << from.layer << ") to=("
                                << to.point.x << ',' << to.point.y << ",L" << to.layer
                                << ") contact=" << endHasContact
                                << " junctions=" << junctions.size();
                        for( const ROUTER_POINT& point : junctions )
                            message << " (" << point.x << ',' << point.y << ')';
                        autorouterDebugLog( message.str() );
                    }
                    // Also collapse an overlapping end back to its first
                    // centre-line contact. A point inside another trace is
                    // electrically connected, but retaining a doubled stub
                    // past that junction still fails KiCad's dangling check.
                    if( endHasContact && junctions.empty() )
                    { m_occupancy.Add( original ); break; }
                    const auto next = junctions.empty() ? to.point : junctions.front();
                    if( next == from.point )
                    { m_occupancy.Add( original ); break; }
                    if( next == to.point )
                        RemoveRouteEndpoint( connection, front );
                    else
                        ( front ? connection.nodes.front() : connection.nodes.back() ).point = next;
                    promoteChangedAutorouterCopper( connection, original );
                    m_occupancy.Add( connection );
                    bool preserves = true;
                    for( const auto& group : groups )
                        for( auto pad : group )
                            if( !m_occupancy.Board()->Connected( group.front(), pad ) )
                                preserves = false;
                    if( !preserves )
                    {
                        m_occupancy.Remove( connection );
                        connection = original;
                        m_occupancy.Add( original );
                        break;
                    }
                    changed = true;
                }
            if( connection.nodes.size() == 1 )
            {
                m_occupancy.Remove( connection );
                ClearRouteGeometry( connection );
            }
        }
    } while( changed ); // Strictly removes vertices/length; never grows copper.
}


int BATCH_OPTIMIZER::Optimize( std::vector<ROUTING_CONNECTION>& aConnections,
                               const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    if( !m_settings.optimizeAfterComplete )
        return 0;

    // An optimization candidate is compared only after strict insertion.  Do
    // not let a negotiated-search crossing delete an unrelated connection in
    // this item-local transaction; Freerouting's wider rip-up optimization is
    // represented by subsequent ordinary batch passes, while this native
    // slice remains monotonically safe.
    int completedPasses = 0;
    int optimizedItems = 0;
    bool useIncreasedRipupCosts = true;

    for( int pass = 0; pass < std::max( 0, m_settings.optimizationPasses ); ++pass )
    {
        if( aCancel && aCancel() )
            break;

        const ROUTE_QUALITY passBefore = routeQuality( m_board, aConnections,
                                                       *m_occupancy.Board() );
        const double scoreBeforePass = optimizerScore( m_board, passBefore, m_settings );
        const double threshold = std::max( 0.0, m_settings.optimizationImprovementThreshold );
        if( threshold > 0.0 && scoreBeforePass * ( 1.0 + threshold ) >= 1000.0 )
            break;

        AUTOROUTER_SETTINGS optimizationSettings = m_settings;
        optimizationSettings.allowRipupRouted = false;
        // BatchOptimizer alternates its preferred-direction costs by pass to
        // generate a different legal candidate ordering.  On even source
        // pass numbers both directions use the preferred (minimum) cost.
        if( ( pass + 1 ) % 2 == 0 )
            for( ROUTER_LAYER_SETTINGS& layer : optimizationSettings.layers )
                layer.preferredDirection = 0;

        if( useIncreasedRipupCosts )
        {
            const std::int64_t scaled = static_cast<std::int64_t>(
                    optimizationSettings.startRipupCost )
                    * std::max( 1, m_settings.optimizationAdditionalRipupCostFactorAtStart );
            optimizationSettings.startRipupCost = static_cast<int>( std::clamp<std::int64_t>(
                    scaled, 0, std::numeric_limits<int>::max() ) );
        }

        bool changedThisPass = false;

        int consecutiveFailures = 0;

        // ReadSortedRouteItems.next() re-reads the mutable board after every
        // accepted or rolled-back item transaction.  Keep the source's
        // monotonic x/y/layer cursor instead of sorting one stale pass list.
        READ_SORTED_ROUTE_ITEMS sortedRouteItems;

        while( true )
        {
            const auto next = sortedRouteItems.Next( *m_occupancy.Board(), aConnections );
            if( !next )
                break;

            const std::size_t connectionIndex = next->connectionIndex;
            ROUTING_CONNECTION& connection = aConnections[connectionIndex];

            AUTOROUTER_SETTINGS itemOptimizationSettings = optimizationSettings;
            if( next->key.kind == 1 )
            {
                const double scaled = std::round(
                        std::max( 0.0, m_settings.optimizationTraceRipupCostFactor )
                        * itemOptimizationSettings.startRipupCost );
                itemOptimizationSettings.startRipupCost = static_cast<int>(
                        std::clamp( scaled, 0.0,
                                    static_cast<double>( std::numeric_limits<int>::max() ) ) );
            }
            MAZE_SEARCH_ENGINE search( m_board, itemOptimizationSettings, m_occupancy );

            if( isProtectedSourceCopper( connection ) )
                continue;
            if( aCancel && aCancel() )
                break;

            if( m_settings.maxOptimizationItems > 0
                && optimizedItems >= m_settings.maxOptimizationItems )
            {
                return completedPasses;
            }

            if( !connection.complete || connection.nodes.size() < 2 )
                continue;

            ++optimizedItems;
            const ROUTING_CONNECTION original = connection;
            const ROUTE_QUALITY before = routeQuality( m_board, aConnections,
                                                       *m_occupancy.Board() );
            auto groups = m_occupancy.Board()->ConnectedPadGroups( connection.netCode );
            for( auto& group : groups )
                std::erase_if( group, [&]( std::size_t aPad )
                              { return m_board.pads[aPad].isFanoutTarget; } );
            const auto movableViaEdges = VIA_OPTIMIZER::MovableViaEdges(
                    original, m_board, *m_occupancy.Board() );

            ROUTING_OCCUPANCY::TRANSACTION transaction( m_occupancy );
            m_occupancy.Remove( original );

            // Connection.get() in Freerouting stops a candidate chain at
            // forks. Native route records can still contain a trunk whose
            // interior is contacted by another record. Until that graph is
            // split into the same chains, fail closed instead of moving the
            // trunk and leaving an unconnected branch end in the KiCad
            // proposal. The transaction restores the removed route here.
            if( hasExternalInteriorContact( original, *m_occupancy.Board() ) )
            {
                ++consecutiveFailures;
                if( m_settings.maxOptimizationConsecutiveFailures > 0
                    && consecutiveFailures >= m_settings.maxOptimizationConsecutiveFailures )
                {
                    break;
                }
                continue;
            }

            std::optional<ROUTING_CONNECTION> bestConnection;
            ROUTE_QUALITY bestQuality = routeQuality( m_board, aConnections,
                                                       *m_occupancy.Board(),
                                                       connectionIndex );
            bool bestIsDeletion = isImprovement( before, bestQuality )
                                  && preservesPadGroups( groups, *m_occupancy.Board() );
            bool haveBest = bestIsDeletion;

            const auto relocateSyntheticEnds = [&]( const ROUTING_CONNECTION& aRoute )
            {
                if( aRoute.nodes.empty() )
                    return;
                m_occupancy.Board()->RelocateSyntheticPad( aRoute.fromPadIndex,
                                                            aRoute.nodes.front().point );
                m_occupancy.Board()->RelocateSyntheticPad( aRoute.toPadIndex,
                                                            aRoute.nodes.back().point );
            };

            const auto consider = [&]( ROUTING_CONNECTION aCandidate )
            {
                if( aCancel && aCancel() )
                    return;

                aCandidate.complete = true;
                aCandidate.netCode = original.netCode;
                aCandidate.fromPadIndex = original.fromPadIndex;
                aCandidate.toPadIndex = original.toPadIndex;
                aCandidate.isPlaneConnection = original.isPlaneConnection;
                aCandidate.isFanoutConnection = original.isFanoutConnection;
                aCandidate.sourceBoardItemIds = original.sourceBoardItemIds;
                promoteChangedAutorouterCopper( aCandidate, original );

                const auto inserted = FOUND_CONNECTION_INSERTER::Insert(
                        aCandidate, {}, m_occupancy, search, aCancel, false );
                if( inserted.state != FOUND_CONNECTION_INSERTER::STATE::INSERTED
                    || !inserted.shoved.empty() )
                {
                    return;
                }

                ROUTING_CONNECTION effective = inserted.connection
                        ? *inserted.connection : std::move( aCandidate );
                effective.complete = true;
                effective.fromPadIndex = original.fromPadIndex;
                effective.toPadIndex = original.toPadIndex;
                effective.isPlaneConnection = original.isPlaneConnection;
                effective.isFanoutConnection = original.isFanoutConnection;
                effective.sourceBoardItemIds = original.sourceBoardItemIds;
                promoteChangedAutorouterCopper( effective, original );

                // ViaOptimizer may move a plane/fanout drill at a synthetic
                // connection endpoint.  Freerouting moves the Via item itself;
                // the native planning terminal must follow temporarily so
                // CountMissing evaluates the same physical copper. Restore it
                // after this alternative, or retain it with the winner below.
                relocateSyntheticEnds( effective );

                const ROUTE_QUALITY quality = routeQuality(
                        m_board, aConnections, *m_occupancy.Board(), connectionIndex,
                        &effective );
                const bool preserves = preservesPadGroups( groups, *m_occupancy.Board() );
                // Insert owns its own atomic transaction. With an empty
                // victim list it cannot move another route, so removing its
                // effective candidate restores the post-removal base for the
                // next alternative without another full board snapshot.
                m_occupancy.Remove( effective );
                relocateSyntheticEnds( original );

                if( preserves && isImprovement( before, quality )
                    && ( !haveBest || betterThan( quality, bestQuality ) ) )
                {
                    bestConnection = std::move( effective );
                    bestQuality = quality;
                    bestIsDeletion = false;
                    haveBest = true;
                }
            };

            // Pull-tight remains useful when the complete maze cannot find a
            // better route inside its bounded work budget.  It is evaluated
            // through the same strict insertion and contact checks as a full
            // reroute, rather than mutating the route in place.
            ROUTING_CONNECTION shortened = original;
            simplifyConnection( shortened, search );
            if( !SameRouteGeometry( shortened, original ) )
                consider( std::move( shortened ) );

            // Freerouting optimizes unfixed Via items before traces at the
            // same board position.  Translate its weighted two-trace via
            // repositioning candidates while the original connection is
            // absent from occupancy, then subject every candidate to the
            // same atomic insertion/contact/quality gate as a full reroute.
            for( ROUTING_CONNECTION viaCandidate :
                 VIA_OPTIMIZER::Candidates( original, m_board, itemOptimizationSettings,
                                            *m_occupancy.Board(), search,
                                            movableViaEdges, aCancel ) )
            {
                consider( std::move( viaCandidate ) );
            }

            // Freerouting removes the item's complete connection chain and
            // invokes bounded autoroute passes on the remaining components.
            // Native ROUTING_CONNECTION records already delimit a connection;
            // route between its post-removal pad components and keep only a
            // lexicographically better, contact-preserving candidate.
            const bool validPads = original.fromPadIndex < m_board.pads.size()
                                   && original.toPadIndex < m_board.pads.size();
            if( validPads && !original.isFanoutConnection
                && !( aCancel && aCancel() ) )
            {
                const ROUTING_PAD& from = m_board.pads[original.fromPadIndex];
                const ROUTING_PAD& to = m_board.pads[original.toPadIndex];
                const auto starts = m_occupancy.Board()->Terminals( original.fromPadIndex );
                const auto targets = m_occupancy.Board()->Terminals( original.toPadIndex );

                for( int retry = 0;
                     retry < std::max( 1, m_settings.maxOptimizationAutoroutePasses );
                     ++retry )
                {
                    if( aCancel && aCancel() )
                        break;

                    int expanded = 0;
                    auto rerouted = search.FindConnection( from, to, retry, expanded,
                                                           aCancel, {}, starts, targets, false );
                    if( rerouted )
                    {
                        consider( std::move( *rerouted ) );
                        // An item-local batch pass is complete after its one
                        // disconnected component has routed. Later retries
                        // exist for failed attempts; they are not alternative
                        // searches after success.
                        break;
                    }
                }
            }

            if( aCancel && aCancel() )
                break;

            if( bestConnection )
            {
                m_occupancy.Add( *bestConnection );
                relocateSyntheticEnds( *bestConnection );
                connection = std::move( *bestConnection );
                transaction.Commit();
                changedThisPass = true;
                consecutiveFailures = 0;
            }
            else if( bestIsDeletion )
            {
                transaction.Commit();
                aConnections.erase( aConnections.begin()
                                    + static_cast<std::ptrdiff_t>( connectionIndex ) );
                changedThisPass = true;
                consecutiveFailures = 0;
            }
            else
            {
                ++consecutiveFailures;
            }

            if( m_settings.maxOptimizationConsecutiveFailures > 0
                && consecutiveFailures >= m_settings.maxOptimizationConsecutiveFailures )
            {
                break;
            }
        }

        ++completedPasses;

        const ROUTE_QUALITY passAfter = routeQuality( m_board, aConnections,
                                                      *m_occupancy.Board() );
        const double scoreAfterPass = optimizerScore( m_board, passAfter, m_settings );

        // The source always gives a no-improvement increased-cost pass one
        // more chance using normal prices, regardless of the threshold.
        if( useIncreasedRipupCosts && scoreAfterPass <= scoreBeforePass )
        {
            useIncreasedRipupCosts = false;
            continue;
        }

        if( !changedThisPass )
            break;

        const double improvement = scoreBeforePass > 0.0
                ? ( scoreAfterPass - scoreBeforePass ) / scoreBeforePass : 0.0;
        if( improvement < threshold )
            break;
    }

    return completedPasses;
}

} // namespace KICAD_AUTOROUTER
