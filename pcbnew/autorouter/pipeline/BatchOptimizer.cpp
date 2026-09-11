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
#include <set>
#include <sstream>
#include <tuple>

#include "../AutorouterDebug.h"
#include "../BoardHistory.h"
#include "../ItemRouteResult.h"
#include "../board/optimize/ViaOptimizer.h"
#include "../maze/MazeTraceShover.h"
#include "../path/FoundConnectionInserter.h"
#include "BatchAutorouter.h"
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


ROUTING_BOARD::ITEM_ID_SET optimizerRippedItems( const ROUTING_BOARD& aBoard,
                                                 ROUTING_BOARD::ITEM_ID aSelected )
{
    const auto selected = aBoard.GetItemInfo( aSelected );
    if( !selected || !selected->routable )
        return {};

    // BatchOptimizer.optRouteItem first expands an unfixed trace selection to
    // every unfixed trace touching either endpoint of a fork. It then takes
    // Item.getConnectionItems(NONE) for each expanded item.
    ROUTING_BOARD::ITEM_ID_SET selectedItems{ aSelected };
    if( selected->kind == ROUTING_BOARD::ITEM_KIND::TRACE )
    {
        for( ROUTER_POINT endpoint : { selected->first, selected->last } )
        {
            const auto contacts = aBoard.NormalContactsAt( aSelected, endpoint );
            const bool onlyUnfixedTraces = std::all_of(
                    contacts.begin(), contacts.end(), [&]( ROUTING_BOARD::ITEM_ID aContact )
                    {
                        const auto item = aBoard.GetItemInfo( aContact );
                        return item && item->routable
                               && item->kind == ROUTING_BOARD::ITEM_KIND::TRACE;
                    } );
            if( onlyUnfixedTraces )
                selectedItems.insert( contacts.begin(), contacts.end() );
        }
    }

    ROUTING_BOARD::ITEM_ID_SET result;
    for( ROUTING_BOARD::ITEM_ID item : selectedItems )
    {
        const auto connection = aBoard.GetConnectionItems( item );
        result.insert( connection.begin(), connection.end() );
    }
    return result;
}


struct ITEM_BOUNDARY
{
    ROUTER_NODE node;
    ROUTING_BOARD::ITEM_ID outsideItem = 0;
    std::size_t padIndex = std::numeric_limits<std::size_t>::max();
};


std::vector<ITEM_BOUNDARY> itemBoundaries( const ROUTING_BOARD& aBoard,
                                          const ROUTING_BOARD::ITEM_ID_SET& aItems )
{
    std::vector<ITEM_BOUNDARY> result;
    std::set<std::tuple<ROUTING_BOARD::ITEM_ID, std::int64_t, std::int64_t, int>> seen;
    for( ROUTING_BOARD::ITEM_ID item : aItems )
    {
        for( ROUTING_BOARD::ITEM_ID contact : aBoard.GetNormalContacts( item ) )
        {
            if( aItems.contains( contact ) )
                continue;
            const auto point = aBoard.NormalContactPoint( item, contact );
            const int layer = aBoard.FirstCommonLayer( item, contact );
            const auto outside = aBoard.GetItemInfo( contact );
            if( !point || layer < 0 || !outside )
                continue;
            const auto key = std::tuple{ contact, point->x, point->y, layer };
            if( seen.insert( key ).second )
                result.push_back( { { *point, layer }, contact, outside->padIndex } );
        }
    }
    return result;
}


std::int64_t normalTrackWidth( const BOARD_SNAPSHOT& aBoard, int aNetCode )
{
    for( const ROUTING_NET& net : aBoard.nets )
    {
        if( net.netCode != aNetCode )
            continue;
        std::int64_t width = 0;
        for( std::size_t pad : net.padIndices )
            if( pad < aBoard.pads.size() )
                width = std::max( width, aBoard.pads[pad].trackWidth );
        return width > 0 ? width : 150000;
    }
    return 150000;
}


std::int64_t netClearance( const BOARD_SNAPSHOT& aBoard, int aNetCode )
{
    const auto net = std::find_if( aBoard.nets.begin(), aBoard.nets.end(),
                                   [&]( const ROUTING_NET& aNet )
                                   { return aNet.netCode == aNetCode; } );
    return net == aBoard.nets.end() ? 0 : std::max<std::int64_t>( 0, net->clearance );
}


ROUTING_PAD boundaryPad( const ITEM_BOUNDARY& aBoundary, int aNetCode,
                         std::int64_t aWidth, std::int64_t aClearance )
{
    ROUTING_PAD result;
    result.netCode = aNetCode;
    result.position = aBoundary.node.point;
    result.layers = { aBoundary.node.layer };
    result.trackWidth = aWidth;
    result.clearance = aClearance;
    result.isExactTarget = true;
    return result;
}


bool fanoutEscapeIsRedundant( const BOARD_SNAPSHOT& aBoard,
                              const ROUTING_CONNECTION& aConnection,
                              const ROUTING_BOARD& aRoutingBoard )
{
    std::size_t source = std::numeric_limits<std::size_t>::max();
    for( std::size_t endpoint : { aConnection.fromPadIndex, aConnection.toPadIndex } )
    {
        if( endpoint < aBoard.pads.size() && !aBoard.pads[endpoint].isFanoutTarget
            && aBoard.pads[endpoint].isSmd
            && aBoard.pads[endpoint].netCode == aConnection.netCode )
        {
            source = endpoint;
            break;
        }
    }

    if( source >= aBoard.pads.size() || aBoard.pads[source].layers.empty() )
        return false;

    const int sourceLayer = aBoard.pads[source].layers.front();
    if( aRoutingBoard.ConnectedSetTouchesOtherLayer( source, sourceLayer ) )
        return true;

    const auto net = std::find_if( aBoard.nets.begin(), aBoard.nets.end(),
                                   [&]( const ROUTING_NET& aNet )
                                   { return aNet.netCode == aConnection.netCode; } );
    if( net == aBoard.nets.end() )
        return false;

    const auto allConnected = [&]( const std::vector<std::size_t>& aItems )
    {
        return std::all_of( aItems.begin(), aItems.end(), [&]( std::size_t aItem )
        {
            return aItem >= aBoard.pads.size() || aItem == source
                   || aBoard.pads[aItem].isFanoutTarget
                   || aRoutingBoard.Connected( source, aItem );
        } );
    };

    // RoutingBoard.fanout() reports NO_UNCONNECTED_NETS instead of inserting
    // an escape when the pin's current item component already reaches every
    // real net item.  This is the only same-layer case in which deleting the
    // terminal drill is legitimate.
    return allConnected( net->padIndices ) && allConnected( net->planeTargetIndices );
}
}

void BATCH_OPTIMIZER::simplifyConnection( ROUTING_CONNECTION& aConnection,
                                          const MAZE_SEARCH_ENGINE& aSearch ) const
{
    MAZE_TRACE_SHOVER::Shorten( aConnection, aSearch );
}


void BATCH_OPTIMIZER::RemoveRedundantViaTails( std::vector<ROUTING_CONNECTION>& aConnections,
                                             const ROUTER_CANCEL_CALLBACK& aCancel,
                                             int aOnlyNetCode ) const
{
    if( !m_occupancy.Board() )
        return;

    for( auto& connection : aConnections )
    {
        if( aOnlyNetCode > 0 && connection.netCode != aOnlyNetCode )
            continue;
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
            if( preservesContacts
                && fanoutEscapeIsRedundant( m_board, connection,
                                            *m_occupancy.Board() ) )
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
            if( connection.isFanoutConnection
                && !fanoutEscapeIsRedundant( m_board, connection,
                                              *m_occupancy.Board() ) )
            {
                preservesContacts = false;
            }
            if( !preservesContacts )
            {
                m_occupancy.Remove( connection );
                connection = original;
                m_occupancy.Add( connection );
            }
        }
    }
    removeTraceTails( aConnections, aCancel, aOnlyNetCode );
    std::erase_if( aConnections, []( const auto& route ) { return route.nodes.empty(); } );
}


void BATCH_OPTIMIZER::removeTraceTails( std::vector<ROUTING_CONNECTION>& connections,
                                      const ROUTER_CANCEL_CALLBACK& cancel,
                                      int onlyNetCode ) const
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
            if( onlyNetCode > 0 && connection.netCode != onlyNetCode )
                continue;
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
                    if( connection.isFanoutConnection
                        && !fanoutEscapeIsRedundant( m_board, connection,
                                                     *m_occupancy.Board() ) )
                    {
                        preserves = false;
                    }
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
            if( connectionIndex >= aConnections.size() )
                continue;
            const ROUTING_CONNECTION original = aConnections[connectionIndex];

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

            if( isProtectedSourceCopper( original ) )
                continue;
            if( aCancel && aCancel() )
                break;

            if( m_settings.maxOptimizationItems > 0
                && optimizedItems >= m_settings.maxOptimizationItems )
            {
                return completedPasses;
            }

            if( !original.complete || original.nodes.size() < 2 )
                continue;

            ++optimizedItems;
            const ROUTE_QUALITY before = routeQuality( m_board, aConnections,
                                                       *m_occupancy.Board() );
            auto groups = m_occupancy.Board()->ConnectedPadGroups( original.netCode );
            for( auto& group : groups )
                std::erase_if( group, [&]( std::size_t aPad )
                              { return m_board.pads[aPad].isFanoutTarget; } );
            const auto movableViaEdges = VIA_OPTIMIZER::MovableViaEdges(
                    original, m_board, *m_occupancy.Board() );

            const auto rippedItems = optimizerRippedItems( *m_occupancy.Board(),
                                                            next->key.item );
            if( rippedItems.empty() )
            {
                ++consecutiveFailures;
                continue;
            }

            const auto boundaries = itemBoundaries( *m_occupancy.Board(), rippedItems );
            std::vector<std::string> sourceBoardItemIds;
            for( ROUTING_BOARD::ITEM_ID item : rippedItems )
            {
                const auto itemRoute = m_occupancy.Board()->ItemRoute( item );
                if( !itemRoute )
                    continue;
                for( const std::string& id : itemRoute->sourceBoardItemIds )
                    if( std::find( sourceBoardItemIds.begin(), sourceBoardItemIds.end(), id )
                        == sourceBoardItemIds.end() )
                    {
                        sourceBoardItemIds.push_back( id );
                    }
            }

            const std::vector<ROUTING_CONNECTION> connectionsBefore = aConnections;
            ROUTING_OCCUPANCY::TRANSACTION transaction( m_occupancy );
            if( !m_occupancy.RemoveItems( rippedItems ) )
            {
                aConnections = connectionsBefore;
                ++consecutiveFailures;
                if( m_settings.maxOptimizationConsecutiveFailures > 0
                    && consecutiveFailures >= m_settings.maxOptimizationConsecutiveFailures )
                {
                    break;
                }
                continue;
            }
            aConnections = m_occupancy.Connections();

            // Several boundary contacts may already belong to the same
            // post-removal copper component. BatchAutorouter sees components,
            // not raw contact count, so retain one deterministic representative.
            std::vector<ITEM_BOUNDARY> boundaryComponents;
            std::vector<std::set<ROUTING_BOARD::ITEM_ID>> boundarySets;
            for( const ITEM_BOUNDARY& boundary : boundaries )
            {
                const auto component = m_occupancy.Board()->ConnectedSet(
                        boundary.outsideItem );
                if( component.empty() )
                    continue;
                const bool represented = std::any_of(
                        boundarySets.begin(), boundarySets.end(), [&]( const auto& existing )
                        {
                            return std::any_of( component.begin(), component.end(),
                                                [&]( auto id ) { return existing.contains( id ); } );
                        } );
                if( !represented )
                {
                    boundaryComponents.push_back( boundary );
                    boundarySets.emplace_back( component.begin(), component.end() );
                }
            }

            std::vector<ROUTING_CONNECTION> bestConnections;
            ROUTE_QUALITY bestQuality = routeQuality( m_board, m_occupancy.Connections(),
                                                       *m_occupancy.Board() );
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

            const auto saveCandidate = [&]( const std::vector<ROUTING_CONNECTION>& aCandidate )
            {
                const ROUTE_QUALITY quality = routeQuality(
                        m_board, m_occupancy.Connections(), *m_occupancy.Board() );
                const bool preserves = preservesPadGroups( groups, *m_occupancy.Board() );
                if( preserves && isImprovement( before, quality )
                    && ( !haveBest || betterThan( quality, bestQuality ) ) )
                {
                    bestConnections = aCandidate;
                    bestQuality = quality;
                    bestIsDeletion = false;
                    haveBest = true;
                }
            };

            // BatchOptimizer.optRouteItem does not reconnect only the two
            // ends of the selected route.  After removing the complete item
            // chain it invokes BatchAutorouter.autoroutePassesForOptimizingItem,
            // which takes fresh whole-board item snapshots and may negotiate
            // another movable connection out of the way.  Run that operation
            // in a nested transaction: an accepted sub-pass keeps every
            // partial insertion/rip-up as one candidate, while rejection
            // restores the exact post-removal board before the bounded
            // pull-tight/via fallbacks below are considered.
            bool acceptedBatchCandidate = false;
            // Native fanout and plane routes use synthetic target pads which
            // Freerouting does not have. Feeding either ripped item to the
            // ordinary pass can bypass or duplicate the required SMD escape.
            // Keep their via movement in the dedicated candidate path below
            // until both use real drill/conduction-area items end to end.
            if( !original.isFanoutConnection && !original.isPlaneConnection )
            {
                ROUTING_OCCUPANCY::TRANSACTION batchTransaction( m_occupancy );
                std::vector<ROUTING_CONNECTION> batchConnections =
                        m_occupancy.Connections();
                BATCH_AUTOROUTER::AutoroutePassesForOptimizingItem(
                        m_board, itemOptimizationSettings,
                        std::max( 0, m_settings.maxOptimizationAutoroutePasses ),
                        m_occupancy, batchConnections, aCancel );

                // The source helper removes tails and optimizes the changed
                // area before BatchOptimizer evaluates board statistics.
                // Native cleanup is still a bounded subset of optChangedArea,
                // but it must run inside the same speculative transaction.
                RemoveRedundantViaTails( batchConnections, aCancel );
                batchConnections = m_occupancy.Connections();

                const ROUTE_QUALITY batchQuality = routeQuality(
                        m_board, batchConnections, *m_occupancy.Board() );
                if( !( aCancel && aCancel() )
                    && preservesPadGroups( groups, *m_occupancy.Board() )
                    && isImprovement( before, batchQuality ) )
                {
                    batchTransaction.Commit();
                    transaction.Commit();
                    aConnections = m_occupancy.Connections();
                    changedThisPass = true;
                    consecutiveFailures = 0;
                    acceptedBatchCandidate = true;
                }
            }

            if( acceptedBatchCandidate )
                continue;

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

                ROUTING_OCCUPANCY::TRANSACTION candidateTransaction( m_occupancy );
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
                relocateSyntheticEnds( effective );
                saveCandidate( { std::move( effective ) } );
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

            // The source reruns BatchAutorouter after deleting the exact item
            // set. Route the resulting boundary components, rather than the
            // stale from/to pads of the compound insertion request. A fork can
            // therefore produce several replacement PolylineTrace records.
            if( boundaryComponents.size() >= 2 && !original.isFanoutConnection
                && !( aCancel && aCancel() ) )
            {
                for( int retry = 0;
                     retry < std::max( 1, m_settings.maxOptimizationAutoroutePasses );
                     ++retry )
                {
                    if( aCancel && aCancel() )
                        break;

                    ROUTING_OCCUPANCY::TRANSACTION candidateTransaction( m_occupancy );
                    std::vector<ROUTING_CONNECTION> reroutedConnections;
                    bool routedAll = true;
                    const ITEM_BOUNDARY& anchor = boundaryComponents.front();
                    for( std::size_t boundaryIndex = 1;
                         boundaryIndex < boundaryComponents.size(); ++boundaryIndex )
                    {
                        const ITEM_BOUNDARY& target = boundaryComponents[boundaryIndex];
                        if( m_occupancy.Board()->ConnectedSet( anchor.outsideItem )
                                    .contains( target.outsideItem ) )
                        {
                            continue;
                        }

                        const std::int64_t width = normalTrackWidth( m_board,
                                                                     original.netCode );
                        const std::int64_t clearance = netClearance( m_board,
                                                                     original.netCode );
                        const ROUTING_PAD from = boundaryPad( anchor, original.netCode,
                                                              width, clearance );
                        const ROUTING_PAD to = boundaryPad( target, original.netCode,
                                                            width, clearance );
                        int expanded = 0;
                        auto rerouted = search.FindConnection( from, to, retry, expanded,
                                                               aCancel, {}, {}, {} );
                        if( !rerouted )
                        {
                            routedAll = false;
                            break;
                        }

                        rerouted->complete = true;
                        rerouted->netCode = original.netCode;
                        rerouted->fromPadIndex = anchor.padIndex;
                        rerouted->toPadIndex = target.padIndex;
                        rerouted->isPlaneConnection = original.isPlaneConnection;
                        if( reroutedConnections.empty() )
                            rerouted->sourceBoardItemIds = sourceBoardItemIds;

                        const auto inserted = FOUND_CONNECTION_INSERTER::Insert(
                                *rerouted, {}, m_occupancy, search, aCancel, false );
                        if( inserted.state != FOUND_CONNECTION_INSERTER::STATE::INSERTED
                            || !inserted.shoved.empty() )
                        {
                            routedAll = false;
                            break;
                        }

                        ROUTING_CONNECTION effective = inserted.connection
                                ? *inserted.connection : std::move( *rerouted );
                        effective.complete = true;
                        effective.fromPadIndex = anchor.padIndex;
                        effective.toPadIndex = target.padIndex;
                        effective.isPlaneConnection = original.isPlaneConnection;
                        if( reroutedConnections.empty() )
                            effective.sourceBoardItemIds = sourceBoardItemIds;
                        reroutedConnections.push_back( std::move( effective ) );
                    }

                    if( routedAll )
                        saveCandidate( reroutedConnections );
                    if( routedAll && haveBest )
                        break;
                }
            }

            if( aCancel && aCancel() )
            {
                aConnections = connectionsBefore;
                break;
            }

            if( !bestConnections.empty() )
            {
                for( const ROUTING_CONNECTION& connection : bestConnections )
                {
                    m_occupancy.Add( connection );
                    relocateSyntheticEnds( connection );
                }
                transaction.Commit();
                aConnections = m_occupancy.Connections();
                changedThisPass = true;
                consecutiveFailures = 0;
            }
            else if( bestIsDeletion )
            {
                transaction.Commit();
                aConnections = m_occupancy.Connections();
                changedThisPass = true;
                consecutiveFailures = 0;
            }
            else
            {
                aConnections = connectionsBefore;
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
