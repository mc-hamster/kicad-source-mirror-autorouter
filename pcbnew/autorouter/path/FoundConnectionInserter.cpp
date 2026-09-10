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

#include "FoundConnectionInserter.h"
#include "../maze/MazeSearchEngine.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace KICAD_AUTOROUTER
{

namespace
{

ROUTER_POINT pointAlong( const ROUTER_POINT& aFrom, const ROUTER_POINT& aTo,
                         std::int64_t aNumerator, std::int64_t aDenominator )
{
    if( aDenominator <= 0 || aNumerator <= 0 )
        return aFrom;

    if( aNumerator >= aDenominator )
        return aTo;

    const long double ratio = static_cast<long double>( aNumerator ) / aDenominator;
    return { static_cast<std::int64_t>( std::llround(
                     static_cast<long double>( aFrom.x )
                     + ( static_cast<long double>( aTo.x ) - aFrom.x ) * ratio ) ),
             static_cast<std::int64_t>( std::llround(
                     static_cast<long double>( aFrom.y )
                     + ( static_cast<long double>( aTo.y ) - aFrom.y ) * ratio ) ) };
}


bool strictlyInsertable( const ROUTING_CONNECTION& aConnection,
                         const MAZE_SEARCH_ENGINE& aEngine )
{
    if( !HasValidEdgeStyles( aConnection ) )
        return false;

    for( std::size_t index = 1; index < aConnection.nodes.size(); ++index )
    {
        const ROUTING_EDGE_STYLE* style = aConnection.edgeStyles.empty()
                ? nullptr : &aConnection.edgeStyles[index - 1];
        if( !aEngine.CanInsertSegment( aConnection.netCode, aConnection.nodes[index - 1],
                                       aConnection.nodes[index], style ) )
        {
            return false;
        }
    }

    return true;
}


std::optional<ROUTING_CONNECTION> tryTerminalNeckdown(
        const ROUTING_CONNECTION& aConnection, std::size_t aBlockedEdge,
        bool aAtStart, const MAZE_SEARCH_ENGINE& aEngine, bool aMicroFallback = false )
{
    if( aBlockedEdge == 0 || aBlockedEdge >= aConnection.nodes.size()
        || !HasValidEdgeStyles( aConnection ) )
    {
        return {};
    }

    const std::size_t pinIndex = aAtStart ? aConnection.fromPadIndex
                                           : aConnection.toPadIndex;
    const ROUTER_NODE& start = aConnection.nodes[aBlockedEdge - 1];
    const ROUTER_NODE& end = aConnection.nodes[aBlockedEdge];
    const ROUTER_NODE& pin = aAtStart ? start : end;
    const ROUTER_NODE& far = aAtStart ? end : start;

    if( start.layer != end.layer || start.point == end.point )
        return {};

    const ROUTING_EDGE_STYLE& oldStyle = EdgeStyle( aConnection, aBlockedEdge - 1 );
    const std::int64_t normalWidth = aEngine.ResolveTrackWidth( aConnection.netCode, oldStyle );
    const std::optional<MAZE_SEARCH_ENGINE::PIN_ENTRY_STYLE> entry =
            aEngine.PinEntryStyle( pinIndex, pin, aConnection.netCode, normalWidth );

    // FoundConnectionInserter.tryNeckDown() reverses a start-pin segment and
    // calls the corresponding end-pin form directly. In both source calls
    // `toCorner` is the selected pin centre, so its pin-distance guard is a
    // validity check on that endpoint—not a cap on the length of the whole
    // terminal edge. Applying the guard to `far` here incorrectly disabled
    // neckdown for every ordinary long trace that enters a start pin and
    // forced a much larger spring-over instead.
    if( !entry )
        return {};

    // The regular source path first uses Pin.getTraceNeckdownHalfwidth(). If
    // that is unavailable or still cannot enter a dense SMD escape, its
    // fanout fallback tries a short deterministic series of reduced widths.
    // Preserve the old edge's clearance/via metadata; only a copper trace
    // width changes at this terminal edge.
    std::vector<ROUTING_EDGE_STYLE> narrowStyles;
    const auto appendNarrowStyle = [&]( std::int64_t aWidth )
    {
        if( aWidth <= 0 || aWidth >= normalWidth )
            return;
        ROUTING_EDGE_STYLE style = oldStyle;
        style.trackWidth = aWidth;
        if( std::find( narrowStyles.begin(), narrowStyles.end(), style ) == narrowStyles.end() )
            narrowStyles.push_back( std::move( style ) );
    };

    appendNarrowStyle( entry->style.trackWidth );
    if( aMicroFallback )
    {
        appendNarrowStyle( std::max<std::int64_t>( 1, normalWidth * 3 / 4 ) );
        appendNarrowStyle( std::max<std::int64_t>( 1, normalWidth * 3 / 5 ) );
        appendNarrowStyle( std::max<std::int64_t>( 1, normalWidth / 2 ) );
    }

    if( narrowStyles.empty() )
        return {};

    // RoutingBoard.checkTraceSegment() in the source reports the longest
    // legal full-width prefix.  The worker has no mutable partial insertion,
    // so find that same prefix by monotonic bisection from the non-pin end,
    // then preflight the reconstructed connection atomically below.
    const std::int64_t span = std::max( std::llabs( pin.point.x - far.point.x ),
                                        std::llabs( pin.point.y - far.point.y ) );
    if( span <= 0 )
        return {};

    const ROUTING_EDGE_STYLE* normalStyle = aConnection.edgeStyles.empty()
            ? nullptr : &aConnection.edgeStyles[aBlockedEdge - 1];
    if( !aEngine.CanInsertSegment( aConnection.netCode, far, far, normalStyle ) )
        return {};

    std::int64_t legal = 0;
    std::int64_t blocked = span;

    while( legal + 1 < blocked )
    {
        const std::int64_t probe = legal + ( blocked - legal ) / 2;
        const ROUTER_NODE candidate{ pointAlong( far.point, pin.point, probe, span ), far.layer };

        if( aEngine.CanInsertSegment( aConnection.netCode, far, candidate, normalStyle ) )
            legal = probe;
        else
            blocked = probe;
    }

    // Pin.getTraceNeckdownHalfwidth() keeps a two-coordinate tolerance from
    // the collision boundary before it emits the ordinary-width segment.
    // The worker coordinate is KiCad IU, so this is intentionally only an
    // integer rounding guard, not a copied physical clearance.
    constexpr std::int64_t neckdownTolerance = 2;
    legal = legal > neckdownTolerance ? legal - neckdownTolerance : 0;
    const ROUTER_POINT splitPoint = pointAlong( far.point, pin.point, legal, span );

    if( splitPoint == pin.point )
        return {};

    for( const ROUTING_EDGE_STYLE& narrowStyle : narrowStyles )
    {
        ROUTING_CONNECTION result = aConnection;
        EnsureEdgeStyles( result );
        const ROUTING_EDGE_STYLE ordinaryStyle = result.edgeStyles[aBlockedEdge - 1];

        if( splitPoint == far.point )
        {
            result.edgeStyles[aBlockedEdge - 1] = narrowStyle;
            if( strictlyInsertable( result, aEngine ) )
                return result;
            continue;
        }

        result.nodes.insert( result.nodes.begin() + static_cast<std::ptrdiff_t>( aBlockedEdge ),
                             { splitPoint, pin.layer } );
        if( aAtStart )
        {
            result.edgeStyles[aBlockedEdge - 1] = narrowStyle;
            result.edgeStyles.insert( result.edgeStyles.begin()
                                              + static_cast<std::ptrdiff_t>( aBlockedEdge ),
                                      ordinaryStyle );
        }
        else
        {
            result.edgeStyles[aBlockedEdge - 1] = ordinaryStyle;
            result.edgeStyles.insert( result.edgeStyles.begin()
                                              + static_cast<std::ptrdiff_t>( aBlockedEdge ),
                                      narrowStyle );
        }

        if( strictlyInsertable( result, aEngine ) )
            return result;
    }

    return {};
}


std::optional<ROUTING_CONNECTION> tryNeckdown( const ROUTING_CONNECTION& aConnection,
                                                std::size_t aBlockedEdge,
                                                const MAZE_SEARCH_ENGINE& aEngine )
{
    // The pinned source tries the start pin first, then the end pin. Limit
    // the mapped fallback to terminal edges; a non-terminal source corner is
    // not necessarily a physical PAD in the immutable KiCad snapshot.
    const bool atStart = aBlockedEdge == 1;
    const bool atEnd = aBlockedEdge + 1 == aConnection.nodes.size();
    if( atStart )
        if( auto result = tryTerminalNeckdown( aConnection, aBlockedEdge, true, aEngine ) )
            return result;

    if( atEnd )
        if( auto result = tryTerminalNeckdown( aConnection, aBlockedEdge, false, aEngine ) )
            return result;

    // FoundConnectionInserter.insertFanoutMicroNeckdown() runs only after
    // both source pin-width attempts failed. Keep that order so an exact
    // pad-derived neckdown wins over the generic fanout escape widths.
    if( atStart )
        if( auto result = tryTerminalNeckdown( aConnection, aBlockedEdge, true, aEngine, true ) )
            return result;

    if( atEnd )
        return tryTerminalNeckdown( aConnection, aBlockedEdge, false, aEngine, true );

    return {};
}


std::optional<std::vector<FOUND_CONNECTION_INSERTER::RESULT::SHOVED_CONNECTION>>
tryShoveGeneratedConnections( const ROUTING_CONNECTION& aCandidate,
                              const std::vector<ROUTING_CONNECTION>& aVictims,
                              ROUTING_OCCUPANCY& aOccupancy,
                              const MAZE_SEARCH_ENGINE& aEngine,
                              const ROUTER_CANCEL_CALLBACK& aCancel )
{
    // TraceShover has a source recursion guard of 20. This maps mutable
    // generated worker routes, including an unfixed generated fanout escape;
    // original board copper remains fixed/fail-closed unless a source-style
    // via move can atomically materialise every supported trace contact. The
    // previous implementation handled only the first conflict set: a
    // replacement trace that collided with a second generated trace was
    // rejected even when that trace could itself be shoved. Preserve the
    // source's bounded recursive behaviour by resolving that secondary
    // conflict transactionally before the parent replacement is committed.
    constexpr std::size_t maxTraceShoveDepth = 20;
    if( aVictims.empty() || aVictims.size() > maxTraceShoveDepth )
        return {};

    ROUTING_OCCUPANCY::TRANSACTION transaction( aOccupancy );
    aOccupancy.Add( aCandidate );
    std::vector<ROUTING_CONNECTION> transient{ aCandidate };
    std::vector<FOUND_CONNECTION_INSERTER::RESULT::SHOVED_CONNECTION> result;
    result.reserve( maxTraceShoveDepth );

    const auto isTransient = [&]( const ROUTING_CONNECTION& aRoute )
    {
        return std::any_of( transient.begin(), transient.end(),
                            [&]( const ROUTING_CONNECTION& aTransient )
                            { return SameRouteGeometry( aTransient, aRoute ); } );
    };

    const auto isMovable = [&]( const ROUTING_CONNECTION& aRoute )
    {
        // A fanout trace/via is generated, unfixed copper in the source
        // board. It may be recursively shoved just like another generated
        // route, provided every reconstructed edge passes strict insertion.
        // We still never treat original snapshot copper as mutable here.
        return aRoute.netCode != aCandidate.netCode && HasValidEdgeStyles( aRoute )
               && ( !aRoute.isExistingBoardRoute || aRoute.isShoveMovable );
    };

    const auto alreadyRelocated = [&]( const ROUTING_CONNECTION& aRoute )
    {
        return std::any_of( result.begin(), result.end(),
                            [&]( const auto& aShove )
                            {
                                return SameRouteGeometry( aShove.original, aRoute )
                                       || std::any_of(
                                               aShove.materializedContacts.begin(),
                                               aShove.materializedContacts.end(),
                                               [&]( const ROUTING_CONNECTION_REPLACEMENT& aContact )
                                               {
                                                   return SameRouteGeometry( aContact.original,
                                                                             aRoute );
                                               } );
                            } );
    };

    const auto isStaticStandaloneVia = []( const ROUTING_CONNECTION& aRoute )
    {
        return aRoute.isExistingBoardRoute && aRoute.sourceBoardItemIds.size() == 1
               && aRoute.nodes.size() == 2
               && aRoute.nodes.front().point == aRoute.nodes.back().point
               && aRoute.nodes.front().layer != aRoute.nodes.back().layer
               && aRoute.edgeStyles.size() == 1;
    };

    const auto contactCandidates = [&]()
    {
        // A static source trace can have been removed from occupancy with the
        // initial conflict set before its adjacent via is examined. Retain
        // those original records alongside the live occupancy view so the via
        // plan sees exactly the same trace contacts in either ordering.
        std::vector<ROUTING_CONNECTION> candidates = aOccupancy.Connections();
        for( const ROUTING_CONNECTION& victim : aVictims )
        {
            if( alreadyRelocated( victim )
                || std::any_of( candidates.begin(), candidates.end(),
                                [&]( const ROUTING_CONNECTION& aKnown )
                                { return SameRouteGeometry( aKnown, victim ); } ) )
            {
                continue;
            }
            candidates.push_back( victim );
        }
        return candidates;
    };

    const auto movementObstacles = [&]( const ROUTING_CONNECTION& aVictim )
    {
        // TraceShover obtains every overlapping board item from its search
        // tree before choosing a contour.  A generated route is not part of
        // the immutable snapshot, so rebuild that live subset explicitly.
        // Without this, a first shove could wrap only the incoming trace and
        // manufacture a collision with an unrelated generated route that was
        // already known to the occupancy index.
        std::vector<ROUTING_CONNECTION> obstacles = transient;
        for( const ROUTING_CONNECTION& existing : aOccupancy.Connections() )
        {
            if( SameRouteGeometry( existing, aVictim )
                || std::any_of( obstacles.begin(), obstacles.end(),
                                [&]( const ROUTING_CONNECTION& obstacle )
                                { return SameRouteGeometry( obstacle, existing ); } ) )
            {
                continue;
            }
            obstacles.push_back( existing );
        }
        return obstacles;
    };

    auto relocate = [&]( auto&& aSelf, const ROUTING_CONNECTION& aVictim,
                         std::size_t aDepth ) -> bool
    {
        if( aCancel && aCancel() )
            return false;
        if( aDepth >= maxTraceShoveDepth || !isMovable( aVictim )
            || alreadyRelocated( aVictim ) )
        {
            return false;
        }

        const std::vector<ROUTING_CONNECTION> obstacles = movementObstacles( aVictim );
        ROUTING_VIA_SHOVE_PLAN plan;
        if( auto moved = aEngine.SpringOverConnection( aVictim, obstacles, aCancel ) )
        {
            plan.replacement = std::move( *moved );
        }
        else
        {
            auto viaPlan = aEngine.ShoveViaConnectionPlan(
                    aVictim, obstacles, contactCandidates(), aCancel );
            if( !viaPlan )
                return false;
            plan = std::move( *viaPlan );
        }

        if( !plan.replacement.complete || !HasValidEdgeStyles( plan.replacement ) )
            return false;

        // A retained host trace/via is deliberately static in the worker
        // board until this exact checked relocation succeeds.  The moved
        // geometry is new proposal copper, while the copied UUID provenance
        // tells the adapter which original BOARD_ITEMs to remove on accept.
        // Once materialized it may participate in a deeper generated-copper
        // shove just like Freerouting's newly unfixed replacement item.
        if( aVictim.isExistingBoardRoute )
        {
            plan.replacement.isExistingBoardRoute = false;
            plan.replacement.isAutorouterOwned = false;
            plan.replacement.isShoveMovable = true;
        }

        for( ROUTING_CONNECTION_REPLACEMENT& contact : plan.materializedContacts )
        {
            // A plan never mutates the static BOARD_ITEM in place. It
            // publishes an equivalent worker trace and preserves the UUID so
            // batch output can remove the original only after this complete
            // replacement transaction succeeds.
            contact.replacement.isExistingBoardRoute = false;
            contact.replacement.isAutorouterOwned = false;
            contact.replacement.isShoveMovable = true;
            if( !contact.replacement.complete || !HasValidEdgeStyles( contact.replacement ) )
                return false;
        }
        for( ROUTING_CONNECTION& bridge : plan.bridges )
        {
            bridge.isExistingBoardRoute = false;
            bridge.isAutorouterOwned = false;
            bridge.isShoveMovable = true;
            if( !bridge.complete || !HasValidEdgeStyles( bridge ) )
                return false;
        }

        const auto forEachPlannedRoute = [&]( auto&& aVisitor )
        {
            aVisitor( plan.replacement );
            for( const ROUTING_CONNECTION_REPLACEMENT& contact : plan.materializedContacts )
                aVisitor( contact.replacement );
            for( const ROUTING_CONNECTION& bridge : plan.bridges )
                aVisitor( bridge );
        };

        // Child shoves must route around every member of the proposed via
        // move: the translated drill, materialised source traces, and bridge
        // traces. Keep the complete plan transient until all foreign copper
        // moves and every strict check succeeds.
        forEachPlannedRoute( [&]( const ROUTING_CONNECTION& aRoute )
        { transient.push_back( aRoute ); } );
        std::vector<ROUTING_CONNECTION> conflicts;
        forEachPlannedRoute( [&]( const ROUTING_CONNECTION& aRoute )
        {
            for( const ROUTING_CONNECTION& conflict : aEngine.FindConflictingConnections( aRoute ) )
            {
                if( std::none_of( conflicts.begin(), conflicts.end(),
                                  [&]( const ROUTING_CONNECTION& aKnown )
                                  { return SameRouteGeometry( aKnown, conflict ); } ) )
                {
                    conflicts.push_back( conflict );
                }
            }
        } );

        for( const ROUTING_CONNECTION& conflict : conflicts )
        {
            if( aCancel && aCancel() )
                return false;

            // Candidate copper and already-planned replacements are fixed
            // within this recursion branch.  Spring-over should have avoided
            // them exactly; accepting a collision here would create an
            // unstable swap/cycle rather than a source-style shove.
            if( isTransient( conflict ) || !isMovable( conflict )
                || alreadyRelocated( conflict ) )
            {
                return false;
            }

            aOccupancy.Remove( conflict );
            if( !aSelf( aSelf, conflict, aDepth + 1 ) )
                return false;
        }

        bool insertable = true;
        forEachPlannedRoute( [&]( const ROUTING_CONNECTION& aRoute )
        { insertable = insertable && strictlyInsertable( aRoute, aEngine ); } );
        if( !insertable )
            return false;

        // Remove static records before adding their proposal equivalents.
        // SameRouteGeometry intentionally treats the two as the same physical
        // BOARD_ITEM, so this order is essential: removing afterwards would
        // erase the freshly materialised trace instead.
        for( const ROUTING_CONNECTION_REPLACEMENT& contact : plan.materializedContacts )
            aOccupancy.Remove( contact.original );
        forEachPlannedRoute( [&]( const ROUTING_CONNECTION& aRoute )
        { aOccupancy.Add( aRoute ); } );
        result.push_back( { aVictim, std::move( plan.replacement ),
                            std::move( plan.materializedContacts ), std::move( plan.bridges ) } );
        return true;
    };

    // A source via must be considered before one of its static trace contacts
    // if both overlap the incoming candidate. Otherwise a fixed contact trace
    // would be rejected before DrillItem.moveBy() gets the chance to
    // materialise it and add its bridge.
    std::vector<ROUTING_CONNECTION> orderedVictims = aVictims;
    std::stable_sort( orderedVictims.begin(), orderedVictims.end(),
                      [&]( const ROUTING_CONNECTION& aLeft, const ROUTING_CONNECTION& aRight )
                      {
                          return isStaticStandaloneVia( aLeft )
                                 && !isStaticStandaloneVia( aRight );
                      } );
    for( const ROUTING_CONNECTION& victim : orderedVictims )
        if( !alreadyRelocated( victim ) && !relocate( relocate, victim, 0 ) )
            return {};

    transaction.Commit();
    return result;
}

} // namespace

FOUND_CONNECTION_INSERTER::RESULT FOUND_CONNECTION_INSERTER::Insert(
        const ROUTING_CONNECTION& connection, const std::vector<ROUTING_CONNECTION>& ripups,
        ROUTING_OCCUPANCY& occupancy, const MAZE_SEARCH_ENGINE& engine,
        const ROUTER_CANCEL_CALLBACK& requestedCancel, bool allowRipupFallback )
{
    bool cancelled = false;
    const ROUTER_CANCEL_CALLBACK cancel = [&]
    {
        cancelled = cancelled || ( requestedCancel && requestedCancel() );
        return cancelled;
    };
    if( !occupancy.Board() || !connection.complete || connection.netCode <= 0
        || connection.nodes.empty() || !HasValidEdgeStyles( connection ) )
        return { STATE::INVALID };
    if( cancel && cancel() )
        return { STATE::CANCELLED };
    ROUTING_OCCUPANCY::TRANSACTION transaction( occupancy );
    for( const auto& victim : ripups )
    {
        // A static host item may participate in a source-style shove, but it
        // must never fall through to a destructive ordinary rip-up if the
        // checked move fails.  Generated fanout copper has the same rule.
        if( std::none_of( occupancy.Connections().begin(), occupancy.Connections().end(),
                [&]( const auto& route )
                { return SameRouteGeometry( route, victim ); } ) )
            return { STATE::INVALID };
        occupancy.Remove( victim );
        if( cancel && cancel() )
            return { STATE::CANCELLED };

    }
    if( connection.nodes.size() == 1
        && !engine.CanInsertSegment( connection.netCode, connection.nodes[0], connection.nodes[0] ) )
        return { STATE::BLOCKED };
    std::optional<ROUTING_CONNECTION> replacement;
    std::size_t blockedEdge = 0;
    for( std::size_t i = 1; i < connection.nodes.size(); ++i )
    {
        if( cancel && cancel() ) return { STATE::CANCELLED, i };
        const ROUTING_EDGE_STYLE* style = connection.edgeStyles.empty()
                ? nullptr : &connection.edgeStyles[i - 1];
        if( !engine.CanInsertSegment( connection.netCode, connection.nodes[i - 1],
                                      connection.nodes[i], style ) )
        { blockedEdge = i; break; }
    }
    if( blockedEdge != 0 )
    {
        // This mirrors FoundConnectionInserter.insertTrace(): first retain
        // the ordinary-width prefix and enter a terminal pin at its legal
        // neckdown width; only then try spring-over for an obstacle that
        // cannot be solved by the pin-entry rule.
        replacement = tryNeckdown( connection, blockedEdge, engine );
        if( !replacement )
            replacement = engine.SpringOverConnection( connection, cancel );
        if( cancel && cancel() ) return { STATE::CANCELLED };
        if( !replacement ) return { STATE::BLOCKED, blockedEdge };
        for( std::size_t i = 1; i < replacement->nodes.size(); ++i )
        {
            if( cancel && cancel() ) return { STATE::CANCELLED, i };
            const ROUTING_EDGE_STYLE* style = replacement->edgeStyles.empty()
                    ? nullptr : &replacement->edgeStyles[i - 1];
            if( !engine.CanInsertSegment( replacement->netCode, replacement->nodes[i - 1],
                                          replacement->nodes[i], style ) )
                return { STATE::BLOCKED, blockedEdge };
        }
    }

    const ROUTING_CONNECTION& candidate = replacement ? *replacement : connection;
    if( !ripups.empty() )
    {
        if( auto shoved = tryShoveGeneratedConnections( candidate, ripups, occupancy, engine, cancel ) )
        {
            if( cancel && cancel() )
                return { STATE::CANCELLED };

            RESULT result{ STATE::INSERTED, 0, std::move( replacement ) };
            result.shoved = std::move( *shoved );
            transaction.Commit();
            return result;
        }

        if( cancel && cancel() )
            return { STATE::CANCELLED };

        // A batch pass may have exhausted its negotiated-congestion rip-up
        // budget, but source-style forced insertion is still allowed to move
        // every mutable generated victim out of the way. Do not silently turn
        // a failed shove into an over-budget deletion; the outer transaction
        // restores all original route identities on this path.
        const bool wouldDiscardProtectedCopper = std::any_of(
                ripups.begin(), ripups.end(), []( const ROUTING_CONNECTION& aVictim )
                {
                    return aVictim.isFanoutConnection
                           || ( aVictim.isExistingBoardRoute
                                && !aVictim.isAutorouterOwned );
                } );
        if( !allowRipupFallback || wouldDiscardProtectedCopper )
            return { STATE::BLOCKED, blockedEdge };
    }

    // Preflight uses the post-ripup board. Geometry/normal-contact splitting,
    // generated route records and congestion cells then commit together.
    occupancy.Add( candidate );
    if( cancel && cancel() )
        return { STATE::CANCELLED };
    transaction.Commit();
    return { STATE::INSERTED, 0, std::move( replacement ) };
}


void FOUND_CONNECTION_INSERTER::Append( const ROUTING_CONNECTION& aConnection,
                                        std::int64_t aTrackWidth,
                                        std::int64_t aViaDiameter,
                                        std::int64_t aViaDrill,
                                        const std::vector<int>& aViaLayers,
                                        ROUTING_RESULT& aResult )
{
    if( !HasValidEdgeStyles( aConnection ) )
        throw std::invalid_argument( "Cannot emit a route with malformed edge styles" );

    for( std::size_t index = 1; index < aConnection.nodes.size(); ++index )
    {
        const ROUTER_NODE& previous = aConnection.nodes[index - 1];
        const ROUTER_NODE& current = aConnection.nodes[index];
        const ROUTING_EDGE_STYLE& style = EdgeStyle( aConnection, index - 1 );
        const std::int64_t trackWidth = style.trackWidth > 0 ? style.trackWidth : aTrackWidth;
        const std::int64_t viaDiameter = style.viaDiameter > 0 ? style.viaDiameter : aViaDiameter;
        const std::int64_t viaDrill = style.viaDrill > 0 ? style.viaDrill : aViaDrill;
        const std::vector<int>& viaLayers = style.viaLayers.empty() ? aViaLayers : style.viaLayers;

        AppendEdge( aConnection.netCode, previous, current, trackWidth, viaDiameter, viaDrill,
                    viaLayers, aResult, std::max<std::int64_t>( 0, style.clearance ),
                    style.viaType );
    }
}


void FOUND_CONNECTION_INSERTER::AppendEdge( int aNetCode, const ROUTER_NODE& aPrevious,
                                            const ROUTER_NODE& aCurrent,
                                            std::int64_t aTrackWidth,
                                            std::int64_t aViaDiameter,
                                            std::int64_t aViaDrill,
                                            const std::vector<int>& aViaLayers,
                                            ROUTING_RESULT& aResult,
                                            std::int64_t aClearance,
                                            ROUTER_VIA_TYPE aViaType )
{
    const ROUTER_NODE& previous = aPrevious;
    const ROUTER_NODE& current = aCurrent;

    if( previous.layer == current.layer )
    {
        if( previous.point != current.point )
        {
            aResult.segments.push_back( { aNetCode, previous.layer, previous.point, current.point,
                                          aTrackWidth,
                                          std::max<std::int64_t>( 0, aClearance ) } );
        }
        return;
    }

    ROUTING_VIA via;
    via.netCode = aNetCode;
    via.position = previous.point;
    via.topLayer = aViaLayers.empty() ? previous.layer : aViaLayers.front();
    via.bottomLayer = aViaLayers.empty() ? current.layer : aViaLayers.back();
    via.diameter = aViaDiameter;
    via.drill = aViaDrill;
    via.layers = aViaLayers;
    via.clearance = std::max<std::int64_t>( 0, aClearance );
    via.type = aViaType;
    aResult.vias.push_back( std::move( via ) );
}

} // namespace KICAD_AUTOROUTER
