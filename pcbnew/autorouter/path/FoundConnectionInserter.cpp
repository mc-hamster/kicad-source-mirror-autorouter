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
#include "FoundConnectionLocator45Degree.h"
#include "../AutorouterDebug.h"
#include "../maze/MazeSearchEngine.h"
#include "../maze/MazeTraceShover.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace KICAD_AUTOROUTER
{

namespace
{

std::string formatConnectionForDebug( const char* aStage,
                                      const ROUTING_CONNECTION& aConnection )
{
    std::ostringstream log;
    log << "INSERT_CONNECTION stage=" << aStage
        << " net=" << aConnection.netCode
        << " nodes=" << aConnection.nodes.size();
    for( std::size_t index = 0; index < aConnection.nodes.size(); ++index )
    {
        const ROUTER_NODE& node = aConnection.nodes[index];
        log << " {i=" << index << ",x=" << node.point.x << ",y=" << node.point.y
            << ",layer=" << node.layer;
        if( index > 0 )
        {
            const ROUTING_EDGE_STYLE& style = EdgeStyle( aConnection, index - 1 );
            log << ",width=" << style.trackWidth << ",clearance=" << style.clearance;
        }
        log << '}';
    }
    return log.str();
}


void reverseConnection( ROUTING_CONNECTION& aConnection )
{
    std::reverse( aConnection.nodes.begin(), aConnection.nodes.end() );
    std::reverse( aConnection.edgeStyles.begin(), aConnection.edgeStyles.end() );
    std::swap( aConnection.fromPadIndex, aConnection.toPadIndex );
}


bool strictlyInsertable( const ROUTING_CONNECTION& aConnection,
                         const MAZE_SEARCH_ENGINE& aEngine )
{
    if( !HasValidEdgeStyles( aConnection ) )
        return false;

    if( aEngine.CanInsertTraceSpan( aConnection ) )
        return true;

    for( std::size_t index = 1; index < aConnection.nodes.size(); ++index )
    {
        const ROUTING_EDGE_STYLE* style = aConnection.edgeStyles.empty()
                ? nullptr : &aConnection.edgeStyles[index - 1];
        if( !aEngine.CanInsertSegment( aConnection.netCode, aConnection.nodes[index - 1],
                                       aConnection.nodes[index], style ) )
        {
            if( autorouterDebugEnabled() )
            {
                const ROUTER_NODE& from = aConnection.nodes[index - 1];
                const ROUTER_NODE& to = aConnection.nodes[index];
                autorouterDebugLog(
                        "INSERT_SEGMENT_BLOCKED net="
                        + std::to_string( aConnection.netCode ) + " edge="
                        + std::to_string( index - 1 ) + " from=("
                        + std::to_string( from.point.x ) + ','
                        + std::to_string( from.point.y ) + ",L"
                        + std::to_string( from.layer ) + ") to=("
                        + std::to_string( to.point.x ) + ','
                        + std::to_string( to.point.y ) + ",L"
                        + std::to_string( to.layer ) + ')' );
            }
            return false;
        }
    }

    return false;
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
        aWidth = std::max( aWidth, aEngine.MinimumTrackWidth() );
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

    const ROUTING_EDGE_STYLE* normalStyle = aConnection.edgeStyles.empty()
            ? nullptr : &aConnection.edgeStyles[aBlockedEdge - 1];
    double okLength = aEngine.CheckTraceSegmentLength(
            aConnection.netCode, far, pin, normalStyle );
    if( okLength == std::numeric_limits<double>::max() )
        return {};

    // tryNeckDown() uses two Freerouting coordinates, not two KiCad internal
    // units. Keep this second source tolerance after checkTraceSegment's own
    // one-coordinate projection guard.
    constexpr double neckdownTolerance =
            2.0 * FREEROUTING_COORDINATE_UNIT_IU;
    okLength -= neckdownTolerance;

    const FLOAT_POINT floatFar{ static_cast<double>( far.point.x ),
                                static_cast<double>( far.point.y ) };
    const FLOAT_POINT floatPin{ static_cast<double>( pin.point.x ),
                                static_cast<double>( pin.point.y ) };

    for( const ROUTING_EDGE_STYLE& narrowStyle : narrowStyles )
    {
        std::vector<ROUTER_NODE> orientedNodes{ far };
        std::vector<ROUTING_EDGE_STYLE> orientedStyles;
        const auto append = [&]( const ROUTER_POINT& aPoint,
                                 const ROUTING_EDGE_STYLE& aStyle )
        {
            if( orientedNodes.back().point == aPoint )
                return;
            orientedNodes.push_back( { aPoint, pin.layer } );
            orientedStyles.push_back( aStyle );
        };

        if( okLength > neckdownTolerance )
        {
            const FLOAT_POINT floatSplit = floatFar.ChangeLength( floatPin, okLength );
            const ROUTER_POINT split = floatSplit.Round();
            const bool horizontalFirst = std::abs( floatFar.x - floatSplit.x )
                                         >= std::abs( floatFar.y - floatSplit.y );
            const ROUTER_POINT approachCorner =
                    FOUND_CONNECTION_LOCATOR_45_DEGREE::CalculateAdditionalCorner(
                            floatFar, floatSplit, horizontalFirst, false ).Round();
            append( approachCorner, oldStyle );
            append( split, oldStyle );

            const ROUTER_POINT exitCorner =
                    FOUND_CONNECTION_LOCATOR_45_DEGREE::CalculateAdditionalCorner(
                            floatSplit, floatPin, !horizontalFirst, false ).Round();
            if( exitCorner != pin.point )
                append( exitCorner, oldStyle );
        }

        append( pin.point, narrowStyle );
        if( orientedStyles.empty() )
            continue;

        if( aAtStart )
        {
            std::reverse( orientedNodes.begin(), orientedNodes.end() );
            std::reverse( orientedStyles.begin(), orientedStyles.end() );
        }

        ROUTING_CONNECTION result = aConnection;
        EnsureEdgeStyles( result );
        result.nodes.erase( result.nodes.begin()
                                    + static_cast<std::ptrdiff_t>( aBlockedEdge - 1 ),
                            result.nodes.begin()
                                    + static_cast<std::ptrdiff_t>( aBlockedEdge + 1 ) );
        result.nodes.insert( result.nodes.begin()
                                     + static_cast<std::ptrdiff_t>( aBlockedEdge - 1 ),
                             orientedNodes.begin(), orientedNodes.end() );
        result.edgeStyles.erase( result.edgeStyles.begin()
                                         + static_cast<std::ptrdiff_t>( aBlockedEdge - 1 ) );
        result.edgeStyles.insert( result.edgeStyles.begin()
                                          + static_cast<std::ptrdiff_t>( aBlockedEdge - 1 ),
                                  orientedStyles.begin(), orientedStyles.end() );

        if( strictlyInsertable( result, aEngine ) )
            return result;

        if( autorouterDebugEnabled() )
        {
            std::ostringstream log;
            log << "NECKDOWN_REJECTED net=" << result.netCode << " at_start=" << aAtStart
                << " ok_length=" << okLength << " nodes=" << result.nodes.size();
            for( std::size_t edge = 1; edge < result.nodes.size(); ++edge )
            {
                const ROUTING_EDGE_STYLE& style = EdgeStyle( result, edge - 1 );
                log << " {(" << result.nodes[edge - 1].point.x << ','
                    << result.nodes[edge - 1].point.y << ",L"
                    << result.nodes[edge - 1].layer << ")->("
                    << result.nodes[edge].point.x << ',' << result.nodes[edge].point.y
                    << ",L" << result.nodes[edge].layer << "),w="
                    << style.trackWidth << ",c=" << style.clearance << ",ok="
                    << aEngine.CanInsertSegment( result.netCode, result.nodes[edge - 1],
                                                 result.nodes[edge], &style ) << '}';
            }
            autorouterDebugLog( log.str() );
        }
    }

    return {};
}


std::optional<ROUTING_CONNECTION> tryFanoutMicroNeckdown(
        const ROUTING_CONNECTION& aConnection, std::size_t aBlockedEdge,
        const MAZE_SEARCH_ENGINE& aEngine )
{
    if( !aConnection.isFanoutConnection || aBlockedEdge == 0
        || aBlockedEdge >= aConnection.nodes.size()
        || !HasValidEdgeStyles( aConnection ) )
    {
        return {};
    }

    const ROUTER_NODE& first = aConnection.nodes[aBlockedEdge - 1];
    const ROUTER_NODE& last = aConnection.nodes[aBlockedEdge];
    if( first.layer != last.layer || first.point == last.point )
        return {};

    const ROUTING_EDGE_STYLE& oldStyle = EdgeStyle( aConnection, aBlockedEdge - 1 );
    const std::int64_t normalWidth = aEngine.ResolveTrackWidth(
            aConnection.netCode, oldStyle );
    for( const std::int64_t width : {
                 std::max<std::int64_t>( 1, normalWidth * 3 / 4 ),
                 std::max<std::int64_t>( 1, normalWidth * 3 / 5 ),
                 std::max<std::int64_t>( 1, normalWidth / 2 ) } )
    {
        // Freerouting owns its board rules and can commit the raw 3/4, 3/5
        // or 1/2 width.  Here the final owner is KiCad: accepting a width
        // below BOARD_DESIGN_SETTINGS::m_TrackMinWidth only postpones an
        // inevitable transaction rejection and can make an otherwise valid
        // route appear to finish with new DRC errors.  Clamp before
        // deduplication/insertion while preserving source candidate order.
        const std::int64_t safeWidth = std::max(
                width, aEngine.MinimumTrackWidth() );
        if( safeWidth >= normalWidth )
            continue;

        ROUTING_CONNECTION result = aConnection;
        EnsureEdgeStyles( result );
        result.edgeStyles[aBlockedEdge - 1].trackWidth = safeWidth;
        if( strictlyInsertable( result, aEngine ) )
            return result;
    }

    return {};
}


std::optional<ROUTING_CONNECTION> tryNeckdown( const ROUTING_CONNECTION& aConnection,
                                                std::size_t aBlockedEdge,
                                                const MAZE_SEARCH_ENGINE& aEngine,
                                                bool aAtPhysicalStart,
                                                bool aAtPhysicalEnd )
{
    // The pinned source tries the start pin first, then the end pin. Limit
    // the mapped fallback to terminal edges; a non-terminal source corner is
    // not necessarily a physical PAD in the immutable KiCad snapshot.
    const bool atStart = aAtPhysicalStart && aBlockedEdge == 1;
    const bool atEnd = aAtPhysicalEnd && aBlockedEdge + 1 == aConnection.nodes.size();
    if( atStart )
        if( auto result = tryTerminalNeckdown( aConnection, aBlockedEdge, true, aEngine ) )
            return result;

    if( atEnd )
        if( auto result = tryTerminalNeckdown( aConnection, aBlockedEdge, false, aEngine ) )
            return result;

    // FoundConnectionInserter.insertFanoutMicroNeckdown() runs only after
    // both source pin-width attempts failed. Keep that order so an exact
    // pad-derived neckdown wins over the generic fanout escape widths.
    if( aConnection.isFanoutConnection )
    {
        if( atStart )
            if( auto result = tryTerminalNeckdown(
                        aConnection, aBlockedEdge, true, aEngine, true ) )
            {
                return result;
            }

        if( atEnd )
            if( auto result = tryTerminalNeckdown(
                        aConnection, aBlockedEdge, false, aEngine, true ) )
            {
                return result;
            }
    }

    // insertFanoutMicroNeckdown() is not restricted to a physical terminal
    // edge.  It is called after every failed two-corner fanout insertion and
    // can therefore narrow an intermediate locator segment before the next
    // longer spring-over retry.  This matters when the normal-width path is
    // legal in the compensated search tree but needs a short 3/4-width
    // segment to reproduce the source insertion exactly.
    if( auto result = tryFanoutMicroNeckdown(
                aConnection, aBlockedEdge, aEngine ) )
    {
        return result;
    }

    return {};
}


struct INCREMENTAL_INSERTION
{
    std::optional<ROUTING_CONNECTION> connection;
    std::size_t                       blockedEdge = 0;
    bool                              cancelled = false;
};


ROUTING_CONNECTION connectionSlice( const ROUTING_CONNECTION& aConnection,
                                    std::size_t aFirstNode, std::size_t aLastNode )
{
    ROUTING_CONNECTION result = aConnection;
    // A slice is a transient trace/via item, not another owner of the complete
    // connection's mutation journal.
    result.traceInsertionSteps.clear();
    result.nodes.assign( aConnection.nodes.begin() + static_cast<std::ptrdiff_t>( aFirstNode ),
                         aConnection.nodes.begin()
                                 + static_cast<std::ptrdiff_t>( aLastNode + 1 ) );

    if( !aConnection.edgeStyles.empty() )
    {
        result.edgeStyles.assign(
                aConnection.edgeStyles.begin()
                        + static_cast<std::ptrdiff_t>( aFirstNode ),
                aConnection.edgeStyles.begin()
                        + static_cast<std::ptrdiff_t>( aLastNode ) );
    }

    return result;
}


bool appendConnection( ROUTING_CONNECTION& aDestination,
                       const ROUTING_CONNECTION& aSource,
                       std::optional<std::size_t>* aNormalizedSplitNode = nullptr )
{
    if( aSource.nodes.empty() || !HasValidEdgeStyles( aSource ) )
        return false;

    std::size_t firstSourceNode = 0;
    if( aDestination.nodes.empty() )
    {
        aDestination.nodes.push_back( aSource.nodes.front() );
    }
    else if( aDestination.nodes.back() != aSource.nodes.front() )
    {
        // FoundConnectionInserter deliberately rewinds one source corner
        // after a two-corner forced insertion fails.  The copper inserted by
        // the preceding call is not rolled back: the next, longer polyline
        // begins one corner before its existing keep point and normalize()
        // removes that leading cycle/stub.  Represent the same union as one
        // path by joining at the retained keep point when it is an exact
        // corner of the longer span.
        const auto sourceJoin = std::find( aSource.nodes.begin(), aSource.nodes.end(),
                                           aDestination.nodes.back() );

        if( sourceJoin != aSource.nodes.end() )
        {
            firstSourceNode = static_cast<std::size_t>(
                    sourceJoin - aSource.nodes.begin() );

            // PolylineTrace.normalize() splits a newly inserted trace at the
            // first geometric overlap with existing same-net copper.  A
            // forced-insertion rewind commonly starts one corner before the
            // existing endpoint, so that overlap can begin in the interior
            // of the new trace's first segment rather than at one of its
            // explicit corners.  Retain that source item boundary: pullTight
            // is applied only to the newly normalized suffix.
            const auto pointOnSegment = []( const ROUTER_POINT& aPoint,
                                            const ROUTER_POINT& aStart,
                                            const ROUTER_POINT& aEnd )
            {
                using WIDE = __int128_t;
                const WIDE dx = WIDE( aEnd.x ) - aStart.x;
                const WIDE dy = WIDE( aEnd.y ) - aStart.y;
                const WIDE px = WIDE( aPoint.x ) - aStart.x;
                const WIDE py = WIDE( aPoint.y ) - aStart.y;
                if( dx * py != dy * px )
                    return false;
                return aPoint.x >= std::min( aStart.x, aEnd.x )
                       && aPoint.x <= std::max( aStart.x, aEnd.x )
                       && aPoint.y >= std::min( aStart.y, aEnd.y )
                       && aPoint.y <= std::max( aStart.y, aEnd.y );
            };
            const auto liesOnRewoundPrefix = [&]( const ROUTER_POINT& aPoint )
            {
                for( std::size_t edge = 0; edge < firstSourceNode; ++edge )
                    if( pointOnSegment( aPoint, aSource.nodes[edge].point,
                                       aSource.nodes[edge + 1].point ) )
                        return true;
                return false;
            };

            if( aNormalizedSplitNode && firstSourceNode > 0 )
            {
                std::size_t split = aDestination.nodes.size() - 1;
                while( split > 0
                       && liesOnRewoundPrefix( aDestination.nodes[split - 1].point ) )
                {
                    --split;
                }
                *aNormalizedSplitNode = split;
            }
        }
        else
        {
            // A spring-over retry may replace, rather than merely overlap,
            // the previously accepted approach.  The source board mutates
            // that aggregate PolylineTrace in place: when the longer retry
            // starts at an earlier retained corner, the old suffix is
            // discarded and the replacement span becomes authoritative.
            // Mirror that union on the private native candidate before it is
            // published to the occupancy transaction.
            const auto destinationJoin = std::find(
                    aDestination.nodes.begin(), aDestination.nodes.end(),
                    aSource.nodes.front() );

            if( destinationJoin == aDestination.nodes.end() )
                return false;

            const std::size_t keepNode = static_cast<std::size_t>(
                    destinationJoin - aDestination.nodes.begin() );
            aDestination.nodes.resize( keepNode + 1 );

            if( !aDestination.edgeStyles.empty() )
                aDestination.edgeStyles.resize( keepNode );
        }
    }

    for( std::size_t edge = firstSourceNode; edge + 1 < aSource.nodes.size(); ++edge )
    {
        aDestination.nodes.push_back( aSource.nodes[edge + 1] );
        aDestination.edgeStyles.push_back( EdgeStyle( aSource, edge ) );
    }

    return true;
}


/**
 * Reconstruct FoundConnectionInserter.insertTrace's fromCornerNo loop without
 * publishing an incomplete host proposal.  A failed short insertion is
 * allowed to consume another source corner; when the failed span started at
 * the previously accepted corner, the source rewinds one corner so its next
 * spring-over sees enough approach geometry to repair a compensated-clearance
 * violation.  The already inserted prefix remains present.  The longer span
 * overlaps it at the retained keep point and normalization removes the
 * resulting leading stub/cycle.
 *
 * Freerouting mutates its private RoutingBoard after every successful span and
 * relies on the caller's board snapshot for rollback.  The native worker keeps
 * the equivalent geometry private until this function has reconstructed the
 * whole connection; the surrounding ROUTING_OCCUPANCY::TRANSACTION provides
 * the same all-or-nothing externally visible result.
 */
INCREMENTAL_INSERTION buildIncrementalConnection(
        const ROUTING_CONNECTION& aConnection, const MAZE_SEARCH_ENGINE& aEngine,
        const ROUTER_CANCEL_CALLBACK& aCancel )
{
    INCREMENTAL_INSERTION outcome;
    ROUTING_CONNECTION result = aConnection;
    result.nodes.clear();
    result.edgeStyles.clear();
    result.traceInsertionSteps.clear();
    result.nodes.push_back( aConnection.nodes.front() );

    std::size_t edge = 0;
    while( edge + 1 < aConnection.nodes.size() )
    {
        if( aCancel && aCancel() )
        {
            outcome.cancelled = true;
            return outcome;
        }

        // ResultItems are layer-local traces separated by forced via inserts.
        // A via is not a trace-polyline corner and remains atomic.
        if( aConnection.nodes[edge].layer != aConnection.nodes[edge + 1].layer )
        {
            const ROUTING_EDGE_STYLE* style = aConnection.edgeStyles.empty()
                    ? nullptr : &aConnection.edgeStyles[edge];
            if( !aEngine.CanInsertSegment( aConnection.netCode, aConnection.nodes[edge],
                                           aConnection.nodes[edge + 1], style ) )
            {
                outcome.blockedEdge = edge + 1;
                return outcome;
            }

            const ROUTING_CONNECTION via = connectionSlice( aConnection, edge, edge + 1 );
            if( !appendConnection( result, via ) )
            {
                outcome.blockedEdge = edge + 1;
                return outcome;
            }
            ++edge;
            continue;
        }

        // TraceShover receives one layer and one width/clearance class at a
        // time.  Preserve native per-edge style boundaries by treating them
        // as separate source ResultItems.
        const std::size_t runFirstEdge = edge;
        std::size_t runLastEdge = edge;
        while( runLastEdge + 2 < aConnection.nodes.size()
               && aConnection.nodes[runLastEdge + 1].layer
                          == aConnection.nodes[runLastEdge + 2].layer
               && EdgeStyle( aConnection, runLastEdge + 1 )
                          == EdgeStyle( aConnection, runFirstEdge ) )
        {
            ++runLastEdge;
        }

        const std::size_t runFirstNode = runFirstEdge;
        const std::size_t runLastNode = runLastEdge + 1;
        std::size_t fromCornerNo = runFirstNode;

        for( std::size_t currentCornerNo = runFirstNode + 1;
             currentCornerNo <= runLastNode; ++currentCornerNo )
        {
            if( aCancel && aCancel() )
            {
                outcome.cancelled = true;
                return outcome;
            }

            ROUTING_CONNECTION span = connectionSlice( aConnection, fromCornerNo,
                                                        currentCornerNo );
            std::optional<ROUTING_CONNECTION> accepted;
            const char* acceptedBy = "none";

            // RoutingBoard.insertForcedTracePolyline() always calls
            // TraceShover.springOverObstacles() on the isolated new span
            // before combining it with the same-net trace at fromCorner.
            // This is not merely a fallback for a failed clearance check: a
            // short span whose endpoint lies inside foreign copper must fail
            // here even when a segment-only predicate would accept the
            // endpoint as a contact.  Returning an unchanged path is
            // therefore distinct from spring-over failure for this caller.
            std::optional<ROUTING_CONNECTION> prepared =
                    aEngine.SpringOverConnection( span, {}, aCancel, nullptr, true );

            ROUTING_CONNECTION insertionProbe = result;
            if( prepared && appendConnection( insertionProbe, *prepared )
                && strictlyInsertable( insertionProbe, aEngine ) )
            {
                acceptedBy = SameRouteGeometry( *prepared, span )
                                     ? "direct" : "spring_over";
                accepted = std::move( prepared );
            }
            else
            {
                std::size_t blockedInSpan = 0;
                for( std::size_t spanEdge = 1; spanEdge < span.nodes.size(); ++spanEdge )
                {
                    const ROUTING_EDGE_STYLE* style = span.edgeStyles.empty()
                            ? nullptr : &span.edgeStyles[spanEdge - 1];
                    if( !aEngine.CanInsertSegment( span.netCode, span.nodes[spanEdge - 1],
                                                   span.nodes[spanEdge], style ) )
                    {
                        blockedInSpan = spanEdge;
                        break;
                    }
                }

                // Source neckdown is attempted only for a two-corner trace
                // insertion and only when that endpoint really is a pin of
                // the complete connection.  A local ResultItem boundary is
                // not sufficient evidence of a physical pad.
                if( span.nodes.size() == 2 )
                {
                    // insertForcedTracePolyline() can return its first corner
                    // when the compensated segment predicate succeeds but
                    // spring-over of the physical obstacle shapes cannot
                    // preserve the normal-width span.  Freerouting still
                    // tries terminal/fanout neckdown in that case; requiring
                    // CanInsertSegment() to identify a blocked edge skipped
                    // the retry because that predicate intentionally sees
                    // only the compensated broad phase.
                    const std::size_t neckdownEdge = blockedInSpan != 0
                            ? blockedInSpan : 1;
                    accepted = tryNeckdown( span, neckdownEdge, aEngine,
                                            fromCornerNo == 0,
                                            currentCornerNo + 1 == aConnection.nodes.size() );
                    if( accepted )
                        acceptedBy = "neckdown";
                }

                if( aCancel && aCancel() )
                {
                    outcome.cancelled = true;
                    return outcome;
                }

                if( accepted )
                {
                    insertionProbe = result;
                    if( !appendConnection( insertionProbe, *accepted )
                        || !strictlyInsertable( insertionProbe, aEngine ) )
                    {
                        accepted.reset();
                    }
                }
            }

            if( accepted )
            {
                ROUTING_TRACE_INSERTION_STEP replayStep;
                replayStep.insertedSpan.nodes = accepted->nodes;
                replayStep.insertedSpan.edgeStyles = accepted->edgeStyles;
                if( autorouterDebugEnabled() )
                {
                    std::ostringstream log;
                    log << "INSERT_SPAN net=" << aConnection.netCode
                        << " from_corner=" << fromCornerNo
                        << " to_corner=" << currentCornerNo
                        << " accepted_by=" << acceptedBy
                        << " result_nodes=" << accepted->nodes.size();
                    autorouterDebugLog( log.str() );
                }
                std::optional<std::size_t> normalizedSplit;
                if( !appendConnection( result, *accepted, &normalizedSplit ) )
                {
                    outcome.blockedEdge = currentCornerNo;
                    return outcome;
                }
                replayStep.normalizedSplitNode = normalizedSplit;
                replayStep.combinedBeforeTighten.nodes = result.nodes;
                replayStep.combinedBeforeTighten.edgeStyles = result.edgeStyles;

                // RoutingBoard.insertForcedTracePolyline() combines the new
                // span with copper ending at its first corner, normalizes the
                // combined PolylineTrace and calls pullTight before returning
                // the accepted endpoint.  Delaying this until the complete
                // found connection has been assembled changes the obstacle
                // seen by the next batch item.  Keep the same incremental
                // mutation order on the private native candidate; Shorten
                // retains both endpoints, including the source keep-point at
                // currentCornerNo.
                // normalize() may have split this insertion into a new source
                // trace item.  Freerouting selects the item at the accepted
                // endpoint and tightens only that item; tightening the whole
                // concatenated native connection moves the already-finished
                // prefix and changes the board seen by the next net.
                if( normalizedSplit && *normalizedSplit + 1 < result.nodes.size() )
                {
                    ROUTING_CONNECTION suffix = result;
                    suffix.nodes.assign(
                            result.nodes.begin()
                                    + static_cast<std::ptrdiff_t>( *normalizedSplit ),
                            result.nodes.end() );
                    if( !result.edgeStyles.empty() )
                    {
                        suffix.edgeStyles.assign(
                                result.edgeStyles.begin()
                                        + static_cast<std::ptrdiff_t>( *normalizedSplit ),
                                result.edgeStyles.end() );
                    }
                    MAZE_TRACE_SHOVER::Shorten( suffix, aEngine );
                    result.nodes.resize( *normalizedSplit + 1 );
                    if( !result.edgeStyles.empty() )
                        result.edgeStyles.resize( *normalizedSplit );
                    if( !appendConnection( result, suffix ) )
                    {
                        outcome.blockedEdge = currentCornerNo;
                        return outcome;
                    }
                }
                else
                {
                    MAZE_TRACE_SHOVER::Shorten( result, aEngine );
                }
                replayStep.tightenedResult.nodes = result.nodes;
                replayStep.tightenedResult.edgeStyles = result.edgeStyles;
                result.traceInsertionSteps.push_back( std::move( replayStep ) );
                if( autorouterDebugEnabled() )
                    autorouterDebugLog( formatConnectionForDebug(
                            "incremental_pull_tight", result ) );

                fromCornerNo = currentCornerNo;
                continue;
            }

            if( currentCornerNo != runLastNode )
            {
                // An insertion which is valid only with compensated shapes
                // can fail its real-shape spring-over at the next corner. The
                // source retries on the next loop iteration with more distant
                // corners, and on the first correction includes one already
                // accepted approach edge as well.
                if( fromCornerNo > runFirstNode
                    && currentCornerNo == fromCornerNo + 1 )
                {
                    --fromCornerNo;
                }
                continue;
            }

            if( autorouterDebugEnabled() )
            {
                std::ostringstream log;
                log << "INSERT_SPAN net=" << aConnection.netCode
                    << " from_corner=" << fromCornerNo
                    << " to_corner=" << currentCornerNo
                    << " accepted_by=none final=true";
                autorouterDebugLog( log.str() );
            }
            outcome.blockedEdge = currentCornerNo;
            return outcome;
        }

        edge = runLastEdge + 1;
    }

    // Keep the historical inherited-style representation when no operation
    // changed it.  This avoids publishing an otherwise identical replacement
    // solely because the reconstruction needed explicit temporary styles.
    if( aConnection.edgeStyles.empty()
        && std::all_of( result.edgeStyles.begin(), result.edgeStyles.end(),
                        []( const ROUTING_EDGE_STYLE& aStyle )
                        { return aStyle == ROUTING_EDGE_STYLE{}; } ) )
    {
        result.edgeStyles.clear();
    }

    outcome.connection = std::move( result );
    return outcome;
}


std::optional<std::vector<FOUND_CONNECTION_INSERTER::RESULT::SHOVED_CONNECTION>>
tryShoveGeneratedConnections( const ROUTING_CONNECTION& aCandidate,
                              const std::vector<ROUTING_CONNECTION>& aVictims,
                              ROUTING_OCCUPANCY& aOccupancy,
                              const MAZE_SEARCH_ENGINE& aEngine,
                              const ROUTER_CANCEL_CALLBACK& aCancel,
                              const ROUTING_SHOVE_DIRECTION* aShoveDirection )
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
        if( auto moved = aEngine.SpringOverConnection(
                    aVictim, obstacles, aCancel, aShoveDirection ) )
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
        const ROUTER_CANCEL_CALLBACK& requestedCancel, bool allowRipupFallback,
        const ROUTING_SHOVE_DIRECTION* shoveDirection )
{
    if( autorouterDebugEnabled() )
        autorouterDebugLog( formatConnectionForDebug( "input", connection ) );

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

    ROUTING_CONNECTION sourceOrdered = connection;
    if( sourceOrdered.insertionBacktracksFromTarget )
    {
        reverseConnection( sourceOrdered );
        // FoundConnectionLocator emits the corridor in backtrack order.  The
        // Java inserter materialises its PolylineTrace in the opposite
        // (target-to-start insertion) order and that order subsequently
        // defines the item's shape indices and obstacle-room IDs.  Once the
        // native proposal has been canonicalised it is ordinary board copper;
        // retaining this flag would reverse it again during optimization.
        sourceOrdered.insertionBacktracksFromTarget = false;
    }

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
    if( sourceOrdered.nodes.size() == 1
        && !engine.CanInsertSegment( sourceOrdered.netCode, sourceOrdered.nodes[0],
                                     sourceOrdered.nodes[0] ) )
        return { STATE::BLOCKED };
    std::optional<ROUTING_CONNECTION> replacement;
    std::size_t blockedEdge = 0;
    if( sourceOrdered.nodes.size() > 1 )
    {
        INCREMENTAL_INSERTION rebuilt = buildIncrementalConnection( sourceOrdered, engine, cancel );
        if( rebuilt.cancelled )
            return { STATE::CANCELLED, rebuilt.blockedEdge };
        if( !rebuilt.connection )
            return { STATE::BLOCKED, rebuilt.blockedEdge };

        if( autorouterDebugEnabled() )
            autorouterDebugLog( formatConnectionForDebug( "rebuilt", *rebuilt.connection ) );

        blockedEdge = rebuilt.blockedEdge;
        if( !SameRouteGeometry( *rebuilt.connection, sourceOrdered )
            || !rebuilt.connection->traceInsertionSteps.empty() )
            replacement = std::move( rebuilt.connection );
    }

    const ROUTING_CONNECTION& candidate = replacement ? *replacement : sourceOrdered;
    const bool canonicalOrderChanged = connection.insertionBacktracksFromTarget;
    const auto publishedConnection = [&]() -> std::optional<ROUTING_CONNECTION>
    {
        if( replacement )
            return *replacement;
        if( canonicalOrderChanged )
            return sourceOrdered;
        return std::nullopt;
    };
    if( autorouterDebugEnabled() )
        autorouterDebugLog( formatConnectionForDebug( "candidate", candidate ) );
    if( !ripups.empty() )
    {
        if( auto shoved = tryShoveGeneratedConnections(
                    candidate, ripups, occupancy, engine, cancel,
                    shoveDirection ) )
        {
            if( cancel && cancel() )
                return { STATE::CANCELLED };

            RESULT result{ STATE::INSERTED, 0, publishedConnection() };
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
    return { STATE::INSERTED, 0, publishedConnection() };
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

        AppendEdge( aConnection.netCode, previous, current, trackWidth, viaDiameter, viaDrill, viaLayers,
                    style.viaLayerGeometry, aResult, std::max<std::int64_t>( 0, style.clearance ), style.viaType );
    }
}


void FOUND_CONNECTION_INSERTER::AppendEdge( int aNetCode, const ROUTER_NODE& aPrevious, const ROUTER_NODE& aCurrent,
                                            std::int64_t aTrackWidth, std::int64_t aViaDiameter, std::int64_t aViaDrill,
                                            const std::vector<int>&                        aViaLayers,
                                            const std::vector<ROUTING_VIA_LAYER_GEOMETRY>& aViaLayerGeometry,
                                            ROUTING_RESULT& aResult, std::int64_t aClearance, ROUTER_VIA_TYPE aViaType )
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
    via.layerGeometry = aViaLayerGeometry;
    aResult.vias.push_back( std::move( via ) );
}

} // namespace KICAD_AUTOROUTER
