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
#include <memory>
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


std::int64_t routeWidth( const BOARD_SNAPSHOT& aBoard, const ROUTING_NET& aNet )
{
    std::int64_t width = 0;

    for( std::size_t padIndex : aNet.padIndices )
    {
        if( padIndex < aBoard.pads.size() )
            width = std::max( width, aBoard.pads[padIndex].trackWidth );
    }

    return width > 0 ? width : 150000;
}


bool fanoutEscapeFitsEnvelope( const ROUTING_CONNECTION& aConnection,
                               const ROUTING_PAD& aSource,
                               const ROUTING_PAD& aLanding )
{
    if( !aLanding.isFanoutTarget || aLanding.fanoutSourceLayer < 0
        || aLanding.fanoutTargetLayer < 0 || aConnection.nodes.size() < 2 )
    {
        return true;
    }

    const std::int64_t minimum = std::max<std::int64_t>( 0,
                                                           aLanding.fanoutMinEscapeLength );
    const std::int64_t maximum = aLanding.fanoutMaxEscapeLength > 0
            ? aLanding.fanoutMaxEscapeLength
            : std::numeric_limits<std::int64_t>::max();
    // A malformed or contradictory synthetic target must not turn an
    // impossible source envelope into a wider one.  This matches the
    // independent min-drill and max-start-room filters in Freerouting's
    // MazeSearchEngine.
    if( maximum < minimum )
        return false;
    bool sawTransition = false;

    for( std::size_t index = 0; index < aConnection.nodes.size(); ++index )
    {
        const ROUTER_NODE& node = aConnection.nodes[index];
        if( !sawTransition && node.layer == aLanding.fanoutSourceLayer
            && distance( node.point, aSource.position ) > static_cast<double>( maximum ) )
        {
            return false;
        }

        if( index == 0 || aConnection.nodes[index - 1].layer == node.layer )
            continue;

        if( !sawTransition )
        {
            const ROUTER_NODE& prior = aConnection.nodes[index - 1];
            if( prior.point != node.point
                || prior.layer != aLanding.fanoutSourceLayer )
            {
                return false;
            }

            const double drillDistance = distance( prior.point, aSource.position );
            if( drillDistance < static_cast<double>( minimum )
                || drillDistance > static_cast<double>( maximum ) )
            {
                return false;
            }
            sawTransition = true;
        }
    }

    // A synthetic landing on a different layer must be reached through its
    // fanout via.  Returning a same-layer path would make the later graph
    // rewrite look connected while it has no physical layer transition.
    return sawTransition || aLanding.fanoutSourceLayer == aLanding.fanoutTargetLayer;
}


void applyFanoutViaStyle( ROUTING_CONNECTION& aConnection, const ROUTING_PAD& aLanding )
{
    if( !aLanding.isFanoutTarget || aLanding.fanoutViaDiameter <= 0
        || aConnection.nodes.size() < 2 )
    {
        return;
    }

    EnsureEdgeStyles( aConnection );
    for( std::size_t index = 1; index < aConnection.nodes.size(); ++index )
    {
        if( aConnection.nodes[index - 1].layer == aConnection.nodes[index].layer )
            continue;

        ROUTING_EDGE_STYLE& style = aConnection.edgeStyles[index - 1];
        style.viaDiameter = aLanding.fanoutViaDiameter;
        style.viaDrill = aLanding.fanoutViaDrill;
    }
}


/**
 * Reconstruct the deliberately small, safe subset of pre-existing KiCad
 * copper that can participate in a source-style forced shove.
 *
 * `allowRipupExisting` makes the adapter omit routable source copper from the
 * immutable obstacle list so a complete proposal can replace it.  That must
 * not make the copper invisible while the batch search is in progress: it is
 * still a real collision surface and, in Freerouting, an unlocked trace/via
 * may be moved rather than deleted.  The snapshot records direct trace/via
 * pieces by UUID.  Here we rebuild:
 *
 * - a single straight trace as a static route whose endpoints stay fixed;
 * - each uniform drilled via as its own static route.  Its normal endpoint
 *   trace contacts remain separate source items so a later
 *   `ShoveViaConnectionPlan` can reproduce `DrillItem.moveBy()`: retain
 *   each trace and add one bridge per contacted layer.
 *
 * Anything with an interior/non-normal contact, pad-attached drill, arc,
 * non-uniform padstack, or unsupported shape remains a static obstacle and is
 * never destructively ripped up by forced insertion.  This maps the source's
 * `DrillItemMover` normal-contact guard instead of pretending a UUID alone is
 * enough to mutate arbitrary host topology.
 */
std::vector<ROUTING_CONNECTION> existingMovableConnections(
        const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings )
{
    struct EXISTING_ATOM
    {
        enum class KIND { TRACE, VIA };

        KIND               kind = KIND::TRACE;
        ROUTING_CONNECTION connection;
    };

    std::map<std::string, std::vector<const ROUTING_OBSTACLE*>> byItem;
    for( const ROUTING_OBSTACLE& obstacle : aBoard.removableExistingRoutes )
    {
        if( obstacle.isExistingRoute && obstacle.isMovable && !obstacle.boardItemId.empty() )
            byItem[obstacle.boardItemId].push_back( &obstacle );
    }

    const auto layerOrder = [&]( int aLayer )
    {
        const auto found = std::find_if(
                aSettings.layers.begin(), aSettings.layers.end(),
                [&]( const ROUTER_LAYER_SETTINGS& aLayerSetting )
                { return aLayerSetting.layerId == aLayer; } );
        if( found == aSettings.layers.end() )
            return std::pair{ std::numeric_limits<int>::max(), aLayer };
        const int index = static_cast<int>( std::distance( aSettings.layers.begin(), found ) );
        return std::pair{ found->layerOrdinal >= 0 ? found->layerOrdinal : index, aLayer };
    };

    std::vector<EXISTING_ATOM> atoms;
    atoms.reserve( byItem.size() );
    for( const auto& [id, pieces] : byItem )
    {
        std::vector<const ROUTING_OBSTACLE*> copper;
        const ROUTING_OBSTACLE* hole = nullptr;
        bool valid = true;

        for( const ROUTING_OBSTACLE* piece : pieces )
        {
            if( piece->kind != ROUTER_OBSTACLE_KIND::SEGMENT )
            {
                valid = false;
                break;
            }
            if( piece->isHole )
            {
                if( hole || piece->radius <= 0 )
                {
                    valid = false;
                    break;
                }
                hole = piece;
            }
            else
            {
                copper.push_back( piece );
            }
        }

        if( !valid || copper.empty() )
            continue;

        ROUTING_CONNECTION connection;
        connection.complete = true;
        connection.isExistingBoardRoute = true;
        connection.isShoveMovable = false;
        connection.sourceBoardItemIds = { id };
        connection.netCode = copper.front()->netCode;

        // A direct PCB_TRACK is represented by exactly one capsule.  Do not
        // turn a tessellated arc or a compound shape into a chord.
        if( copper.size() == 1 && !hole && copper.front()->start != copper.front()->end
            && copper.front()->layers.size() == 1 && copper.front()->radius > 0 )
        {
            const ROUTING_OBSTACLE& trace = *copper.front();
            connection.nodes = { { trace.start, trace.layers.front() },
                                 { trace.end, trace.layers.front() } };
            ROUTING_EDGE_STYLE traceStyle;
            traceStyle.trackWidth = 2 * trace.radius;
            traceStyle.clearance = std::max<std::int64_t>( 0, trace.clearance );
            connection.edgeStyles = { std::move( traceStyle ) };
            atoms.push_back( { EXISTING_ATOM::KIND::TRACE, std::move( connection ) } );
            continue;
        }

        // A uniform drilled via becomes one layer transition.  The adapter
        // marks non-uniform padstacks as non-movable, but revalidate the
        // captured pieces here because this function is also used by
        // data-only regression tests.
        const ROUTER_POINT position = copper.front()->start;
        std::vector<int> layers;
        std::int64_t diameter = 0;
        std::int64_t clearance = 0;
        bool via = hole != nullptr && copper.size() >= 2;
        for( const ROUTING_OBSTACLE* piece : copper )
        {
            if( piece->start != position || piece->end != position
                || piece->layers.size() != 1 || piece->radius <= 0
                || piece->netCode != connection.netCode )
            {
                via = false;
                break;
            }
            const std::int64_t currentDiameter = 2 * piece->radius;
            if( diameter != 0 && diameter != currentDiameter )
            {
                via = false;
                break;
            }
            diameter = currentDiameter;
            clearance = std::max( clearance, std::max<std::int64_t>( 0, piece->clearance ) );
            layers.push_back( piece->layers.front() );
        }

        std::sort( layers.begin(), layers.end(), [&]( int aLeft, int aRight )
        { return layerOrder( aLeft ) < layerOrder( aRight ); } );
        layers.erase( std::unique( layers.begin(), layers.end() ), layers.end() );
        if( !via || !hole || hole->start != position || hole->end != position
            || hole->radius <= 0 || layers.size() < 2 )
        {
            continue;
        }

        ROUTING_EDGE_STYLE viaStyle;
        viaStyle.viaDiameter = diameter;
        viaStyle.viaDrill = 2 * hole->radius;
        viaStyle.viaLayers = layers;
        viaStyle.clearance = clearance;
        connection.nodes = { { position, layers.front() }, { position, layers.back() } };
        connection.edgeStyles = { std::move( viaStyle ) };
        atoms.push_back( { EXISTING_ATOM::KIND::VIA, std::move( connection ) } );
    }

    const auto onInterior = []( const ROUTER_POINT& aPoint, const ROUTER_POINT& aStart,
                                const ROUTER_POINT& aEnd )
    {
        if( aPoint == aStart || aPoint == aEnd || aStart == aEnd )
            return false;
        const long double dx = static_cast<long double>( aEnd.x ) - aStart.x;
        const long double dy = static_cast<long double>( aEnd.y ) - aStart.y;
        const long double px = static_cast<long double>( aPoint.x ) - aStart.x;
        const long double py = static_cast<long double>( aPoint.y ) - aStart.y;
        const long double cross = dx * py - dy * px;
        if( std::fabs( cross ) > 0.5L )
            return false;
        const long double dot = px * dx + py * dy;
        const long double lengthSquared = dx * dx + dy * dy;
        return dot > 0.0L && dot < lengthSquared;
    };

    const auto hasNormalTraceEndpoint = []( const ROUTER_POINT& aPoint,
                                             const ROUTER_POINT& aStart,
                                             const ROUTER_POINT& aEnd )
    {
        // DrillItem.getNormalContacts() deliberately does not turn an
        // arbitrary through-trace crossing into a normal contact.  A moved
        // DrillItem only creates a bridge for a Trace whose first or last
        // corner is exactly its old centre.  Treating every centreline
        // crossing as a contact here used to manufacture a bridge to copper
        // that Java leaves untouched, and made a host via appear movable in
        // a topology that DrillItemMover rejects.
        return aPoint == aStart || aPoint == aEnd;
    };

    const auto isOnLayer = []( const std::vector<int>& aLayers, int aLayer )
    {
        return std::find( aLayers.begin(), aLayers.end(), aLayer ) != aLayers.end();
    };

    const auto hasAreaOnLayer = [&]( int aNetCode, int aLayer )
    {
        // We have not yet ported the full mutable ConductionArea contact
        // update from DrillItemMover.  Never move an existing trace/via on a
        // plane layer rather than risking disconnecting an island.
        return std::any_of( aBoard.conductionAreas.begin(), aBoard.conductionAreas.end(),
                            [&]( const ROUTING_OBSTACLE& aArea )
                            {
                                return aArea.netCode == aNetCode
                                       && isOnLayer( aArea.layers, aLayer );
                            } );
    };

    const auto padTouchesTraceInterior = [&]( const ROUTING_PAD& aPad,
                                               const ROUTING_CONNECTION& aTrace )
    {
        if( aTrace.nodes.size() != 2 || aTrace.nodes.front().layer != aTrace.nodes.back().layer
            || aPad.position == aTrace.nodes.front().point
            || aPad.position == aTrace.nodes.back().point )
        {
            return false;
        }

        const ROUTER_POINT& start = aTrace.nodes.front().point;
        const ROUTER_POINT& end = aTrace.nodes.back().point;
        const long double dx = static_cast<long double>( end.x ) - start.x;
        const long double dy = static_cast<long double>( end.y ) - start.y;
        const long double lengthSquared = dx * dx + dy * dy;
        if( lengthSquared <= 0.0L )
            return false;

        const long double px = static_cast<long double>( aPad.position.x ) - start.x;
        const long double py = static_cast<long double>( aPad.position.y ) - start.y;
        const long double factor = ( px * dx + py * dy ) / lengthSquared;
        if( factor <= 0.0L || factor >= 1.0L )
            return false;

        const long double nearestX = static_cast<long double>( start.x ) + factor * dx;
        const long double nearestY = static_cast<long double>( start.y ) + factor * dy;
        const long double deltaX = static_cast<long double>( aPad.position.x ) - nearestX;
        const long double deltaY = static_cast<long double>( aPad.position.y ) - nearestY;
        const ROUTING_EDGE_STYLE& style = aTrace.edgeStyles.front();
        const std::int64_t traceRadius = std::max<std::int64_t>( 0, style.trackWidth / 2 );
        const std::int64_t contactRadius = traceRadius + std::max<std::int64_t>( 0, aPad.radius );
        return deltaX * deltaX + deltaY * deltaY
               <= static_cast<long double>( contactRadius ) * contactRadius;
    };

    const auto sourceGeometryTouchesTraceInterior =
            [&]( const ROUTING_OBSTACLE& aObstacle, const ROUTING_CONNECTION& aTrace )
    {
        // A source trace is reconstructed as one independently movable
        // worker capsule.  That is only sound when all of its electrical
        // topology is represented by its two retained endpoints.  KiCad does
        // not require a track to be split at a via or another track contact,
        // however, and a locked/unsupported source item is deliberately not
        // turned into an EXISTING_ATOM above.  If it touches the open interior
        // of this trace, springing the trace would retain neither the contact
        // nor an equivalent bridge.  Keep the trace fixed instead.
        //
        // This is intentionally about copper geometry, rather than the
        // source's `getNormalContacts()` graph.  The latter expects imported
        // routes to have already been normalized at every contact; the KiCad
        // snapshot can contain legitimate unsplit host geometry.  Exact
        // endpoint contacts remain allowed below because their endpoint stays
        // in place when a trace is springed.
        if( !aObstacle.isExistingRoute || aObstacle.isHole || aObstacle.isKeepout
            || aTrace.nodes.size() != 2
            || aTrace.nodes.front().layer != aTrace.nodes.back().layer
            || aTrace.edgeStyles.empty()
            || aObstacle.netCode != aTrace.netCode
            || !isOnLayer( aObstacle.layers, aTrace.nodes.front().layer )
            || ( !aTrace.sourceBoardItemIds.empty()
                 && aObstacle.boardItemId == aTrace.sourceBoardItemIds.front() ) )
        {
            return false;
        }

        const ROUTER_POINT& traceStart = aTrace.nodes.front().point;
        const ROUTER_POINT& traceEnd = aTrace.nodes.back().point;
        const long double traceDx = static_cast<long double>( traceEnd.x ) - traceStart.x;
        const long double traceDy = static_cast<long double>( traceEnd.y ) - traceStart.y;
        const long double traceLengthSquared = traceDx * traceDx + traceDy * traceDy;
        if( traceLengthSquared <= 0.0L )
            return true;

        const std::int64_t traceRadius = std::max<std::int64_t>(
                0, aTrace.edgeStyles.front().trackWidth / 2 );

        const auto pointTouchesOpenInterior = [&]( const ROUTER_POINT& aPoint,
                                                    std::int64_t aOtherRadius )
        {
            if( aPoint == traceStart || aPoint == traceEnd )
                return false;

            const long double pointDx = static_cast<long double>( aPoint.x ) - traceStart.x;
            const long double pointDy = static_cast<long double>( aPoint.y ) - traceStart.y;
            const long double factor = ( pointDx * traceDx + pointDy * traceDy )
                                       / traceLengthSquared;
            if( factor <= 0.0L || factor >= 1.0L )
                return false;

            const long double nearestX = static_cast<long double>( traceStart.x )
                                         + factor * traceDx;
            const long double nearestY = static_cast<long double>( traceStart.y )
                                         + factor * traceDy;
            const long double deltaX = static_cast<long double>( aPoint.x ) - nearestX;
            const long double deltaY = static_cast<long double>( aPoint.y ) - nearestY;
            const long double contactRadius = static_cast<long double>( traceRadius )
                                              + std::max<std::int64_t>( 0, aOtherRadius );
            return deltaX * deltaX + deltaY * deltaY <= contactRadius * contactRadius;
        };

        if( aObstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
        {
            const std::int64_t obstacleRadius = std::max<std::int64_t>( 0,
                                                                          aObstacle.radius );
            if( pointTouchesOpenInterior( aObstacle.start, obstacleRadius )
                || pointTouchesOpenInterior( aObstacle.end, obstacleRadius ) )
            {
                return true;
            }

            const long double obstacleDx = static_cast<long double>( aObstacle.end.x )
                                           - aObstacle.start.x;
            const long double obstacleDy = static_cast<long double>( aObstacle.end.y )
                                           - aObstacle.start.y;
            const long double relativeX = static_cast<long double>( aObstacle.start.x )
                                          - traceStart.x;
            const long double relativeY = static_cast<long double>( aObstacle.start.y )
                                          - traceStart.y;
            const long double denominator = traceDx * obstacleDy - traceDy * obstacleDx;
            constexpr long double epsilon = 0.5L;

            if( std::fabs( denominator ) > epsilon )
            {
                const long double traceFactor = ( relativeX * obstacleDy
                                                  - relativeY * obstacleDx )
                                                 / denominator;
                const long double obstacleFactor = ( relativeX * traceDy
                                                     - relativeY * traceDx )
                                                    / denominator;
                // The source trace normal-contact topology already retains a
                // shared endpoint.  A proper/through intersection of the
                // centre lines is an interior attachment that cannot be
                // reconstructed by moving this one capsule.
                if( traceFactor > 0.0L && traceFactor < 1.0L
                    && obstacleFactor >= 0.0L && obstacleFactor <= 1.0L )
                {
                    return true;
                }
            }
            else
            {
                // Parallel overlapping copper is not caught by a line-line
                // intersection.  Project the obstacle onto the host trace;
                // an interval through its open interior is a real attachment
                // whenever the two swept centre lines are close enough.
                const long double collinear = relativeX * traceDy - relativeY * traceDx;
                const long double startProjection = ( relativeX * traceDx
                                                      + relativeY * traceDy )
                                                     / traceLengthSquared;
                const long double endProjection =
                        ( ( static_cast<long double>( aObstacle.end.x ) - traceStart.x )
                                  * traceDx
                          + ( static_cast<long double>( aObstacle.end.y ) - traceStart.y )
                                    * traceDy )
                        / traceLengthSquared;
                const long double overlapStart = std::max(
                        0.0L, std::min( startProjection, endProjection ) );
                const long double overlapEnd = std::min(
                        1.0L, std::max( startProjection, endProjection ) );
                const long double contactRadius = static_cast<long double>( traceRadius )
                                                  + obstacleRadius;

                if( overlapStart < overlapEnd
                    && std::fabs( collinear ) / std::sqrt( traceLengthSquared )
                               <= contactRadius )
                {
                    return true;
                }
            }

            return false;
        }

        // Existing source routes currently arrive as capsule primitives.  If
        // a future adapter supplies an arbitrary source shape, do not claim
        // its contact topology is movable just because its UUID is different.
        // Restrict that fail-closed choice to a coarse overlap with the trace
        // envelope so an unrelated same-net item elsewhere on the layer does
        // not suppress every possible source trace shove.
        ROUTER_BOX obstacleBounds = aObstacle.box;
        bool      haveBounds = aObstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE;
        const auto includePoint = [&]( const ROUTER_POINT& aPoint )
        {
            if( !haveBounds )
            {
                obstacleBounds = { aPoint.x, aPoint.y, aPoint.x, aPoint.y };
                haveBounds = true;
                return;
            }
            obstacleBounds.minX = std::min( obstacleBounds.minX, aPoint.x );
            obstacleBounds.minY = std::min( obstacleBounds.minY, aPoint.y );
            obstacleBounds.maxX = std::max( obstacleBounds.maxX, aPoint.x );
            obstacleBounds.maxY = std::max( obstacleBounds.maxY, aPoint.y );
        };
        for( const ROUTER_POINT& point : aObstacle.polygon )
            includePoint( point );
        if( !haveBounds )
            return true;

        const long double obstacleRadius = std::max<std::int64_t>( 0, aObstacle.radius );
        const long double traceMinX = static_cast<long double>(
                std::min( traceStart.x, traceEnd.x ) ) - traceRadius;
        const long double traceMinY = static_cast<long double>(
                std::min( traceStart.y, traceEnd.y ) ) - traceRadius;
        const long double traceMaxX = static_cast<long double>(
                std::max( traceStart.x, traceEnd.x ) ) + traceRadius;
        const long double traceMaxY = static_cast<long double>(
                std::max( traceStart.y, traceEnd.y ) ) + traceRadius;
        return static_cast<long double>( obstacleBounds.minX ) - obstacleRadius <= traceMaxX
               && static_cast<long double>( obstacleBounds.maxX ) + obstacleRadius >= traceMinX
               && static_cast<long double>( obstacleBounds.minY ) - obstacleRadius <= traceMaxY
               && static_cast<long double>( obstacleBounds.maxY ) + obstacleRadius >= traceMinY;
    };

    const auto viaHasSupportedNormalContacts = [&]( std::size_t aViaIndex )
    {
        if( aViaIndex >= atoms.size() || atoms[aViaIndex].kind != EXISTING_ATOM::KIND::VIA )
            return false;

        const ROUTING_CONNECTION& via = atoms[aViaIndex].connection;
        if( via.nodes.size() != 2 || via.nodes.front().point != via.nodes.back().point
            || via.nodes.front().layer == via.nodes.back().layer
            || via.edgeStyles.size() != 1 || via.sourceBoardItemIds.size() != 1 )
        {
            return false;
        }

        const ROUTER_POINT position = via.nodes.front().point;
        const ROUTING_EDGE_STYLE& style = via.edgeStyles.front();
        const std::int64_t viaRadius = std::max<std::int64_t>( 1, style.viaDiameter / 2 );
        const std::vector<int>& layers = style.viaLayers;
        if( layers.empty() )
            return false;

        // DrillItemMover rejects a drill item with a normal PAD contact.  A
        // point-only test misses the ordinary KiCad case where the via lands
        // inside a pad away from its centre, so use the captured conservative
        // pad radius.  False positives are intentionally fail-closed.
        for( const ROUTING_PAD& pad : aBoard.pads )
        {
            if( pad.isFanoutTarget || pad.isPlaneTarget || pad.netCode != via.netCode
                || !std::any_of( layers.begin(), layers.end(), [&]( int aLayer )
                   { return isOnLayer( pad.layers, aLayer ); } ) )
            {
                continue;
            }

            const long double dx = static_cast<long double>( position.x ) - pad.position.x;
            const long double dy = static_cast<long double>( position.y ) - pad.position.y;
            const long double radius = static_cast<long double>( viaRadius )
                                       + std::max<std::int64_t>( 0, pad.radius );
            if( dx * dx + dy * dy <= radius * radius )
                return false;
        }

        const auto pointToSegmentDistanceSquared = [&]( const ROUTER_POINT& aStart,
                                                         const ROUTER_POINT& aEnd )
        {
            const long double dx = static_cast<long double>( aEnd.x ) - aStart.x;
            const long double dy = static_cast<long double>( aEnd.y ) - aStart.y;
            const long double lengthSquared = dx * dx + dy * dy;
            const long double factor = lengthSquared <= 0.0L
                    ? 0.0L
                    : std::clamp(
                              ( ( static_cast<long double>( position.x ) - aStart.x ) * dx
                                + ( static_cast<long double>( position.y ) - aStart.y ) * dy )
                                      / lengthSquared,
                              0.0L, 1.0L );
            const long double nearestX = static_cast<long double>( aStart.x ) + factor * dx;
            const long double nearestY = static_cast<long double>( aStart.y ) + factor * dy;
            const long double deltaX = static_cast<long double>( position.x ) - nearestX;
            const long double deltaY = static_cast<long double>( position.y ) - nearestY;
            return deltaX * deltaX + deltaY * deltaY;
        };

        // DrillItem.moveBy() remembers a TraceInfo for every *normal* trace
        // contact, moves the via, and creates a bridge for that layer.
        // DrillItem.getNormalContacts() defines a trace contact as an exact
        // endpoint match, not an annulus overlap or a trace centreline
        // crossing. Admit exactly the source subset that can reproduce that
        // operation. A tangential or through-trace contact remains fixed
        // rather than becoming a dangling proposal stub.
        std::set<std::string> supportedTraceIds;
        for( const EXISTING_ATOM& atom : atoms )
        {
            if( atom.kind != EXISTING_ATOM::KIND::TRACE
                || atom.connection.netCode != via.netCode
                || atom.connection.nodes.size() != 2
                || atom.connection.nodes.front().layer != atom.connection.nodes.back().layer
                || atom.connection.edgeStyles.size() != 1
                || atom.connection.sourceBoardItemIds.size() != 1
                || !isOnLayer( layers, atom.connection.nodes.front().layer )
                || !hasNormalTraceEndpoint( position, atom.connection.nodes.front().point,
                                             atom.connection.nodes.back().point ) )
            {
                continue;
            }
            supportedTraceIds.insert( atom.connection.sourceBoardItemIds.front() );
        }

        const auto inspectSourceGeometry = [&]( const std::vector<ROUTING_OBSTACLE>& aObstacles )
        {
            for( const ROUTING_OBSTACLE& obstacle : aObstacles )
            {
                if( obstacle.boardItemId == via.sourceBoardItemIds.front()
                    || obstacle.netCode != via.netCode
                    || !std::any_of( layers.begin(), layers.end(), [&]( int aLayer )
                       { return isOnLayer( obstacle.layers, aLayer ); } ) )
                {
                    continue;
                }

                // An unsupported source shape on a physical via layer might
                // be a normal contact. Without its exact mutation/bridge
                // semantics the only safe answer is to keep the via fixed.
                if( obstacle.kind != ROUTER_OBSTACLE_KIND::SEGMENT )
                    return false;

                const long double radius = static_cast<long double>( viaRadius )
                                           + std::max<std::int64_t>( 0, obstacle.radius );
                if( pointToSegmentDistanceSquared( obstacle.start, obstacle.end )
                    > radius * radius )
                {
                    continue;
                }

                if( !obstacle.isHole && supportedTraceIds.contains( obstacle.boardItemId )
                    && hasNormalTraceEndpoint( position, obstacle.start, obstacle.end ) )
                {
                    continue;
                }

                // A second via/pad/hole, an arc tessellation, a locked trace,
                // or a direct trace that ends somewhere inside the annulus is
                // not represented by the source-style old-centre bridge.
                return false;
            }
            return true;
        };

        if( !inspectSourceGeometry( aBoard.removableExistingRoutes )
            || !inspectSourceGeometry( aBoard.obstacles ) )
        {
            return false;
        }

        return true;
    };

    const auto traceHasInteriorContact = [&]( std::size_t aTrace )
    {
        const ROUTING_CONNECTION& trace = atoms[aTrace].connection;
        if( trace.nodes.size() != 2 || trace.nodes.front().layer != trace.nodes.back().layer )
            return true;
        const int layer = trace.nodes.front().layer;
        if( hasAreaOnLayer( trace.netCode, layer ) )
            return true;

        for( std::size_t other = 0; other < atoms.size(); ++other )
        {
            if( other == aTrace || atoms[other].connection.netCode != trace.netCode )
                continue;
            for( const ROUTER_NODE& node : atoms[other].connection.nodes )
                if( node.layer == layer
                    && onInterior( node.point, trace.nodes.front().point, trace.nodes.back().point ) )
                {
                    return true;
                }
        }
        for( const ROUTING_PAD& pad : aBoard.pads )
        {
            if( pad.isFanoutTarget || pad.isPlaneTarget || pad.netCode != trace.netCode
                || !isOnLayer( pad.layers, layer ) )
            {
                continue;
            }
            // A KiCad track can enter a pad away from its centre.  The old
            // centreline-only probe marked that as movable, then a spring-over
            // detached an interior pad contact that Java's item topology keeps
            // fixed.  The snapshot carries a conservative pad radius, so use
            // the actual copper contact envelope while still allowing an
            // ordinary trace endpoint to remain anchored at a pad centre.
            if( padTouchesTraceInterior( pad, trace ) )
                return true;
        }

        const auto hasSourceGeometryContact = [&]( const std::vector<ROUTING_OBSTACLE>& aObstacles )
        {
            return std::any_of( aObstacles.begin(), aObstacles.end(),
                                [&]( const ROUTING_OBSTACLE& aObstacle )
                                {
                                    return sourceGeometryTouchesTraceInterior( aObstacle, trace );
                                } );
        };

        // Movable atoms cover the source subset we can reconstruct.  Inspect
        // the complete snapshot as well: a locked via/arc or an unsupported
        // source item is intentionally absent from atoms, but can still make
        // a physical same-net attachment in the interior of this trace.
        if( hasSourceGeometryContact( aBoard.removableExistingRoutes )
            || hasSourceGeometryContact( aBoard.obstacles ) )
        {
            return true;
        }
        return false;
    };

    std::vector<ROUTING_CONNECTION> result;
    result.reserve( atoms.size() );

    for( std::size_t index = 0; index < atoms.size(); ++index )
    {
        ROUTING_CONNECTION route = atoms[index].connection;

        // Do not collapse a source trace-via-trace chain into a synthetic
        // mutable polyline. DrillItem.moveBy() moves the Via, retains every
        // contacted Trace in place, and emits old-centre -> new-centre bridge
        // copper for its normal endpoint contacts. Keeping each host item as
        // its own static route lets ShoveViaConnectionPlan reproduce that
        // mutation and preserves branches/multiple same-layer contacts that
        // a two-legged composite silently discarded.
        route.isShoveMovable = ( atoms[index].kind == EXISTING_ATOM::KIND::TRACE
                                 && !traceHasInteriorContact( index ) )
                                || viaHasSupportedNormalContacts( index );
        result.push_back( std::move( route ) );
    }

    return result;
}


/**
 * Return the physical source item identities represented completely by a
 * static occupancy route.
 *
 * `removableExistingRoutes` is deliberately a collection of primitive
 * snapshot shapes: a through via has one annulus per copper layer plus a
 * drill, and an arc can have several tessellated capsules.  A static
 * ROUTING_CONNECTION may replace *all* of those shapes in the live
 * occupancy model, but only when it owns every shape for the corresponding
 * BOARD_ITEM UUID.  Every other source item must remain in the immutable
 * search obstacle set.  Otherwise an unsupported arc, a non-uniform via, or
 * a trace whose topology failed the fail-closed reconstruction becomes
 * invisible during a whole-net reroute.
 */
std::set<std::string> representedExistingItemIds(
        const std::vector<ROUTING_CONNECTION>& aConnections )
{
    std::set<std::string> result;

    for( const ROUTING_CONNECTION& connection : aConnections )
    {
        if( !connection.isExistingBoardRoute )
            continue;

        result.insert( connection.sourceBoardItemIds.begin(),
                       connection.sourceBoardItemIds.end() );
    }

    return result;
}


/**
 * Add source copper that has no live static-route representation back to the
 * immutable obstacle model.
 *
 * This helper is used twice with different scopes:
 *
 * - the fanout pre-pass sees every removable source item as fixed copper;
 *   current native fanout planning has no host-item shove transaction;
 * - the ordinary batch stage sees only items that could not be represented
 *   by a complete static connection.  Represented items instead participate
 *   in occupancy conflict discovery and can be promoted to proposal copper
 *   only after a checked shove.
 *
 * Existing routes in `aSnapshot.obstacles` are never removed here.  Their
 * UUID cannot also occur in `removableExistingRoutes`, because the adapter
 * places each BOARD_ITEM in exactly one collection.
 */
void appendUnrepresentedExistingObstacles( BOARD_SNAPSHOT& aSnapshot,
                                           const std::set<std::string>& aRepresentedIds )
{
    for( ROUTING_OBSTACLE& obstacle : aSnapshot.removableExistingRoutes )
    {
        if( !obstacle.isExistingRoute || obstacle.boardItemId.empty()
            || aRepresentedIds.contains( obstacle.boardItemId ) )
        {
            continue;
        }

        // Keep the canonical removable record for output ownership, but make
        // the immutable-search copy collision-only so ROUTING_BOARD cannot
        // accidentally use old copper to satisfy a full-reroute task.
        ROUTING_OBSTACLE collisionOnly = obstacle;
        collisionOnly.isCollisionOnly = true;
        aSnapshot.obstacles.push_back( std::move( collisionOnly ) );
        obstacle.isMirroredToObstacleModel = true;
    }
}


/** Add every removable source shape to a planning-only immutable snapshot. */
void appendAllRemovableExistingObstacles( BOARD_SNAPSHOT& aSnapshot )
{
    // This input is planning-only, so avoid setting the DRC mirror bit on
    // its canonical source records.  BatchFanout itself never constructs a
    // ROUTING_BOARD, but marking copies collision-only documents and
    // preserves the invariant should that change later.
    for( const ROUTING_OBSTACLE& obstacle : aSnapshot.removableExistingRoutes )
    {
        if( !obstacle.isExistingRoute || obstacle.boardItemId.empty() )
            continue;

        ROUTING_OBSTACLE collisionOnly = obstacle;
        collisionOnly.isCollisionOnly = true;
        aSnapshot.obstacles.push_back( std::move( collisionOnly ) );
    }
}


/**
 * Remove planning-only copies of source copper after BatchFanout has chosen
 * its local escapes.  The returned fanout snapshot must not retain these
 * copies: represented source items are subsequently supplied by live
 * occupancy so a forced insertion can shove them; unrepresented items are
 * appended once by appendUnrepresentedExistingObstacles().
 */
void removePlanningOnlyExistingObstacles( BOARD_SNAPSHOT& aSnapshot,
                                          const BOARD_SNAPSHOT& aOriginal )
{
    std::set<std::string> removableIds;
    for( const ROUTING_OBSTACLE& obstacle : aOriginal.removableExistingRoutes )
    {
        if( obstacle.isExistingRoute && !obstacle.boardItemId.empty() )
            removableIds.insert( obstacle.boardItemId );
    }

    aSnapshot.obstacles.erase(
            std::remove_if( aSnapshot.obstacles.begin(), aSnapshot.obstacles.end(),
                            [&]( const ROUTING_OBSTACLE& obstacle )
                            {
                                return obstacle.isExistingRoute
                                       && !obstacle.boardItemId.empty()
                                       && removableIds.contains( obstacle.boardItemId );
                            } ),
            aSnapshot.obstacles.end() );
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
    std::size_t                     newConnectionCount = 0;
    bool                            newConnectionsFlushed = false;
    const auto flushNewConnections = [&]()
    {
        if( newConnectionsFlushed )
            return;

        newConnectionCount = newConnections.size();
        aConnections.insert( aConnections.end(),
                             std::make_move_iterator( newConnections.begin() ),
                             std::make_move_iterator( newConnections.end() ) );
        newConnectionsFlushed = true;
    };
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
        {
            // Per-pin fanout budgets use the regular cancellation plumbing.
            // Keep all previously committed worker routes visible to the
            // caller before returning so occupancy, emitted geometry and the
            // subsequent ordinary batch stage never diverge.
            flushNewConnections();
            return false;
        }

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
        {
            flushNewConnections();
            return false;
        }

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
            // Fanout retains its explicit escape transition. Plane routing
            // starts at the entire connected set (AutorouteConnectionRouter),
            // not just the chosen synthetic landing on one layer.
            const bool fanoutTask = target.isFanoutTarget
                                    && target.fanoutSourcePadIndex == routeSourceIndex;
            if( !fanoutTask && targetIsPlane && !source.isExactTarget )
            {
                // Post-refill repair can request an exact island anchor. That
                // host-only subproblem must retain its specified start point;
                // it is not a general connected-set search request.
                starts = aOccupancy.Board()->Terminals( routeSourceIndex );
            }
            else if( !fanoutTask && aNet.planeTargetIndices.empty() )
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
                {
                    flushNewConnections();
                    return false;
                }

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

                // This is distinct from a pin-entry neckdown.  Freerouting's
                // AutorouteConnectionRouter retries a complete failed search
                // at the configured neck width, so a narrow corridor can be
                // discovered by the maze itself rather than only after a
                // normal-width path reaches a terminal pad.  The retry owns a
                // separate immutable engine: its spatial padding, room
                // clearance predicates, output edge styles, and strict
                // insertion all agree on the selected width.
                int neckExpanded = 0;
                const std::int64_t normalWidth = routeWidth( aBoard, aNet );
                const std::int64_t neckWidth = std::max<std::int64_t>(
                        0, aSettings.neckWidthIU );
                if( !connection && neckWidth > 0 && neckWidth < normalWidth
                    && !( aCancel && aCancel() ) )
                {
                    const int expandedBeforeNeckSearch = aExpandedNodes;
                    const ROUTER_SEARCH_PROGRESS_CALLBACK neckSearchProgress =
                            [&]( int aSearchExpanded )
                    {
                        if( aSearchProgress )
                            aSearchProgress( expandedBeforeNeckSearch + neckExpanded
                                             + aSearchExpanded );
                    };
                    AUTOROUTE_ENGINE neckEngine( aBoard, aSettings, aOccupancy, 0,
                                                  std::nullopt, aNet.netCode, neckWidth );
                    connection = neckEngine.AutorouteConnection(
                            source, target, aRetry + iteration, neckExpanded, aCancel,
                            neckSearchProgress, starts, destinations );
                    aExpandedNodes += neckExpanded;
                }

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
                            << " expanded=" << expanded << " neckExpanded=" << neckExpanded
                            << " neckWidth=" << neckWidth << " elapsed=" << elapsed << " ms";
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
        const ROUTING_PAD* fanoutSource = nullptr;
        const ROUTING_PAD* fanoutLanding = nullptr;
        if( target.isFanoutTarget && target.fanoutSourcePadIndex == routeSourceIndex )
        {
            fanoutSource = &source;
            fanoutLanding = &target;
        }
        else if( source.isFanoutTarget && source.fanoutSourcePadIndex == routeTargetIndex )
        {
            // The connected-component walk may legally choose the synthetic
            // landing as its source. A fanout bridge is directional only in
            // its local geometry; its selected via profile and envelope are
            // still properties of the same source-pad/landing pair.
            fanoutSource = &target;
            fanoutLanding = &source;
        }
        connection->isFanoutConnection = fanoutSource != nullptr;

        if( connection->isFanoutConnection )
        {
            if( !fanoutEscapeFitsEnvelope( *connection, *fanoutSource, *fanoutLanding ) )
            {
                if( autorouterDebugEnabled() )
                    autorouterDebugLog( "fanout escape rejected outside configured envelope" );
                continue;
            }

            // A board-via fallback is selected only for this synthetic
            // fanout edge.  Carry it as per-edge style before both conflict
            // discovery and strict insertion so search, DRC and emitted
            // KiCad geometry see exactly the same padstack.
            applyFanoutViaStyle( *connection, *fanoutLanding );
        }

        // Ripup and insertion are ONE speculative edit. A late blocked edge,
        // invalid via, cancellation or exception must not lose earlier routes.
        const auto conflicts = aSettings.allowRipupRouted
                ? aEngine.FindConflictingConnections( *connection )
                : std::vector<ROUTING_CONNECTION>{};
        const std::size_t remainingRipups = static_cast<std::size_t>(
                std::max( 0, aSettings.maxRipups - aRipups ) );
        // A source-style forced shove preserves every victim instead of
        // consuming a negotiated-congestion rip-up. Let insertion attempt
        // that transactional move even when ordinary deletions are exhausted;
        // only the fallback that would discard a victim remains forbidden.
        const bool allowRipupFallback = conflicts.size() <= remainingRipups;
        const auto inserted = aEngine.InsertConnection(
                *connection, conflicts, aOccupancy, aCancel, allowRipupFallback );
        if( inserted.state != FOUND_CONNECTION_INSERTER::STATE::INSERTED )
        {
            if( autorouterDebugEnabled() )
            {
                std::ostringstream message;
                message << "insertion rejected net=" << aNet.netCode
                        << " state=" << static_cast<int>( inserted.state )
                        << " edge=" << inserted.edge;
                autorouterDebugLog( message.str() );
            }
            continue;
        }
        if( inserted.connection )
            *connection = *inserted.connection;

        // Recursive forced insertion may relocate a generated route that was
        // not in the candidate's original conflict set.  Publish *every*
        // transactional replacement before accounting for ordinary rip-ups;
        // otherwise the worker occupancy and the emitted proposal diverge.
        for( const auto& shove : inserted.shoved )
        {
            const auto replace = [&]( auto& aRoutes )
            {
                for( auto& route : aRoutes )
                    if( SameRouteGeometry( route, shove.original ) )
                        route = shove.replacement;
            };
            replace( aConnections );
            replace( newConnections );

            // DrillItem.moveBy() leaves its contacted traces in place and
            // adds a bridge to the moved via. In a proposal worker those
            // source BOARD_ITEMs must instead be re-emitted explicitly: swap
            // each static collision record for its materialised equivalent,
            // then retain every newly generated bridge in the global route
            // set. Do not put bridges in newConnections: their source net can
            // differ from the currently routed net, while aConnections owns
            // all cross-net proposal geometry.
            for( const ROUTING_CONNECTION_REPLACEMENT& contact : shove.materializedContacts )
            {
                const auto materialize = [&]( auto& aRoutes )
                {
                    for( auto& route : aRoutes )
                        if( SameRouteGeometry( route, contact.original ) )
                            route = contact.replacement;
                };
                materialize( aConnections );
                materialize( newConnections );
            }
            for( const ROUTING_CONNECTION& bridge : shove.bridges )
            {
                const bool alreadyPublished = std::any_of(
                        aConnections.begin(), aConnections.end(),
                        [&]( const ROUTING_CONNECTION& aRoute )
                        { return SameRouteGeometry( aRoute, bridge ); } )
                                              || std::any_of(
                                                      newConnections.begin(), newConnections.end(),
                                                      [&]( const ROUTING_CONNECTION& aRoute )
                                                      { return SameRouteGeometry( aRoute, bridge ); } );
                if( !alreadyPublished )
                    aConnections.push_back( bridge );
            }
        }

        for( const auto& conflict : conflicts )
        {
            const auto shoved = std::find_if(
                    inserted.shoved.begin(), inserted.shoved.end(),
                    [&]( const FOUND_CONNECTION_INSERTER::RESULT::SHOVED_CONNECTION& aShove )
                    { return SameRouteGeometry( aShove.original, conflict ); } );

            if( shoved != inserted.shoved.end() )
                continue;

            auto matches = [&]( const auto& route )
            { return SameRouteGeometry( route, conflict ); };
            std::erase_if( aConnections, matches );
            std::erase_if( newConnections, matches );
            ++aRipups;
        }

        newConnections.push_back( std::move( *connection ) );

        refreshContacts();
    }

    flushNewConnections();

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
        // A static host route is retained only for occupancy/history while a
        // full-net reroute is planned.  It has neither been moved nor
        // regenerated, so emitting it would duplicate source copper in the
        // proposal.  Its UUID is preserved on the route for a later checked
        // replacement; unchanged static copper is deliberately not output.
        if( connection.isExistingBoardRoute )
            continue;
        if( connection.isFanoutConnection )
            ++aResult.metrics.fanoutConnections;

        // The path layer owns route-to-geometry materialization.  KiCad board
        // objects are still not created here; the editor adapter does that
        // only after the proposal is accepted.  Keep individual edge styles
        // intact here: a terminal neckdown's width/clearance and a selected
        // blind-via stack must reach both the proposal DRC and the adapter.
        FOUND_CONNECTION_INSERTER::Append(
                connection, netWidths[connection.netCode], netViaDiameters[connection.netCode],
                netViaDrills[connection.netCode], VIA_RULE::ThroughLayers( aSettings ), aResult );
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
        const auto duplicate = std::find_if(
                uniqueVias.begin(), uniqueVias.end(),
                [&]( const ROUTING_VIA& existing )
                {
                    return existing.netCode == via.netCode && existing.position == via.position
                           && existing.topLayer == via.topLayer
                           && existing.bottomLayer == via.bottomLayer
                           && existing.diameter == via.diameter && existing.drill == via.drill
                           && existing.layers == via.layers;
                } );
        if( duplicate == uniqueVias.end() )
            uniqueVias.push_back( std::move( via ) );
        else
            duplicate->clearance = std::max( duplicate->clearance, via.clearance );
    }
    aResult.vias = std::move( uniqueVias );

    std::set<std::string> removed;
    std::set<int> completeNets;
    ROUTING_BOARD copper( aBoard, aSettings );
    for( const auto& connection : aConnections )
        if( !connection.isExistingBoardRoute )
            copper.AddRoute( connection );
    for( const ROUTING_NET& net : aBoard.nets )
    {
        if( !net.connections.empty()
            && copper.CountMissing( net ) == 0 )
        {
            completeNets.insert( net.netCode );
        }
    }

    // A checked forced shove can safely replace a source item even when the
    // rest of that source net is still incomplete.  Removing source UUIDs
    // only after a whole net completed used to emit the moved via/trace while
    // retaining its original BOARD_ITEM, producing duplicate copper in a
    // partial-but-otherwise-valid proposal.  Limit this early removal to
    // UUIDs known to come from the removable snapshot and only when the route
    // carrying that provenance was materialised as new proposal copper.
    std::set<std::string> removableSourceIds;
    for( const ROUTING_OBSTACLE& obstacle : aBoard.removableExistingRoutes )
        if( obstacle.isExistingRoute && !obstacle.boardItemId.empty() )
            removableSourceIds.insert( obstacle.boardItemId );
    for( const ROUTING_CONNECTION& connection : aConnections )
    {
        if( connection.isExistingBoardRoute || !connection.complete )
            continue;
        for( const std::string& id : connection.sourceBoardItemIds )
            if( removableSourceIds.contains( id ) )
                removed.insert( id );
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

    std::vector<ROUTING_CONNECTION> emittedConnections;
    emittedConnections.reserve( aResult.connections.size() );
    for( const ROUTING_CONNECTION& connection : aResult.connections )
        if( !connection.isExistingBoardRoute )
            emittedConnections.push_back( connection );
    aResult.metrics.airlineLengthIU = AUTOROUTE_AIRLINE_CALCULATOR::TotalLength(
            aBoard, emittedConnections );
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
    // KiCad board remains untouched until the proposal is accepted.  Keep a
    // stage-local deadline separate from user cancellation: Freerouting
    // stops fanout at its deadline and continues ordinary autorouting, while
    // a user cancellation must still discard the complete proposal.
    const auto fanoutStarted = std::chrono::steady_clock::now();
    const std::optional<std::chrono::steady_clock::time_point> fanoutDeadline =
            aSettings.fanoutTimeoutMilliseconds > 0
                    ? std::optional{ fanoutStarted
                                     + std::chrono::milliseconds(
                                             aSettings.fanoutTimeoutMilliseconds ) }
                    : std::nullopt;
    bool fanoutTimedOut = false;
    const ROUTER_CANCEL_CALLBACK fanoutStageCancel = [&]()
    {
        if( aCancel && aCancel() )
            return true;

        if( fanoutDeadline && std::chrono::steady_clock::now() >= *fanoutDeadline )
        {
            fanoutTimedOut = true;
            return true;
        }

        return false;
    };

    // Fanout does not yet own a host-item shove transaction.  Give its
    // planning snapshot every removable source shape as a fixed collision
    // surface, rather than allowing a local escape to be selected through an
    // unlocked arc, custom via, or trace branch that the later batch stage
    // correctly refuses to move.  The planning-only copies are stripped
    // immediately after the pre-pass; represented source items then enter
    // live occupancy and unsupported ones are appended exactly once below.
    BOARD_SNAPSHOT fanoutInput = aBoard;
    appendAllRemovableExistingObstacles( fanoutInput );
    BOARD_SNAPSHOT routedBoard = BATCH_FANOUT::PrepareSnapshot( fanoutInput, aSettings,
                                                                  fanoutStageCancel );

    if( !fanoutTimedOut && fanoutDeadline
        && std::chrono::steady_clock::now() >= *fanoutDeadline )
    {
        fanoutTimedOut = true;
    }

    // Preparation only plans synthetic terminals.  If it timed out partway
    // through, discard that partial graph and route the original snapshot;
    // keeping half-created landings would allow the ordinary stage to start
    // from a point without an electrical escape back to its SMD pad.
    if( fanoutTimedOut )
        routedBoard = aBoard;
    else
        removePlanningOnlyExistingObstacles( routedBoard, aBoard );

    if( autorouterDebugEnabled() )
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - fanoutStarted )
                                      .count();
        std::ostringstream message;
        message << "fanout preparation elapsed=" << elapsed << " ms pads="
                << routedBoard.pads.size() << " obstacles=" << routedBoard.obstacles.size()
                << " timedOut=" << fanoutTimedOut;
        autorouterDebugLog( message.str() );
    }

    if( aCancel && aCancel() )
    {
        result.cancelled = true;
        result.message = "Autorouter cancelled";
        return result;
    }

    if( fanoutTimedOut )
    {
        result.fanoutTimedOut = true;
        result.message = "SMD fanout preparation timed out; continuing with ordinary routing.";
    }

    // Keep the fanout-expanded snapshot mutable until the fanout pre-pass has
    // decided which synthetic landings are actually usable.  A failed
    // synthetic escape must be removed from the graph and its ordinary
    // connections must point back at the real pad; otherwise the main router
    // can route successfully from an electrically disconnected landing.
    BOARD_SNAPSHOT        workingBoard = routedBoard;
    BOARD_SNAPSHOT&       board = workingBoard;
    // Existing, unlocked host copper omitted from the immutable obstacle set
    // during a whole-net reroute still has to block the first search.  Keep a
    // fully reconstructed item static in occupancy until forced insertion
    // proves an atomic shove; anything not reconstructed remains an immutable
    // collision-only obstacle rather than silently disappearing.
    const std::vector<ROUTING_CONNECTION> staticExistingRoutes =
            existingMovableConnections( board, aSettings );
    appendUnrepresentedExistingObstacles( board,
                                          representedExistingItemIds( staticExistingRoutes ) );

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
    // AddStatic intentionally does not make source copper satisfy electrical
    // tasks.  It does, however, expose represented host routes to forced
    // insertion conflict discovery.
    for( const ROUTING_CONNECTION& route : staticExistingRoutes )
        occupancy.AddStatic( route );
    // The search engine builds the immutable obstacle/spatial index once per
    // engine.  Reuse it for every connection in a pass; reconstructing it for
    // each net makes large boards spend most of their runtime re-indexing the
    // same pads and keepouts rather than expanding paths.
    AUTOROUTE_ENGINE routeEngine( board, aSettings, occupancy );
    autorouterDebugLog( "route engine constructed" );
    std::vector<ROUTING_CONNECTION> connections = staticExistingRoutes;
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

    if( fanoutConnectionTotal > 0 && !fanoutTimedOut )
    {
        // Fanout has its own rip-up policy.  Build a separate immutable
        // search engine for this stage so a disabled fanout rip-up cannot
        // accidentally inherit the ordinary batch's negotiated-congestion
        // permission through MAZE_SEARCH_ENGINE::m_settings.
        AUTOROUTER_SETTINGS fanoutSettings = aSettings;
        fanoutSettings.allowRipupRouted = aSettings.fanoutRipupAllowed;

        AUTOROUTE_ENGINE fanoutRouteEngine( fanoutBoard, fanoutSettings, occupancy );
        struct FANOUT_ENGINE
        {
            int                                  netCode = 0;
            ROUTING_VIA_DIMENSION                via;
            std::unique_ptr<AUTOROUTE_ENGINE>    engine;
        };
        // Re-indexing a large board for every SMD pin would undo the batch
        // router's persistent-index performance work.  Cache one immutable
        // engine per (net, selected ViaRule profile); those engines share the
        // occupancy transaction but have their own net-local via radii.
        std::vector<FANOUT_ENGINE> fanoutEngines;
        const auto fanoutEngineFor = [&]( int aNetCode, const ROUTING_PAD& aLanding )
                -> const AUTOROUTE_ENGINE&
        {
            if( !aLanding.isFanoutTarget || aLanding.fanoutViaDiameter <= 0
                || aLanding.fanoutViaDrill <= 0 )
            {
                return fanoutRouteEngine;
            }

            const ROUTING_VIA_DIMENSION via{ aLanding.fanoutViaDiameter,
                                              aLanding.fanoutViaDrill };
            const auto found = std::find_if(
                    fanoutEngines.begin(), fanoutEngines.end(),
                    [&]( const FANOUT_ENGINE& aEntry )
                    { return aEntry.netCode == aNetCode && aEntry.via == via; } );
            if( found != fanoutEngines.end() )
                return *found->engine;

            FANOUT_ENGINE entry;
            entry.netCode = aNetCode;
            entry.via = via;
            entry.engine = std::make_unique<AUTOROUTE_ENGINE>(
                    fanoutBoard, fanoutSettings, occupancy, aNetCode, via );
            fanoutEngines.push_back( std::move( entry ) );
            return *fanoutEngines.back().engine;
        };
        const auto orderedPins = BATCH_FANOUT::OrderedPins( fanoutBoard,
                                                             aSettings.fanoutPinOrder,
                                                             fanoutStageCancel );
        std::map<std::size_t, std::pair<const ROUTING_NET*, std::size_t>> tasks;
        for( const auto& net : fanoutBoard.nets )
            for( const auto& [source, target] : net.connections )
                tasks.emplace( source, std::pair{ &net, target } );
        std::optional<std::pair<int, std::size_t>> previousOutcome;
        int identicalPasses = 0;
        int totalItemsFanouted = 0;
        bool maxItemLimitReached = false;

        // A deadline is a normal fallback, not a partially successful
        // fanout.  Retaining only the early escapes poisons the ordinary
        // batch search with provisional vias/traces and can create hundreds
        // of DRC errors.  Keep the shared occupancy and emitted connection
        // list transactional until the complete fanout stage is known to fit
        // its budget.
        const std::vector<ROUTING_CONNECTION> connectionsBeforeFanout = connections;
        const std::set<int> failedNetsBeforeFanout = failedNets;
        const int ripupsBeforeFanout = ripups;
        const int expandedBeforeFanout = totalExpandedNodes;
        ROUTING_OCCUPANCY::TRANSACTION fanoutTransaction( occupancy );

        for( int pass = 0; pass < aSettings.maxFanoutPasses; ++pass )
        {
            if( fanoutStageCancel() )
            {
                if( aCancel && aCancel() )
                {
                    result.cancelled = true;
                    result.message = "Autorouter cancelled";
                }
                break;
            }

            if( aSettings.maxFanoutItems > 0
                && totalItemsFanouted >= aSettings.maxFanoutItems )
            {
                maxItemLimitReached = true;
                break;
            }

            int routedPins = 0;
            const auto before = occupancy.Connections();
            for( auto pin : orderedPins )
            {
                if( fanoutStageCancel() )
                {
                    if( aCancel && aCancel() )
                    {
                        result.cancelled = true;
                        result.message = "Autorouter cancelled";
                    }
                    break;
                }

                if( aSettings.maxFanoutItems > 0
                    && totalItemsFanouted >= aSettings.maxFanoutItems )
                {
                    maxItemLimitReached = true;
                    break;
                }

                const auto task = tasks.find( pin );
                if( task == tasks.end() || occupancy.Board()->Connected( pin, task->second.second ) )
                    continue;

                ROUTING_NET net = *task->second.first;
                net.connections = { { pin, task->second.second } };
                const auto pinStarted = std::chrono::steady_clock::now();
                // This is a per-pin *maximum*, not a value that grows on
                // each fanout pass.  Multiplying it by the pass number makes
                // later retries arbitrarily slower and defeats the global
                // stage deadline on boards with many SMD pins.
                const std::int64_t pinBudget = std::max<std::int64_t>(
                        0, aSettings.maxFanoutMillisecondsPerPin );
                bool pinTimedOut = false;
                const ROUTER_CANCEL_CALLBACK pinCancel = [&]()
                {
                    if( fanoutStageCancel() )
                        return true;

                    if( std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - pinStarted )
                                .count()
                        >= pinBudget )
                    {
                        pinTimedOut = true;
                        return true;
                    }

                    return false;
                };

                ++totalItemsFanouted;
                const ROUTING_PAD& landing = fanoutBoard.pads[task->second.second];
                const AUTOROUTE_ENGINE& pinRouteEngine = fanoutEngineFor( net.netCode, landing );
                int expanded = 0;
                const bool routed = routeNet(
                        fanoutBoard, fanoutSettings, net, pass, occupancy, pinRouteEngine,
                        connections, expanded, ripups, pinCancel,
                        [&]( int aSearchExpanded )
                        {
                            if( !aProgress )
                                return;

                            ROUTER_PROGRESS progress;
                            progress.pass = pass + 1;
                            progress.maxPasses = aSettings.maxFanoutPasses;
                            progress.totalConnections = fanoutConnectionTotal;
                            progress.routedConnections = routedPins;
                            progress.ripups = ripups;
                            progress.expandedNodes = totalExpandedNodes + aSearchExpanded;
                            progress.elapsedMilliseconds = elapsedMilliseconds();
                            progress.stage = "Searching SMD fanout escape";
                            aProgress( progress );
                        } );
                totalExpandedNodes += expanded;

                if( aCancel && aCancel() )
                {
                    result.cancelled = true;
                    result.message = "Autorouter cancelled";
                    break;
                }

                if( routed )
                    ++routedPins;
                else
                    failedNets.insert( net.netCode );

                result.metrics.routedConnections = routedConnectionCount();
                if( autorouterDebugEnabled() )
                {
                    std::ostringstream message;
                    const auto& pad = fanoutBoard.pads[pin];
                    message << "FANOUT_PIN pass=" << pass + 1 << " component=" << pad.componentId
                            << " pin=" << pad.pinIndex << " net=" << net.netCode
                            << " routed=" << routed << " pinTimedOut=" << pinTimedOut
                            << " stageTimedOut=" << fanoutTimedOut;
                    autorouterDebugLog( message.str() );
                }
                if( aProgress )
                {
                    ROUTER_PROGRESS progress;
                    progress.pass = pass + 1;
                    progress.maxPasses = aSettings.maxFanoutPasses;
                    progress.totalConnections = fanoutConnectionTotal;
                    progress.routedConnections = routedPins;
                    progress.ripups = ripups;
                    progress.expandedNodes = totalExpandedNodes;
                    progress.elapsedMilliseconds = elapsedMilliseconds();
                    progress.stage = pinTimedOut ? "SMD fanout pin timed out"
                                                 : "Routing SMD fanout";
                    aProgress( progress );
                }

                if( fanoutTimedOut || result.cancelled )
                    break;
            }

            // Pinned fanout loop: zero routed pins, three repeated (routed,
            // via-count) outcomes, unchanged geometry, item cap, deadline,
            // or cancellation stops.  A timeout is intentionally not a job
            // cancellation; the fallback graph below restores failed SMD
            // pads and ordinary routing continues.
            if( result.cancelled || fanoutTimedOut || maxItemLimitReached || routedPins == 0 )
                break;
            std::set<std::pair<std::int64_t, std::int64_t>> vias;
            for( const auto& route : occupancy.Connections() )
                for( std::size_t i = 1; i < route.nodes.size(); ++i )
                    if( route.nodes[i - 1].layer != route.nodes[i].layer )
                        vias.emplace( route.nodes[i].point.x, route.nodes[i].point.y );
            const auto outcome = std::pair{ routedPins, vias.size() };
            if( previousOutcome == outcome )
            {
                if( ++identicalPasses >= 3 )
                    break;
            }
            else
            {
                identicalPasses = 0;
                previousOutcome = outcome;
            }
            const auto& after = occupancy.Connections();
            if( before.size() == after.size() && std::equal( before.begin(), before.end(), after.begin(),
                    []( const auto& a, const auto& b ) { return SameRouteGeometry( a, b ); } ) )
                break;
        }

        if( fanoutTimedOut )
        {
            connections = connectionsBeforeFanout;
            failedNets = failedNetsBeforeFanout;
            ripups = ripupsBeforeFanout;
            totalExpandedNodes = expandedBeforeFanout;
            result.fanoutTimedOut = true;
            result.message = "SMD fanout stage timed out; continuing with ordinary routing.";
        }
        else
        {
            fanoutTransaction.Commit();
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
            {
                if( connection.isExistingBoardRoute )
                    occupancy.AddStatic( connection );
                else
                    occupancy.Add( connection );
            }
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
                if( connection.complete && !connection.isExistingBoardRoute )
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
                if( connection.complete && !connection.isExistingBoardRoute )
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
