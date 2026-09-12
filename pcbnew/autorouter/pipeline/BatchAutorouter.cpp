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
#include "../board/optimize/TraceTightener.h"
#include "../drc/DesignRulesChecker.h"
#include "../path/FoundConnectionInserter.h"
#include "AutorouteAirlineCalculator.h"
#include "AutoroutePassRunner.h"
#include "AutorouteConnectionRouter.h"
#include "AutorouteBatchLoop.h"
#include "BatchOptimizer.h"
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
                               const ROUTING_PAD& aLanding,
                               bool aAllowSameLayerFinish = false )
{
    if( !aLanding.isFanoutTarget || aLanding.fanoutSourceLayer < 0
        || aLanding.fanoutTargetLayer < 0 || aConnection.nodes.size() < 2 )
    {
        return true;
    }

    // A direct TargetItemExpansionDoor completion has no nextRoom in the
    // reference queue, so neither the maximum source-room envelope nor the
    // minimum drill distance is applied.  The room frontier already checked
    // every non-target state leading to this completion.  Re-validating the
    // reconstructed centre-line here rejected legal contacts to an existing
    // same-net via just beyond the envelope even though Freerouting routes it.
    if( aAllowSameLayerFinish )
        return true;

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

    // A synthetic landing on a different layer must normally be reached
    // through its fanout via.  Dynamic fanout is the exception: reaching a
    // real same-layer destination before a drill is a completed fanout
    // attempt in the reference implementation.
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
        style.viaLayers = aLanding.fanoutViaLayers;
        style.viaType = aLanding.fanoutViaType;
        style.viaLayerGeometry = aLanding.fanoutViaLayerGeometry;
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
 * - each circular KiCad drilled via, including per-layer diameters, as its
 *   own static route.  Its normal endpoint
 *   trace contacts remain separate source items so a later
 *   `ShoveViaConnectionPlan` can reproduce `DrillItem.moveBy()`: retain
 *   each trace and add one bridge per contacted layer.
 *
 * Anything with an interior/non-normal contact, pad-attached drill, arc,
 * non-circular padstack shape, or unsupported shape remains a static obstacle and is
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
        connection.isAutorouterOwned = std::all_of(
                pieces.begin(), pieces.end(), []( const ROUTING_OBSTACLE* aPiece )
                { return aPiece->isAutorouterOwned; } );
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
            // Reaching this reconstruction path means the user selected a
            // whole-net replacement and the adapter proved this host item is
            // a candidate for transactional movement.  That operation is the
            // native equivalent of unfixing a protected DSN wire before the
            // source shove algorithm mutates it.
            traceStyle.fixedState = ROUTER_FIXED_STATE::UNFIXED;
            connection.edgeStyles = { std::move( traceStyle ) };
            atoms.push_back( { EXISTING_ATOM::KIND::TRACE, std::move( connection ) } );
            continue;
        }

        // A drilled KiCad via becomes one layer transition while retaining
        // the circular copper diameter and effective clearance of every
        // physical padstack layer. Revalidate the captured pieces here because
        // this function is also used by data-only regression tests.
        const ROUTER_POINT position = copper.front()->start;
        std::vector<int> layers;
        std::int64_t diameter = 0;
        std::int64_t clearance = 0;
        std::vector<ROUTING_VIA_LAYER_GEOMETRY> layerGeometry;
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
            diameter = std::max( diameter, currentDiameter );
            const std::int64_t currentClearance = std::max<std::int64_t>( 0, piece->clearance );
            clearance = std::max( clearance, currentClearance );
            layerGeometry.push_back( { piece->layers.front(), currentDiameter, currentClearance } );
            layers.push_back( piece->layers.front() );
        }

        std::sort( layers.begin(), layers.end(), [&]( int aLeft, int aRight )
        { return layerOrder( aLeft ) < layerOrder( aRight ); } );
        std::sort( layerGeometry.begin(), layerGeometry.end(),
                   [&]( const ROUTING_VIA_LAYER_GEOMETRY& aLeft, const ROUTING_VIA_LAYER_GEOMETRY& aRight )
                   {
                       return layerOrder( aLeft.layer ) < layerOrder( aRight.layer );
                   } );
        if( std::adjacent_find( layerGeometry.begin(), layerGeometry.end(),
                                []( const ROUTING_VIA_LAYER_GEOMETRY& aLeft, const ROUTING_VIA_LAYER_GEOMETRY& aRight )
                                {
                                    return aLeft.layer == aRight.layer;
                                } )
            != layerGeometry.end() )
        {
            via = false;
        }
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
        viaStyle.viaLayerGeometry = std::move( layerGeometry );
        viaStyle.fixedState = ROUTER_FIXED_STATE::UNFIXED;
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
        const std::vector<int>& layers = style.viaLayers;
        if( layers.empty() )
            return false;

        // DrillItemMover rejects a drill item with a normal PAD contact.  A
        // point-only test misses the ordinary KiCad case where the via lands
        // inside a pad away from its centre, so use the captured conservative
        // pad radius.  False positives are intentionally fail-closed.
        for( const ROUTING_PAD& pad : aBoard.pads )
        {
            if( pad.isFanoutTarget || pad.isPlaneTarget || pad.netCode != via.netCode )
            {
                continue;
            }

            std::int64_t sharedLayerRadius = 0;
            for( int layer : layers )
            {
                if( !isOnLayer( pad.layers, layer ) )
                    continue;

                sharedLayerRadius = std::max(
                        sharedLayerRadius,
                        std::max<std::int64_t>( 1, ViaStyleDiameterOnLayer( style, layer, style.viaDiameter ) / 2 ) );
            }
            if( sharedLayerRadius <= 0 )
                continue;

            const long double dx = static_cast<long double>( position.x ) - pad.position.x;
            const long double dy = static_cast<long double>( position.y ) - pad.position.y;
            const long double radius =
                    static_cast<long double>( sharedLayerRadius ) + std::max<std::int64_t>( 0, pad.radius );
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

                std::int64_t layerRadius = 0;
                for( int layer : obstacle.layers )
                {
                    if( isOnLayer( layers, layer ) )
                    {
                        layerRadius =
                                std::max( layerRadius,
                                          std::max<std::int64_t>(
                                                  1, ViaStyleDiameterOnLayer( style, layer, style.viaDiameter ) / 2 ) );
                    }
                }
                const long double radius =
                        static_cast<long double>( layerRadius ) + std::max<std::int64_t>( 0, obstacle.radius );
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

    // Freerouting v2.3 deliberately disabled the experimental airline-length
    // sort because it regressed convergence. Preserve the adapter's stable
    // source order; do not silently replace item iteration with a
    // largest-net/longest-airline heuristic.

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
                                 const ROUTER_SEARCH_PROGRESS_CALLBACK& aSearchProgress,
                                 std::size_t aPreferredPad,
                                 int aMaximumNewConnections ) const
{
    const auto netStarted = std::chrono::steady_clock::now();
    const std::string preferredSourceId =
            aPreferredPad < aBoard.pads.size() ? aBoard.pads[aPreferredPad].sourceId
                                               : std::string();
    AUTOROUTER_DECISION_CONTEXT_SCOPE decisionContext(
            { { "net", std::to_string( aNet.netCode ) },
              { "net_name", aNet.name },
              { "preferred_pad", std::to_string( aPreferredPad ) },
              { "source_id", preferredSourceId } } );
    autorouterDecisionLog(
            "ROUTE_ITEM_SELECTED",
            { { "retry", std::to_string( aRetry ) },
              { "maximum_new_connections", std::to_string( aMaximumNewConnections ) } } );

    if( autorouterDebugEnabled() )
    {
        std::ostringstream message;
        message << "BEGIN net code=" << aNet.netCode << " pads=" << aNet.padIndices.size()
                << " connections=" << aNet.connections.size() << " retry=" << aRetry
                << " preferredPad=" << aPreferredPad
                << " maxNew=" << aMaximumNewConnections;
        autorouterDebugLog( message.str() );
    }

    if( aNet.padIndices.empty() )
        return true;

    std::vector<ROUTING_CONNECTION> newConnections;
    std::size_t                     newConnectionCount = 0;
    const auto flushNewConnections = [&]()
    {
        if( newConnections.empty() )
            return;

        aConnections.insert( aConnections.end(),
                             std::make_move_iterator( newConnections.begin() ),
                             std::make_move_iterator( newConnections.end() ) );
        newConnections.clear();
    };
    if( aNet.connections.empty() )
        return true;

    // The source router consumes its natural board-item order. The KiCad
    // adapter's ratsnest edge order is the stable native representation of
    // that order; an earlier shortest-edge/component-class sort was a
    // separate heuristic and produced systematically different congestion
    // decisions from the reference.
    std::vector<std::pair<std::size_t, std::size_t>> pendingConnections = aNet.connections;

    // BatchAutorouter.getAutorouteItems() schedules one natural-order board
    // item at a time. Model that item by its real pad representative and let
    // the worker contact graph supply its complete connected/unconnected
    // sets. A ratsnest edge is only a fallback when no item was requested
    // (the isolated fanout stage and data-only callers).  Post-refill repair
    // is the deliberate exception: its exact synthetic endpoints identify
    // two particular KiCad zone islands.  Replacing that edge with the
    // preferred real pad's broad plane target set can route copper that never
    // joins those islands and then report the exact ratsnest edge missing.
    const bool exactRepairTask = std::any_of(
            pendingConnections.begin(), pendingConnections.end(),
            [&]( const auto& aConnection )
            {
                return ( aConnection.first < aBoard.pads.size()
                         && aBoard.pads[aConnection.first].isExactTarget )
                       || ( aConnection.second < aBoard.pads.size()
                            && aBoard.pads[aConnection.second].isExactTarget );
            } );
    if( aPreferredPad != std::numeric_limits<std::size_t>::max()
        && !exactRepairTask )
    {
        if( aPreferredPad >= aBoard.pads.size()
            || aBoard.pads[aPreferredPad].netCode != aNet.netCode )
        {
            flushNewConnections();
            return false;
        }

        std::optional<std::size_t> unconnectedTarget;
        for( std::size_t target : aNet.planeTargetIndices )
        {
            if( target < aBoard.pads.size()
                && !aOccupancy.Board()->Connected( aPreferredPad, target ) )
            {
                unconnectedTarget = target;
                break;
            }
        }

        if( !unconnectedTarget )
        {
            for( std::size_t target : aNet.padIndices )
            {
                if( target < aBoard.pads.size() && target != aPreferredPad
                    && !aOccupancy.Board()->Connected( aPreferredPad, target ) )
                {
                    unconnectedTarget = target;
                    break;
                }
            }
        }

        if( !unconnectedTarget )
        {
            flushNewConnections();
            return true;
        }

        pendingConnections = { { aPreferredPad, *unconnectedTarget } };
    }

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

        const auto [sourceIndex, targetIndex] = pendingConnections.front();
        pendingConnections.erase( pendingConnections.begin() );

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
        const bool requestedFanoutTask =
                aBoard.pads[targetIndex].isFanoutTarget
                && aBoard.pads[targetIndex].fanoutSourcePadIndex == sourceIndex;

        // The scheduled item remains the anchor even when another component
        // already contains routed copper.  AutorouteConnectionRouter derives
        // both sets from that exact Item; it never swaps to whichever side of
        // a ratsnest edge happens to look more connected.  Such a swap reverses
        // start/destination room construction and changes maze ordering even
        // though the eventual electrical connection would be equivalent.

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
            std::vector<std::vector<ROUTING_TERMINAL>> destinationAttempts;
            // Upstream AutorouteConnectionRouter routes from the unconnected
            // set to the selected item's connected set for ordinary nets.
            // Preserve all legal pad terminals, not just the ratsnest pair.
            // Fanout retains its explicit escape transition. Plane routing
            // starts at the entire connected set (AutorouteConnectionRouter),
            // not just the chosen synthetic landing on one layer.
            if( requestedFanoutTask )
            {
                starts = aOccupancy.Board()->Terminals( sourceIndex );

                // RoutingBoard.fanout() searches the pin's complete
                // unconnected item set.  Real pad representatives expose
                // their attached trace/via terminals through the worker
                // board, so this remains dynamic after every insertion and
                // rip-up instead of freezing one ratsnest edge.
                auto unconnected = aOccupancy.Board()->UnconnectedTargetItems(
                        sourceIndex, aNet.netCode );
                std::stable_sort(
                        unconnected.begin(), unconnected.end(),
                        [&]( const ROUTING_BOARD::TARGET_ITEM& aLeft,
                             const ROUTING_BOARD::TARGET_ITEM& aRight )
                        {
                            const long double leftX =
                                    ( static_cast<long double>( aLeft.bounds.minX )
                                      + aLeft.bounds.maxX ) / 2.0L;
                            const long double leftY =
                                    ( static_cast<long double>( aLeft.bounds.minY )
                                      + aLeft.bounds.maxY ) / 2.0L;
                            const long double rightX =
                                    ( static_cast<long double>( aRight.bounds.minX )
                                      + aRight.bounds.maxX ) / 2.0L;
                            const long double rightY =
                                    ( static_cast<long double>( aRight.bounds.minY )
                                      + aRight.bounds.maxY ) / 2.0L;
                            const long double leftDx = leftX - source.position.x;
                            const long double leftDy = leftY - source.position.y;
                            const long double rightDx = rightX - source.position.x;
                            const long double rightDy = rightY - source.position.y;
                            const long double left = leftDx * leftDx + leftDy * leftDy;
                            const long double right = rightDx * rightDx + rightDy * rightDy;
                            return left != right ? left < right : aLeft.id > aRight.id;
                        } );
                std::vector<std::vector<ROUTING_TERMINAL>> unconnectedTerminalSets;
                unconnectedTerminalSets.reserve( unconnected.size() );
                for( const ROUTING_BOARD::TARGET_ITEM& item : unconnected )
                {
                    unconnectedTerminalSets.push_back( item.terminals );
                    destinations.insert( destinations.end(), item.terminals.begin(),
                                         item.terminals.end() );
                }

                // RoutingBoard.fanout() deliberately avoids searching a
                // small multi-item net as one broad target set on its first
                // attempt. For one to four unconnected items it first tries
                // only the item whose bounding-box centre is closest to the
                // pin; only a failed search falls back to the complete set.
                // Larger nets use the complete set immediately. The native
                // terminals are exact item representatives, so preserving
                // them as separate attempts reproduces that source decision
                // without inventing another ratsnest edge.
                if( !destinations.empty() && unconnected.size() <= 4 )
                {
                    destinationAttempts.push_back( unconnectedTerminalSets.front() );

                    if( unconnected.size() > 1 )
                        destinationAttempts.push_back( destinations );
                }
                else if( !destinations.empty() )
                {
                    destinationAttempts.push_back( destinations );
                }

                // Plane conduction areas are represented by grouped target
                // terminals above. If no real destination remains, the reference
                // returns FAILED instead of routing toward the synthetic
                // control item itself.
            }
            else if( targetIsPlane && !source.isExactTarget )
            {
                // Plane routing searches from the selected item's complete
                // connected set to its complete unconnected item set.  The
                // latter includes the ConductionArea itself, whose terminal
                // carries the exact finite filled region rather than one of
                // the adapter's synthetic sampling coordinates.
                auto sets = AUTOROUTE_CONNECTION_ROUTER::TerminalSetsForItem(
                        *aOccupancy.Board(), routeSourceIndex, aNet.netCode, true );
                starts = std::move( sets.starts );
                destinations = std::move( sets.destinations );
            }
            else if( aNet.planeTargetIndices.empty() )
            {
                auto sets = AUTOROUTE_CONNECTION_ROUTER::TerminalSetsForItem(
                        *aOccupancy.Board(), routeSourceIndex, aNet.netCode, false );
                starts = std::move( sets.starts );
                destinations = std::move( sets.destinations );
            }

            // An empty explicit set selects the ordinary pair/synthetic
            // landing behavior in MAZE_SEARCH_ENGINE. All non-fanout routes
            // have exactly one destination attempt.
            if( destinationAttempts.empty()
                && ( !requestedFanoutTask || !destinations.empty() ) )
                destinationAttempts.push_back( destinations );

            {
                const auto describeTerminals = []( const auto& aTerminals )
                {
                    std::ostringstream result;
                    bool first = true;
                    for( const ROUTING_TERMINAL& terminal : aTerminals )
                    {
                        if( !first )
                            result << ';';
                        first = false;
                        result << terminal.padIndex << '@'
                               << terminal.pad.position.x << ','
                               << terminal.pad.position.y;
                    }
                    return result.str();
                };
                autorouterDecisionLog(
                        "ROUTE_TERMINAL_SETS",
                        { { "route_source", std::to_string( routeSourceIndex ) },
                          { "route_target", std::to_string( routeTargetIndex ) },
                          { "starts", describeTerminals( starts ) },
                          { "destinations", describeTerminals( destinations ) } } );
            }

            for( std::size_t destinationAttempt = 0;
                 destinationAttempt < destinationAttempts.size() && !connection;
                 ++destinationAttempt )
            {
                const std::vector<ROUTING_TERMINAL>& searchDestinations =
                        destinationAttempts[destinationAttempt];

                // AutorouteConnectionRouter performs one normal-width search
                // for an item in each batch pass, followed only by its
                // optional neck-width search.  The old native implementation
                // multiplied this work by maxIterations while also refining
                // the grid and retry costs; a single difficult item could
                // therefore consume millions of nodes before the next item
                // was seen.  Pass-level retry/rip-up state is aRetry.
                for( int iteration = 0; iteration < 1; ++iteration )
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
                    ROUTING_PAD searchTarget = target;
                    if( !requestedFanoutTask )
                        searchTarget.isFanoutTarget = false;
                    if( requestedFanoutTask && !aNet.viaProfiles.empty() )
                    {
                        // The combined fanout ViaRule is ordered, but each
                        // ViaInfo owns a different clearance envelope. Using
                        // the largest annulus as one shared drill-page broad
                        // phase can erase a legal landing for a later,
                        // smaller profile before SelectViaStyle gets to test
                        // it. Run the same room frontier once per declared
                        // profile, preserving rule order and keeping the
                        // selected padstack local to this pin attempt.
                        for( const ROUTING_VIA_PROFILE& profile : aNet.viaProfiles )
                        {
                            int profileExpanded = 0;
                            const ROUTER_SEARCH_PROGRESS_CALLBACK profileProgress =
                                    [&]( int aProfileExpanded )
                            {
                                if( aSearchProgress )
                                    aSearchProgress( expandedBeforeSearch + expanded
                                                     + aProfileExpanded );
                            };
                            AUTOROUTE_ENGINE profileEngine(
                                    aBoard, aSettings, aOccupancy, aNet.netCode, profile );
                            connection = profileEngine.AutorouteConnection(
                                    source, searchTarget, aRetry, profileExpanded, aCancel,
                                    profileProgress, starts, searchDestinations );
                            expanded += profileExpanded;
                            if( connection || ( aCancel && aCancel() ) )
                                break;
                        }
                    }
                    else
                    {
                        const bool hasProtectedFanout = !requestedFanoutTask
                                && std::any_of(
                                        aConnections.begin(), aConnections.end(),
                                        [&]( const ROUTING_CONNECTION& aConnection )
                                        {
                                            return aConnection.netCode == aNet.netCode
                                                   && aConnection.isFanoutConnection;
                                        } );
                        if( hasProtectedFanout )
                        {
                            // A completed fanout has already paid for the
                            // layer transition and exposes trace terminals on
                            // its exit layer. Prefer connecting to that copper
                            // without another drill. This is the native
                            // equivalent of removeTails(FANOUT_VIA): the
                            // protected escape remains useful rather than
                            // being bypassed by a cheaper microscopic test via
                            // and then deleted as a redundant tail.
                            AUTOROUTER_SETTINGS sameLayerSettings = aSettings;
                            sameLayerSettings.allowVias = false;
                            AUTOROUTE_ENGINE sameLayerEngine(
                                    aBoard, sameLayerSettings, aOccupancy );
                            connection = sameLayerEngine.AutorouteConnection(
                                    source, searchTarget, aRetry, expanded, aCancel,
                                    searchProgress, starts, searchDestinations );
                        }
                        if( !connection && !( aCancel && aCancel() ) )
                        {
                            connection = aEngine.AutorouteConnection(
                                    source, searchTarget, aRetry, expanded, aCancel,
                                    searchProgress, starts, searchDestinations );
                        }
                    }
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
                                source, searchTarget, aRetry, neckExpanded, aCancel,
                                neckSearchProgress, starts, searchDestinations );
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
                                << " destinationAttempt=" << destinationAttempt
                                << " destinations=" << searchDestinations.size()
                                << " iteration=" << iteration
                                << " found=" << connection.has_value()
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

        const bool searchMarkedFanout = connection->isFanoutConnection;
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
        if( searchMarkedFanout && fanoutSource == nullptr )
        {
            fanoutSource = &aBoard.pads[sourceIndex];
            fanoutLanding = &aBoard.pads[targetIndex];
        }
        connection->isFanoutConnection = searchMarkedFanout || fanoutSource != nullptr;

        if( connection->isFanoutConnection )
        {
            const bool sameLayerFinish = searchMarkedFanout
                                         && routeTargetIndex != targetIndex;
            if( !fanoutEscapeFitsEnvelope( *connection, *fanoutSource,
                                           *fanoutLanding, sameLayerFinish ) )
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
        if( autorouterDebugEnabled() )
        {
            std::ostringstream message;
            message << "candidate net=" << connection->netCode << " nodes=";
            for( const ROUTER_NODE& node : connection->nodes )
                message << " (" << node.point.x << ',' << node.point.y << ",L"
                        << node.layer << ')';
            message << " conflicts=" << conflicts.size();
            for( const ROUTING_CONNECTION& conflict : conflicts )
                message << " {net=" << conflict.netCode << ",existing="
                        << conflict.isExistingBoardRoute << ",movable="
                        << conflict.isShoveMovable << ",nodes=" << conflict.nodes.size() << '}';
            autorouterDebugLog( message.str() );
        }
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

        // AutoroutePassRunner starts changed-area marking before every item,
        // and AutorouteConnectionRouter runs optChangedArea after a successful
        // insertion.  Capture every part of this atomic edit before publishing
        // the new route so the next queued item sees the same tightened copper
        // as Freerouting rather than the raw locator polyline.
        CHANGED_AREA changedArea(
                TRACE_TIGHTENER::LayerCount( aBoard, aSettings ) );
        TRACE_TIGHTENER::MarkConnection(
                changedArea, *connection, aBoard, aSettings );
        for( const ROUTING_CONNECTION& conflict : conflicts )
            TRACE_TIGHTENER::MarkConnection(
                    changedArea, conflict, aBoard, aSettings );
        for( const auto& shove : inserted.shoved )
        {
            TRACE_TIGHTENER::MarkConnection(
                    changedArea, shove.original, aBoard, aSettings );
            TRACE_TIGHTENER::MarkConnection(
                    changedArea, shove.replacement, aBoard, aSettings );
            for( const ROUTING_CONNECTION_REPLACEMENT& contact : shove.materializedContacts )
            {
                TRACE_TIGHTENER::MarkConnection(
                        changedArea, contact.original, aBoard, aSettings );
                TRACE_TIGHTENER::MarkConnection(
                        changedArea, contact.replacement, aBoard, aSettings );
            }
            for( const ROUTING_CONNECTION& bridge : shove.bridges )
                TRACE_TIGHTENER::MarkConnection(
                        changedArea, bridge, aBoard, aSettings );
        }

        newConnections.push_back( std::move( *connection ) );
        ++newConnectionCount;
        flushNewConnections();
        TRACE_TIGHTENER( aBoard, aSettings, aOccupancy )
                .OptChangedArea( changedArea, aConnections, 0, aCancel, 1000 );

        if( aMaximumNewConnections > 0
            && static_cast<int>( newConnectionCount ) >= aMaximumNewConnections )
        {
            break;
        }
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

    return allConnectionsRouted
           || ( aMaximumNewConnections > 0 && newConnectionCount > 0 );
}


int BATCH_AUTOROUTER::AutoroutePassesForOptimizingItem(
        const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
        int aMaxPassCount, ROUTING_OCCUPANCY& aOccupancy,
        std::vector<ROUTING_CONNECTION>& aConnections,
        const ROUTER_CANCEL_CALLBACK& aCancel, int* aExpandedNodes, int* aRipups )
{
    // The source constructs a dedicated BatchAutorouter with the optimizer's
    // calculated start rip-up cost. AutorouteConnectionRouter enables rip-up
    // on every one of these passes, including pass one.
    AUTOROUTER_SETTINGS optimizerSettings = aSettings;
    optimizerSettings.allowRipupRouted = true;
    optimizerSettings.allowRipupOnFirstIteration = true;

    BATCH_AUTOROUTER router;
    AUTOROUTE_ENGINE engine( aBoard, optimizerSettings, aOccupancy );
    int expandedNodes = 0;
    int ripups = 0;
    bool stillUnroutedItems = true;
    int currentPassNo = 1;

    while( stillUnroutedItems && !( aCancel && aCancel() )
           && currentPassNo <= aMaxPassCount )
    {
        const std::vector<AUTOROUTE_ITEM> items =
                AUTOROUTE_PASS_RUNNER::GetAutorouteItems( aBoard, *aOccupancy.Board() );

        if( items.empty() )
            stillUnroutedItems = false;
        else
        {
            // AutoroutePassRunner snapshots the to-do list before mutating the
            // board. An item connected by an earlier task in this same snapshot
            // is skipped and does not by itself keep the pass loop alive.
            bool attemptedItem = false;
            for( const AUTOROUTE_ITEM& item : items )
            {
                if( aCancel && aCancel() )
                    break;

                const auto net = std::find_if(
                        aBoard.nets.begin(), aBoard.nets.end(),
                        [&]( const ROUTING_NET& aNet ) { return aNet.netCode == item.netCode; } );
                if( net == aBoard.nets.end() || aOccupancy.Board()->CountMissing( *net ) == 0 )
                    continue;

                attemptedItem = true;
                int itemExpanded = 0;
                // Java pass numbers are one-based. The native search takes a
                // zero-based retry and derives ripupPassNo = retry + 1.
                router.routeNet( aBoard, optimizerSettings, *net, currentPassNo - 1,
                                 aOccupancy, engine, aConnections, itemExpanded,
                                 ripups, aCancel, {}, item.pad, 1 );
                expandedNodes += itemExpanded;
            }

            stillUnroutedItems = attemptedItem;
        }

        ++currentPassNo;
    }

    if( !stillUnroutedItems )
        --currentPassNo;

    if( aExpandedNodes )
        *aExpandedNodes += expandedNodes;
    if( aRipups )
        *aRipups += ripups;
    return currentPassNo;
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
        if( autorouterDebugEnabled() )
        {
            int transitions = 0;
            for( std::size_t index = 1; index < connection.nodes.size(); ++index )
                if( connection.nodes[index - 1].layer != connection.nodes[index].layer )
                    ++transitions;
            std::ostringstream message;
            message << "ROUTE_GEOMETRY fanout=" << connection.isFanoutConnection
                    << " net=" << connection.netCode
                    << " fromPad=" << connection.fromPadIndex
                    << " toPad=" << connection.toPadIndex
                    << " nodes=" << connection.nodes.size()
                    << " transitions=" << transitions;
            if( !connection.nodes.empty() )
            {
                message << " first=(" << connection.nodes.front().point.x << ','
                        << connection.nodes.front().point.y << ",L"
                        << connection.nodes.front().layer << ") last=("
                        << connection.nodes.back().point.x << ','
                        << connection.nodes.back().point.y << ",L"
                        << connection.nodes.back().layer << ')';
            }
            autorouterDebugLog( message.str() );
        }
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
        const auto duplicate =
                std::find_if( uniqueVias.begin(), uniqueVias.end(),
                              [&]( const ROUTING_VIA& existing )
                              {
                                  return existing.netCode == via.netCode && existing.position == via.position
                                         && existing.topLayer == via.topLayer && existing.bottomLayer == via.bottomLayer
                                         && existing.diameter == via.diameter && existing.drill == via.drill
                                         && existing.layers == via.layers
                                         && existing.layerGeometry == via.layerGeometry;
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
        if( !connection.isExistingBoardRoute || connection.isAutorouterOwned )
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

    // A job-owned BOARD_ITEM that disappeared from the final worker route set
    // was conventionally ripped up rather than shoved.  Delete it from the
    // private KiCad board before applying replacement geometry.  Unchanged
    // job-owned routes remain represented by static records and are retained.
    std::set<std::string> autorouterOwnedIds;
    for( const ROUTING_OBSTACLE& obstacle : aBoard.removableExistingRoutes )
    {
        if( obstacle.isExistingRoute && obstacle.isAutorouterOwned
            && !obstacle.boardItemId.empty() )
        {
            autorouterOwnedIds.insert( obstacle.boardItemId );
        }
    }

    for( const std::string& id : autorouterOwnedIds )
    {
        const bool retainedAsHostItem = std::any_of(
                aConnections.begin(), aConnections.end(), [&]( const ROUTING_CONNECTION& route )
                {
                    return route.complete && route.isExistingBoardRoute
                           && std::find( route.sourceBoardItemIds.begin(),
                                         route.sourceBoardItemIds.end(), id )
                                      != route.sourceBoardItemIds.end();
                } );

        if( !retainedAsHostItem )
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

    for( const ROUTING_CONNECTION& connection : aResult.connections )
    {
        std::optional<std::pair<std::int64_t, std::int64_t>> previousDirection;
        for( std::size_t edge = 0; edge + 1 < connection.nodes.size(); ++edge )
        {
            const ROUTER_NODE& from = connection.nodes[edge];
            const ROUTER_NODE& to = connection.nodes[edge + 1];
            if( from.layer != to.layer || from.point == to.point )
            {
                previousDirection.reset();
                continue;
            }
            std::int64_t dx = to.point.x - from.point.x;
            std::int64_t dy = to.point.y - from.point.y;
            const std::int64_t divisor = std::gcd( std::abs( dx ), std::abs( dy ) );
            dx /= std::max<std::int64_t>( 1, divisor );
            dy /= std::max<std::int64_t>( 1, divisor );
            const std::pair<std::int64_t, std::int64_t> direction{ dx, dy };
            if( previousDirection && *previousDirection != direction )
                ++aResult.metrics.bendCount;
            previousDirection = direction;
        }
    }

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
    // Protected source copper remains collision-only.  Copper materialized by
    // an earlier stage of this same job has a host UUID too, but stays a live
    // worker route so rip-up updates connectivity during post-refill repair.
    for( const ROUTING_CONNECTION& route : staticExistingRoutes )
    {
        if( route.isAutorouterOwned )
            occupancy.Add( route );
        else
            occupancy.AddStatic( route );
    }
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
    BOARD_HISTORY history( aSettings );
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
        for( std::size_t target = 0; target < fanoutBoard.pads.size(); ++target )
        {
            const ROUTING_PAD& control = fanoutBoard.pads[target];
            const std::size_t source = control.fanoutSourcePadIndex;
            if( control.isFanoutTarget && control.netCode == net.netCode
                && source < fanoutBoard.pads.size()
                && !fanoutBoard.pads[source].isFanoutTarget )
            {
                fanoutConnections.emplace_back( source, target );
            }
        }

        net.connections = std::move( fanoutConnections );
        if( !net.connections.empty() )
        {
            // RoutingBoard.fanout() evaluates the netclass ViaRule followed
            // by the optional board-rule fallback as one ordered rule for
            // each pin.  Keep that combined rule local to the fanout board;
            // ordinary routing must continue to use the net's real rule.
            net.viaProfiles = BATCH_FANOUT::ViaProfilesFor(
                    board, net.netCode, aSettings );
        }
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

        // The source fanout stage retains every successfully inserted escape
        // when its optional stage deadline expires. Keep a transaction so an
        // exception can still unwind the worker state, but commit every normal
        // return path; a user-cancelled proposal is discarded by the session.
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
                if( task == tasks.end() )
                    continue;

                const ROUTING_NET& taskNet = *task->second.first;
                const ROUTING_PAD& taskPad = fanoutBoard.pads[pin];
                const int sourceLayer = taskPad.layers.empty() ? -1 : taskPad.layers.front();

                // RoutingBoard.fanout() returns ALREADY_CONNECTED as soon as
                // the pin's connected set reaches another layer.  It returns
                // NO_UNCONNECTED_NETS when no real item of the net remains
                // outside that set.  Synthetic landing pads are a native
                // planning detail and must not make either condition false.
                const bool touchesOtherLayer = sourceLayer >= 0
                        && occupancy.Board()->ConnectedSetTouchesOtherLayer( pin, sourceLayer );
                if( sourceLayer < 0 || touchesOtherLayer )
                {
                    if( autorouterDebugEnabled() )
                    {
                        autorouterDebugLog(
                                "FANOUT_PIN_SKIP component="
                                + std::to_string( taskPad.componentId ) + " pin="
                                + std::to_string( taskPad.pinIndex ) + " net="
                                + std::to_string( taskNet.netCode )
                                + ( sourceLayer < 0 ? " reason=no-source-layer"
                                                  : " reason=connected-set-other-layer" ) );
                    }
                    continue;
                }

                // RoutingBoard.fanout() decides this from
                // Pin.getUnconnectedSet(net), which includes traces, vias and
                // conduction areas as individual items.  A ratsnest-pad-only
                // check can skip a legal source fanout or retain false work.
                if( occupancy.Board()->UnconnectedTargetItems( pin, taskNet.netCode ).empty() )
                {
                    if( autorouterDebugEnabled() )
                    {
                        autorouterDebugLog(
                                "FANOUT_PIN_SKIP component="
                                + std::to_string( taskPad.componentId ) + " pin="
                                + std::to_string( taskPad.pinIndex ) + " net="
                                + std::to_string( taskNet.netCode )
                                + " reason=no-unconnected-items" );
                    }
                    continue;
                }

                ROUTING_NET net = taskNet;
                net.connections = { { pin, task->second.second } };
                const auto pinStarted = std::chrono::steady_clock::now();
                // BatchFanout gives harder later passes a linearly increasing
                // per-pin budget. Saturate rather than overflowing when a
                // programmatic caller supplies an extreme value.
                const std::int64_t basePinBudget = std::max<std::int64_t>(
                        0, aSettings.maxFanoutMillisecondsPerPin );
                const std::int64_t passMultiplier = static_cast<std::int64_t>( pass ) + 1;
                const std::int64_t pinBudget =
                        basePinBudget > std::numeric_limits<std::int64_t>::max()
                                                / passMultiplier
                                ? std::numeric_limits<std::int64_t>::max()
                                : basePinBudget * passMultiplier;
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
                const auto reportSearchProgress = [&]( int aSearchExpanded )
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
                };

                const auto connectionsBeforePin = connections;
                int expanded = 0;
                const bool taskComplete = routeNet(
                        fanoutBoard, fanoutSettings, net, pass, occupancy, fanoutRouteEngine,
                        connections, expanded, ripups, pinCancel,
                        reportSearchProgress );
                totalExpandedNodes += expanded;

                const auto insertedFanout = std::find_if(
                        connections.begin(), connections.end(),
                        [&]( const ROUTING_CONNECTION& aConnection )
                        {
                            if( !aConnection.isFanoutConnection )
                                return false;
                            return std::none_of(
                                    connectionsBeforePin.begin(), connectionsBeforePin.end(),
                                    [&]( const ROUTING_CONNECTION& aBefore )
                                    { return SameRouteGeometry( aConnection, aBefore ); } );
                        } );
                const bool insertedFanoutFound = insertedFanout != connections.end();

                // A reference fanout search ends at the first drill it finds;
                // that drill is not constrained to the planning direction's
                // provisional landing.  Move the synthetic terminal to the
                // actual endpoint so the ordinary batch starts from physical
                // copper rather than from the stale guessed coordinate.
                if( insertedFanout != connections.end()
                    && insertedFanout->toPadIndex == task->second.second
                    && !insertedFanout->nodes.empty() )
                {
                    const std::size_t insertedIndex = static_cast<std::size_t>(
                            std::distance( connections.begin(), insertedFanout ) );
                    const ROUTER_POINT actualLanding = insertedFanout->nodes.back().point;
                    ROUTING_PAD& mutableLanding = fanoutBoard.pads[task->second.second];
                    mutableLanding.position = actualLanding;
                    mutableLanding.fanoutEscapePath.clear();
                    for( std::size_t nodeIndex = 0;
                         nodeIndex < insertedFanout->nodes.size(); ++nodeIndex )
                    {
                        const ROUTER_NODE& node = insertedFanout->nodes[nodeIndex];
                        if( node.layer != mutableLanding.fanoutSourceLayer )
                            break;
                        if( mutableLanding.fanoutEscapePath.empty()
                            || mutableLanding.fanoutEscapePath.back() != node.point )
                        {
                            mutableLanding.fanoutEscapePath.push_back( node.point );
                        }

                        if( nodeIndex + 1 < insertedFanout->nodes.size()
                            && insertedFanout->nodes[nodeIndex + 1].layer != node.layer )
                        {
                            const ROUTER_NODE& exit = insertedFanout->nodes[nodeIndex + 1];
                            mutableLanding.fanoutTargetLayer = exit.layer;
                            mutableLanding.layers = { exit.layer };
                            if( nodeIndex < insertedFanout->edgeStyles.size() )
                            {
                                const ROUTING_EDGE_STYLE& selected =
                                        insertedFanout->edgeStyles[nodeIndex];
                                mutableLanding.fanoutViaDiameter = selected.viaDiameter;
                                mutableLanding.fanoutViaDrill = selected.viaDrill;
                                mutableLanding.fanoutViaLayers = selected.viaLayers;
                                mutableLanding.fanoutViaType = selected.viaType;
                                mutableLanding.fanoutViaLayerGeometry = selected.viaLayerGeometry;
                            }
                        }
                    }
                    board.pads[task->second.second] = mutableLanding;
                    occupancy.Board()->RelocateSyntheticPad( task->second.second,
                                                              actualLanding );

                    // The control terminal has served its only purpose.  The
                    // source board keeps just the real pin, inserted trace and
                    // drill items; it never leaves a landing item in the net
                    // graph.  Normalize the route metadata to its real SMD
                    // endpoint and retire the synthetic pad from all later
                    // item-set/component queries.
                    ROUTING_CONNECTION normalized = connections[insertedIndex];
                    occupancy.Remove( normalized );
                    const std::size_t noPad = std::numeric_limits<std::size_t>::max();
                    if( !normalized.nodes.empty()
                        && normalized.nodes.front().point == taskPad.position
                        && normalized.nodes.front().layer == sourceLayer )
                    {
                        normalized.fromPadIndex = pin;
                        normalized.toPadIndex = noPad;
                    }
                    else
                    {
                        normalized.fromPadIndex = noPad;
                        normalized.toPadIndex = pin;
                    }
                    occupancy.Board()->RetireSyntheticPad( task->second.second );
                    board.pads[task->second.second].netCode = 0;
                    board.pads[task->second.second].layers.clear();
                    occupancy.Add( normalized );
                    connections[insertedIndex] = std::move( normalized );

                }

                // A faithful fanout search can stop at its first inserted
                // drill without reaching the nominal net destination.  That
                // is a successful escape even though the synthetic
                // pad-to-landing task remains electrically incomplete.
                const bool routed = taskComplete || insertedFanoutFound;

                if( routed )
                {
                    // BatchFanout starts changed-area marking immediately
                    // before each pin and RoutingBoard.fanout() runs the
                    // source TraceTightener before the next pin.  Build the
                    // same physical edit region from every removed/inserted
                    // route in this attempt, including negotiated shove or
                    // rip-up work, then converge only this net inside it.
                    CHANGED_AREA changedArea(
                            TRACE_TIGHTENER::LayerCount( board, fanoutSettings ) );
                    for( const ROUTING_CONNECTION& previous : connectionsBeforePin )
                    {
                        if( std::none_of(
                                    connections.begin(), connections.end(),
                                    [&]( const ROUTING_CONNECTION& current )
                                    { return SameRouteGeometry( previous, current ); } ) )
                        {
                            TRACE_TIGHTENER::MarkConnection(
                                    changedArea, previous, board, fanoutSettings );
                        }
                    }
                    for( const ROUTING_CONNECTION& current : connections )
                    {
                        if( std::none_of(
                                    connectionsBeforePin.begin(), connectionsBeforePin.end(),
                                    [&]( const ROUTING_CONNECTION& previous )
                                    { return SameRouteGeometry( current, previous ); } ) )
                        {
                            TRACE_TIGHTENER::MarkConnection(
                                    changedArea, current, board, fanoutSettings );
                        }
                    }

                    TRACE_TIGHTENER( board, fanoutSettings, occupancy )
                            .OptChangedArea( changedArea, connections, net.netCode,
                                             fanoutStageCancel, 1000 );

                    // The source normalizes trace items around each accepted
                    // edit. Keep the native exact-junction tail/via subset at
                    // this same boundary after pull-tight and via movement.
                    BATCH_OPTIMIZER( board, fanoutSettings, occupancy )
                            .RemoveRedundantViaTails( connections, fanoutStageCancel,
                                                      net.netCode );
                    connections = occupancy.Connections();
                }

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
            result.fanoutTimedOut = true;
            result.message = "SMD fanout stage timed out; continuing with ordinary routing.";
        }

        fanoutTransaction.Commit();
    }

    // Failed controls are equally absent from the source item model.  Since
    // the real ratsnest was never rewritten through them, retiring each one
    // is sufficient; ordinary routing continues directly from the real SMD
    // pad without graph remapping or a dangling virtual endpoint.
    for( std::size_t index = 0; index < board.pads.size(); ++index )
    {
        ROUTING_PAD& pad = board.pads[index];
        if( pad.isFanoutTarget && pad.netCode > 0 )
        {
            occupancy.Board()->RetireSyntheticPad( index );
            pad.netCode = 0;
            pad.layers.clear();
        }
    }

    auto makeCheckpoint = [&]()
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
        if( autorouterDebugEnabled() )
        {
            autorouterDebugLog(
                    "CHECKPOINT routed="
                    + std::to_string( candidate.metrics.routedConnections ) + "/"
                    + std::to_string( candidate.metrics.totalConnections )
                    + " unrouted="
                    + std::to_string( candidate.metrics.unroutedConnections )
                    + " drc=" + std::to_string( candidate.metrics.drcViolations )
                    + " segments=" + std::to_string( candidate.segments.size() )
                    + " vias=" + std::to_string( candidate.vias.size() ) );
        }
        return candidate;
    };

    auto checkpoint = [&]()
    {
        ROUTING_RESULT candidate = makeCheckpoint();
        history.Add( candidate );
        return candidate;
    };

    auto restoreCheckpoint = [&]( const ROUTING_RESULT& aCheckpoint )
    {
        connections = aCheckpoint.connections;
        occupancy.Clear();
        for( const ROUTING_CONNECTION& connection : connections )
        {
            if( connection.complete )
            {
                if( connection.isExistingBoardRoute && !connection.isAutorouterOwned )
                    occupancy.AddStatic( connection );
                else
                    occupancy.Add( connection );
            }
        }
        result.metrics.routedConnections = routedConnectionCount();
    };

    auto restoreBestCheckpoint = [&]()
    {
        const std::optional<ROUTING_RESULT> best = history.Best();
        if( best )
        {
            if( autorouterDebugEnabled() )
            {
                autorouterDebugLog(
                        "RESTORE_CHECKPOINT routed="
                        + std::to_string( best->metrics.routedConnections ) + "/"
                        + std::to_string( best->metrics.totalConnections )
                        + " unrouted="
                        + std::to_string( best->metrics.unroutedConnections )
                        + " drc=" + std::to_string( best->metrics.drcViolations )
                        + " segments=" + std::to_string( best->segments.size() )
                        + " vias=" + std::to_string( best->vias.size() ) );
            }
            restoreCheckpoint( *best );
        }
    };

    auto refreshFailedNets = [&]()
    {
        failedNets.clear();
        for( const ROUTING_NET& net : board.nets )
            if( occupancy.Board()->CountMissing( net ) > 0 )
                failedNets.insert( net.netCode );
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

        // BoardHistory.add() records the board entering a pass.  The result
        // after this pass becomes the next entry (or the final current board),
        // exactly as in the source loop.
        checkpoint();
        complete = true;

        // AutoroutePassRunner.getAutorouteItems() takes a fresh, source-order
        // snapshot for every pass and queues one representative of each
        // connected item set.  Do not retain the pass-one pad list after the
        // routing board's contact graph has changed.
        const std::vector<AUTOROUTE_ITEM> autorouteItems =
                AUTOROUTE_PASS_RUNNER::GetAutorouteItems( board, *occupancy.Board() );

        if( autorouterDebugEnabled() )
        {
            std::ostringstream message;
            message << "AUTOROUTE_ITEMS pass=" << pass + 1
                    << " count=" << autorouteItems.size();
            for( const AUTOROUTE_ITEM& item : autorouteItems )
                message << " {net=" << item.netCode << ",pad=" << item.pad << '}';
            autorouterDebugLog( message.str() );
            if( autorouteItems.empty() )
                for( std::size_t index = 0; index < board.pads.size(); ++index )
                {
                    const ROUTING_PAD& pad = board.pads[index];
                    autorouterDebugLog( "AUTOROUTE_ITEM_SKIPPED pad="
                            + std::to_string( index ) + " net="
                            + std::to_string( pad.netCode ) + " fanout="
                            + std::to_string( pad.isFanoutTarget ) + " plane="
                            + std::to_string( pad.isPlaneTarget ) + " exact="
                            + std::to_string( pad.isExactTarget ) );
                }
        }

        if( autorouteItems.empty() )
        {
            complete = true;
            result.metrics.passes = pass + 1;
            break;
        }

        for( const AUTOROUTE_ITEM& task : autorouteItems )
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
                if( connection.complete
                    && ( !connection.isExistingBoardRoute
                         || connection.isAutorouterOwned ) )
                {
                    ++routedBefore[connection.netCode];
                }
            }

            const auto netIt = std::find_if(
                    board.nets.begin(), board.nets.end(),
                    [&]( const ROUTING_NET& aNet ) { return aNet.netCode == task.netCode; } );
            if( netIt == board.nets.end() )
                continue;
            const ROUTING_NET& taskNet = *netIt;

            const bool completeNet = occupancy.Board()->CountMissing( taskNet ) == 0;
            const bool retryFailedNet = failedNets.contains( task.netCode );

            // Keep successful nets stable while negotiated-congestion passes
            // revisit only nets that failed or were ripped up.  Re-routing
            // every complete net on every pass causes a dense board to churn
            // thousands of valid connections and can leave the final pass
            // with fewer routes than an earlier checkpoint.
            if( completeNet && ( pass == 0 || !retryFailedNet ) )
                continue;

            const bool routed = routeNet( board, aSettings, taskNet, pass, occupancy,
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
            }, task.pad, 1 );
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
                failedNets.insert( task.netCode );

                // Only a found path identifies the copper that must be
                // removed. Never sacrifice an unrelated completed route
                // merely because this search exhausted its budget.

                ++retries;
            }
            else
            {
                if( occupancy.Board()->CountMissing( taskNet ) == 0 )
                    failedNets.erase( task.netCode );
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
                if( connection.complete
                    && ( !connection.isExistingBoardRoute
                         || connection.isAutorouterOwned ) )
                {
                    ++routedAfter[connection.netCode];
                }
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
                message << "connection batch net=" << task.netCode
                        << " itemPad=" << task.pad << " routed=" << routed
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
        complete = std::all_of(
                board.nets.begin(), board.nets.end(),
                [&]( const ROUTING_NET& aNet )
                { return occupancy.Board()->CountMissing( aNet ) == 0; } );
        for( const ROUTING_NET& net : board.nets )
            if( occupancy.Board()->CountMissing( net ) == 0 )
                failedNets.erase( net.netCode );
        // The source pass returns "no more work" as soon as getAutorouteItems
        // is empty. Continuing routing passes on a complete board only delays
        // the separate optimizer and cannot discover another route.
        if( complete )
            break;

        ROUTING_RESULT currentCheckpoint = makeCheckpoint();
        double currentScore = BOARD_HISTORY::NormalizedScore( currentCheckpoint, aSettings );
        const int passNo = pass + 1;

        if( history.Size() >= AUTOROUTE_BATCH_LOOP::STOP_AT_PASS_MINIMUM
            && passNo >= AUTOROUTE_BATCH_LOOP::STOP_AT_PASS_MINIMUM
            && passNo % AUTOROUTE_BATCH_LOOP::STOP_AT_PASS_MODULO == 0
            && history.MaxScore() > currentScore )
        {
            const auto restored = history.Restore( 3 );
            if( !restored || history.Rank( *restored )
                                > static_cast<int>( BOARD_HISTORY::MAX_HISTORY_SIZE ) )
            {
                break;
            }

            restoreCheckpoint( *restored );
            refreshFailedNets();
            currentCheckpoint = makeCheckpoint();
            currentScore = BOARD_HISTORY::NormalizedScore( currentCheckpoint, aSettings );
            batchLoop.RestoredBoard( currentScore );
        }

        auto decision = batchLoop.Observe(
                passNo, currentScore, currentCheckpoint.metrics.unroutedConnections,
                true, aSettings.enableFanout );

        if( decision.recoverFanout )
        {
            BATCH_OPTIMIZER( board, aSettings, occupancy ).RemoveRedundantViaTails(
                    connections, aCancel );
            refreshFailedNets();
            currentCheckpoint = makeCheckpoint();
            currentScore = BOARD_HISTORY::NormalizedScore( currentCheckpoint, aSettings );
            const auto recoveryDecision =
                    batchLoop.ApplyFanoutRecoveryScore( passNo, currentScore );
            decision.stop = decision.stop || recoveryDecision.stop;
        }

        if( decision.stop )
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
