/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Source-faithful data-only helpers translated from Freerouting's Pin.java.
 */
#pragma once

#include "../../../AutorouterTypes.h"
#include "../../../geometry/planar/IntOctagon.h"
#include "../../../geometry/planar/Polyline.h"
#include "../../../geometry/planar/Simplex.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <optional>
#include <vector>

namespace KICAD_AUTOROUTER
{

/** Geometry-only Pin helpers which are safe on the autorouter worker thread. */
class PIN
{
public:
    using TRACE_EXIT_RESTRICTION =
            ROUTING_PAD::LAYER_GEOMETRY::TRACE_EXIT_RESTRICTION;

    /** Return the legal exit corner nearest a prospective drill.
     *
     * This is Pin.nearestTraceExitCorner(): each restriction already carries
     * the distance from the pin centre to its physical border, and aOffset is
     * the source pin-edge-to-turn distance plus compensated trace half-width.
     */
    static std::optional<ROUTER_POINT> NearestTraceExitCorner(
            const ROUTER_POINT& aCenter,
            const std::vector<TRACE_EXIT_RESTRICTION>& aRestrictions,
            const ROUTER_POINT& aFrom, double aOffset )
    {
        std::optional<ROUTER_POINT> nearest;
        long double                nearestDistance =
                std::numeric_limits<long double>::infinity();

        for( const TRACE_EXIT_RESTRICTION& restriction : aRestrictions )
        {
            const long double dx = restriction.direction.x;
            const long double dy = restriction.direction.y;
            const long double directionLength = std::hypotl( dx, dy );

            if( directionLength <= 0 || restriction.minLength < 0 )
                continue;

            const long double distance = restriction.minLength
                                         + std::max( 0.0, aOffset );
            const ROUTER_POINT candidate{
                    static_cast<std::int64_t>( std::llround(
                            aCenter.x + distance * dx / directionLength ) ),
                    static_cast<std::int64_t>( std::llround(
                            aCenter.y + distance * dy / directionLength ) ) };
            const long double candidateDx =
                    static_cast<long double>( candidate.x ) - aFrom.x;
            const long double candidateDy =
                    static_cast<long double>( candidate.y ) - aFrom.y;
            const long double candidateDistance =
                    candidateDx * candidateDx + candidateDy * candidateDy;

            // Source iteration order resolves exact ties.
            if( candidateDistance < nearestDistance )
            {
                nearest = candidate;
                nearestDistance = candidateDistance;
            }
        }

        return nearest;
    }

    /** Freerouting Direction.equals(): collinear and not opposite. */
    static bool MatchesTraceDirection( const TRACE_EXIT_RESTRICTION& aRestriction,
                                       const ROUTER_POINT& aDirection )
    {
        const std::int64_t divisor = std::gcd(
                std::llabs( aDirection.x ), std::llabs( aDirection.y ) );
        return divisor > 0
               && aDirection.x / divisor == aRestriction.direction.x
               && aDirection.y / divisor == aRestriction.direction.y;
    }

    static const ROUTING_PAD::LAYER_GEOMETRY* LayerGeometry(
            const ROUTING_PAD& aPin, int aLayer )
    {
        const auto geometry = std::find_if(
                aPin.layerGeometry.begin(), aPin.layerGeometry.end(),
                [&]( const ROUTING_PAD::LAYER_GEOMETRY& aGeometry )
                {
                    return aGeometry.layer == aLayer;
                } );
        return geometry == aPin.layerGeometry.end() ? nullptr : &*geometry;
    }

    /** PolylineTrace.checkConnectionToPin() for one native route endpoint. */
    static bool CheckConnectionToPin( const ROUTING_CONNECTION& aConnection,
                                      const ROUTING_PAD& aPin, bool aAtStart,
                                      std::int64_t aTraceWidth,
                                      std::int64_t aClearance,
                                      double aEdgeToTurnDistance )
    {
        if( aConnection.nodes.size() < 2 || aTraceWidth <= 0 )
            return true;

        const ROUTER_NODE& end = aAtStart ? aConnection.nodes.front()
                                          : aConnection.nodes.back();
        const ROUTER_NODE& previous = aAtStart ? aConnection.nodes[1]
                                               : aConnection.nodes[aConnection.nodes.size() - 2];
        if( end.point != aPin.position || end.layer != previous.layer )
            return true;

        const ROUTING_PAD::LAYER_GEOMETRY* geometry =
                LayerGeometry( aPin, end.layer );
        if( !geometry || geometry->traceExitRestrictions.empty() )
            return true;

        const ROUTER_POINT direction{ previous.point.x - end.point.x,
                                      previous.point.y - end.point.y };
        if( direction.x == 0 && direction.y == 0 )
            return true;

        const auto restriction = std::find_if(
                geometry->traceExitRestrictions.begin(),
                geometry->traceExitRestrictions.end(),
                [&]( const TRACE_EXIT_RESTRICTION& aRestriction )
                {
                    return MatchesTraceDirection( aRestriction, direction );
                } );
        if( restriction == geometry->traceExitRestrictions.end() )
            return false;
        if( aEdgeToTurnDistance < 0 )
            return false;

        const long double length = std::hypotl(
                static_cast<long double>( direction.x ),
                static_cast<long double>( direction.y ) );
        const double additional = std::max(
                std::max( 0.0, aEdgeToTurnDistance ),
                static_cast<double>( std::max<std::int64_t>( 0, aClearance ) ) + 1.0 );
        const double preserveLength = restriction->minLength
                                      + static_cast<double>( aTraceWidth ) / 2.0
                                      + additional;
        return length >= preserveLength;
    }

    /** Correct one restricted route endpoint and emit its SHOVE_FIXED stub.
     *
     * This is PolylineTrace.correctConnectionToPin(): enlarge the exact pin
     * contour, locate the last trace entrance, select the nearest legal exit
     * with source tie order, and walk the shorter intervening border.  The
     * worker publishes only an entirely integral replacement because KiCad
     * cannot materialize Freerouting's rational Polyline corners.
     */
    static bool CorrectConnectionToPin( ROUTING_CONNECTION& aConnection,
                                        const ROUTING_PAD& aPin,
                                        const BOARD_SNAPSHOT& aBoard,
                                        bool aAtStart,
                                        std::int64_t aTraceWidth,
                                        std::int64_t aClearance,
                                        double aEdgeToTurnDistance )
    {
        if( CheckConnectionToPin( aConnection, aPin, aAtStart, aTraceWidth,
                                  aClearance, aEdgeToTurnDistance )
            || aConnection.nodes.size() < 2 || !HasValidEdgeStyles( aConnection )
            || aEdgeToTurnDistance < 0 )
        {
            return false;
        }

        ROUTING_CONNECTION oriented = aConnection;
        EnsureEdgeStyles( oriented );
        if( !aAtStart )
        {
            std::reverse( oriented.nodes.begin(), oriented.nodes.end() );
            std::reverse( oriented.edgeStyles.begin(), oriented.edgeStyles.end() );
        }

        const ROUTER_NODE& pinNode = oriented.nodes.front();
        const ROUTER_NODE& adjacent = oriented.nodes[1];
        const ROUTING_PAD::LAYER_GEOMETRY* geometry =
                LayerGeometry( aPin, pinNode.layer );
        if( !geometry || geometry->traceExitRestrictions.empty()
            || pinNode.layer != adjacent.layer )
        {
            return false;
        }

        const double additional = std::max(
                std::max( 0.0, aEdgeToTurnDistance ),
                static_cast<double>( std::max<std::int64_t>( 0, aClearance ) ) + 1.0 );

        // A native connection may continue through a via, while the source
        // operation changes exactly one PolylineTrace.  Restrict the shape
        // walk to the same-layer run which starts at the contacted pin.
        std::vector<ROUTER_POINT> tracePoints;
        tracePoints.reserve( oriented.nodes.size() );
        tracePoints.push_back( pinNode.point );
        for( std::size_t index = 1; index < oriented.nodes.size(); ++index )
        {
            if( oriented.nodes[index].layer != pinNode.layer )
                break;
            tracePoints.push_back( oriented.nodes[index].point );
        }
        if( tracePoints.size() < 2 )
            return false;

        PLANAR::POLYLINE tracePolyline = PLANAR::POLYLINE::FromPoints( tracePoints );
        if( tracePolyline.Empty() )
            return false;

        const auto pinShape = [&]() -> std::optional<PLANAR::SIMPLEX>
        {
            if( geometry->copperShapeIndices.size() != 1 )
                return {};
            const std::size_t shapeIndex = geometry->copperShapeIndices.front();
            if( shapeIndex >= aBoard.obstacles.size() )
                return {};
            const ROUTING_OBSTACLE& copper = aBoard.obstacles[shapeIndex];
            if( !copper.isPad
                || std::find( copper.layers.begin(), copper.layers.end(),
                              pinNode.layer ) == copper.layers.end()
                || ( !aPin.sourceId.empty()
                     && copper.boardItemId != aPin.sourceId ) )
            {
                return {};
            }
            if( copper.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
                return PLANAR::SIMPLEX::FromBox( copper.box );
            if( copper.kind == ROUTER_OBSTACLE_KIND::POLYGON )
                return PLANAR::SIMPLEX::FromConvexPolygon( copper.polygon );
            return {};
        }();
        if( !pinShape || pinShape->IsEmpty() || !pinShape->IsBounded() )
            return false;

        auto offsetPinShape = pinShape->Offset(
                static_cast<double>( aTraceWidth ) / 2.0 + additional );
        if( !offsetPinShape || offsetPinShape->IsEmpty()
            || !offsetPinShape->IsBounded() )
        {
            return false;
        }

        // The native room router's primary fixed-angle domain is 45 degrees;
        // its 90-degree search is only a transition fallback.  Preserve the
        // source's shape simplification in that domain: boxes remain boxes,
        // octagons remain octagons, and arbitrary rotations retain their
        // exact convex supports for the any-angle room path.
        if( offsetPinShape->IsIntBox() )
        {
            const auto bounds = offsetPinShape->BoundingBox();
            if( !bounds )
                return false;
            offsetPinShape = PLANAR::SIMPLEX::FromBox( *bounds );
        }
        else if( offsetPinShape->IsIntOctagon() )
        {
            const auto octagon = offsetPinShape->BoundingOctagon();
            if( !octagon )
                return false;
            offsetPinShape = octagon->ToSimplex();
            if( !offsetPinShape )
                return false;
        }

        const auto entries = offsetPinShape->EntrancePoints( tracePolyline );
        if( entries.empty() )
            return false;
        const auto [entryLineIndex, entryBorderIndex] = entries.back();
        if( entryLineIndex == 0 || entryLineIndex >= tracePolyline.lines.size()
            || entryLineIndex - 1 >= oriented.edgeStyles.size()
            || entryBorderIndex >= offsetPinShape->Borders().size() )
        {
            return false;
        }

        const auto entryPoint = tracePolyline.lines[entryLineIndex].Intersection(
                offsetPinShape->Borders()[entryBorderIndex] );
        if( !entryPoint )
            return false;

        const FLOAT_POINT entryApprox{ entryPoint->X(), entryPoint->Y() };
        double nearestDistance = std::numeric_limits<double>::max();
        std::optional<PLANAR::LINE> nearestExitRay;
        std::optional<PLANAR::POINT> nearestExitCorner;
        std::size_t nearestBorderIndex = 0;
        constexpr double tolerance = 1.0;
        for( const TRACE_EXIT_RESTRICTION& restriction :
             geometry->traceExitRestrictions )
        {
            const int borderIndex = offsetPinShape->IntersectingBorderLineNo(
                    PLANAR::POINT( pinNode.point ), restriction.direction );
            const auto ray = PLANAR::LINE::FromDirection(
                    pinNode.point, restriction.direction.x,
                    restriction.direction.y );
            if( borderIndex < 0 || !ray )
                continue;
            const auto corner = ray->Intersection(
                    offsetPinShape->Borders()[static_cast<std::size_t>( borderIndex )] );
            if( !corner )
                continue;

            const double dx = corner->X() - entryApprox.x;
            const double dy = corner->Y() - entryApprox.y;
            const double distance = dx * dx + dy * dy;
            bool replace = distance + tolerance < nearestDistance;
            if( !replace && nearestExitCorner
                && distance < nearestDistance + tolerance )
            {
                // Preserve the source's deterministic near-tie walk over the
                // remaining trace corners rather than relying on container or
                // floating sort order.
                for( std::size_t index = 1;
                     index < tracePolyline.CornerCount(); ++index )
                {
                    const PLANAR::POINT traceCorner = tracePolyline.Corner( index );
                    const double currentDistance = traceCorner.DistanceSquared( *corner );
                    const double oldDistance =
                            traceCorner.DistanceSquared( *nearestExitCorner );
                    if( currentDistance + tolerance < oldDistance )
                    {
                        replace = true;
                        break;
                    }
                    if( currentDistance > oldDistance + tolerance )
                        break;
                }
            }
            if( replace || !nearestExitCorner )
            {
                nearestDistance = distance;
                nearestExitRay = *ray;
                nearestExitCorner = *corner;
                nearestBorderIndex = static_cast<std::size_t>( borderIndex );
            }
        }
        if( !nearestExitRay || !nearestExitCorner )
            return false;

        // Append the shorter piece of the enlarged pad border between the
        // legal exit ray and the trace's last entrance.  This is the part the
        // former direct-stub approximation omitted, allowing an immediate
        // turn through the protected pin-edge-to-turn envelope.
        const std::vector<PLANAR::LINE>& borders = offsetPinShape->Borders();
        const std::size_t borderCount = borders.size();
        const std::size_t clockwise =
                ( nearestBorderIndex + borderCount - entryBorderIndex ) % borderCount;
        const std::size_t counterClockwise =
                ( entryBorderIndex + borderCount - nearestBorderIndex ) % borderCount;
        std::vector<PLANAR::LINE> borderLines;
        borderLines.reserve( std::min( clockwise, counterClockwise ) + 3 );
        borderLines.push_back( *nearestExitRay );
        std::size_t borderIndex = nearestBorderIndex;
        if( counterClockwise <= clockwise )
        {
            for( std::size_t index = 0; index <= counterClockwise; ++index )
            {
                borderLines.push_back( borders[borderIndex] );
                borderIndex = ( borderIndex + 1 ) % borderCount;
            }
        }
        else
        {
            for( std::size_t index = 0; index <= clockwise; ++index )
            {
                borderLines.push_back( borders[borderIndex] );
                borderIndex = ( borderIndex + borderCount - 1 ) % borderCount;
            }
        }
        borderLines.push_back( tracePolyline.lines[entryLineIndex] );

        const PLANAR::POLYLINE borderPolyline( std::move( borderLines ) );
        const auto borderCorners = borderPolyline.IntegralCorners();
        // Rational corners are valid in Freerouting's board model but cannot
        // be represented as KiCad copper.  Fail closed instead of rounding a
        // route through an offset support line.
        if( !borderCorners || borderCorners->size() < 2
            || borderCorners->front() == pinNode.point )
        {
            return false;
        }

        ROUTING_EDGE_STYLE exitStyle = oriented.edgeStyles.front();
        exitStyle.trackWidth = aTraceWidth;
        exitStyle.clearance = std::max<std::int64_t>( 0, aClearance );
        exitStyle.fixedState = ROUTER_FIXED_STATE::SHOVE_FIXED;
        ROUTING_EDGE_STYLE borderStyle = oriented.edgeStyles[entryLineIndex - 1];
        borderStyle.trackWidth = aTraceWidth;
        borderStyle.clearance = std::max<std::int64_t>( 0, aClearance );

        std::vector<ROUTER_NODE> nodes{ pinNode };
        std::vector<ROUTING_EDGE_STYLE> styles;
        const auto appendEdge = [&]( const ROUTER_POINT& aPoint,
                                     const ROUTING_EDGE_STYLE& aStyle )
        {
            if( nodes.back().point == aPoint )
                return;
            nodes.push_back( { aPoint, pinNode.layer } );
            styles.push_back( aStyle );
        };
        appendEdge( borderCorners->front(), exitStyle );
        for( std::size_t index = 1; index < borderCorners->size(); ++index )
            appendEdge( ( *borderCorners )[index], borderStyle );

        const std::size_t entryEdge = entryLineIndex - 1;
        for( std::size_t edge = entryEdge; edge < oriented.edgeStyles.size(); ++edge )
        {
            if( edge + 1 >= oriented.nodes.size() )
                return false;
            if( nodes.back() == oriented.nodes[edge + 1] )
                continue;
            nodes.push_back( oriented.nodes[edge + 1] );
            styles.push_back( oriented.edgeStyles[edge] );
        }

        if( nodes.size() < 2 || styles.size() + 1 != nodes.size() )
            return false;
        if( !aAtStart )
        {
            std::reverse( nodes.begin(), nodes.end() );
            std::reverse( styles.begin(), styles.end() );
        }
        aConnection.nodes = std::move( nodes );
        aConnection.edgeStyles = std::move( styles );

        return CheckConnectionToPin( aConnection, aPin, aAtStart, aTraceWidth,
                                     aClearance, aEdgeToTurnDistance );
    }
};

} // namespace KICAD_AUTOROUTER
