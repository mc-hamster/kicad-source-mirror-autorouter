/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Source-faithful data-only helpers translated from Freerouting's Pin.java.
 */
#pragma once

#include "../../../AutorouterTypes.h"

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
     * The complete source method can walk an arbitrary TileShape border.
     * Rectangular padstacks are the only shapes for which the KiCad Specctra
     * path creates restrictions, so the stored ray/border distance is enough
     * to construct the same mandatory straight exit before reconnecting the
     * already-located route.
     */
    static bool CorrectConnectionToPin( ROUTING_CONNECTION& aConnection,
                                        const ROUTING_PAD& aPin, bool aAtStart,
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

        const ROUTER_NODE& pinNode = aAtStart ? aConnection.nodes.front()
                                              : aConnection.nodes.back();
        const ROUTER_NODE& adjacent = aAtStart ? aConnection.nodes[1]
                                               : aConnection.nodes[aConnection.nodes.size() - 2];
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
        const auto exit = NearestTraceExitCorner(
                pinNode.point, geometry->traceExitRestrictions, adjacent.point,
                static_cast<double>( aTraceWidth ) / 2.0 + additional );
        if( !exit || *exit == pinNode.point )
            return false;

        EnsureEdgeStyles( aConnection );
        ROUTING_EDGE_STYLE exitStyle = aAtStart ? aConnection.edgeStyles.front()
                                                : aConnection.edgeStyles.back();
        exitStyle.trackWidth = aTraceWidth;
        exitStyle.clearance = std::max<std::int64_t>( 0, aClearance );
        exitStyle.fixedState = ROUTER_FIXED_STATE::SHOVE_FIXED;
        const ROUTER_NODE exitNode{ *exit, pinNode.layer };

        if( aAtStart )
        {
            if( exitNode == aConnection.nodes[1] )
                aConnection.edgeStyles.front() = exitStyle;
            else
            {
                aConnection.nodes.insert( aConnection.nodes.begin() + 1, exitNode );
                aConnection.edgeStyles.insert( aConnection.edgeStyles.begin(), exitStyle );
            }
        }
        else
        {
            if( exitNode == aConnection.nodes[aConnection.nodes.size() - 2] )
                aConnection.edgeStyles.back() = exitStyle;
            else
            {
                aConnection.nodes.insert( aConnection.nodes.end() - 1, exitNode );
                aConnection.edgeStyles.push_back( exitStyle );
            }
        }

        return CheckConnectionToPin( aConnection, aPin, aAtStart, aTraceWidth,
                                     aClearance, aEdgeToTurnDistance );
    }
};

} // namespace KICAD_AUTOROUTER
