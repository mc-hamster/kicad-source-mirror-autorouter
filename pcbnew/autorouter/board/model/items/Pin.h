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
};

} // namespace KICAD_AUTOROUTER
