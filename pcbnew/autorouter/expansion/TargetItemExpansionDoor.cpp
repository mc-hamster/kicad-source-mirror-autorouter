/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#include "TargetItemExpansionDoor.h"

#include <algorithm>
#include <numeric>

#include <boost/multiprecision/cpp_int.hpp>


namespace KICAD_AUTOROUTER
{
namespace
{
using WIDE = boost::multiprecision::int128_t;


WIDE floorDivide( WIDE aNumerator, WIDE aDenominator )
{
    WIDE quotient = aNumerator / aDenominator;
    const WIDE remainder = aNumerator % aDenominator;
    if( remainder != 0 && aNumerator < 0 )
        --quotient;
    return quotient;
}


WIDE ceilDivide( WIDE aNumerator, WIDE aDenominator )
{
    return -floorDivide( -aNumerator, aDenominator );
}


WIDE nearestInteger( WIDE aNumerator, WIDE aDenominator )
{
    WIDE result = floorDivide( aNumerator, aDenominator );
    const WIDE remainder = aNumerator - result * aDenominator;
    // Java's geometric rounding convention is floor(value + 0.5), including
    // negative values. The lower point therefore wins only below a half tie.
    if( 2 * remainder >= aDenominator )
        ++result;
    return result;
}


bool constrainAxis( std::int64_t aOrigin, std::int64_t aStep,
                    std::int64_t aMinimum, std::int64_t aMaximum,
                    WIDE& aFirst, WIDE& aLast )
{
    if( aStep == 0 )
        return aOrigin >= aMinimum && aOrigin <= aMaximum;

    if( aStep > 0 )
    {
        aFirst = std::max( aFirst, ceilDivide( WIDE( aMinimum ) - aOrigin, aStep ) );
        aLast = std::min( aLast, floorDivide( WIDE( aMaximum ) - aOrigin, aStep ) );
    }
    else
    {
        const WIDE positiveStep = -WIDE( aStep );
        aFirst = std::max( aFirst,
                           ceilDivide( WIDE( aOrigin ) - aMaximum, positiveStep ) );
        aLast = std::min( aLast,
                          floorDivide( WIDE( aOrigin ) - aMinimum, positiveStep ) );
    }

    return aFirst <= aLast;
}
} // namespace


std::optional<ROUTER_POINT> TARGET_ITEM_EXPANSION_DOOR::NearestIntegralPointInRoom(
        const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
        const ROUTER_POINT& aFrom, const ROUTER_BOX& aRoom )
{
    if( aRoom.minX > aRoom.maxX || aRoom.minY > aRoom.maxY )
        return std::nullopt;

    const std::int64_t dx = aEnd.x - aStart.x;
    const std::int64_t dy = aEnd.y - aStart.y;
    const std::int64_t divisor = std::gcd( std::abs( dx ), std::abs( dy ) );

    if( divisor == 0 )
        return aRoom.Contains( aStart ) ? std::optional( aStart ) : std::nullopt;

    const std::int64_t stepX = dx / divisor;
    const std::int64_t stepY = dy / divisor;
    WIDE first = 0;
    WIDE last = divisor;

    if( !constrainAxis( aStart.x, stepX, aRoom.minX, aRoom.maxX, first, last )
        || !constrainAxis( aStart.y, stepY, aRoom.minY, aRoom.maxY, first, last ) )
    {
        return std::nullopt;
    }

    const WIDE denominator = WIDE( stepX ) * stepX + WIDE( stepY ) * stepY;
    const WIDE numerator = ( WIDE( aFrom.x ) - aStart.x ) * stepX
                           + ( WIDE( aFrom.y ) - aStart.y ) * stepY;
    const WIDE index = std::clamp( nearestInteger( numerator, denominator ), first, last );

    return ROUTER_POINT{ ( WIDE( aStart.x ) + index * stepX ).convert_to<std::int64_t>(),
                         ( WIDE( aStart.y ) + index * stepY ).convert_to<std::int64_t>() };
}

} // namespace KICAD_AUTOROUTER
