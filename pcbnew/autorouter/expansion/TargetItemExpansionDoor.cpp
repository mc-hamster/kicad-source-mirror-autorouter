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
#include <set>

#include <boost/multiprecision/cpp_int.hpp>


namespace KICAD_AUTOROUTER
{
namespace
{
using WIDE = boost::multiprecision::cpp_int;


WIDE absolute( WIDE aValue )
{
    return aValue < 0 ? -aValue : aValue;
}


WIDE greatestCommonDivisor( WIDE aLeft, WIDE aRight )
{
    aLeft = absolute( aLeft );
    aRight = absolute( aRight );

    while( aRight != 0 )
    {
        const WIDE remainder = aLeft % aRight;
        aLeft = aRight;
        aRight = remainder;
    }

    return aLeft;
}


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


bool constrainAxis( const WIDE& aOrigin, const WIDE& aStep,
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
        const WIDE positiveStep = -aStep;
        aFirst = std::max( aFirst,
                           ceilDivide( WIDE( aOrigin ) - aMaximum, positiveStep ) );
        aLast = std::min( aLast,
                          floorDivide( WIDE( aOrigin ) - aMinimum, positiveStep ) );
    }

    return aFirst <= aLast;
}


bool constrainNonPositive( const WIDE& aOrigin, const WIDE& aStep,
                           WIDE& aFirst, WIDE& aLast )
{
    if( aStep == 0 )
        return aOrigin <= 0;

    if( aStep > 0 )
        aLast = std::min( aLast, floorDivide( -aOrigin, aStep ) );
    else
        aFirst = std::max( aFirst, ceilDivide( aOrigin, -aStep ) );

    return aFirst <= aLast;
}


void addCutIndices( const WIDE& aOrigin, const WIDE& aStep,
                    std::int64_t aCoordinate, const WIDE& aLast,
                    std::set<WIDE>& aIndices )
{
    if( aStep == 0 )
        return;

    WIDE numerator = WIDE( aCoordinate ) - aOrigin;
    WIDE denominator = aStep;
    if( denominator < 0 )
    {
        numerator = -numerator;
        denominator = -denominator;
    }

    const WIDE lower = floorDivide( numerator, denominator );
    const WIDE upper = ceilDivide( numerator, denominator );
    for( const WIDE& base : { lower, upper } )
    {
        for( int offset = -1; offset <= 1; ++offset )
        {
            const WIDE candidate = base + offset;
            if( candidate >= 0 && candidate <= aLast )
                aIndices.insert( candidate );
        }
    }
}
} // namespace


std::optional<ROUTER_POINT> TARGET_ITEM_EXPANSION_DOOR::NearestIntegralPointInRoom(
        const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
        const ROUTER_POINT& aFrom, const ROUTER_BOX& aRoom )
{
    if( aRoom.minX > aRoom.maxX || aRoom.minY > aRoom.maxY )
        return std::nullopt;

    const WIDE dx = WIDE( aEnd.x ) - aStart.x;
    const WIDE dy = WIDE( aEnd.y ) - aStart.y;
    const WIDE divisor = greatestCommonDivisor( dx, dy );

    if( divisor == 0 )
        return aRoom.Contains( aStart ) ? std::optional( aStart ) : std::nullopt;

    const WIDE stepX = dx / divisor;
    const WIDE stepY = dy / divisor;
    WIDE first = 0;
    WIDE last = divisor;

    if( !constrainAxis( aStart.x, stepX, aRoom.minX, aRoom.maxX, first, last )
        || !constrainAxis( aStart.y, stepY, aRoom.minY, aRoom.maxY, first, last ) )
    {
        return std::nullopt;
    }

    const WIDE denominator = stepX * stepX + stepY * stepY;
    const WIDE numerator = ( WIDE( aFrom.x ) - aStart.x ) * stepX
                           + ( WIDE( aFrom.y ) - aStart.y ) * stepY;
    const WIDE index = std::clamp( nearestInteger( numerator, denominator ), first, last );

    return ROUTER_POINT{ ( WIDE( aStart.x ) + index * stepX ).convert_to<std::int64_t>(),
                         ( WIDE( aStart.y ) + index * stepY ).convert_to<std::int64_t>() };
}


std::optional<ROUTER_POINT> TARGET_ITEM_EXPANSION_DOOR::NearestIntegralPointInRoom(
        const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
        const ROUTER_POINT& aFrom, const PLANAR::SIMPLEX& aRoom )
{
    if( aRoom.Dimension() < 0 )
        return std::nullopt;

    const WIDE dx = WIDE( aEnd.x ) - aStart.x;
    const WIDE dy = WIDE( aEnd.y ) - aStart.y;
    const WIDE divisor = greatestCommonDivisor( dx, dy );

    if( divisor == 0 )
        return aRoom.Contains( PLANAR::POINT( aStart ) )
                       ? std::optional( aStart ) : std::nullopt;

    const WIDE stepX = dx / divisor;
    const WIDE stepY = dy / divisor;
    WIDE first = 0;
    WIDE last = divisor;

    // SIMPLEX interiors are the right/non-positive side of every directed
    // support.  Substitute p(k)=start+k*step and solve the resulting exact
    // linear inequality for the integral lattice index k.
    for( const PLANAR::LINE& border : aRoom.Borders() )
    {
        const WIDE borderDx = border.Dx();
        const WIDE borderDy = border.Dy();
        const WIDE origin = borderDy * ( WIDE( aStart.x ) - border.a.x )
                            - borderDx * ( WIDE( aStart.y ) - border.a.y );
        const WIDE step = borderDy * stepX - borderDx * stepY;
        if( !constrainNonPositive( origin, step, first, last ) )
            return std::nullopt;
    }

    const WIDE denominator = stepX * stepX + stepY * stepY;
    const WIDE numerator = ( WIDE( aFrom.x ) - aStart.x ) * stepX
                           + ( WIDE( aFrom.y ) - aStart.y ) * stepY;
    const WIDE index = std::clamp( nearestInteger( numerator, denominator ), first, last );
    const ROUTER_POINT result{
            ( WIDE( aStart.x ) + index * stepX ).convert_to<std::int64_t>(),
            ( WIDE( aStart.y ) + index * stepY ).convert_to<std::int64_t>() };
    return aRoom.Contains( PLANAR::POINT( result ) )
                   ? std::optional( result ) : std::nullopt;
}


std::optional<ROUTER_POINT> TARGET_ITEM_EXPANSION_DOOR::NearestIntegralPointInRoom(
        const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
        const ROUTER_POINT& aFrom, const PLANAR::INT_OCTAGON& aRoom )
{
    if( aRoom.Dimension() < 0 )
        return std::nullopt;

    const WIDE dx = WIDE( aEnd.x ) - aStart.x;
    const WIDE dy = WIDE( aEnd.y ) - aStart.y;
    const WIDE divisor = greatestCommonDivisor( dx, dy );

    if( divisor == 0 )
        return aRoom.Contains( aStart ) ? std::optional( aStart ) : std::nullopt;

    const WIDE stepX = dx / divisor;
    const WIDE stepY = dy / divisor;
    WIDE first = 0;
    WIDE last = divisor;

    if( !constrainAxis( aStart.x, stepX, aRoom.leftX, aRoom.rightX, first, last )
        || !constrainAxis( aStart.y, stepY, aRoom.bottomY, aRoom.topY, first, last )
        || !constrainAxis( WIDE( aStart.x ) - aStart.y, stepX - stepY,
                           aRoom.upperLeftDiagonalX, aRoom.lowerRightDiagonalX,
                           first, last )
        || !constrainAxis( WIDE( aStart.x ) + aStart.y, stepX + stepY,
                           aRoom.lowerLeftDiagonalX, aRoom.upperRightDiagonalX,
                           first, last ) )
    {
        return std::nullopt;
    }

    const WIDE denominator = stepX * stepX + stepY * stepY;
    const WIDE numerator = ( WIDE( aFrom.x ) - aStart.x ) * stepX
                           + ( WIDE( aFrom.y ) - aStart.y ) * stepY;
    const WIDE index = std::clamp( nearestInteger( numerator, denominator ), first, last );
    return ROUTER_POINT{ ( WIDE( aStart.x ) + index * stepX ).convert_to<std::int64_t>(),
                         ( WIDE( aStart.y ) + index * stepY ).convert_to<std::int64_t>() };
}


std::vector<ROUTER_POINT> TARGET_ITEM_EXPANSION_DOOR::IntegralRoomSeedPoints(
        const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
        const std::vector<ROUTER_BOX>& aOrthogonalCuts )
{
    const WIDE dx = WIDE( aEnd.x ) - aStart.x;
    const WIDE dy = WIDE( aEnd.y ) - aStart.y;
    const WIDE divisor = greatestCommonDivisor( dx, dy );
    if( divisor == 0 )
        return { aStart };

    const WIDE stepX = dx / divisor;
    const WIDE stepY = dy / divisor;
    std::set<WIDE> indices{ 0, divisor };
    if( divisor > 1 )
    {
        indices.insert( 1 );
        indices.insert( divisor - 1 );
    }

    for( const ROUTER_BOX& cut : aOrthogonalCuts )
    {
        if( cut.minX > cut.maxX || cut.minY > cut.maxY )
            continue;

        addCutIndices( aStart.x, stepX, cut.minX, divisor, indices );
        addCutIndices( aStart.x, stepX, cut.maxX, divisor, indices );
        addCutIndices( aStart.y, stepY, cut.minY, divisor, indices );
        addCutIndices( aStart.y, stepY, cut.maxY, divisor, indices );
    }

    std::vector<ROUTER_POINT> result;
    result.reserve( indices.size() );
    for( const WIDE& index : indices )
    {
        result.push_back( {
                ( WIDE( aStart.x ) + index * stepX ).convert_to<std::int64_t>(),
                ( WIDE( aStart.y ) + index * stepY ).convert_to<std::int64_t>() } );
    }
    return result;
}


std::vector<ROUTER_POINT> TARGET_ITEM_EXPANSION_DOOR::IntegralRoomSeedPoints(
        const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
        const std::vector<PLANAR::SIMPLEX>& aAnyAngleCuts )
{
    const WIDE dx = WIDE( aEnd.x ) - aStart.x;
    const WIDE dy = WIDE( aEnd.y ) - aStart.y;
    const WIDE divisor = greatestCommonDivisor( dx, dy );
    if( divisor == 0 )
        return { aStart };

    const WIDE stepX = dx / divisor;
    const WIDE stepY = dy / divisor;
    std::set<WIDE> indices{ 0, divisor };
    if( divisor > 1 )
    {
        indices.insert( 1 );
        indices.insert( divisor - 1 );
    }

    // A segment can enter or leave a convex room only at one of its support
    // lines.  Sample the lattice points bracketing every exact crossing; this
    // is the unrestricted equivalent of the x/y/diagonal cut enumeration.
    for( const PLANAR::SIMPLEX& cut : aAnyAngleCuts )
    {
        for( const PLANAR::LINE& border : cut.Borders() )
        {
            const WIDE borderDx = border.Dx();
            const WIDE borderDy = border.Dy();
            WIDE numerator = -( borderDy * ( WIDE( aStart.x ) - border.a.x )
                                - borderDx * ( WIDE( aStart.y ) - border.a.y ) );
            WIDE denominator = borderDy * stepX - borderDx * stepY;
            if( denominator == 0 )
                continue;
            if( denominator < 0 )
            {
                numerator = -numerator;
                denominator = -denominator;
            }
            const WIDE lower = floorDivide( numerator, denominator );
            const WIDE upper = ceilDivide( numerator, denominator );
            for( const WIDE& base : { lower, upper } )
            {
                for( int offset = -1; offset <= 1; ++offset )
                {
                    const WIDE candidate = base + offset;
                    if( candidate >= 0 && candidate <= divisor )
                        indices.insert( candidate );
                }
            }
        }
    }

    std::vector<ROUTER_POINT> result;
    result.reserve( indices.size() );
    for( const WIDE& index : indices )
    {
        result.push_back( {
                ( WIDE( aStart.x ) + index * stepX ).convert_to<std::int64_t>(),
                ( WIDE( aStart.y ) + index * stepY ).convert_to<std::int64_t>() } );
    }
    return result;
}


std::vector<ROUTER_POINT> TARGET_ITEM_EXPANSION_DOOR::IntegralRoomSeedPoints(
        const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
        const std::vector<PLANAR::INT_OCTAGON>& aFortyFiveDegreeCuts )
{
    const WIDE dx = WIDE( aEnd.x ) - aStart.x;
    const WIDE dy = WIDE( aEnd.y ) - aStart.y;
    const WIDE divisor = greatestCommonDivisor( dx, dy );
    if( divisor == 0 )
        return { aStart };

    const WIDE stepX = dx / divisor;
    const WIDE stepY = dy / divisor;
    std::set<WIDE> indices{ 0, divisor };
    if( divisor > 1 )
    {
        indices.insert( 1 );
        indices.insert( divisor - 1 );
    }

    for( const PLANAR::INT_OCTAGON& cut : aFortyFiveDegreeCuts )
    {
        if( cut.Dimension() < 0 )
            continue;

        addCutIndices( aStart.x, stepX, cut.leftX, divisor, indices );
        addCutIndices( aStart.x, stepX, cut.rightX, divisor, indices );
        addCutIndices( aStart.y, stepY, cut.bottomY, divisor, indices );
        addCutIndices( aStart.y, stepY, cut.topY, divisor, indices );
        addCutIndices( WIDE( aStart.x ) - aStart.y, stepX - stepY,
                       cut.upperLeftDiagonalX, divisor, indices );
        addCutIndices( WIDE( aStart.x ) - aStart.y, stepX - stepY,
                       cut.lowerRightDiagonalX, divisor, indices );
        addCutIndices( WIDE( aStart.x ) + aStart.y, stepX + stepY,
                       cut.lowerLeftDiagonalX, divisor, indices );
        addCutIndices( WIDE( aStart.x ) + aStart.y, stepX + stepY,
                       cut.upperRightDiagonalX, divisor, indices );
    }

    std::vector<ROUTER_POINT> result;
    result.reserve( indices.size() );
    for( const WIDE& index : indices )
    {
        result.push_back( {
                ( WIDE( aStart.x ) + index * stepX ).convert_to<std::int64_t>(),
                ( WIDE( aStart.y ) + index * stepY ).convert_to<std::int64_t>() } );
    }
    return result;
}

} // namespace KICAD_AUTOROUTER
