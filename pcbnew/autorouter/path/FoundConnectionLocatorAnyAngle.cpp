/* This file is part of KiCad, licensed under GPL version 3 or later.
 * Unrestricted-angle connection realization derived from Freerouting
 * FoundConnectionLocatorAnyAngle.java at a11c0a42.
 */
#include "FoundConnectionLocatorAnyAngle.h"

#include <boost/multiprecision/cpp_int.hpp>

#include <algorithm>
#include <tuple>

namespace KICAD_AUTOROUTER
{
namespace
{
using INTEGER = boost::multiprecision::cpp_int;

INTEGER floorDivide( INTEGER aNumerator, const INTEGER& aDenominator )
{
    INTEGER quotient = aNumerator / aDenominator;
    const INTEGER remainder = aNumerator % aDenominator;
    if( remainder != 0 && aNumerator < 0 )
        --quotient;
    return quotient;
}

INTEGER ceilDivide( const INTEGER& aNumerator, const INTEGER& aDenominator )
{
    return -floorDivide( -aNumerator, aDenominator );
}

INTEGER nearestInteger( const INTEGER& aNumerator, const INTEGER& aDenominator )
{
    INTEGER result = floorDivide( aNumerator, aDenominator );
    if( 2 * ( aNumerator - result * aDenominator ) >= aDenominator )
        ++result;
    return result;
}

bool constrainSupport( const PLANAR::LINE& aBorder, ROUTER_POINT aOrigin,
                       const INTEGER& aStepX, const INTEGER& aStepY,
                       INTEGER& aFirst, INTEGER& aLast )
{
    const INTEGER dx = aBorder.Dx();
    const INTEGER dy = aBorder.Dy();
    const INTEGER origin = dy * ( INTEGER( aOrigin.x ) - aBorder.a.x )
                           - dx * ( INTEGER( aOrigin.y ) - aBorder.a.y );
    const INTEGER step = dy * aStepX - dx * aStepY;
    if( step == 0 )
        return origin <= 0;
    if( step > 0 )
        aLast = std::min( aLast, floorDivide( -origin, step ) );
    else
        aFirst = std::max( aFirst, ceilDivide( origin, -step ) );
    return aFirst <= aLast;
}

std::optional<ROUTER_POINT> nearestOnOneDimensionalSimplex(
        const PLANAR::SIMPLEX& aShape, ROUTER_POINT aFrom )
{
    std::optional<PLANAR::LINE> support;
    const auto& borders = aShape.Borders();
    for( std::size_t first = 0; first < borders.size() && !support; ++first )
    {
        for( std::size_t second = first + 1; second < borders.size(); ++second )
        {
            if( borders[first].EqualOrOpposite( borders[second] )
                && !borders[first].SameDirection( borders[second] ) )
            {
                support = borders[first];
                break;
            }
        }
    }
    if( !support )
        return std::nullopt;

    const INTEGER divisor = PLANAR::Gcd( support->Dx(), support->Dy() );
    if( divisor == 0 )
        return std::nullopt;
    const INTEGER stepX = support->Dx() / divisor;
    const INTEGER stepY = support->Dy() / divisor;
    const INTEGER limit = std::numeric_limits<std::int64_t>::max();
    INTEGER first = -limit;
    INTEGER last = limit;
    for( const PLANAR::LINE& border : borders )
    {
        if( !constrainSupport( border, support->a, stepX, stepY, first, last ) )
            return std::nullopt;
    }

    const INTEGER denominator = stepX * stepX + stepY * stepY;
    const INTEGER numerator = ( INTEGER( aFrom.x ) - support->a.x ) * stepX
                              + ( INTEGER( aFrom.y ) - support->a.y ) * stepY;
    const INTEGER index = std::clamp(
            nearestInteger( numerator, denominator ), first, last );
    const INTEGER x = INTEGER( support->a.x ) + index * stepX;
    const INTEGER y = INTEGER( support->a.y ) + index * stepY;
    if( x < std::numeric_limits<std::int64_t>::min()
        || x > std::numeric_limits<std::int64_t>::max()
        || y < std::numeric_limits<std::int64_t>::min()
        || y > std::numeric_limits<std::int64_t>::max() )
    {
        return std::nullopt;
    }
    const ROUTER_POINT result{ x.convert_to<std::int64_t>(),
                               y.convert_to<std::int64_t>() };
    return aShape.Contains( PLANAR::POINT( result ) )
                   ? std::optional( result ) : std::nullopt;
}

void appendPoint( std::vector<ROUTER_POINT>& aPoints, ROUTER_POINT aPoint )
{
    if( aPoints.back() == aPoint )
        return;
    if( aPoints.size() > 1 )
    {
        const ROUTER_POINT a = aPoints[aPoints.size() - 2];
        const ROUTER_POINT b = aPoints.back();
        const INTEGER dx1 = b.x - a.x;
        const INTEGER dy1 = b.y - a.y;
        const INTEGER dx2 = aPoint.x - b.x;
        const INTEGER dy2 = aPoint.y - b.y;
        if( dx1 * dy2 == dy1 * dx2 && dx1 * dx2 + dy1 * dy2 >= 0 )
            aPoints.pop_back();
    }
    aPoints.push_back( aPoint );
}
} // namespace


std::optional<ROUTER_POINT> FOUND_CONNECTION_LOCATOR_ANY_ANGLE::NearestIntegralPoint(
        const PLANAR::SIMPLEX& aShape, ROUTER_POINT aFrom )
{
    const int dimension = aShape.Dimension();
    if( dimension < 0 )
        return std::nullopt;
    if( aShape.Contains( PLANAR::POINT( aFrom ) ) )
        return aFrom;
    if( dimension == 0 )
    {
        if( aShape.Borders().empty() || !aShape.CornerIsBounded( 0 ) )
            return std::nullopt;
        return aShape.Corner( 0 ).Integral();
    }
    if( dimension == 1 )
        return nearestOnOneDimensionalSimplex( aShape, aFrom );

    const auto nearest = aShape.NearestPoint( PLANAR::POINT( aFrom ) );
    if( !nearest )
        return std::nullopt;
    const auto nearestBox = nearest->SurroundingBox();
    if( !nearestBox )
        return std::nullopt;

    std::optional<ROUTER_POINT> best;
    long double bestDistance = std::numeric_limits<long double>::infinity();
    for( int radius = 0; radius <= 4 && !best; ++radius )
    {
        const std::int64_t minX = nearestBox->minX - radius;
        const std::int64_t maxX = nearestBox->maxX + radius;
        const std::int64_t minY = nearestBox->minY - radius;
        const std::int64_t maxY = nearestBox->maxY + radius;
        for( std::int64_t x = minX; x <= maxX; ++x )
        {
            for( std::int64_t y = minY; y <= maxY; ++y )
            {
                if( radius > 0 && x > minX && x < maxX && y > minY && y < maxY )
                    continue;
                const ROUTER_POINT candidate{ x, y };
                if( !aShape.Contains( PLANAR::POINT( candidate ) ) )
                    continue;
                const long double dx = static_cast<long double>( x ) - aFrom.x;
                const long double dy = static_cast<long double>( y ) - aFrom.y;
                const long double distance = dx * dx + dy * dy;
                if( !best || distance < bestDistance
                    || ( distance == bestDistance
                         && std::tie( x, y ) < std::tie( best->x, best->y ) ) )
                {
                    best = candidate;
                    bestDistance = distance;
                }
            }
        }
    }
    return best;
}


std::optional<std::vector<ROUTER_POINT>> FOUND_CONNECTION_LOCATOR_ANY_ANGLE::Locate(
        ROUTER_POINT aStart, const std::vector<GENERAL_CORRIDOR_STEP>& aSteps )
{
    std::vector<ROUTER_POINT> points{ aStart };
    for( const GENERAL_CORRIDOR_STEP& step : aSteps )
    {
        const ROUTER_POINT from = points.back();
        if( !step.room.Contains( PLANAR::POINT( from ) ) )
            return std::nullopt;

        std::optional<ROUTER_POINT> target;
        if( step.door )
            target = NearestIntegralPoint( *step.door, from );
        else
        {
            const ROUTER_POINT requested = step.section.Middle().Round();
            target = step.room.Contains( PLANAR::POINT( requested ) )
                             ? std::optional( requested )
                             : NearestIntegralPoint( step.room, requested );
        }

        if( !target || !step.room.Contains( PLANAR::POINT( *target ) )
            || ( step.door && !step.door->Contains( PLANAR::POINT( *target ) ) ) )
        {
            return std::nullopt;
        }
        appendPoint( points, *target );
    }
    return points;
}

} // namespace KICAD_AUTOROUTER
