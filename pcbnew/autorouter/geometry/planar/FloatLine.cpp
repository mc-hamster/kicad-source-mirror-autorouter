/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct C++ translation of Freerouting geometry/planar/{FloatPoint,
 * FloatLine}.java at a11c0a42d1b3827e5126429c5c9820c4ab5bec7c.
 */
#include "FloatLine.h"

#include "IntOctagon.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace KICAD_AUTOROUTER
{
namespace
{

// Freerouting's Limits.CRIT_INT is expressed in source routing units.  The
// native engine stores geometry in KiCad IU, so retain the same physical
// range at this adapter boundary rather than rejecting ordinary board
// coordinates above 33.5 mm.
constexpr double CRITICAL_INTEGER =
        33554432.0 * FREEROUTING_COORDINATE_UNIT_IU;

std::int64_t javaRound( double aValue )
{
    const double rounded = std::floor( aValue + 0.5 );
    if( std::isnan( rounded ) )
        return 0;
    return static_cast<std::int64_t>( std::clamp(
            rounded,
            static_cast<double>( std::numeric_limits<std::int64_t>::min() ),
            static_cast<double>( std::numeric_limits<std::int64_t>::max() ) ) );
}

std::int64_t javaIntegerCast( double aValue )
{
    if( std::isnan( aValue ) )
        return 0;
    return static_cast<std::int64_t>( std::clamp( aValue, -2147483648.0,
                                                  2147483647.0 ) );
}

int sideOf( double aValue )
{
    return aValue > 0 ? 1 : aValue < 0 ? -1 : 0;
}

} // namespace


PLANAR::INT_OCTAGON FLOAT_POINT::BoundingOctagon(
        const std::vector<FLOAT_POINT>& aPoints )
{
    double minimumX = std::numeric_limits<std::int32_t>::max();
    double minimumY = std::numeric_limits<std::int32_t>::max();
    double maximumX = std::numeric_limits<std::int32_t>::min();
    double maximumY = std::numeric_limits<std::int32_t>::min();
    double minimumUpperLeft = minimumX;
    double maximumLowerRight = maximumX;
    double minimumLowerLeft = minimumX;
    double maximumUpperRight = maximumX;
    for( const FLOAT_POINT point : aPoints )
    {
        minimumX = std::min( minimumX, point.x );
        minimumY = std::min( minimumY, point.y );
        maximumX = std::max( maximumX, point.x );
        maximumY = std::max( maximumY, point.y );
        minimumUpperLeft = std::min( minimumUpperLeft, point.x - point.y );
        maximumLowerRight = std::max( maximumLowerRight, point.x - point.y );
        minimumLowerLeft = std::min( minimumLowerLeft, point.x + point.y );
        maximumUpperRight = std::max( maximumUpperRight, point.x + point.y );
    }
    return PLANAR::INT_OCTAGON(
            javaIntegerCast( std::floor( minimumX ) ),
            javaIntegerCast( std::floor( minimumY ) ),
            javaIntegerCast( std::ceil( maximumX ) ),
            javaIntegerCast( std::ceil( maximumY ) ),
            javaIntegerCast( std::floor( minimumUpperLeft ) ),
            javaIntegerCast( std::ceil( maximumLowerRight ) ),
            javaIntegerCast( std::floor( minimumLowerLeft ) ),
            javaIntegerCast( std::ceil( maximumUpperRight ) ) );
}


double FLOAT_POINT::SizeSquared() const
{
    return x * x + y * y;
}


double FLOAT_POINT::Size() const
{
    return std::sqrt( SizeSquared() );
}


double FLOAT_POINT::DistanceSquared( FLOAT_POINT aOther ) const
{
    const double dx = aOther.x - x;
    const double dy = aOther.y - y;
    return dx * dx + dy * dy;
}


double FLOAT_POINT::Distance( FLOAT_POINT aOther ) const
{
    return std::sqrt( DistanceSquared( aOther ) );
}


double FLOAT_POINT::WeightedDistance(
        FLOAT_POINT aOther, double aHorizontal, double aVertical ) const
{
    const double dx = ( x - aOther.x ) * aHorizontal;
    const double dy = ( y - aOther.y ) * aVertical;
    return std::sqrt( dx * dx + dy * dy );
}


ROUTER_POINT FLOAT_POINT::Round() const
{
    return { javaRound( x ), javaRound( y ) };
}


ROUTER_POINT FLOAT_POINT::RoundToGridJava( std::int64_t aGrid ) const
{
    if( aGrid <= 1 )
        return Round();
    const auto roundOnGrid = [&]( double aValue )
    {
        const long double unit = aGrid;
        const long double rounded = std::floor(
                static_cast<long double>( aValue ) / unit + 0.5L ) * unit;
        return static_cast<std::int64_t>( std::clamp(
                rounded,
                static_cast<long double>( std::numeric_limits<std::int64_t>::min() ),
                static_cast<long double>( std::numeric_limits<std::int64_t>::max() ) ) );
    };
    return { roundOnGrid( x ), roundOnGrid( y ) };
}


ROUTER_POINT FLOAT_POINT::RoundToGridJavaYDown( std::int64_t aGrid ) const
{
    const ROUTER_POINT source = FLOAT_POINT{ x, -y }.RoundToGridJava( aGrid );
    return { source.x, -source.y };
}


ROUTER_POINT FLOAT_POINT::RoundToSourceGrid() const
{
    return RoundToGridJava( static_cast<std::int64_t>(
            FREEROUTING_COORDINATE_UNIT_IU ) );
}


ROUTER_POINT FLOAT_POINT::RoundToSourceGridYDown() const
{
    return RoundToGridJavaYDown( static_cast<std::int64_t>(
            FREEROUTING_COORDINATE_UNIT_IU ) );
}


ROUTER_POINT FLOAT_POINT::RoundToTheRight( ROUTER_POINT aDirection ) const
{
    const std::int64_t roundedX = aDirection.y > 0 ? javaIntegerCast( std::ceil( x ) )
                                : aDirection.y < 0 ? javaIntegerCast( std::floor( x ) )
                                                   : javaRound( x );
    const std::int64_t roundedY = aDirection.x > 0 ? javaIntegerCast( std::floor( y ) )
                                : aDirection.x < 0 ? javaIntegerCast( std::ceil( y ) )
                                                   : javaRound( y );
    return { roundedX, roundedY };
}


ROUTER_POINT FLOAT_POINT::RoundToGrid(
        std::int64_t aHorizontalGrid, std::int64_t aVerticalGrid ) const
{
    const double roundedX = aHorizontalGrid > 0
                            ? std::nearbyint( x / aHorizontalGrid ) * aHorizontalGrid : x;
    const double roundedY = aVerticalGrid > 0
                            ? std::nearbyint( y / aVerticalGrid ) * aVerticalGrid : y;
    return { javaIntegerCast( roundedX ), javaIntegerCast( roundedY ) };
}


ROUTER_POINT FLOAT_POINT::RoundToTheLeft( ROUTER_POINT aDirection ) const
{
    const std::int64_t roundedX = aDirection.y > 0 ? javaIntegerCast( std::floor( x ) )
                                : aDirection.y < 0 ? javaIntegerCast( std::ceil( x ) )
                                                   : javaRound( x );
    const std::int64_t roundedY = aDirection.x > 0 ? javaIntegerCast( std::ceil( y ) )
                                : aDirection.x < 0 ? javaIntegerCast( std::floor( y ) )
                                                   : javaRound( y );
    return { roundedX, roundedY };
}


FLOAT_POINT FLOAT_POINT::Add( FLOAT_POINT aOther ) const
{
    return { x + aOther.x, y + aOther.y };
}


FLOAT_POINT FLOAT_POINT::Subtract( FLOAT_POINT aOther ) const
{
    return { x - aOther.x, y - aOther.y };
}


double FLOAT_POINT::ScalarProduct(
        FLOAT_POINT aFirst, FLOAT_POINT aSecond ) const
{
    return ( aFirst.x - x ) * ( aSecond.x - x )
           + ( aFirst.y - y ) * ( aSecond.y - y );
}


FLOAT_POINT FLOAT_POINT::ChangeSize( double aNewSize ) const
{
    if( x == 0 && y == 0 )
        return *this;
    const double length = std::sqrt( x * x + y * y );
    return { x * aNewSize / length, y * aNewSize / length };
}


FLOAT_POINT FLOAT_POINT::ChangeLength(
        FLOAT_POINT aToPoint, double aNewLength ) const
{
    const double dx = aToPoint.x - x;
    const double dy = aToPoint.y - y;
    if( dx == 0 && dy == 0 )
        return aToPoint;
    const double length = std::sqrt( dx * dx + dy * dy );
    return { x + dx * aNewLength / length,
             y + dy * aNewLength / length };
}


FLOAT_POINT FLOAT_POINT::MiddlePoint( FLOAT_POINT aToPoint ) const
{
    return { 0.5 * ( x + aToPoint.x ), 0.5 * ( y + aToPoint.y ) };
}


int FLOAT_POINT::SideOf( FLOAT_POINT aFirst, FLOAT_POINT aSecond ) const
{
    const double dx = aSecond.x - aFirst.x;
    const double dy = aSecond.y - aFirst.y;
    return sideOf( dx * ( y - aFirst.y ) - dy * ( x - aFirst.x ) );
}


FLOAT_POINT FLOAT_POINT::Rotate( double aAngle, FLOAT_POINT aPole ) const
{
    if( aAngle == 0 )
        return *this;
    const double dx = x - aPole.x;
    const double dy = y - aPole.y;
    const double sine = std::sin( aAngle );
    const double cosine = std::cos( aAngle );
    return { aPole.x + dx * cosine - dy * sine,
             aPole.y + dx * sine + dy * cosine };
}


FLOAT_POINT FLOAT_POINT::Turn90Degree( int aFactor ) const
{
    int factor = aFactor % 4;
    if( factor < 0 )
        factor += 4;
    switch( factor )
    {
    case 0: return { x, y };
    case 1: return { -y, x };
    case 2: return { -x, -y };
    default: return { y, -x };
    }
}


FLOAT_POINT FLOAT_POINT::Turn90Degree(
        int aFactor, FLOAT_POINT aPole ) const
{
    return aPole.Add( Subtract( aPole ).Turn90Degree( aFactor ) );
}


bool FLOAT_POINT::IsContainedInBox(
        FLOAT_POINT aFirst, FLOAT_POINT aSecond, double aTolerance ) const
{
    const double minimumX = std::min( aFirst.x, aSecond.x );
    const double maximumX = std::max( aFirst.x, aSecond.x );
    if( x < minimumX - aTolerance || x > maximumX + aTolerance )
        return false;
    const double minimumY = std::min( aFirst.y, aSecond.y );
    const double maximumY = std::max( aFirst.y, aSecond.y );
    return y >= minimumY - aTolerance && y <= maximumY + aTolerance;
}


ROUTER_BOX FLOAT_POINT::BoundingBox() const
{
    return { javaIntegerCast( std::floor( x ) ), javaIntegerCast( std::floor( y ) ),
             javaIntegerCast( std::ceil( x ) ), javaIntegerCast( std::ceil( y ) ) };
}


std::vector<FLOAT_POINT> FLOAT_POINT::TangentialPoints(
        FLOAT_POINT aToPoint, double aDistance ) const
{
    double dx = std::abs( x - aToPoint.x );
    double dy = std::abs( y - aToPoint.y );
    const bool situationTurned = dy > dx;
    const FLOAT_POINT pole = situationTurned ? FLOAT_POINT{ -y, x } : *this;
    const FLOAT_POINT centre = situationTurned
                               ? FLOAT_POINT{ -aToPoint.y, aToPoint.x } : aToPoint;
    dx = pole.x - centre.x;
    dy = pole.y - centre.y;
    const double dxSquared = dx * dx;
    const double dySquared = dy * dy;
    const double distanceSquared = dxSquared + dySquared;
    const double radiusSquared = aDistance * aDistance;
    const double discriminant = radiusSquared * dySquared
                                - ( radiusSquared - dxSquared ) * distanceSquared;
    if( discriminant <= 0 )
        return {};
    const double squareRoot = std::sqrt( discriminant );
    const double first = radiusSquared * dy;
    const double firstDy = ( first + aDistance * squareRoot ) / distanceSquared;
    const double secondDy = ( first - aDistance * squareRoot ) / distanceSquared;
    const double firstY = firstDy + centre.y;
    const double firstX = ( radiusSquared - dy * firstDy ) / dx + centre.x;
    const double secondY = secondDy + centre.y;
    const double secondX = ( radiusSquared - dy * secondDy ) / dx + centre.x;
    if( situationTurned )
        return { { firstY, -firstX }, { secondY, -secondX } };
    return { { firstX, firstY }, { secondX, secondY } };
}


std::optional<FLOAT_POINT> FLOAT_POINT::LeftTangentialPoint(
        FLOAT_POINT aToPoint, double aDistance ) const
{
    const auto points = TangentialPoints( aToPoint, aDistance );
    if( points.size() < 2 )
        return {};
    return aToPoint.SideOf( *this, points[0] ) < 0 ? points[0] : points[1];
}


std::optional<FLOAT_POINT> FLOAT_POINT::RightTangentialPoint(
        FLOAT_POINT aToPoint, double aDistance ) const
{
    const auto points = TangentialPoints( aToPoint, aDistance );
    if( points.size() < 2 )
        return {};
    return aToPoint.SideOf( *this, points[0] ) > 0 ? points[0] : points[1];
}


FLOAT_POINT FLOAT_POINT::CircleCenter(
        FLOAT_POINT aFirst, FLOAT_POINT aSecond ) const
{
    const double firstSlope = ( aFirst.y - y ) / ( aFirst.x - x );
    const double secondSlope = ( aSecond.y - aFirst.y )
                               / ( aSecond.x - aFirst.x );
    const double centreX = ( firstSlope * secondSlope * ( y - aSecond.y )
                             + secondSlope * ( x + aFirst.x )
                             - firstSlope * ( aFirst.x + aSecond.x ) )
                           / ( 2 * ( secondSlope - firstSlope ) );
    const double centreY = ( 0.5 * ( x + aFirst.x ) - centreX ) / firstSlope
                           + 0.5 * ( y + aFirst.y );
    return { centreX, centreY };
}


bool FLOAT_POINT::InsideCircle(
        FLOAT_POINT aFirst, FLOAT_POINT aSecond, FLOAT_POINT aThird ) const
{
    const FLOAT_POINT centre = aFirst.CircleCenter( aSecond, aThird );
    const double radiusSquared = centre.DistanceSquared( aFirst );
    return DistanceSquared( centre ) < radiusSquared - 1;
}


FLOAT_POINT FLOAT_LINE::Middle() const
{
    return { ( a.x + b.x ) / 2, ( a.y + b.y ) / 2 };
}


FLOAT_LINE FLOAT_LINE::Opposite() const
{
    return { b, a };
}


FLOAT_LINE FLOAT_LINE::AdjustDirection( const FLOAT_LINE& aOther ) const
{
    if( b.SideOf( a, aOther.a ) == aOther.b.SideOf( a, aOther.a ) )
        return *this;
    return Opposite();
}


std::optional<FLOAT_POINT> FLOAT_LINE::Intersection(
        const FLOAT_LINE& aOther ) const
{
    const double firstDx = b.x - a.x;
    const double firstDy = b.y - a.y;
    const double secondDx = aOther.b.x - aOther.a.x;
    const double secondDy = aOther.b.y - aOther.a.y;
    const double firstDeterminant = a.x * b.y - a.y * b.x;
    const double secondDeterminant = aOther.a.x * aOther.b.y
                                     - aOther.a.y * aOther.b.x;
    const double determinant = secondDx * firstDy - secondDy * firstDx;
    if( determinant == 0 )
        return {};
    return FLOAT_POINT{ ( secondDx * firstDeterminant
                          - firstDx * secondDeterminant ) / determinant,
                        ( secondDy * firstDeterminant
                          - firstDy * secondDeterminant ) / determinant };
}


FLOAT_LINE FLOAT_LINE::Translate( double aDistance ) const
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dxSquared = dx * dx;
    const double dySquared = dy * dy;
    const double length = std::sqrt( dxSquared + dySquared );
    FLOAT_POINT newA;
    if( dxSquared <= dySquared )
        newA = { a.x - aDistance * length / dy, a.y };
    else
        newA = { a.x, a.y + aDistance * length / dx };
    return { newA, { newA.x + dx, newA.y + dy } };
}


double FLOAT_LINE::SignedDistance( FLOAT_POINT aPoint ) const
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return ( dy * ( aPoint.x - a.x ) - dx * ( aPoint.y - a.y ) )
           / std::sqrt( dx * dx + dy * dy );
}


FLOAT_POINT FLOAT_LINE::PerpendicularProjection( FLOAT_POINT aPoint ) const
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if( dx == 0 && dy == 0 )
        return a;
    const double dxSquared = dx * dx;
    const double dySquared = dy * dy;
    const double dxDy = dx * dy;
    const double denominator = dxSquared + dySquared;
    const double determinant = a.x * b.y - b.x * a.y;
    return { ( aPoint.x * dxSquared + aPoint.y * dxDy
               + determinant * dy ) / denominator,
             ( aPoint.x * dxDy + aPoint.y * dySquared
               - determinant * dx ) / denominator };
}


double FLOAT_LINE::SegmentDistance( FLOAT_POINT aPoint ) const
{
    const FLOAT_POINT projection = PerpendicularProjection( aPoint );
    if( projection.IsContainedInBox( a, b, 0.01 ) )
        return aPoint.Distance( projection );
    return std::min( aPoint.Distance( a ), aPoint.Distance( b ) );
}


std::optional<FLOAT_LINE> FLOAT_LINE::SegmentProjection(
        const FLOAT_LINE& aLineSegment ) const
{
    if( b.ScalarProduct( a, aLineSegment.a ) < 0
        || a.ScalarProduct( b, aLineSegment.b ) < 0 )
    {
        return {};
    }
    FLOAT_POINT projectedA;
    if( a.ScalarProduct( b, aLineSegment.a ) < 0 )
        projectedA = a;
    else
    {
        projectedA = PerpendicularProjection( aLineSegment.a );
        if( std::abs( projectedA.x ) >= CRITICAL_INTEGER
            || std::abs( projectedA.y ) >= CRITICAL_INTEGER )
        {
            return {};
        }
    }
    FLOAT_POINT projectedB;
    if( b.ScalarProduct( a, aLineSegment.b ) < 0 )
        projectedB = b;
    else
        projectedB = PerpendicularProjection( aLineSegment.b );
    if( std::abs( projectedB.x ) >= CRITICAL_INTEGER
        || std::abs( projectedB.y ) >= CRITICAL_INTEGER )
    {
        return {};
    }
    return FLOAT_LINE{ projectedA, projectedB };
}


std::optional<FLOAT_LINE> FLOAT_LINE::SegmentProjection2(
        const FLOAT_LINE& aLineSegment ) const
{
    if( aLineSegment.a.ScalarProduct( aLineSegment.b, b ) <= 0
        || aLineSegment.b.ScalarProduct( aLineSegment.a, a ) <= 0 )
    {
        return {};
    }
    FLOAT_POINT projectedA;
    if( aLineSegment.a.ScalarProduct( aLineSegment.b, a ) < 0 )
    {
        const FLOAT_LINE perpendicular{ aLineSegment.a,
                aLineSegment.b.Turn90Degree( 1, aLineSegment.a ) };
        const auto intersection = perpendicular.Intersection( *this );
        if( !intersection || std::abs( intersection->x ) >= CRITICAL_INTEGER
            || std::abs( intersection->y ) >= CRITICAL_INTEGER )
        {
            return {};
        }
        projectedA = *intersection;
    }
    else
        projectedA = a;

    FLOAT_POINT projectedB;
    if( aLineSegment.b.ScalarProduct( aLineSegment.a, b ) < 0 )
    {
        const FLOAT_LINE perpendicular{ aLineSegment.b,
                aLineSegment.a.Turn90Degree( 1, aLineSegment.b ) };
        const auto intersection = perpendicular.Intersection( *this );
        if( !intersection || std::abs( intersection->x ) >= CRITICAL_INTEGER
            || std::abs( intersection->y ) >= CRITICAL_INTEGER )
        {
            return {};
        }
        projectedB = *intersection;
    }
    else
        projectedB = b;
    return FLOAT_LINE{ projectedA, projectedB };
}


FLOAT_LINE FLOAT_LINE::ShrinkSegment( double aOffset ) const
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if( dx == 0 && dy == 0 )
        return *this;
    const double length = std::sqrt( dx * dx + dy * dy );
    const double effectiveOffset = std::min( aOffset, length / 2 );
    const FLOAT_POINT newA{ a.x + dx * effectiveOffset / length,
                            a.y + dy * effectiveOffset / length };
    const double newLength = length - effectiveOffset;
    const FLOAT_POINT newB{ a.x + dx * newLength / length,
                            a.y + dy * newLength / length };
    return { newA, newB };
}


FLOAT_POINT FLOAT_LINE::NearestSegmentPoint( FLOAT_POINT aFromPoint ) const
{
    const FLOAT_POINT projection = PerpendicularProjection( aFromPoint );
    if( projection.IsContainedInBox( a, b, 0.01 ) )
        return projection;
    return aFromPoint.DistanceSquared( a ) <= aFromPoint.DistanceSquared( b ) ? a : b;
}


std::vector<FLOAT_LINE> FLOAT_LINE::DivideSegmentIntoSections( int aCount ) const
{
    if( aCount <= 0 )
        return {};
    if( aCount == 1 )
        return { *this };
    std::vector<FLOAT_LINE> result;
    result.reserve( aCount );
    const double lineLength = b.Distance( a );
    const double sectionLength = lineLength / aCount;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    FLOAT_POINT currentA = a;
    for( int index = 0; index < aCount; ++index )
    {
        FLOAT_POINT currentB;
        if( index == aCount - 1 )
            currentB = b;
        else
        {
            const double currentDistance = ( index + 1 ) * sectionLength;
            currentB = { a.x + dx * currentDistance / lineLength,
                         a.y + dy * currentDistance / lineLength };
        }
        result.push_back( { currentA, currentB } );
        currentA = currentB;
    }
    return result;
}

} // namespace KICAD_AUTOROUTER
