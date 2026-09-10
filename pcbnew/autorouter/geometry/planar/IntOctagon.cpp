/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct C++ translation of Freerouting geometry/planar/IntOctagon.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#include "IntOctagon.h"
#include "IntBox.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace KICAD_AUTOROUTER::PLANAR
{
namespace
{

std::int64_t narrow( const INTEGER& aValue )
{
    if( aValue < std::numeric_limits<std::int64_t>::min()
        || aValue > std::numeric_limits<std::int64_t>::max() )
    {
        throw std::overflow_error( "IntOctagon coordinate overflow" );
    }

    return aValue.convert_to<std::int64_t>();
}


std::int64_t add( std::int64_t aLeft, std::int64_t aRight )
{
    return narrow( INTEGER( aLeft ) + aRight );
}


std::int64_t subtract( std::int64_t aLeft, std::int64_t aRight )
{
    return narrow( INTEGER( aLeft ) - aRight );
}


std::int64_t floorHalf( const INTEGER& aValue )
{
    INTEGER quotient = aValue / 2;
    if( aValue < 0 && aValue % 2 != 0 )
        --quotient;
    return narrow( quotient );
}


std::int64_t ceilHalf( const INTEGER& aValue )
{
    INTEGER quotient = aValue / 2;
    if( aValue > 0 && aValue % 2 != 0 )
        ++quotient;
    return narrow( quotient );
}


std::int64_t javaRound( double aValue )
{
    if( !std::isfinite( aValue ) )
        throw std::invalid_argument( "non-finite IntOctagon offset" );

    const double rounded = std::floor( aValue + 0.5 );
    if( rounded < static_cast<double>( std::numeric_limits<std::int64_t>::min() )
        || rounded > static_cast<double>( std::numeric_limits<std::int64_t>::max() ) )
    {
        throw std::overflow_error( "IntOctagon offset overflow" );
    }

    return static_cast<std::int64_t>( rounded );
}

} // namespace


INT_OCTAGON INT_OCTAGON::Empty()
{
    constexpr std::int64_t critical = CRITICAL_COORDINATE;
    return { critical, critical, -critical, -critical,
             critical, -critical, critical, -critical };
}


INT_OCTAGON INT_OCTAGON::FromBox( const ROUTER_BOX& aBox )
{
    if( INT_BOX::Dimension( aBox ) < 0 )
        return Empty();

    return { aBox.minX, aBox.minY, aBox.maxX, aBox.maxY,
             subtract( aBox.minX, aBox.maxY ),
             subtract( aBox.maxX, aBox.minY ),
             add( aBox.minX, aBox.minY ),
             add( aBox.maxX, aBox.maxY ) };
}


bool INT_OCTAGON::IsEmpty() const
{
    return *this == Empty();
}


ROUTER_BOX INT_OCTAGON::BoundingBox() const
{
    return { leftX, bottomY, rightX, topY };
}


int INT_OCTAGON::Dimension() const
{
    if( IsEmpty() )
        return -1;

    if( rightX > leftX && topY > bottomY
        && lowerRightDiagonalX > upperLeftDiagonalX
        && upperRightDiagonalX > lowerLeftDiagonalX )
    {
        return 2;
    }

    if( rightX == leftX && topY == bottomY )
        return 0;

    return 1;
}


std::int64_t INT_OCTAGON::CornerX( int aIndex ) const
{
    switch( aIndex )
    {
    case 0: return subtract( lowerLeftDiagonalX, bottomY );
    case 1: return add( lowerRightDiagonalX, bottomY );
    case 2:
    case 3: return rightX;
    case 4: return subtract( upperRightDiagonalX, topY );
    case 5: return add( upperLeftDiagonalX, topY );
    case 6:
    case 7: return leftX;
    default: throw std::out_of_range( "IntOctagon corner index" );
    }
}


std::int64_t INT_OCTAGON::CornerY( int aIndex ) const
{
    switch( aIndex )
    {
    case 0:
    case 1: return bottomY;
    case 2: return subtract( rightX, lowerRightDiagonalX );
    case 3: return subtract( upperRightDiagonalX, rightX );
    case 4:
    case 5: return topY;
    case 6: return subtract( leftX, upperLeftDiagonalX );
    case 7: return subtract( lowerLeftDiagonalX, leftX );
    default: throw std::out_of_range( "IntOctagon corner index" );
    }
}


ROUTER_POINT INT_OCTAGON::Corner( int aIndex ) const
{
    return { CornerX( aIndex ), CornerY( aIndex ) };
}


double INT_OCTAGON::Area() const
{
    double result = static_cast<double>( lowerLeftDiagonalX - bottomY )
                    * static_cast<double>( bottomY - lowerLeftDiagonalX + leftX );
    result += static_cast<double>( lowerRightDiagonalX + bottomY )
              * static_cast<double>( rightX - lowerRightDiagonalX - bottomY );
    result += static_cast<double>( rightX )
              * static_cast<double>( upperRightDiagonalX - 2 * rightX - bottomY
                                     + topY + lowerRightDiagonalX );
    result += static_cast<double>( upperRightDiagonalX - topY )
              * static_cast<double>( topY - upperRightDiagonalX + rightX );
    result += static_cast<double>( upperLeftDiagonalX + topY )
              * static_cast<double>( leftX - upperLeftDiagonalX - topY );
    result += static_cast<double>( leftX )
              * static_cast<double>( lowerLeftDiagonalX - 2 * leftX - topY
                                     + bottomY + upperLeftDiagonalX );
    return 0.5 * std::abs( result );
}


LINE INT_OCTAGON::BorderLine( int aIndex ) const
{
    switch( aIndex )
    {
    case 0: return { { 0, bottomY }, { 1, bottomY } };
    case 1: return { { lowerRightDiagonalX, 0 }, { add( lowerRightDiagonalX, 1 ), 1 } };
    case 2: return { { rightX, 0 }, { rightX, 1 } };
    case 3: return { { upperRightDiagonalX, 0 }, { subtract( upperRightDiagonalX, 1 ), 1 } };
    case 4: return { { 0, topY }, { -1, topY } };
    case 5: return { { upperLeftDiagonalX, 0 }, { subtract( upperLeftDiagonalX, 1 ), -1 } };
    case 6: return { { leftX, 0 }, { leftX, -1 } };
    case 7: return { { lowerLeftDiagonalX, 0 }, { add( lowerLeftDiagonalX, 1 ), -1 } };
    default: throw std::out_of_range( "IntOctagon border index" );
    }
}


INT_OCTAGON INT_OCTAGON::TranslateBy( ROUTER_POINT aVector ) const
{
    if( aVector == ROUTER_POINT{} )
        return *this;

    return { add( leftX, aVector.x ), add( bottomY, aVector.y ),
             add( rightX, aVector.x ), add( topY, aVector.y ),
             add( upperLeftDiagonalX, subtract( aVector.x, aVector.y ) ),
             add( lowerRightDiagonalX, subtract( aVector.x, aVector.y ) ),
             add( lowerLeftDiagonalX, add( aVector.x, aVector.y ) ),
             add( upperRightDiagonalX, add( aVector.x, aVector.y ) ) };
}


double INT_OCTAGON::MaxWidth() const
{
    const double width1 = std::max( static_cast<double>( rightX - leftX ),
                                    static_cast<double>( topY - bottomY ) );
    const double width2 = std::max( static_cast<double>( upperRightDiagonalX
                                                         - lowerLeftDiagonalX ),
                                    static_cast<double>( lowerRightDiagonalX
                                                         - upperLeftDiagonalX ) );
    return std::max( width1, width2 / std::sqrt( 2.0 ) );
}


double INT_OCTAGON::MinWidth() const
{
    const double width1 = std::min( static_cast<double>( rightX - leftX ),
                                    static_cast<double>( topY - bottomY ) );
    const double width2 = std::min( static_cast<double>( upperRightDiagonalX
                                                         - lowerLeftDiagonalX ),
                                    static_cast<double>( lowerRightDiagonalX
                                                         - upperLeftDiagonalX ) );
    return std::min( width1, width2 / std::sqrt( 2.0 ) );
}


INT_OCTAGON INT_OCTAGON::Offset( double aDistance ) const
{
    const std::int64_t width = javaRound( aDistance );
    if( width == 0 )
        return *this;

    const std::int64_t diagonalWidth = javaRound( std::sqrt( 2.0 ) * aDistance );
    return INT_OCTAGON( subtract( leftX, width ), subtract( bottomY, width ),
                        add( rightX, width ), add( topY, width ),
                        subtract( upperLeftDiagonalX, diagonalWidth ),
                        add( lowerRightDiagonalX, diagonalWidth ),
                        subtract( lowerLeftDiagonalX, diagonalWidth ),
                        add( upperRightDiagonalX, diagonalWidth ) ).Normalize();
}


bool INT_OCTAGON::Contains( ROUTER_POINT aPoint ) const
{
    if( leftX > aPoint.x || bottomY > aPoint.y || rightX < aPoint.x || topY < aPoint.y )
        return false;

    const INTEGER difference = INTEGER( aPoint.x ) - aPoint.y;
    const INTEGER sum = INTEGER( aPoint.x ) + aPoint.y;
    return INTEGER( upperLeftDiagonalX ) <= difference
           && INTEGER( lowerRightDiagonalX ) >= difference
           && INTEGER( lowerLeftDiagonalX ) <= sum
           && INTEGER( upperRightDiagonalX ) >= sum;
}


INT_OCTAGON INT_OCTAGON::Union( const INT_OCTAGON& aOther ) const
{
    return { std::min( leftX, aOther.leftX ), std::min( bottomY, aOther.bottomY ),
             std::max( rightX, aOther.rightX ), std::max( topY, aOther.topY ),
             std::min( upperLeftDiagonalX, aOther.upperLeftDiagonalX ),
             std::max( lowerRightDiagonalX, aOther.lowerRightDiagonalX ),
             std::min( lowerLeftDiagonalX, aOther.lowerLeftDiagonalX ),
             std::max( upperRightDiagonalX, aOther.upperRightDiagonalX ) };
}


INT_OCTAGON INT_OCTAGON::Intersection( const INT_OCTAGON& aOther ) const
{
    return INT_OCTAGON( std::max( leftX, aOther.leftX ),
                        std::max( bottomY, aOther.bottomY ),
                        std::min( rightX, aOther.rightX ),
                        std::min( topY, aOther.topY ),
                        std::max( upperLeftDiagonalX, aOther.upperLeftDiagonalX ),
                        std::min( lowerRightDiagonalX, aOther.lowerRightDiagonalX ),
                        std::max( lowerLeftDiagonalX, aOther.lowerLeftDiagonalX ),
                        std::min( upperRightDiagonalX, aOther.upperRightDiagonalX ) ).Normalize();
}


INT_OCTAGON INT_OCTAGON::Normalize() const
{
    if( leftX > rightX || bottomY > topY
        || lowerLeftDiagonalX > upperRightDiagonalX
        || upperLeftDiagonalX > lowerRightDiagonalX )
    {
        return Empty();
    }

    std::int64_t newLx = leftX;
    std::int64_t newRx = rightX;
    std::int64_t newLy = bottomY;
    std::int64_t newUy = topY;
    std::int64_t newLlx = lowerLeftDiagonalX;
    std::int64_t newUlx = upperLeftDiagonalX;
    std::int64_t newLrx = lowerRightDiagonalX;
    std::int64_t newUrx = upperRightDiagonalX;

    if( newLx < subtract( newLlx, newUy ) ) newLx = subtract( newLlx, newUy );
    if( newLx < add( newUlx, newLy ) ) newLx = add( newUlx, newLy );
    if( newRx > subtract( newUrx, newLy ) ) newRx = subtract( newUrx, newLy );
    if( newRx > add( newLrx, newUy ) ) newRx = add( newLrx, newUy );
    if( newLy < subtract( newLx, newLrx ) ) newLy = subtract( newLx, newLrx );
    if( newLy < subtract( newLlx, newRx ) ) newLy = subtract( newLlx, newRx );
    if( newUy > subtract( newUrx, newLx ) ) newUy = subtract( newUrx, newLx );
    if( newUy > subtract( newRx, newUlx ) ) newUy = subtract( newRx, newUlx );
    if( subtract( newLlx, newLx ) < newLy ) newLlx = add( newLx, newLy );
    if( subtract( newRx, newLrx ) < newLy ) newLrx = subtract( newRx, newLy );
    if( subtract( newUrx, newRx ) > newUy ) newUrx = add( newUy, newRx );
    if( subtract( newLx, newUlx ) > newUy ) newUlx = subtract( newLx, newUy );

    const std::int64_t diagonalUpperY = ceilHalf( INTEGER( newUrx ) - newUlx );
    if( newUy > diagonalUpperY ) newUy = diagonalUpperY;
    const std::int64_t diagonalLowerY = floorHalf( INTEGER( newLlx ) - newLrx );
    if( newLy < diagonalLowerY ) newLy = diagonalLowerY;
    const std::int64_t diagonalRightX = ceilHalf( INTEGER( newUrx ) + newLrx );
    if( newRx > diagonalRightX ) newRx = diagonalRightX;
    const std::int64_t diagonalLeftX = floorHalf( INTEGER( newLlx ) + newUlx );
    if( newLx < diagonalLeftX ) newLx = diagonalLeftX;

    if( newLx > newRx || newLy > newUy || newLlx > newUrx || newUlx > newLrx )
        return Empty();

    return { newLx, newLy, newRx, newUy, newUlx, newLrx, newLlx, newUrx };
}


bool INT_OCTAGON::IsNormalized() const
{
    return *this == Normalize();
}


bool INT_OCTAGON::IsContainedIn( const ROUTER_BOX& aBox ) const
{
    return leftX >= aBox.minX && bottomY >= aBox.minY
           && rightX <= aBox.maxX && topY <= aBox.maxY;
}


bool INT_OCTAGON::IsContainedIn( const INT_OCTAGON& aOther ) const
{
    return leftX >= aOther.leftX && bottomY >= aOther.bottomY
           && rightX <= aOther.rightX && topY <= aOther.topY
           && lowerLeftDiagonalX >= aOther.lowerLeftDiagonalX
           && upperLeftDiagonalX >= aOther.upperLeftDiagonalX
           && lowerRightDiagonalX <= aOther.lowerRightDiagonalX
           && upperRightDiagonalX <= aOther.upperRightDiagonalX;
}


bool INT_OCTAGON::Intersects( const INT_OCTAGON& aOther ) const
{
    if( std::max( leftX, aOther.leftX ) > std::min( rightX, aOther.rightX ) ) return false;
    if( std::max( bottomY, aOther.bottomY ) > std::min( topY, aOther.topY ) ) return false;
    if( std::max( lowerLeftDiagonalX, aOther.lowerLeftDiagonalX )
        > std::min( upperRightDiagonalX, aOther.upperRightDiagonalX ) ) return false;
    return std::max( upperLeftDiagonalX, aOther.upperLeftDiagonalX )
           <= std::min( lowerRightDiagonalX, aOther.lowerRightDiagonalX );
}


bool INT_OCTAGON::Overlaps( const INT_OCTAGON& aOther ) const
{
    if( std::max( leftX, aOther.leftX ) >= std::min( rightX, aOther.rightX ) ) return false;
    if( std::max( bottomY, aOther.bottomY ) >= std::min( topY, aOther.topY ) ) return false;
    if( std::max( lowerLeftDiagonalX, aOther.lowerLeftDiagonalX )
        >= std::min( upperRightDiagonalX, aOther.upperRightDiagonalX ) ) return false;
    return std::max( upperLeftDiagonalX, aOther.upperLeftDiagonalX )
           < std::min( lowerRightDiagonalX, aOther.lowerRightDiagonalX );
}


std::int64_t INT_OCTAGON::LeftXValue( std::int64_t aY ) const
{
    return std::max( leftX, std::max( add( upperLeftDiagonalX, aY ),
                                      subtract( lowerLeftDiagonalX, aY ) ) );
}


std::int64_t INT_OCTAGON::RightXValue( std::int64_t aY ) const
{
    return std::min( rightX, std::min( subtract( upperRightDiagonalX, aY ),
                                       add( lowerRightDiagonalX, aY ) ) );
}


std::int64_t INT_OCTAGON::LowerYValue( std::int64_t aX ) const
{
    return std::max( bottomY, std::max( subtract( lowerLeftDiagonalX, aX ),
                                        subtract( aX, lowerRightDiagonalX ) ) );
}


std::int64_t INT_OCTAGON::UpperYValue( std::int64_t aX ) const
{
    return std::min( topY, std::min( subtract( aX, upperLeftDiagonalX ),
                                     subtract( upperRightDiagonalX, aX ) ) );
}


int INT_OCTAGON::SideOfBorderLine( std::int64_t aX, std::int64_t aY,
                                   int aBorderIndex ) const
{
    INTEGER value;
    switch( aBorderIndex )
    {
    case 0: value = INTEGER( bottomY ) - aY; break;
    case 1: value = INTEGER( aX ) - aY - lowerRightDiagonalX; break;
    case 2: value = INTEGER( aX ) - rightX; break;
    case 3: value = INTEGER( aX ) + aY - upperRightDiagonalX; break;
    case 4: value = INTEGER( aY ) - topY; break;
    case 5: value = INTEGER( upperLeftDiagonalX ) + aY - aX; break;
    case 6: value = INTEGER( leftX ) - aX; break;
    case 7: value = INTEGER( lowerLeftDiagonalX ) - aX - aY; break;
    default: throw std::out_of_range( "IntOctagon border index" );
    }

    // Freerouting Side: negative tmp is ON_THE_LEFT, positive is ON_THE_RIGHT.
    return Sign( value );
}


int INT_OCTAGON::Compare( const INT_OCTAGON& aOther, int aEdgeIndex ) const
{
    switch( aEdgeIndex )
    {
    case 0: return bottomY == aOther.bottomY ? 0 : bottomY > aOther.bottomY ? -1 : 1;
    case 1: return lowerRightDiagonalX == aOther.lowerRightDiagonalX ? 0
                    : lowerRightDiagonalX < aOther.lowerRightDiagonalX ? -1 : 1;
    case 2: return rightX == aOther.rightX ? 0 : rightX < aOther.rightX ? -1 : 1;
    case 3: return upperRightDiagonalX == aOther.upperRightDiagonalX ? 0
                    : upperRightDiagonalX < aOther.upperRightDiagonalX ? -1 : 1;
    case 4: return topY == aOther.topY ? 0 : topY < aOther.topY ? -1 : 1;
    case 5: return upperLeftDiagonalX == aOther.upperLeftDiagonalX ? 0
                    : upperLeftDiagonalX > aOther.upperLeftDiagonalX ? -1 : 1;
    case 6: return leftX == aOther.leftX ? 0 : leftX > aOther.leftX ? -1 : 1;
    case 7: return lowerLeftDiagonalX == aOther.lowerLeftDiagonalX ? 0
                    : lowerLeftDiagonalX > aOther.lowerLeftDiagonalX ? -1 : 1;
    default: throw std::out_of_range( "IntOctagon compare edge index" );
    }
}


bool INT_OCTAGON::IsIntBox() const
{
    return lowerLeftDiagonalX == add( leftX, bottomY )
           && lowerRightDiagonalX == subtract( rightX, bottomY )
           && upperRightDiagonalX == add( rightX, topY )
           && upperLeftDiagonalX == subtract( leftX, topY );
}


std::optional<SIMPLEX> INT_OCTAGON::ToSimplex() const
{
    if( Dimension() != 2 )
        return {};

    std::vector<ROUTER_POINT> corners;
    corners.reserve( 8 );
    for( int i = 0; i < 8; ++i )
    {
        const ROUTER_POINT corner = Corner( i );
        if( corners.empty() || corners.back() != corner )
            corners.push_back( corner );
    }
    if( corners.size() > 1 && corners.front() == corners.back() )
        corners.pop_back();
    return SIMPLEX::FromConvexPolygon( corners );
}


bool INT_OCTAGON::operator==( const INT_OCTAGON& aOther ) const
{
    return leftX == aOther.leftX && bottomY == aOther.bottomY
           && rightX == aOther.rightX && topY == aOther.topY
           && upperLeftDiagonalX == aOther.upperLeftDiagonalX
           && lowerRightDiagonalX == aOther.lowerRightDiagonalX
           && lowerLeftDiagonalX == aOther.lowerLeftDiagonalX
           && upperRightDiagonalX == aOther.upperRightDiagonalX;
}

} // namespace KICAD_AUTOROUTER::PLANAR
