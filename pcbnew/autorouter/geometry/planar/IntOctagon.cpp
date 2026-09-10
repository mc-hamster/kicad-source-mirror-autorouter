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


bool INT_OCTAGON::ContainsInside( ROUTER_POINT aPoint ) const
{
    if( leftX >= aPoint.x || bottomY >= aPoint.y
        || rightX <= aPoint.x || topY <= aPoint.y )
    {
        return false;
    }

    const INTEGER difference = INTEGER( aPoint.x ) - aPoint.y;
    const INTEGER sum = INTEGER( aPoint.x ) + aPoint.y;
    return INTEGER( upperLeftDiagonalX ) < difference
           && INTEGER( lowerRightDiagonalX ) > difference
           && INTEGER( lowerLeftDiagonalX ) < sum
           && INTEGER( upperRightDiagonalX ) > sum;
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


std::vector<INT_OCTAGON> INT_OCTAGON::Cutout( const INT_OCTAGON& aCutout ) const
{
    // Direct translation of IntOctagon.cutoutFrom(IntOctagon).  `d` is the
    // divide shape and `c` is the part of the requested cutout which actually
    // lies in it.  The order and the circumference-reducing divider switches
    // are routing decisions: DrillPage consumes the resulting shapes in this
    // sequence and therefore tests candidate vias in this sequence.
    const INT_OCTAGON& d = *this;
    const INT_OCTAGON c = aCutout.Intersection( d );
    if( aCutout.IsEmpty() || c.Dimension() < aCutout.Dimension() )
    {
        return { d };
    }

    std::int64_t temporary = subtract( c.lowerLeftDiagonalX, c.leftX );
    std::array<INT_OCTAGON, 8> result = {
        INT_OCTAGON( d.leftX, temporary, c.leftX,
                     subtract( c.leftX, c.upperLeftDiagonalX ),
                     d.upperLeftDiagonalX, d.lowerRightDiagonalX,
                     d.lowerLeftDiagonalX, d.upperRightDiagonalX ),
        Empty(), Empty(), Empty(), Empty(), Empty(), Empty(), Empty()
    };

    std::int64_t temporary2 = subtract( c.lowerLeftDiagonalX, c.bottomY );
    result[1] = INT_OCTAGON( d.leftX, d.bottomY, temporary2, temporary,
                             d.upperLeftDiagonalX, d.lowerRightDiagonalX,
                             d.lowerLeftDiagonalX, c.lowerLeftDiagonalX );

    temporary = add( c.lowerRightDiagonalX, c.bottomY );
    result[2] = INT_OCTAGON( temporary2, d.bottomY, temporary, c.bottomY,
                             d.upperLeftDiagonalX, d.lowerRightDiagonalX,
                             d.lowerLeftDiagonalX, d.upperRightDiagonalX );

    temporary2 = subtract( c.rightX, c.lowerRightDiagonalX );
    result[3] = INT_OCTAGON( temporary, d.bottomY, d.rightX, temporary2,
                             c.lowerRightDiagonalX, d.lowerRightDiagonalX,
                             d.lowerLeftDiagonalX, d.upperRightDiagonalX );

    temporary = subtract( c.upperRightDiagonalX, c.rightX );
    result[4] = INT_OCTAGON( c.rightX, temporary2, d.rightX, temporary,
                             d.upperLeftDiagonalX, d.lowerRightDiagonalX,
                             d.lowerLeftDiagonalX, d.upperRightDiagonalX );

    temporary2 = subtract( c.upperRightDiagonalX, c.topY );
    result[5] = INT_OCTAGON( temporary2, temporary, d.rightX, d.topY,
                             d.upperLeftDiagonalX, d.lowerRightDiagonalX,
                             c.upperRightDiagonalX, d.upperRightDiagonalX );

    temporary = add( c.upperLeftDiagonalX, c.topY );
    result[6] = INT_OCTAGON( temporary, c.topY, temporary2, d.topY,
                             d.upperLeftDiagonalX, d.lowerRightDiagonalX,
                             d.lowerLeftDiagonalX, d.upperRightDiagonalX );

    temporary2 = subtract( c.leftX, c.upperLeftDiagonalX );
    result[7] = INT_OCTAGON( d.leftX, temporary2, temporary, d.topY,
                             d.upperLeftDiagonalX, c.upperLeftDiagonalX,
                             d.lowerLeftDiagonalX, d.upperRightDiagonalX );

    for( INT_OCTAGON& piece : result )
        piece = piece.Normalize();

    INT_OCTAGON first = result[0];
    INT_OCTAGON second = result[7];
    if( !( first.IsEmpty() || second.IsEmpty() )
        && first.rightX - first.LeftXValue( first.topY )
                   > second.UpperYValue( first.rightX ) - second.bottomY )
    {
        first = INT_OCTAGON(
                std::min( first.leftX, second.leftX ), first.bottomY,
                first.rightX, second.topY, second.upperLeftDiagonalX,
                first.lowerRightDiagonalX, first.lowerLeftDiagonalX,
                second.upperRightDiagonalX );
        second = INT_OCTAGON(
                first.rightX, second.bottomY, second.rightX, second.topY,
                second.upperLeftDiagonalX, second.lowerRightDiagonalX,
                second.lowerLeftDiagonalX, second.upperRightDiagonalX );
        result[0] = first.Normalize();
        result[7] = second.Normalize();
    }

    first = result[7];
    second = result[6];
    if( !( first.IsEmpty() || second.IsEmpty() )
        && second.UpperYValue( first.rightX ) - second.bottomY
                   > first.rightX - first.LeftXValue( second.bottomY ) )
    {
        second = INT_OCTAGON(
                first.leftX, second.bottomY, second.rightX,
                std::max( second.topY, first.topY ), first.upperLeftDiagonalX,
                second.lowerRightDiagonalX, first.lowerLeftDiagonalX,
                second.upperRightDiagonalX );
        first = INT_OCTAGON(
                first.leftX, first.bottomY, first.rightX, second.bottomY,
                first.upperLeftDiagonalX, first.lowerRightDiagonalX,
                first.lowerLeftDiagonalX, first.upperRightDiagonalX );
        result[7] = first.Normalize();
        result[6] = second.Normalize();
    }

    first = result[6];
    second = result[5];
    if( !( first.IsEmpty() || second.IsEmpty() )
        && second.UpperYValue( first.rightX ) - first.bottomY
                   > second.RightXValue( first.bottomY ) - second.leftX )
    {
        first = INT_OCTAGON(
                first.leftX, first.bottomY, second.rightX,
                std::max( second.topY, first.topY ), first.upperLeftDiagonalX,
                second.lowerRightDiagonalX, first.lowerLeftDiagonalX,
                second.upperRightDiagonalX );
        second = INT_OCTAGON(
                second.leftX, second.bottomY, second.rightX, first.bottomY,
                second.upperLeftDiagonalX, second.lowerRightDiagonalX,
                second.lowerLeftDiagonalX, second.upperRightDiagonalX );
        result[6] = first.Normalize();
        result[5] = second.Normalize();
    }

    first = result[5];
    second = result[4];
    if( !( first.IsEmpty() || second.IsEmpty() )
        && second.RightXValue( second.topY ) - second.leftX
                   > first.UpperYValue( second.leftX ) - second.topY )
    {
        second = INT_OCTAGON(
                second.leftX, second.bottomY, std::max( second.rightX, first.rightX ),
                first.topY, first.upperLeftDiagonalX, second.lowerRightDiagonalX,
                second.lowerLeftDiagonalX, first.upperRightDiagonalX );
        first = INT_OCTAGON(
                first.leftX, first.bottomY, second.leftX, first.topY,
                first.upperLeftDiagonalX, first.lowerRightDiagonalX,
                first.lowerLeftDiagonalX, first.upperRightDiagonalX );
        result[5] = first.Normalize();
        result[4] = second.Normalize();
    }

    first = result[4];
    second = result[3];
    if( !( first.IsEmpty() || second.IsEmpty() )
        && first.RightXValue( first.bottomY ) - first.leftX
                   > first.bottomY - second.LowerYValue( first.leftX ) )
    {
        first = INT_OCTAGON(
                first.leftX, second.bottomY, std::max( second.rightX, first.rightX ),
                first.topY, first.upperLeftDiagonalX, second.lowerRightDiagonalX,
                second.lowerLeftDiagonalX, first.upperRightDiagonalX );
        second = INT_OCTAGON(
                second.leftX, second.bottomY, first.leftX, second.topY,
                second.upperLeftDiagonalX, second.lowerRightDiagonalX,
                second.lowerLeftDiagonalX, second.upperRightDiagonalX );
        result[4] = first.Normalize();
        result[3] = second.Normalize();
    }

    first = result[3];
    second = result[2];
    if( !( first.IsEmpty() || second.IsEmpty() )
        && second.topY - second.LowerYValue( second.rightX )
                   > first.RightXValue( second.topY ) - second.rightX )
    {
        second = INT_OCTAGON(
                second.leftX, std::min( first.bottomY, second.bottomY ), first.rightX,
                second.topY, second.upperLeftDiagonalX, first.lowerRightDiagonalX,
                second.lowerLeftDiagonalX, first.upperRightDiagonalX );
        first = INT_OCTAGON(
                first.leftX, second.topY, first.rightX, first.topY,
                first.upperLeftDiagonalX, first.lowerRightDiagonalX,
                first.lowerLeftDiagonalX, first.upperRightDiagonalX );
        result[3] = first.Normalize();
        result[2] = second.Normalize();
    }

    first = result[2];
    second = result[1];
    if( !( first.IsEmpty() || second.IsEmpty() )
        && first.topY - first.LowerYValue( first.leftX )
                   > first.leftX - second.LeftXValue( first.topY ) )
    {
        first = INT_OCTAGON(
                second.leftX, std::min( first.bottomY, second.bottomY ), first.rightX,
                first.topY, second.upperLeftDiagonalX, first.lowerRightDiagonalX,
                second.lowerLeftDiagonalX, first.upperRightDiagonalX );
        second = INT_OCTAGON(
                second.leftX, first.topY, second.rightX, second.topY,
                second.upperLeftDiagonalX, second.lowerRightDiagonalX,
                second.lowerLeftDiagonalX, second.upperRightDiagonalX );
        result[2] = first.Normalize();
        result[1] = second.Normalize();
    }

    first = result[1];
    second = result[0];
    if( !( first.IsEmpty() || second.IsEmpty() )
        && second.rightX - second.LeftXValue( second.bottomY )
                   > second.bottomY - first.LowerYValue( second.rightX ) )
    {
        second = INT_OCTAGON(
                std::min( second.leftX, first.leftX ), first.bottomY, second.rightX,
                second.topY, second.upperLeftDiagonalX, first.lowerRightDiagonalX,
                first.lowerLeftDiagonalX, second.upperRightDiagonalX );
        first = INT_OCTAGON(
                second.rightX, first.bottomY, first.rightX, first.topY,
                first.upperLeftDiagonalX, first.lowerRightDiagonalX,
                first.lowerLeftDiagonalX, first.upperRightDiagonalX );
        result[1] = first.Normalize();
        result[0] = second.Normalize();
    }

    return { result.begin(), result.end() };
}


std::vector<INT_OCTAGON> INT_OCTAGON::CutoutFromBox(
        const ROUTER_BOX& aOuter ) const
{
    const auto rawBoxOctagon = []( const ROUTER_BOX& aBox )
    {
        // IntBox.toIntOctagon() preserves a lower-dimensional/inverted
        // intermediate instead of replacing it with IntOctagon.EMPTY.  The
        // PolylineArea caller performs dimension filtering afterwards.
        return INT_OCTAGON(
                aBox.minX, aBox.minY, aBox.maxX, aBox.maxY,
                subtract( aBox.minX, aBox.maxY ),
                subtract( aBox.maxX, aBox.minY ),
                add( aBox.minX, aBox.minY ),
                add( aBox.maxX, aBox.maxY ) );
    };
    const INT_OCTAGON outer = FromBox( aOuter );
    const INT_OCTAGON c = Intersection( outer );
    if( IsEmpty() || c.Dimension() < Dimension() )
    {
        return { outer };
    }

    std::array<ROUTER_BOX, 4> boxes = {
        ROUTER_BOX{ aOuter.minX, subtract( c.lowerLeftDiagonalX, c.leftX ),
                    c.leftX, subtract( c.leftX, c.upperLeftDiagonalX ) },
        ROUTER_BOX{ c.rightX, subtract( c.rightX, c.lowerRightDiagonalX ),
                    aOuter.maxX, subtract( c.upperRightDiagonalX, c.rightX ) },
        ROUTER_BOX{ subtract( c.lowerLeftDiagonalX, c.bottomY ), aOuter.minY,
                    add( c.lowerRightDiagonalX, c.bottomY ), c.bottomY },
        ROUTER_BOX{ add( c.upperLeftDiagonalX, c.topY ), c.topY,
                    subtract( c.upperRightDiagonalX, c.topY ), aOuter.maxY }
    };

    // The source uses +/-Limits.CRIT_INT as unbounded support sentinels.
    // KiCad IU routinely exceed Java's 25-bit safe-coordinate range, so use
    // the same construction with a proportionally wide int64 sentinel.  It
    // must stay clear of the ends because Normalize adds/subtracts supports.
    constexpr std::int64_t critical =
            std::numeric_limits<std::int64_t>::max() / 16;
    std::array<INT_OCTAGON, 4> octagons = {
        INT_OCTAGON( aOuter.minX, boxes[0].maxY, boxes[3].minX, aOuter.maxY,
                     -critical, c.upperLeftDiagonalX,
                     -critical, critical ).Normalize(),
        INT_OCTAGON( aOuter.minX, aOuter.minY, boxes[2].minX, boxes[0].minY,
                     -critical, critical,
                     -critical, c.lowerLeftDiagonalX ).Normalize(),
        INT_OCTAGON( boxes[2].maxX, aOuter.minY, aOuter.maxX, boxes[1].minY,
                     c.lowerRightDiagonalX, critical,
                     -critical, critical ).Normalize(),
        INT_OCTAGON( boxes[3].maxX, boxes[1].maxY, aOuter.maxX, aOuter.maxY,
                     -critical, critical,
                     c.upperRightDiagonalX, critical ).Normalize()
    };

    ROUTER_BOX box = boxes[0];
    INT_OCTAGON octagon = octagons[0];
    if( box.maxX - box.minX > octagon.topY - octagon.bottomY )
    {
        boxes[0] = { box.minX, box.minY, box.maxX, octagon.topY };
        octagons[0] = INT_OCTAGON(
                box.maxX, octagon.bottomY, octagon.rightX, octagon.topY,
                octagon.upperLeftDiagonalX, octagon.lowerRightDiagonalX,
                octagon.lowerLeftDiagonalX, octagon.upperRightDiagonalX ).Normalize();
    }

    box = boxes[3];
    octagon = octagons[0];
    if( box.maxY - box.minY > octagon.rightX - octagon.leftX )
    {
        boxes[3] = { octagon.leftX, box.minY, box.maxX, box.maxY };
        octagons[0] = INT_OCTAGON(
                octagon.leftX, octagon.bottomY, octagon.rightX, box.minY,
                octagon.upperLeftDiagonalX, octagon.lowerRightDiagonalX,
                octagon.lowerLeftDiagonalX, octagon.upperRightDiagonalX ).Normalize();
    }

    box = boxes[3];
    octagon = octagons[3];
    if( box.maxY - box.minY > octagon.rightX - octagon.leftX )
    {
        boxes[3] = { box.minX, box.minY, octagon.rightX, box.maxY };
        octagons[3] = INT_OCTAGON(
                octagon.leftX, octagon.bottomY, octagon.rightX, octagon.topY,
                octagon.upperLeftDiagonalX, octagon.lowerRightDiagonalX,
                octagon.lowerLeftDiagonalX, octagon.upperRightDiagonalX ).Normalize();
    }

    box = boxes[1];
    octagon = octagons[3];
    if( box.maxX - box.minX > octagon.topY - octagon.bottomY )
    {
        boxes[1] = { box.minX, box.minY, box.maxX, octagon.topY };
        octagons[3] = INT_OCTAGON(
                octagon.leftX, octagon.bottomY, box.minX, octagon.topY,
                octagon.upperLeftDiagonalX, octagon.lowerRightDiagonalX,
                octagon.lowerLeftDiagonalX, octagon.upperRightDiagonalX ).Normalize();
    }

    box = boxes[1];
    octagon = octagons[2];
    if( box.maxX - box.minX > octagon.topY - octagon.bottomY )
    {
        boxes[1] = { box.minX, octagon.bottomY, box.maxX, box.maxY };
        octagons[2] = INT_OCTAGON(
                octagon.leftX, octagon.bottomY, box.minX, octagon.topY,
                octagon.upperLeftDiagonalX, octagon.lowerRightDiagonalX,
                octagon.lowerLeftDiagonalX, octagon.upperRightDiagonalX ).Normalize();
    }

    box = boxes[2];
    octagon = octagons[2];
    if( box.maxY - box.minY > octagon.rightX - octagon.leftX )
    {
        boxes[2] = { box.minX, box.minY, octagon.rightX, box.maxY };
        octagons[2] = INT_OCTAGON(
                octagon.leftX, box.maxY, octagon.rightX, octagon.topY,
                octagon.upperLeftDiagonalX, octagon.lowerRightDiagonalX,
                octagon.lowerLeftDiagonalX, octagon.upperRightDiagonalX ).Normalize();
    }

    box = boxes[2];
    octagon = octagons[1];
    if( box.maxY - box.minY > octagon.rightX - octagon.leftX )
    {
        boxes[2] = { octagon.leftX, box.minY, box.maxX, box.maxY };
        octagons[1] = INT_OCTAGON(
                octagon.leftX, box.maxY, octagon.rightX, octagon.topY,
                octagon.upperLeftDiagonalX, octagon.lowerRightDiagonalX,
                octagon.lowerLeftDiagonalX, octagon.upperRightDiagonalX ).Normalize();
    }

    box = boxes[0];
    octagon = octagons[1];
    if( box.maxX - box.minX > octagon.topY - octagon.bottomY )
    {
        boxes[0] = { box.minX, octagon.bottomY, box.maxX, box.maxY };
        octagons[1] = INT_OCTAGON(
                box.maxX, octagon.bottomY, octagon.rightX, octagon.topY,
                octagon.upperLeftDiagonalX, octagon.lowerRightDiagonalX,
                octagon.lowerLeftDiagonalX, octagon.upperRightDiagonalX ).Normalize();
    }

    return { rawBoxOctagon( boxes[0] ), rawBoxOctagon( boxes[1] ),
             rawBoxOctagon( boxes[2] ), rawBoxOctagon( boxes[3] ),
             octagons[0], octagons[1], octagons[2], octagons[3] };
}


std::pair<double, double> INT_OCTAGON::CentreOfGravity() const
{
    if( Dimension() < 0 )
        return { 0.0, 0.0 };

    long double x = 0;
    long double y = 0;
    for( int index = 0; index < 8; ++index )
    {
        x += CornerX( index );
        y += CornerY( index );
    }
    return { static_cast<double>( x / 8.0L ), static_cast<double>( y / 8.0L ) };
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


ROUTER_POINT INT_OCTAGON::BorderPoint( ROUTER_POINT aPoint,
                                       DIRECTION_45 aDirection ) const
{
    std::int64_t x = 0;
    std::int64_t y = 0;
    switch( aDirection )
    {
    case DIRECTION_45::RIGHT:
        x = std::min( rightX, subtract( upperRightDiagonalX, aPoint.y ) );
        x = std::min( x, add( lowerRightDiagonalX, aPoint.y ) );
        y = aPoint.y;
        break;
    case DIRECTION_45::LEFT:
        x = std::max( leftX, add( upperLeftDiagonalX, aPoint.y ) );
        x = std::max( x, subtract( lowerLeftDiagonalX, aPoint.y ) );
        y = aPoint.y;
        break;
    case DIRECTION_45::UP:
        x = aPoint.x;
        y = std::min( topY, subtract( aPoint.x, upperLeftDiagonalX ) );
        y = std::min( y, subtract( upperRightDiagonalX, aPoint.x ) );
        break;
    case DIRECTION_45::DOWN:
        x = aPoint.x;
        y = std::max( bottomY, subtract( lowerLeftDiagonalX, aPoint.x ) );
        y = std::max( y, subtract( aPoint.x, lowerRightDiagonalX ) );
        break;
    case DIRECTION_45::RIGHT45:
        x = ceilHalf( INTEGER( aPoint.x ) - aPoint.y + upperRightDiagonalX );
        x = std::min( x, rightX );
        x = std::min( x, add( subtract( aPoint.x, aPoint.y ), topY ) );
        y = add( subtract( aPoint.y, aPoint.x ), x );
        break;
    case DIRECTION_45::UP45:
        x = floorHalf( INTEGER( aPoint.x ) + aPoint.y + upperLeftDiagonalX );
        x = std::max( x, leftX );
        x = std::max( x, subtract( add( aPoint.x, aPoint.y ), topY ) );
        y = subtract( add( aPoint.y, aPoint.x ), x );
        break;
    case DIRECTION_45::LEFT45:
        x = floorHalf( INTEGER( aPoint.x ) - aPoint.y + lowerLeftDiagonalX );
        x = std::max( x, leftX );
        x = std::max( x, add( subtract( aPoint.x, aPoint.y ), bottomY ) );
        y = add( subtract( aPoint.y, aPoint.x ), x );
        break;
    case DIRECTION_45::DOWN45:
        x = ceilHalf( INTEGER( aPoint.x ) + aPoint.y + lowerRightDiagonalX );
        x = std::min( x, rightX );
        x = std::min( x, subtract( add( aPoint.x, aPoint.y ), bottomY ) );
        y = subtract( add( aPoint.y, aPoint.x ), x );
        break;
    }
    return { x, y };
}


std::vector<ROUTER_POINT> INT_OCTAGON::NearestBorderProjections(
        ROUTER_POINT aPoint, int aMaximumResultPoints ) const
{
    if( !Contains( aPoint ) || aMaximumResultPoints <= 0 )
        return {};

    const int resultCount = std::min( aMaximumResultPoints, 8 );
    struct CANDIDATE
    {
        ROUTER_POINT point;
        INTEGER distanceSquared;
    };
    std::vector<CANDIDATE> candidates;
    candidates.reserve( 8 );
    for( int direction = 0; direction < 8; ++direction )
    {
        const ROUTER_POINT point = BorderPoint(
                aPoint, static_cast<DIRECTION_45>( direction ) );
        const INTEGER dx = INTEGER( point.x ) - aPoint.x;
        const INTEGER dy = INTEGER( point.y ) - aPoint.y;
        candidates.push_back( { point, dx * dx + dy * dy } );
    }
    std::stable_sort( candidates.begin(), candidates.end(),
                      []( const CANDIDATE& aLeft, const CANDIDATE& aRight )
                      {
                          return aLeft.distanceSquared < aRight.distanceSquared;
                      } );

    std::vector<ROUTER_POINT> result;
    result.reserve( resultCount );
    for( int index = 0; index < resultCount; ++index )
        result.push_back( candidates[index].point );
    return result;
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
    if( IsEmpty() )
        return SIMPLEX::Empty();

    // IntOctagon.toSimplex constructs all eight source border supports and
    // then removes redundant lines.  Retaining these exact anchors is
    // important for deterministic line identity after enlarge/intersection;
    // rebuilding from corners creates equivalent geometry but different
    // Line values.
    std::vector<LINE> lines;
    lines.reserve( 8 );
    for( int index = 0; index < 8; ++index )
        lines.push_back( BorderLine( index ) );
    return SIMPLEX::GetInstance( std::move( lines ) );
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
