/* KiCad, GPL-3.0-or-later. Freerouting Line's directed integer support lines
 * with arbitrary precision intermediate arithmetic and rational intersections.
 */
#pragma once
#include "Point.h"
#include <cmath>

namespace KICAD_AUTOROUTER::PLANAR
{
class LINE
{
public:
    ROUTER_POINT a, b;
    LINE( ROUTER_POINT aA, ROUTER_POINT aB ) : a( aA ), b( aB )
    { if( a == b ) throw std::invalid_argument( "degenerate support line" ); }
    static std::optional<LINE> FromDirection( ROUTER_POINT aPoint,
                                               INTEGER aDx, INTEGER aDy )
    {
        const INTEGER divisor = Gcd( aDx, aDy );
        if( divisor == 0 )
            return {};
        aDx /= divisor;
        aDy /= divisor;
        const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
        const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
        const INTEGER endX = INTEGER( aPoint.x ) + aDx;
        const INTEGER endY = INTEGER( aPoint.y ) + aDy;
        if( endX < minimum || endX > maximum || endY < minimum || endY > maximum )
            return {};
        return LINE( aPoint, { endX.convert_to<std::int64_t>(),
                               endY.convert_to<std::int64_t>() } );
    }
    INTEGER Dx() const { return INTEGER( b.x ) - a.x; }
    INTEGER Dy() const { return INTEGER( b.y ) - a.y; }
    INTEGER DirectionDeterminant( const LINE& aOther ) const
    { return Dx() * aOther.Dy() - Dy() * aOther.Dx(); }
    INTEGER DirectionScalarProduct( const LINE& aOther ) const
    { return Dx() * aOther.Dx() + Dy() * aOther.Dy(); }
    // Java Line.sideOf(Point), NOT Point.sideOf(Line): left is +1.
    int SideOf( const POINT& p ) const
    { return Sign( Dy() * ( p.x - INTEGER( a.x ) * p.z )
                   - Dx() * ( p.y - INTEGER( a.y ) * p.z ) ); }
    bool Parallel( const LINE& line ) const { return Dx() * line.Dy() == Dy() * line.Dx(); }
    bool Perpendicular( const LINE& aOther ) const
    { return DirectionScalarProduct( aOther ) == 0; }
    bool SameDirection( const LINE& aOther ) const
    { return Parallel( aOther ) && DirectionScalarProduct( aOther ) > 0; }
    bool EqualOrOpposite( const LINE& line ) const
    { return Parallel( line ) && SideOf( POINT( line.a ) ) == 0; }
    bool SameDirectedSupport( const LINE& aOther ) const
    { return SameDirection( aOther ) && SideOf( POINT( aOther.a ) ) == 0; }

    /** Freerouting Line.compareTo direction order.  The positive x axis is
     * first, followed counter-clockwise through the upper half-plane, the
     * negative x axis, and the lower half-plane.  Collinear forward
     * directions compare equal independently of vector magnitude. */
    int CompareDirection( const LINE& aOther ) const
    {
        const INTEGER dx1 = Dx();
        const INTEGER dy1 = Dy();
        const INTEGER dx2 = aOther.Dx();
        const INTEGER dy2 = aOther.Dy();

        if( dy1 > 0 )
        {
            if( dy2 < 0 )
                return -1;
            if( dy2 == 0 )
                return dx2 > 0 ? 1 : -1;
        }
        else if( dy1 < 0 )
        {
            if( dy2 >= 0 )
                return 1;
        }
        else
        {
            if( dx1 > 0 )
                return dy2 != 0 || dx2 < 0 ? -1 : 0;

            if( dy2 > 0 || ( dy2 == 0 && dx2 > 0 ) )
                return 1;
            if( dy2 < 0 )
                return -1;
            return 0;
        }

        // Same open horizontal half-plane.  This is the exact equivalent of
        // source dx2 * dy1 - dy2 * dx1 without its floating overflow risk.
        return Sign( dx2 * dy1 - dy2 * dx1 );
    }

    bool IsOrthogonal() const { return Dx() == 0 || Dy() == 0; }
    bool IsDiagonal() const
    {
        const INTEGER dx = Dx();
        const INTEGER dy = Dy();
        return ( dx < 0 ? -dx : dx ) == ( dy < 0 ? -dy : dy );
    }
    bool IsMultipleOf45Degree() const { return IsOrthogonal() || IsDiagonal(); }
    LINE Opposite() const { return { b, a }; }
    /** Freerouting Line.translate. Positive distance moves this directed
     * support to its left; Java Math.round semantics are preserved. */
    std::optional<LINE> Translate( double aDistance ) const
    {
        const INTEGER divisor = Gcd( Dx(), Dy() );
        if( divisor == 0 )
            return {};
        const INTEGER normalizedDx = Dx() / divisor;
        const INTEGER normalizedDy = Dy() / divisor;
        const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
        const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
        if( normalizedDx < minimum || normalizedDx > maximum
            || normalizedDy < minimum || normalizedDy > maximum )
        {
            return {};
        }
        const std::int64_t dx = normalizedDx.convert_to<std::int64_t>();
        const std::int64_t dy = normalizedDy.convert_to<std::int64_t>();
        const double dxSquared = static_cast<double>( dx ) * dx;
        const double dySquared = static_cast<double>( dy ) * dy;
        const double length = std::sqrt( dxSquared + dySquared );
        INTEGER newX = a.x;
        INTEGER newY = a.y;
        const auto javaRound = []( double aValue )
        {
            const double result = std::floor( aValue + 0.5 );
            if( result < static_cast<double>( std::numeric_limits<std::int64_t>::min() )
                || result > static_cast<double>( std::numeric_limits<std::int64_t>::max() ) )
            {
                return std::optional<std::int64_t>{};
            }
            return std::optional<std::int64_t>{ static_cast<std::int64_t>( result ) };
        };
        if( dxSquared <= dySquared )
        {
            const auto relativeX = javaRound( aDistance * length / dy );
            if( !relativeX )
                return {};
            newX -= *relativeX;
        }
        else
        {
            const auto relativeY = javaRound( aDistance * length / dx );
            if( !relativeY )
                return {};
            newY += *relativeY;
        }
        const INTEGER endX = newX + dx;
        const INTEGER endY = newY + dy;
        if( newX < minimum || newX > maximum || newY < minimum || newY > maximum
            || endX < minimum || endX > maximum || endY < minimum || endY > maximum )
        {
            return {};
        }
        return LINE( { newX.convert_to<std::int64_t>(), newY.convert_to<std::int64_t>() },
                     { endX.convert_to<std::int64_t>(), endY.convert_to<std::int64_t>() } );
    }
    std::optional<LINE> TranslateBy( ROUTER_POINT aVector ) const
    {
        const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
        const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
        const INTEGER ax = INTEGER( a.x ) + aVector.x;
        const INTEGER ay = INTEGER( a.y ) + aVector.y;
        const INTEGER bx = INTEGER( b.x ) + aVector.x;
        const INTEGER by = INTEGER( b.y ) + aVector.y;
        if( ax < minimum || ax > maximum || ay < minimum || ay > maximum
            || bx < minimum || bx > maximum || by < minimum || by > maximum )
        {
            return {};
        }
        return LINE( { ax.convert_to<std::int64_t>(), ay.convert_to<std::int64_t>() },
                     { bx.convert_to<std::int64_t>(), by.convert_to<std::int64_t>() } );
    }
    double SignedDistance( double aX, double aY ) const
    {
        const double dx = Dx().convert_to<double>();
        const double dy = Dy().convert_to<double>();
        return ( dy * ( aX - a.x ) - dx * ( aY - a.y ) )
               / std::hypot( dx, dy );
    }
    std::pair<double, double> ProjectionApprox( double aX, double aY ) const
    {
        const double dx = Dx().convert_to<double>();
        const double dy = Dy().convert_to<double>();
        const double ratio = ( ( aX - a.x ) * dx + ( aY - a.y ) * dy )
                             / ( dx * dx + dy * dy );
        return { a.x + ratio * dx, a.y + ratio * dy };
    }
    double CosAngle( const LINE& aOther ) const
    {
        const double dx = Dx().convert_to<double>();
        const double dy = Dy().convert_to<double>();
        const double otherDx = aOther.Dx().convert_to<double>();
        const double otherDy = aOther.Dy().convert_to<double>();
        return ( dx * otherDx + dy * otherDy )
               / ( std::hypot( dx, dy ) * std::hypot( otherDx, otherDy ) );
    }
    double FunctionValueApprox( double aX ) const
    {
        const double dx = Dx().convert_to<double>();
        if( dx == 0 )
            return 0;
        const double dy = Dy().convert_to<double>();
        const double determinant = static_cast<double>( a.x ) * b.y
                                   - static_cast<double>( b.x ) * a.y;
        return ( dy * aX - determinant ) / dx;
    }
    double FunctionInYValueApprox( double aY ) const
    {
        const double dy = Dy().convert_to<double>();
        if( dy == 0 )
            return 0;
        const double dx = Dx().convert_to<double>();
        const double determinant = static_cast<double>( a.x ) * b.y
                                   - static_cast<double>( b.x ) * a.y;
        return ( dx * aY + determinant ) / dy;
    }
    /** Normalized direction from aPoint toward this support line, matching
     * Line.perpendicularDirection. A null result means aPoint is collinear. */
    std::optional<ROUTER_POINT> PerpendicularDirection( const POINT& aPoint ) const
    {
        const int lineSide = SideOf( aPoint );
        if( lineSide == 0 )
            return {};
        INTEGER dx = Dx();
        INTEGER dy = Dy();
        const INTEGER divisor = Gcd( dx, dy );
        dx /= divisor;
        dy /= divisor;
        const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
        const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
        const auto integralDirection = [&]( const INTEGER& aDirectionX,
                                            const INTEGER& aDirectionY )
                -> std::optional<ROUTER_POINT>
        {
            if( aDirectionX < minimum || aDirectionX > maximum
                || aDirectionY < minimum || aDirectionY > maximum )
            {
                return {};
            }
            return ROUTER_POINT{ aDirectionX.convert_to<std::int64_t>(),
                                 aDirectionY.convert_to<std::int64_t>() };
        };
        const auto first = integralDirection( -dy, dx );
        const auto second = integralDirection( dy, -dx );
        if( !first || !second )
            return {};
        const auto moved = [&]( ROUTER_POINT aDirection )
        {
            return POINT( aPoint.x + INTEGER( aDirection.x ) * aPoint.z,
                          aPoint.y + INTEGER( aDirection.y ) * aPoint.z,
                          aPoint.z );
        };
        const POINT firstCheck = moved( *first );
        if( SideOf( firstCheck ) != lineSide )
            return first;
        const POINT secondCheck = moved( *second );
        if( SideOf( secondCheck ) != lineSide )
            return second;
        const auto projection = ProjectionApprox( aPoint.X(), aPoint.Y() );
        const auto distanceSquared = [&]( const POINT& aCheck )
        {
            const double checkX = aCheck.X() - projection.first;
            const double checkY = aCheck.Y() - projection.second;
            return checkX * checkX + checkY * checkY;
        };
        return distanceSquared( firstCheck ) <= distanceSquared( secondCheck )
               ? first : second;
    }
    std::optional<LINE> Turn90Degree( int aFactor, ROUTER_POINT aPole ) const
    {
        int factor = aFactor % 4;
        if( factor < 0 )
            factor += 4;
        const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
        const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
        const auto turn = [&]( ROUTER_POINT aPoint ) -> std::optional<ROUTER_POINT>
        {
            const INTEGER dx = INTEGER( aPoint.x ) - aPole.x;
            const INTEGER dy = INTEGER( aPoint.y ) - aPole.y;
            INTEGER x;
            INTEGER y;
            switch( factor )
            {
            case 0: x = dx; y = dy; break;
            case 1: x = -dy; y = dx; break;
            case 2: x = -dx; y = -dy; break;
            default: x = dy; y = -dx; break;
            }
            x += aPole.x;
            y += aPole.y;
            if( x < minimum || x > maximum || y < minimum || y > maximum )
                return {};
            return ROUTER_POINT{ x.convert_to<std::int64_t>(),
                                 y.convert_to<std::int64_t>() };
        };
        const auto newA = turn( a );
        const auto newB = turn( b );
        if( !newA || !newB )
            return {};
        return LINE( *newA, *newB );
    }
    std::optional<LINE> MirrorVertical( ROUTER_POINT aPole ) const
    {
        const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
        const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
        const auto mirror = [&]( ROUTER_POINT aPoint ) -> std::optional<ROUTER_POINT>
        {
            const INTEGER x = INTEGER( 2 ) * aPole.x - aPoint.x;
            if( x < minimum || x > maximum )
                return {};
            return ROUTER_POINT{ x.convert_to<std::int64_t>(), aPoint.y };
        };
        // Reflection changes handedness; source reverses the endpoints.
        const auto newA = mirror( b );
        const auto newB = mirror( a );
        if( !newA || !newB )
            return {};
        return LINE( *newA, *newB );
    }
    std::optional<LINE> MirrorHorizontal( ROUTER_POINT aPole ) const
    {
        const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
        const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
        const auto mirror = [&]( ROUTER_POINT aPoint ) -> std::optional<ROUTER_POINT>
        {
            const INTEGER y = INTEGER( 2 ) * aPole.y - aPoint.y;
            if( y < minimum || y > maximum )
                return {};
            return ROUTER_POINT{ aPoint.x, y.convert_to<std::int64_t>() };
        };
        const auto newA = mirror( b );
        const auto newB = mirror( a );
        if( !newA || !newB )
            return {};
        return LINE( *newA, *newB );
    }
    float Length() const
    {
        const double dx = Dx().convert_to<double>();
        const double dy = Dy().convert_to<double>();
        return static_cast<float>( std::sqrt( dx * dx + dy * dy ) );
    }
    POINT PerpendicularProjection( const POINT& aPoint ) const
    {
        const INTEGER dx = Dx();
        const INTEGER dy = Dy();
        const INTEGER denominator = dx * dx + dy * dy;
        const INTEGER determinant = INTEGER( a.x ) * b.y - INTEGER( a.y ) * b.x;
        const INTEGER projectedX = dx * dx * aPoint.x + dx * dy * aPoint.y
                                   + determinant * dy * aPoint.z;
        // RationalPoint.perpendicularProjection has a distinct signed term
        // from IntPoint in the pinned source. Preserve that observable class
        // behavior after collapsing both Java point classes into POINT.
        INTEGER signedTerm;
        if( aPoint.z == 1 )
            signedTerm = -determinant * dx;
        else
            signedTerm = determinant * dx * aPoint.z;
        const INTEGER projectedY = dx * dy * aPoint.x + dy * dy * aPoint.y
                                   + signedTerm;
        return POINT( projectedX, projectedY, denominator );
    }
    std::optional<POINT> Intersection( const LINE& line ) const
    {
        const INTEGER den = Dx() * line.Dy() - Dy() * line.Dx();
        if( den == 0 ) return {};
        const INTEGER t = ( INTEGER( line.a.x ) - a.x ) * line.Dy()
                          - ( INTEGER( line.a.y ) - a.y ) * line.Dx();
        return POINT( INTEGER( a.x ) * den + Dx() * t,
                      INTEGER( a.y ) * den + Dy() * t, den );
    }
};
} // namespace KICAD_AUTOROUTER::PLANAR
