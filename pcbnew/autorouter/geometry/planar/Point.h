/* KiCad, GPL-3.0-or-later.
 * Freerouting geometry/planar/{Point,RationalPoint}.java at a11c0a42.
 * Exact finite homogeneous coordinates. C++ value semantics replace the Java
 * IntPoint/RationalPoint class split; infinity is rejected, never rounded.
 */
#pragma once
#include "../../AutorouterTypes.h"
#include <boost/multiprecision/cpp_int.hpp>
#include <limits>
#include <stdexcept>

namespace KICAD_AUTOROUTER::PLANAR
{
using INTEGER = boost::multiprecision::cpp_int;
inline int Sign( const INTEGER& n ) { return n == 0 ? 0 : n > 0 ? 1 : -1; }
inline INTEGER Gcd( INTEGER a, INTEGER b )
{
    if( a < 0 ) a = -a;
    if( b < 0 ) b = -b;
    while( b != 0 ) { INTEGER r = a % b; a = b; b = r; }
    return a;
}

class POINT
{
public:
    INTEGER x, y, z;
    POINT( INTEGER aX, INTEGER aY, INTEGER aZ = 1 ) :
            x( std::move( aX ) ), y( std::move( aY ) ), z( std::move( aZ ) )
    {
        if( z == 0 ) throw std::domain_error( "non-finite planar point" );
        if( z < 0 ) { x = -x; y = -y; z = -z; }
        const INTEGER gcd = Gcd( Gcd( x, y ), z );
        x /= gcd; y /= gcd; z /= gcd;
    }
    explicit POINT( ROUTER_POINT p ) : POINT( p.x, p.y ) {}
    int CompareX( const POINT& p ) const { return Sign( x * p.z - p.x * z ); }
    int CompareY( const POINT& p ) const { return Sign( y * p.z - p.y * z ); }
    bool operator==( const POINT& p ) const { return x == p.x && y == p.y && z == p.z; }
    double X() const { return x.convert_to<double>() / z.convert_to<double>(); }
    double Y() const { return y.convert_to<double>() / z.convert_to<double>(); }
    double DistanceSquared( const POINT& p ) const
    { const double dx = X() - p.X(), dy = Y() - p.Y(); return dx * dx + dy * dy; }
    std::optional<ROUTER_POINT> Integral() const
    {
        if( z != 1 || x < INT64_MIN || x > INT64_MAX || y < INT64_MIN || y > INT64_MAX )
            return {};
        return ROUTER_POINT{ x.convert_to<std::int64_t>(), y.convert_to<std::int64_t>() };
    }

    /** Smallest integer coordinate box containing this exact finite point.
     *
     * Freerouting keeps rational support intersections until its board-item
     * insertion layer chooses an integral realization.  KiCad geometry is
     * integral, so callers must enumerate the surrounding integer candidates
     * and then prove them legal; silently rounding a support corner can move
     * it through the compensated obstacle boundary.
     */
    std::optional<ROUTER_BOX> SurroundingBox() const
    {
        const auto floorDivide = []( const INTEGER& aNumerator, const INTEGER& aDenominator )
        {
            INTEGER quotient = aNumerator / aDenominator;
            const INTEGER remainder = aNumerator % aDenominator;
            if( remainder != 0 && aNumerator < 0 )
                --quotient;
            return quotient;
        };
        const auto ceilDivide = []( const INTEGER& aNumerator, const INTEGER& aDenominator )
        {
            INTEGER quotient = aNumerator / aDenominator;
            const INTEGER remainder = aNumerator % aDenominator;
            if( remainder != 0 && aNumerator > 0 )
                ++quotient;
            return quotient;
        };
        const INTEGER minX = floorDivide( x, z );
        const INTEGER minY = floorDivide( y, z );
        const INTEGER maxX = ceilDivide( x, z );
        const INTEGER maxY = ceilDivide( y, z );
        const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
        const INTEGER maximum = std::numeric_limits<std::int64_t>::max();

        if( minX < minimum || minX > maximum || minY < minimum || minY > maximum
            || maxX < minimum || maxX > maximum || maxY < minimum || maxY > maximum )
        {
            return {};
        }

        return ROUTER_BOX{ minX.convert_to<std::int64_t>(), minY.convert_to<std::int64_t>(),
                           maxX.convert_to<std::int64_t>(), maxY.convert_to<std::int64_t>() };
    }
};
} // namespace KICAD_AUTOROUTER::PLANAR
