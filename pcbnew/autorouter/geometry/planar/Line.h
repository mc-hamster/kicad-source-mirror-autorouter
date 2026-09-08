/* KiCad, GPL-3.0-or-later. Freerouting Line's directed integer support lines
 * with arbitrary precision intermediate arithmetic and rational intersections.
 */
#pragma once
#include "Point.h"

namespace KICAD_AUTOROUTER::PLANAR
{
class LINE
{
public:
    ROUTER_POINT a, b;
    LINE( ROUTER_POINT aA, ROUTER_POINT aB ) : a( aA ), b( aB )
    { if( a == b ) throw std::invalid_argument( "degenerate support line" ); }
    INTEGER Dx() const { return INTEGER( b.x ) - a.x; }
    INTEGER Dy() const { return INTEGER( b.y ) - a.y; }
    // Java Line.sideOf(Point), NOT Point.sideOf(Line): left is +1.
    int SideOf( const POINT& p ) const
    { return Sign( Dy() * ( p.x - INTEGER( a.x ) * p.z )
                   - Dx() * ( p.y - INTEGER( a.y ) * p.z ) ); }
    bool Parallel( const LINE& line ) const { return Dx() * line.Dy() == Dy() * line.Dx(); }
    bool EqualOrOpposite( const LINE& line ) const
    { return Parallel( line ) && SideOf( POINT( line.a ) ) == 0; }
    LINE Opposite() const { return { b, a }; }
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
