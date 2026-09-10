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
    INTEGER DirectionDeterminant( const LINE& aOther ) const
    { return Dx() * aOther.Dy() - Dy() * aOther.Dx(); }
    INTEGER DirectionScalarProduct( const LINE& aOther ) const
    { return Dx() * aOther.Dx() + Dy() * aOther.Dy(); }
    // Java Line.sideOf(Point), NOT Point.sideOf(Line): left is +1.
    int SideOf( const POINT& p ) const
    { return Sign( Dy() * ( p.x - INTEGER( a.x ) * p.z )
                   - Dx() * ( p.y - INTEGER( a.y ) * p.z ) ); }
    bool Parallel( const LINE& line ) const { return Dx() * line.Dy() == Dy() * line.Dx(); }
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
