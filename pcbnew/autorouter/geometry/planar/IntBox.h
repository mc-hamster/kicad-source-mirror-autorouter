/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Rectangle operations used by the Freerouting orthogonal search port.
 */
#pragma once

#include "../../AutorouterTypes.h"
#include <cmath>

namespace KICAD_AUTOROUTER::INT_BOX
{
inline ROUTER_BOX Empty() { return { 1, 1, 0, 0 }; }

inline int Dimension( const ROUTER_BOX& aBox )
{
    if( aBox.minX > aBox.maxX || aBox.minY > aBox.maxY )
        return -1;
    return ( aBox.minX < aBox.maxX ) + ( aBox.minY < aBox.maxY );
}

inline ROUTER_BOX Intersection( const ROUTER_BOX& aLeft, const ROUTER_BOX& aRight )
{
    return { std::max( aLeft.minX, aRight.minX ), std::max( aLeft.minY, aRight.minY ),
             std::min( aLeft.maxX, aRight.maxX ), std::min( aLeft.maxY, aRight.maxY ) };
}

inline bool Intersects( const ROUTER_BOX& aLeft, const ROUTER_BOX& aRight )
{
    return Dimension( aLeft ) >= 0 && Dimension( aRight ) >= 0
           && Dimension( Intersection( aLeft, aRight ) ) >= 0;
}

inline bool Overlaps( const ROUTER_BOX& aLeft, const ROUTER_BOX& aRight )
{
    // IntBox.overlaps tests separating edges, NOT intersection.dimension()==2.
    // A zero-width restraint strictly inside a room must still split it.
    return aRight.minX < aLeft.maxX && aRight.minY < aLeft.maxY
           && aLeft.minX < aRight.maxX && aLeft.minY < aRight.maxY;
}

inline bool Contains( const ROUTER_BOX& aOuter, const ROUTER_BOX& aInner )
{
    return Dimension( aInner ) >= 0 && aOuter.minX <= aInner.minX
           && aOuter.minY <= aInner.minY && aOuter.maxX >= aInner.maxX
           && aOuter.maxY >= aInner.maxY;
}

inline ROUTER_BOX Union( const ROUTER_BOX& aLeft, const ROUTER_BOX& aRight )
{
    if( Dimension( aLeft ) < 0 )
        return aRight;
    if( Dimension( aRight ) < 0 )
        return aLeft;
    return { std::min( aLeft.minX, aRight.minX ), std::min( aLeft.minY, aRight.minY ),
             std::max( aLeft.maxX, aRight.maxX ), std::max( aLeft.maxY, aRight.maxY ) };
}

inline double Area( const ROUTER_BOX& aBox )
{
    if( Dimension( aBox ) != 2 )
        return 0;
    // Match the reference's floating area comparison, without integer overflow.
    return ( static_cast<double>( aBox.maxX ) - aBox.minX )
           * ( static_cast<double>( aBox.maxY ) - aBox.minY );
}

// IntBox.weightedDistance, including the overlapping-axis fast paths.
inline double WeightedDistance( const ROUTER_BOX& aBox, const ROUTER_BOX& aOther,
                                double aHorizontalWeight, double aVerticalWeight )
{
    const double maxLlX = std::max( aBox.minX, aOther.minX );
    const double maxLlY = std::max( aBox.minY, aOther.minY );
    const double minUrX = std::min( aBox.maxX, aOther.maxX );
    const double minUrY = std::min( aBox.maxY, aOther.maxY );
    if( minUrX >= maxLlX )
        return std::max( aVerticalWeight * ( maxLlY - minUrY ), 0.0 );
    if( minUrY >= maxLlY )
        return std::max( aHorizontalWeight * ( maxLlX - minUrX ), 0.0 );
    const double dx = ( maxLlX - minUrX ) * aHorizontalWeight;
    const double dy = ( maxLlY - minUrY ) * aVerticalWeight;
    return std::sqrt( dx * dx + dy * dy );
}

// IntPoint/IntBox IDs use Java's defined 32-bit wrap, not signed C++ overflow.
inline std::int32_t PointId( ROUTER_POINT aPoint )
{
    return static_cast<std::int32_t>( 31u * static_cast<std::uint32_t>( aPoint.x )
                                     + static_cast<std::uint32_t>( aPoint.y ) );
}

inline std::int32_t Id( ROUTER_BOX aBox )
{
    return static_cast<std::int32_t>( 31u * static_cast<std::uint32_t>( PointId( { aBox.minX, aBox.minY } ) )
                                     + static_cast<std::uint32_t>( PointId( { aBox.maxX, aBox.maxY } ) ) );
}

/** IntBox.cutout/cutoutFrom at a11c0a42, including perimeter-minimizing ties.
 * Keep degenerate pieces here; PolylineArea, not cutout, filters them.
 */
inline std::vector<ROUTER_BOX> Cutout( ROUTER_BOX d, ROUTER_BOX aHole )
{
    const auto c = Intersection( aHole, d );
    if( Dimension( aHole ) < 0 || Dimension( c ) < Dimension( aHole ) )
        return { d };
    std::vector<ROUTER_BOX> r{
        { d.minX, d.minY, c.maxX, c.minY },
        { d.minX, c.minY, c.minX, d.maxY },
        { c.maxX, d.minY, d.maxX, c.maxY },
        { c.minX, c.maxY, d.maxX, d.maxY }
    };
    if( c.minX - d.minX > c.minY - d.minY )
    { r[0].minX = c.minX; r[1].minY = d.minY; }
    if( d.maxY - c.maxY > c.minX - d.minX )
    { r[1].maxY = c.maxY; r[3].minX = d.minX; }
    if( d.maxX - c.maxX > d.maxY - c.maxY )
    { r[2].maxY = d.maxY; r[3].maxX = c.maxX; }
    if( c.minY - d.minY > d.maxX - c.maxX )
    { r[0].maxX = d.maxX; r[2].minY = c.minY; }
    return r;
}
} // namespace KICAD_AUTOROUTER::INT_BOX
