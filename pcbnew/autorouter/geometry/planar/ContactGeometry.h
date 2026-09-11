/* KiCad, GPL-3.0-or-later. Exact contact adapter translated from Freerouting's
 * rational Point/Line/Polyline geometry. Host-facing adapters never round.
 */
#pragma once
#include <boost/multiprecision/cpp_int.hpp>
#include "../../AutorouterTypes.h"
#include "Line.h"

namespace KICAD_AUTOROUTER::CONTACT_GEOMETRY
{
// KiCad coordinates fit signed 32 bits. Products of differences need more
// than int64; intersection numerators need more than 64 bits as well.
using WIDE = boost::multiprecision::int128_t;
inline WIDE Cross( ROUTER_POINT a, ROUTER_POINT b, ROUTER_POINT c )
{
    return ( WIDE( b.x ) - a.x ) * ( WIDE( c.y ) - a.y )
           - ( WIDE( b.y ) - a.y ) * ( WIDE( c.x ) - a.x );
}
inline bool OnSegment( ROUTER_POINT a, ROUTER_POINT b, ROUTER_POINT p )
{
    return Cross( a, b, p ) == 0 && p.x >= std::min( a.x, b.x )
           && p.x <= std::max( a.x, b.x ) && p.y >= std::min( a.y, b.y )
           && p.y <= std::max( a.y, b.y );
}

/** Exact LineSegment.contains(Point), including a RationalPoint. */
inline bool ExactOnSegment( ROUTER_POINT a, ROUTER_POINT b, const PLANAR::POINT& p )
{
    if( a == b )
        return p == PLANAR::POINT( a );

    const PLANAR::LINE support( a, b );
    if( support.SideOf( p ) != 0 )
        return false;

    const PLANAR::POINT first( a );
    const PLANAR::POINT last( b );
    const bool betweenX = ( p.CompareX( first ) >= 0 && p.CompareX( last ) <= 0 )
                          || ( p.CompareX( last ) >= 0 && p.CompareX( first ) <= 0 );
    const bool betweenY = ( p.CompareY( first ) >= 0 && p.CompareY( last ) <= 0 )
                          || ( p.CompareY( last ) >= 0 && p.CompareY( first ) <= 0 );
    return betweenX && betweenY;
}
/** -1 outside, 0 boundary, 1 inside. No epsilon or floating ray crossings. */
inline int Locate( const std::vector<ROUTER_POINT>& polygon, ROUTER_POINT p )
{
    if( polygon.size() < 3 )
        return -1;
    int winding = 0;
    for( std::size_t i = 0; i < polygon.size(); ++i )
    {
        const auto a = polygon[i], b = polygon[( i + 1 ) % polygon.size()];
        if( OnSegment( a, b, p ) )
            return 0;
        const auto side = Cross( a, b, p );
        if( a.y <= p.y && b.y > p.y && side > 0 )
            ++winding;
        else if( a.y > p.y && b.y <= p.y && side < 0 )
            --winding;
    }
    return winding == 0 ? -1 : 1;
}
/** PolylineArea.contains(Point): hole interiors excluded, boundaries included.
 * Copper dilation/curves are not modeled here; this is the straight area subset.
 */
inline bool ContainsArea( const ROUTING_OBSTACLE& area, ROUTER_POINT p )
{
    if( area.radius != 0 )
        return false;
    const bool inside = area.kind == ROUTER_OBSTACLE_KIND::RECTANGLE
                                ? area.box.Contains( p )
                                : area.kind == ROUTER_OBSTACLE_KIND::POLYGON
                                          && Locate( area.polygon, p ) >= 0;
    return inside && std::none_of( area.polygonHoles.begin(), area.polygonHoles.end(),
                                  [&]( const auto& hole ) { return Locate( hole, p ) > 0; } );
}
/** Exact finite intersection of two closed integer line segments. */
inline std::optional<PLANAR::POINT> ExactIntersection( ROUTER_POINT a, ROUTER_POINT b,
                                                       ROUTER_POINT c, ROUTER_POINT d )
{
    if( a == b || c == d ) return {};
    const auto exact = PLANAR::LINE( a, b ).Intersection( PLANAR::LINE( c, d ) );
    if( !exact || !ExactOnSegment( a, b, *exact )
        || !ExactOnSegment( c, d, *exact ) )
        return {};
    return exact;
}

inline std::optional<ROUTER_POINT> Intersection( ROUTER_POINT a, ROUTER_POINT b,
                                                ROUTER_POINT c, ROUTER_POINT d )
{
    const auto exact = ExactIntersection( a, b, c, d );
    if( !exact ) return {};
    const auto p = exact->Integral();
    // Host copper still has integral vertices. The rational intersection is
    // preserved by the planar kernel; never round it into a false junction.
    return p;
}
} // namespace KICAD_AUTOROUTER::CONTACT_GEOMETRY
