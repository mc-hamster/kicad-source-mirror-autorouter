/* KiCad, GPL-3.0-or-later. Exact integer contact adapter, NOT a full port of
 * Freerouting's rational Point/Line/Polyline geometry. Never rounds junctions.
 */
#pragma once
#include <boost/multiprecision/cpp_int.hpp>
#include "../../AutorouterTypes.h"

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
inline std::optional<ROUTER_POINT> Intersection( ROUTER_POINT a, ROUTER_POINT b,
                                                ROUTER_POINT c, ROUTER_POINT d )
{
    WIDE rx = WIDE( b.x ) - a.x, ry = WIDE( b.y ) - a.y;
    WIDE sx = WIDE( d.x ) - c.x, sy = WIDE( d.y ) - c.y;
    WIDE den = rx * sy - ry * sx;
    if( den == 0 )
        return {}; // collinear endpoints are handled separately
    WIDE t = ( WIDE( c.x ) - a.x ) * sy - ( WIDE( c.y ) - a.y ) * sx;
    WIDE x = WIDE( a.x ) * den + rx * t, y = WIDE( a.y ) * den + ry * t;
    if( x % den != 0 || y % den != 0 )
        return {}; // rational contact unsupported: never manufacture a rounded split
    x /= den;
    y /= den;
    if( x < std::min( a.x, b.x ) || x > std::max( a.x, b.x )
        || y < std::min( a.y, b.y ) || y > std::max( a.y, b.y ) )
        return {};
    ROUTER_POINT p{ x.convert_to<std::int64_t>(), y.convert_to<std::int64_t>() };
    return OnSegment( c, d, p ) ? std::optional( p ) : std::nullopt;
}
} // namespace KICAD_AUTOROUTER::CONTACT_GEOMETRY
