/* KiCad, GPL-3.0-or-later. Translated LineSegment.borderIntersections and
 * TileShape.entrancePoints/cutout(Polyline), Freerouting a11c0a42.
 */
#include "Simplex.h"

#include <algorithm>
#include <array>
#include <limits>

namespace KICAD_AUTOROUTER::PLANAR
{
namespace
{

std::optional<std::int64_t> checkedOffset( std::int64_t aValue, std::int64_t aOffset )
{
    const INTEGER result = INTEGER( aValue ) + aOffset;
    if( result < std::numeric_limits<std::int64_t>::min()
        || result > std::numeric_limits<std::int64_t>::max() )
    {
        return {};
    }

    return result.convert_to<std::int64_t>();
}


int sign( const INTEGER& aValue )
{
    return aValue == 0 ? 0 : aValue < 0 ? -1 : 1;
}


bool isForwardCollinear( const ROUTER_POINT& aPrevious, const ROUTER_POINT& aCurrent,
                         const ROUTER_POINT& aNext )
{
    const INTEGER firstX = INTEGER( aCurrent.x ) - aPrevious.x;
    const INTEGER firstY = INTEGER( aCurrent.y ) - aPrevious.y;
    const INTEGER secondX = INTEGER( aNext.x ) - aCurrent.x;
    const INTEGER secondY = INTEGER( aNext.y ) - aCurrent.y;

    // Freerouting's Polygon constructor removes redundant collinear corners
    // before it constructs TileShape support lines.  Do the same only for a
    // forward continuation: a 180-degree reversal is malformed polygon
    // input, not a harmless support-line subdivision that may be erased.
    return firstX * secondY - firstY * secondX == 0
           && firstX * secondX + firstY * secondY > 0;
}


std::optional<LINE> chebyshevOffset( const LINE& aLine, std::int64_t aRadius )
{
    if( aRadius < 0 )
        return {};

    // For an oriented line whose inside is SideOf <= 0, a square radius r
    // moves the support outward by r * (|dx| + |dy|).  Translating either
    // endpoint by (r*sign(dy), -r*sign(dx)) produces exactly that support
    // constant without a floating normal, division, or rounded vertex.
    const INTEGER dx = aLine.Dx();
    const INTEGER dy = aLine.Dy();
    const INTEGER shiftX = INTEGER( aRadius ) * sign( dy );
    const INTEGER shiftY = -INTEGER( aRadius ) * sign( dx );
    if( shiftX < std::numeric_limits<std::int64_t>::min()
        || shiftX > std::numeric_limits<std::int64_t>::max()
        || shiftY < std::numeric_limits<std::int64_t>::min()
        || shiftY > std::numeric_limits<std::int64_t>::max() )
    {
        return {};
    }

    const auto ax = checkedOffset( aLine.a.x, shiftX.convert_to<std::int64_t>() );
    const auto ay = checkedOffset( aLine.a.y, shiftY.convert_to<std::int64_t>() );
    const auto bx = checkedOffset( aLine.b.x, shiftX.convert_to<std::int64_t>() );
    const auto by = checkedOffset( aLine.b.y, shiftY.convert_to<std::int64_t>() );
    if( !ax || !ay || !bx || !by )
        return {};

    return LINE( { *ax, *ay }, { *bx, *by } );
}

} // namespace


SIMPLEX::SIMPLEX( std::vector<LINE> borders ) : m_borders( std::move( borders ) )
{
    if( m_borders.size() < 3 ) throw std::invalid_argument( "bounded convex shape needs 3 borders" );
    for( std::size_t i = 0; i < m_borders.size(); ++i )
    {
        auto p = m_borders[( i + m_borders.size() - 1 ) % m_borders.size()].Intersection( m_borders[i] );
        if( !p ) throw std::invalid_argument( "parallel adjacent convex borders" );
        for( const auto& line : m_borders )
            if( line.SideOf( *p ) > 0 ) throw std::invalid_argument( "non-convex or unordered borders" );
        m_corners.push_back( *p );
    }
    for( std::size_t i = 0; i < m_corners.size(); ++i )
        if( m_corners[i] == m_corners[( i + 1 ) % m_corners.size()] )
            throw std::invalid_argument( "redundant convex border" );
}
SIMPLEX SIMPLEX::Box( ROUTER_BOX b )
{
    if( b.minX >= b.maxX || b.minY >= b.maxY ) throw std::invalid_argument( "empty convex box" );
    return SIMPLEX( { { { 0, b.minY }, { 1, b.minY } }, { { b.maxX, 0 }, { b.maxX, 1 } },
                      { { 0, b.maxY }, { -1, b.maxY } }, { { b.minX, 0 }, { b.minX, -1 } } } );
}


std::optional<SIMPLEX> SIMPLEX::FromExpandedSegment(
        const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
        std::int64_t aChebyshevRadius )
{
    if( aChebyshevRadius <= 0 || aStart == aEnd )
        return {};

    // The square Minkowski sweep of a segment is the convex hull of the
    // endpoint squares.  Construct that hull with exact integer cross
    // products rather than turning a diagonal trace into its enclosing
    // rectangle (which closes free-space wedges and changes shove order).
    std::vector<ROUTER_POINT> points;
    points.reserve( 8 );
    for( const ROUTER_POINT& endpoint : { aStart, aEnd } )
    {
        for( const std::int64_t offsetX : { -aChebyshevRadius, aChebyshevRadius } )
        {
            for( const std::int64_t offsetY : { -aChebyshevRadius, aChebyshevRadius } )
            {
                const auto x = checkedOffset( endpoint.x, offsetX );
                const auto y = checkedOffset( endpoint.y, offsetY );
                if( !x || !y )
                    return {};
                points.push_back( { *x, *y } );
            }
        }
    }

    std::sort( points.begin(), points.end(), []( const ROUTER_POINT& aLeft,
                                                 const ROUTER_POINT& aRight )
    {
        return aLeft.x != aRight.x ? aLeft.x < aRight.x : aLeft.y < aRight.y;
    } );
    points.erase( std::unique( points.begin(), points.end() ), points.end() );

    const auto cross = []( const ROUTER_POINT& aFirst, const ROUTER_POINT& aSecond,
                           const ROUTER_POINT& aThird )
    {
        return ( INTEGER( aSecond.x ) - aFirst.x ) * ( INTEGER( aThird.y ) - aFirst.y )
               - ( INTEGER( aSecond.y ) - aFirst.y ) * ( INTEGER( aThird.x ) - aFirst.x );
    };
    const auto appendHull = [&]( std::vector<ROUTER_POINT>& aHull,
                                 const ROUTER_POINT& aPoint )
    {
        while( aHull.size() >= 2
               && cross( aHull[aHull.size() - 2], aHull.back(), aPoint ) <= 0 )
        {
            aHull.pop_back();
        }
        aHull.push_back( aPoint );
    };

    std::vector<ROUTER_POINT> lower;
    std::vector<ROUTER_POINT> upper;
    lower.reserve( points.size() );
    upper.reserve( points.size() );
    for( const ROUTER_POINT& point : points )
        appendHull( lower, point );
    for( auto it = points.rbegin(); it != points.rend(); ++it )
        appendHull( upper, *it );

    if( lower.size() < 2 || upper.size() < 2 )
        return {};

    lower.pop_back();
    upper.pop_back();
    lower.insert( lower.end(), upper.begin(), upper.end() );
    return FromConvexPolygon( lower );
}


std::optional<SIMPLEX> SIMPLEX::FromConvexPolygon(
        const std::vector<ROUTER_POINT>& aPolygon, std::int64_t aChebyshevOffset )
{
    if( aChebyshevOffset < 0 || aPolygon.size() < 3 )
        return {};

    std::vector<ROUTER_POINT> points;
    points.reserve( aPolygon.size() );
    for( const ROUTER_POINT& point : aPolygon )
    {
        if( points.empty() || points.back() != point )
            points.push_back( point );
    }
    if( points.size() > 1 && points.front() == points.back() )
        points.pop_back();
    if( points.size() < 3 )
        return {};

    // KiCad polygonal shapes commonly retain collinear vertices after
    // tessellation or line-segment merging.  A convex support-line model has
    // no distinct border at such a point, and rejecting the whole shape
    // silently sent otherwise ordinary convex copper through the sampled
    // fallback.  Normalize the same harmless subdivisions that the source
    // Polygon constructor removes, then validate the remaining turns below.
    bool removedCorner = true;
    while( removedCorner && points.size() >= 3 )
    {
        removedCorner = false;
        std::vector<ROUTER_POINT> normalized;
        normalized.reserve( points.size() );
        for( std::size_t index = 0; index < points.size(); ++index )
        {
            const ROUTER_POINT& previous = points[( index + points.size() - 1 ) % points.size()];
            const ROUTER_POINT& current = points[index];
            const ROUTER_POINT& next = points[( index + 1 ) % points.size()];
            if( isForwardCollinear( previous, current, next ) )
            {
                removedCorner = true;
                continue;
            }
            normalized.push_back( current );
        }
        points = std::move( normalized );
    }
    if( points.size() < 3 )
        return {};

    int winding = 0;
    INTEGER centroidX = 0;
    INTEGER centroidY = 0;
    for( std::size_t index = 0; index < points.size(); ++index )
    {
        const ROUTER_POINT& previous = points[( index + points.size() - 1 ) % points.size()];
        const ROUTER_POINT& current = points[index];
        const ROUTER_POINT& next = points[( index + 1 ) % points.size()];
        const INTEGER firstX = INTEGER( current.x ) - previous.x;
        const INTEGER firstY = INTEGER( current.y ) - previous.y;
        const INTEGER secondX = INTEGER( next.x ) - current.x;
        const INTEGER secondY = INTEGER( next.y ) - current.y;
        const int cornerWinding = sign( firstX * secondY - firstY * secondX );
        if( cornerWinding == 0 || ( winding != 0 && winding != cornerWinding ) )
            return {};
        winding = cornerWinding;
        centroidX += current.x;
        centroidY += current.y;
    }

    const POINT centroid( centroidX, centroidY, points.size() );
    std::vector<LINE> borders;
    borders.reserve( points.size() );
    for( std::size_t index = 0; index < points.size(); ++index )
    {
        LINE border( points[index], points[( index + 1 ) % points.size()] );
        if( border.SideOf( centroid ) > 0 )
            border = border.Opposite();
        const auto expanded = chebyshevOffset( border, aChebyshevOffset );
        if( !expanded )
            return {};
        borders.push_back( *expanded );
    }

    try
    {
        return SIMPLEX( std::move( borders ) );
    }
    catch( const std::invalid_argument& )
    {
        return {};
    }
}


bool SIMPLEX::Contains( const POINT& p ) const
{ return std::all_of( m_borders.begin(), m_borders.end(), [&]( const auto& l ) { return l.SideOf( p ) <= 0; } ); }
bool SIMPLEX::ContainsInside( const POINT& p ) const
{ return std::all_of( m_borders.begin(), m_borders.end(), [&]( const auto& l ) { return l.SideOf( p ) < 0; } ); }

bool SIMPLEX::IntersectsSegment( const POLYLINE& polyline, std::size_t index ) const
{
    const auto a = polyline.Corner( index - 1 ), b = polyline.Corner( index );
    if( Contains( a ) || Contains( b ) ) return true;
    for( const auto& border : m_borders )
    {
        const auto p = polyline.lines.at( index ).Intersection( border );
        if( p && Contains( *p ) && p->CompareX( a ) * p->CompareX( b ) <= 0
            && p->CompareY( a ) * p->CompareY( b ) <= 0 ) return true;
    }
    return false;
}

std::vector<std::size_t> SIMPLEX::BorderIntersections( const POLYLINE& polyline, std::size_t index ) const
{
    const auto start = polyline.Corner( index - 1 ), end = polyline.Corner( index );
    const auto& middle = polyline.lines.at( index );
    std::vector<std::size_t> result;
    std::vector<POINT> intersections;
    const std::size_t n = m_borders.size();
    for( std::size_t i = 0; i < n; ++i )
    {
        const auto& current = m_borders[i];
        const int startSide = current.SideOf( start ), endSide = current.SideOf( end );
        if( startSide > 0 && endSide > 0 ) return {};
        if( startSide == 0 && endSide != -1 ) return {};
        if( endSide == 0 && startSide != -1 ) return {};
        if( startSide != -1 || endSide != -1 )
        {
            const auto p = middle.Intersection( current );
            if( !p ) return {};
            const int prevSide = m_borders[( i + n - 1 ) % n].SideOf( *p );
            const int nextSide = m_borders[( i + 1 ) % n].SideOf( *p );
            if( prevSide <= 0 && nextSide <= 0 )
            {
                if( prevSide == 0 )
                {
                    int a = middle.SideOf( Corner( ( i + n - 1 ) % n ) );
                    int b = middle.SideOf( Corner( ( i + 1 ) % n ) );
                    if( a == 0 || b == 0 || a == b ) return {};
                }
                if( nextSide == 0 )
                {
                    int a = middle.SideOf( Corner( i ) ), b = middle.SideOf( Corner( ( i + 2 ) % n ) );
                    if( a == 0 || b == 0 || a == b ) return {};
                }
                if( std::find( intersections.begin(), intersections.end(), *p ) == intersections.end() )
                {
                    if( result.size() == 2 ) throw std::logic_error( "more than two convex intersections" );
                    result.push_back( i ); intersections.push_back( *p );
                }
            }
        }
    }
    if( result.size() == 2 && start.DistanceSquared( intersections[1] ) < start.DistanceSquared( intersections[0] ) )
        std::swap( result[0], result[1] );
    return result;
}

std::vector<std::pair<std::size_t, std::size_t>> SIMPLEX::EntrancePoints( const POLYLINE& line ) const
{
    std::vector<std::pair<std::size_t, std::size_t>> result;
    for( std::size_t i = 1; i + 1 < line.lines.size(); ++i )
        for( auto side : BorderIntersections( line, i ) ) result.emplace_back( i, side );
    return result;
}

std::vector<POLYLINE> SIMPLEX::Cutout( const POLYLINE& polyline ) const
{
    if( polyline.Empty() ) return {};
    const auto intersections = EntrancePoints( polyline );
    const bool inside = ContainsInside( polyline.FirstCorner() );
    if( intersections.empty() ) return inside ? std::vector<POLYLINE>{} : std::vector<POLYLINE>{ polyline };
    std::vector<POLYLINE> result;
    const auto append = [&]( std::vector<LINE> lines )
    { POLYLINE piece( std::move( lines ) ); if( !piece.Empty() ) result.push_back( std::move( piece ) ); };
    std::size_t cursor = 0;
    auto [lineIndex, side] = intersections[0];
    const auto first = polyline.lines[lineIndex].Intersection( m_borders[side] ).value();
    if( !inside )
    {
        if( !( polyline.FirstCorner() == first ) )
        {
            std::vector<LINE> lines( polyline.lines.begin(), polyline.lines.begin() + lineIndex + 1 );
            lines.push_back( m_borders[side] ); append( std::move( lines ) );
        }
        ++cursor;
    }
    while( cursor + 1 < intersections.size() )
    {
        const auto [from, firstSide] = intersections[cursor];
        const auto [to, lastSide] = intersections[cursor + 1];
        bool insert = false;
        for( auto i = from + 1; i < to; ++i ) if( !Contains( polyline.Corner( i ) ) ) { insert = true; break; }
        if( insert )
        {
            std::vector<LINE> lines{ m_borders[firstSide] };
            lines.insert( lines.end(), polyline.lines.begin() + from, polyline.lines.begin() + to + 1 );
            lines.push_back( m_borders[lastSide] ); append( std::move( lines ) );
        }
        cursor += 2;
    }
    if( cursor < intersections.size() )
    {
        const auto [from, lastSide] = intersections[cursor];
        std::vector<LINE> lines{ m_borders[lastSide] };
        lines.insert( lines.end(), polyline.lines.begin() + from, polyline.lines.end() );
        append( std::move( lines ) );
    }
    return result;
}
} // namespace KICAD_AUTOROUTER::PLANAR
