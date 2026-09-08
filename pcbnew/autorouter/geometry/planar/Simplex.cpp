/* KiCad, GPL-3.0-or-later. Translated LineSegment.borderIntersections and
 * TileShape.entrancePoints/cutout(Polyline), Freerouting a11c0a42.
 */
#include "Simplex.h"

namespace KICAD_AUTOROUTER::PLANAR
{
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
