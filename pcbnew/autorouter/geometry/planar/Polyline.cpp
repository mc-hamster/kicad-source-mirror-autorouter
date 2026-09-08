/* KiCad, GPL-3.0-or-later. Translated from Freerouting Polyline.java a11c0a42.
 * Normalization uses exact side predicates instead of the Java float fast path.
 */
#include "Polyline.h"

namespace KICAD_AUTOROUTER::PLANAR
{
POLYLINE::POLYLINE( std::vector<LINE> input )
{
    for( const LINE& line : input )
        if( lines.empty() || !lines.back().Parallel( line ) ) lines.push_back( line );
    if( lines.size() >= 4 )
    {
        // removeOverlaps: preserve Java's treatment of the two end caps.
        input = std::move( lines );
        lines.clear();
        if( !input[0].EqualOrOpposite( input[2] ) ) lines.push_back( input[0] );
        lines.push_back( input[1] );
        for( std::size_t i = 2; i + 2 < input.size(); ++i )
        {
            if( !lines.empty() && lines.back().EqualOrOpposite( input[i + 1] ) )
                lines.pop_back();
            else
                lines.push_back( input[i] );
        }
        lines.push_back( input[input.size() - 2] );
        if( lines.size() >= 2 && !input.back().EqualOrOpposite( lines[lines.size() - 2] ) )
            lines.push_back( input.back() );
    }
    if( lines.size() < 3 ) { lines.clear(); return; }
    for( std::size_t i = 1; i + 1 < lines.size(); ++i )
    {
        const auto next = lines[i].Intersection( lines[i + 1] );
        if( !next ) { lines.clear(); return; }
        const int side = lines[i - 1].SideOf( *next );
        const int directionSide = Sign( lines[i - 1].Dx() * lines[i].Dy()
                                        - lines[i - 1].Dy() * lines[i].Dx() );
        if( side != 0 && directionSide != side ) lines[i] = lines[i].Opposite();
    }
}

POLYLINE POLYLINE::FromPoints( const std::vector<ROUTER_POINT>& points )
{
    if( points.size() < 2 ) return POLYLINE( {} );
    std::vector<LINE> result;
    // Integer perpendicular cap without int64 subtraction/addition overflow.
    const auto cap = []( ROUTER_POINT a, ROUTER_POINT b )
    {
        INTEGER dx = INTEGER( b.x ) - a.x, dy = INTEGER( b.y ) - a.y;
        const INTEGER gcd = Gcd( dx, dy );
        if( gcd == 0 ) throw std::invalid_argument( "duplicate polyline endpoint" );
        auto end = POINT( INTEGER( a.x ) - dy / gcd, INTEGER( a.y ) + dx / gcd ).Integral();
        if( !end ) throw std::overflow_error( "polyline cap outside host coordinate domain" );
        return LINE( a, *end );
    };
    std::vector<ROUTER_POINT> filtered;
    for( auto p : points ) if( filtered.empty() || filtered.back() != p ) filtered.push_back( p );
    if( filtered.size() < 2 ) return POLYLINE( {} );
    result.push_back( cap( filtered[0], filtered[1] ) );
    for( std::size_t i = 1; i < filtered.size(); ++i ) result.emplace_back( filtered[i - 1], filtered[i] );
    result.push_back( cap( filtered.back(), filtered[filtered.size() - 2] ) );
    return POLYLINE( std::move( result ) );
}

POLYLINE POLYLINE::Reverse() const
{
    std::vector<LINE> result;
    for( auto it = lines.rbegin(); it != lines.rend(); ++it ) result.push_back( it->Opposite() );
    return POLYLINE( std::move( result ) );
}

POLYLINE POLYLINE::Combine( const POLYLINE& other ) const
{
    if( Empty() || other.Empty() ) return *this;
    bool atStart, otherAtStart;
    if( FirstCorner() == other.FirstCorner() ) { atStart = true; otherAtStart = true; }
    else if( FirstCorner() == other.LastCorner() ) { atStart = true; otherAtStart = false; }
    else if( LastCorner() == other.FirstCorner() ) { atStart = false; otherAtStart = true; }
    else if( LastCorner() == other.LastCorner() ) { atStart = false; otherAtStart = false; }
    else return *this;
    std::vector<LINE> result;
    if( atStart )
    {
        if( otherAtStart )
            for( std::size_t i = other.lines.size() - 1; i > 0; --i ) result.push_back( other.lines[i].Opposite() );
        else result.insert( result.end(), other.lines.begin(), other.lines.end() - 1 );
        result.insert( result.end(), lines.begin() + 1, lines.end() );
    }
    else
    {
        result.insert( result.end(), lines.begin(), lines.end() - 1 );
        if( otherAtStart ) result.insert( result.end(), other.lines.begin() + 1, other.lines.end() );
        else for( std::size_t i = other.lines.size() - 1; i > 0; --i ) result.push_back( other.lines[i - 1].Opposite() );
    }
    return POLYLINE( std::move( result ) );
}

double POLYLINE::LengthApprox() const
{
    double result = 0;
    for( std::size_t i = 1; i + 1 < lines.size(); ++i ) result += std::sqrt( Corner( i - 1 ).DistanceSquared( Corner( i ) ) );
    return result;
}

std::optional<std::vector<ROUTER_POINT>> POLYLINE::IntegralCorners() const
{
    std::vector<ROUTER_POINT> result;
    for( std::size_t i = 0; i + 1 < lines.size(); ++i )
    {
        auto p = Corner( i ).Integral();
        if( !p ) return {};
        if( result.empty() || result.back() != *p ) result.push_back( *p );
    }
    return result;
}
} // namespace KICAD_AUTOROUTER::PLANAR
