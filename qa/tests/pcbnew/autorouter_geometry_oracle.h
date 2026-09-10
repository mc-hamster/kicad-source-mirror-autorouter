/* QA-only differential reader. Expectations come from the pinned Java JAR. */
#pragma once
#include <autorouter/board/optimize/TraceShover.h>
#include <iomanip>
#include <sstream>

namespace AUTOROUTER_GEOMETRY_QA
{
using namespace KICAD_AUTOROUTER;
using namespace KICAD_AUTOROUTER::PLANAR;
inline void Print( std::ostream& out, const POINT& p ) { out << p.x << ' ' << p.y << ' ' << p.z; }
inline void Print( std::ostream& out, const POLYLINE& p )
{
    out << ( p.Empty() ? 0 : p.lines.size() - 1 );
    for( std::size_t i = 0; i + 1 < p.lines.size(); ++i ) { out << ' '; Print( out, p.Corner( i ) ); }
}
inline LINE ReadLine( std::istream& in )
{ ROUTER_POINT a, b; in >> a.x >> a.y >> b.x >> b.y; return { a, b }; }
inline void PrintLine( std::ostream& out, const LINE& line )
{ out << line.a.x << ' ' << line.a.y << ' ' << line.b.x << ' ' << line.b.y; }
inline void PrintSimplex( std::ostream& out, const SIMPLEX& simplex )
{
    out << simplex.Borders().size();
    for( const LINE& border : simplex.Borders() )
    {
        out << ' ';
        PrintLine( out, border );
    }
}
inline std::string CheckRecord( const std::string& record )
{
    std::istringstream in( record );
    std::ostringstream actual;
    std::string tag;
    in >> tag;
    if( tag == "LINE" )
    {
        const auto a = ReadLine( in ), b = ReadLine( in );
        if( auto p = a.Intersection( b ) ) Print( actual, *p );
        else actual << "INF";
    }
    else if( tag == "CONVEX" )
    {
        std::size_t n; in >> n;
        std::vector<LINE> lines;
        for( std::size_t i = 0; i < n; ++i ) lines.push_back( ReadLine( in ) );
        POLYLINE path( std::move( lines ) );
        in >> n;
        std::vector<LINE> borders;
        for( std::size_t i = 0; i < n; ++i ) borders.push_back( ReadLine( in ) );
        SIMPLEX shape( std::move( borders ) );
        Print( actual, path );
        auto entries = shape.EntrancePoints( path );
        actual << ' ' << entries.size();
        for( auto [line, side] : entries ) actual << ' ' << line << ' ' << side;
        auto pieces = shape.Cutout( path ); actual << ' ' << pieces.size();
        for( const auto& piece : pieces ) { actual << ' '; Print( actual, piece ); }
    }
    else if( tag == "SPRING" )
    {
        int radius, clearance, n; in >> radius >> clearance >> n;
        std::vector<TRACE_SHOVER::OBSTACLE> obstacles;
        for( int i = 0; i < n; ++i )
        {
            std::uint64_t id; ROUTER_BOX b;
            in >> id >> b.minX >> b.minY >> b.maxX >> b.maxY;
            const auto inflate = [&]( int r )
            { return SIMPLEX::Box( { b.minX - r, b.minY - r, b.maxX + r, b.maxY + r } ); };
            obstacles.push_back( { id, b, inflate( radius ), inflate( radius + clearance + 1 ) } );
        }
        std::sort( obstacles.begin(), obstacles.end(), []( const auto& a, const auto& b ) { return a.id > b.id; } );
        in >> n; std::vector<ROUTER_POINT> points( n );
        for( auto& p : points ) in >> p.x >> p.y;
        auto result = TRACE_SHOVER::SpringOverObstacles( POLYLINE::FromPoints( points ), obstacles );
        if( result.polyline ) Print( actual, *result.polyline );
        else actual << -1;
    }
    else if( tag == "SIMPLEX" )
    {
        std::int64_t dx, dy;
        std::size_t count, removeIndex;
        in >> dx >> dy >> removeIndex >> count;
        std::vector<LINE> supplied;
        supplied.reserve( count );
        for( std::size_t index = 0; index < count; ++index )
            supplied.push_back( ReadLine( in ) );

        const SIMPLEX simplex = SIMPLEX::GetInstance( std::move( supplied ) );
        PrintSimplex( actual, simplex );
        actual << ' ' << simplex.Dimension()
               << ' ' << ( simplex.IsBounded() ? 1 : 0 )
               << ' ' << ( simplex.IsIntBox() ? 1 : 0 )
               << ' ' << ( simplex.IsIntOctagon() ? 1 : 0 );

        if( !simplex.IsEmpty() && simplex.IsBounded() )
        {
            const auto box = simplex.BoundingBox();
            if( !box )
                throw std::runtime_error( "bounded simplex has no finite box" );
            actual << " 1 " << box->minX << ' ' << box->minY << ' '
                   << box->maxX << ' ' << box->maxY;
            actual << ' ' << simplex.IndexOfRightMostCorner( POINT( dx + 60, dy - 55 ) );
            actual << ' ' << simplex.Borders().size();
            for( std::size_t index = 0; index < simplex.Borders().size(); ++index )
            {
                actual << ' ';
                Print( actual, simplex.Corner( index ) );
            }
        }
        else
        {
            actual << " 0 -1 0";
        }

        for( const ROUTER_POINT point : { ROUTER_POINT{ dx, dy },
                                          ROUTER_POINT{ dx - 30, dy },
                                          ROUTER_POINT{ dx + 30, dy },
                                          ROUTER_POINT{ dx, dy - 30 },
                                          ROUTER_POINT{ dx, dy + 30 } } )
        {
            actual << ' ' << ( simplex.Contains( POINT( point ) ) ? 1 : 0 );
            actual << ' ' << ( simplex.ContainsInside( POINT( point ) ) ? 1 : 0 );
        }

        const SIMPLEX clip = SIMPLEX::Box( { dx - 16, dy - 11, dx + 16, dy + 11 } );
        actual << ' ';
        PrintSimplex( actual, simplex.Intersection( clip ) );
        actual << ' ';
        PrintSimplex( actual, simplex.IsEmpty()
                                      ? simplex
                                      : simplex.RemoveBorderLine( removeIndex ) );
        const auto translated = simplex.TranslateBy( { 7, -11 } );
        if( !translated )
            throw std::runtime_error( "small simplex translation overflowed" );
        actual << ' ';
        PrintSimplex( actual, *translated );
    }
    else if( tag == "SCUT" )
    {
        std::size_t count;
        in >> count;
        std::vector<LINE> outerLines;
        outerLines.reserve( count );
        for( std::size_t index = 0; index < count; ++index )
            outerLines.push_back( ReadLine( in ) );
        in >> count;
        std::vector<LINE> innerLines;
        innerLines.reserve( count );
        for( std::size_t index = 0; index < count; ++index )
            innerLines.push_back( ReadLine( in ) );

        const SIMPLEX outer = SIMPLEX::GetInstance( std::move( outerLines ) );
        const SIMPLEX inner = SIMPLEX::GetInstance( std::move( innerLines ) );
        const auto pieces = inner.CutoutFrom( outer );
        if( !pieces )
        {
            actual << -1;
        }
        else
        {
            actual << pieces->size();
            for( const SIMPLEX& piece : *pieces )
            {
                actual << ' ';
                PrintSimplex( actual, piece );
                actual << ' ' << piece.Dimension();
            }
        }
    }
    else if( tag == "SOFF" )
    {
        std::size_t count;
        in >> count;
        std::vector<LINE> supplied;
        supplied.reserve( count );
        for( std::size_t index = 0; index < count; ++index )
            supplied.push_back( ReadLine( in ) );
        double width, queryX, queryY;
        in >> width >> queryX >> queryY;
        const SIMPLEX simplex = SIMPLEX::GetInstance( std::move( supplied ) );
        const auto offset = simplex.Offset( width );
        const auto enlarged = simplex.Enlarge( width );
        if( !offset || !enlarged )
            throw std::runtime_error( "small simplex offset overflowed" );
        PrintSimplex( actual, *offset );
        actual << ' ';
        PrintSimplex( actual, *enlarged );
        const auto gravity = simplex.CentreOfGravity();
        const auto nearest = simplex.NearestPointApprox( queryX, queryY );
        const auto nearestBorder = simplex.NearestBorderPointApprox( queryX, queryY );
        actual << std::fixed << std::setprecision( 9 )
               << ' ' << gravity.first << ' ' << gravity.second
               << ' ' << nearest.first << ' ' << nearest.second
               << ' ' << nearestBorder.first << ' ' << nearestBorder.second;
    }
    else throw std::runtime_error( "Unexpected oracle record: " + tag );
    if( !in ) throw std::runtime_error( "Malformed oracle input" );
    std::string expected; std::getline( in >> std::ws, expected );
    return expected == actual.str() ? "" : "Expected: " + expected + "\nActual:   " + actual.str();
}
} // namespace AUTOROUTER_GEOMETRY_QA
