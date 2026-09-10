/* QA-only differential reader. Expectations come from the pinned Java JAR. */
#pragma once
#include <autorouter/board/optimize/TraceShover.h>
#include <autorouter/geometry/planar/IntOctagon.h>
#include <autorouter/geometry/planar/LineSegment.h>
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
    else if( tag == "STILE" )
    {
        std::size_t count;
        in >> count;
        std::vector<LINE> supplied;
        for( std::size_t index = 0; index < count; ++index )
            supplied.push_back( ReadLine( in ) );
        in >> count;
        std::vector<LINE> otherSupplied;
        for( std::size_t index = 0; index < count; ++index )
            otherSupplied.push_back( ReadLine( in ) );
        const LINE probe = ReadLine( in );
        ROUTER_POINT segmentStart, segmentEnd;
        double sectionWidth;
        in >> segmentStart.x >> segmentStart.y >> segmentEnd.x >> segmentEnd.y
           >> sectionWidth;

        const SIMPLEX simplex = SIMPLEX::GetInstance( std::move( supplied ) );
        const SIMPLEX other = SIMPLEX::GetInstance( std::move( otherSupplied ) );
        actual << std::fixed << std::setprecision( 9 )
               << simplex.Area() << ' ' << simplex.Circumference() << ' '
               << simplex.MaxWidth() << ' ' << simplex.MinWidth();
        actual << ' ' << simplex.EqualsCorner( simplex.Corner( 0 ) );
        actual << ' ' << simplex.ContainsOnBorderLineNo( simplex.Corner( 0 ) );
        const auto touching = simplex.TouchingSides( other );
        actual << ' ' << touching.size();
        for( int side : touching )
            actual << ' ' << side;
        actual << ' ' << simplex.DistanceToTheLeft( probe );
        actual << ' ' << simplex.SideOf( probe );
        actual << ' ' << ( simplex.IsIntersectedInteriorBy(
                POINT( segmentStart ), POINT( segmentEnd ),
                LINE( segmentStart, segmentEnd ) ) ? 1 : 0 );
        const auto sections = simplex.DivideIntoSections( sectionWidth );
        actual << ' ' << sections.size();
        for( const SIMPLEX& section : sections )
        {
            actual << ' ';
            PrintSimplex( actual, section );
        }
    }
    else if( tag == "POLY" )
    {
        std::size_t count;
        in >> count;
        std::vector<LINE> supplied;
        supplied.reserve( count );
        for( std::size_t index = 0; index < count; ++index )
            supplied.push_back( ReadLine( in ) );
        int width, from, to;
        double queryX, queryY;
        ROUTER_POINT translation;
        std::size_t splitIndex;
        in >> width >> from >> to >> queryX >> queryY
           >> translation.x >> translation.y >> splitIndex;
        const LINE endLine = ReadLine( in );
        in >> count;
        std::vector<LINE> otherSupplied;
        otherSupplied.reserve( count );
        for( std::size_t index = 0; index < count; ++index )
            otherSupplied.push_back( ReadLine( in ) );
        std::array<ROUTER_POINT, 4> probes;
        for( ROUTER_POINT& probe : probes )
            in >> probe.x >> probe.y;

        const POLYLINE polyline( std::move( supplied ) );
        const POLYLINE other( std::move( otherSupplied ) );
        actual << polyline.lines.size();
        for( const LINE& line : polyline.lines )
        {
            actual << ' ';
            PrintLine( actual, line );
        }
        actual << ' ' << polyline.CornerCount();
        for( std::size_t index = 0; index < polyline.CornerCount(); ++index )
        {
            actual << ' ';
            Print( actual, polyline.Corner( index ) );
        }
        actual << ' ' << ( polyline.IsPoint() ? 1 : 0 )
               << ' ' << ( polyline.IsOrthogonal() ? 1 : 0 )
               << ' ' << ( polyline.IsMultipleOf45Degree() ? 1 : 0 );
        actual << std::fixed << std::setprecision( 9 )
               << ' ' << polyline.LengthApprox()
               << ' ' << polyline.LengthApprox( 1,
                        static_cast<int>( polyline.CornerCount() ) - 1 );
        const auto box = polyline.BoundingBox();
        if( !box )
            throw std::runtime_error( "nonempty polyline has no bounds" );
        actual << ' ' << box->minX << ' ' << box->minY << ' '
               << box->maxX << ' ' << box->maxY;
        const auto octagon = polyline.BoundingOctagon();
        if( !octagon )
            throw std::runtime_error( "nonempty polyline has no octagon" );
        actual << ' ' << octagon->leftX << ' ' << octagon->bottomY << ' '
               << octagon->rightX << ' ' << octagon->topY << ' '
               << octagon->upperLeftDiagonalX << ' '
               << octagon->lowerRightDiagonalX << ' '
               << octagon->lowerLeftDiagonalX << ' '
               << octagon->upperRightDiagonalX;
        const auto nearest = polyline.NearestPointApprox( queryX, queryY );
        if( !nearest )
            throw std::runtime_error( "nonempty polyline has no nearest point" );
        actual << ' ' << nearest->first << ' ' << nearest->second;
        for( const ROUTER_POINT& probe : probes )
            actual << ' ' << ( polyline.Contains( POINT( probe ) ) ? 1 : 0 );
        const auto printPolylineLines = [&]( const POLYLINE& value )
        {
            actual << value.lines.size();
            for( const LINE& line : value.lines )
            {
                actual << ' ';
                PrintLine( actual, line );
            }
        };
        actual << ' ';
        printPolylineLines( polyline.Reverse() );
        const auto translated = polyline.TranslateBy( translation );
        if( !translated )
            throw std::runtime_error( "small polyline translation overflowed" );
        actual << ' ';
        printPolylineLines( *translated );
        actual << ' ';
        printPolylineLines( polyline.Combine( other ) );
        const auto split = polyline.Split( splitIndex, endLine );
        if( split.empty() )
        {
            actual << " -1";
        }
        else
        {
            actual << " 2 ";
            printPolylineLines( split[0] );
            actual << ' ';
            printPolylineLines( split[1] );
        }
        const std::size_t skipIndex = std::min(
                std::max<std::size_t>( splitIndex, 1 ),
                polyline.lines.size() - 2 );
        actual << ' ';
        printPolylineLines( polyline.SkipLines( skipIndex, skipIndex ) );
        const auto offsets = polyline.OffsetShapes( width, from, to );
        actual << ' ' << offsets.size();
        for( const SIMPLEX& shape : offsets )
        {
            actual << ' ';
            PrintSimplex( actual, shape );
        }
        const LINE& probeLine = polyline.lines[1];
        const auto translatedLine = probeLine.Translate( width - 3.5 );
        if( !translatedLine )
            throw std::runtime_error( "small line translation overflowed" );
        actual << ' ';
        PrintLine( actual, *translatedLine );
        const auto projection = probeLine.ProjectionApprox( queryX, queryY );
        actual << ' ' << probeLine.SignedDistance( queryX, queryY )
               << ' ' << projection.first << ' ' << projection.second;
        actual << ' ';
        Print( actual, probeLine.PerpendicularProjection( POINT( probes[0] ) ) );
    }
    else if( tag == "SEGMENT" )
    {
        const auto readSegment = [&]()
        {
            const LINE start = ReadLine( in );
            const LINE middle = ReadLine( in );
            const LINE end = ReadLine( in );
            return LINE_SEGMENT( start, middle, end );
        };
        const auto printSegment = [&]( const LINE_SEGMENT& value )
        {
            PrintLine( actual, value.GetStartClosingLine() );
            actual << ' ';
            PrintLine( actual, value.GetLine() );
            actual << ' ';
            PrintLine( actual, value.GetEndClosingLine() );
        };
        const auto printPolylineLines = [&]( const POLYLINE& value )
        {
            actual << value.lines.size();
            for( const LINE& line : value.lines )
            {
                actual << ' ';
                PrintLine( actual, line );
            }
        };
        const auto printPoints = [&]( const std::vector<ROUTER_POINT>& values )
        {
            actual << values.size();
            for( const ROUTER_POINT& point : values )
                actual << ' ' << point.x << ' ' << point.y;
        };

        const LINE_SEGMENT segment = readSegment();
        const LINE_SEGMENT other = readSegment();
        double width, newLength;
        int toTheRight;
        ROUTER_POINT probe;
        std::size_t count;
        in >> width >> toTheRight >> probe.x >> probe.y >> newLength >> count;
        std::vector<LINE> shapeLines;
        shapeLines.reserve( count );
        for( std::size_t index = 0; index < count; ++index )
            shapeLines.push_back( ReadLine( in ) );
        const SIMPLEX shape = SIMPLEX::GetInstance( std::move( shapeLines ) );
        std::size_t shapeLineIndex;
        in >> shapeLineIndex;

        Print( actual, segment.StartPoint() );
        actual << ' ';
        Print( actual, segment.EndPoint() );
        const auto startApprox = segment.StartPointApprox();
        const auto endApprox = segment.EndPointApprox();
        actual << std::fixed << std::setprecision( 9 )
               << ' ' << startApprox.first << ' ' << startApprox.second
               << ' ' << endApprox.first << ' ' << endApprox.second << ' ';
        printSegment( segment.Opposite() );
        actual << ' ';
        printPolylineLines( segment.ToPolyline() );
        actual << ' ';
        PrintSimplex( actual, segment.ToSimplex() );
        actual << ' ' << ( segment.Contains( POINT( probe ) ) ? 1 : 0 );
        const ROUTER_BOX box = segment.BoundingBox();
        actual << ' ' << box.minX << ' ' << box.minY << ' '
               << box.maxX << ' ' << box.maxY;
        const INT_OCTAGON octagon = segment.BoundingOctagon();
        actual << ' ' << octagon.leftX << ' ' << octagon.bottomY << ' '
               << octagon.rightX << ' ' << octagon.topY << ' '
               << octagon.upperLeftDiagonalX << ' '
               << octagon.lowerRightDiagonalX << ' '
               << octagon.lowerLeftDiagonalX << ' '
               << octagon.upperRightDiagonalX;
        const LINE_SEGMENT changed = segment.ChangeLengthApprox( newLength );
        actual << ' ';
        printSegment( changed );
        actual << ' ';
        Print( actual, changed.EndPoint() );
        const std::vector<LINE> intersections = segment.Intersection( other );
        actual << ' ' << intersections.size();
        for( const LINE& line : intersections )
        {
            actual << ' ';
            PrintLine( actual, line );
        }
        actual << ' ' << ( segment.Intersects( other ) ? 1 : 0 )
               << ' ' << ( segment.Overlaps( other ) ? 1 : 0 ) << ' ';
        printPoints( segment.StairApproximation( width, toTheRight != 0 ) );
        actual << ' ';
        printPoints( segment.StairApproximation45( width, toTheRight != 0 ) );
        const std::vector<int> borders = segment.BorderIntersections( shape );
        actual << ' ' << borders.size();
        for( int border : borders )
            actual << ' ' << border;
        actual << ' ';
        printSegment( segment.SortEndpointsInXY() );
        const auto shapeSegment = LINE_SEGMENT::FromShape( shape, shapeLineIndex );
        if( !shapeSegment )
            throw std::runtime_error( "valid shape segment was rejected" );
        actual << ' ';
        printSegment( *shapeSegment );
    }
    else throw std::runtime_error( "Unexpected oracle record: " + tag );
    if( !in ) throw std::runtime_error( "Malformed oracle input" );
    std::string expected; std::getline( in >> std::ws, expected );
    return expected == actual.str() ? "" : "Expected: " + expected + "\nActual:   " + actual.str();
}
} // namespace AUTOROUTER_GEOMETRY_QA
