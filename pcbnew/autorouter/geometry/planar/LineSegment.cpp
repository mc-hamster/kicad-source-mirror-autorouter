/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct C++ translation of Freerouting geometry/planar/LineSegment.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#include "LineSegment.h"

#include "IntBox.h"

#include <algorithm>
#include <limits>

namespace KICAD_AUTOROUTER::PLANAR
{
namespace
{

std::int64_t javaRound( double aValue )
{
    const double rounded = std::floor( aValue + 0.5 );
    if( rounded < static_cast<double>( std::numeric_limits<std::int64_t>::min() )
        || rounded > static_cast<double>( std::numeric_limits<std::int64_t>::max() ) )
    {
        throw std::overflow_error( "rounded segment point outside host coordinate domain" );
    }
    return static_cast<std::int64_t>( rounded );
}


LINE directionLine( ROUTER_POINT aPoint, INTEGER aDx, INTEGER aDy )
{
    const INTEGER divisor = Gcd( aDx, aDy );
    if( divisor == 0 )
        throw std::invalid_argument( "zero segment direction" );
    aDx /= divisor;
    aDy /= divisor;
    const INTEGER endX = INTEGER( aPoint.x ) + aDx;
    const INTEGER endY = INTEGER( aPoint.y ) + aDy;
    if( endX < std::numeric_limits<std::int64_t>::min()
        || endX > std::numeric_limits<std::int64_t>::max()
        || endY < std::numeric_limits<std::int64_t>::min()
        || endY > std::numeric_limits<std::int64_t>::max() )
    {
        throw std::overflow_error( "segment support outside host coordinate domain" );
    }
    return LINE( aPoint, { endX.convert_to<std::int64_t>(),
                           endY.convert_to<std::int64_t>() } );
}


ROUTER_POINT roundedPoint( const std::pair<double, double>& aPoint )
{
    return { javaRound( aPoint.first ), javaRound( aPoint.second ) };
}


int compareXY( const POINT& aLeft, const POINT& aRight )
{
    const int compareX = aLeft.CompareX( aRight );
    return compareX != 0 ? compareX : aLeft.CompareY( aRight );
}


double functionValueApprox( const LINE& aLine, double aX )
{
    const double dx = aLine.Dx().convert_to<double>();
    if( dx == 0 )
        return 0;
    const double dy = aLine.Dy().convert_to<double>();
    const double determinant = static_cast<double>( aLine.a.x ) * aLine.b.y
                               - static_cast<double>( aLine.b.x ) * aLine.a.y;
    return ( dy * aX - determinant ) / dx;
}


double functionInYValueApprox( const LINE& aLine, double aY )
{
    const double dy = aLine.Dy().convert_to<double>();
    if( dy == 0 )
        return 0;
    const double dx = aLine.Dx().convert_to<double>();
    const double determinant = static_cast<double>( aLine.a.x ) * aLine.b.y
                               - static_cast<double>( aLine.b.x ) * aLine.a.y;
    return ( dx * aY + determinant ) / dy;
}

} // namespace


LINE_SEGMENT::LINE_SEGMENT( LINE aStartLine, LINE aMiddleLine, LINE aEndLine ) :
        m_start( std::move( aStartLine ) ),
        m_middle( std::move( aMiddleLine ) ),
        m_end( std::move( aEndLine ) )
{
    if( m_start.Parallel( m_middle ) || m_middle.Parallel( m_end ) )
        throw std::invalid_argument( "segment closing line is parallel" );
}


std::optional<LINE_SEGMENT> LINE_SEGMENT::FromPolyline(
        const POLYLINE& aPolyline, std::size_t aLineIndex )
{
    if( aLineIndex == 0 || aLineIndex + 1 >= aPolyline.lines.size() )
        return {};
    return LINE_SEGMENT( aPolyline.lines[aLineIndex - 1],
                         aPolyline.lines[aLineIndex],
                         aPolyline.lines[aLineIndex + 1] );
}


std::optional<LINE_SEGMENT> LINE_SEGMENT::FromShape(
        const SIMPLEX& aShape, std::size_t aLineIndex )
{
    const std::vector<LINE>& borders = aShape.Borders();
    if( aLineIndex >= borders.size() || borders.empty() )
        return {};
    const std::size_t previous = aLineIndex == 0
                                 ? borders.size() - 1 : aLineIndex - 1;
    const std::size_t next = aLineIndex + 1 == borders.size()
                             ? 0 : aLineIndex + 1;
    return LINE_SEGMENT( borders[previous], borders[aLineIndex], borders[next] );
}


POINT LINE_SEGMENT::StartPoint() const
{
    return m_middle.Intersection( m_start ).value();
}


POINT LINE_SEGMENT::EndPoint() const
{
    return m_middle.Intersection( m_end ).value();
}


std::pair<double, double> LINE_SEGMENT::StartPointApprox() const
{
    const POINT point = StartPoint();
    return { point.X(), point.Y() };
}


std::pair<double, double> LINE_SEGMENT::EndPointApprox() const
{
    const POINT point = EndPoint();
    return { point.X(), point.Y() };
}


LINE_SEGMENT LINE_SEGMENT::Opposite() const
{
    return LINE_SEGMENT( m_end.Opposite(), m_middle.Opposite(),
                         m_start.Opposite() );
}


POLYLINE LINE_SEGMENT::ToPolyline() const
{
    return POLYLINE( { m_start, m_middle, m_end } );
}


SIMPLEX LINE_SEGMENT::ToSimplex() const
{
    std::vector<LINE> lines;
    lines.reserve( 4 );
    lines.push_back( m_start.SideOf( EndPoint() ) > 0
                     ? m_start.Opposite() : m_start );
    lines.push_back( m_middle );
    lines.push_back( m_middle.Opposite() );
    lines.push_back( m_end.SideOf( StartPoint() ) > 0
                     ? m_end.Opposite() : m_end );
    return SIMPLEX::GetInstance( std::move( lines ) );
}


bool LINE_SEGMENT::Contains( const POINT& aPoint ) const
{
    if( !aPoint.Integral() || m_middle.SideOf( aPoint ) != 0 )
        return false;
    const POINT start = StartPoint();
    const POINT end = EndPoint();
    const bool betweenX = ( aPoint.CompareX( start ) >= 0
                            && aPoint.CompareX( end ) <= 0 )
                          || ( aPoint.CompareX( end ) >= 0
                               && aPoint.CompareX( start ) <= 0 );
    const bool betweenY = ( aPoint.CompareY( start ) >= 0
                            && aPoint.CompareY( end ) <= 0 )
                          || ( aPoint.CompareY( end ) >= 0
                               && aPoint.CompareY( start ) <= 0 );
    return betweenX && betweenY;
}


ROUTER_BOX LINE_SEGMENT::BoundingBox() const
{
    const auto start = StartPointApprox();
    const auto end = EndPointApprox();
    return { static_cast<std::int64_t>( std::floor( std::min( start.first, end.first ) ) ),
             static_cast<std::int64_t>( std::floor( std::min( start.second, end.second ) ) ),
             static_cast<std::int64_t>( std::ceil( std::max( start.first, end.first ) ) ),
             static_cast<std::int64_t>( std::ceil( std::max( start.second, end.second ) ) ) };
}


INT_OCTAGON LINE_SEGMENT::BoundingOctagon() const
{
    const auto start = StartPointApprox();
    const auto end = EndPointApprox();
    const auto floorMinimum = []( double aFirst, double aSecond )
    { return static_cast<std::int64_t>( std::floor( std::min( aFirst, aSecond ) ) ); };
    const auto ceilMaximum = []( double aFirst, double aSecond )
    { return static_cast<std::int64_t>( std::ceil( std::max( aFirst, aSecond ) ) ); };
    return INT_OCTAGON(
            floorMinimum( start.first, end.first ),
            floorMinimum( start.second, end.second ),
            ceilMaximum( start.first, end.first ),
            ceilMaximum( start.second, end.second ),
            floorMinimum( start.first - start.second, end.first - end.second ),
            ceilMaximum( start.first - start.second, end.first - end.second ),
            floorMinimum( start.first + start.second, end.first + end.second ),
            ceilMaximum( start.first + start.second, end.first + end.second ) ).Normalize();
}


LINE_SEGMENT LINE_SEGMENT::ChangeLengthApprox( double aNewLength ) const
{
    const auto start = StartPointApprox();
    const auto end = EndPointApprox();
    const double dx = end.first - start.first;
    const double dy = end.second - start.second;
    const double length = std::hypot( dx, dy );
    const auto newEnd = length == 0
                        ? end
                        : std::pair<double, double>{ start.first + dx * aNewLength / length,
                                                     start.second + dy * aNewLength / length };
    const ROUTER_POINT roundedEnd = roundedPoint( newEnd );
    const LINE closing = directionLine( roundedEnd, -m_middle.Dy(), m_middle.Dx() );
    return LINE_SEGMENT( m_start, m_middle, closing );
}


std::vector<LINE> LINE_SEGMENT::Intersection( const LINE_SEGMENT& aOther ) const
{
    if( !INT_BOX::Intersects( BoundingBox(), aOther.BoundingBox() ) )
        return {};
    const int startSide = aOther.m_middle.SideOf( StartPoint() );
    const int endSide = aOther.m_middle.SideOf( EndPoint() );
    if( startSide == 0 && endSide == 0 )
    {
        const LINE_SEGMENT thisSorted = SortEndpointsInXY();
        const LINE_SEGMENT otherSorted = aOther.SortEndpointsInXY();
        const LINE_SEGMENT* left = &thisSorted;
        const LINE_SEGMENT* right = &otherSorted;
        if( compareXY( thisSorted.StartPoint(), otherSorted.StartPoint() ) > 0 )
            std::swap( left, right );
        const int comparison = compareXY( left->EndPoint(), right->StartPoint() );
        if( comparison < 0 )
            return {};
        if( comparison == 0 )
            return { left->m_end };
        if( compareXY( right->EndPoint(), left->EndPoint() ) >= 0 )
            return { right->m_start, left->m_end };
        return { right->m_start, right->m_end };
    }
    if( startSide == endSide
        || m_middle.SideOf( aOther.StartPoint() )
           == m_middle.SideOf( aOther.EndPoint() ) )
    {
        return {};
    }
    return { aOther.m_middle };
}


bool LINE_SEGMENT::Intersects( const LINE_SEGMENT& aOther ) const
{
    return !Intersection( aOther ).empty();
}


bool LINE_SEGMENT::Overlaps( const LINE_SEGMENT& aOther ) const
{
    return Intersection( aOther ).size() > 1;
}


std::vector<ROUTER_POINT> LINE_SEGMENT::StairApproximation(
        double aWidth, bool aToTheRight ) const
{
    const ROUTER_POINT start = roundedPoint( StartPointApprox() );
    const ROUTER_POINT end = roundedPoint( EndPointApprox() );
    if( start == end )
        return {};
    if( start.x == end.x || start.y == end.y )
        return { start, end };

    const std::int64_t dx = end.x - start.x;
    const std::int64_t dy = end.y - start.y;
    const std::int64_t absDx = std::abs( dx );
    const std::int64_t absDy = std::abs( dy );
    const bool functionOfX = absDx >= absDy;
    std::int64_t stairWidth;
    std::int64_t stairCount;
    if( functionOfX )
    {
        stairWidth = javaRound( aWidth * absDx / absDy );
        if( stairWidth == 0 )
            throw std::invalid_argument( "zero orthogonal stair width" );
        stairCount = ( absDx - 1 ) / stairWidth + 1;
        if( end.x < start.x )
            stairWidth = -stairWidth;
    }
    else
    {
        stairWidth = javaRound( aWidth * absDy / absDx );
        if( stairWidth == 0 )
            throw std::invalid_argument( "zero orthogonal stair width" );
        stairCount = ( absDy - 1 ) / stairWidth + 1;
        if( end.y < start.y )
            stairWidth = -stairWidth;
    }

    std::vector<ROUTER_POINT> result;
    result.reserve( 2 * stairCount + 1 );
    result.push_back( start );
    const double determinant = static_cast<double>( dx ) * dy;
    const bool changeXFirst = ( aToTheRight && determinant > 0 )
                              || ( !aToTheRight && determinant < 0 );
    std::int64_t previousX = start.x;
    std::int64_t previousY = start.y;
    for( std::int64_t index = 1; index < stairCount; ++index )
    {
        std::int64_t currentX;
        std::int64_t currentY;
        if( functionOfX )
        {
            currentX = start.x + index * stairWidth;
            currentY = javaRound( functionValueApprox( m_middle, currentX ) );
        }
        else
        {
            currentY = start.y + index * stairWidth;
            currentX = javaRound( functionInYValueApprox( m_middle, currentY ) );
        }
        result.push_back( changeXFirst ? ROUTER_POINT{ currentX, previousY }
                                       : ROUTER_POINT{ previousX, currentY } );
        result.push_back( { currentX, currentY } );
        previousX = currentX;
        previousY = currentY;
    }
    result.push_back( changeXFirst ? ROUTER_POINT{ end.x, previousY }
                                   : ROUTER_POINT{ previousX, end.y } );
    result.push_back( end );
    return result;
}


std::vector<ROUTER_POINT> LINE_SEGMENT::StairApproximation45(
        double aWidth, bool aToTheRight ) const
{
    const ROUTER_POINT start = roundedPoint( StartPointApprox() );
    const ROUTER_POINT end = roundedPoint( EndPointApprox() );
    if( start == end )
        return {};
    const std::int64_t dx = end.x - start.x;
    const std::int64_t dy = end.y - start.y;
    const std::int64_t absDx = std::abs( dx );
    const std::int64_t absDy = std::abs( dy );
    if( dx == 0 || dy == 0 || absDx == absDy )
        return { start, end };
    const bool functionOfX = absDx >= absDy;
    const double determinant = static_cast<double>( dx ) * dy;
    std::int64_t stairWidth;
    std::int64_t stairCount;
    if( functionOfX )
    {
        stairWidth = javaRound( aWidth * absDx / absDy );
        if( stairWidth == 0 )
            throw std::invalid_argument( "zero 45-degree stair width" );
        stairCount = ( absDx - 1 ) / stairWidth + 1;
        if( end.x < start.x )
            stairWidth = -stairWidth;
    }
    else
    {
        stairWidth = javaRound( aWidth * absDy / absDx );
        if( stairWidth == 0 )
            throw std::invalid_argument( "zero 45-degree stair width" );
        stairCount = ( absDy - 1 ) / stairWidth + 1;
        if( end.y < start.y )
            stairWidth = -stairWidth;
    }

    std::vector<ROUTER_POINT> result;
    result.reserve( 2 * stairCount + 1 );
    result.push_back( start );
    ROUTER_POINT previous = start;
    for( std::int64_t index = 1; index <= stairCount; ++index )
    {
        ROUTER_POINT current;
        if( index == stairCount )
        {
            current = end;
        }
        else if( functionOfX )
        {
            current.x = start.x + index * stairWidth;
            current.y = javaRound( functionValueApprox( m_middle, current.x ) );
        }
        else
        {
            current.y = start.y + index * stairWidth;
            // Preserve the pinned source's functionValueApprox call in this
            // branch (rather than silently correcting it to functionInY).
            current.x = javaRound( functionValueApprox( m_middle, current.y ) );
        }

        ROUTER_POINT intermediate;
        const std::int64_t stairSign = stairWidth > 0 ? 1 : -1;
        if( functionOfX )
        {
            const bool diagonalFirst = ( aToTheRight && determinant < 0 )
                                       || ( !aToTheRight && determinant > 0 );
            if( diagonalFirst )
            {
                intermediate.x = previous.x
                                 + stairSign * std::abs( current.y - previous.y );
                intermediate.y = current.y;
            }
            else
            {
                intermediate.x = current.x
                                 - stairSign * std::abs( current.y - previous.y );
                intermediate.y = previous.y;
            }
        }
        else
        {
            const bool diagonalFirst = ( aToTheRight && determinant > 0 )
                                       || ( !aToTheRight && determinant < 0 );
            if( diagonalFirst )
            {
                intermediate.x = current.x;
                intermediate.y = previous.y
                                 + stairSign * std::abs( current.x - previous.x );
            }
            else
            {
                intermediate.x = previous.x;
                intermediate.y = current.y
                                 - stairSign * std::abs( current.x - previous.x );
            }
        }
        result.push_back( intermediate );
        result.push_back( current );
        previous = current;
    }
    return result;
}


std::vector<int> LINE_SEGMENT::BorderIntersections( const SIMPLEX& aShape ) const
{
    const auto shapeBox = aShape.BoundingBox();
    if( !shapeBox || !INT_BOX::Intersects( BoundingBox(), *shapeBox ) )
        return {};
    const std::size_t edgeCount = aShape.Borders().size();
    if( edgeCount == 0 )
        return {};
    LINE previousLine = aShape.Borders().back();
    LINE currentLine = aShape.Borders().front();
    std::vector<int> result;
    std::vector<POINT> intersections;
    const POINT lineStart = StartPoint();
    const POINT lineEnd = EndPoint();

    for( std::size_t edge = 0; edge < edgeCount; ++edge )
    {
        const LINE nextLine = edge + 1 == edgeCount
                              ? aShape.Borders().front()
                              : aShape.Borders()[edge + 1];
        const int startSide = currentLine.SideOf( lineStart );
        const int endSide = currentLine.SideOf( lineEnd );
        if( startSide > 0 && endSide > 0 )
            return {};
        if( startSide == 0 && endSide >= 0 )
            return {};
        if( endSide == 0 && startSide >= 0 )
            return {};

        if( startSide >= 0 || endSide >= 0 )
        {
            const auto intersection = m_middle.Intersection( currentLine );
            if( !intersection )
                return {};
            const int previousSide = previousLine.SideOf( *intersection );
            const int nextSide = nextLine.SideOf( *intersection );
            if( previousSide <= 0 && nextSide <= 0 )
            {
                if( previousSide == 0 )
                {
                    const POINT previousPrevious = edge == 0
                                                   ? aShape.Corner( edgeCount - 1 )
                                                   : aShape.Corner( edge - 1 );
                    const POINT nextCorner = edge + 1 == edgeCount
                                             ? aShape.Corner( 0 )
                                             : aShape.Corner( edge + 1 );
                    const int previousPreviousSide = m_middle.SideOf( previousPrevious );
                    const int nextCornerSide = m_middle.SideOf( nextCorner );
                    if( previousPreviousSide == 0 || nextCornerSide == 0
                        || previousPreviousSide == nextCornerSide )
                    {
                        return {};
                    }
                }
                if( nextSide == 0 )
                {
                    const POINT previousCorner = aShape.Corner( edge );
                    const POINT nextNextCorner = edge == edgeCount - 2
                                                 ? aShape.Corner( 0 )
                                                 : edge == edgeCount - 1
                                                   ? aShape.Corner( 1 )
                                                   : aShape.Corner( edge + 2 );
                    const int previousCornerSide = m_middle.SideOf( previousCorner );
                    const int nextNextSide = m_middle.SideOf( nextNextCorner );
                    if( previousCornerSide == 0 || nextNextSide == 0
                        || previousCornerSide == nextNextSide )
                    {
                        return {};
                    }
                }
                if( std::find( intersections.begin(), intersections.end(), *intersection )
                    == intersections.end() && result.size() < 2 )
                {
                    result.push_back( static_cast<int>( edge ) );
                    intersections.push_back( *intersection );
                }
            }
        }
        previousLine = currentLine;
        currentLine = nextLine;
    }

    if( result.size() == 2
        && lineStart.DistanceSquared( intersections[1] )
           < lineStart.DistanceSquared( intersections[0] ) )
    {
        std::swap( result[0], result[1] );
    }
    return result;
}


LINE_SEGMENT LINE_SEGMENT::SortEndpointsInXY() const
{
    if( compareXY( StartPoint(), EndPoint() ) > 0 )
        return LINE_SEGMENT( m_end, m_middle, m_start );
    return *this;
}

} // namespace KICAD_AUTOROUTER::PLANAR
