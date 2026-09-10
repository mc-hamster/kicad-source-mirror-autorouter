/* KiCad, GPL-3.0-or-later. Translated from Freerouting Polyline.java a11c0a42.
 * Normalization uses exact side predicates instead of the Java float fast path.
 */
#include "Polyline.h"
#include "IntOctagon.h"
#include "Simplex.h"

#include <algorithm>
#include <limits>

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
        // Direction.sideOf asks on which side the receiver lies relative to
        // the argument direction.  Its determinant therefore has the
        // argument first (current x previous), unlike the ordinary turn
        // determinant used by offsetShapes below.
        const int directionSide = Sign( lines[i].Dx() * lines[i - 1].Dy()
                                        - lines[i].Dy() * lines[i - 1].Dx() );
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


bool POLYLINE::IsPoint() const
{
    if( Empty() )
        return true;
    const POINT first = FirstCorner();
    for( std::size_t index = 1; index < CornerCount(); ++index )
    {
        if( !( Corner( index ) == first ) )
            return false;
    }
    return true;
}


bool POLYLINE::IsOrthogonal() const
{
    return std::all_of( lines.begin(), lines.end(),
                        []( const LINE& aLine ) { return aLine.IsOrthogonal(); } );
}


bool POLYLINE::IsMultipleOf45Degree() const
{
    return std::all_of( lines.begin(), lines.end(),
                        []( const LINE& aLine )
                        { return aLine.IsMultipleOf45Degree(); } );
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


std::vector<POLYLINE> POLYLINE::Split( std::size_t aLineIndex,
                                       const LINE& aEndLine ) const
{
    if( aLineIndex < 1 || aLineIndex + 1 >= lines.size()
        || lines[aLineIndex].Parallel( aEndLine ) )
    {
        return {};
    }
    const auto newEndCorner = lines[aLineIndex].Intersection( aEndLine );
    if( !newEndCorner )
        return {};
    if( ( aLineIndex == 1 && *newEndCorner == FirstCorner() )
        || ( aLineIndex >= lines.size() - 2 && *newEndCorner == LastCorner() ) )
    {
        return {};
    }

    std::vector<LINE> first;
    if( Corner( aLineIndex - 1 ) == *newEndCorner )
    {
        first.insert( first.end(), lines.begin(), lines.begin() + aLineIndex + 1 );
    }
    else
    {
        first.insert( first.end(), lines.begin(), lines.begin() + aLineIndex + 1 );
        first.push_back( aEndLine );
    }

    std::vector<LINE> second;
    if( Corner( aLineIndex ) == *newEndCorner )
    {
        second.insert( second.end(), lines.begin() + aLineIndex, lines.end() );
    }
    else
    {
        second.push_back( aEndLine );
        second.insert( second.end(), lines.begin() + aLineIndex, lines.end() );
    }

    POLYLINE firstPiece( std::move( first ) );
    POLYLINE secondPiece( std::move( second ) );
    if( firstPiece.IsPoint() || secondPiece.IsPoint() )
        return {};
    std::vector<POLYLINE> result;
    result.push_back( std::move( firstPiece ) );
    result.push_back( std::move( secondPiece ) );
    return result;
}


POLYLINE POLYLINE::SkipLines( std::size_t aFrom, std::size_t aTo ) const
{
    if( aFrom > aTo || aTo >= lines.size() )
        return *this;
    std::vector<LINE> result;
    result.insert( result.end(), lines.begin(), lines.begin() + aFrom );
    result.insert( result.end(), lines.begin() + aTo + 1, lines.end() );
    return POLYLINE( std::move( result ) );
}


std::optional<POLYLINE> POLYLINE::TranslateBy( ROUTER_POINT aVector ) const
{
    if( aVector.x == 0 && aVector.y == 0 )
        return *this;
    std::vector<LINE> result;
    result.reserve( lines.size() );
    for( const LINE& line : lines )
    {
        const auto translated = line.TranslateBy( aVector );
        if( !translated )
            return {};
        result.push_back( *translated );
    }
    return POLYLINE( std::move( result ) );
}


double POLYLINE::LengthApprox( int aRequestedFromCorner,
                               int aRequestedToCorner ) const
{
    const int from = std::max( aRequestedFromCorner, 0 );
    const int to = std::min( aRequestedToCorner,
                             static_cast<int>( lines.size() ) - 2 );
    double result = 0;
    for( int index = from; index < to; ++index )
        result += std::sqrt( Corner( index + 1 ).DistanceSquared( Corner( index ) ) );
    return result;
}


double POLYLINE::LengthApprox() const
{
    return LengthApprox( 0, static_cast<int>( lines.size() ) - 2 );
}


std::optional<ROUTER_BOX> POLYLINE::BoundingBox(
        int aRequestedFromCorner, int aRequestedToCorner ) const
{
    if( Empty() )
        return {};
    const int from = std::max( aRequestedFromCorner, 0 );
    const int requestedTo = aRequestedToCorner < 0
                            ? static_cast<int>( CornerCount() ) - 1
                            : aRequestedToCorner;
    const int to = std::min( requestedTo, static_cast<int>( lines.size() ) - 2 );
    if( from > to )
        return {};
    std::optional<ROUTER_BOX> result;
    for( int index = from; index <= to; ++index )
    {
        const auto cornerBounds = Corner( index ).SurroundingBox();
        if( !cornerBounds )
            return {};
        if( !result )
            result = cornerBounds;
        else
        {
            result->minX = std::min( result->minX, cornerBounds->minX );
            result->minY = std::min( result->minY, cornerBounds->minY );
            result->maxX = std::max( result->maxX, cornerBounds->maxX );
            result->maxY = std::max( result->maxY, cornerBounds->maxY );
        }
    }
    return result;
}


std::optional<INT_OCTAGON> POLYLINE::BoundingOctagon(
        int aRequestedFromCorner, int aRequestedToCorner ) const
{
    if( Empty() )
        return {};
    const int from = std::max( aRequestedFromCorner, 0 );
    const int requestedTo = aRequestedToCorner < 0
                            ? static_cast<int>( CornerCount() ) - 1
                            : aRequestedToCorner;
    const int to = std::min( requestedTo, static_cast<int>( lines.size() ) - 2 );
    if( from > to )
        return {};

    double left = std::numeric_limits<std::int32_t>::max();
    double bottom = left;
    double right = std::numeric_limits<std::int32_t>::min();
    double top = right;
    double upperLeft = left, lowerRight = right;
    double lowerLeft = left, upperRight = right;
    for( int index = from; index <= to; ++index )
    {
        const double x = Corner( index ).X();
        const double y = Corner( index ).Y();
        left = std::min( left, x );
        bottom = std::min( bottom, y );
        right = std::max( right, x );
        top = std::max( top, y );
        upperLeft = std::min( upperLeft, x - y );
        lowerRight = std::max( lowerRight, x - y );
        lowerLeft = std::min( lowerLeft, x + y );
        upperRight = std::max( upperRight, x + y );
    }
    return INT_OCTAGON( static_cast<std::int64_t>( std::floor( left ) ),
                        static_cast<std::int64_t>( std::floor( bottom ) ),
                        static_cast<std::int64_t>( std::ceil( right ) ),
                        static_cast<std::int64_t>( std::ceil( top ) ),
                        static_cast<std::int64_t>( std::floor( upperLeft ) ),
                        static_cast<std::int64_t>( std::ceil( lowerRight ) ),
                        static_cast<std::int64_t>( std::floor( lowerLeft ) ),
                        static_cast<std::int64_t>( std::ceil( upperRight ) ) );
}


std::optional<std::pair<double, double>> POLYLINE::NearestPointApprox(
        double aX, double aY ) const
{
    if( Empty() )
        return {};
    double minimum = std::numeric_limits<double>::max();
    std::pair<double, double> nearest{};
    for( std::size_t index = 0; index < CornerCount(); ++index )
    {
        const double x = Corner( index ).X();
        const double y = Corner( index ).Y();
        const double distance = std::hypot( x - aX, y - aY );
        if( distance < minimum )
        {
            minimum = distance;
            nearest = { x, y };
        }
    }
    constexpr double tolerance = 1;
    for( std::size_t index = 1; index + 1 < lines.size(); ++index )
    {
        const auto projection = lines[index].ProjectionApprox( aX, aY );
        const double distance = std::hypot( projection.first - aX,
                                            projection.second - aY );
        if( distance >= minimum )
            continue;
        const double firstX = Corner( index - 1 ).X();
        const double firstY = Corner( index - 1 ).Y();
        const double secondX = Corner( index ).X();
        const double secondY = Corner( index ).Y();
        const double segmentLength = std::hypot( secondX - firstX,
                                                 secondY - firstY );
        if( std::hypot( projection.first - firstX, projection.second - firstY )
            + std::hypot( projection.first - secondX, projection.second - secondY )
            < segmentLength + tolerance )
        {
            minimum = distance;
            nearest = projection;
        }
    }
    return nearest;
}


bool POLYLINE::Contains( const POINT& aPoint ) const
{
    if( !aPoint.Integral() )
        return false;
    for( std::size_t index = 1; index + 1 < lines.size(); ++index )
    {
        if( lines[index].SideOf( aPoint ) != 0 )
            continue;
        const POINT start = Corner( index - 1 );
        const POINT end = Corner( index );
        const bool betweenX = ( aPoint.CompareX( start ) >= 0
                                && aPoint.CompareX( end ) <= 0 )
                              || ( aPoint.CompareX( end ) >= 0
                                   && aPoint.CompareX( start ) <= 0 );
        const bool betweenY = ( aPoint.CompareY( start ) >= 0
                                && aPoint.CompareY( end ) <= 0 )
                              || ( aPoint.CompareY( end ) >= 0
                                   && aPoint.CompareY( start ) <= 0 );
        if( betweenX && betweenY )
            return true;
    }
    return false;
}


std::vector<SIMPLEX> POLYLINE::OffsetShapes(
        int aHalfWidth, int aRequestedFromLine, int aRequestedToLine ) const
{
    const int from = std::max( aRequestedFromLine, 0 );
    const int requestedTo = aRequestedToLine < 0
                            ? static_cast<int>( lines.size() ) - 1
                            : aRequestedToLine;
    const int to = std::min( requestedTo, static_cast<int>( lines.size() ) - 1 );
    const int shapeCount = std::max( to - from - 1, 0 );
    std::vector<SIMPLEX> result;
    result.reserve( shapeCount );
    if( shapeCount == 0 )
        return result;

    auto turnSide = []( const LINE& aPrevious, const LINE& aNext )
    { return Sign( aPrevious.DirectionDeterminant( aNext ) ); };
    auto approximate = []( const POINT& aPoint )
    { return std::pair<double, double>{ aPoint.X(), aPoint.Y() }; };
    auto distanceSquared = []( const std::pair<double, double>& a,
                               const std::pair<double, double>& b )
    { const double dx = a.first - b.first, dy = a.second - b.second; return dx * dx + dy * dy; };
    auto sideApprox = []( const LINE& aLine, const std::pair<double, double>& aPoint )
    {
        const double determinant = aLine.Dy().convert_to<double>() * ( aPoint.first - aLine.a.x )
                                   - aLine.Dx().convert_to<double>() * ( aPoint.second - aLine.a.y );
        return determinant > 0 ? 1 : determinant < 0 ? -1 : 0;
    };

    LINE previousDirection = lines[from];
    LINE currentDirection = lines[from + 1];
    for( int index = from + 1; index < to; ++index )
    {
        const LINE nextDirection = lines[index + 1];
        std::vector<LINE> offsetLines;
        const auto right = lines[index].Translate( -aHalfWidth );
        if( !right )
            return {};
        offsetLines.push_back( *right );

        const int nextTurn = turnSide( currentDirection, nextDirection );
        const auto front = ( nextTurn > 0 ? lines[index + 1]
                                          : lines[index + 1].Opposite() ).Translate( -aHalfWidth );
        const auto left = lines[index].Opposite().Translate( -aHalfWidth );
        if( !front || !left )
            return {};
        offsetLines.push_back( *front );
        offsetLines.push_back( *left );

        const int previousTurn = turnSide( previousDirection, currentDirection );
        const auto back = ( previousTurn > 0 ? lines[index - 1]
                                             : lines[index - 1].Opposite() ).Translate( -aHalfWidth );
        if( !back )
            return {};
        offsetLines.push_back( *back );

        std::vector<LINE> dogEarLines;
        LINE currentLine = offsetLines[1];
        LINE checkLine = nextTurn > 0 ? offsetLines[2] : offsetLines[0];
        const auto checkCorner = approximate( Corner( index ) );
        const double checkDistance = 2.0 * aHalfWidth * aHalfWidth;
        LINE temporaryCurrentDirection = nextDirection;
        bool directionChanged = false;
        std::optional<POINT> cornerToCheck;
        for( int nextIndex = index + 2;
             nextIndex < static_cast<int>( lines.size() ) - 1; ++nextIndex )
        {
            if( distanceSquared( approximate( Corner( nextIndex - 1 ) ), checkCorner )
                > checkDistance )
            {
                break;
            }
            if( !directionChanged )
                cornerToCheck = currentLine.Intersection( checkLine );
            const LINE temporaryNextDirection = lines[nextIndex];
            const int temporaryTurn = turnSide( temporaryCurrentDirection,
                                                temporaryNextDirection );
            directionChanged = temporaryTurn != nextTurn;
            if( !directionChanged )
            {
                const auto border = ( temporaryTurn > 0 ? lines[nextIndex]
                                                        : lines[nextIndex].Opposite() )
                                            .Translate( -aHalfWidth );
                if( !border )
                    return {};
                if( cornerToCheck && sideApprox( *border, approximate( *cornerToCheck ) ) > 0
                    && border->SideOf( Corner( index ) ) < 0
                    && border->SideOf( Corner( index - 1 ) ) < 0 )
                {
                    dogEarLines.push_back( *border );
                }
                temporaryCurrentDirection = temporaryNextDirection;
                currentLine = *border;
            }
        }

        const auto previousCheckCorner = approximate( Corner( index - 1 ) );
        checkLine = previousTurn > 0 ? offsetLines[2] : offsetLines[0];
        currentLine = offsetLines[3];
        temporaryCurrentDirection = previousDirection;
        directionChanged = false;
        cornerToCheck.reset();
        for( int previousIndex = index - 2; previousIndex >= 1; --previousIndex )
        {
            if( distanceSquared( approximate( Corner( previousIndex ) ),
                                 previousCheckCorner ) > checkDistance )
            {
                break;
            }
            if( !directionChanged )
                cornerToCheck = currentLine.Intersection( checkLine );
            const LINE temporaryPreviousDirection = lines[previousIndex];
            const int temporaryTurn = turnSide( temporaryPreviousDirection,
                                                temporaryCurrentDirection );
            directionChanged = temporaryTurn != previousTurn;
            if( !directionChanged )
            {
                const auto border = ( temporaryTurn > 0 ? lines[previousIndex]
                                                        : lines[previousIndex].Opposite() )
                                            .Translate( -aHalfWidth );
                if( !border )
                    return {};
                if( cornerToCheck && sideApprox( *border, approximate( *cornerToCheck ) ) > 0
                    && border->SideOf( Corner( index ) ) < 0
                    && border->SideOf( Corner( index - 1 ) ) < 0 )
                {
                    dogEarLines.push_back( *border );
                }
                temporaryCurrentDirection = temporaryPreviousDirection;
                currentLine = *border;
            }
        }

        SIMPLEX shape = SIMPLEX::GetInstance( std::move( offsetLines ) );
        if( !dogEarLines.empty() )
            shape = shape.Intersection( SIMPLEX::GetInstance( std::move( dogEarLines ) ) );
        const auto bounds = BoundingOctagon( index - 1, index );
        if( !bounds )
            return {};
        const auto boundingSimplex = bounds->Offset( aHalfWidth ).ToSimplex();
        if( !boundingSimplex )
            return {};
        result.push_back( boundingSimplex->Intersection( shape ).Simplify() );
        previousDirection = currentDirection;
        currentDirection = nextDirection;
    }
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
