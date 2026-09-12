/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct native counterpart of Freerouting
 * board/optimize/TraceTightener45.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#include "TraceTightener45.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <vector>

#include "../../AutorouterDebug.h"
#include "../../geometry/planar/Polyline.h"
#include "../../maze/MazeSearchEngine.h"


namespace KICAD_AUTOROUTER
{
namespace
{

using PLANAR::INTEGER;
using PLANAR::LINE;
using PLANAR::POINT;
using PLANAR::POLYLINE;

constexpr double MIN_TRANSLATE_DISTANCE =
        500.0 * FREEROUTING_COORDINATE_UNIT_IU;
constexpr double ONE_SOURCE_COORDINATE = FREEROUTING_COORDINATE_UNIT_IU;
constexpr std::int64_t SOURCE_COORDINATE_IU =
        static_cast<std::int64_t>( FREEROUTING_COORDINATE_UNIT_IU );


int sign( double aValue )
{
    return aValue > 0 ? 1 : aValue < 0 ? -1 : 0;
}


bool collinear( ROUTER_POINT aPoint, ROUTER_POINT aFirst,
                ROUTER_POINT aSecond )
{
    const INTEGER firstX = INTEGER( aPoint.x ) - aFirst.x;
    const INTEGER firstY = INTEGER( aPoint.y ) - aFirst.y;
    const INTEGER secondX = INTEGER( aSecond.x ) - aFirst.x;
    const INTEGER secondY = INTEGER( aSecond.y ) - aFirst.y;
    return firstX * secondY == firstY * secondX;
}


std::optional<ROUTER_POINT> translatedPoint( ROUTER_POINT aPoint,
                                             ROUTER_POINT aDelta )
{
    const INTEGER x = INTEGER( aPoint.x ) + aDelta.x;
    const INTEGER y = INTEGER( aPoint.y ) + aDelta.y;
    const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
    const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
    if( x < minimum || x > maximum || y < minimum || y > maximum )
        return {};
    return ROUTER_POINT{ x.convert_to<std::int64_t>(),
                         y.convert_to<std::int64_t>() };
}


std::optional<ROUTER_POINT> difference( ROUTER_POINT aFirst,
                                        ROUTER_POINT aSecond )
{
    const INTEGER x = INTEGER( aFirst.x ) - aSecond.x;
    const INTEGER y = INTEGER( aFirst.y ) - aSecond.y;
    const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
    const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
    if( x < minimum || x > maximum || y < minimum || y > maximum )
        return {};
    return ROUTER_POINT{ x.convert_to<std::int64_t>(),
                         y.convert_to<std::int64_t>() };
}


INTEGER floorDivide( const INTEGER& aNumerator, const INTEGER& aDenominator )
{
    INTEGER quotient = aNumerator / aDenominator;
    if( aNumerator % aDenominator != 0 && aNumerator < 0 )
        --quotient;
    return quotient;
}


INTEGER ceilDivide( const INTEGER& aNumerator, const INTEGER& aDenominator )
{
    INTEGER quotient = aNumerator / aDenominator;
    if( aNumerator % aDenominator != 0 && aNumerator > 0 )
        ++quotient;
    return quotient;
}


std::optional<std::int64_t> sourceCoordinateToIu( const INTEGER& aCoordinate )
{
    const INTEGER scaled = aCoordinate * SOURCE_COORDINATE_IU;
    const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
    const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
    if( scaled < minimum || scaled > maximum )
        return {};
    return scaled.convert_to<std::int64_t>();
}


std::optional<std::int64_t> roundedSourceCoordinate(
        const INTEGER& aNumerator, const INTEGER& aDenominator )
{
    // Java Math.round(v) is floor(v + 0.5), including for negative values.
    const INTEGER sourceDenominator = aDenominator * SOURCE_COORDINATE_IU;
    const INTEGER rounded = floorDivide(
            2 * aNumerator + sourceDenominator, 2 * sourceDenominator );
    return sourceCoordinateToIu( rounded );
}


std::optional<std::int64_t> floorSourceCoordinate(
        const INTEGER& aNumerator, const INTEGER& aDenominator )
{
    return sourceCoordinateToIu( floorDivide(
            aNumerator, aDenominator * SOURCE_COORDINATE_IU ) );
}


std::optional<std::int64_t> ceilSourceCoordinate(
        const INTEGER& aNumerator, const INTEGER& aDenominator )
{
    return sourceCoordinateToIu( ceilDivide(
            aNumerator, aDenominator * SOURCE_COORDINATE_IU ) );
}


bool isIntegralSourceCoordinate( const INTEGER& aNumerator,
                                 const INTEGER& aDenominator )
{
    return aNumerator % ( aDenominator * SOURCE_COORDINATE_IU ) == 0;
}


std::optional<ROUTER_POINT> rounded( const POINT& aPoint )
{
    const auto x = roundedSourceCoordinate( aPoint.x, aPoint.z );
    const auto y = roundedSourceCoordinate( aPoint.y, aPoint.z );
    if( !x || !y )
    {
        return {};
    }
    return ROUTER_POINT{ *x, *y };
}


std::optional<ROUTER_POINT> integralSourcePoint( const POINT& aPoint )
{
    if( !isIntegralSourceCoordinate( aPoint.x, aPoint.z )
        || !isIntegralSourceCoordinate( aPoint.y, aPoint.z ) )
    {
        return {};
    }

    const INTEGER x = aPoint.x / aPoint.z;
    const INTEGER y = aPoint.y / aPoint.z;
    const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
    const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
    if( x < minimum || x > maximum || y < minimum || y > maximum )
        return {};
    return ROUTER_POINT{ x.convert_to<std::int64_t>(),
                         y.convert_to<std::int64_t>() };
}


std::optional<LINE> translatedSourceLine( const LINE& aLine,
                                          double aDistance )
{
    if( !aLine.IsMultipleOf45Degree() )
        return {};
    const ROUTER_POINT direction{ PLANAR::Sign( aLine.Dx() ),
                                  PLANAR::Sign( aLine.Dy() ) };

    const double sourceDistance = aDistance / ONE_SOURCE_COORDINATE;
    const double length = std::hypot( static_cast<double>( direction.x ),
                                      static_cast<double>( direction.y ) );
    ROUTER_POINT anchor = aLine.a;
    if( direction.x * direction.x <= direction.y * direction.y )
    {
        const std::int64_t relativeX = static_cast<std::int64_t>( std::floor(
                sourceDistance * length / direction.y + 0.5 ) );
        const INTEGER x = INTEGER( anchor.x )
                          - INTEGER( relativeX ) * SOURCE_COORDINATE_IU;
        const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
        const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
        if( x < minimum || x > maximum )
            return {};
        anchor.x = x.convert_to<std::int64_t>();
    }
    else
    {
        const std::int64_t relativeY = static_cast<std::int64_t>( std::floor(
                sourceDistance * length / direction.x + 0.5 ) );
        const INTEGER y = INTEGER( anchor.y )
                          + INTEGER( relativeY ) * SOURCE_COORDINATE_IU;
        const INTEGER minimum = std::numeric_limits<std::int64_t>::min();
        const INTEGER maximum = std::numeric_limits<std::int64_t>::max();
        if( y < minimum || y > maximum )
            return {};
        anchor.y = y.convert_to<std::int64_t>();
    }
    return LINE::FromDirection( anchor, direction.x, direction.y );
}


bool samePolyline( const POLYLINE& aFirst, const POLYLINE& aSecond )
{
    if( aFirst.lines.size() != aSecond.lines.size() )
        return false;
    for( std::size_t index = 0; index < aFirst.lines.size(); ++index )
    {
        if( aFirst.lines[index].a != aSecond.lines[index].a
            || aFirst.lines[index].b != aSecond.lines[index].b )
        {
            return false;
        }
    }
    return true;
}


POLYLINE avoidAcidTraps( const POLYLINE& aPolyline )
{
    // The pinned TraceTightener.avoidAcidTraps() returns immediately behind
    // an unconditional `if (true)`.  Keep the named lifecycle stage so a
    // future upstream re-enable has one explicit synchronization point, but
    // do not invent spring-over behavior absent from the reference commit.
    return aPolyline;
}


bool replaceRun( ROUTING_CONNECTION& aConnection, std::size_t aFirstNode,
                 std::size_t aLastNode,
                 const std::vector<ROUTER_POINT>& aCorners )
{
    if( aFirstNode >= aLastNode || aLastNode >= aConnection.nodes.size()
        || aCorners.size() < 2
        || aCorners.front() != aConnection.nodes[aFirstNode].point
        || aCorners.back() != aConnection.nodes[aLastNode].point
        || !CanCollapseRouteEdges( aConnection, aFirstNode, aLastNode - 1 ) )
    {
        return false;
    }

    const int layer = aConnection.nodes[aFirstNode].layer;
    std::vector<ROUTER_NODE> nodes;
    nodes.reserve( aConnection.nodes.size() - ( aLastNode - aFirstNode + 1 )
                   + aCorners.size() );
    nodes.insert( nodes.end(), aConnection.nodes.begin(),
                  aConnection.nodes.begin()
                          + static_cast<std::ptrdiff_t>( aFirstNode ) );
    for( ROUTER_POINT corner : aCorners )
    {
        if( nodes.empty() || nodes.back().point != corner
            || nodes.back().layer != layer )
        {
            nodes.push_back( { corner, layer } );
        }
    }
    nodes.insert( nodes.end(),
                  aConnection.nodes.begin()
                          + static_cast<std::ptrdiff_t>( aLastNode + 1 ),
                  aConnection.nodes.end() );

    if( nodes.size() < 2 )
        return false;

    if( !aConnection.edgeStyles.empty() )
    {
        const ROUTING_EDGE_STYLE style =
                EdgeStyle( aConnection, aFirstNode );
        std::vector<ROUTING_EDGE_STYLE> styles;
        styles.reserve( nodes.size() - 1 );
        styles.insert( styles.end(), aConnection.edgeStyles.begin(),
                       aConnection.edgeStyles.begin()
                               + static_cast<std::ptrdiff_t>( aFirstNode ) );
        styles.insert( styles.end(), aCorners.size() - 1, style );
        styles.insert( styles.end(),
                       aConnection.edgeStyles.begin()
                               + static_cast<std::ptrdiff_t>( aLastNode ),
                       aConnection.edgeStyles.end() );
        aConnection.edgeStyles = std::move( styles );
    }

    aConnection.nodes = std::move( nodes );
    return HasValidEdgeStyles( aConnection );
}


struct RUN_CONTEXT
{
    const ROUTING_CONNECTION& connection;
    std::size_t firstNode;
    std::size_t lastNode;
    int layer;
    ROUTING_EDGE_STYLE style;
    const MAZE_SEARCH_ENGINE& search;

    bool SegmentAllowed( ROUTER_POINT aFirst, ROUTER_POINT aSecond ) const
    {
        if( aFirst == aSecond )
            return true;
        return search.CanInsertSegment( connection.netCode,
                                        { aFirst, layer }, { aSecond, layer },
                                        &style );
    }

    bool PolylineAllowed( const POLYLINE& aPolyline ) const
    {
        const auto corners = aPolyline.IntegralCorners();
        if( !corners )
            return false;
        ROUTING_CONNECTION candidate = connection;
        return replaceRun( candidate, firstNode, lastNode, *corners )
               && search.CanInsertTraceSpan( candidate );
    }

    /** Check the one local segment represented by three support lines.
     *
     * TraceTightener checks the offset shape of only the support being moved;
     * it does not re-check the complete trace after every intermediate line
     * operation.  The complete native candidate is still validated once at
     * the publication boundary in pullRun().
     */
    bool LocalSupportAllowed( const LINE& aPrevious, const LINE& aCandidate,
                              const LINE& aNext ) const
    {
        const POLYLINE local( { aPrevious, aCandidate, aNext } );
        const auto corners = local.IntegralCorners();
        return corners && corners->size() == 2
               && SegmentAllowed( ( *corners )[0], ( *corners )[1] );
    }
};


void debugPolyline( const char* aStage, int aPass, const POLYLINE& aPolyline,
                    const RUN_CONTEXT& aContext )
{
    if( !autorouterDebugEnabled() )
        return;
    std::ostringstream message;
    message << std::fixed << std::setprecision( 0 );
    message << "COMPARE_T45 stage=" << aStage << '-' << aPass
            << " net=" << aContext.connection.netCode << " corners=";
    for( std::size_t index = 0; index < aPolyline.CornerCount(); ++index )
    {
        if( index )
            message << ';';
        const POINT corner = aPolyline.Corner( index );
        message << '(' << corner.X() << ',' << corner.Y() << ')';
    }
    autorouterDebugLog( message.str() );
}


POLYLINE reduceCorners( const POLYLINE& aPolyline, const RUN_CONTEXT& aContext )
{
    if( aPolyline.lines.size() <= 4 )
        return aPolyline;

    const auto sourceCorners = aPolyline.IntegralCorners();
    if( !sourceCorners || sourceCorners->size() < 4 )
        return aPolyline;

    std::array<ROUTER_POINT, 4> current{
            ( *sourceCorners )[0], ( *sourceCorners )[1],
            ( *sourceCorners )[2], ( *sourceCorners )[3] };
    std::vector<ROUTER_POINT> newCorners;
    newCorners.reserve( sourceCorners->size() );
    newCorners.push_back( current[0] );

    bool changed = false;
    std::size_t cornerIndex = 3;
    while( cornerIndex < sourceCorners->size() )
    {
        current[3] = ( *sourceCorners )[cornerIndex];
        if( current[1] == current[2]
            || ( cornerIndex + 1 < sourceCorners->size()
                 && collinear( current[3], current[1], current[2] ) ) )
        {
            ++cornerIndex;
            current[2] = current[3];
            if( cornerIndex < sourceCorners->size() )
                current[3] = ( *sourceCorners )[cornerIndex];
            changed = true;
        }

        bool removed = false;
        std::optional<ROUTER_POINT> newCorner;
        if( cornerIndex <= sourceCorners->size() )
        {
            if( const auto delta = difference( current[3], current[2] ) )
                newCorner = translatedPoint( current[1], *delta );
            if( newCorner )
            {
                if( current[3] == current[2] )
                {
                    removed = true;
                }
                else if( collinear( *newCorner, current[0], current[1] )
                         && aContext.SegmentAllowed( *newCorner, current[1] )
                         && aContext.SegmentAllowed( *newCorner, current[3] ) )
                {
                    removed = true;
                }
            }
        }

        if( !removed )
        {
            if( const auto delta = difference( current[0], current[1] ) )
                newCorner = translatedPoint( current[2], *delta );
            else
                newCorner.reset();
            if( newCorner )
            {
                if( current[0] == current[1] )
                {
                    removed = true;
                }
                else if( collinear( *newCorner, current[2], current[3] )
                         && aContext.SegmentAllowed( *newCorner, current[0] )
                         && aContext.SegmentAllowed( *newCorner, current[2] ) )
                {
                    removed = true;
                }
            }
        }

        if( removed && newCorner )
        {
            changed = true;
            current[1] = *newCorner;
        }
        else
        {
            newCorners.push_back( current[1] );
            current[0] = current[1];
            current[1] = current[2];
        }

        current[2] = current[3];
        ++cornerIndex;
    }

    if( !changed )
        return aPolyline;

    newCorners.push_back( current[1] );
    newCorners.push_back( current[2] );
    newCorners.erase( std::unique( newCorners.begin(), newCorners.end() ),
                      newCorners.end() );
    if( newCorners.size() < 2 )
        return aPolyline;

    POLYLINE result = POLYLINE::FromPoints( newCorners );
    return !result.Empty() ? result : aPolyline;
}


std::optional<ROUTER_POINT> primitive45Direction( const LINE& aLine )
{
    if( !aLine.IsMultipleOf45Degree() )
        return {};
    return ROUTER_POINT{ PLANAR::Sign( aLine.Dx() ),
                         PLANAR::Sign( aLine.Dy() ) };
}


std::optional<LINE> lineThroughRoundedIntersection(
        const LINE& aFirst, const LINE& aSecond, ROUTER_POINT aDirection )
{
    const auto intersection = aFirst.Intersection( aSecond );
    if( !intersection )
        return {};
    const auto point = rounded( *intersection );
    if( !point )
        return {};
    return LINE::FromDirection( *point, aDirection.x, aDirection.y );
}


/** Native translation of TraceTightener45.smoothenNonIntegerCorner().
 *
 * Two diagonal source support lines can intersect on a half-coordinate.  A
 * short axis-parallel support on the adjacent source lattice removes that
 * rational corner without changing the route's 45-degree direction family.
 */
std::optional<LINE> smoothenNonIntegerCorner(
        const POLYLINE& aPolyline, std::size_t aIndex )
{
    if( aIndex == 0 || aIndex + 2 >= aPolyline.lines.size() )
        return {};

    const LINE& previousLine = aPolyline.lines[aIndex];
    const LINE& nextLine = aPolyline.lines[aIndex + 1];
    if( previousLine.EqualOrOpposite( nextLine )
        || !previousLine.IsDiagonal() || !nextLine.IsDiagonal() )
    {
        return {};
    }

    const auto currentCorner = previousLine.Intersection( nextLine );
    const auto previousCorner = previousLine.Intersection(
            aPolyline.lines[aIndex - 1] );
    const auto nextCorner = nextLine.Intersection(
            aPolyline.lines[aIndex + 2] );
    if( !currentCorner || !previousCorner || !nextCorner )
        return {};

    bool vertical = false;
    bool horizontal = false;
    bool useCeiling = false;
    if( previousCorner->CompareX( *currentCorner ) > 0
        && nextCorner->CompareX( *currentCorner ) > 0 )
    {
        vertical = true;
        useCeiling = true;
    }
    else if( previousCorner->CompareX( *currentCorner ) < 0
             && nextCorner->CompareX( *currentCorner ) < 0 )
    {
        vertical = true;
    }
    else if( previousCorner->CompareY( *currentCorner ) > 0
             && nextCorner->CompareY( *currentCorner ) > 0 )
    {
        horizontal = true;
        useCeiling = true;
    }
    else if( previousCorner->CompareY( *currentCorner ) < 0
             && nextCorner->CompareY( *currentCorner ) < 0 )
    {
        horizontal = true;
    }
    else
    {
        return {};
    }

    const auto newX = useCeiling
            ? ceilSourceCoordinate( currentCorner->x, currentCorner->z )
            : floorSourceCoordinate( currentCorner->x, currentCorner->z );
    const auto newY = useCeiling
            ? ceilSourceCoordinate( currentCorner->y, currentCorner->z )
            : floorSourceCoordinate( currentCorner->y, currentCorner->z );
    if( !newX || !newY )
        return {};

    ROUTER_POINT direction;
    if( vertical )
    {
        direction = { 0, previousCorner->CompareY( *nextCorner ) < 0 ? 1 : -1 };
    }
    else if( horizontal )
    {
        direction = { previousCorner->CompareX( *nextCorner ) < 0 ? 1 : -1, 0 };
    }
    else
    {
        return {};
    }

    return LINE::FromDirection( { *newX, *newY }, direction.x, direction.y );
}


std::optional<LINE> smoothenCorner( const POLYLINE& aPolyline,
                                    std::size_t aIndex,
                                    const RUN_CONTEXT& aContext )
{
    const auto previousCorner =
            aPolyline.lines[aIndex].Intersection( aPolyline.lines[aIndex - 1] );
    const auto currentCorner =
            aPolyline.lines[aIndex].Intersection( aPolyline.lines[aIndex + 1] );
    const auto nextCorner =
            aPolyline.lines[aIndex + 1].Intersection( aPolyline.lines[aIndex + 2] );
    const auto previousDirection = primitive45Direction( aPolyline.lines[aIndex] );
    const auto nextDirection = primitive45Direction( aPolyline.lines[aIndex + 1] );
    if( !previousCorner || !currentCorner || !nextCorner
        || !previousDirection || !nextDirection )
    {
        return {};
    }

    const ROUTER_POINT direction{
            previousDirection->x + nextDirection->x,
            previousDirection->y + nextDirection->y };
    if( direction.x == 0 && direction.y == 0 )
        return {};
    const auto translateLine = lineThroughRoundedIntersection(
            aPolyline.lines[aIndex], aPolyline.lines[aIndex + 1], direction );
    if( !translateLine )
        return {};

    const double previousDistance = std::abs( translateLine->SignedDistance(
            previousCorner->X(), previousCorner->Y() ) );
    const double nextDistance = std::abs( translateLine->SignedDistance(
            nextCorner->X(), nextCorner->Y() ) );
    if( previousDistance == 0 || nextDistance == 0 )
        return {};

    const POINT& nearestCorner = previousDistance <= nextDistance
                                         ? *previousCorner : *nextCorner;
    double maximumDistance = std::min( previousDistance, nextDistance );
    if( maximumDistance < ONE_SOURCE_COORDINATE )
        return {};
    maximumDistance = std::max( maximumDistance - ONE_SOURCE_COORDINATE,
                                ONE_SOURCE_COORDINATE );
    if( translateLine->SideOf( *nextCorner ) > 0 )
        maximumDistance = -maximumDistance;

    double translateDistance = maximumDistance;
    double deltaDistance = maximumDistance;
    const int nearestSide = translateLine->SideOf( nearestCorner );
    const int directionSign = sign( maximumDistance );
    std::optional<LINE> result;
    while( std::abs( deltaDistance ) > MIN_TRANSLATE_DISTANCE )
    {
        const auto candidateLine = translatedSourceLine(
                *translateLine, translateDistance );
        if( !candidateLine )
            break;
        const int candidateSide = candidateLine->SideOf( nearestCorner );
        if( candidateSide != nearestSide && candidateSide != 0 )
        {
            const double shorten = directionSign * 0.5 * ONE_SOURCE_COORDINATE;
            maximumDistance -= shorten;
            translateDistance -= shorten;
            deltaDistance -= shorten;
            continue;
        }

        const bool allowed = aContext.LocalSupportAllowed(
                aPolyline.lines[aIndex], *candidateLine,
                aPolyline.lines[aIndex + 1] );
        if( autorouterDebugEnabled() )
        {
            std::ostringstream message;
            message << std::fixed << std::setprecision( 3 )
                    << "COMPARE_T45_CHECK net=" << aContext.connection.netCode
                    << " max=" << maximumDistance
                    << " distance=" << translateDistance
                    << " allowed=" << allowed
                    << " line=" << candidateLine->a.x << ','
                    << candidateLine->a.y << "->" << candidateLine->b.x
                    << ',' << candidateLine->b.y;
            autorouterDebugLog( message.str() );
        }
        deltaDistance /= 2;
        if( allowed )
        {
            result = candidateLine;
            if( translateDistance == maximumDistance )
                break;
            translateDistance += deltaDistance;
        }
        else
        {
            translateDistance -= deltaDistance;
        }
    }
    return result;
}


std::optional<LINE> smoothenSharpCorner( const POLYLINE& aPolyline,
                                         std::size_t aIndex,
                                         const RUN_CONTEXT& aContext )
{
    const auto previousCorner =
            aPolyline.lines[aIndex].Intersection( aPolyline.lines[aIndex - 1] );
    const auto currentCorner =
            aPolyline.lines[aIndex].Intersection( aPolyline.lines[aIndex + 1] );
    const auto nextCorner =
            aPolyline.lines[aIndex + 1].Intersection( aPolyline.lines[aIndex + 2] );
    const auto previousDirection = primitive45Direction( aPolyline.lines[aIndex] );
    const auto nextDirection = primitive45Direction( aPolyline.lines[aIndex + 1] );
    if( !previousCorner || !currentCorner || !nextCorner
        || !previousDirection || !nextDirection )
    {
        return {};
    }

    if( !isIntegralSourceCoordinate( currentCorner->x, currentCorner->z ) )
    {
        if( const auto result = smoothenNonIntegerCorner( aPolyline, aIndex ) )
            return result;
    }

    const ROUTER_POINT direction{
            previousDirection->x + nextDirection->x,
            previousDirection->y + nextDirection->y };
    if( direction.x == 0 && direction.y == 0 )
        return {};
    const auto translateLine = lineThroughRoundedIntersection(
            aPolyline.lines[aIndex], aPolyline.lines[aIndex + 1], direction );
    if( !translateLine )
        return {};

    // TraceTightener obtains the default (uncompensated) board search tree at
    // the pinned source commit, so currentHalfWidth is the physical trace
    // half-width. Pair clearance is still enforced by the checked local move
    // and by the complete native publication check.
    double distance = ( std::sqrt( 2.0 ) - 1.0 )
                      * ( aContext.search.ResolveTrackWidth(
                                  aContext.connection.netCode, aContext.style ) / 2.0 );
    distance = std::min( distance, std::abs( translateLine->SignedDistance(
                                      previousCorner->X(), previousCorner->Y() ) ) );
    distance = std::min( distance, std::abs( translateLine->SignedDistance(
                                      nextCorner->X(), nextCorner->Y() ) ) );
    if( distance < 0.99 * ONE_SOURCE_COORDINATE )
        return {};
    distance = std::max( distance - ONE_SOURCE_COORDINATE,
                         ONE_SOURCE_COORDINATE );
    if( translateLine->SideOf( *nextCorner ) > 0 )
        distance = -distance;
    return translatedSourceLine( *translateLine, distance );
}


POLYLINE smoothenCorners( const POLYLINE& aPolyline,
                          const RUN_CONTEXT& aContext )
{
    POLYLINE result = aPolyline;
    bool changed = true;
    int passCount = 0;
    while( changed && passCount++ < 256 )
    {
        if( result.lines.size() < 4 )
            return result;
        changed = false;
        std::vector<LINE> lines = result.lines;
        for( std::size_t index = 1; index + 2 < lines.size(); ++index )
        {
            if( !lines[index].IsMultipleOf45Degree()
                || !lines[index + 1].IsMultipleOf45Degree()
                || lines[index].DirectionScalarProduct( lines[index + 1] ) > 0 )
            {
                continue;
            }

            const POLYLINE current( lines );
            std::optional<LINE> newLine = smoothenCorner(
                    current, index, aContext );
            if( !newLine )
                newLine = smoothenSharpCorner( current, index, aContext );
            if( !newLine )
                continue;

            lines.insert( lines.begin()
                                  + static_cast<std::ptrdiff_t>( index + 1 ),
                          *newLine );
            changed = true;
            ++index;
        }
        if( changed )
            result = POLYLINE( std::move( lines ) );
    }
    return result;
}


std::optional<LINE> repositionLine( const POLYLINE& aPolyline,
                                    std::size_t aIndex,
                                    const RUN_CONTEXT& aContext )
{
    if( aIndex < 2 || aIndex + 2 >= aPolyline.lines.size() )
        return {};
    const LINE& translateLine = aPolyline.lines[aIndex];
    const auto previousCorner = aPolyline.lines[aIndex - 2].Intersection(
            aPolyline.lines[aIndex - 1] );
    const auto nextCorner = aPolyline.lines[aIndex + 1].Intersection(
            aPolyline.lines[aIndex + 2] );
    if( !previousCorner || !nextCorner )
        return {};

    const double previousDistance = translateLine.SignedDistance(
            previousCorner->X(), previousCorner->Y() );
    const double nextDistance = translateLine.SignedDistance(
            nextCorner->X(), nextCorner->Y() );
    if( sign( previousDistance ) != sign( nextDistance ) )
        return {};

    const POINT& nearest = std::abs( previousDistance ) < std::abs( nextDistance )
                                   ? *previousCorner : *nextCorner;
    double maximumDistance = std::abs( previousDistance ) < std::abs( nextDistance )
                                     ? previousDistance : nextDistance;
    double translateDistance = maximumDistance;
    double deltaDistance = maximumDistance;
    const int nearestSide = translateLine.SideOf( nearest );
    const int directionSign = sign( maximumDistance );
    bool firstTime = true;
    std::optional<LINE> result;

    while( firstTime || std::abs( deltaDistance ) > MIN_TRANSLATE_DISTANCE )
    {
        std::optional<LINE> candidateLine;
        if( firstTime )
        {
            if( const auto integral = integralSourcePoint( nearest ) )
                candidateLine = LINE::FromDirection(
                        *integral, translateLine.Dx(), translateLine.Dy() );
        }
        if( !candidateLine )
            candidateLine = translatedSourceLine( translateLine,
                                                   -translateDistance );
        if( !candidateLine || candidateLine->SameDirectedSupport( translateLine ) )
            return {};

        const int candidateSide = candidateLine->SideOf( nearest );
        if( candidateSide != nearestSide && candidateSide != 0 )
        {
            const double shorten = directionSign * 0.5 * ONE_SOURCE_COORDINATE;
            maximumDistance -= shorten;
            translateDistance -= shorten;
            deltaDistance -= shorten;
            continue;
        }

        const bool allowed = aContext.LocalSupportAllowed(
                aPolyline.lines[aIndex - 1], *candidateLine,
                aPolyline.lines[aIndex + 1] );
        deltaDistance /= 2;
        if( allowed )
        {
            result = candidateLine;
            if( firstTime )
                break;
            translateDistance += deltaDistance;
        }
        else
        {
            translateDistance -= deltaDistance;
        }
        firstTime = false;
    }
    return result;
}


POLYLINE repositionLines( const POLYLINE& aPolyline,
                          const RUN_CONTEXT& aContext )
{
    if( aPolyline.lines.size() < 5 )
        return aPolyline;
    for( std::size_t index = 2; index + 2 < aPolyline.lines.size(); ++index )
    {
        const auto line = repositionLine( aPolyline, index, aContext );
        if( !line )
            continue;
        std::vector<LINE> lines = aPolyline.lines;
        lines[index] = *line;
        POLYLINE result( std::move( lines ) );
        if( !result.Empty() )
            return result;
    }
    return aPolyline;
}


bool pullRun( ROUTING_CONNECTION& aConnection, std::size_t aFirstNode,
              std::size_t aLastNode, const MAZE_SEARCH_ENGINE& aSearch )
{
    std::vector<ROUTER_POINT> points;
    points.reserve( aLastNode - aFirstNode + 1 );
    for( std::size_t index = aFirstNode; index <= aLastNode; ++index )
        points.push_back( aConnection.nodes[index].point );
    POLYLINE current = avoidAcidTraps( POLYLINE::FromPoints( points ) );
    if( current.Empty() || !current.IsMultipleOf45Degree() )
        return false;

    const RUN_CONTEXT context{ aConnection, aFirstNode, aLastNode,
                               aConnection.nodes[aFirstNode].layer,
                               EdgeStyle( aConnection, aFirstNode ), aSearch };
    debugPolyline( "input", 0, current, context );
    int fixedPointPasses = 0;
    while( fixedPointPasses++ < 256 )
    {
        const POLYLINE before = current;
        current = reduceCorners( current, context );
        debugPolyline( "reduce", fixedPointPasses - 1, current, context );
        current = smoothenCorners( current, context );
        debugPolyline( "smooth", fixedPointPasses - 1, current, context );
        current = repositionLines( current, context );
        debugPolyline( "reposition", fixedPointPasses - 1, current, context );
        if( samePolyline( current, before ) )
            break;
    }

    const auto corners = current.IntegralCorners();
    if( !corners )
        return false;
    if( corners->size() == points.size()
        && std::equal( corners->begin(), corners->end(), points.begin() ) )
    {
        return false;
    }
    ROUTING_CONNECTION candidate = aConnection;
    if( !replaceRun( candidate, aFirstNode, aLastNode, *corners )
        || !aSearch.CanInsertTraceSpan( candidate ) )
    {
        return false;
    }
    aConnection = std::move( candidate );
    return true;
}

} // namespace


bool TRACE_TIGHTENER_45::PullTight( ROUTING_CONNECTION& aConnection,
                                    const MAZE_SEARCH_ENGINE& aSearch )
{
    if( aConnection.nodes.size() < 3 || !HasValidEdgeStyles( aConnection ) )
        return false;

    bool changedAny = false;
    bool changed = true;
    int passCount = 0;
    while( changed && passCount++ < 256 )
    {
        changed = false;
        for( std::size_t first = 0; first + 2 < aConnection.nodes.size(); )
        {
            if( aConnection.nodes[first].layer
                != aConnection.nodes[first + 1].layer )
            {
                ++first;
                continue;
            }

            const int layer = aConnection.nodes[first].layer;
            const ROUTING_EDGE_STYLE style = EdgeStyle( aConnection, first );
            std::size_t last = first + 1;
            while( last + 1 < aConnection.nodes.size()
                   && aConnection.nodes[last].layer == layer
                   && aConnection.nodes[last + 1].layer == layer
                   && EdgeStyle( aConnection, last ) == style )
            {
                ++last;
            }

            if( last >= first + 2 && pullRun( aConnection, first, last, aSearch ) )
            {
                changed = true;
                changedAny = true;
                break;
            }
            first = last;
        }
    }
    return changedAny;
}

} // namespace KICAD_AUTOROUTER
