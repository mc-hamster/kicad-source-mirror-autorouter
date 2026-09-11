/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Freerouting a11c0a42 (GPL-3.0): FoundConnectionLocator.calculateAdditionalCorner,
 * FoundConnectionLocator45Degree.calculateNextTraceCorners (rectangular subset).
 */
#include "FoundConnectionLocator45Degree.h"
#include "FoundConnectionLocatorAnyAngle.h"
#include "../AutorouterDebug.h"

#include <algorithm>
#include <cmath>

namespace KICAD_AUTOROUTER
{
namespace
{
int signum( double aValue )
{
    return aValue < 0 ? -1 : aValue > 0 ? 1 : 0;
}


std::optional<ROUTER_POINT> nearestIntegralPoint(
        const PLANAR::INT_OCTAGON& aShape, ROUTER_POINT aFrom )
{
    if( aShape.Dimension() < 0 )
        return std::nullopt;

    if( aShape.Contains( aFrom ) )
        return aFrom;

    const auto simplex = aShape.ToSimplex();

    if( !simplex )
        return std::nullopt;

    return FOUND_CONNECTION_LOCATOR_ANY_ANGLE::NearestIntegralPoint( *simplex, aFrom );
}


void appendPoint( std::vector<ROUTER_POINT>& aPoints, ROUTER_POINT aPoint )
{
    if( aPoints.empty() || aPoints.back() != aPoint )
        aPoints.push_back( aPoint );
}


bool horizontalFirstFromDoor( const std::optional<PLANAR::INT_OCTAGON>& aDoor,
                              FLOAT_POINT aFrom, FLOAT_POINT aTo )
{
    if( !aDoor || aDoor->Dimension() < 0 )
        return true;

    const ROUTER_BOX box = aDoor->BoundingBox();

    if( aDoor->Dimension() != 1 )
        return box.maxY - box.minY >= box.maxX - box.minX;

    FLOAT_POINT left{ static_cast<double>( aDoor->Corner( 0 ).x ),
                      static_cast<double>( aDoor->Corner( 0 ).y ) };
    FLOAT_POINT right{ static_cast<double>( aDoor->Corner( 4 ).x ),
                       static_cast<double>( aDoor->Corner( 4 ).y ) };

    if( right.x < left.x || ( right.x == left.x && right.y < left.y ) )
        std::swap( left, right );

    const double dx = right.x - left.x;
    const double dy = right.y - left.y;
    const double maximumWidth = std::max( dx, std::abs( dy ) );
    const double halfMaximumWidth = 0.5 * maximumWidth;

    if( box.maxX - box.minX <= halfMaximumWidth )
        return true;

    if( box.maxY - box.minY <= halfMaximumWidth )
        return false;

    const double pathDx = aTo.x - aFrom.x;
    const double pathDy = aTo.y - aFrom.y;
    const bool sameSign = signum( pathDx ) == signum( pathDy );

    if( left.y < right.y )
        return sameSign ? std::abs( pathDx ) > std::abs( pathDy )
                        : std::abs( pathDx ) < std::abs( pathDy );

    return sameSign ? std::abs( pathDx ) < std::abs( pathDy )
                    : std::abs( pathDx ) > std::abs( pathDy );
}


bool horizontalFirstToDoor( const PLANAR::INT_OCTAGON& aDoor,
                            FLOAT_POINT aFrom, FLOAT_POINT aTo )
{
    const ROUTER_BOX box = aDoor.BoundingBox();

    if( aDoor.Dimension() != 1 )
        return box.maxY - box.minY <= box.maxX - box.minX;

    FLOAT_POINT left{ static_cast<double>( aDoor.Corner( 0 ).x ),
                      static_cast<double>( aDoor.Corner( 0 ).y ) };
    FLOAT_POINT right{ static_cast<double>( aDoor.Corner( 4 ).x ),
                       static_cast<double>( aDoor.Corner( 4 ).y ) };

    if( right.x < left.x || ( right.x == left.x && right.y < left.y ) )
        std::swap( left, right );

    const double dx = right.x - left.x;
    const double dy = right.y - left.y;
    const double maximumWidth = std::max( dx, std::abs( dy ) );
    const double halfMaximumWidth = 0.5 * maximumWidth;

    if( box.maxX - box.minX <= halfMaximumWidth )
        return false;

    if( box.maxY - box.minY <= halfMaximumWidth )
        return true;

    const double pathDx = aTo.x - aFrom.x;
    const double pathDy = aTo.y - aFrom.y;
    const bool sameSign = signum( pathDx ) == signum( pathDy );

    if( left.y < right.y )
        return sameSign ? std::abs( pathDx ) < std::abs( pathDy )
                        : std::abs( pathDx ) > std::abs( pathDy );

    return sameSign ? std::abs( pathDx ) > std::abs( pathDy )
                    : std::abs( pathDx ) < std::abs( pathDy );
}


bool appendFortyFiveDegreeMove( std::vector<ROUTER_POINT>& aPoints,
                                ROUTER_POINT aTarget, bool aHorizontalFirst,
                                const PLANAR::INT_OCTAGON* aRequiredRoom = nullptr )
{
    if( aPoints.empty() )
        return false;

    const ROUTER_POINT from = aPoints.back();
    const FLOAT_POINT fromFloat{ static_cast<double>( from.x ),
                                 static_cast<double>( from.y ) };
    const FLOAT_POINT toFloat{ static_cast<double>( aTarget.x ),
                               static_cast<double>( aTarget.y ) };
    ROUTER_POINT corner = FOUND_CONNECTION_LOCATOR_45_DEGREE::CalculateAdditionalCorner(
            fromFloat, toFloat, aHorizontalFirst, false ).Round();

    if( aRequiredRoom && !aRequiredRoom->Contains( corner ) )
    {
        corner = FOUND_CONNECTION_LOCATOR_45_DEGREE::CalculateAdditionalCorner(
                fromFloat, toFloat, !aHorizontalFirst, false ).Round();

        if( !aRequiredRoom->Contains( corner ) )
            return false;
    }

    appendPoint( aPoints, corner );
    appendPoint( aPoints, aTarget );
    return true;
}
} // namespace

FLOAT_POINT FOUND_CONNECTION_LOCATOR_45_DEGREE::CalculateAdditionalCorner(
        FLOAT_POINT aFrom, FLOAT_POINT aTo, bool aHorizontalFirst, bool aOrthogonal )
{
    if( aOrthogonal )
        return aHorizontalFirst ? FLOAT_POINT{ aTo.x, aFrom.y } : FLOAT_POINT{ aFrom.x, aTo.y };
    const double dx = std::abs( aTo.x - aFrom.x );
    const double dy = std::abs( aTo.y - aFrom.y );
    if( dx <= dy )
    {
        if( aHorizontalFirst )
            return { aTo.x, aTo.y >= aFrom.y ? aFrom.y + dx : aFrom.y - dx };
        return { aFrom.x, aTo.y > aFrom.y ? aTo.y - dx : aTo.y + dx };
    }
    if( aHorizontalFirst )
        return { aTo.x > aFrom.x ? aTo.x - dy : aTo.x + dy, aFrom.y };
    return { aTo.x > aFrom.x ? aFrom.x + dy : aFrom.x - dy, aTo.y };
}

std::optional<std::vector<ROUTER_POINT>> FOUND_CONNECTION_LOCATOR_45_DEGREE::LocateRectangular(
        ROUTER_POINT aStart, const std::vector<RECTANGULAR_CORRIDOR_STEP>& aSteps,
        bool aOrthogonal )
{
    std::vector<ROUTER_POINT> points{ aStart };
    for( const auto& step : aSteps )
    {
        const auto from = points.back();
        if( !step.room.Contains( from ) )
            return std::nullopt;
        FLOAT_POINT to;
        bool horizontalFirst = true;
        if( step.door && INT_BOX::Dimension( *step.door ) == 2 )
        {
            // Reference uses the overlap shape here, not its diagonal section.
            to = { static_cast<double>( std::clamp( from.x, step.door->minX, step.door->maxX ) ),
                   static_cast<double>( std::clamp( from.y, step.door->minY, step.door->maxY ) ) };
            horizontalFirst = step.door->maxY - step.door->minY
                              <= step.door->maxX - step.door->minX;
        }
        else
        {
            const auto& line = step.section;
            const double dx = line.b.x - line.a.x, dy = line.b.y - line.a.y;
            const double lengthSquared = dx * dx + dy * dy;
            const double fraction = lengthSquared == 0 ? 0 : std::clamp(
                    ( ( from.x - line.a.x ) * dx + ( from.y - line.a.y ) * dy ) / lengthSquared, 0.0, 1.0 );
            to = { line.a.x + fraction * dx, line.a.y + fraction * dy };
            // Axis-aligned 1D doors: approach vertical doors vertically first,
            // horizontal doors horizontally first, matching the reference.
            if( step.door )
                horizontalFirst = step.door->minY == step.door->maxY;
        }
        const auto target = to.Round();
        const FLOAT_POINT roundedTo{ static_cast<double>( target.x ), static_cast<double>( target.y ) };
        const FLOAT_POINT roundedFrom{ static_cast<double>( from.x ), static_cast<double>( from.y ) };
        const auto corner = CalculateAdditionalCorner( roundedFrom, roundedTo,
                                                       horizontalFirst, aOrthogonal ).Round();
        // Compensation is already in the input rectangles; shrinking by the
        // trace width here again would close valid narrow channels.
        if( !step.room.Contains( target ) || !step.room.Contains( corner ) )
            return std::nullopt;
        for( const auto& point : { corner, target } )
        {
            if( points.back() == point )
                continue;
            if( points.size() > 1 )
            {
                const auto a = points[points.size() - 2], b = points.back();
                const long double dx1 = b.x - a.x, dy1 = b.y - a.y;
                const long double dx2 = point.x - b.x, dy2 = point.y - b.y;
                if( dx1 * dy2 == dy1 * dx2 && dx1 * dx2 + dy1 * dy2 >= 0 )
                    points.pop_back();
            }
            points.push_back( point );
        }
    }
    return points;
}


std::optional<std::vector<ROUTER_POINT>> FOUND_CONNECTION_LOCATOR_45_DEGREE::LocateOctagonal(
        ROUTER_POINT aStart, const std::vector<OCTAGONAL_CORRIDOR_STEP>& aSteps,
        double aCompensatedTraceHalfWidth, double aTraceWidthTolerance )
{
    if( aSteps.empty() )
        return std::vector<ROUTER_POINT>{ aStart };

    // Preserve the earlier zero-width helper for the geometry-only callers.
    // Source FoundConnectionLocator starts at the maze destination and walks
    // the backtrack chain toward the start item.  The native search records
    // the same corridor in start-to-destination order, so locate in reverse
    // and reverse the resulting corners at the end.
    if( aCompensatedTraceHalfWidth <= 0 )
    {
        std::vector<ROUTER_POINT> points{ aStart };
        for( const OCTAGONAL_CORRIDOR_STEP& step : aSteps )
        {
            const ROUTER_POINT from = points.back();
            if( !step.room.Contains( from ) )
                return std::nullopt;

            const FLOAT_LINE& line = step.section;
            const double dx = line.b.x - line.a.x;
            const double dy = line.b.y - line.a.y;
            const double lengthSquared = dx * dx + dy * dy;
            const double fraction = lengthSquared == 0 ? 0 : std::clamp(
                    ( ( from.x - line.a.x ) * dx + ( from.y - line.a.y ) * dy )
                            / lengthSquared,
                    0.0, 1.0 );
            const ROUTER_POINT target = FLOAT_POINT{
                    line.a.x + fraction * dx, line.a.y + fraction * dy }.Round();
            if( !step.room.Contains( target )
                || ( step.door && !step.door->Contains( target ) ) )
            {
                return std::nullopt;
            }

            const bool horizontalFirst = std::abs( dx ) >= std::abs( dy );
            if( !appendFortyFiveDegreeMove( points, target, horizontalFirst,
                                            &step.room ) )
                return std::nullopt;
        }
        return points;
    }

    const ROUTER_POINT destination = aSteps.back().section.Middle().Round();
    std::vector<ROUTER_POINT> reversePoints{ destination };
    const auto fail = [&]( const char* aReason, std::size_t aReverseIndex )
            -> std::optional<std::vector<ROUTER_POINT>>
    {
        autorouterDecisionLog(
                "LOCATOR_45_REJECTED",
                { { "reason", aReason },
                  { "reverse_index", std::to_string( aReverseIndex ) },
                  { "steps", std::to_string( aSteps.size() ) },
                  { "current", std::to_string( reversePoints.back().x ) + ','
                                         + std::to_string( reversePoints.back().y ) } } );
        return std::nullopt;
    };

    for( std::size_t reverseIndex = aSteps.size(); reverseIndex-- > 0; )
    {
        const OCTAGONAL_CORRIDOR_STEP& step = aSteps[reverseIndex];
        if( step.room.Dimension() != 2 )
            return fail( "room_dimension", reverseIndex );

        // Enter a room far enough for the complete trace cross-section.  This
        // is FoundConnectionLocator.adjustStartCorner plus the first part of
        // FoundConnectionLocator45Degree.calculateNextTraceCorners.
        const double shrinkOffset = aCompensatedTraceHalfWidth
                                    + ( step.obstacleRoom
                                                ? 0.0 : aTraceWidthTolerance );
        PLANAR::INT_OCTAGON shrunkenRoom = step.room.Offset( -shrinkOffset );
        std::optional<ROUTER_POINT> enteredRoomAt;
        if( shrunkenRoom.Dimension() == 2 )
        {
            const auto nearestRoom = nearestIntegralPoint(
                    shrunkenRoom, reversePoints.back() );
            if( !nearestRoom )
                return fail( "nearest_room", reverseIndex );
            enteredRoomAt = *nearestRoom;

            const std::optional<PLANAR::INT_OCTAGON> fromDoor = step.door;
            const FLOAT_POINT from{ static_cast<double>( reversePoints.back().x ),
                                    static_cast<double>( reversePoints.back().y ) };
            const FLOAT_POINT to{ static_cast<double>( nearestRoom->x ),
                                  static_cast<double>( nearestRoom->y ) };
            if( !appendFortyFiveDegreeMove(
                        reversePoints, *nearestRoom,
                        horizontalFirstFromDoor( fromDoor, from, to ) ) )
                return fail( "enter_room", reverseIndex );
        }
        else
        {
            shrunkenRoom = step.room;
        }

        ROUTER_POINT nextPoint;
        std::optional<PLANAR::INT_OCTAGON> toDoor;

        if( reverseIndex == 0 )
        {
            nextPoint = aStart;
        }
        else
        {
            const OCTAGONAL_CORRIDOR_STEP& previous = aSteps[reverseIndex - 1];
            toDoor = previous.door;
            if( !toDoor || toDoor->Dimension() < 1 )
                return fail( "to_door_dimension", reverseIndex );

            if( toDoor->Dimension() == 2 )
            {
                PLANAR::INT_OCTAGON shrunkenDoor = toDoor->Offset( -shrinkOffset );
                if( shrunkenDoor.Dimension() < 0 )
                    shrunkenDoor = *toDoor;
                const auto nearest = nearestIntegralPoint(
                        shrunkenDoor, reversePoints.back() );
                if( !nearest )
                    return fail( "nearest_door", reverseIndex );
                nextPoint = *nearest;
            }
            else
            {
                nextPoint = previous.section.NearestSegmentPoint(
                        { static_cast<double>( reversePoints.back().x ),
                          static_cast<double>( reversePoints.back().y ) } ).Round();

                // An acute corner at the far side of a one-dimensional door
                // can leave insufficient trace width.  Freerouting switches
                // to the section midpoint in exactly this case.
                const auto nextRoom = previous.room.ToSimplex();
                if( nextRoom )
                {
                    const auto borderPoints = nextRoom->NearestBorderPointsApprox(
                            { static_cast<double>( nextPoint.x ),
                              static_cast<double>( nextPoint.y ) }, 2 );
                    if( borderPoints.size() >= 2
                        && borderPoints[1].Distance(
                                   { static_cast<double>( nextPoint.x ),
                                     static_cast<double>( nextPoint.y ) } )
                                   < aCompensatedTraceHalfWidth
                                             + aTraceWidthTolerance )
                    {
                        nextPoint = previous.section.Middle().Round();
                    }
                }
            }
        }

        const FLOAT_POINT from{ static_cast<double>( reversePoints.back().x ),
                                static_cast<double>( reversePoints.back().y ) };
        const FLOAT_POINT to{ static_cast<double>( nextPoint.x ),
                              static_cast<double>( nextPoint.y ) };
        const bool horizontalFirst = toDoor
                ? horizontalFirstToDoor( *toDoor, from, to ) : true;
        const PLANAR::INT_OCTAGON* requiredRoom = reverseIndex == 0
                ? &shrunkenRoom : nullptr;
        if( !appendFortyFiveDegreeMove( reversePoints, nextPoint,
                                        horizontalFirst, requiredRoom ) )
            return fail( "leave_room", reverseIndex );

        const ROUTER_BOX roomBounds = step.room.BoundingBox();
        const ROUTER_BOX shrunkenBounds = shrunkenRoom.BoundingBox();
        autorouterDecisionLog(
                "LOCATOR_45_STEP",
                { { "reverse_index", std::to_string( reverseIndex ) },
                  { "room_bounds", autorouterDecisionBounds( roomBounds ) },
                  { "shrunken_bounds", autorouterDecisionBounds( shrunkenBounds ) },
                  { "entered_room_at", enteredRoomAt
                                                   ? std::to_string( enteredRoomAt->x ) + ','
                                                             + std::to_string( enteredRoomAt->y )
                                                   : "" },
                  { "next_point", std::to_string( nextPoint.x ) + ','
                                          + std::to_string( nextPoint.y ) },
                  { "to_door_bounds", toDoor
                                                  ? autorouterDecisionBounds(
                                                            toDoor->BoundingBox() )
                                                  : "" },
                  { "to_section",
                    reverseIndex > 0
                            ? std::to_string( aSteps[reverseIndex - 1].section.a.x ) + ','
                                      + std::to_string(
                                                aSteps[reverseIndex - 1].section.a.y ) + ".."
                                      + std::to_string(
                                                aSteps[reverseIndex - 1].section.b.x ) + ','
                                      + std::to_string(
                                                aSteps[reverseIndex - 1].section.b.y )
                            : "" },
                  { "points", std::to_string( reversePoints.size() ) } } );
    }

    std::reverse( reversePoints.begin(), reversePoints.end() );

    // Rounding can create repeated points while reversing the source chain.
    // Do not remove merely collinear door points: Freerouting retains those
    // boundaries in the located connection, and a later one-coordinate jog
    // can make an apparently redundant point topologically significant.
    std::vector<ROUTER_POINT> result;
    for( const ROUTER_POINT& point : reversePoints )
    {
        if( !result.empty() && result.back() == point )
            continue;
        result.push_back( point );
    }
    return result;
}
} // namespace KICAD_AUTOROUTER
