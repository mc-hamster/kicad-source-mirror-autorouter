/* This file is part of KiCad, licensed under GPL version 3 or later.
 * Unrestricted-angle connection realization derived from Freerouting
 * FoundConnectionLocatorAnyAngle.java at a11c0a42.
 */
#include "FoundConnectionLocatorAnyAngle.h"

#include <boost/multiprecision/cpp_int.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace KICAD_AUTOROUTER
{
namespace
{
using INTEGER = boost::multiprecision::cpp_int;

INTEGER floorDivide( INTEGER aNumerator, const INTEGER& aDenominator )
{
    INTEGER quotient = aNumerator / aDenominator;
    const INTEGER remainder = aNumerator % aDenominator;
    if( remainder != 0 && aNumerator < 0 )
        --quotient;
    return quotient;
}

INTEGER ceilDivide( const INTEGER& aNumerator, const INTEGER& aDenominator )
{
    return -floorDivide( -aNumerator, aDenominator );
}

INTEGER nearestInteger( const INTEGER& aNumerator, const INTEGER& aDenominator )
{
    INTEGER result = floorDivide( aNumerator, aDenominator );
    if( 2 * ( aNumerator - result * aDenominator ) >= aDenominator )
        ++result;
    return result;
}

bool constrainSupport( const PLANAR::LINE& aBorder, ROUTER_POINT aOrigin,
                       const INTEGER& aStepX, const INTEGER& aStepY,
                       INTEGER& aFirst, INTEGER& aLast )
{
    const INTEGER dx = aBorder.Dx();
    const INTEGER dy = aBorder.Dy();
    const INTEGER origin = dy * ( INTEGER( aOrigin.x ) - aBorder.a.x )
                           - dx * ( INTEGER( aOrigin.y ) - aBorder.a.y );
    const INTEGER step = dy * aStepX - dx * aStepY;
    if( step == 0 )
        return origin <= 0;
    if( step > 0 )
        aLast = std::min( aLast, floorDivide( -origin, step ) );
    else
        aFirst = std::max( aFirst, ceilDivide( origin, -step ) );
    return aFirst <= aLast;
}

std::optional<ROUTER_POINT> nearestOnOneDimensionalSimplex(
        const PLANAR::SIMPLEX& aShape, ROUTER_POINT aFrom )
{
    std::optional<PLANAR::LINE> support;
    const auto& borders = aShape.Borders();
    for( std::size_t first = 0; first < borders.size() && !support; ++first )
    {
        for( std::size_t second = first + 1; second < borders.size(); ++second )
        {
            if( borders[first].EqualOrOpposite( borders[second] )
                && !borders[first].SameDirection( borders[second] ) )
            {
                support = borders[first];
                break;
            }
        }
    }
    if( !support )
        return std::nullopt;

    const INTEGER divisor = PLANAR::Gcd( support->Dx(), support->Dy() );
    if( divisor == 0 )
        return std::nullopt;
    const INTEGER stepX = support->Dx() / divisor;
    const INTEGER stepY = support->Dy() / divisor;
    const INTEGER limit = std::numeric_limits<std::int64_t>::max();
    INTEGER first = -limit;
    INTEGER last = limit;
    for( const PLANAR::LINE& border : borders )
    {
        if( !constrainSupport( border, support->a, stepX, stepY, first, last ) )
            return std::nullopt;
    }

    const INTEGER denominator = stepX * stepX + stepY * stepY;
    const INTEGER numerator = ( INTEGER( aFrom.x ) - support->a.x ) * stepX
                              + ( INTEGER( aFrom.y ) - support->a.y ) * stepY;
    const INTEGER index = std::clamp(
            nearestInteger( numerator, denominator ), first, last );
    const INTEGER x = INTEGER( support->a.x ) + index * stepX;
    const INTEGER y = INTEGER( support->a.y ) + index * stepY;
    if( x < std::numeric_limits<std::int64_t>::min()
        || x > std::numeric_limits<std::int64_t>::max()
        || y < std::numeric_limits<std::int64_t>::min()
        || y > std::numeric_limits<std::int64_t>::max() )
    {
        return std::nullopt;
    }
    const ROUTER_POINT result{ x.convert_to<std::int64_t>(),
                               y.convert_to<std::int64_t>() };
    return aShape.Contains( PLANAR::POINT( result ) )
                   ? std::optional( result ) : std::nullopt;
}

void appendPoint( std::vector<ROUTER_POINT>& aPoints, ROUTER_POINT aPoint )
{
    if( aPoints.back() == aPoint )
        return;
    if( aPoints.size() > 1 )
    {
        const ROUTER_POINT a = aPoints[aPoints.size() - 2];
        const ROUTER_POINT b = aPoints.back();
        const INTEGER dx1 = b.x - a.x;
        const INTEGER dy1 = b.y - a.y;
        const INTEGER dx2 = aPoint.x - b.x;
        const INTEGER dy2 = aPoint.y - b.y;
        if( dx1 * dy2 == dy1 * dx2 && dx1 * dx2 + dy1 * dy2 >= 0 )
            aPoints.pop_back();
    }
    aPoints.push_back( aPoint );
}

struct PORTAL
{
    FLOAT_POINT left;
    FLOAT_POINT right;
    const PLANAR::SIMPLEX* shape = nullptr;
};

std::optional<PORTAL> portalForStep(
        const GENERAL_CORRIDOR_STEP& aStep )
{
    if( !aStep.door || aStep.door->Dimension() < 0
        || !aStep.door->IsBounded() )
    {
        return std::nullopt;
    }

    const auto gravity = aStep.room.CentreOfGravity();
    const FLOAT_POINT pole{ gravity.first, gravity.second };
    const int left = aStep.door->IndexOfLeftMostCorner( pole );
    const int right = aStep.door->IndexOfRightMostCorner( pole );

    if( left < 0 || right < 0 )
        return std::nullopt;

    return PORTAL{ aStep.door->CornerApprox( static_cast<std::size_t>( left ) ),
                   aStep.door->CornerApprox( static_cast<std::size_t>( right ) ),
                   &*aStep.door };
}


std::optional<PORTAL> portalForShape(
        const PLANAR::SIMPLEX& aDoor, const PLANAR::SIMPLEX& aViewRoom )
{
    if( aDoor.Dimension() < 0 || !aDoor.IsBounded() )
        return std::nullopt;

    const auto gravity = aViewRoom.CentreOfGravity();
    const FLOAT_POINT pole{ gravity.first, gravity.second };
    const int left = aDoor.IndexOfLeftMostCorner( pole );
    const int right = aDoor.IndexOfRightMostCorner( pole );

    if( left < 0 || right < 0 )
        return std::nullopt;

    return PORTAL{ aDoor.CornerApprox( static_cast<std::size_t>( left ) ),
                   aDoor.CornerApprox( static_cast<std::size_t>( right ) ),
                   &aDoor };
}

std::optional<ROUTER_POINT> integralPortalCorner(
        const PORTAL& aPortal, bool aLeft )
{
    const FLOAT_POINT corner = aLeft ? aPortal.left : aPortal.right;
    return FOUND_CONNECTION_LOCATOR_ANY_ANGLE::NearestIntegralPoint(
            *aPortal.shape, corner.Round() );
}


bool samePoint( FLOAT_POINT aFirst, FLOAT_POINT aSecond )
{
    return aFirst.x == aSecond.x && aFirst.y == aSecond.y;
}


FLOAT_POINT rightTurnNextCorner( FLOAT_POINT aFrom, double aDistance,
                                 FLOAT_POINT aTo, FLOAT_POINT aNext )
{
    const auto firstTangent = aFrom.LeftTangentialPoint( aTo, aDistance );
    if( !firstTangent )
        return aFrom;

    const auto secondTangent = aTo.RightTangentialPoint(
            aNext, 2 * aDistance + 1.0 );
    if( !secondTangent )
        return aFrom;

    const FLOAT_LINE firstLine{ aFrom, *firstTangent };
    const FLOAT_LINE secondLine = FLOAT_LINE{ aTo, *secondTangent }.Translate(
            aDistance );
    return firstLine.Intersection( secondLine ).value_or( aFrom );
}


FLOAT_POINT leftTurnNextCorner( FLOAT_POINT aFrom, double aDistance,
                                FLOAT_POINT aTo, FLOAT_POINT aNext )
{
    const auto firstTangent = aFrom.RightTangentialPoint( aTo, aDistance );
    if( !firstTangent )
        return aFrom;

    const auto secondTangent = aTo.LeftTangentialPoint(
            aNext, 2 * aDistance + 1.0 );
    if( !secondTangent )
        return aFrom;

    const FLOAT_LINE firstLine{ aFrom, *firstTangent };
    const FLOAT_LINE secondLine = FLOAT_LINE{ aTo, *secondTangent }.Translate(
            -aDistance );
    return firstLine.Intersection( secondLine ).value_or( aFrom );
}


std::optional<FLOAT_POINT> rightLeftTangentialPoint(
        FLOAT_POINT aFrom, FLOAT_POINT aTo, FLOAT_POINT aCentre,
        double aDistance )
{
    const auto firstTangent = aFrom.RightTangentialPoint( aCentre, aDistance );
    const auto secondTangent = aTo.LeftTangentialPoint( aCentre, aDistance );
    if( !firstTangent || !secondTangent )
        return std::nullopt;
    return FLOAT_LINE{ aFrom, *firstTangent }.Intersection(
            FLOAT_LINE{ aTo, *secondTangent } );
}


std::optional<FLOAT_POINT> leftRightTangentialPoint(
        FLOAT_POINT aFrom, FLOAT_POINT aTo, FLOAT_POINT aCentre,
        double aDistance )
{
    const auto firstTangent = aFrom.LeftTangentialPoint( aCentre, aDistance );
    const auto secondTangent = aTo.RightTangentialPoint( aCentre, aDistance );
    if( !firstTangent || !secondTangent )
        return std::nullopt;
    return FLOAT_LINE{ aFrom, *firstTangent }.Intersection(
            FLOAT_LINE{ aTo, *secondTangent } );
}


std::optional<std::vector<ROUTER_POINT>> locateWithSourceRadius(
        ROUTER_POINT aStart, const std::vector<GENERAL_CORRIDOR_STEP>& aSteps,
        double aCompensatedTraceHalfWidth, double aTraceWidthTolerance )
{
    // FoundConnectionLocator starts at the maze destination and walks the
    // backtrack doors toward the start item.  The native corridor is stored in
    // the opposite direction, so construct the same door sequence here and
    // reverse the rounded result at the end.
    std::vector<PORTAL> doors;
    doors.reserve( aSteps.size() );
    for( std::size_t index = aSteps.size(); index-- > 0; )
    {
        if( !aSteps[index].door )
            continue;
        const PLANAR::SIMPLEX& viewRoom = index + 1 < aSteps.size()
                ? aSteps[index + 1].room : aSteps[index].room;
        const auto portal = portalForShape( *aSteps[index].door, viewRoom );
        if( !portal )
            return std::nullopt;
        doors.push_back( *portal );
    }

    FLOAT_POINT current = aSteps.back().section.Middle();
    FLOAT_POINT previous = current;
    std::vector<FLOAT_POINT> reversePoints{ current };

    // FoundConnectionLocator.adjustStartCorner(): the point chosen on the
    // destination item must first enter the room far enough for the complete
    // trace cross-section.  An empty shrink keeps the original point, exactly
    // as the source does.
    const auto destinationRoom = aSteps.back().room.Offset(
            -aCompensatedTraceHalfWidth );
    if( destinationRoom && destinationRoom->Dimension() >= 0
        && !destinationRoom->Contains( current ) )
    {
        const auto nearest = destinationRoom->NearestPoint(
                PLANAR::POINT( current.Round() ) );
        if( nearest )
        {
            const FLOAT_POINT adjusted{ nearest->X(), nearest->Y() };
            if( !samePoint( adjusted, current ) )
            {
                reversePoints.push_back( adjusted );
                previous = current;
                current = adjusted;
            }
        }
    }

    const double maximumHalfWidth = aCompensatedTraceHalfWidth
                                    + aTraceWidthTolerance;
    const double middleHalfWidth = aCompensatedTraceHalfWidth + 1.0;
    std::size_t currentDoor = 0;
    const std::size_t targetDoor = doors.size();
    const FLOAT_POINT target{ static_cast<double>( aStart.x ),
                              static_cast<double>( aStart.y ) };

    // A successful source iteration either advances the door index or emits a
    // new corner.  The guard fails closed on malformed/degenerate geometry.
    const std::size_t iterationLimit = 64 + 16 * ( doors.size() + 1 );
    for( std::size_t iteration = 0; iteration < iterationLimit; ++iteration )
    {
        if( currentDoor > targetDoor )
            break;
        if( currentDoor == targetDoor )
        {
            ++currentDoor;
            if( !samePoint( target, current ) )
            {
                reversePoints.push_back( target );
                previous = current;
                current = target;
            }
            continue;
        }

        std::optional<FLOAT_POINT> doorLeft = doors[currentDoor].left;
        std::optional<FLOAT_POINT> doorRight = doors[currentDoor].right;
        if( current.SideOf( *doorLeft, *doorRight ) != -1 )
        {
            if( current.ScalarProduct( previous, *doorLeft ) >= 0 )
                doorLeft.reset();
            if( current.ScalarProduct( previous, *doorRight ) >= 0 )
                doorRight.reset();
            if( !doorLeft && !doorRight )
            {
                ++currentDoor;
                continue;
            }
        }

        bool endOfTrace = false;
        std::optional<FLOAT_POINT> leftTangent;
        std::optional<FLOAT_POINT> rightTangent;
        std::size_t newDoor = currentDoor;
        std::size_t leftIndex = newDoor;
        std::size_t rightIndex = newDoor;
        std::size_t nextDoorIndex = currentDoor + 1;
        std::optional<FLOAT_POINT> resultCorner;
        bool passedUnexpectedDoor = false;

        for( ;; )
        {
            leftTangent = doorLeft
                    ? current.RightTangentialPoint( *doorLeft, maximumHalfWidth )
                    : std::nullopt;
            if( doorLeft && !leftTangent )
                leftTangent = doorLeft;
            rightTangent = doorRight
                    ? current.LeftTangentialPoint( *doorRight, maximumHalfWidth )
                    : std::nullopt;
            if( doorRight && !rightTangent )
                rightTangent = doorRight;

            if( leftTangent && rightTangent
                && rightTangent->SideOf( current, *leftTangent ) != -1 )
            {
                if( !doorLeft || !doorRight )
                    return std::nullopt;
                if( doorLeft->Distance( current ) <= doorRight->Distance( current ) )
                {
                    newDoor = leftIndex;
                    resultCorner = leftTurnNextCorner(
                            current, maximumHalfWidth, *doorLeft, *doorRight );
                }
                else
                {
                    newDoor = rightIndex;
                    resultCorner = rightTurnNextCorner(
                            current, maximumHalfWidth, *doorRight, *doorLeft );
                }
                break;
            }

            if( nextDoorIndex >= targetDoor )
            {
                endOfTrace = true;
                break;
            }

            std::optional<FLOAT_POINT> nextLeft = doors[nextDoorIndex].left;
            std::optional<FLOAT_POINT> nextRight = doors[nextDoorIndex].right;
            if( current.SideOf( *nextLeft, *nextRight ) != -1 )
            {
                if( !doorLeft && current.ScalarProduct( previous, *nextLeft ) >= 0 )
                    nextLeft.reset();
                if( !doorRight && current.ScalarProduct( previous, *nextRight ) >= 0 )
                    nextRight.reset();
                if( !nextLeft && !nextRight )
                {
                    newDoor = currentDoor + 1;
                    passedUnexpectedDoor = true;
                    break;
                }
            }

            if( doorLeft && doorRight && nextLeft && nextRight )
            {
                if( nextLeft->SideOf( current, *doorRight ) == -1 )
                {
                    newDoor = rightIndex + 1;
                    resultCorner = rightTurnNextCorner(
                            current, maximumHalfWidth, *doorRight, *nextLeft );
                    break;
                }
                if( nextRight->SideOf( current, *doorLeft ) == 1 )
                {
                    newDoor = leftIndex + 1;
                    resultCorner = leftTurnNextCorner(
                            current, maximumHalfWidth, *doorLeft, *nextRight );
                    break;
                }
            }

            bool smallerRight = !doorRight;
            if( doorRight && nextRight
                && nextRight->SideOf( current, *doorRight ) != -1 )
            {
                const auto tangent = current.LeftTangentialPoint(
                        *nextRight, maximumHalfWidth );
                if( tangent
                    && FLOAT_LINE{ current, *tangent }.SegmentDistance( *doorRight )
                               >= maximumHalfWidth )
                {
                    smallerRight = true;
                }
            }
            if( smallerRight )
            {
                doorRight = nextRight;
                rightIndex = nextDoorIndex;
            }

            bool smallerLeft = !doorLeft;
            if( doorLeft && nextLeft
                && nextLeft->SideOf( current, *doorLeft ) != 1 )
            {
                const auto tangent = current.RightTangentialPoint(
                        *nextLeft, maximumHalfWidth );
                if( tangent
                    && FLOAT_LINE{ current, *tangent }.SegmentDistance( *doorLeft )
                               >= maximumHalfWidth )
                {
                    smallerLeft = true;
                }
            }
            if( smallerLeft )
            {
                doorLeft = nextLeft;
                leftIndex = nextDoorIndex;
            }
            ++nextDoorIndex;
        }

        if( passedUnexpectedDoor )
        {
            currentDoor = newDoor;
            continue;
        }

        if( endOfTrace )
        {
            resultCorner = target;
            if( leftTangent
                && target.SideOf( current, *leftTangent ) == 1 && doorLeft )
            {
                newDoor = leftIndex + 1;
                const auto corner = rightLeftTangentialPoint(
                        current, target, *doorLeft, maximumHalfWidth );
                if( corner )
                {
                    resultCorner = corner;
                    endOfTrace = false;
                }
            }
            else if( rightTangent
                     && target.SideOf( current, *rightTangent ) == -1
                     && doorRight )
            {
                newDoor = rightIndex + 1;
                const auto corner = leftRightTangentialPoint(
                        current, target, *doorRight, maximumHalfWidth );
                if( corner )
                {
                    resultCorner = corner;
                    endOfTrace = false;
                }
            }
        }
        if( endOfTrace )
            newDoor = targetDoor;
        if( !resultCorner || !std::isfinite( resultCorner->x )
            || !std::isfinite( resultCorner->y ) )
        {
            return std::nullopt;
        }

        const FLOAT_LINE checkLine{ current, *resultCorner };
        const std::size_t checkFrom = currentDoor > 5 ? currentDoor - 5 : 0;
        std::optional<FLOAT_POINT> corrected;
        std::size_t correctedDoor = 0;
        for( std::size_t index = checkFrom; index < newDoor; ++index )
        {
            const FLOAT_POINT left = doors[index].left;
            if( std::abs( checkLine.SegmentDistance( left ) ) < middleHalfWidth )
            {
                const auto candidate = rightLeftTangentialPoint(
                        checkLine.a, checkLine.b, left, maximumHalfWidth );
                if( candidate
                    && ( !corrected
                         || candidate->SideOf( current, *corrected ) == -1 ) )
                {
                    correctedDoor = index;
                    corrected = candidate;
                }
            }
            const FLOAT_POINT right = doors[index].right;
            if( std::abs( checkLine.SegmentDistance( right ) ) < middleHalfWidth )
            {
                const auto candidate = leftRightTangentialPoint(
                        checkLine.a, checkLine.b, right, maximumHalfWidth );
                if( candidate
                    && ( !corrected
                         || candidate->SideOf( current, *corrected ) == 1 ) )
                {
                    correctedDoor = index;
                    corrected = candidate;
                }
            }
        }
        if( corrected )
        {
            resultCorner = corrected;
            newDoor = std::max( correctedDoor, currentDoor );
        }

        const std::size_t previousDoor = currentDoor;
        currentDoor = newDoor;
        if( !samePoint( *resultCorner, current ) )
        {
            reversePoints.push_back( *resultCorner );
            previous = current;
            current = *resultCorner;
        }
        else if( currentDoor == previousDoor )
        {
            return std::nullopt;
        }
    }

    if( currentDoor <= targetDoor || !samePoint( current, target ) )
        return std::nullopt;

    std::vector<ROUTER_POINT> rounded;
    rounded.reserve( reversePoints.size() );
    for( const FLOAT_POINT point : reversePoints )
    {
        const ROUTER_POINT integral = point.Round();
        if( rounded.empty() || rounded.back() != integral )
            rounded.push_back( integral );
    }
    std::reverse( rounded.begin(), rounded.end() );
    return rounded;
}
} // namespace


std::optional<ROUTER_POINT> FOUND_CONNECTION_LOCATOR_ANY_ANGLE::NearestIntegralPoint(
        const PLANAR::SIMPLEX& aShape, ROUTER_POINT aFrom )
{
    const int dimension = aShape.Dimension();
    if( dimension < 0 )
        return std::nullopt;
    if( aShape.Contains( PLANAR::POINT( aFrom ) ) )
        return aFrom;
    if( dimension == 0 )
    {
        if( aShape.Borders().empty() || !aShape.CornerIsBounded( 0 ) )
            return std::nullopt;
        return aShape.Corner( 0 ).Integral();
    }
    if( dimension == 1 )
        return nearestOnOneDimensionalSimplex( aShape, aFrom );

    const auto nearest = aShape.NearestPoint( PLANAR::POINT( aFrom ) );
    if( !nearest )
        return std::nullopt;
    const auto nearestBox = nearest->SurroundingBox();
    if( !nearestBox )
        return std::nullopt;

    std::optional<ROUTER_POINT> best;
    long double bestDistance = std::numeric_limits<long double>::infinity();
    for( int radius = 0; radius <= 4 && !best; ++radius )
    {
        const std::int64_t minX = nearestBox->minX - radius;
        const std::int64_t maxX = nearestBox->maxX + radius;
        const std::int64_t minY = nearestBox->minY - radius;
        const std::int64_t maxY = nearestBox->maxY + radius;
        for( std::int64_t x = minX; x <= maxX; ++x )
        {
            for( std::int64_t y = minY; y <= maxY; ++y )
            {
                if( radius > 0 && x > minX && x < maxX && y > minY && y < maxY )
                    continue;
                const ROUTER_POINT candidate{ x, y };
                if( !aShape.Contains( PLANAR::POINT( candidate ) ) )
                    continue;
                const long double dx = static_cast<long double>( x ) - aFrom.x;
                const long double dy = static_cast<long double>( y ) - aFrom.y;
                const long double distance = dx * dx + dy * dy;
                if( !best || distance < bestDistance
                    || ( distance == bestDistance
                         && std::tie( x, y ) < std::tie( best->x, best->y ) ) )
                {
                    best = candidate;
                    bestDistance = distance;
                }
            }
        }
    }
    return best;
}


std::optional<std::vector<ROUTER_POINT>> FOUND_CONNECTION_LOCATOR_ANY_ANGLE::Locate(
        ROUTER_POINT aStart, const std::vector<GENERAL_CORRIDOR_STEP>& aSteps,
        double aCompensatedTraceHalfWidth, double aTraceWidthTolerance )
{
    if( aSteps.empty() )
        return std::vector<ROUTER_POINT>{ aStart };

    if( aCompensatedTraceHalfWidth > 0 )
    {
        if( !std::isfinite( aCompensatedTraceHalfWidth )
            || !std::isfinite( aTraceWidthTolerance )
            || aTraceWidthTolerance < 0 )
        {
            return std::nullopt;
        }
        return locateWithSourceRadius( aStart, aSteps,
                                       aCompensatedTraceHalfWidth,
                                       aTraceWidthTolerance );
    }

    // The Java locator computes a maximum visible range through successive
    // doors and emits a bend only when the next door closes that range.  Its
    // tangent-circle radius is the compensated trace half-width.  Native room
    // search has already moved that radius into the obstacle/room boundaries,
    // so the equivalent operation here is the zero-radius portal funnel.  The
    // left/right tests and constraining-door indices deliberately retain the
    // source control flow and tie directions.
    std::vector<PORTAL> portals;
    portals.reserve( aSteps.size() );
    for( std::size_t index = 0; index + 1 < aSteps.size(); ++index )
    {
        const auto portal = portalForStep( aSteps[index] );
        if( !portal )
            return std::nullopt;
        portals.push_back( *portal );
    }

    const GENERAL_CORRIDOR_STEP& targetStep = aSteps.back();
    if( targetStep.door )
    {
        const auto portal = portalForStep( targetStep );
        if( !portal )
            return std::nullopt;
        portals.push_back( *portal );
    }

    const ROUTER_POINT requestedTarget = targetStep.section.Middle().Round();
    const std::optional<ROUTER_POINT> target =
            targetStep.room.Contains( PLANAR::POINT( requestedTarget ) )
                    ? std::optional( requestedTarget )
                    : NearestIntegralPoint( targetStep.room, requestedTarget );
    if( !target )
        return std::nullopt;

    std::vector<ROUTER_POINT> points{ aStart };
    std::size_t firstPortal = 0;

    while( firstPortal < portals.size()
           && portals[firstPortal].shape->Contains( PLANAR::POINT( points.back() ) ) )
    {
        ++firstPortal;
    }

    while( firstPortal < portals.size() )
    {
        const FLOAT_POINT from{ static_cast<double>( points.back().x ),
                                static_cast<double>( points.back().y ) };
        FLOAT_POINT left = portals[firstPortal].left;
        FLOAT_POINT right = portals[firstPortal].right;
        std::size_t leftIndex = firstPortal;
        std::size_t rightIndex = firstPortal;
        bool restarted = false;

        for( std::size_t index = firstPortal + 1; index <= portals.size(); ++index )
        {
            const FLOAT_POINT nextLeft = index < portals.size()
                    ? portals[index].left
                    : FLOAT_POINT{ static_cast<double>( target->x ),
                                   static_cast<double>( target->y ) };
            const FLOAT_POINT nextRight = index < portals.size()
                    ? portals[index].right : nextLeft;

            // The next left boundary crossed to the right of the current
            // right boundary: the current right constraining corner is the
            // next source bend.
            if( nextLeft.SideOf( from, right ) < 0 )
            {
                const auto corner = integralPortalCorner(
                        portals[rightIndex], false );
                if( !corner )
                    return std::nullopt;
                appendPoint( points, *corner );
                firstPortal = rightIndex + 1;
                restarted = true;
                break;
            }

            // Symmetric closure of the left side of the visibility range.
            if( nextRight.SideOf( from, left ) > 0 )
            {
                const auto corner = integralPortalCorner(
                        portals[leftIndex], true );
                if( !corner )
                    return std::nullopt;
                appendPoint( points, *corner );
                firstPortal = leftIndex + 1;
                restarted = true;
                break;
            }

            if( nextRight.SideOf( from, right ) >= 0 )
            {
                right = nextRight;
                rightIndex = index;
            }

            if( nextLeft.SideOf( from, left ) <= 0 )
            {
                left = nextLeft;
                leftIndex = index;
            }
        }

        if( !restarted )
            break;
    }

    appendPoint( points, *target );

    // Every emitted constraining corner must remain an exact lattice point of
    // its door.  The final host CanUseSegment validation remains authoritative
    // for copper/rule legality across decomposition-only room boundaries.
    for( std::size_t index = 1; index + 1 < points.size(); ++index )
    {
        if( std::none_of( portals.begin(), portals.end(), [&]( const PORTAL& aPortal )
            {
                return aPortal.shape->Contains( PLANAR::POINT( points[index] ) );
            } ) )
        {
            return std::nullopt;
        }
    }
    return points;
}

} // namespace KICAD_AUTOROUTER
