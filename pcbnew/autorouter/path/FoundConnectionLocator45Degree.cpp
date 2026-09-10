/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Freerouting a11c0a42 (GPL-3.0): FoundConnectionLocator.calculateAdditionalCorner,
 * FoundConnectionLocator45Degree.calculateNextTraceCorners (rectangular subset).
 */
#include "FoundConnectionLocator45Degree.h"

namespace KICAD_AUTOROUTER
{
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
        ROUTER_POINT aStart, const std::vector<OCTAGONAL_CORRIDOR_STEP>& aSteps )
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

        bool horizontalFirst = std::abs( dx ) >= std::abs( dy );
        const FLOAT_POINT roundedFrom{ static_cast<double>( from.x ),
                                       static_cast<double>( from.y ) };
        const FLOAT_POINT roundedTo{ static_cast<double>( target.x ),
                                     static_cast<double>( target.y ) };
        ROUTER_POINT corner = CalculateAdditionalCorner(
                roundedFrom, roundedTo, horizontalFirst, false ).Round();
        if( !step.room.Contains( corner ) )
        {
            corner = CalculateAdditionalCorner(
                    roundedFrom, roundedTo, !horizontalFirst, false ).Round();
        }
        if( !step.room.Contains( corner ) )
            return std::nullopt;

        for( const ROUTER_POINT& point : { corner, target } )
        {
            if( points.back() == point )
                continue;
            if( points.size() > 1 )
            {
                const ROUTER_POINT a = points[points.size() - 2];
                const ROUTER_POINT b = points.back();
                const long double dx1 = b.x - a.x;
                const long double dy1 = b.y - a.y;
                const long double dx2 = point.x - b.x;
                const long double dy2 = point.y - b.y;
                if( dx1 * dy2 == dy1 * dx2 && dx1 * dx2 + dy1 * dy2 >= 0 )
                    points.pop_back();
            }
            points.push_back( point );
        }
    }
    return points;
}
} // namespace KICAD_AUTOROUTER
