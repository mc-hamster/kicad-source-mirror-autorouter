/*
 * This program source code file is part of KiCad, a free EDA application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#include "MazeTraceShover.h"

#include <cmath>
#include <limits>

#include "MazeSearchEngine.h"
#include "../AutorouterDebug.h"
#include "../board/optimize/TraceTightener45.h"
#include "../expansion/ExpansionDoor.h"
#include "../expansion/ObstacleExpansionRoom.h"
#include "../path/FoundConnectionLocator45Degree.h"


namespace KICAD_AUTOROUTER
{

namespace
{

FLOAT_POINT asFloat( ROUTER_POINT aPoint )
{
    return { static_cast<double>( aPoint.x ), static_cast<double>( aPoint.y ) };
}


std::optional<FLOAT_LINE> sourcePolarLineSegment(
        const PLANAR::SIMPLEX& aShape, FLOAT_POINT aFromPoint )
{
    auto polar = aShape.PolarLineSegment( aFromPoint );
    if( polar )
    {
        // Freerouting routes in a y-up coordinate system while KiCad stores
        // the reflected y-down geometry.  Reflection exchanges the source's
        // left-most and right-most polar corners.  SIMPLEX intentionally
        // retains coordinate-local semantics for its geometry oracle, so
        // reverse only at this routing integration boundary.
        *polar = polar->Opposite();
    }
    return polar;
}


std::optional<FLOAT_LINE> sourceDiagonalCornerSegment(
        const PLANAR::SIMPLEX& aShape )
{
    auto diagonal = aShape.DiagonalCornerSegment();
    if( diagonal )
    {
        // One-dimensional source door shapes order their endpoints by x and
        // then by y in Freerouting's y-up board coordinates.  Reflection into
        // KiCad leaves the x ordering unchanged, but reverses the tie-break
        // for a vertical segment.  Preserve those source endpoint labels
        // because checkShoveTraceLine associates section zero with diagonal.a
        // and the final section with diagonal.b.
        if( diagonal->a.x > diagonal->b.x
            || ( diagonal->a.x == diagonal->b.x
                 && diagonal->a.y < diagonal->b.y ) )
        {
            *diagonal = diagonal->Opposite();
        }
    }
    return diagonal;
}


std::optional<FLOAT_LINE> traceSegment(
        const MAZE_TRACE_ROOM_INFO& aInfo, int aShapeIndex )
{
    if( aShapeIndex < 0
        || static_cast<std::size_t>( aShapeIndex ) < aInfo.firstShapeIndex )
    {
        return std::nullopt;
    }

    const std::size_t local = static_cast<std::size_t>( aShapeIndex )
                              - aInfo.firstShapeIndex;
    if( local + 1 >= aInfo.corners.size() )
        return std::nullopt;

    const FLOAT_LINE result{ asFloat( aInfo.corners[local] ),
                             asFloat( aInfo.corners[local + 1] ) };
    if( result.a.DistanceSquared( result.b ) < 1 )
        return std::nullopt;

    return result;
}


std::optional<FLOAT_LINE> oneDimensionalShoveSegment(
        EXPANSION_DOOR& aFromDoor, const MAZE_TRACE_ROOM_INFO& aInfo,
        int aShapeIndex, double aCompensatedTraceHalfWidth,
        bool aShoveToTheLeft )
{
    const auto obstacleSegment = traceSegment( aInfo, aShapeIndex );
    if( !obstacleSegment )
        return std::nullopt;

    EXPANSION_ROOM* obstacleRoom = nullptr;
    if( dynamic_cast<OBSTACLE_EXPANSION_ROOM*>( aFromDoor.FirstRoom() ) )
        obstacleRoom = aFromDoor.FirstRoom();
    else if( dynamic_cast<OBSTACLE_EXPANSION_ROOM*>( aFromDoor.SecondRoom() ) )
        obstacleRoom = aFromDoor.SecondRoom();
    if( !obstacleRoom )
        return std::nullopt;

    EXPANSION_ROOM* fromRoom = aFromDoor.OtherRoom( obstacleRoom );
    if( !fromRoom )
        return std::nullopt;

    const auto gravity = fromRoom->GetSimplex().CentreOfGravity();
    const FLOAT_POINT fromPoint{ gravity.first, gravity.second };
    const PLANAR::SIMPLEX doorShape = aFromDoor.GetSimplexShape();
    const auto polar = sourcePolarLineSegment( doorShape, fromPoint );
    const auto diagonal = sourceDiagonalCornerSegment( doorShape );
    if( !polar || !diagonal )
        return std::nullopt;

    const FLOAT_LINE shrunk = polar->ShrinkSegment( aCompensatedTraceHalfWidth );
    const FLOAT_POINT closingPoint = aShoveToTheLeft ? shrunk.b : shrunk.a;
    const FLOAT_POINT direction = obstacleSegment->b.Subtract( obstacleSegment->a );
    const FLOAT_LINE closingLine{
            closingPoint,
            { closingPoint.x - direction.y, closingPoint.y + direction.x } };
    const auto start = closingLine.Intersection( *obstacleSegment );
    if( !start )
        return std::nullopt;

    // Polyline.lines[shapeIndex + 1] is the source segment represented by
    // this room.  The adjacent support-line intersection is its corresponding
    // integral corner, so the native corner sequence supplies the same end.
    const int sideOfTrace = diagonal->a.SideOf(
            obstacleSegment->a, obstacleSegment->b );
    const bool towardNext = ( sideOfTrace > 0 ) == aShoveToTheLeft;
    const FLOAT_POINT end = towardNext ? obstacleSegment->b : obstacleSegment->a;
    if( start->DistanceSquared( end ) < 0.1 )
        return FLOAT_LINE{ *start, *start };

    return FLOAT_LINE{ *start, end };
}


bool sectionCanStartShove(
        EXPANSION_DOOR& aFromDoor, std::size_t aFromSection,
        const FLOAT_LINE& aShapeEntry, double aCompensatedTraceHalfWidth,
        bool aShoveToTheLeft, double aTraceWidthTolerance )
{
    const PLANAR::SIMPLEX doorShape = aFromDoor.GetSimplexShape();
    const auto diagonal = sourceDiagonalCornerSegment( doorShape );
    if( !diagonal )
        return false;

    EXPANSION_ROOM* obstacleRoom = nullptr;
    if( dynamic_cast<OBSTACLE_EXPANSION_ROOM*>( aFromDoor.FirstRoom() ) )
        obstacleRoom = aFromDoor.FirstRoom();
    else if( dynamic_cast<OBSTACLE_EXPANSION_ROOM*>( aFromDoor.SecondRoom() ) )
        obstacleRoom = aFromDoor.SecondRoom();
    if( !obstacleRoom )
        return false;

    EXPANSION_ROOM* fromRoom = aFromDoor.OtherRoom( obstacleRoom );
    if( !fromRoom )
        return false;

    const auto gravity = fromRoom->GetSimplex().CentreOfGravity();
    const auto polar = sourcePolarLineSegment(
            doorShape, { gravity.first, gravity.second } );
    if( !polar )
        return false;

    const bool swapped = polar->b.DistanceSquared( diagonal->a )
                         < polar->a.DistanceSquared( diagonal->a );
    const auto sections = aFromDoor.GetSectionSegments(
            aCompensatedTraceHalfWidth, aTraceWidthTolerance );
    if( sections.empty() || aFromSection >= sections.size() )
        return false;

    // Java's literal five is measured in Freerouting board coordinates.  The
    // native geometry is stored in KiCad IU, so scale the tolerance at this
    // adapter boundary just like TRACE_WIDTH_TOLERANCE.
    const double checkDistance = aCompensatedTraceHalfWidth
                                 + 5 * FREEROUTING_COORDINATE_UNIT_IU;
    const double checkDistanceSquared = checkDistance * checkDistance;
    autorouterDecisionLog(
            "TRACE_SHOVE_SECTION_CHECK",
            { { "side", aShoveToTheLeft ? "LEFT" : "RIGHT" },
              { "swapped", swapped ? "true" : "false" },
              { "section", std::to_string( aFromSection ) },
              { "section_count", std::to_string( sections.size() ) },
              { "diagonal",
                std::to_string( diagonal->a.x ) + ','
                        + std::to_string( diagonal->a.y ) + ','
                        + std::to_string( diagonal->b.x ) + ','
                        + std::to_string( diagonal->b.y ) },
              { "polar",
                std::to_string( polar->a.x ) + ','
                        + std::to_string( polar->a.y ) + ','
                        + std::to_string( polar->b.x ) + ','
                        + std::to_string( polar->b.y ) },
              { "entry_to_a",
                std::to_string( std::min(
                        aShapeEntry.a.DistanceSquared( diagonal->a ),
                        aShapeEntry.b.DistanceSquared( diagonal->a ) ) ) },
              { "entry_to_b",
                std::to_string( std::min(
                        aShapeEntry.a.DistanceSquared( diagonal->b ),
                        aShapeEntry.b.DistanceSquared( diagonal->b ) ) ) },
              { "limit", std::to_string( checkDistanceSquared ) } } );
    if( ( aShoveToTheLeft && !swapped )
        || ( !aShoveToTheLeft && swapped ) )
    {
        return aFromSection + 1 == sections.size()
               && ( aShapeEntry.a.DistanceSquared( diagonal->b ) <= checkDistanceSquared
                    || aShapeEntry.b.DistanceSquared( diagonal->b ) <= checkDistanceSquared );
    }

    return aFromSection == 0
           && ( aShapeEntry.a.DistanceSquared( diagonal->a ) <= checkDistanceSquared
                || aShapeEntry.b.DistanceSquared( diagonal->a ) <= checkDistanceSquared );
}


bool doorIsOnShoveSide( const FLOAT_LINE& aShoveLine,
                        const FLOAT_LINE& aDoorLine, bool aShoveToTheLeft )
{
    const int firstSide = aDoorLine.a.SideOf( aShoveLine.a, aShoveLine.b );
    const int secondSide = aDoorLine.b.SideOf( aShoveLine.a, aShoveLine.b );
    return aShoveToTheLeft ? firstSide > 0 && secondSide > 0
                           : firstSide < 0 && secondSide < 0;
}

double lengthBetween( const ROUTING_CONNECTION& aConnection, std::size_t aFirst,
                      std::size_t aLast )
{
    double length = 0.0;

    for( std::size_t index = aFirst + 1; index <= aLast; ++index )
    {
        const long double dx = static_cast<long double>(
                                        aConnection.nodes[index].point.x )
                                - aConnection.nodes[index - 1].point.x;
        const long double dy = static_cast<long double>(
                                        aConnection.nodes[index].point.y )
                                - aConnection.nodes[index - 1].point.y;
        length += std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
    }

    return length;
}


bool isOrthogonalEdge( ROUTER_POINT aStart, ROUTER_POINT aEnd )
{
    return aStart.x == aEnd.x || aStart.y == aEnd.y;
}


bool isFortyFiveDegreeEdge( ROUTER_POINT aStart, ROUTER_POINT aEnd )
{
    const std::int64_t dx = std::abs( aEnd.x - aStart.x );
    const std::int64_t dy = std::abs( aEnd.y - aStart.y );

    // Exact rational support-line intersections can round to adjacent KiCad
    // integer coordinates.  Freerouting still treats that one-unit residue
    // as fixed 45-degree geometry; classifying it as any-angle here lets the
    // fallback visibility simplifier replace a valid neckdown by an oblique
    // chord that the source could never create.
    return dx == 0 || dy == 0 || std::abs( dx - dy ) <= 1;
}


bool replaceSpanWithCorner( ROUTING_CONNECTION& aConnection,
                            std::size_t aFirst, std::size_t aLast,
                            ROUTER_POINT aCorner )
{
    if( aFirst >= aLast || aLast >= aConnection.nodes.size()
        || !CanCollapseRouteEdges( aConnection, aFirst, aLast - 1 ) )
    {
        return false;
    }

    if( aCorner == aConnection.nodes[aFirst].point
        || aCorner == aConnection.nodes[aLast].point )
    {
        return CollapseRouteNodes( aConnection, aFirst, aLast );
    }

    const ROUTING_EDGE_STYLE style = EdgeStyle( aConnection, aFirst );
    std::vector<ROUTER_NODE> nodes;
    nodes.reserve( aConnection.nodes.size() - ( aLast - aFirst ) + 2 );
    nodes.insert( nodes.end(), aConnection.nodes.begin(),
                  aConnection.nodes.begin() + static_cast<std::ptrdiff_t>( aFirst + 1 ) );
    nodes.push_back( { aCorner, aConnection.nodes[aFirst].layer } );
    nodes.insert( nodes.end(),
                  aConnection.nodes.begin() + static_cast<std::ptrdiff_t>( aLast ),
                  aConnection.nodes.end() );

    if( !aConnection.edgeStyles.empty() )
    {
        std::vector<ROUTING_EDGE_STYLE> styles;
        styles.reserve( nodes.size() - 1 );
        styles.insert( styles.end(), aConnection.edgeStyles.begin(),
                       aConnection.edgeStyles.begin()
                               + static_cast<std::ptrdiff_t>( aFirst ) );
        styles.push_back( style );
        styles.push_back( style );
        styles.insert( styles.end(),
                       aConnection.edgeStyles.begin()
                               + static_cast<std::ptrdiff_t>( aLast ),
                       aConnection.edgeStyles.end() );
        aConnection.edgeStyles = std::move( styles );
    }

    aConnection.nodes = std::move( nodes );
    return true;
}

} // namespace


bool MAZE_TRACE_SHOVER::CheckShoveTraceLine(
        EXPANSION_DOOR& aFromDoor, std::size_t aFromSection,
        const FLOAT_LINE& aShapeEntry, OBSTACLE_EXPANSION_ROOM& aObstacleRoom,
        double aCompensatedTraceHalfWidth, bool aShoveToTheLeft,
        std::vector<MAZE_SHOVE_DOOR_SECTION>& aToDoors,
        double aTraceWidthTolerance )
{
    const std::shared_ptr<const MAZE_TRACE_ROOM_INFO>& info =
            aObstacleRoom.GetTraceInfo();
    if( !info || !info->sourceStyleMatches )
        return true;

    if( aFromDoor.GetOctagonShape().MaxWidth()
        < 2 * aCompensatedTraceHalfWidth )
    {
        return true;
    }

    const auto obstacleSegment = traceSegment( *info, aObstacleRoom.GetShapeIndex() );
    if( !obstacleSegment )
        return false;

    std::optional<FLOAT_LINE> shoveSegment;
    if( aFromDoor.GetDimension() == 2 )
    {
        EXPANSION_ROOM* other = aFromDoor.OtherRoom( &aObstacleRoom );
        const auto* otherObstacle = dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>( other );
        if( !otherObstacle || otherObstacle->GetGroup() != aObstacleRoom.GetGroup() )
            return false;

        const auto gravity = aFromDoor.GetSimplexShape().CentreOfGravity();
        const FLOAT_POINT doorCentre{ gravity.first, gravity.second };
        shoveSegment = doorCentre.DistanceSquared( obstacleSegment->b )
                               < doorCentre.DistanceSquared( obstacleSegment->a )
                               ? obstacleSegment->Opposite() : *obstacleSegment;
    }
    else
    {
        if( !sectionCanStartShove(
                    aFromDoor, aFromSection, aShapeEntry,
                    aCompensatedTraceHalfWidth, aShoveToTheLeft,
                    aTraceWidthTolerance ) )
        {
            autorouterDecisionLog(
                    "TRACE_SHOVE_REJECT",
                    { { "reason", "section_start" },
                      { "side", aShoveToTheLeft ? "LEFT" : "RIGHT" },
                      { "door_id", std::to_string( aFromDoor.GetId() ) },
                      { "section", std::to_string( aFromSection ) } } );
            return false;
        }

        shoveSegment = oneDimensionalShoveSegment(
                aFromDoor, *info, aObstacleRoom.GetShapeIndex(),
                aCompensatedTraceHalfWidth, aShoveToTheLeft );
        if( !shoveSegment )
        {
            autorouterDecisionLog(
                    "TRACE_SHOVE_REJECT",
                    { { "reason", "shove_segment" },
                      { "side", aShoveToTheLeft ? "LEFT" : "RIGHT" },
                      { "door_id", std::to_string( aFromDoor.GetId() ) },
                      { "section", std::to_string( aFromSection ) } } );
            return false;
        }
    }

    const FLOAT_POINT fromCorner = shoveSegment->a;
    FLOAT_POINT toCorner = shoveSegment->b;
    const bool segmentIsPoint = fromCorner.DistanceSquared( toCorner ) < 0.1;
    double shoveWidth = std::numeric_limits<double>::infinity();
    if( !segmentIsPoint )
    {
        if( !info->maxShoveLength )
            return true;
        shoveWidth = info->maxShoveLength( *shoveSegment, aShoveToTheLeft );
        autorouterDecisionLog(
                "TRACE_SHOVE_SEGMENT",
                { { "side", aShoveToTheLeft ? "LEFT" : "RIGHT" },
                  { "segment",
                    std::to_string( shoveSegment->a.x ) + ','
                            + std::to_string( shoveSegment->a.y ) + ','
                            + std::to_string( shoveSegment->b.x ) + ','
                            + std::to_string( shoveSegment->b.y ) },
                  { "shove_width", std::to_string( shoveWidth ) } } );
        if( shoveWidth <= 0 )
            return true;

        const double segmentLength = fromCorner.Distance( toCorner );
        if( std::isfinite( shoveWidth ) && shoveWidth < segmentLength )
            toCorner = fromCorner.ChangeLength( toCorner, shoveWidth );
    }

    const FLOAT_LINE shoveLine{ fromCorner, toCorner };
    const PLANAR::SIMPLEX fromDoorShape = aFromDoor.GetSimplexShape();
    const auto fromDoorDiagonal = sourceDiagonalCornerSegment( fromDoorShape );
    const double fromDoorCompareDistance =
            aFromDoor.GetDimension() == 2 || segmentIsPoint
                    ? std::numeric_limits<double>::infinity()
                    // A one-dimensional door's source corner zero is the
                    // first diagonal endpoint.  Reflection into KiCad's
                    // y-down coordinates reverses that endpoint label; using
                    // the coordinate-local CornerApprox( 0 ) truncates a
                    // shove exactly at the first forward door.
                    : fromDoorDiagonal
                              ? toCorner.DistanceSquared( fromDoorDiagonal->a )
                              : std::numeric_limits<double>::infinity();

    for( EXPANSION_DOOR* door : aObstacleRoom.GetDoors() )
    {
        if( !door || door == &aFromDoor )
            continue;

        const auto* firstObstacle = dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>(
                door->FirstRoom() );
        const auto* secondObstacle = dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>(
                door->SecondRoom() );
        if( firstObstacle && secondObstacle
            && firstObstacle->GetGroup() != secondObstacle->GetGroup() )
        {
            continue;
        }

        const PLANAR::SIMPLEX doorShape = door->GetSimplexShape();
        if( door->GetDimension() == 2 && std::isinf( shoveWidth ) )
        {
            if( doorShape.Contains( toCorner ) )
            {
                const auto sections = door->GetSectionSegments(
                        aCompensatedTraceHalfWidth, aTraceWidthTolerance );
                if( !sections.empty() )
                    aToDoors.push_back( { door, 0, sections.front() } );
            }
            continue;
        }

        if( segmentIsPoint )
            continue;

        const auto doorLine = doorShape.DiagonalCornerSegment();
        if( !doorLine || !doorIsOnShoveSide(
                    shoveLine, *doorLine, aShoveToTheLeft ) )
        {
            continue;
        }

        const auto polar = sourcePolarLineSegment( doorShape, fromCorner );
        if( !polar )
            continue;
        const FLOAT_POINT nearest =
                polar->a.DistanceSquared( fromCorner )
                                <= polar->b.DistanceSquared( fromCorner )
                        ? polar->a : polar->b;
        if( toCorner.DistanceSquared( nearest ) >= fromDoorCompareDistance )
            continue;

        const FLOAT_POINT projection = shoveLine.PerpendicularProjection( nearest );
        if( projection.Distance( fromCorner ) + aCompensatedTraceHalfWidth > shoveWidth )
            continue;

        const auto sections = door->GetSectionSegments(
                aCompensatedTraceHalfWidth, aTraceWidthTolerance );
        for( std::size_t section = 0; section < sections.size(); ++section )
        {
            const FLOAT_LINE& line = sections[section];
            const FLOAT_POINT sectionNearest =
                    line.a.DistanceSquared( fromCorner )
                                    <= line.b.DistanceSquared( fromCorner )
                            ? line.a : line.b;
            const FLOAT_POINT sectionProjection =
                    shoveLine.PerpendicularProjection( sectionNearest );
            if( sectionProjection.Distance( fromCorner ) <= shoveWidth )
                aToDoors.push_back( { door, section, line } );
        }
    }

    return true;
}


bool MAZE_TRACE_SHOVER::Shorten( ROUTING_CONNECTION& aConnection,
                                 const MAZE_SEARCH_ENGINE& aSearch )
{
    bool changedAny = TRACE_TIGHTENER_45::PullTight( aConnection, aSearch );

    bool fixedDirection = true;
    for( std::size_t edge = 1; edge < aConnection.nodes.size(); ++edge )
    {
        if( aConnection.nodes[edge - 1].layer == aConnection.nodes[edge].layer
            && !isFortyFiveDegreeEdge( aConnection.nodes[edge - 1].point,
                                       aConnection.nodes[edge].point ) )
        {
            fixedDirection = false;
            break;
        }
    }

    // TraceTightener45 is authoritative for fixed-direction copper.  Retain
    // the older visibility simplifier only for an any-angle connection; it
    // cannot reproduce the support-line ordering used by the source.
    bool changed = true;

    while( !fixedDirection && changed && aConnection.nodes.size() > 2 )
    {
        changed = false;

        for( std::size_t first = 0;
             first + 2 < aConnection.nodes.size() && !changed; ++first )
        {
            for( std::size_t last = first + 2; last < aConnection.nodes.size(); ++last )
            {
                const ROUTER_NODE& start = aConnection.nodes[first];
                const ROUTER_NODE& end = aConnection.nodes[last];

                if( start.layer != end.layer
                    || !CanCollapseRouteEdges( aConnection, first, last - 1 ) )
                {
                    continue;
                }

                const double oldLength = lengthBetween( aConnection, first, last );
                const long double dx = static_cast<long double>( end.point.x ) - start.point.x;
                const long double dy = static_cast<long double>( end.point.y ) - start.point.y;
                const double directLength = std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
                const __int128 exactDx = static_cast<__int128>( end.point.x ) - start.point.x;
                const __int128 exactDy = static_cast<__int128>( end.point.y ) - start.point.y;
                const __int128 exactLengthSquared = exactDx * exactDx + exactDy * exactDy;
                bool straightRun = true;
                for( std::size_t node = first + 1; node < last; ++node )
                {
                    const __int128 pointDx = static_cast<__int128>(
                                                        aConnection.nodes[node].point.x )
                                                   - start.point.x;
                    const __int128 pointDy = static_cast<__int128>(
                                                        aConnection.nodes[node].point.y )
                                                   - start.point.y;
                    const __int128 dot = pointDx * exactDx + pointDy * exactDy;
                    straightRun = straightRun
                                  && pointDx * exactDy == pointDy * exactDx
                                  && dot >= 0 && dot <= exactLengthSquared;
                }

                bool sourceOrthogonal = true;
                bool sourceFortyFive = true;
                for( std::size_t edge = first + 1; edge <= last; ++edge )
                {
                    sourceOrthogonal = sourceOrthogonal
                                       && isOrthogonalEdge(
                                               aConnection.nodes[edge - 1].point,
                                               aConnection.nodes[edge].point );
                    sourceFortyFive = sourceFortyFive
                                      && isFortyFiveDegreeEdge(
                                              aConnection.nodes[edge - 1].point,
                                              aConnection.nodes[edge].point );
                }

                // TraceTightener90/45 repositions support lines; neither can
                // replace a fixed-direction trace by an arbitrary-angle chord.
                // The previous native line-of-sight simplifier did exactly
                // that, producing geometry which the next maze item could
                // never see in the pinned source.  Preserve the observed
                // source direction family.  Any-angle input retains the old
                // direct shortcut.
                if( ( sourceOrthogonal && isOrthogonalEdge( start.point, end.point ) )
                    || ( !sourceOrthogonal && sourceFortyFive
                         && isFortyFiveDegreeEdge( start.point, end.point ) )
                    || !sourceFortyFive )
                {
                    // PolylineTrace.normalize removes forward collinear
                    // corners even when Euclidean length is unchanged.  This
                    // is essential after an aggregate terminal span: the raw
                    // penultimate locator corner may be inside the plated
                    // hole, while the normalized edge ends at its centre.
                    ROUTING_CONNECTION candidate = aConnection;
                    if( ( straightRun || directLength + 1.0 < oldLength )
                        && CollapseRouteNodes( candidate, first, last )
                        && aSearch.CanInsertTraceSpan( candidate ) )
                    {
                        aConnection = std::move( candidate );
                        changed = true;
                        changedAny = true;
                        break;
                    }
                    continue;
                }

                const auto tryFixedDirectionCorner = [&]( bool aHorizontalFirst )
                {
                    const FLOAT_POINT cornerFloat =
                            FOUND_CONNECTION_LOCATOR_45_DEGREE::CalculateAdditionalCorner(
                                    asFloat( start.point ), asFloat( end.point ),
                                    aHorizontalFirst, sourceOrthogonal );
                    const ROUTER_POINT corner = cornerFloat.Round();
                    const long double firstDx = static_cast<long double>( corner.x )
                                                - start.point.x;
                    const long double firstDy = static_cast<long double>( corner.y )
                                                - start.point.y;
                    const long double secondDx = static_cast<long double>( end.point.x )
                                                 - corner.x;
                    const long double secondDy = static_cast<long double>( end.point.y )
                                                 - corner.y;
                    const double replacementLength =
                            std::sqrt( static_cast<double>( firstDx * firstDx
                                                            + firstDy * firstDy ) )
                            + std::sqrt( static_cast<double>( secondDx * secondDx
                                                              + secondDy * secondDy ) );
                    const bool sameSingleCorner = last == first + 2
                                                  && aConnection.nodes[first + 1].point
                                                             == corner;
                    // Source pull-tight may choose either fixed-direction
                    // corner, but only when it shortens the route.  Allowing
                    // equal-length exchanges makes the two alternatives
                    // oscillate forever.
                    if( sameSingleCorner || replacementLength + 1.0 >= oldLength )
                    {
                        return false;
                    }

                    ROUTING_CONNECTION candidate = aConnection;
                    if( !replaceSpanWithCorner(
                                candidate, first, last, corner )
                        || !aSearch.CanInsertTraceSpan( candidate ) )
                    {
                        return false;
                    }
                    aConnection = std::move( candidate );
                    return true;
                };

                if( tryFixedDirectionCorner( true )
                    || tryFixedDirectionCorner( false ) )
                {
                    changed = true;
                    changedAny = true;
                    break;
                }
            }
        }
    }

    for( std::size_t index = 1; index + 1 < aConnection.nodes.size(); )
    {
        const ROUTER_NODE& previous = aConnection.nodes[index - 1];
        const ROUTER_NODE& current = aConnection.nodes[index];
        const ROUTER_NODE& next = aConnection.nodes[index + 1];
        const ROUTING_EDGE_STYLE* style = aConnection.edgeStyles.empty()
                ? nullptr : &aConnection.edgeStyles[index - 1];

        if( previous.point == current.point && current.point == next.point
            && previous.layer != next.layer
            && CanCollapseRouteEdges( aConnection, index - 1, index )
            && aSearch.CanInsertSegment( aConnection.netCode, previous, next, style ) )
        {
            if( !CollapseRouteNodes( aConnection, index - 1, index + 1 ) )
            {
                ++index;
                continue;
            }
            changedAny = true;
            continue;
        }

        ++index;
    }

    return changedAny;
}

} // namespace KICAD_AUTOROUTER
