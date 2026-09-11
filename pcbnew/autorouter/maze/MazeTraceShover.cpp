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
#include "../expansion/ExpansionDoor.h"
#include "../expansion/ObstacleExpansionRoom.h"


namespace KICAD_AUTOROUTER
{

namespace
{

FLOAT_POINT asFloat( ROUTER_POINT aPoint )
{
    return { static_cast<double>( aPoint.x ), static_cast<double>( aPoint.y ) };
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
    const auto polar = doorShape.PolarLineSegment( fromPoint );
    const auto diagonal = doorShape.DiagonalCornerSegment();
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
    const auto diagonal = doorShape.DiagonalCornerSegment();
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
    const auto polar = doorShape.PolarLineSegment( { gravity.first, gravity.second } );
    if( !polar )
        return false;

    const bool swapped = polar->b.DistanceSquared( diagonal->a )
                         < polar->a.DistanceSquared( diagonal->a );
    const auto sections = aFromDoor.GetSectionSegments(
            aCompensatedTraceHalfWidth, aTraceWidthTolerance );
    if( sections.empty() || aFromSection >= sections.size() )
        return false;

    const double checkDistance = aCompensatedTraceHalfWidth + 5;
    const double checkDistanceSquared = checkDistance * checkDistance;
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
            return false;
        }

        shoveSegment = oneDimensionalShoveSegment(
                aFromDoor, *info, aObstacleRoom.GetShapeIndex(),
                aCompensatedTraceHalfWidth, aShoveToTheLeft );
        if( !shoveSegment )
            return false;
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
        if( shoveWidth <= 0 )
            return true;

        const double segmentLength = fromCorner.Distance( toCorner );
        if( std::isfinite( shoveWidth ) && shoveWidth < segmentLength )
            toCorner = fromCorner.ChangeLength( toCorner, shoveWidth );
    }

    const FLOAT_LINE shoveLine{ fromCorner, toCorner };
    const PLANAR::SIMPLEX fromDoorShape = aFromDoor.GetSimplexShape();
    const double fromDoorCompareDistance =
            aFromDoor.GetDimension() == 2 || segmentIsPoint
                    ? std::numeric_limits<double>::infinity()
                    : toCorner.DistanceSquared( fromDoorShape.CornerApprox( 0 ) );

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

        const auto polar = doorShape.PolarLineSegment( fromCorner );
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
    bool changedAny = false;
    bool changed = true;

    while( changed && aConnection.nodes.size() > 2 )
    {
        changed = false;

        for( std::size_t first = 0;
             first + 2 < aConnection.nodes.size() && !changed; ++first )
        {
            for( std::size_t last = first + 2; last < aConnection.nodes.size(); ++last )
            {
                const ROUTER_NODE& start = aConnection.nodes[first];
                const ROUTER_NODE& end = aConnection.nodes[last];
                const ROUTING_EDGE_STYLE* style = aConnection.edgeStyles.empty()
                        ? nullptr : &aConnection.edgeStyles[first];

                if( start.layer != end.layer
                    || !CanCollapseRouteEdges( aConnection, first, last - 1 )
                    || !aSearch.CanInsertSegment( aConnection.netCode, start, end, style ) )
                {
                    continue;
                }

                const double oldLength = lengthBetween( aConnection, first, last );
                const long double dx = static_cast<long double>( end.point.x ) - start.point.x;
                const long double dy = static_cast<long double>( end.point.y ) - start.point.y;
                const double directLength = std::sqrt( static_cast<double>( dx * dx + dy * dy ) );

                if( directLength + 1.0 < oldLength )
                {
                    if( !CollapseRouteNodes( aConnection, first, last ) )
                        continue;
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
