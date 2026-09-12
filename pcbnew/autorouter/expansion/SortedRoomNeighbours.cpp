/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct translation of Freerouting SortedRoomNeighbours.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#include "SortedRoomNeighbours.h"

#include "ExpansionRoom.h"
#include "ObstacleExpansionRoom.h"
#include "../maze/MazeTraceShover.h"

#include <algorithm>
#include <cmath>

namespace KICAD_AUTOROUTER
{
using PLANAR::LINE;
using PLANAR::POINT;
using PLANAR::SIMPLEX;

namespace
{
int signum( double aValue )
{
    return aValue > 0 ? 1 : aValue < 0 ? -1 : 0;
}


std::optional<LINE> roundedLine( const POINT& aFirst, const POINT& aSecond )
{
    const ROUTER_POINT first = FLOAT_POINT{ aFirst.X(), aFirst.Y() }.Round();
    const ROUTER_POINT second = FLOAT_POINT{ aSecond.X(), aSecond.Y() }.Round();
    if( first == second )
        return std::nullopt;
    return LINE( first, second );
}


std::optional<LINE> firstDoorLine( const ROUTER_BOX& aShape )
{
    if( INT_BOX::Dimension( aShape ) != 1 )
        return std::nullopt;

    if( aShape.minX != aShape.maxX )
        return LINE( { aShape.minX, aShape.minY },
                     { aShape.maxX, aShape.minY } );
    if( aShape.minY != aShape.maxY )
        return LINE( { aShape.minX, aShape.minY },
                     { aShape.minX, aShape.maxY } );
    return std::nullopt;
}


std::optional<LINE> firstDoorLine( const PLANAR::INT_OCTAGON& aShape )
{
    if( aShape.Dimension() != 1 )
        return std::nullopt;

    const ROUTER_POINT first = aShape.Corner( 0 );
    for( int index = 1; index < 8; ++index )
    {
        if( aShape.Corner( index ) != first )
            return aShape.BorderLine( index - 1 );
    }
    return std::nullopt;
}


std::optional<LINE> firstDoorLine( const SIMPLEX& aShape )
{
    if( aShape.Dimension() != 1 || aShape.Borders().empty()
        || !aShape.CornerIsBounded( 0 ) )
    {
        return std::nullopt;
    }

    const POINT& first = aShape.Corner( 0 );
    for( std::size_t index = 1; index < aShape.Borders().size(); ++index )
    {
        if( aShape.CornerIsBounded( index ) && aShape.Corner( index ) != first )
            return aShape.Borders()[index - 1];
    }
    return std::nullopt;
}


bool obstacleAllowsDoor( const OBSTACLE_EXPANSION_ROOM& aRoom,
                         const std::optional<LINE>& aDoorLine )
{
    const auto& info = aRoom.GetTraceInfo();
    if( !info )
        return true;

    if( !aDoorLine )
        return false;

    // SortedRoomNeighbours.insertDoorOk() only applies the parallel-door
    // restriction when the obstacle room still names the first or last tree
    // shape of the *current* PolylineTrace.  Room indices can be stale after
    // pull-tight or shove mutation; a stale index is deliberately allowed
    // through here and MazeTraceShover then returns false so the maze delays
    // occupation and requeues the same section with paid rip-up.  Rejecting
    // stale metadata here made that source lifecycle unreachable and forced
    // every search to take a geometric detour instead.
    if( aRoom.GetShapeIndex() < 0
        || static_cast<std::size_t>( aRoom.GetShapeIndex() )
                   < info->firstShapeIndex )
    {
        return true;
    }

    const std::size_t local = static_cast<std::size_t>( aRoom.GetShapeIndex() )
                              - info->firstShapeIndex;
    if( local + 1 >= info->corners.size() )
        return true;

    // Inner trace rooms are deliberately enterable from either side.  The
    // source applies the parallel-door restriction only to the first and last
    // tree shape of a PolylineTrace item.
    if( local != 0 && local + 2 != info->corners.size() )
        return true;

    if( info->corners[local] == info->corners[local + 1] )
        return false;

    return LINE( info->corners[local], info->corners[local + 1] )
            .Parallel( *aDoorLine );
}


template <typename SHAPE>
bool insertDoorOk( EXPANSION_ROOM* aFirstRoom, EXPANSION_ROOM* aSecondRoom,
                   const SHAPE& aDoorShape )
{
    if( !aFirstRoom || !aSecondRoom || aFirstRoom->DoorExists( aSecondRoom ) )
        return false;

    auto* firstObstacle = dynamic_cast<OBSTACLE_EXPANSION_ROOM*>( aFirstRoom );
    auto* secondObstacle = dynamic_cast<OBSTACLE_EXPANSION_ROOM*>( aSecondRoom );
    if( firstObstacle && secondObstacle )
    {
        // Source Item.sharesNet().  A native obstacle group is one source
        // item; netCode extends the same rule to distinct same-net items.
        return firstObstacle->GetGroup() == secondObstacle->GetGroup()
               || ( firstObstacle->GetNetCode() != 0
                    && firstObstacle->GetNetCode()
                               == secondObstacle->GetNetCode() );
    }

    if( !firstObstacle && !secondObstacle )
        return true;

    const std::optional<LINE> doorLine = firstDoorLine( aDoorShape );
    return ( !firstObstacle || obstacleAllowsDoor( *firstObstacle, doorLine ) )
           && ( !secondObstacle || obstacleAllowsDoor( *secondObstacle, doorLine ) );
}
} // namespace


bool SORTED_ROOM_NEIGHBOURS::InsertDoorOk(
        EXPANSION_ROOM* aFirstRoom, EXPANSION_ROOM* aSecondRoom,
        const ROUTER_BOX& aDoorShape )
{
    return insertDoorOk( aFirstRoom, aSecondRoom, aDoorShape );
}


bool SORTED_ROOM_NEIGHBOURS::InsertDoorOk(
        EXPANSION_ROOM* aFirstRoom, EXPANSION_ROOM* aSecondRoom,
        const PLANAR::INT_OCTAGON& aDoorShape )
{
    return insertDoorOk( aFirstRoom, aSecondRoom, aDoorShape );
}


bool SORTED_ROOM_NEIGHBOURS::InsertDoorOk(
        EXPANSION_ROOM* aFirstRoom, EXPANSION_ROOM* aSecondRoom,
        const PLANAR::SIMPLEX& aDoorShape )
{
    return insertDoorOk( aFirstRoom, aSecondRoom, aDoorShape );
}


const POINT& SORTED_ROOM_NEIGHBOURS::NEIGHBOUR::FirstCorner(
        const SIMPLEX& aRoom ) const
{
    if( roomTouchIsCorner )
        return aRoom.Corner( touchingSideNoOfRoom );
    if( neighbourRoomTouchIsCorner )
        return shape.Corner( touchingSideNoOfNeighbourRoom );

    const POINT& currentFirst = shape.Corner(
            shape.NextNo( touchingSideNoOfNeighbourRoom ) );
    const LINE& previousLine = aRoom.Borders()[aRoom.PrevNo(
            touchingSideNoOfRoom )];
    return previousLine.SideOf( currentFirst ) == -1
                   ? currentFirst
                   : aRoom.Corner( touchingSideNoOfRoom );
}


const POINT& SORTED_ROOM_NEIGHBOURS::NEIGHBOUR::LastCorner(
        const SIMPLEX& aRoom ) const
{
    if( roomTouchIsCorner )
        return aRoom.Corner( touchingSideNoOfRoom );
    if( neighbourRoomTouchIsCorner )
        return shape.Corner( touchingSideNoOfNeighbourRoom );

    const POINT& currentLast = shape.Corner( touchingSideNoOfNeighbourRoom );
    const LINE& nextLine = aRoom.Borders()[aRoom.NextNo(
            touchingSideNoOfRoom )];
    return nextLine.SideOf( currentLast ) == -1
                   ? currentLast
                   : aRoom.Corner( aRoom.NextNo( touchingSideNoOfRoom ) );
}


SORTED_ROOM_NEIGHBOURS::SORTED_ROOM_NEIGHBOURS(
        SIMPLEX aRoom, const std::vector<SHAPE_TREE_ENTRY>& aEntries ) :
        m_room( std::move( aRoom ) )
{
    std::vector<SHAPE_TREE_ENTRY> entries = aEntries;
    std::stable_sort( entries.begin(), entries.end(),
                      []( const SHAPE_TREE_ENTRY& aLeft,
                          const SHAPE_TREE_ENTRY& aRight )
                      {
                          if( aLeft.objectId != aRight.objectId )
                              return aLeft.objectId < aRight.objectId;
                          return aLeft.shapeIndex < aRight.shapeIndex;
                      } );

    for( const SHAPE_TREE_ENTRY& entry : entries )
    {
        const SIMPLEX shape = entry.BoundingSimplex();
        // TileShape.intersection double-dispatches to the argument.  With
        // two Simplex values the neighbour supports therefore precede the
        // room supports before stable equal-direction normalization.
        const SIMPLEX intersection = shape.Intersection( m_room );
        const int dimension = intersection.Dimension();
        if( dimension < 0 || dimension > 1 )
            continue;
        addNeighbour( entry, shape, intersection );
    }

    std::stable_sort( m_neighbours.begin(), m_neighbours.end(),
                      [&]( const NEIGHBOUR& aLeft, const NEIGHBOUR& aRight )
                      {
                          return compare( aLeft, aRight ) < 0;
                      } );
    m_neighbours.erase(
            std::unique( m_neighbours.begin(), m_neighbours.end(),
                         [&]( const NEIGHBOUR& aLeft, const NEIGHBOUR& aRight )
                         {
                             return compare( aLeft, aRight ) == 0;
                         } ),
            m_neighbours.end() );
}


void SORTED_ROOM_NEIGHBOURS::addNeighbour(
        const SHAPE_TREE_ENTRY& aEntry, const SIMPLEX& aShape,
        const SIMPLEX& aIntersection )
{
    NEIGHBOUR neighbour{ aEntry, aShape, aIntersection };
    if( aIntersection.Dimension() == 1 )
    {
        const std::vector<int> sides = m_room.TouchingSides( aShape );
        if( sides.size() != 2 )
            return;
        neighbour.touchingSideNoOfRoom = sides[0];
        neighbour.touchingSideNoOfNeighbourRoom = sides[1];
    }
    else
    {
        if( aIntersection.Borders().empty()
            || !aIntersection.CornerIsBounded( 0 ) )
        {
            return;
        }
        const POINT& touchingPoint = aIntersection.Corner( 0 );
        const int roomCorner = m_room.EqualsCorner( touchingPoint );
        if( roomCorner >= 0 )
        {
            neighbour.roomTouchIsCorner = true;
            neighbour.touchingSideNoOfRoom = roomCorner;
        }
        else
        {
            neighbour.touchingSideNoOfRoom =
                    m_room.ContainsOnBorderLineNo( touchingPoint );
        }

        const int neighbourCorner = aShape.EqualsCorner( touchingPoint );
        if( neighbourCorner >= 0 )
        {
            neighbour.neighbourRoomTouchIsCorner = true;
            neighbour.touchingSideNoOfNeighbourRoom = static_cast<int>(
                    aShape.PrevNo( neighbourCorner ) );
        }
        else
        {
            neighbour.touchingSideNoOfNeighbourRoom =
                    aShape.ContainsOnBorderLineNo( touchingPoint );
        }
    }

    if( neighbour.touchingSideNoOfRoom < 0
        || neighbour.touchingSideNoOfNeighbourRoom < 0 )
    {
        return;
    }
    m_neighbours.push_back( std::move( neighbour ) );
}


int SORTED_ROOM_NEIGHBOURS::compareDirectionsFrom(
        const LINE& aBase, const LINE& aFirst, const LINE& aSecond )
{
    if( aFirst.CompareDirection( aBase ) >= 0 )
    {
        return aSecond.CompareDirection( aBase ) >= 0
                       ? aFirst.CompareDirection( aSecond )
                       : -1;
    }
    return aSecond.CompareDirection( aBase ) >= 0
                   ? 1
                   : aFirst.CompareDirection( aSecond );
}


int SORTED_ROOM_NEIGHBOURS::compare(
        const NEIGHBOUR& aLeft, const NEIGHBOUR& aRight ) const
{
    if( aLeft.touchingSideNoOfRoom != aRight.touchingSideNoOfRoom )
    {
        return aLeft.touchingSideNoOfRoom < aRight.touchingSideNoOfRoom
                       ? -1 : 1;
    }

    constexpr double tolerance = 1;
    const FLOAT_POINT compareCorner = m_room.CornerApprox(
            aLeft.touchingSideNoOfRoom );
    auto cornerDistance = [&]( const POINT& aPoint )
    {
        return FLOAT_POINT{ aPoint.X(), aPoint.Y() }.Distance( compareCorner );
    };
    double delta = cornerDistance( aLeft.FirstCorner( m_room ) )
                   - cornerDistance( aRight.FirstCorner( m_room ) );
    if( std::abs( delta ) <= tolerance
        && aLeft.FirstCorner( m_room ) == aRight.FirstCorner( m_room ) )
    {
        delta = cornerDistance( aLeft.LastCorner( m_room ) )
                - cornerDistance( aRight.LastCorner( m_room ) );
        if( std::abs( delta ) <= tolerance
            && aLeft.neighbourRoomTouchIsCorner
            && aRight.neighbourRoomTouchIsCorner )
        {
            std::size_t compareLine = aLeft.touchingSideNoOfRoom;
            if( aLeft.roomTouchIsCorner )
                compareLine = m_room.PrevNo( compareLine );
            const LINE base = m_room.Borders()[compareLine].Opposite();
            delta = compareDirectionsFrom(
                    base,
                    aLeft.shape.Borders()[aLeft.touchingSideNoOfNeighbourRoom],
                    aRight.shape.Borders()[aRight.touchingSideNoOfNeighbourRoom] );
        }
    }

    int result = signum( delta );
    if( result == 0 && aLeft.entry.objectId != aRight.entry.objectId )
        result = aLeft.entry.objectId < aRight.entry.objectId ? -1 : 1;
    return result;
}


int SORTED_ROOM_NEIGHBOURS::FirstUnrestrainedSide() const
{
    int previousEdge = -1;
    int currentEdge = 0;
    for( const NEIGHBOUR& neighbour : m_neighbours )
    {
        if( neighbour.touchingSideNoOfRoom == previousEdge )
            continue;
        if( neighbour.touchingSideNoOfRoom == currentEdge )
        {
            previousEdge = currentEdge;
            ++currentEdge;
        }
        else
        {
            return currentEdge;
        }
    }
    return currentEdge < static_cast<int>( m_room.Borders().size() )
                   ? currentEdge : -1;
}


std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM>
SORTED_ROOM_NEIGHBOURS::calculateIncompleteRooms(
        int aLayer, bool aFromIncomplete, const SIMPLEX& aContainedShape,
        SIMPLEX* aCompletedShape ) const
{
    std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> result;
    SIMPLEX completedShape = m_room;
    if( m_neighbours.empty() )
    {
        if( aCompletedShape )
            *aCompletedShape = completedShape;
        return result;
    }

    const NEIGHBOUR* previous = &m_neighbours.back();
    for( const NEIGHBOUR& next : m_neighbours )
    {
        int firstTouchingSide = previous->touchingSideNoOfRoom;
        int lastTouchingSide = next.touchingSideNoOfRoom;
        const int currentNext = static_cast<int>( m_room.NextNo(
                firstTouchingSide ) );
        const bool previousEndsAtCorner =
                ( firstTouchingSide != lastTouchingSide
                  || previous == &m_neighbours.back() )
                && previous->LastCorner( m_room ) == m_room.Corner( currentNext );
        const bool nextStartsAtCorner =
                ( firstTouchingSide != lastTouchingSide
                  || previous == &m_neighbours.back() )
                && next.FirstCorner( m_room ) == m_room.Corner(
                        lastTouchingSide );
        if( previousEndsAtCorner )
            firstTouchingSide = currentNext;
        if( nextStartsAtCorner )
            lastTouchingSide = static_cast<int>( m_room.PrevNo(
                    lastTouchingSide ) );

        const bool neighboursTouch = m_neighbours.size() > 1
                                     && previous->LastCorner( m_room )
                                                == next.FirstCorner( m_room );
        if( !neighboursTouch )
        {
            int lastBoundingLine = previous->touchingSideNoOfNeighbourRoom;
            if( !( previousEndsAtCorner || previous->roomTouchIsCorner ) )
                lastBoundingLine = static_cast<int>( previous->shape.PrevNo(
                        lastBoundingLine ) );

            int firstBoundingLine = next.touchingSideNoOfNeighbourRoom;
            if( !( nextStartsAtCorner || next.neighbourRoomTouchIsCorner ) )
                firstBoundingLine = static_cast<int>( next.shape.NextNo(
                        firstBoundingLine ) );

            std::optional<LINE> startEdgeLine =
                    next.shape.Borders()[firstBoundingLine].Opposite();
            std::optional<LINE> middleEdgeLine;
            int currentTouchingSide = lastTouchingSide;
            bool firstTime = true;

            for( ;; )
            {
                bool cornerCutOff = false;
                if( aFromIncomplete
                    && currentTouchingSide == lastTouchingSide
                    && firstTouchingSide != lastTouchingSide )
                {
                    const auto cutLine = roundedLine(
                            previous->LastCorner( m_room ),
                            next.FirstCorner( m_room ) );
                    if( cutLine )
                    {
                        completedShape = SIMPLEX::GetInstance( { *cutLine } )
                                                 .Intersection( completedShape );
                        cornerCutOff = aContainedShape.SideOf( *cutLine ) == 1;
                        if( cornerCutOff )
                            middleEdgeLine = cutLine->Opposite();
                    }
                }

                const int nextTouchingSide = static_cast<int>( m_room.PrevNo(
                        currentTouchingSide ) );
                if( !cornerCutOff )
                {
                    middleEdgeLine = m_room.Borders()[currentTouchingSide]
                                             .Opposite();
                }

                const bool lastTime =
                        ( currentTouchingSide == firstTouchingSide
                          && !( previous == &m_neighbours.back() && firstTime ) )
                        || cornerCutOff;

                std::optional<LINE> endEdgeLine;
                if( lastTime )
                {
                    endEdgeLine = previous->shape.Borders()[lastBoundingLine]
                                          .Opposite();
                    if( middleEdgeLine->DirectionDeterminant( *endEdgeLine ) <= 0 )
                        endEdgeLine.reset();
                }

                if( startEdgeLine
                    && startEdgeLine->DirectionDeterminant(
                               *middleEdgeLine ) <= 0 )
                {
                    startEdgeLine.reset();
                }

                std::vector<LINE> edgeLines;
                if( startEdgeLine )
                    edgeLines.push_back( *startEdgeLine );
                edgeLines.push_back( *middleEdgeLine );
                if( endEdgeLine )
                    edgeLines.push_back( *endEdgeLine );
                const SIMPLEX roomShape = SIMPLEX::GetInstance(
                        std::move( edgeLines ) );
                if( !roomShape.IsEmpty() )
                {
                    const SIMPLEX contained = roomShape.Intersection(
                            completedShape );
                    if( !contained.IsEmpty() )
                        result.push_back( { roomShape, aLayer, contained } );
                }

                if( lastTime )
                    break;
                currentTouchingSide = nextTouchingSide;
                startEdgeLine.reset();
                firstTime = false;
            }
        }
        previous = &next;
    }

    if( aCompletedShape )
        *aCompletedShape = completedShape;
    return result;
}


std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM>
SORTED_ROOM_NEIGHBOURS::IncompleteRooms(
        int aLayer, const SIMPLEX& aContainedShape,
        SIMPLEX* aCompletedShape ) const
{
    return calculateIncompleteRooms( aLayer, true, aContainedShape,
                                     aCompletedShape );
}


std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM>
SORTED_ROOM_NEIGHBOURS::ObstacleIncompleteRooms( int aLayer ) const
{
    if( !m_neighbours.empty() )
        return calculateIncompleteRooms( aLayer, false, m_room, nullptr );

    std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> result;
    result.reserve( m_room.Borders().size() );
    for( const LINE& border : m_room.Borders() )
    {
        const SIMPLEX shape = SIMPLEX::GetInstance( { border.Opposite() } );
        const SIMPLEX contained = shape.Intersection( m_room );
        if( !shape.IsEmpty() && !contained.IsEmpty() )
            result.push_back( { shape, aLayer, contained } );
    }
    return result;
}

} // namespace KICAD_AUTOROUTER
