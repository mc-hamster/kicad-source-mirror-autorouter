/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct translation of Freerouting Sorted45DegreeRoomNeighbours.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#include "Sorted45DegreeRoomNeighbours.h"

#include <algorithm>

namespace KICAD_AUTOROUTER
{
using PLANAR::INT_OCTAGON;

SORTED_45_DEGREE_ROOM_NEIGHBOURS::SORTED_45_DEGREE_ROOM_NEIGHBOURS(
        INT_OCTAGON aRoom, const std::vector<SHAPE_TREE_ENTRY>& aEntries ) :
        m_room( std::move( aRoom ) )
{
    for( const SHAPE_TREE_ENTRY& entry : aEntries )
    {
        const INT_OCTAGON shape = entry.BoundingOctagon();
        const INT_OCTAGON intersection = m_room.Intersection( shape );
        if( intersection.Dimension() < 0 || intersection.Dimension() > 1 )
            continue;
        addNeighbour( entry, shape, intersection );
    }

    std::stable_sort( m_neighbours.begin(), m_neighbours.end(),
                      []( const NEIGHBOUR& aLeft, const NEIGHBOUR& aRight )
                      {
                          return compare( aLeft, aRight ) < 0;
                      } );
    m_neighbours.erase(
            std::unique( m_neighbours.begin(), m_neighbours.end(),
                         []( const NEIGHBOUR& aLeft, const NEIGHBOUR& aRight )
                         {
                             return compare( aLeft, aRight ) == 0;
                         } ),
            m_neighbours.end() );
}


void SORTED_45_DEGREE_ROOM_NEIGHBOURS::addNeighbour(
        const SHAPE_TREE_ENTRY& aEntry, const INT_OCTAGON& aShape,
        const INT_OCTAGON& aIntersection )
{
    NEIGHBOUR neighbour{ aEntry, aShape, aIntersection };
    if( aIntersection.bottomY == m_room.bottomY
        && aIntersection.lowerLeftDiagonalX > m_room.lowerLeftDiagonalX )
        neighbour.firstTouchingSide = 0;
    else if( aIntersection.lowerRightDiagonalX == m_room.lowerRightDiagonalX
             && aIntersection.bottomY > m_room.bottomY )
        neighbour.firstTouchingSide = 1;
    else if( aIntersection.rightX == m_room.rightX
             && aIntersection.lowerRightDiagonalX < m_room.lowerRightDiagonalX )
        neighbour.firstTouchingSide = 2;
    else if( aIntersection.upperRightDiagonalX == m_room.upperRightDiagonalX
             && aIntersection.rightX < m_room.rightX )
        neighbour.firstTouchingSide = 3;
    else if( aIntersection.topY == m_room.topY
             && aIntersection.upperRightDiagonalX < m_room.upperRightDiagonalX )
        neighbour.firstTouchingSide = 4;
    else if( aIntersection.upperLeftDiagonalX == m_room.upperLeftDiagonalX
             && aIntersection.topY < m_room.topY )
        neighbour.firstTouchingSide = 5;
    else if( aIntersection.leftX == m_room.leftX
             && aIntersection.upperLeftDiagonalX > m_room.upperLeftDiagonalX )
        neighbour.firstTouchingSide = 6;
    else if( aIntersection.lowerLeftDiagonalX == m_room.lowerLeftDiagonalX
             && aIntersection.leftX > m_room.leftX )
        neighbour.firstTouchingSide = 7;
    else
        return;

    if( aIntersection.lowerLeftDiagonalX == m_room.lowerLeftDiagonalX
        && aIntersection.bottomY > m_room.bottomY )
        neighbour.lastTouchingSide = 7;
    else if( aIntersection.leftX == m_room.leftX
             && aIntersection.lowerLeftDiagonalX > m_room.lowerLeftDiagonalX )
        neighbour.lastTouchingSide = 6;
    else if( aIntersection.upperLeftDiagonalX == m_room.upperLeftDiagonalX
             && aIntersection.leftX > m_room.leftX )
        neighbour.lastTouchingSide = 5;
    else if( aIntersection.topY == m_room.topY
             && aIntersection.upperLeftDiagonalX > m_room.upperLeftDiagonalX )
        neighbour.lastTouchingSide = 4;
    else if( aIntersection.upperRightDiagonalX == m_room.upperRightDiagonalX
             && aIntersection.topY < m_room.topY )
        neighbour.lastTouchingSide = 3;
    else if( aIntersection.rightX == m_room.rightX
             && aIntersection.upperRightDiagonalX < m_room.upperRightDiagonalX )
        neighbour.lastTouchingSide = 2;
    else if( aIntersection.lowerRightDiagonalX == m_room.lowerRightDiagonalX
             && aIntersection.rightX < m_room.rightX )
        neighbour.lastTouchingSide = 1;
    else if( aIntersection.bottomY == m_room.bottomY
             && aIntersection.lowerRightDiagonalX < m_room.lowerRightDiagonalX )
        neighbour.lastTouchingSide = 0;
    else
        return;

    int nextSide = neighbour.firstTouchingSide;
    for( ;; )
    {
        const int currentSide = nextSide;
        nextSide = ( nextSide + 1 ) % 8;
        if( !m_edgeTouches[currentSide] )
        {
            bool touchesOnlyAtCorner = false;
            if( currentSide == neighbour.firstTouchingSide
                && aIntersection.Corner( currentSide ) == m_room.Corner( nextSide ) )
            {
                touchesOnlyAtCorner = true;
            }
            if( currentSide == neighbour.lastTouchingSide
                && aIntersection.Corner( nextSide ) == m_room.Corner( currentSide ) )
            {
                touchesOnlyAtCorner = true;
            }
            if( !touchesOnlyAtCorner )
                m_edgeTouches[currentSide] = true;
        }
        if( currentSide == neighbour.lastTouchingSide )
            break;
    }
    m_neighbours.push_back( std::move( neighbour ) );
}


int SORTED_45_DEGREE_ROOM_NEIGHBOURS::compare(
        const NEIGHBOUR& aLeft, const NEIGHBOUR& aRight )
{
    if( aLeft.firstTouchingSide != aRight.firstTouchingSide )
        return aLeft.firstTouchingSide < aRight.firstTouchingSide ? -1 : 1;

    const auto coordinateCompare = []( std::int64_t aFirst, std::int64_t aSecond )
    {
        return aFirst == aSecond ? 0 : aFirst < aSecond ? -1 : 1;
    };
    const INT_OCTAGON& left = aLeft.intersection;
    const INT_OCTAGON& right = aRight.intersection;
    int result = 0;
    switch( aLeft.firstTouchingSide )
    {
    case 0: result = coordinateCompare( left.CornerX( 0 ), right.CornerX( 0 ) ); break;
    case 1: result = coordinateCompare( left.CornerX( 1 ), right.CornerX( 1 ) ); break;
    case 2: result = coordinateCompare( left.CornerY( 2 ), right.CornerY( 2 ) ); break;
    case 3: result = coordinateCompare( left.CornerY( 3 ), right.CornerY( 3 ) ); break;
    case 4: result = coordinateCompare( right.CornerX( 4 ), left.CornerX( 4 ) ); break;
    case 5: result = coordinateCompare( right.CornerX( 5 ), left.CornerX( 5 ) ); break;
    case 6: result = coordinateCompare( right.CornerY( 6 ), left.CornerY( 6 ) ); break;
    case 7: result = coordinateCompare( right.CornerY( 7 ), left.CornerY( 7 ) ); break;
    default: return 0;
    }

    if( result == 0 )
    {
        const int leftDifference = ( aLeft.lastTouchingSide - aLeft.firstTouchingSide + 8 ) % 8;
        const int rightDifference = ( aRight.lastTouchingSide - aRight.firstTouchingSide + 8 ) % 8;
        if( leftDifference != rightDifference )
            result = leftDifference < rightDifference ? -1 : 1;
    }
    if( result == 0 )
    {
        switch( aLeft.lastTouchingSide )
        {
        case 0: result = coordinateCompare( left.CornerX( 1 ), right.CornerX( 1 ) ); break;
        case 1: result = coordinateCompare( left.CornerX( 2 ), right.CornerX( 2 ) ); break;
        case 2: result = coordinateCompare( left.CornerY( 3 ), right.CornerY( 3 ) ); break;
        case 3: result = coordinateCompare( left.CornerY( 4 ), right.CornerY( 4 ) ); break;
        case 4: result = coordinateCompare( right.CornerX( 5 ), left.CornerX( 5 ) ); break;
        case 5: result = coordinateCompare( right.CornerX( 6 ), left.CornerX( 6 ) ); break;
        case 6: result = coordinateCompare( right.CornerY( 7 ), left.CornerY( 7 ) ); break;
        case 7: result = coordinateCompare( right.CornerY( 0 ), left.CornerY( 0 ) ); break;
        }
    }
    if( result == 0 )
        result = coordinateCompare( aLeft.entry.objectId, aRight.entry.objectId );
    return result;
}


INT_OCTAGON SORTED_45_DEGREE_ROOM_NEIGHBOURS::RemoveNotTouchingBorderLines(
        const INT_OCTAGON& aRoom, const std::array<bool, 8>& aEdgeTouches )
{
    constexpr std::int64_t critical = INT_OCTAGON::CRITICAL_COORDINATE;
    return INT_OCTAGON(
            aEdgeTouches[6] ? aRoom.leftX : -critical,
            aEdgeTouches[0] ? aRoom.bottomY : -critical,
            aEdgeTouches[2] ? aRoom.rightX : critical,
            aEdgeTouches[4] ? aRoom.topY : critical,
            aEdgeTouches[5] ? aRoom.upperLeftDiagonalX : -critical,
            aEdgeTouches[1] ? aRoom.lowerRightDiagonalX : critical,
            aEdgeTouches[7] ? aRoom.lowerLeftDiagonalX : -critical,
            aEdgeTouches[3] ? aRoom.upperRightDiagonalX : critical ).Normalize();
}

} // namespace KICAD_AUTOROUTER
