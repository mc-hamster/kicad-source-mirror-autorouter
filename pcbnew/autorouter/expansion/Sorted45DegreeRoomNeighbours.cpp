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
        // A completed free-space room deliberately overlaps its predecessor
        // through a two-dimensional door.  Freerouting keeps that room in the
        // sorted boundary-neighbour cycle; omitting it merges the two gaps on
        // either side into one oversized incomplete room.  Obstacle-room
        // callers already remove illegal 2-D obstacle overlaps before this
        // constructor, so only empty intersections are discarded here.
        if( intersection.Dimension() < 0 )
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


void SORTED_45_DEGREE_ROOM_NEIGHBOURS::insertIncompleteRoom(
        std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>& aResult,
        int aLayer, std::int64_t aLeftX, std::int64_t aBottomY,
        std::int64_t aRightX, std::int64_t aTopY,
        std::int64_t aUpperLeftDiagonalX, std::int64_t aLowerRightDiagonalX,
        std::int64_t aLowerLeftDiagonalX, std::int64_t aUpperRightDiagonalX ) const
{
    const INT_OCTAGON shape = INT_OCTAGON(
            aLeftX, aBottomY, aRightX, aTopY,
            aUpperLeftDiagonalX, aLowerRightDiagonalX,
            aLowerLeftDiagonalX, aUpperRightDiagonalX ).Normalize();
    if( shape.Dimension() != 2 )
        return;
    const INT_OCTAGON contained = m_room.Intersection( shape );
    if( !contained.IsEmpty() && contained.Dimension() > 0 )
        aResult.push_back( { shape, aLayer, contained } );
}


void SORTED_45_DEGREE_ROOM_NEIGHBOURS::appendObstacleEdgeRooms(
        std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>& aResult,
        const INT_OCTAGON& aBoardBounds, int aLayer,
        int aFromSide, int aToSide ) const
{
    const ROUTER_POINT initialCorner = m_room.Corner( aFromSide );
    int currentSide = aFromSide;
    for( ;; )
    {
        const int nextSide = ( currentSide + 1 ) % 8;
        if( initialCorner != m_room.Corner( nextSide ) )
        {
            INT_OCTAGON shape = aBoardBounds;
            switch( currentSide )
            {
            case 0: shape.topY = m_room.bottomY; break;
            case 1: shape.upperLeftDiagonalX = m_room.lowerRightDiagonalX; break;
            case 2: shape.leftX = m_room.rightX; break;
            case 3: shape.lowerLeftDiagonalX = m_room.upperRightDiagonalX; break;
            case 4: shape.bottomY = m_room.topY; break;
            case 5: shape.lowerRightDiagonalX = m_room.upperLeftDiagonalX; break;
            case 6: shape.rightX = m_room.leftX; break;
            case 7: shape.upperRightDiagonalX = m_room.lowerLeftDiagonalX; break;
            default: return;
            }
            insertIncompleteRoom( aResult, aLayer, shape.leftX, shape.bottomY,
                                  shape.rightX, shape.topY,
                                  shape.upperLeftDiagonalX,
                                  shape.lowerRightDiagonalX,
                                  shape.lowerLeftDiagonalX,
                                  shape.upperRightDiagonalX );
        }
        if( currentSide == aToSide )
            break;
        currentSide = nextSide;
    }
}


void SORTED_45_DEGREE_ROOM_NEIGHBOURS::appendObstacleGapRooms(
        std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>& aResult,
        const INT_OCTAGON& aBoardBounds, int aLayer,
        const NEIGHBOUR& aPrevious, const NEIGHBOUR& aNext ) const
{
    const int fromSide = aPrevious.lastTouchingSide;
    const int toSide = aNext.firstTouchingSide;
    if( fromSide == toSide && &aPrevious != &aNext )
        return;

    INT_OCTAGON shape = aBoardBounds;
    switch( fromSide )
    {
    case 0:
        shape.topY = m_room.bottomY;
        shape.upperLeftDiagonalX = aPrevious.intersection.lowerRightDiagonalX;
        break;
    case 1:
        shape.upperLeftDiagonalX = m_room.lowerRightDiagonalX;
        shape.leftX = aPrevious.intersection.rightX;
        break;
    case 2:
        shape.leftX = m_room.rightX;
        shape.lowerLeftDiagonalX = aPrevious.intersection.upperRightDiagonalX;
        break;
    case 3:
        shape.lowerLeftDiagonalX = m_room.upperRightDiagonalX;
        shape.bottomY = aPrevious.intersection.topY;
        break;
    case 4:
        shape.bottomY = m_room.topY;
        shape.lowerRightDiagonalX = aPrevious.intersection.upperLeftDiagonalX;
        break;
    case 5:
        shape.lowerRightDiagonalX = m_room.upperLeftDiagonalX;
        shape.rightX = aPrevious.intersection.leftX;
        break;
    case 6:
        shape.rightX = m_room.leftX;
        shape.upperRightDiagonalX = aPrevious.intersection.lowerLeftDiagonalX;
        break;
    case 7:
        shape.upperRightDiagonalX = m_room.lowerLeftDiagonalX;
        shape.topY = aPrevious.intersection.bottomY;
        break;
    }
    insertIncompleteRoom( aResult, aLayer, shape.leftX, shape.bottomY,
                          shape.rightX, shape.topY, shape.upperLeftDiagonalX,
                          shape.lowerRightDiagonalX, shape.lowerLeftDiagonalX,
                          shape.upperRightDiagonalX );

    shape = aBoardBounds;
    switch( toSide )
    {
    case 0:
        shape.topY = m_room.bottomY;
        shape.upperRightDiagonalX = aNext.intersection.lowerLeftDiagonalX;
        break;
    case 1:
        shape.upperLeftDiagonalX = m_room.lowerRightDiagonalX;
        shape.topY = aNext.intersection.bottomY;
        break;
    case 2:
        shape.leftX = m_room.rightX;
        shape.upperLeftDiagonalX = aNext.intersection.lowerRightDiagonalX;
        break;
    case 3:
        shape.lowerLeftDiagonalX = m_room.upperRightDiagonalX;
        shape.leftX = aNext.intersection.rightX;
        break;
    case 4:
        shape.bottomY = m_room.topY;
        shape.lowerLeftDiagonalX = aNext.intersection.upperRightDiagonalX;
        break;
    case 5:
        shape.lowerRightDiagonalX = m_room.upperLeftDiagonalX;
        shape.bottomY = aNext.intersection.topY;
        break;
    case 6:
        shape.rightX = m_room.leftX;
        shape.lowerRightDiagonalX = aNext.intersection.upperLeftDiagonalX;
        break;
    case 7:
        shape.upperRightDiagonalX = m_room.lowerLeftDiagonalX;
        shape.rightX = aNext.intersection.leftX;
        break;
    }
    insertIncompleteRoom( aResult, aLayer, shape.leftX, shape.bottomY,
                          shape.rightX, shape.topY, shape.upperLeftDiagonalX,
                          shape.lowerRightDiagonalX, shape.lowerLeftDiagonalX,
                          shape.upperRightDiagonalX );

    const int firstFreeSide = ( fromSide + 1 ) % 8;
    if( firstFreeSide != toSide )
        appendObstacleEdgeRooms( aResult, aBoardBounds, aLayer,
                                 firstFreeSide, ( toSide + 7 ) % 8 );
}


std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>
SORTED_45_DEGREE_ROOM_NEIGHBOURS::IncompleteRooms(
        const INT_OCTAGON& aBoardBounds, int aLayer ) const
{
    std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> result;
    if( m_neighbours.empty() )
        return result;

    const NEIGHBOUR* previous = &m_neighbours.back();
    for( const NEIGHBOUR& next : m_neighbours )
    {
        if( !next.intersection.Intersects( previous->intersection ) )
        {
            INT_OCTAGON shape = aBoardBounds;
            switch( next.firstTouchingSide )
            {
            case 0:
                if( previous->intersection.lowerLeftDiagonalX
                    < next.intersection.lowerLeftDiagonalX )
                {
                    shape.upperRightDiagonalX = next.intersection.lowerLeftDiagonalX;
                    shape.topY = previous->intersection.bottomY;
                    if( previous->lastTouchingSide == 0 )
                        shape.upperLeftDiagonalX = previous->intersection.lowerRightDiagonalX;
                }
                else if( previous->intersection.lowerLeftDiagonalX
                         > next.intersection.lowerLeftDiagonalX )
                {
                    shape.rightX = next.intersection.leftX;
                    shape.upperRightDiagonalX = previous->intersection.lowerLeftDiagonalX;
                }
                else
                    shape.upperRightDiagonalX = next.intersection.lowerLeftDiagonalX;
                break;
            case 1:
                if( previous->intersection.bottomY < next.intersection.bottomY )
                {
                    shape.topY = next.intersection.bottomY;
                    shape.upperLeftDiagonalX = previous->intersection.lowerRightDiagonalX;
                    if( previous->lastTouchingSide == 1 )
                        shape.leftX = previous->intersection.rightX;
                }
                else if( previous->intersection.bottomY > next.intersection.bottomY )
                {
                    shape.topY = previous->intersection.bottomY;
                    shape.upperRightDiagonalX = next.intersection.lowerLeftDiagonalX;
                }
                else
                    shape.topY = next.intersection.bottomY;
                break;
            case 2:
                if( previous->intersection.lowerRightDiagonalX
                    > next.intersection.lowerRightDiagonalX )
                {
                    shape.upperLeftDiagonalX = next.intersection.lowerRightDiagonalX;
                    shape.leftX = previous->intersection.rightX;
                    if( previous->lastTouchingSide == 2 )
                        shape.lowerLeftDiagonalX = previous->intersection.upperRightDiagonalX;
                }
                else if( previous->intersection.lowerRightDiagonalX
                         < next.intersection.lowerRightDiagonalX )
                {
                    shape.topY = next.intersection.bottomY;
                    shape.upperLeftDiagonalX = previous->intersection.lowerRightDiagonalX;
                }
                else
                    shape.upperLeftDiagonalX = next.intersection.lowerRightDiagonalX;
                break;
            case 3:
                if( previous->intersection.rightX > next.intersection.rightX )
                {
                    shape.leftX = next.intersection.rightX;
                    shape.lowerLeftDiagonalX = previous->intersection.upperRightDiagonalX;
                    if( previous->lastTouchingSide == 3 )
                        shape.bottomY = previous->intersection.topY;
                }
                else if( previous->intersection.rightX < next.intersection.rightX )
                {
                    shape.leftX = previous->intersection.rightX;
                    shape.upperLeftDiagonalX = next.intersection.lowerRightDiagonalX;
                }
                else
                    shape.leftX = next.intersection.rightX;
                break;
            case 4:
                if( previous->intersection.upperRightDiagonalX
                    > next.intersection.upperRightDiagonalX )
                {
                    shape.lowerLeftDiagonalX = next.intersection.upperRightDiagonalX;
                    shape.bottomY = previous->intersection.topY;
                    if( previous->lastTouchingSide == 4 )
                        shape.lowerRightDiagonalX = previous->intersection.upperLeftDiagonalX;
                }
                else if( previous->intersection.upperRightDiagonalX
                         < next.intersection.upperRightDiagonalX )
                {
                    shape.leftX = next.intersection.rightX;
                    shape.lowerLeftDiagonalX = previous->intersection.upperRightDiagonalX;
                }
                else
                    shape.lowerLeftDiagonalX = next.intersection.upperRightDiagonalX;
                break;
            case 5:
                if( previous->intersection.topY > next.intersection.topY )
                {
                    shape.bottomY = next.intersection.topY;
                    shape.lowerRightDiagonalX = previous->intersection.upperLeftDiagonalX;
                    if( previous->lastTouchingSide == 5 )
                        shape.rightX = previous->intersection.leftX;
                }
                else if( previous->intersection.topY < next.intersection.topY )
                {
                    shape.bottomY = previous->intersection.topY;
                    shape.lowerLeftDiagonalX = next.intersection.upperRightDiagonalX;
                }
                else
                    shape.bottomY = next.intersection.topY;
                break;
            case 6:
                if( previous->intersection.upperLeftDiagonalX
                    < next.intersection.upperLeftDiagonalX )
                {
                    shape.lowerRightDiagonalX = next.intersection.upperLeftDiagonalX;
                    shape.rightX = previous->intersection.leftX;
                    if( previous->lastTouchingSide == 6 )
                        shape.upperRightDiagonalX = previous->intersection.lowerLeftDiagonalX;
                }
                else if( previous->intersection.upperLeftDiagonalX
                         > next.intersection.upperLeftDiagonalX )
                {
                    shape.bottomY = next.intersection.topY;
                    shape.lowerRightDiagonalX = previous->intersection.upperLeftDiagonalX;
                }
                else
                    shape.lowerRightDiagonalX = next.intersection.upperLeftDiagonalX;
                break;
            case 7:
                if( previous->intersection.leftX < next.intersection.leftX )
                {
                    shape.rightX = next.intersection.leftX;
                    shape.upperRightDiagonalX = previous->intersection.lowerLeftDiagonalX;
                    if( previous->lastTouchingSide == 7 )
                        shape.topY = previous->intersection.bottomY;
                }
                else if( previous->intersection.leftX > next.intersection.leftX )
                {
                    shape.rightX = previous->intersection.leftX;
                    shape.lowerRightDiagonalX = next.intersection.upperLeftDiagonalX;
                }
                else
                    shape.rightX = next.intersection.leftX;
                break;
            }
            insertIncompleteRoom( result, aLayer, shape.leftX, shape.bottomY,
                                  shape.rightX, shape.topY,
                                  shape.upperLeftDiagonalX,
                                  shape.lowerRightDiagonalX,
                                  shape.lowerLeftDiagonalX,
                                  shape.upperRightDiagonalX );
        }
        previous = &next;
    }
    return result;
}


void SORTED_45_DEGREE_ROOM_NEIGHBOURS::appendFreeGapRoom(
        std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>& aResult,
        const INT_OCTAGON& aBoardBounds, int aLayer,
        const NEIGHBOUR& aPrevious, const NEIGHBOUR& aNext ) const
{
    INT_OCTAGON shape = aBoardBounds;
    switch( aNext.firstTouchingSide )
    {
    case 0:
        if( aPrevious.intersection.lowerLeftDiagonalX
            < aNext.intersection.lowerLeftDiagonalX )
        {
            shape.upperRightDiagonalX = aNext.intersection.lowerLeftDiagonalX;
            shape.topY = aPrevious.intersection.bottomY;
            if( aPrevious.lastTouchingSide == 0 )
                shape.upperLeftDiagonalX = aPrevious.intersection.lowerRightDiagonalX;
        }
        else if( aPrevious.intersection.lowerLeftDiagonalX
                 > aNext.intersection.lowerLeftDiagonalX )
        {
            shape.rightX = aNext.intersection.leftX;
            shape.upperRightDiagonalX = aPrevious.intersection.lowerLeftDiagonalX;
        }
        else
            shape.upperRightDiagonalX = aNext.intersection.lowerLeftDiagonalX;
        break;
    case 1:
        if( aPrevious.intersection.bottomY < aNext.intersection.bottomY )
        {
            shape.topY = aNext.intersection.bottomY;
            shape.upperLeftDiagonalX = aPrevious.intersection.lowerRightDiagonalX;
            if( aPrevious.lastTouchingSide == 1 )
                shape.leftX = aPrevious.intersection.rightX;
        }
        else if( aPrevious.intersection.bottomY > aNext.intersection.bottomY )
        {
            shape.topY = aPrevious.intersection.bottomY;
            shape.upperRightDiagonalX = aNext.intersection.lowerLeftDiagonalX;
        }
        else
            shape.topY = aNext.intersection.bottomY;
        break;
    case 2:
        if( aPrevious.intersection.lowerRightDiagonalX
            > aNext.intersection.lowerRightDiagonalX )
        {
            shape.upperLeftDiagonalX = aNext.intersection.lowerRightDiagonalX;
            shape.leftX = aPrevious.intersection.rightX;
            if( aPrevious.lastTouchingSide == 2 )
                shape.lowerLeftDiagonalX = aPrevious.intersection.upperRightDiagonalX;
        }
        else if( aPrevious.intersection.lowerRightDiagonalX
                 < aNext.intersection.lowerRightDiagonalX )
        {
            shape.topY = aNext.intersection.bottomY;
            shape.upperLeftDiagonalX = aPrevious.intersection.lowerRightDiagonalX;
        }
        else
            shape.upperLeftDiagonalX = aNext.intersection.lowerRightDiagonalX;
        break;
    case 3:
        if( aPrevious.intersection.rightX > aNext.intersection.rightX )
        {
            shape.leftX = aNext.intersection.rightX;
            shape.lowerLeftDiagonalX = aPrevious.intersection.upperRightDiagonalX;
            if( aPrevious.lastTouchingSide == 3 )
                shape.bottomY = aPrevious.intersection.topY;
        }
        else if( aPrevious.intersection.rightX < aNext.intersection.rightX )
        {
            shape.leftX = aPrevious.intersection.rightX;
            shape.upperLeftDiagonalX = aNext.intersection.lowerRightDiagonalX;
        }
        else
            shape.leftX = aNext.intersection.rightX;
        break;
    case 4:
        if( aPrevious.intersection.upperRightDiagonalX
            > aNext.intersection.upperRightDiagonalX )
        {
            shape.lowerLeftDiagonalX = aNext.intersection.upperRightDiagonalX;
            shape.bottomY = aPrevious.intersection.topY;
            if( aPrevious.lastTouchingSide == 4 )
                shape.lowerRightDiagonalX = aPrevious.intersection.upperLeftDiagonalX;
        }
        else if( aPrevious.intersection.upperRightDiagonalX
                 < aNext.intersection.upperRightDiagonalX )
        {
            shape.leftX = aNext.intersection.rightX;
            shape.lowerLeftDiagonalX = aPrevious.intersection.upperRightDiagonalX;
        }
        else
            shape.lowerLeftDiagonalX = aNext.intersection.upperRightDiagonalX;
        break;
    case 5:
        if( aPrevious.intersection.topY > aNext.intersection.topY )
        {
            shape.bottomY = aNext.intersection.topY;
            shape.lowerRightDiagonalX = aPrevious.intersection.upperLeftDiagonalX;
            if( aPrevious.lastTouchingSide == 5 )
                shape.rightX = aPrevious.intersection.leftX;
        }
        else if( aPrevious.intersection.topY < aNext.intersection.topY )
        {
            shape.bottomY = aPrevious.intersection.topY;
            shape.lowerLeftDiagonalX = aNext.intersection.upperRightDiagonalX;
        }
        else
            shape.bottomY = aNext.intersection.topY;
        break;
    case 6:
        if( aPrevious.intersection.upperLeftDiagonalX
            < aNext.intersection.upperLeftDiagonalX )
        {
            shape.lowerRightDiagonalX = aNext.intersection.upperLeftDiagonalX;
            shape.rightX = aPrevious.intersection.leftX;
            if( aPrevious.lastTouchingSide == 6 )
                shape.upperRightDiagonalX = aPrevious.intersection.lowerLeftDiagonalX;
        }
        else if( aPrevious.intersection.upperLeftDiagonalX
                 > aNext.intersection.upperLeftDiagonalX )
        {
            shape.bottomY = aNext.intersection.topY;
            shape.lowerRightDiagonalX = aPrevious.intersection.upperLeftDiagonalX;
        }
        else
            shape.lowerRightDiagonalX = aNext.intersection.upperLeftDiagonalX;
        break;
    case 7:
        if( aPrevious.intersection.leftX < aNext.intersection.leftX )
        {
            shape.rightX = aNext.intersection.leftX;
            shape.upperRightDiagonalX = aPrevious.intersection.lowerLeftDiagonalX;
            if( aPrevious.lastTouchingSide == 7 )
                shape.topY = aPrevious.intersection.bottomY;
        }
        else if( aPrevious.intersection.leftX > aNext.intersection.leftX )
        {
            shape.rightX = aPrevious.intersection.leftX;
            shape.lowerRightDiagonalX = aNext.intersection.upperLeftDiagonalX;
        }
        else
            shape.rightX = aNext.intersection.leftX;
        break;
    }
    insertIncompleteRoom( aResult, aLayer, shape.leftX, shape.bottomY,
                          shape.rightX, shape.topY,
                          shape.upperLeftDiagonalX,
                          shape.lowerRightDiagonalX,
                          shape.lowerLeftDiagonalX,
                          shape.upperRightDiagonalX );
}


std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>
SORTED_45_DEGREE_ROOM_NEIGHBOURS::ObstacleIncompleteRooms(
        const INT_OCTAGON& aBoardBounds, int aLayer ) const
{
    std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> result;
    if( m_neighbours.empty() )
    {
        appendObstacleEdgeRooms( result, aBoardBounds, aLayer, 0, 7 );
        return result;
    }
    if( m_neighbours.size() == 1 )
    {
        appendObstacleGapRooms( result, aBoardBounds, aLayer,
                                m_neighbours.front(), m_neighbours.front() );
        return result;
    }

    const NEIGHBOUR* previous = &m_neighbours.back();
    for( const NEIGHBOUR& next : m_neighbours )
    {
        bool insertRoom;
        if( m_neighbours.size() == 2 )
        {
            const INT_OCTAGON intersection = next.intersection.Intersection(
                    previous->intersection );
            if( intersection.IsEmpty() )
                insertRoom = true;
            else if( intersection.Dimension() >= 1 )
                insertRoom = false;
            else if( previous->lastTouchingSide == next.firstTouchingSide )
                insertRoom = false;
            else
                insertRoom = previous->lastTouchingSide
                             != ( next.firstTouchingSide + 1 ) % 8;
        }
        else
            insertRoom = !next.intersection.Intersects( previous->intersection );

        if( insertRoom )
        {
            if( next.firstTouchingSide != previous->lastTouchingSide )
                appendObstacleGapRooms( result, aBoardBounds, aLayer, *previous, next );
            else
                appendFreeGapRoom( result, aBoardBounds, aLayer, *previous, next );
        }
        previous = &next;
    }
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
