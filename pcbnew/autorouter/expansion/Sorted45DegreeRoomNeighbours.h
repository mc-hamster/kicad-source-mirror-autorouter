/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct translation of Freerouting Sorted45DegreeRoomNeighbours.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "../datastructures/MinAreaTree.h"
#include "../board/searchtree/ShapeSearchTree45Degree.h"

#include <array>

namespace KICAD_AUTOROUTER
{

/** Counter-clockwise octagonal neighbour ordering used by the 45-degree room
 * lifecycle.  Expansion-room object and door ownership stays in the caller.
 */
class SORTED_45_DEGREE_ROOM_NEIGHBOURS
{
public:
    struct NEIGHBOUR
    {
        SHAPE_TREE_ENTRY entry;
        PLANAR::INT_OCTAGON shape;
        PLANAR::INT_OCTAGON intersection;
        int firstTouchingSide = -1;
        int lastTouchingSide = -1;
    };

    SORTED_45_DEGREE_ROOM_NEIGHBOURS(
            PLANAR::INT_OCTAGON aRoom,
            const std::vector<SHAPE_TREE_ENTRY>& aEntries );

    const std::vector<NEIGHBOUR>& Neighbours() const { return m_neighbours; }
    const std::array<bool, 8>& EdgeInteriorTouchesObstacle() const { return m_edgeTouches; }

    static PLANAR::INT_OCTAGON RemoveNotTouchingBorderLines(
            const PLANAR::INT_OCTAGON& aRoom,
            const std::array<bool, 8>& aEdgeTouches );

    /** Source calculateNewIncompleteRooms branches, without room/door object
     * allocation.  Returned shapes are ready for the caller's ownership
     * layer to attach with their exact octagonal overlap doors. */
    std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> IncompleteRooms(
            const PLANAR::INT_OCTAGON& aBoardBounds, int aLayer ) const;
    std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> ObstacleIncompleteRooms(
            const PLANAR::INT_OCTAGON& aBoardBounds, int aLayer ) const;

private:
    static int compare( const NEIGHBOUR& aLeft, const NEIGHBOUR& aRight );
    void addNeighbour( const SHAPE_TREE_ENTRY& aEntry,
                       const PLANAR::INT_OCTAGON& aShape,
                       const PLANAR::INT_OCTAGON& aIntersection );
    void insertIncompleteRoom(
            std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>& aResult,
            int aLayer, std::int64_t aLeftX, std::int64_t aBottomY,
            std::int64_t aRightX, std::int64_t aTopY,
            std::int64_t aUpperLeftDiagonalX,
            std::int64_t aLowerRightDiagonalX,
            std::int64_t aLowerLeftDiagonalX,
            std::int64_t aUpperRightDiagonalX ) const;
    void appendObstacleEdgeRooms(
            std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>& aResult,
            const PLANAR::INT_OCTAGON& aBoardBounds, int aLayer,
            int aFromSide, int aToSide ) const;
    void appendObstacleGapRooms(
            std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>& aResult,
            const PLANAR::INT_OCTAGON& aBoardBounds, int aLayer,
            const NEIGHBOUR& aPrevious, const NEIGHBOUR& aNext ) const;
    void appendFreeGapRoom(
            std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>& aResult,
            const PLANAR::INT_OCTAGON& aBoardBounds, int aLayer,
            const NEIGHBOUR& aPrevious, const NEIGHBOUR& aNext ) const;

    PLANAR::INT_OCTAGON m_room;
    std::array<bool, 8> m_edgeTouches{};
    std::vector<NEIGHBOUR> m_neighbours;
};

} // namespace KICAD_AUTOROUTER
