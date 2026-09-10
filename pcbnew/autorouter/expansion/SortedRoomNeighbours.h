/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct translation of Freerouting SortedRoomNeighbours.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "../board/searchtree/ShapeSearchTree.h"

namespace KICAD_AUTOROUTER
{

/** Counter-clockwise neighbour topology for unrestricted-angle Simplex rooms.
 *
 * The source class also owns expansion-room allocation and target-item doors.
 * Those lifecycle concerns remain in the maze context; this class translates
 * the exact sorting, edge-removal and incomplete-room geometry without
 * replacing rational support intersections by their bounding boxes.
 */
class SORTED_ROOM_NEIGHBOURS
{
public:
    struct NEIGHBOUR
    {
        SHAPE_TREE_ENTRY entry;
        PLANAR::SIMPLEX shape;
        PLANAR::SIMPLEX intersection;
        int touchingSideNoOfRoom = -1;
        int touchingSideNoOfNeighbourRoom = -1;
        bool roomTouchIsCorner = false;
        bool neighbourRoomTouchIsCorner = false;

        const PLANAR::POINT& FirstCorner( const PLANAR::SIMPLEX& aRoom ) const;
        const PLANAR::POINT& LastCorner( const PLANAR::SIMPLEX& aRoom ) const;
    };

    SORTED_ROOM_NEIGHBOURS(
            PLANAR::SIMPLEX aRoom,
            const std::vector<SHAPE_TREE_ENTRY>& aEntries );

    const std::vector<NEIGHBOUR>& Neighbours() const { return m_neighbours; }

    /** First source border without a touching neighbour, or -1. */
    int FirstUnrestrainedSide() const;

    /** Source calculateNewIncompleteRooms for an incomplete free room.
     * aCompletedShape receives the corner-smoothed completed-room shape.
     */
    std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> IncompleteRooms(
            int aLayer, const PLANAR::SIMPLEX& aContainedShape,
            PLANAR::SIMPLEX* aCompletedShape = nullptr ) const;

    /** Source obstacle branch, including the no-neighbour half-plane fan. */
    std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> ObstacleIncompleteRooms(
            int aLayer ) const;

private:
    static int compareDirectionsFrom( const PLANAR::LINE& aBase,
                                      const PLANAR::LINE& aFirst,
                                      const PLANAR::LINE& aSecond );
    int compare( const NEIGHBOUR& aLeft, const NEIGHBOUR& aRight ) const;
    void addNeighbour( const SHAPE_TREE_ENTRY& aEntry,
                       const PLANAR::SIMPLEX& aShape,
                       const PLANAR::SIMPLEX& aIntersection );
    std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> calculateIncompleteRooms(
            int aLayer, bool aFromIncomplete,
            const PLANAR::SIMPLEX& aContainedShape,
            PLANAR::SIMPLEX* aCompletedShape ) const;

    PLANAR::SIMPLEX m_room;
    std::vector<NEIGHBOUR> m_neighbours;
};

} // namespace KICAD_AUTOROUTER
