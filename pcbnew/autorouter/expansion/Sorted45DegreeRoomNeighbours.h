/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct translation of Freerouting Sorted45DegreeRoomNeighbours.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "../datastructures/MinAreaTree.h"

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

private:
    static int compare( const NEIGHBOUR& aLeft, const NEIGHBOUR& aRight );
    void addNeighbour( const SHAPE_TREE_ENTRY& aEntry,
                       const PLANAR::INT_OCTAGON& aShape,
                       const PLANAR::INT_OCTAGON& aIntersection );

    PLANAR::INT_OCTAGON m_room;
    std::array<bool, 8> m_edgeTouches{};
    std::vector<NEIGHBOUR> m_neighbours;
};

} // namespace KICAD_AUTOROUTER
