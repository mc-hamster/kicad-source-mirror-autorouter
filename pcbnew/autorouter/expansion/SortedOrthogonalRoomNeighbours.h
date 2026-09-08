/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <array>
#include "../datastructures/MinAreaTree.h"
#include "IncompleteFreeSpaceExpansionRoom.h"

namespace KICAD_AUTOROUTER
{

/** Freerouting a11c0a42: boundary ordering/gaps for free (not shove) rooms. */
class SORTED_ORTHOGONAL_ROOM_NEIGHBOURS
{
public:
    struct NEIGHBOUR
    {
        SHAPE_TREE_ENTRY entry;
        ROUTER_BOX intersection;
        int firstSide = -1;
        int lastSide = -1;
    };

    SORTED_ORTHOGONAL_ROOM_NEIGHBOURS( ROUTER_BOX aRoom,
                                     const std::vector<SHAPE_TREE_ENTRY>& aEntries );
    const std::vector<NEIGHBOUR>& Neighbours() const { return m_neighbours; }
    int FirstUnrestrainedSide() const;
    std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM> IncompleteRooms(
            ROUTER_BOX aBounds, int aLayer ) const;

private:
    ROUTER_BOX m_room;
    std::array<bool, 4> m_edgeTouches{};
    std::vector<NEIGHBOUR> m_neighbours;
};

} // namespace KICAD_AUTOROUTER
