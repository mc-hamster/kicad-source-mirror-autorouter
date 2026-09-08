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

#include "../AutorouterTypes.h"
#include "../geometry/planar/IntBox.h"

namespace KICAD_AUTOROUTER
{

class EXPANSION_ROOM;

/** Layer-change object: one maze section and expansion room per physical layer. */
struct EXPANSION_DRILL
{
    ROUTER_POINT location;
    ROUTER_BOX   freeShape;
    int          firstLayer = -1;
    int          lastLayer = -1;
    bool         valid = true;
    std::vector<EXPANSION_ROOM*> rooms;
    std::vector<bool> occupied;

    int GetId() const
    {
        return static_cast<std::int32_t>( 31u * ( 31u * static_cast<std::uint32_t>(
                INT_BOX::PointId( location ) ) + static_cast<std::uint32_t>( firstLayer ) )
                + static_cast<std::uint32_t>( lastLayer ) );
    }
    void Reset() { std::fill( occupied.begin(), occupied.end(), false ); }
};

} // namespace KICAD_AUTOROUTER
