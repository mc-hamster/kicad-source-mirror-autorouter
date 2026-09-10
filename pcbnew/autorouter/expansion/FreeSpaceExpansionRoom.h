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

#include "ExpansionRoom.h"

namespace KICAD_AUTOROUTER
{

/** Freerouting equivalent: autoroute/expansion/FreeSpaceExpansionRoom. */
class FREE_SPACE_EXPANSION_ROOM : public EXPANSION_ROOM
{
public:
    FREE_SPACE_EXPANSION_ROOM( int aId, int aLayer, ROUTER_BOX aShape ) :
            EXPANSION_ROOM( aId, aLayer, aShape, false )
    {
    }

    FREE_SPACE_EXPANSION_ROOM( int aId, int aLayer,
                               PLANAR::INT_OCTAGON aShape ) :
            EXPANSION_ROOM( aId, aLayer, std::move( aShape ), false )
    {
    }
};

} // namespace KICAD_AUTOROUTER
