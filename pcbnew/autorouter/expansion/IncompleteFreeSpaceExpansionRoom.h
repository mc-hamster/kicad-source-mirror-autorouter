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

#include "FreeSpaceExpansionRoom.h"

namespace KICAD_AUTOROUTER
{

/** Freerouting equivalent: autoroute/expansion/IncompleteFreeSpaceExpansionRoom. */
class INCOMPLETE_FREE_SPACE_EXPANSION_ROOM : public FREE_SPACE_EXPANSION_ROOM
{
public:
    using FREE_SPACE_EXPANSION_ROOM::FREE_SPACE_EXPANSION_ROOM;

    INCOMPLETE_FREE_SPACE_EXPANSION_ROOM( ROUTER_BOX aShape, int aLayer,
                                          ROUTER_BOX aContainedShape ) :
            FREE_SPACE_EXPANSION_ROOM( 0, aLayer, aShape ),
            m_containedShape( aContainedShape )
    {
    }

    const ROUTER_BOX& GetContainedShape() const { return m_containedShape; }
    void SetContainedShape( ROUTER_BOX aShape ) { m_containedShape = aShape; }

private:
    ROUTER_BOX m_containedShape{ 1, 1, 0, 0 };
};

} // namespace KICAD_AUTOROUTER
