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

#include "ExpansionDoor.h"

namespace KICAD_AUTOROUTER
{

/** Door connecting a free-space room to a terminal/target item. */
class TARGET_ITEM_EXPANSION_DOOR : public EXPANSION_DOOR
{
public:
    TARGET_ITEM_EXPANSION_DOOR( EXPANSION_ROOM* aRoom, std::size_t aPadIndex,
                                const ROUTER_POINT& aTarget ) :
            EXPANSION_DOOR( aRoom, nullptr, 0 ),
            m_padIndex( aPadIndex ),
            m_target( aTarget )
    {
    }

    std::size_t PadIndex() const { return m_padIndex; }
    const ROUTER_POINT& Target() const { return m_target; }

private:
    std::size_t  m_padIndex;
    ROUTER_POINT m_target;
};

} // namespace KICAD_AUTOROUTER
