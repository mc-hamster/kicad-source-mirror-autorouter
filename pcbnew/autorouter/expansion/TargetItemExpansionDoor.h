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

#include <optional>

#include "ExpansionDoor.h"

namespace KICAD_AUTOROUTER
{

/** Door connecting a free-space room to a terminal/target item. */
class TARGET_ITEM_EXPANSION_DOOR : public EXPANSION_DOOR
{
public:
    TARGET_ITEM_EXPANSION_DOOR( EXPANSION_ROOM* aRoom, std::size_t aPadIndex,
                                const ROUTER_POINT& aTarget ) :
            TARGET_ITEM_EXPANSION_DOOR( aRoom, aPadIndex, aTarget, aTarget )
    {
    }

    TARGET_ITEM_EXPANSION_DOOR( EXPANSION_ROOM* aRoom, std::size_t aPadIndex,
                                const ROUTER_POINT& aStart,
                                const ROUTER_POINT& aEnd ) :
            EXPANSION_DOOR( aRoom, nullptr, 0 ),
            m_padIndex( aPadIndex ),
            m_target( aStart ),
            m_targetEnd( aEnd )
    {
    }

    std::size_t PadIndex() const { return m_padIndex; }
    const ROUTER_POINT& Target() const { return m_target; }
    const ROUTER_POINT& TargetEnd() const { return m_targetEnd; }

    /**
     * Host-safe target-door attachment for an integral trace segment.
     *
     * Freerouting intersects the item's exact connection shape with the room
     * and then locates the nearest point on that shape. KiCad track endpoints
     * are integral, so the native worker returns the nearest integral point
     * that is both on the segment and inside the rectangular room. Returning
     * no point is preferable to rounding an AABB projection into a false
     * electrical contact.
     */
    static std::optional<ROUTER_POINT> NearestIntegralPointInRoom(
            const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
            const ROUTER_POINT& aFrom, const ROUTER_BOX& aRoom );

private:
    std::size_t  m_padIndex;
    ROUTER_POINT m_target;
    ROUTER_POINT m_targetEnd;
};

} // namespace KICAD_AUTOROUTER
