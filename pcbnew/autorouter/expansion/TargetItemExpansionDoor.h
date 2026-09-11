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
#include <vector>

#include "ExpansionDoor.h"

namespace KICAD_AUTOROUTER
{

/** Door connecting a free-space room to a terminal/target item. */
class TARGET_ITEM_EXPANSION_DOOR : public EXPANSION_DOOR
{
public:
    TARGET_ITEM_EXPANSION_DOOR( EXPANSION_ROOM* aRoom, std::size_t aPadIndex,
                                const ROUTER_POINT& aTarget ) :
            TARGET_ITEM_EXPANSION_DOOR( aRoom, static_cast<int>( aPadIndex ),
                                        aPadIndex, aTarget, aTarget,
                                        { aTarget.x, aTarget.y,
                                          aTarget.x, aTarget.y } )
    {
    }

    TARGET_ITEM_EXPANSION_DOOR( EXPANSION_ROOM* aRoom, std::size_t aPadIndex,
                                const ROUTER_POINT& aStart,
                                const ROUTER_POINT& aEnd ) :
            TARGET_ITEM_EXPANSION_DOOR(
                    aRoom, static_cast<int>( aPadIndex ), aPadIndex,
                    aStart, aEnd,
                    { std::min( aStart.x, aEnd.x ),
                      std::min( aStart.y, aEnd.y ),
                      std::max( aStart.x, aEnd.x ),
                      std::max( aStart.y, aEnd.y ) } )
    {
    }

    /**
     * Construct the source/destination item door represented by
     * TargetItemExpansionDoor(item, treeEntryNo, room, searchTree).
     *
     * Unlike an ordinary ExpansionDoor it has only one room, has dimension
     * two by definition, and is stored in the room's target-door collection
     * rather than its ordinary neighbour-door collection.  The native room
     * frontiers own these objects separately, so registering this one-sided
     * door in ExpansionRoom::GetDoors() would make neighbour traversal try to
     * follow a null second room.
     */
    TARGET_ITEM_EXPANSION_DOOR( EXPANSION_ROOM* aRoom, int aItemId,
                                std::size_t aPadIndex,
                                const ROUTER_POINT& aStart,
                                const ROUTER_POINT& aEnd,
                                ROUTER_BOX aTreeBounds ) :
            EXPANSION_DOOR( aRoom, nullptr, 2, false ),
            m_itemId( aItemId ),
            m_padIndex( aPadIndex ),
            m_target( aStart ),
            m_targetEnd( aEnd ),
            m_treeBounds( aTreeBounds )
    {
    }

    std::size_t PadIndex() const { return m_padIndex; }
    const ROUTER_POINT& Target() const { return m_target; }
    const ROUTER_POINT& TargetEnd() const { return m_targetEnd; }

    ROUTER_BOX GetShape() const override;
    PLANAR::INT_OCTAGON GetOctagonShape() const override;
    PLANAR::SIMPLEX GetSimplexShape() const override;
    int GetId() const override;

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

    static std::optional<ROUTER_POINT> NearestIntegralPointInRoom(
            const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
            const ROUTER_POINT& aFrom,
            const PLANAR::INT_OCTAGON& aRoom );

    /** Exact unrestricted-angle variant.  The source intersects the target
     * shape with an arbitrary TileShape; the native worker keeps that room as
     * a rational SIMPLEX and solves every support inequality on the segment's
     * primitive lattice parameter before choosing the nearest legal point. */
    static std::optional<ROUTER_POINT> NearestIntegralPointInRoom(
            const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
            const ROUTER_POINT& aFrom, const PLANAR::SIMPLEX& aRoom );

    /** ConductionArea target-door variant. The exact filled polygon (including
     * holes) is inset by the physical trace radius before room intersection;
     * unlike a Pin target, it is never collapsed to a sampled centre point. */
    static std::optional<ROUTER_POINT> NearestIntegralPointInRoom(
            const ROUTING_OBSTACLE& aArea, std::int64_t aInset,
            const ROUTER_POINT& aFrom, const ROUTER_BOX& aRoom );

    static std::optional<ROUTER_POINT> NearestIntegralPointInRoom(
            const ROUTING_OBSTACLE& aArea, std::int64_t aInset,
            const ROUTER_POINT& aFrom, const PLANAR::INT_OCTAGON& aRoom );

    static std::optional<ROUTER_POINT> NearestIntegralPointInRoom(
            const ROUTING_OBSTACLE& aArea, std::int64_t aInset,
            const ROUTER_POINT& aFrom, const PLANAR::SIMPLEX& aRoom );

    /**
     * Return a bounded set of exact lattice points which represents an
     * integral segment in an orthogonal room decomposition.
     *
     * A diagonal source trace cannot be seeded with its axis-aligned bounding
     * box: doing so invents electrical copper in both empty corner wedges.
     * Orthogonal room membership can change only where the segment crosses an
     * x/y side of a compensated obstacle.  Sampling the lattice indices on
     * both sides of every such cut therefore reaches every rectangular room
     * touched by the real centre-line without walking a potentially enormous
     * segment point by point.
     */
    static std::vector<ROUTER_POINT> IntegralRoomSeedPoints(
            const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
            const std::vector<ROUTER_BOX>& aOrthogonalCuts );

    static std::vector<ROUTER_POINT> IntegralRoomSeedPoints(
            const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
            const std::vector<PLANAR::INT_OCTAGON>& aFortyFiveDegreeCuts );

    static std::vector<ROUTER_POINT> IntegralRoomSeedPoints(
            const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
            const std::vector<PLANAR::SIMPLEX>& aAnyAngleCuts );

private:
    int          m_itemId;
    std::size_t  m_padIndex;
    ROUTER_POINT m_target;
    ROUTER_POINT m_targetEnd;
    ROUTER_BOX   m_treeBounds;
};

} // namespace KICAD_AUTOROUTER
