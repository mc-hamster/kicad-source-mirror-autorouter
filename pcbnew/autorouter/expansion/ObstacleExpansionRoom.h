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

/** Freerouting equivalent: autoroute/expansion/ObstacleExpansionRoom. */
class OBSTACLE_EXPANSION_ROOM : public EXPANSION_ROOM
{
public:
    OBSTACLE_EXPANSION_ROOM( int aId, int aLayer, ROUTER_BOX aShape,
                             std::size_t aGroup = std::numeric_limits<std::size_t>::max(),
                             int aRipupCost = 0, int aShapeIndex = 0 ) :
            EXPANSION_ROOM( aId, aLayer, aShape, true ),
            m_group( aGroup ),
            m_ripupCost( aRipupCost ),
            m_shapeIndex( aShapeIndex )
    {
    }

    OBSTACLE_EXPANSION_ROOM( int aId, int aLayer,
                             PLANAR::INT_OCTAGON aShape,
                             std::size_t aGroup = std::numeric_limits<std::size_t>::max(),
                             int aRipupCost = 0, int aShapeIndex = 0 ) :
            EXPANSION_ROOM( aId, aLayer, std::move( aShape ), true ),
            m_group( aGroup ),
            m_ripupCost( aRipupCost ),
            m_shapeIndex( aShapeIndex )
    {
    }

    OBSTACLE_EXPANSION_ROOM( int aId, int aLayer,
                             PLANAR::SIMPLEX aShape,
                             std::size_t aGroup = std::numeric_limits<std::size_t>::max(),
                             int aRipupCost = 0, int aShapeIndex = 0 ) :
            EXPANSION_ROOM( aId, aLayer, std::move( aShape ), true ),
            m_group( aGroup ),
            m_ripupCost( aRipupCost ),
            m_shapeIndex( aShapeIndex )
    {
    }

    std::size_t GetGroup() const { return m_group; }
    int         GetRipupCost() const { return m_ripupCost; }
    int         GetShapeIndex() const { return m_shapeIndex; }

private:
    std::size_t m_group;
    int         m_ripupCost;
    int         m_shapeIndex;
};

} // namespace KICAD_AUTOROUTER
