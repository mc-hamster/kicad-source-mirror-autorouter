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

#include <cstdlib>
#include <memory>

#include "ExpansionRoom.h"

namespace KICAD_AUTOROUTER
{

struct MAZE_TRACE_ROOM_INFO;

/** Freerouting equivalent: autoroute/expansion/ObstacleExpansionRoom. */
class OBSTACLE_EXPANSION_ROOM : public EXPANSION_ROOM
{
public:
    OBSTACLE_EXPANSION_ROOM( int aId, int aLayer, ROUTER_BOX aShape,
                             std::size_t aGroup = std::numeric_limits<std::size_t>::max(),
                             int aRipupCost = 0, int aShapeIndex = 0,
                             std::shared_ptr<const MAZE_TRACE_ROOM_INFO> aTraceInfo = {},
                             int aNetCode = 0,
                             std::uint64_t aSearchObjectId = 0 ) :
            EXPANSION_ROOM( aId, aLayer, aShape, true, aSearchObjectId ),
            m_group( aGroup ),
            m_ripupCost( aRipupCost ),
            m_shapeIndex( aShapeIndex ),
            m_traceInfo( std::move( aTraceInfo ) ),
            m_netCode( aNetCode )
    {
    }

    OBSTACLE_EXPANSION_ROOM( int aId, int aLayer,
                             PLANAR::INT_OCTAGON aShape,
                             std::size_t aGroup = std::numeric_limits<std::size_t>::max(),
                             int aRipupCost = 0, int aShapeIndex = 0,
                             std::shared_ptr<const MAZE_TRACE_ROOM_INFO> aTraceInfo = {},
                             int aNetCode = 0,
                             std::uint64_t aSearchObjectId = 0 ) :
            EXPANSION_ROOM( aId, aLayer, std::move( aShape ), true,
                            aSearchObjectId ),
            m_group( aGroup ),
            m_ripupCost( aRipupCost ),
            m_shapeIndex( aShapeIndex ),
            m_traceInfo( std::move( aTraceInfo ) ),
            m_netCode( aNetCode )
    {
    }

    OBSTACLE_EXPANSION_ROOM( int aId, int aLayer,
                             PLANAR::SIMPLEX aShape,
                             std::size_t aGroup = std::numeric_limits<std::size_t>::max(),
                             int aRipupCost = 0, int aShapeIndex = 0,
                             std::shared_ptr<const MAZE_TRACE_ROOM_INFO> aTraceInfo = {},
                             int aNetCode = 0,
                             std::uint64_t aSearchObjectId = 0 ) :
            EXPANSION_ROOM( aId, aLayer, std::move( aShape ), true,
                            aSearchObjectId ),
            m_group( aGroup ),
            m_ripupCost( aRipupCost ),
            m_shapeIndex( aShapeIndex ),
            m_traceInfo( std::move( aTraceInfo ) ),
            m_netCode( aNetCode )
    {
    }

    std::size_t GetGroup() const { return m_group; }
    int         GetRipupCost() const { return m_ripupCost; }
    int         GetShapeIndex() const { return m_shapeIndex; }
    int         GetNetCode() const { return m_netCode; }
    bool        AllDoorsCalculated() const { return m_doorsCalculated; }
    void        SetDoorsCalculated( bool aValue ) { m_doorsCalculated = aValue; }
    const std::shared_ptr<const MAZE_TRACE_ROOM_INFO>& GetTraceInfo() const
    {
        return m_traceInfo;
    }

    /** Source-equivalent ObstacleExpansionRoom.createOverlapDoor eligibility.
     *
     * Distinct routable items may share a two-dimensional door whenever they
     * share a net.  Shapes belonging to the same PolylineTrace item are the
     * sole exception: only consecutive tree shapes are connected.  Keeping
     * the same-item restriction separate from the shared-net rule matters at
     * trace junctions, where two distinct PolylineTrace items overlap. */
    bool CanCreateOverlapDoorWith( const OBSTACLE_EXPANSION_ROOM& aOther ) const
    {
        if( m_netCode == 0 || m_netCode != aOther.m_netCode )
            return false;

        if( m_group != aOther.m_group )
            return true;

        return m_traceInfo && aOther.m_traceInfo
               && std::abs( m_shapeIndex - aOther.m_shapeIndex ) == 1;
    }

private:
    std::size_t m_group;
    int         m_ripupCost;
    int         m_shapeIndex;
    std::shared_ptr<const MAZE_TRACE_ROOM_INFO> m_traceInfo;
    int         m_netCode;
    bool        m_doorsCalculated = false;
};

} // namespace KICAD_AUTOROUTER
