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

#include "DrillPage.h"

#include <algorithm>
#include "../geometry/planar/FloatLine.h"
#include "../geometry/planar/PolylineArea.h"

namespace KICAD_AUTOROUTER
{

void DRILL_PAGE::AddCandidate( const ROUTER_POINT& aPoint )
{
    if( Contains( aPoint )
        && std::find( m_candidates.begin(), m_candidates.end(), aPoint ) == m_candidates.end() )
    {
        m_candidates.push_back( aPoint );
    }
}


ROUTER_POINT DRILL_PAGE::Center() const
{
    return FLOAT_POINT{ ( static_cast<double>( m_shape.minX ) + m_shape.maxX ) / 2,
                        ( static_cast<double>( m_shape.minY ) + m_shape.maxY ) / 2 }.Round();
}

int DRILL_PAGE::GetId() const
{
    return static_cast<std::int32_t>( 31u * static_cast<std::uint32_t>( INT_BOX::Id( m_shape ) )
                                     + static_cast<std::uint32_t>( m_net ) );
}

std::vector<EXPANSION_DRILL>* DRILL_PAGE::GetDrills(
        const std::vector<SHAPE_TREE_ENTRY>& aObstacles, int aNet, int aLayerCount,
        bool aAttachSmd, const std::vector<DRILL_PIN>& aPins,
        const ROUTER_CANCEL_CALLBACK& aCancel, std::size_t aMaxPieces )
{
    if( aLayerCount < 1 || ( aCancel && aCancel() ) )
        return nullptr;
    if( m_drills && m_net == aNet && m_attachSmd == aAttachSmd && m_layerCount == aLayerCount )
        return &*m_drills;
    Invalidate();
    m_net = aNet;
    m_attachSmd = aAttachSmd;
    m_layerCount = aLayerCount;
    std::vector<ROUTER_BOX> holes;
    auto previous = INT_BOX::Empty();
    for( const auto& entry : aObstacles )
    {
        if( aCancel && aCancel() )
            return nullptr;
        if( entry.isRoom || !entry.IsTraceObstacle( aNet )
            || !INT_BOX::Intersects( entry.shape, m_shape ) )
            continue;
        if( !INT_BOX::Contains( previous, entry.shape ) )
        {
            auto cutout = INT_BOX::Intersection( entry.shape, m_shape );
            if( INT_BOX::Dimension( cutout ) == 2 )
                holes.push_back( cutout );
        }
        previous = entry.shape;
    }
    auto shapes = POLYLINE_AREA::SplitToConvex( m_shape, holes, aCancel, aMaxPieces );
    if( !shapes )
        return nullptr;
    std::vector<EXPANSION_DRILL> drills;
    for( auto shape : *shapes )
    {
        if( aCancel && aCancel() )
            return nullptr;
        std::optional<ROUTER_POINT> pinCenter;
        if( aAttachSmd )
            for( int layer : { 0, aLayerCount - 1 } )
            {
                // calcPinCenterInDrill: last eligible pin on the first outer
                // layer with a candidate, and strict interior containment.
                for( const auto& pin : aPins )
                    if( pin.layer == layer && pin.drillAllowed
                        && pin.position.x > shape.minX && pin.position.x < shape.maxX
                        && pin.position.y > shape.minY && pin.position.y < shape.maxY )
                        pinCenter = pin.position;
                if( pinCenter )
                    break;
            }
        EXPANSION_DRILL drill;
        drill.freeShape = shape;
        drill.location = pinCenter.value_or( FLOAT_POINT{
                ( static_cast<double>( shape.minX ) + shape.maxX ) / 2,
                ( static_cast<double>( shape.minY ) + shape.maxY ) / 2 }.Round() );
        drill.firstLayer = 0;
        drill.lastLayer = aLayerCount - 1;
        drill.rooms.resize( aLayerCount, nullptr );
        drill.occupied.resize( aLayerCount, false );
        drills.push_back( std::move( drill ) );
    }
    m_drills = std::move( drills );
    m_valid = true;
    return &*m_drills;
}

} // namespace KICAD_AUTOROUTER
