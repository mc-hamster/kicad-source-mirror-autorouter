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
        const ROUTER_CANCEL_CALLBACK& aCancel, std::size_t aMaxPieces,
        bool aAnyAngle )
{
    if( aLayerCount < 1 || ( aCancel && aCancel() ) )
        return nullptr;
    if( m_drills && m_net == aNet && m_attachSmd == aAttachSmd
        && m_layerCount == aLayerCount && m_anyAngle == aAnyAngle )
    {
        return &*m_drills;
    }
    Invalidate();
    m_net = aNet;
    m_attachSmd = aAttachSmd;
    m_layerCount = aLayerCount;
    m_anyAngle = aAnyAngle;

    if( aAnyAngle )
    {
        const PLANAR::SIMPLEX pageShape = PLANAR::SIMPLEX::Box( m_shape );
        std::vector<PLANAR::SIMPLEX> holes;
        std::optional<PLANAR::SIMPLEX> previous;

        for( const auto& entry : aObstacles )
        {
            if( aCancel && aCancel() )
                return nullptr;

            if( entry.isRoom || !entry.IsTraceObstacle( aNet )
                || !INT_BOX::Intersects( entry.shape, m_shape ) )
            {
                continue;
            }

            const PLANAR::SIMPLEX obstacle = entry.BoundingSimplex();

            // Preserve DrillPage.getDrills()' source-order suppression of
            // repeated layer shapes (most commonly the identical contours of
            // a through via).  Compare the uncut shapes exactly; page clipping
            // is performed only for a shape that survives this test.
            if( !previous || !previous->Contains( obstacle ) )
            {
                const PLANAR::SIMPLEX cutout = obstacle.Intersection( pageShape );

                if( cutout.Dimension() == 2 )
                    holes.push_back( cutout );
            }

            previous = obstacle;
        }

        auto shapes = POLYLINE_AREA::SplitSimplexToConvex(
                pageShape, holes, aCancel, aMaxPieces );

        if( !shapes )
            return nullptr;

        std::vector<EXPANSION_DRILL> drills;

        for( auto& shape : *shapes )
        {
            if( aCancel && aCancel() )
                return nullptr;

            std::optional<ROUTER_POINT> pinCenter;

            if( aAttachSmd )
            {
                for( int layer : { 0, aLayerCount - 1 } )
                {
                    for( const auto& pin : aPins )
                    {
                        if( pin.layer == layer && pin.drillAllowed
                            && shape.ContainsInside( PLANAR::POINT( pin.position ) ) )
                        {
                            pinCenter = pin.position;
                        }
                    }

                    if( pinCenter )
                        break;
                }
            }

            EXPANSION_DRILL drill;
            drill.generalFreeShape = shape;
            drill.freeShape = shape.BoundingOctagon().value_or(
                    PLANAR::INT_OCTAGON::Empty() );
            const auto center = shape.CentreOfGravity();
            drill.location = pinCenter.value_or(
                    FLOAT_POINT{ center.first, center.second }.Round() );
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

    std::vector<PLANAR::INT_OCTAGON> holes;
    std::optional<PLANAR::INT_OCTAGON> previous;
    for( const auto& entry : aObstacles )
    {
        if( aCancel && aCancel() )
            return nullptr;
        if( entry.isRoom || !entry.IsTraceObstacle( aNet )
            || !INT_BOX::Intersects( entry.shape, m_shape ) )
            continue;
        const PLANAR::INT_OCTAGON obstacle = entry.BoundingOctagon();
        if( !previous || !obstacle.IsContainedIn( *previous ) )
        {
            const PLANAR::INT_OCTAGON cutout = obstacle.Intersection(
                    PLANAR::INT_OCTAGON::FromBox( m_shape ) );
            if( cutout.Dimension() == 2 )
                holes.push_back( cutout );
        }
        previous = obstacle;
    }
    auto shapes = POLYLINE_AREA::SplitOctagonalToConvex(
            m_shape, holes, aCancel, aMaxPieces );
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
                        && shape.ContainsInside( pin.position ) )
                        pinCenter = pin.position;
                if( pinCenter )
                    break;
            }
        EXPANSION_DRILL drill;
        drill.freeShape = shape;
        const auto center = shape.CentreOfGravity();
        drill.location = pinCenter.value_or(
                FLOAT_POINT{ center.first, center.second }.Round() );
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
