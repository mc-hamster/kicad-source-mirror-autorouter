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

/*
 * This program source code file is part of KiCad, a free EDA application.
 */

#include "AutorouteControl.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>


namespace KICAD_AUTOROUTER
{

double AUTOROUTE_CONTROL::ViaCost() const
{
    // Experimental retry penalty; this is not upstream padstack-radius-scaled
    // minNormalViaCost. See the core parity review before changing units.
    const int configuredCost = m_targetIsPlane && m_settings.planeViaCost > 0
                                       ? m_settings.planeViaCost
                                       : m_settings.viaCost;
    return static_cast<double>( std::max( 0, configuredCost ) )
           * ( 1.0 + 0.15 * static_cast<double>( m_retry ) );
}


double AUTOROUTE_CONTROL::TraceCost( double aLength ) const
{
    // Experimental grid-normalized units. Upstream instead uses geometric
    // distances weighted per axis and scales via costs by padstack radius.
    // These models must be replaced together, not described as equivalent.
    const double resolution = std::max( 1, m_settings.gridStepIU );
    return std::max( 0, m_settings.traceLengthCost ) * ( aLength / resolution );
}


double AUTOROUTE_CONTROL::CongestionCost( int aUsage ) const
{
    if( aUsage <= 0 )
        return 0.0;

    // Negotiated congestion: the first user owns a cell at its base cost;
    // subsequent users pay an increasing retry-scaled penalty.
    const double usage = static_cast<double>( aUsage );
    const double priorityBias = std::max(
            0.1, 1.0 + static_cast<double>( std::clamp( m_settings.routingPriority, -90, 1000 ) )
                               / 100.0 );
    const double retryBias = 1.0 + 0.20 * static_cast<double>( std::max( 0, m_retry ) );
    return std::max( 0, m_settings.congestionCost ) * priorityBias * retryBias * usage
           * usage;
}


double AUTOROUTE_CONTROL::DirectionCost( int aLayer, const ROUTER_POINT& aStart,
                                         const ROUTER_POINT& aEnd ) const
{
    auto layerIt = std::find_if( m_settings.layers.begin(), m_settings.layers.end(),
                                 [aLayer]( const ROUTER_LAYER_SETTINGS& aLayerSetting )
                                 {
                                     return aLayerSetting.layerId == aLayer;
                                 } );

    if( layerIt == m_settings.layers.end() || layerIt->preferredDirection == 0 )
        return 0.0;

    const bool horizontal = std::llabs( aEnd.x - aStart.x ) >= std::llabs( aEnd.y - aStart.y );
    const bool preferred = ( layerIt->preferredDirection == 1 && horizontal )
                           || ( layerIt->preferredDirection == 2 && !horizontal );

    return preferred
                   ? 0.0
                   : static_cast<double>( std::max( 0, layerIt->directionCost ) )
                                     * ( 1.0 + 0.05 * std::max( 0, m_retry ) );
}

} // namespace KICAD_AUTOROUTER
