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

#include "MazeRipupResolver.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../path/Connection.h"


namespace KICAD_AUTOROUTER
{

double MAZE_RIPUP_RESOLVER::FanoutViaRipupCostFactor(
        std::int64_t aTraceHalfWidth, double aTraceLength )
{
    if( aTraceHalfWidth <= 0 || aTraceLength <= 0 )
        return 1.0;

    constexpr double fanoutCostConstant = 20000.0;
    const double ratio = static_cast<double>( aTraceHalfWidth ) / aTraceLength;
    return std::max( ratio * ratio * fanoutCostConstant, 1.0 );
}


int MAZE_RIPUP_RESOLVER::CheckRipup( const ROUTING_CONNECTION& aConnection,
                                     std::size_t aEdgeIndex,
                                     std::int64_t aFallbackTraceHalfWidth,
                                     const CONTEXT& aContext,
                                     double aRandomNumber,
                                     const std::vector<std::int64_t>&
                                             aAdditionalViaTraceHalfWidths ) const
{
    if( !aConnection.complete || !HasValidEdgeStyles( aConnection )
        || aEdgeIndex + 1 >= aConnection.nodes.size() )
    {
        return -1;
    }

    const ROUTER_NODE& from = aConnection.nodes[aEdgeIndex];
    const ROUTER_NODE& to = aConnection.nodes[aEdgeIndex + 1];
    const bool isVia = from.layer != to.layer;
    const auto edgeHalfWidth = [&]( std::size_t aEdge )
    {
        const ROUTING_EDGE_STYLE& edgeStyle = EdgeStyle( aConnection, aEdge );
        return std::max<std::int64_t>(
                1, edgeStyle.trackWidth > 0 ? edgeStyle.trackWidth / 2
                                            : aFallbackTraceHalfWidth );
    };

    double costFactor = 1.0;
    double fanoutFactor = 1.0;
    const bool preserveFanoutProtection =
            !aContext.removeUnconnectedVias
            && aContext.ripupCosts <= aContext.startRipupCosts * 2;
    if( !isVia )
    {
        costFactor = static_cast<double>( edgeHalfWidth( aEdgeIndex ) );
        if( preserveFanoutProtection && aConnection.isFanoutConnection )
        {
            const long double dx = static_cast<long double>( to.point.x ) - from.point.x;
            const long double dy = static_cast<long double>( to.point.y ) - from.point.y;
            fanoutFactor = FanoutViaRipupCostFactor(
                    edgeHalfWidth( aEdgeIndex ),
                    std::sqrt( static_cast<double>( dx * dx + dy * dy ) ) );
        }
    }
    else
    {
        int contactCount = 0;
        if( aEdgeIndex > 0
            && aConnection.nodes[aEdgeIndex - 1].layer
                       == aConnection.nodes[aEdgeIndex].layer )
        {
            ++contactCount;
            costFactor = std::max( costFactor,
                                   static_cast<double>( edgeHalfWidth( aEdgeIndex - 1 ) ) );
            if( preserveFanoutProtection && aConnection.isFanoutConnection
                && !aContext.isFanout )
            {
                const CONNECTION connection = CONNECTION::FromRoute( aConnection );
                fanoutFactor = FanoutViaRipupCostFactor(
                        edgeHalfWidth( aEdgeIndex - 1 ), connection.TraceLength() );
            }
        }
        if( aEdgeIndex + 1 < aConnection.nodes.size() - 1
            && aConnection.nodes[aEdgeIndex + 1].layer
                       == aConnection.nodes[aEdgeIndex + 2].layer )
        {
            ++contactCount;
            costFactor = std::max( costFactor,
                                   static_cast<double>( edgeHalfWidth( aEdgeIndex + 1 ) ) );
        }
        for( const std::int64_t halfWidth : aAdditionalViaTraceHalfWidths )
        {
            ++contactCount;
            costFactor = std::max( costFactor,
                                   static_cast<double>( std::max<std::int64_t>( 1,
                                                                                halfWidth ) ) );
        }
        if( fanoutFactor <= 1.0 )
            costFactor *= 0.5 * std::max( contactCount - 1, 0 );
    }

    double detour = 1.0;
    if( fanoutFactor <= 1.0 && !aContext.isFanout )
    {
        const CONNECTION connection = CONNECTION::FromRoute( aConnection );
        detour = std::max( connection.Detour(), 1e-12 );
    }

    if( aContext.ripupPassNo >= 4 && aContext.ripupPassNo % 3 != 0 )
    {
        const double random = std::clamp( aRandomNumber, 0.0, 1.0 );
        detour *= 0.5 + random * random;
    }

    double cost = std::max( 0, aContext.ripupCosts ) * costFactor / detour
                  * fanoutFactor;
    constexpr int maximum = std::numeric_limits<int>::max() / 100;
    if( !std::isfinite( cost ) || cost >= maximum )
        return maximum;
    return std::clamp( static_cast<int>( cost ), 1, maximum );
}

} // namespace KICAD_AUTOROUTER
