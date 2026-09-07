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

#include "FoundConnectionInserter.h"

#include <utility>

namespace KICAD_AUTOROUTER
{

void FOUND_CONNECTION_INSERTER::Append( const ROUTING_CONNECTION& aConnection,
                                        std::int64_t aTrackWidth,
                                        std::int64_t aViaDiameter,
                                        std::int64_t aViaDrill,
                                        const std::vector<int>& aViaLayers,
                                        ROUTING_RESULT& aResult )
{
    for( std::size_t index = 1; index < aConnection.nodes.size(); ++index )
    {
        const ROUTER_NODE& previous = aConnection.nodes[index - 1];
        const ROUTER_NODE& current = aConnection.nodes[index];

        AppendEdge( aConnection.netCode, previous, current, aTrackWidth, aViaDiameter, aViaDrill,
                    aViaLayers, aResult );
    }
}


void FOUND_CONNECTION_INSERTER::AppendEdge( int aNetCode, const ROUTER_NODE& aPrevious,
                                            const ROUTER_NODE& aCurrent,
                                            std::int64_t aTrackWidth,
                                            std::int64_t aViaDiameter,
                                            std::int64_t aViaDrill,
                                            const std::vector<int>& aViaLayers,
                                            ROUTING_RESULT& aResult )
{
    const ROUTER_NODE& previous = aPrevious;
    const ROUTER_NODE& current = aCurrent;

    if( previous.layer == current.layer )
    {
        if( previous.point != current.point )
        {
            aResult.segments.push_back( { aNetCode, previous.layer, previous.point, current.point,
                                          aTrackWidth } );
        }
        return;
    }

    ROUTING_VIA via;
    via.netCode = aNetCode;
    via.position = previous.point;
    via.topLayer = aViaLayers.empty() ? previous.layer : aViaLayers.front();
    via.bottomLayer = aViaLayers.empty() ? current.layer : aViaLayers.back();
    via.diameter = aViaDiameter;
    via.drill = aViaDrill;
    via.layers = aViaLayers;
    aResult.vias.push_back( std::move( via ) );
}

} // namespace KICAD_AUTOROUTER
