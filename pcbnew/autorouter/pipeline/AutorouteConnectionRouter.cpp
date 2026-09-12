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

#include "AutorouteConnectionRouter.h"

#include "../board/facade/RoutingBoard.h"
#include "../maze/AutorouteEngine.h"

namespace KICAD_AUTOROUTER
{

AUTOROUTE_CONNECTION_ROUTER::TERMINAL_SETS
AUTOROUTE_CONNECTION_ROUTER::TerminalSetsForItem( const ROUTING_BOARD& aBoard,
                                                   std::size_t aItemPad,
                                                   int aNetCode,
                                                   bool aContainsPlane )
{
    TERMINAL_SETS result;
    std::vector<ROUTING_TERMINAL> connected = aBoard.Terminals( aItemPad );
    std::vector<ROUTING_TERMINAL> unconnected;

    for( const ROUTING_BOARD::TARGET_ITEM& item :
         aBoard.UnconnectedTargetItems( aItemPad, aNetCode ) )
    {
        unconnected.insert( unconnected.end(), item.terminals.begin(),
                            item.terminals.end() );
    }

    // AutorouteConnectionRouter.route(): ordinary routing expands from the
    // complete unconnected set toward the selected item's connected set.
    // Plane routing intentionally reverses those sets so the path starts at
    // the selected item and terminates on the conduction area.
    if( aContainsPlane )
    {
        result.starts = std::move( connected );
        result.destinations = std::move( unconnected );
    }
    else
    {
        result.starts = std::move( unconnected );
        result.destinations = std::move( connected );
    }

    return result;
}

std::optional<ROUTING_CONNECTION> AUTOROUTE_CONNECTION_ROUTER::Route(
        const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
        const ROUTING_PAD& aSource, const ROUTING_PAD& aTarget, int aRetry,
        ROUTING_OCCUPANCY& aOccupancy, int& aExpandedNodes,
        const ROUTER_CANCEL_CALLBACK& aCancel )
{
    AUTOROUTE_ENGINE engine( aBoard, aSettings, aOccupancy );
    return engine.AutorouteConnection( aSource, aTarget, aRetry, aExpandedNodes, aCancel );
}

} // namespace KICAD_AUTOROUTER
