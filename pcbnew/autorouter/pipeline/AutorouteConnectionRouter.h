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

#include "../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

class ROUTING_OCCUPANCY;
class ROUTING_BOARD;

/** Freerouting equivalent: autoroute/pipeline/AutorouteConnectionRouter. */
class AUTOROUTE_CONNECTION_ROUTER
{
public:
    struct TERMINAL_SETS
    {
        std::vector<ROUTING_TERMINAL> starts;
        std::vector<ROUTING_TERMINAL> destinations;
    };

    /** Translate Item.getConnectedSet()/getUnconnectedSet() into maze
     * terminals while preserving source item order and route direction.
     */
    static TERMINAL_SETS TerminalSetsForItem( const ROUTING_BOARD& aBoard,
                                               std::size_t aItemPad,
                                               int aNetCode,
                                               bool aContainsPlane );

    static std::optional<ROUTING_CONNECTION>
    Route( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
           const ROUTING_PAD& aSource, const ROUTING_PAD& aTarget, int aRetry,
           ROUTING_OCCUPANCY& aOccupancy, int& aExpandedNodes,
           const ROUTER_CANCEL_CALLBACK& aCancel );
};

} // namespace KICAD_AUTOROUTER
