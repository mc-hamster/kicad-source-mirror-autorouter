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

#include <cstdint>

#include "../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

class MAZE_SEARCH_ENGINE;
class ROUTING_OCCUPANCY;

/** Checked, atomic insertion of the current straight-segment/through-via
 * subset, plus result emission. Recursive forced shove, spring-over and
 * neckdown from Java FoundConnectionInserter are NOT implemented by this API.
 */
class FOUND_CONNECTION_INSERTER
{
public:
    enum class STATE { INSERTED, BLOCKED, CANCELLED, INVALID };
    struct RESULT
    {
        STATE state;
        std::size_t edge = 0; // failed edge's end index; 0 for whole-input failure
    };
    static RESULT Insert( const ROUTING_CONNECTION& aConnection,
                          const std::vector<ROUTING_CONNECTION>& aRipups,
                          ROUTING_OCCUPANCY& aOccupancy, const MAZE_SEARCH_ENGINE& aEngine,
                          const ROUTER_CANCEL_CALLBACK& aCancel = {} );

    static void Append( const ROUTING_CONNECTION& aConnection,
                        std::int64_t aTrackWidth,
                        std::int64_t aViaDiameter,
                        std::int64_t aViaDrill,
                        const std::vector<int>& aViaLayers,
                        ROUTING_RESULT& aResult );

    static void AppendEdge( int aNetCode, const ROUTER_NODE& aPrevious,
                            const ROUTER_NODE& aCurrent, std::int64_t aTrackWidth,
                            std::int64_t aViaDiameter, std::int64_t aViaDrill,
                            const std::vector<int>& aViaLayers, ROUTING_RESULT& aResult );
};

} // namespace KICAD_AUTOROUTER
