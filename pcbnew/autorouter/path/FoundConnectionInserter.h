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
#include <utility>

#include "../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

class MAZE_SEARCH_ENGINE;
class ROUTING_OCCUPANCY;

/** Checked, atomic insertion plus result emission. It supports incremental
 * trace-polyline insertion, terminal neckdown, fixed-obstacle spring-over,
 * and bounded recursive displacement of generated trace/via (including
 * generated fanout) copper plus supported source-via trace contacts. Full
 * host-board contact-graph mutation remains outside this data-only API.
 */
class FOUND_CONNECTION_INSERTER
{
public:
    enum class STATE { INSERTED, BLOCKED, CANCELLED, INVALID };
    struct RESULT
    {
        struct SHOVED_CONNECTION
        {
            ROUTING_CONNECTION original;
            ROUTING_CONNECTION replacement;
            // Static trace contacts of a moved source via. Their centreline
            // is unchanged, but they must be emitted as proposal copper when
            // the source BOARD_ITEM is removed.
            std::vector<ROUTING_CONNECTION_REPLACEMENT> materializedContacts;
            // DrillItem.moveBy() bridge traces from the old via centre to its
            // accepted location, one per unique contacted trace style.
            std::vector<ROUTING_CONNECTION> bridges;
        };

        STATE state;
        std::size_t edge = 0; // failed edge's end index; 0 for whole-input failure
        // Present only when insertion changed the path. Batch/result ownership
        // must publish this route, not the search proposal it replaced.
        std::optional<ROUTING_CONNECTION> connection = std::nullopt;
        // Existing generated routes relocated transactionally around this
        // connection. Batch storage replaces these records instead of treating
        // them as conventional rip-up victims.
        std::vector<SHOVED_CONNECTION> shoved;

        RESULT( STATE aState, std::size_t aEdge = 0,
                std::optional<ROUTING_CONNECTION> aConnection = std::nullopt ) :
                state( aState ),
                edge( aEdge ),
                connection( std::move( aConnection ) )
        {
        }
    };
    static RESULT Insert( const ROUTING_CONNECTION& aConnection,
                          const std::vector<ROUTING_CONNECTION>& aRipups,
                          ROUTING_OCCUPANCY& aOccupancy, const MAZE_SEARCH_ENGINE& aEngine,
                          const ROUTER_CANCEL_CALLBACK& aCancel = {},
                          bool aAllowRipupFallback = true );

    static void Append( const ROUTING_CONNECTION& aConnection,
                        std::int64_t aTrackWidth,
                        std::int64_t aViaDiameter,
                        std::int64_t aViaDrill,
                        const std::vector<int>& aViaLayers,
                        ROUTING_RESULT& aResult );

    static void AppendEdge( int aNetCode, const ROUTER_NODE& aPrevious,
                            const ROUTER_NODE& aCurrent, std::int64_t aTrackWidth,
                            std::int64_t aViaDiameter, std::int64_t aViaDrill,
                            const std::vector<int>& aViaLayers, ROUTING_RESULT& aResult,
                            std::int64_t aClearance = 0,
                            ROUTER_VIA_TYPE aViaType = ROUTER_VIA_TYPE::AUTO );
};

} // namespace KICAD_AUTOROUTER
