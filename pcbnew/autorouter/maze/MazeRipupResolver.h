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
 * Freerouting equivalent: autoroute/maze/MazeRipupResolver.java.
 */

#pragma once

#include <cstddef>
#include <vector>

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

class CONNECTION;

class MAZE_RIPUP_RESOLVER
{
public:
    static constexpr int ALREADY_RIPPED_COST = 1;

    struct CONTEXT
    {
        int  ripupCosts = 0;
        int  startRipupCosts = 0;
        int  ripupPassNo = 1;
        bool isFanout = false;
        bool removeUnconnectedVias = false;
    };

    /** Source-shaped MazeRipupResolver.checkRipup cost for one native item.
     *
     * aEdgeIndex identifies the tree shape which created the obstacle room.
     * The complete connection supplies Connection.getDetour() and adjacent
     * trace contacts for a via.  aFallbackTraceHalfWidth is used only when the
     * edge has no explicit style.  aRandomNumber is Java Random.nextDouble()
     * for randomized passes and is ignored on all other passes.
     */
    int CheckRipup( const ROUTING_CONNECTION& aConnection, std::size_t aEdgeIndex,
                    std::int64_t aFallbackTraceHalfWidth, const CONTEXT& aContext,
                    double aRandomNumber = 0.0,
                    const std::vector<std::int64_t>& aAdditionalViaTraceHalfWidths = {},
                    const CONNECTION* aTopologyConnection = nullptr ) const;

    static double FanoutViaRipupCostFactor( std::int64_t aTraceHalfWidth,
                                            double aTraceLength );
};

} // namespace KICAD_AUTOROUTER
