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

#include <optional>
#include <vector>

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

class MAZE_RIPUP_RESOLVER
{
public:
    /**
     * Select the least valuable previously routed connection to make room for
     * a higher-priority retry.  Net-class priority is preserved first; length
     * and route cost are used as deterministic tie breakers.  The pass-scaled
     * rip-up cost makes later passes more willing to evict long/congested
     * routes, matching Freerouting's startRipupCosts behavior.
     */
    std::optional<std::size_t> SelectVictim( const std::vector<ROUTING_CONNECTION>& aConnections,
                                             const std::vector<ROUTING_NET>& aNets,
                                             int aCurrentNet,
                                             int aRipupCost = 0 ) const;

private:
    int priorityFor( int aNetCode, const std::vector<ROUTING_NET>& aNets ) const;
    double lengthFor( const ROUTING_CONNECTION& aConnection ) const;
};

} // namespace KICAD_AUTOROUTER
