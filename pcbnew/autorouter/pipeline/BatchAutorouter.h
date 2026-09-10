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
 * Freerouting equivalent: autoroute/pipeline/BatchAutorouter.java and
 * autoroute/pipeline/AutoroutePassRunner.java.
 */

#pragma once

#include <vector>

#include "BatchOptimizer.h"
#include "../maze/AutorouteEngine.h"
#include "../maze/MazeRipupResolver.h"


namespace KICAD_AUTOROUTER
{

class BATCH_AUTOROUTER
{
public:
    ROUTING_RESULT Run( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
                        const ROUTER_CANCEL_CALLBACK& aCancel,
                        const ROUTER_PROGRESS_CALLBACK& aProgress ) const;

private:
    struct NET_ORDER_ENTRY
    {
        const ROUTING_NET* net = nullptr;
        double             halfPerimeter = 0.0;
    };

    std::vector<NET_ORDER_ENTRY> orderNets( const BOARD_SNAPSHOT& aBoard ) const;
    bool routeNet( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
                   const ROUTING_NET& aNet, int aRetry, ROUTING_OCCUPANCY& aOccupancy,
                   const AUTOROUTE_ENGINE& aEngine,
                   std::vector<ROUTING_CONNECTION>& aConnections, int& aExpandedNodes, int& aRipups,
                   const ROUTER_CANCEL_CALLBACK& aCancel,
                   const ROUTER_SEARCH_PROGRESS_CALLBACK& aSearchProgress,
                   std::size_t aPreferredPad = std::numeric_limits<std::size_t>::max(),
                   int aMaximumNewConnections = 0 ) const;

    void buildGeometry( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
                        const std::vector<ROUTING_CONNECTION>& aConnections,
                        ROUTING_RESULT& aResult ) const;

    const ROUTING_PAD* firstPad( const BOARD_SNAPSHOT& aBoard, const ROUTING_NET& aNet ) const;
    double netHalfPerimeter( const BOARD_SNAPSHOT& aBoard, const ROUTING_NET& aNet ) const;
};

} // namespace KICAD_AUTOROUTER
