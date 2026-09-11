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
 * Freerouting equivalent: autoroute/pipeline/BatchOptimizer and
 * board/optimize/TraceTightener/ViaOptimizer.
 */

#pragma once

#include <vector>

#include "../maze/MazeSearchEngine.h"


namespace KICAD_AUTOROUTER
{

/** Source BoardStatistics.traces.totalWeightedLength contribution for one
 * native connection.  Exposed so parity QA can pin the optimizer's otherwise
 * easy-to-confuse weighted/pass-floor accounting. */
double OptimizerWeightedTraceLength( const ROUTING_CONNECTION& aConnection,
                                     const BOARD_SNAPSHOT& aBoard );


class BATCH_OPTIMIZER
{
public:
    BATCH_OPTIMIZER( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
                     ROUTING_OCCUPANCY& aOccupancy ) :
            m_board( aBoard ),
            m_settings( aSettings ),
            m_occupancy( aOccupancy )
    {
    }

    int Optimize( std::vector<ROUTING_CONNECTION>& aConnections,
                  const ROUTER_CANCEL_CALLBACK& aCancel ) const;

    void RemoveRedundantViaTails( std::vector<ROUTING_CONNECTION>& aConnections,
                                  const ROUTER_CANCEL_CALLBACK& aCancel,
                                  int aOnlyNetCode = 0 ) const;

private:
    void removeTraceTails( std::vector<ROUTING_CONNECTION>& aConnections,
                           const ROUTER_CANCEL_CALLBACK& aCancel,
                           int aOnlyNetCode ) const;

    void simplifyConnection( ROUTING_CONNECTION& aConnection,
                              const MAZE_SEARCH_ENGINE& aSearch ) const;

private:
    const BOARD_SNAPSHOT&      m_board;
    const AUTOROUTER_SETTINGS& m_settings;
    ROUTING_OCCUPANCY&         m_occupancy;
};

} // namespace KICAD_AUTOROUTER
