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

#include "BatchOptimizer.h"

#include <algorithm>

#include "../maze/MazeTraceShover.h"


namespace KICAD_AUTOROUTER
{

void BATCH_OPTIMIZER::simplifyConnection( ROUTING_CONNECTION& aConnection,
                                          const MAZE_SEARCH_ENGINE& aSearch ) const
{
    MAZE_TRACE_SHOVER::Shorten( aConnection, aSearch );
}


int BATCH_OPTIMIZER::Optimize( std::vector<ROUTING_CONNECTION>& aConnections,
                               const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    if( !m_settings.optimizeAfterComplete )
        return 0;

    MAZE_SEARCH_ENGINE search( m_board, m_settings, m_occupancy );
    int completedPasses = 0;
    int optimizedItems = 0;

    for( int pass = 0; pass < std::max( 0, m_settings.optimizationPasses ); ++pass )
    {
        if( aCancel && aCancel() )
            break;

        for( ROUTING_CONNECTION& connection : aConnections )
        {
            if( aCancel && aCancel() )
                break;

            if( m_settings.maxOptimizationItems > 0
                && optimizedItems >= m_settings.maxOptimizationItems )
            {
                return completedPasses;
            }

            if( connection.complete )
            {
                const ROUTING_CONNECTION oldConnection = connection;
                m_occupancy.Remove( oldConnection );
                simplifyConnection( connection, search );
                m_occupancy.Add( connection );
                ++optimizedItems;
            }
        }

        ++completedPasses;
    }

    return completedPasses;
}

} // namespace KICAD_AUTOROUTER
