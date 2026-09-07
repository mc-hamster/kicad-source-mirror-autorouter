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

#include "MazeRipupResolver.h"

#include <algorithm>
#include <limits>

#include "../path/Connection.h"


namespace KICAD_AUTOROUTER
{

namespace
{
} // namespace


int MAZE_RIPUP_RESOLVER::priorityFor( int aNetCode, const std::vector<ROUTING_NET>& aNets ) const
{
    auto it = std::find_if( aNets.begin(), aNets.end(),
                            [aNetCode]( const ROUTING_NET& aNet )
                            {
                                return aNet.netCode == aNetCode;
                            } );

    return it == aNets.end() ? 0 : it->netClassPriority;
}


double MAZE_RIPUP_RESOLVER::lengthFor( const ROUTING_CONNECTION& aConnection ) const
{
    return CONNECTION::FromRoute( aConnection ).TraceLength();
}


std::optional<std::size_t>
MAZE_RIPUP_RESOLVER::SelectVictim( const std::vector<ROUTING_CONNECTION>& aConnections,
                                   const std::vector<ROUTING_NET>& aNets, int aCurrentNet,
                                   int aRipupCost ) const
{
    std::optional<std::size_t> victim;
    int bestPriority = std::numeric_limits<int>::max();
    double highestValue = -1.0;

    for( std::size_t i = 0; i < aConnections.size(); ++i )
    {
        const ROUTING_CONNECTION& connection = aConnections[i];

        if( !connection.complete || connection.netCode == aCurrentNet )
            continue;

        const int priority = priorityFor( connection.netCode, aNets );
        const double length = lengthFor( connection );
        const double value = length
                             * ( 1.0 + std::max( 0, aRipupCost ) / 1000.0 )
                             + std::max( 0.0, connection.cost );

        if( !victim || priority < bestPriority
            || ( priority == bestPriority && value > highestValue ) )
        {
            victim = i;
            bestPriority = priority;
            highestValue = value;
        }
    }

    return victim;
}

} // namespace KICAD_AUTOROUTER
