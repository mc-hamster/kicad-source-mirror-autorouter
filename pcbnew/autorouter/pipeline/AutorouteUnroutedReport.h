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

#include <algorithm>
#include <vector>

#include "../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

/** Deterministic report of nets that still have incomplete connection edges. */
class AUTOROUTE_UNROUTED_REPORT
{
public:
    static std::vector<int> Build( const std::vector<ROUTING_NET>& aNets,
                                   const std::vector<ROUTING_CONNECTION>& aConnections )
    {
        std::vector<int> result;
        for( const ROUTING_NET& net : aNets )
        {
            const std::size_t routed = static_cast<std::size_t>( std::count_if(
                    aConnections.begin(), aConnections.end(),
                    [&net]( const ROUTING_CONNECTION& aConnection )
                    {
                        return aConnection.netCode == net.netCode && aConnection.complete;
                    } ) );
            if( !net.connections.empty() && routed < net.connections.size() )
                result.push_back( net.netCode );
        }
        return result;
    }
};

} // namespace KICAD_AUTOROUTER
