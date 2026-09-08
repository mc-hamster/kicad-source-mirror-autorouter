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
#include <map>
#include <vector>

#include "../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

/** Deterministic report of nets that still have incomplete connection edges. */
class AUTOROUTE_UNROUTED_REPORT
{
public:
    // Count unresolved requests using surviving pad connectivity, not route
    // record counts. Rebuild on every call so ripping up a bridge invalidates
    // every request that depended on it. This remains a pad-terminal model;
    // final electrical acceptance must use the materialized KiCad board.
    static int CountMissing( const ROUTING_NET& aNet,
                             const std::vector<ROUTING_CONNECTION>& aConnections )
    {
        std::map<std::size_t, std::size_t> parent;
        auto root = [&]( std::size_t index )
        {
            parent.try_emplace( index, index );
            while( parent[index] != index )
            {
                parent[index] = parent[parent[index]];
                index = parent[index];
            }
            return index;
        };
        auto unite = [&]( std::size_t left, std::size_t right )
        {
            const auto leftRoot = root( left );
            parent[root( right )] = leftRoot;
        };
        for( const auto& group : aNet.connectedPadGroups )
            for( std::size_t index : group )
                unite( group.front(), index );
        for( const auto& connection : aConnections )
        {
            if( connection.netCode == aNet.netCode && connection.complete
                && connection.nodes.size() >= 2
                && connection.fromPadIndex != std::numeric_limits<std::size_t>::max()
                && connection.toPadIndex != std::numeric_limits<std::size_t>::max() )
            {
                unite( connection.fromPadIndex, connection.toPadIndex );
            }
        }
        int missing = 0;
        for( const auto& [source, target] : aNet.connections )
        {
            bool connected = root( source ) == root( target );
            // The legacy plane model permits an alternative sampled target.
            // Do not globally union plane samples: they can be separate islands.
            if( !connected && std::find( aNet.planeTargetIndices.begin(),
                    aNet.planeTargetIndices.end(), target ) != aNet.planeTargetIndices.end() )
            {
                connected = std::any_of( aNet.planeTargetIndices.begin(),
                        aNet.planeTargetIndices.end(),
                        [&]( std::size_t plane ) { return root( source ) == root( plane ); } );
            }
            if( !connected )
                ++missing;
        }
        return missing;
    }

    static std::vector<int> Build( const std::vector<ROUTING_NET>& aNets,
                                   const std::vector<ROUTING_CONNECTION>& aConnections )
    {
        std::vector<int> result;
        for( const ROUTING_NET& net : aNets )
        {
            if( CountMissing( net, aConnections ) > 0 )
                result.push_back( net.netCode );
        }
        return result;
    }
};

} // namespace KICAD_AUTOROUTER
