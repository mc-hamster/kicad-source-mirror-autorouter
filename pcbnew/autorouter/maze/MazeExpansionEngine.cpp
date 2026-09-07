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

#include "MazeExpansionEngine.h"

#include <algorithm>


namespace KICAD_AUTOROUTER
{

std::vector<ROUTER_NODE> MAZE_EXPANSION_ENGINE::Neighbours(
        const ROUTER_NODE& aNode, std::int64_t aGridStep,
        const std::vector<ROUTER_LAYER_SETTINGS>& aLayers, bool aAllowVias )
{
    const std::int64_t step = std::max<std::int64_t>( 1, aGridStep );

    // The order is deliberate.  It is stable across runs and follows the
    // orthogonal/45-degree preference used by the Freerouting search engines.
    static constexpr int directions[][2] = {
        { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 },
        { 1, 1 }, { -1, 1 }, { -1, -1 }, { 1, -1 }
    };

    std::vector<ROUTER_NODE> result;
    result.reserve( 8 + aLayers.size() );

    for( const auto& direction : directions )
    {
        result.push_back( { { aNode.point.x + direction[0] * step,
                             aNode.point.y + direction[1] * step },
                            aNode.layer } );
    }

    if( aAllowVias )
    {
        for( const ROUTER_LAYER_SETTINGS& layer : aLayers )
        {
            if( layer.enabled && layer.layerId != aNode.layer )
                result.push_back( { aNode.point, layer.layerId } );
        }
    }

    return result;
}

} // namespace KICAD_AUTOROUTER
