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
 * Freerouting equivalent: autoroute/maze/MazeExpansionEngine.java.
 *
 * The native search stores expansion nodes as immutable layer-aware cells.  This
 * helper owns the deterministic neighbor policy so that future upstream changes
 * to expansion ordering have a one-file synchronization point.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

class MAZE_EXPANSION_ENGINE
{
public:
    static std::vector<ROUTER_NODE>
            Neighbours( const ROUTER_NODE& aNode, std::int64_t aGridStep,
                        const std::vector<ROUTER_LAYER_SETTINGS>& aLayers, bool aAllowVias );
};

} // namespace KICAD_AUTOROUTER
