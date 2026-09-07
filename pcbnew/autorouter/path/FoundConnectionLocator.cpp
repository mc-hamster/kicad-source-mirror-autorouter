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

#include "FoundConnectionLocator.h"

#include "../maze/MazeSearchEngine.h"

namespace KICAD_AUTOROUTER
{

std::optional<ROUTING_CONNECTION> FOUND_CONNECTION_LOCATOR::Locate(
        const MAZE_SEARCH_ENGINE& aSearch, const ROUTING_PAD& aStart,
        const ROUTING_PAD& aTarget, int aRetry, int& aExpandedNodes,
        const ROUTER_CANCEL_CALLBACK& aCancel )
{
    return aSearch.FindConnection( aStart, aTarget, aRetry, aExpandedNodes, aCancel );
}

} // namespace KICAD_AUTOROUTER
