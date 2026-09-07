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

#include <cstdint>
#include <vector>

#include "../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

/**
 * Visibility landmark graph used by the native maze search.
 *
 * Freerouting builds expansion rooms and doors from the free-space search
 * tree.  This adapter keeps the same responsibility in a data-only class:
 * obstacle corners, terminal points, board boundaries, and drill-page centers
 * become deterministic graph landmarks.  The maze frontier remains available
 * as the complete fallback, so this graph is an accelerator rather than a
 * reduced-grid approximation.
 */
class EXPANSION_GRAPH
{
public:
    static std::vector<ROUTER_NODE>
    BuildLandmarks( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
                    const ROUTING_PAD& aStart, const ROUTING_PAD& aTarget,
                    std::int64_t aTrackRadius, std::int64_t aViaRadius, int aNetCode );
};

} // namespace KICAD_AUTOROUTER
