/*
 * This program source code file is part of KiCad, a free EDA application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#include "AutorouteEngine.h"

#include <algorithm>

#include "MazeSearchEngine.h"


namespace KICAD_AUTOROUTER
{

AUTOROUTE_ENGINE::AUTOROUTE_ENGINE( const BOARD_SNAPSHOT& aBoard,
                                    const AUTOROUTER_SETTINGS& aSettings,
                                    ROUTING_OCCUPANCY& aOccupancy ) :
        m_search( aBoard, aSettings, aOccupancy )
{
    // The active multilayer search owns its budgeted page/room lifetime.
    // The old wrapper allocated a second, unused array before cancellation
    // or work limits were checked (millions of pages for a default via size).
    // Persistent per-net invalidation/reuse is a separate parity milestone.
}


std::optional<ROUTING_CONNECTION> AUTOROUTE_ENGINE::AutorouteConnection(
        const ROUTING_PAD& aStart, const ROUTING_PAD& aTarget, int aRetry, int& aExpandedNodes,
        const ROUTER_CANCEL_CALLBACK& aCancel,
        const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress,
        const std::vector<ROUTING_TERMINAL>& aStarts,
        const std::vector<ROUTING_TERMINAL>& aTargets ) const
{
    return m_search.FindConnection( aStart, aTarget, aRetry, aExpandedNodes, aCancel, aProgress,
                                    aStarts, aTargets );
}


std::vector<ROUTING_CONNECTION> AUTOROUTE_ENGINE::FindConflictingConnections(
        const ROUTING_CONNECTION& aCandidate ) const
{
    return m_search.FindConflictingConnections( aCandidate );
}


} // namespace KICAD_AUTOROUTER
