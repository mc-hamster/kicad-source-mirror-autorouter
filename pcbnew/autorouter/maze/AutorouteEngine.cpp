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
        m_search( aBoard, aSettings, aOccupancy ),
        m_drillPages( aBoard.bounds,
                      std::max<std::int64_t>(
                              10000,
                              aBoard.nets.empty()
                                      ? 10000
                                      : std::max<std::int64_t>( 10000,
                                                                aBoard.nets.front().viaDiameter
                                                                        * 5 ) ) )
{
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


void AUTOROUTE_ENGINE::Clear()
{
    m_drillPages.Reset();
}

} // namespace KICAD_AUTOROUTER
