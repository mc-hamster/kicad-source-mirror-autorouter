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
#include <utility>

#include "MazeSearchEngine.h"


namespace KICAD_AUTOROUTER
{

AUTOROUTE_ENGINE::AUTOROUTE_ENGINE( const BOARD_SNAPSHOT& aBoard,
                                    const AUTOROUTER_SETTINGS& aSettings,
                                    ROUTING_OCCUPANCY& aOccupancy,
                                    int aViaOverrideNetCode,
                                    std::optional<ROUTING_VIA_PROFILE> aViaOverride,
                                    int aTrackWidthOverrideNetCode,
                                    std::optional<std::int64_t> aTrackWidthOverride ) :
        m_search( aBoard, aSettings, aOccupancy, aViaOverrideNetCode,
                  std::move( aViaOverride ), aTrackWidthOverrideNetCode,
                  aTrackWidthOverride ),
        m_trackWidthOverrideNetCode( aTrackWidthOverrideNetCode ),
        m_trackWidthOverride( aTrackWidthOverride )
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
    auto connection = m_search.FindConnection( aStart, aTarget, aRetry, aExpandedNodes, aCancel,
                                               aProgress, aStarts, aTargets );

    // RouterSettings.neckWidthUm reruns a whole failed connection at a
    // narrower width.  The search engine uses that width for every geometry
    // predicate, but it must also reach the result model explicitly: an empty
    // edge-style vector would make proposal materialization silently restore
    // the ordinary net width.  Vias retain their selected/default padstack;
    // only same-layer trace edges are necked.
    if( connection && m_trackWidthOverride && m_trackWidthOverrideNetCode > 0
        && connection->netCode == m_trackWidthOverrideNetCode )
    {
        EnsureEdgeStyles( *connection );
        for( std::size_t edge = 1; edge < connection->nodes.size(); ++edge )
        {
            if( connection->nodes[edge - 1].layer == connection->nodes[edge].layer )
                connection->edgeStyles[edge - 1].trackWidth = *m_trackWidthOverride;
        }
    }

    return connection;
}


std::vector<ROUTING_CONNECTION> AUTOROUTE_ENGINE::FindConflictingConnections(
        const ROUTING_CONNECTION& aCandidate ) const
{
    return m_search.FindConflictingConnections( aCandidate );
}


} // namespace KICAD_AUTOROUTER
