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

#pragma once

#include <cstdint>
#include <optional>

#include "../AutorouterTypes.h"
#include "MazeSearchEngine.h"
#include "../path/FoundConnectionInserter.h"


namespace KICAD_AUTOROUTER
{

class ROUTING_OCCUPANCY;

/** Per-snapshot maze owner corresponding to Freerouting's AutorouteEngine. */
class AUTOROUTE_ENGINE
{
public:
    /**
     * aViaOverride models one selected ViaRule candidate for an isolated
     * fanout task.  It is intentionally not a global settings mutation:
     * separate SMD pins on the same net may escape with different valid
     * profiles from the source rule's ordered alternatives.
     */
    AUTOROUTE_ENGINE( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
                      ROUTING_OCCUPANCY& aOccupancy, int aViaOverrideNetCode = 0,
                      std::optional<ROUTING_VIA_DIMENSION> aViaOverride = std::nullopt,
                      int aTrackWidthOverrideNetCode = 0,
                      std::optional<std::int64_t> aTrackWidthOverride = std::nullopt );

    std::optional<ROUTING_CONNECTION>
    AutorouteConnection( const ROUTING_PAD& aStart, const ROUTING_PAD& aTarget, int aRetry,
                         int& aExpandedNodes, const ROUTER_CANCEL_CALLBACK& aCancel,
                         const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress = {},
                         const std::vector<ROUTING_TERMINAL>& aStarts = {},
                         const std::vector<ROUTING_TERMINAL>& aTargets = {} ) const;

    std::vector<ROUTING_CONNECTION> FindConflictingConnections(
            const ROUTING_CONNECTION& aCandidate ) const;

    FOUND_CONNECTION_INSERTER::RESULT InsertConnection(
            const ROUTING_CONNECTION& aConnection, const std::vector<ROUTING_CONNECTION>& aRipups,
            ROUTING_OCCUPANCY& aOccupancy, const ROUTER_CANCEL_CALLBACK& aCancel = {},
            bool aAllowRipupFallback = true ) const
    {
        return FOUND_CONNECTION_INSERTER::Insert( aConnection, aRipups, aOccupancy, m_search,
                                                  aCancel, aAllowRipupFallback );
    }

private:
    MAZE_SEARCH_ENGINE         m_search;
    int                        m_trackWidthOverrideNetCode = 0;
    std::optional<std::int64_t> m_trackWidthOverride;
};

} // namespace KICAD_AUTOROUTER
