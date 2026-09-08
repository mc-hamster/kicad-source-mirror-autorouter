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
    AUTOROUTE_ENGINE( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
                      ROUTING_OCCUPANCY& aOccupancy );

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
            ROUTING_OCCUPANCY& aOccupancy, const ROUTER_CANCEL_CALLBACK& aCancel = {} ) const
    {
        return FOUND_CONNECTION_INSERTER::Insert( aConnection, aRipups, aOccupancy, m_search, aCancel );
    }

private:
    MAZE_SEARCH_ENGINE         m_search;
};

} // namespace KICAD_AUTOROUTER
