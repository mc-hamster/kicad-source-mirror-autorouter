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

#include <optional>
#include <vector>

#include "../AutorouterTypes.h"
#include "../board/facade/RoutingBoard.h"


namespace KICAD_AUTOROUTER
{

/**
 * Source-order adapter for BatchOptimizer.ReadSortedRouteItems.
 *
 * Freerouting re-scans mutable Via and PolylineTrace items for every next()
 * call. Native optimization can currently remove only a whole
 * ROUTING_CONNECTION, so each connection is represented by its first eligible
 * source-style item and is optimized once per pass. The item key and via/
 * trace eligibility below are direct translations; item-local partial removal
 * remains a separate parity prerequisite.
 */
class READ_SORTED_ROUTE_ITEMS
{
public:
    struct KEY
    {
        ROUTER_POINT point;
        int          layer = -1;
        int          kind = 0; // 0 = via, 1 = trace
        ROUTING_BOARD::ITEM_ID item = 0;

        bool operator<( const KEY& aOther ) const;
    };

    static std::optional<KEY> Key( const ROUTING_BOARD& aBoard,
                                   const ROUTING_CONNECTION& aConnection );
    static void SortConnections( const ROUTING_BOARD& aBoard,
                                 std::vector<ROUTING_CONNECTION>& aConnections );
};

} // namespace KICAD_AUTOROUTER
