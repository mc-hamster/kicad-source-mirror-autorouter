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

#include <set>
#include <vector>

#include "../../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

class MAZE_SEARCH_ENGINE;
class ROUTING_BOARD;

/**
 * Via-location candidates translated from Freerouting's
 * board/optimize/ViaOptimizer.
 *
 * The source mutates a Via and the one or two PolylineTrace items contacting
 * it, then pulls those traces tight.  Native worker copper stores that same
 * trace-via-trace chain in a ROUTING_CONNECTION, so a candidate moves the
 * coincident layer-transition nodes and revalidates every resulting edge.
 * Exactly representable two-trace and isolated one-trace fanout/plane cases
 * are handled here.  Arbitrary item contact graphs deliberately fail closed.
 */
class VIA_OPTIMIZER
{
public:
    using VIA_EDGE_SET = std::set<std::size_t>;

    /**
     * Capture Via.getNormalContacts()-equivalent eligibility while the route
     * is still present in the worker board.  Candidate validation happens
     * after the connection has been removed, so deriving this information in
     * Candidates() would lose external contacts and could strand a branch.
     */
    static VIA_EDGE_SET MovableViaEdges( const ROUTING_CONNECTION& aConnection,
                                         const BOARD_SNAPSHOT& aBoard,
                                         const ROUTING_BOARD& aRoutingBoard );

    static std::vector<ROUTING_CONNECTION> Candidates(
            const ROUTING_CONNECTION& aConnection, const BOARD_SNAPSHOT& aBoard,
            const AUTOROUTER_SETTINGS& aSettings, const ROUTING_BOARD& aRoutingBoard,
            const MAZE_SEARCH_ENGINE& aSearch, const VIA_EDGE_SET& aMovableViaEdges,
            const ROUTER_CANCEL_CALLBACK& aCancel = {} );
};

} // namespace KICAD_AUTOROUTER
