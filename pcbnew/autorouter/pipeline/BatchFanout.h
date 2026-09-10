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

#include <cstddef>
#include <vector>

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

/** Component/pin ordering and per-pin fanout sequencing follow pinned BatchFanout. */
class BATCH_FANOUT
{
public:
    static std::vector<std::size_t> OrderedPins( const BOARD_SNAPSHOT& aBoard,
                                                FANOUT_PIN_ORDER aOrder,
                                                const ROUTER_CANCEL_CALLBACK& aCancel = {} );
    /**
     * Prepare the worker snapshot for Freerouting's SMD fanout pre-pass.
     *
     * The synthetic pad is a control/item identity only.  It is initially
     * colocated with the real SMD pin; the room/drill frontier chooses the
     * first legal drill and the control item is then moved to that physical
     * endpoint.  No geometric landing is preselected here.
     */
    static BOARD_SNAPSHOT PrepareSnapshot( const BOARD_SNAPSHOT& aBoard,
                                           const AUTOROUTER_SETTINGS& aSettings,
                                           const ROUTER_CANCEL_CALLBACK& aCancel = {} );

    /** Ordered netclass ViaRule plus the optional board-rule fanout fallback. */
    static std::vector<ROUTING_VIA_PROFILE> ViaProfilesFor(
            const BOARD_SNAPSHOT& aBoard, int aNetCode,
            const AUTOROUTER_SETTINGS& aSettings );

    static std::vector<std::size_t> PlaneTargetsFor( const BOARD_SNAPSHOT& aBoard,
                                                     const ROUTING_NET& aNet );
    static bool HasFanoutWork( const BOARD_SNAPSHOT& aBoard, const ROUTING_NET& aNet );
};

} // namespace KICAD_AUTOROUTER
