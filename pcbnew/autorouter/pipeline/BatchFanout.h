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

/**
 * Freerouting-compatible fanout seam.
 *
 * Filled zones are represented as plane targets by the KiCad adapter, so
 * those connections are scheduled by BatchAutorouter.  This helper keeps
 * target classification in the matching upstream filename and provides a
 * safe extension point for a direct SMD-escape port.
 */
class BATCH_FANOUT
{
public:
    /**
     * Prepare the worker snapshot for Freerouting's SMD fanout pre-pass.
     *
     * A fanout is represented as an ordinary router connection from a
     * single-layer SMD pad to a synthetic landing pad on another enabled
     * copper layer.  The main batch stage then routes from that landing pad
     * to the rest of the net.  No KiCad object is created here; the resulting
     * via/trace is materialized by the normal proposal path.
     */
    static BOARD_SNAPSHOT PrepareSnapshot( const BOARD_SNAPSHOT& aBoard,
                                           const AUTOROUTER_SETTINGS& aSettings );

    static std::vector<std::size_t> PlaneTargetsFor( const BOARD_SNAPSHOT& aBoard,
                                                     const ROUTING_NET& aNet );
    static bool HasFanoutWork( const BOARD_SNAPSHOT& aBoard, const ROUTING_NET& aNet );
};

} // namespace KICAD_AUTOROUTER
