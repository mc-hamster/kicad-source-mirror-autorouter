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

#include <vector>

#include "BatchOptimizer.h"


namespace KICAD_AUTOROUTER
{

/** Legacy serial delegate. The reference's parallel optimizer is NOT ported.
 * Retaining its filename does not supply its candidate scheduling or behavior.
 */
class BATCH_OPTIMIZER_MULTI_THREADED
{
public:
    BATCH_OPTIMIZER_MULTI_THREADED( const BOARD_SNAPSHOT& aBoard,
                                    const AUTOROUTER_SETTINGS& aSettings,
                                    ROUTING_OCCUPANCY& aOccupancy ) :
            m_optimizer( aBoard, aSettings, aOccupancy )
    {
    }

    int Optimize( std::vector<ROUTING_CONNECTION>& aConnections,
                  const ROUTER_CANCEL_CALLBACK& aCancel ) const
    {
        return m_optimizer.Optimize( aConnections, aCancel );
    }

private:
    BATCH_OPTIMIZER m_optimizer;
};

} // namespace KICAD_AUTOROUTER
