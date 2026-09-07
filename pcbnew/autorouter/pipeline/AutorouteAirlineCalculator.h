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

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

/** Freerouting equivalent: autoroute/pipeline/AutorouteAirlineCalculator. */
class AUTOROUTE_AIRLINE_CALCULATOR
{
public:
    static double ConnectionLength( const BOARD_SNAPSHOT& aBoard,
                                    const ROUTING_CONNECTION& aConnection );
    static double TotalLength( const BOARD_SNAPSHOT& aBoard,
                               const std::vector<ROUTING_CONNECTION>& aConnections );
};

} // namespace KICAD_AUTOROUTER
