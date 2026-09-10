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

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

class MAZE_SEARCH_ENGINE;

/** Contact-preserving line-of-sight pull-tight used by the batch optimizer.
 * Movable-neighbour displacement is handled by the checked forced inserter;
 * this local operation only accepts strict, immediately insertable copper.
 */
class MAZE_TRACE_SHOVER
{
public:
    static bool Shorten( ROUTING_CONNECTION& aConnection, const MAZE_SEARCH_ENGINE& aSearch );
};

} // namespace KICAD_AUTOROUTER
