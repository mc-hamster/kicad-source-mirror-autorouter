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
#include <optional>

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

/** Backtracking record corresponding to Freerouting's MazeSearchElement. */
struct MAZE_SEARCH_ELEMENT
{
    ROUTER_NODE                node;
    std::optional<ROUTER_NODE> predecessor;
    double                     expansionValue = 0.0;
    double                     sortingValue = 0.0;
    std::size_t                sequence = 0;
    bool                       closed = false;
};

} // namespace KICAD_AUTOROUTER
