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

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

/** Freerouting equivalent: autoroute/maze/MazeListElement. */
struct MAZE_LIST_ELEMENT
{
    ROUTER_NODE node;
    ROUTER_NODE backtrackNode;
    double      expansionValue = 0.0;
    double      sortingValue = 0.0;
    std::size_t sequence = 0;
    bool        isVia = false;
};


struct MAZE_LIST_ELEMENT_COMPARE
{
    bool operator()( const MAZE_LIST_ELEMENT& aLeft, const MAZE_LIST_ELEMENT& aRight ) const
    {
        if( aLeft.sortingValue != aRight.sortingValue )
            return aLeft.sortingValue > aRight.sortingValue;

        if( aLeft.expansionValue != aRight.expansionValue )
            return aLeft.expansionValue > aRight.expansionValue;

        return aLeft.sequence > aRight.sequence;
    }
};

} // namespace KICAD_AUTOROUTER
