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
#include <tuple>

#include "../geometry/planar/FloatLine.h"


namespace KICAD_AUTOROUTER
{

/** Freerouting a11c0a42 frontier ordering key; backtracking lives with each state. */
struct MAZE_LIST_ELEMENT
{
    double      expansionValue = 0.0;
    double      sortingValue = 0.0;
    int         doorId = 0;
    std::size_t sectionNoOfDoor = 0;

    auto SortKey() const
    {
        return std::tuple{ sortingValue, expansionValue, doorId, sectionNoOfDoor };
    }

    static double BendPenalty( FLOAT_POINT aPreviousDoorCentre, FLOAT_POINT aFrom,
                                FLOAT_POINT aTo, double aCost )
    {
        const double px = aFrom.x - aPreviousDoorCentre.x, py = aFrom.y - aPreviousDoorCentre.y;
        const double nx = aTo.x - aFrom.x, ny = aTo.y - aFrom.y;
        const double cross = px * ny - py * nx;
        const double previousLength = px * px + py * py, nextLength = nx * nx + ny * ny;
        return previousLength > 0 && nextLength > 0
                       && cross * cross > 0.01 * previousLength * nextLength ? aCost : 0;
    }
};


struct MAZE_LIST_ELEMENT_COMPARE
{
    bool operator()( const MAZE_LIST_ELEMENT& aLeft, const MAZE_LIST_ELEMENT& aRight ) const
    {
        return aLeft.SortKey() > aRight.SortKey();
    }
};

} // namespace KICAD_AUTOROUTER
