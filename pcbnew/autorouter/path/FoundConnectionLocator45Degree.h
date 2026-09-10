/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "../geometry/planar/FloatLine.h"
#include "../geometry/planar/IntOctagon.h"

namespace KICAD_AUTOROUTER
{

struct RECTANGULAR_CORRIDOR_STEP
{
    ROUTER_BOX room;                 // Already compensated trace-centre space.
    std::optional<ROUTER_BOX> door;   // Absent for the final target point.
    FLOAT_LINE section;
};

struct OCTAGONAL_CORRIDOR_STEP
{
    PLANAR::INT_OCTAGON room;
    std::optional<PLANAR::INT_OCTAGON> door;
    FLOAT_LINE section;
};

/**
 * Active rectangular subset of the reference 90/45-degree locator. This is
 * not the general convex, thin-room or pad-neckdown locator. The multilayer
 * maze composes these same-layer corridors with physical through drills.
 */
class FOUND_CONNECTION_LOCATOR_45_DEGREE
{
public:
    static FLOAT_POINT CalculateAdditionalCorner( FLOAT_POINT aFrom, FLOAT_POINT aTo,
                                                   bool aHorizontalFirst, bool aOrthogonal );
    static std::optional<std::vector<ROUTER_POINT>> LocateRectangular(
            ROUTER_POINT aStart, const std::vector<RECTANGULAR_CORRIDOR_STEP>& aSteps,
            bool aOrthogonal );
    static std::optional<std::vector<ROUTER_POINT>> LocateOctagonal(
            ROUTER_POINT aStart, const std::vector<OCTAGONAL_CORRIDOR_STEP>& aSteps );
};

} // namespace KICAD_AUTOROUTER
