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
#include "../geometry/planar/Simplex.h"

namespace KICAD_AUTOROUTER
{

struct GENERAL_CORRIDOR_STEP
{
    PLANAR::SIMPLEX room;
    std::optional<PLANAR::SIMPLEX> door;
    FLOAT_LINE section;
};

/** Realizer for a backtracked unrestricted-angle room corridor.
 *
 * Unlike the former alias to the legacy grid search, this operates on exact
 * rational room and door shapes.  It chooses only integral points proven to
 * lie in the relevant SIMPLEX; a rational passage with no legal KiCad lattice
 * point fails closed rather than rounding through a compensated obstacle.
 */
class FOUND_CONNECTION_LOCATOR_ANY_ANGLE
{
public:
    static std::optional<ROUTER_POINT> NearestIntegralPoint(
            const PLANAR::SIMPLEX& aShape, ROUTER_POINT aFrom );

    static std::optional<std::vector<ROUTER_POINT>> Locate(
            ROUTER_POINT aStart, const std::vector<GENERAL_CORRIDOR_STEP>& aSteps );
};

} // namespace KICAD_AUTOROUTER
