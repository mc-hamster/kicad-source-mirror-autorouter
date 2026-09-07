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

#include <functional>

#include "AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

/** Headless diagnostic snapshot; GUI consumers can render it without entering the worker. */
struct AUTOROUTE_DIAGNOSTIC
{
    enum class KIND
    {
        FREE_SPACE_ROOM,
        OBSTACLE_ROOM,
        EXPANSION_DRILL
    };

    KIND         kind = KIND::FREE_SPACE_ROOM;
    ROUTER_BOX   shape;
    int          layer = -1;
    double       intensity = 0.0;
};

using AUTOROUTE_DIAGNOSTIC_SINK = std::function<void( const AUTOROUTE_DIAGNOSTIC& )>;

} // namespace KICAD_AUTOROUTER
