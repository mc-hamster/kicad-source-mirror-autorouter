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

#include <cstddef>

#include "AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

/** Per-item transient state; board objects never carry worker-thread state. */
struct ITEM_AUTOROUTE_INFO
{
    bool        startInfo = false;
    std::size_t precalculatedConnection = static_cast<std::size_t>( -1 );
    int         failureCount = 0;
};

} // namespace KICAD_AUTOROUTER
