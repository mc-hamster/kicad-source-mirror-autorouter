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

/*
 * KiCad input-capture boundary, NOT a port of Freerouting RoutingBoard.
 * Worker-thread routing must never mutate the live KiCad BOARD, but the
 * core still needs its own mutable items, connectivity, search tree and undo.
 */

#pragma once

#include <memory>

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

class ROUTING_BOARD_INTERFACE
{
public:
    virtual ~ROUTING_BOARD_INTERFACE() = default;

    /**
     * Capture all board data needed by the routing engine.
     *
     * The caller owns the returned immutable value and may use it from a
     * worker thread after this call returns.
     */
    virtual std::shared_ptr<const BOARD_SNAPSHOT>
            CreateSnapshot( const AUTOROUTER_SETTINGS& aSettings ) const = 0;
};

} // namespace KICAD_AUTOROUTER
