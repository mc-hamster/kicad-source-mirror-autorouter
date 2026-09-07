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
 * Freerouting equivalent: board/facade/RoutingBoard and the board operation
 * facades.  The interface deliberately contains only snapshot operations;
 * worker-thread routing must never mutate a KiCad BOARD.
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
