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

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

/**
 * Data-only final rule check for a proposed result.
 *
 * The KiCad DRC engine remains authoritative once the proposal is committed.
 * This checker deliberately runs on the immutable worker snapshot so that the
 * preview can reject obvious clearance/edge conflicts before the editor is
 * mutated.  It mirrors the collision model used by MazeSearchEngine and also
 * checks the materialized result as a second, independent pass.
 */
class DESIGN_RULES_CHECKER
{
public:
    static int CountViolations( const BOARD_SNAPSHOT& aBoard,
                                const AUTOROUTER_SETTINGS& aSettings,
                                const ROUTING_RESULT& aResult );
};

} // namespace KICAD_AUTOROUTER
