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
 * Freerouting equivalent: autoroute/pipeline/RoutingPipeline.java.
 *
 * The pipeline is intentionally small.  It is the stable orchestration seam
 * used by the UI/job wrapper and is where future fanout, optimizer, and
 * diagnostic stages can be added without coupling them to KiCad.
 */

#pragma once

#include "BatchAutorouter.h"


namespace KICAD_AUTOROUTER
{

class ROUTING_PIPELINE
{
public:
    ROUTING_RESULT Run( const BOARD_SNAPSHOT& aBoard, const AUTOROUTER_SETTINGS& aSettings,
                        const ROUTER_CANCEL_CALLBACK& aCancel,
                        const ROUTER_PROGRESS_CALLBACK& aProgress ) const;
};

} // namespace KICAD_AUTOROUTER
