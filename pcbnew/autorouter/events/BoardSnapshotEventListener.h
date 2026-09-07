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

#include <functional>

#include "BoardSnapshotEvent.h"


namespace KICAD_AUTOROUTER
{

using BOARD_SNAPSHOT_EVENT_LISTENER = std::function<void( const BOARD_SNAPSHOT_EVENT& )>;

} // namespace KICAD_AUTOROUTER
