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

#include <memory>

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

/** Immutable board-snapshot event replacing Freerouting's mutable event. */
struct BOARD_SNAPSHOT_EVENT
{
    std::shared_ptr<const BOARD_SNAPSHOT> snapshot;
};

} // namespace KICAD_AUTOROUTER
