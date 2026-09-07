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

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

/** Worker-side result event; BOARD mutation remains an editor-thread concern. */
struct BOARD_UPDATED_EVENT
{
    const ROUTING_RESULT* result = nullptr;
};

} // namespace KICAD_AUTOROUTER
