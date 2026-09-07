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

#include <string>

#include "../pipeline/TaskState.h"


namespace KICAD_AUTOROUTER
{

struct TASK_STATE_CHANGED_EVENT
{
    TASK_STATE  state = TASK_STATE::IDLE;
    int         passNumber = 0;
    std::string boardHash;
};

} // namespace KICAD_AUTOROUTER
