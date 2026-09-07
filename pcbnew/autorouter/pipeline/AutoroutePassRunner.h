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

#include "AutorouteBatchLoop.h"
#include "TaskState.h"

namespace KICAD_AUTOROUTER
{

/** State seam for one pass of BatchAutorouter, kept separate for future parallel backends. */
class AUTOROUTE_PASS_RUNNER
{
public:
    void Begin() { m_state = TASK_STATE::RUNNING; }
    void BeginOptimization() { m_state = TASK_STATE::OPTIMIZING; }
    void Complete() { m_state = TASK_STATE::COMPLETED; }
    void Cancel() { m_state = TASK_STATE::CANCELLED; }
    void Fail() { m_state = TASK_STATE::FAILED; }
    TASK_STATE State() const { return m_state; }

private:
    TASK_STATE m_state = TASK_STATE::IDLE;
};

} // namespace KICAD_AUTOROUTER
