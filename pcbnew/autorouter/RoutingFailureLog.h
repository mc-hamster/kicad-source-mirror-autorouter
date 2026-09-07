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

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>

#include "AutorouteAttemptState.h"

namespace KICAD_AUTOROUTER
{

/** Bounded failure log used to stop spending every pass on one impossible net. */
class ROUTING_FAILURE_LOG
{
public:
    static constexpr int FAILURE_THRESHOLD = 50;

    void RecordFailure( int aNetCode, int aPass, AUTOROUTE_ATTEMPT_STATE aState,
                        std::string aReason )
    {
        ENTRY& entry = m_entries[aNetCode];
        ++entry.count;
        entry.lastPass = aPass;
        entry.state = aState;
        entry.reason = std::move( aReason );
    }

    bool ShouldSkip( int aNetCode ) const
    {
        const auto it = m_entries.find( aNetCode );
        return it != m_entries.end() && it->second.count >= FAILURE_THRESHOLD;
    }

    int FailureCount( int aNetCode ) const
    {
        const auto it = m_entries.find( aNetCode );
        return it == m_entries.end() ? 0 : it->second.count;
    }

    void Clear() { m_entries.clear(); }

private:
    struct ENTRY
    {
        int                    count = 0;
        int                    lastPass = 0;
        AUTOROUTE_ATTEMPT_STATE state = AUTOROUTE_ATTEMPT_STATE::UNKNOWN;
        std::string            reason;
    };

    std::unordered_map<int, ENTRY> m_entries;
};

} // namespace KICAD_AUTOROUTER
