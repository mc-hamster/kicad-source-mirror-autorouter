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
#include <cstddef>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "BoardHistoryEntry.h"

namespace KICAD_AUTOROUTER
{

/**
 * Freerouting equivalent: autoroute/BoardHistory.
 *
 * A native history stores immutable result snapshots rather than serializing
 * a second PCB representation.  It is bounded so repeated negotiated-congestion
 * passes cannot retain unbounded worker memory.
 */
class BOARD_HISTORY
{
public:
    static constexpr std::size_t MAX_HISTORY_SIZE = 30;

    explicit BOARD_HISTORY( std::size_t aMaximum = MAX_HISTORY_SIZE ) :
            m_maximum( std::max<std::size_t>( 1, aMaximum ) )
    {
    }

    void Add( const ROUTING_RESULT& aResult )
    {
        std::lock_guard lock( m_mutex );
        BOARD_HISTORY_ENTRY entry;
        entry.result = aResult;

        if( m_entries.size() >= m_maximum )
        {
            const auto worst = std::min_element(
                    m_entries.begin(), m_entries.end(),
                    []( const BOARD_HISTORY_ENTRY& aLeft, const BOARD_HISTORY_ENTRY& aRight )
                    {
                        return better( aRight.result, aLeft.result );
                    } );
            if( worst != m_entries.end() && !better( entry.result, worst->result ) )
                return;
            if( worst != m_entries.end() )
                m_entries.erase( worst );
        }

        m_entries.push_back( std::move( entry ) );
    }

    std::optional<ROUTING_RESULT> Best() const
    {
        std::lock_guard lock( m_mutex );
        if( m_entries.empty() )
            return std::nullopt;

        const auto best = std::max_element(
                m_entries.begin(), m_entries.end(),
                []( const BOARD_HISTORY_ENTRY& aLeft, const BOARD_HISTORY_ENTRY& aRight )
                {
                    return better( aRight.result, aLeft.result );
                } );
        return best->result;
    }

    std::size_t Size() const
    {
        std::lock_guard lock( m_mutex );
        return m_entries.size();
    }

    void Clear()
    {
        std::lock_guard lock( m_mutex );
        m_entries.clear();
    }

private:
    static bool better( const ROUTING_RESULT& aLeft, const ROUTING_RESULT& aRight )
    {
        // Lexicographic priorities cannot be overturned by arbitrary board
        // size or length. Never restore more connectivity at the expense of
        // additional clearance violations.
        if( aLeft.metrics.drcViolations != aRight.metrics.drcViolations )
            return aLeft.metrics.drcViolations < aRight.metrics.drcViolations;
        if( aLeft.metrics.unroutedConnections != aRight.metrics.unroutedConnections )
            return aLeft.metrics.unroutedConnections < aRight.metrics.unroutedConnections;
        if( aLeft.metrics.viaCount != aRight.metrics.viaCount )
            return aLeft.metrics.viaCount < aRight.metrics.viaCount;
        return aLeft.metrics.routedLengthIU < aRight.metrics.routedLengthIU;
    }

    std::size_t                    m_maximum;
    mutable std::mutex             m_mutex;
    std::vector<BOARD_HISTORY_ENTRY> m_entries;
};

} // namespace KICAD_AUTOROUTER
