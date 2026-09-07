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

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

namespace KICAD_AUTOROUTER
{

/** Small thread-safe section profiler corresponding to Freerouting's profiler seam. */
class PERFORMANCE_PROFILER
{
public:
    class SCOPE
    {
    public:
        SCOPE( PERFORMANCE_PROFILER& aProfiler, std::string aSection ) :
                m_profiler( aProfiler ),
                m_section( std::move( aSection ) ),
                m_started( std::chrono::steady_clock::now() )
        {
        }

        ~SCOPE()
        {
            const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - m_started );
            m_profiler.Record( m_section, elapsed.count() );
        }

    private:
        PERFORMANCE_PROFILER&                 m_profiler;
        std::string                           m_section;
        std::chrono::steady_clock::time_point m_started;
    };

    void Record( const std::string& aSection, std::int64_t aMicroseconds )
    {
        std::lock_guard lock( m_mutex );
        auto& value = m_sections[aSection];
        value.totalMicroseconds += aMicroseconds;
        ++value.calls;
    }

    struct STATISTICS
    {
        std::int64_t totalMicroseconds = 0;
        std::int64_t calls = 0;
    };

    std::unordered_map<std::string, STATISTICS> Snapshot() const
    {
        std::lock_guard lock( m_mutex );
        return m_sections;
    }

    void Reset()
    {
        std::lock_guard lock( m_mutex );
        m_sections.clear();
    }

private:
    mutable std::mutex                         m_mutex;
    std::unordered_map<std::string, STATISTICS> m_sections;
};

} // namespace KICAD_AUTOROUTER
