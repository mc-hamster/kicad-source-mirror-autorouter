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
#include <cmath>
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

    BOARD_HISTORY( const AUTOROUTER_SETTINGS& aSettings,
                   std::size_t aMaximum = MAX_HISTORY_SIZE ) :
            m_settings( aSettings ),
            m_maximum( std::max<std::size_t>( 1, aMaximum ) )
    {
    }

    void Add( const ROUTING_RESULT& aResult )
    {
        std::lock_guard lock( m_mutex );
        if( std::any_of( m_entries.begin(), m_entries.end(),
                         [&]( const BOARD_HISTORY_ENTRY& aEntry )
                         { return equivalent( aEntry.result, aResult ); } ) )
        {
            return;
        }

        BOARD_HISTORY_ENTRY entry;
        entry.result = aResult;
        entry.score = NormalizedScore( aResult, m_settings );

        if( m_entries.size() >= m_maximum )
        {
            const auto worst = std::min_element(
                    m_entries.begin(), m_entries.end(),
                    []( const BOARD_HISTORY_ENTRY& aLeft, const BOARD_HISTORY_ENTRY& aRight )
                    { return better( aRight, aLeft ); } );
            if( worst != m_entries.end() && !better( entry, *worst ) )
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
                { return better( aRight, aLeft ); } );
        return best->result;
    }

    /** Freerouting BoardHistory.getMaxScore(), constrained to the safest DRC
     * stratum so a native checkpoint can never trade new violations for a
     * nominal score increase before KiCad's authoritative acceptance DRC. */
    double MaxScore() const
    {
        std::lock_guard lock( m_mutex );
        if( m_entries.empty() )
            return 0.0;
        const auto best = std::max_element(
                m_entries.begin(), m_entries.end(),
                []( const BOARD_HISTORY_ENTRY& aLeft, const BOARD_HISTORY_ENTRY& aRight )
                { return better( aRight, aLeft ); } );
        return best->score;
    }

    /** Restore the highest-ranked entry whose source-style restore counter
     * has not exceeded the supplied limit.  A nonpositive limit means no
     * limit, matching BoardHistory.restoreBestBoard(). */
    std::optional<ROUTING_RESULT> Restore( int aMaximumRestoreCount )
    {
        std::lock_guard lock( m_mutex );
        if( aMaximumRestoreCount <= 0 )
            aMaximumRestoreCount = std::numeric_limits<int>::max();

        std::stable_sort( m_entries.begin(), m_entries.end(),
                          []( const BOARD_HISTORY_ENTRY& aLeft,
                              const BOARD_HISTORY_ENTRY& aRight )
                          { return better( aLeft, aRight ); } );
        for( BOARD_HISTORY_ENTRY& entry : m_entries )
        {
            if( entry.restoreCount <= aMaximumRestoreCount )
            {
                ++entry.restoreCount;
                return entry.result;
            }
        }
        return std::nullopt;
    }

    int Rank( const ROUTING_RESULT& aResult ) const
    {
        std::lock_guard lock( m_mutex );
        for( std::size_t index = 0; index < m_entries.size(); ++index )
            if( equivalent( m_entries[index].result, aResult ) )
                return static_cast<int>( index + 1 );
        return -1;
    }

    static double NormalizedScore( const ROUTING_RESULT& aResult,
                                   const AUTOROUTER_SETTINGS& aSettings )
    {
        const double maximum = static_cast<double>( aResult.metrics.totalConnections )
                               * aSettings.unroutedNetPenalty;
        if( !std::isfinite( maximum ) || maximum <= 0.0 )
            return 0.0;

        const double penalties =
                static_cast<double>( aResult.metrics.unroutedConnections )
                        * aSettings.unroutedNetPenalty
                + static_cast<double>( aResult.metrics.drcViolations )
                        * aSettings.clearanceViolationPenalty
                + static_cast<double>( aResult.metrics.bendCount ) * aSettings.bendPenalty;
        // KiCad IU are nanometres; the reference score charges trace length
        // in millimetres independently of a file format's coordinate scale.
        const double traceLengthMm = aResult.metrics.routedLengthIU / 1000000.0;
        const double costs = traceLengthMm * std::max( 0, aSettings.traceLengthCost )
                             + static_cast<double>( aResult.metrics.viaCount )
                                       * std::max( 0, aSettings.viaCost );
        return std::max( 0.0, ( maximum - penalties - costs ) / maximum ) * 1000.0;
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
    static bool equivalent( const ROUTING_RESULT& aLeft, const ROUTING_RESULT& aRight )
    {
        if( aLeft.connections.size() != aRight.connections.size()
            || aLeft.removedBoardItemIds != aRight.removedBoardItemIds
            || aLeft.metrics.totalConnections != aRight.metrics.totalConnections
            || aLeft.metrics.unroutedConnections != aRight.metrics.unroutedConnections
            || aLeft.metrics.drcViolations != aRight.metrics.drcViolations
            || aLeft.metrics.viaCount != aRight.metrics.viaCount
            || aLeft.metrics.bendCount != aRight.metrics.bendCount
            || aLeft.metrics.routedLengthIU != aRight.metrics.routedLengthIU )
        {
            return false;
        }
        for( std::size_t index = 0; index < aLeft.connections.size(); ++index )
            if( !SameRouteGeometry( aLeft.connections[index], aRight.connections[index] ) )
                return false;
        return true;
    }

    static bool better( const BOARD_HISTORY_ENTRY& aLeft,
                        const BOARD_HISTORY_ENTRY& aRight )
    {
        if( aLeft.result.metrics.drcViolations != aRight.result.metrics.drcViolations )
            return aLeft.result.metrics.drcViolations < aRight.result.metrics.drcViolations;
        if( aLeft.score != aRight.score )
            return aLeft.score > aRight.score;
        return better( aLeft.result, aRight.result );
    }

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

    AUTOROUTER_SETTINGS            m_settings;
    std::size_t                    m_maximum;
    mutable std::mutex             m_mutex;
    std::vector<BOARD_HISTORY_ENTRY> m_entries;
};

} // namespace KICAD_AUTOROUTER
