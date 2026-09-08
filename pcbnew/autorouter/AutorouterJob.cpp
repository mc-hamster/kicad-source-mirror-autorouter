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
 * This program source code file is part of KiCad, a free EDA software.
 */

#include "AutorouterJob.h"
#include "board/KicadRoutingSession.h"

#include <algorithm>
#include <exception>
#include <utility>


namespace KICAD_AUTOROUTER
{

AUTOROUTER_JOB::AUTOROUTER_JOB( std::shared_ptr<const BOARD_SNAPSHOT> aSnapshot,
                                AUTOROUTER_SETTINGS aSettings,
                                std::unique_ptr<KICAD_ROUTING_SESSION> aHostSession ) :
        m_hostSession( std::move( aHostSession ) ),
        m_snapshot( std::move( aSnapshot ) ),
        m_settings( std::move( aSettings ) )
{
    // Seed the dialog before the worker thread gets scheduled.  Without this
    // initial snapshot the first paint shows "0 of 0" even though the job is
    // already preparing a board, which looks like the autorouter is hung.
    if( m_snapshot )
    {
        m_progress.maxPasses = std::max( 1, m_settings.maxPasses );
        m_progress.totalConnections = 0;

        for( const ROUTING_NET& net : m_snapshot->nets )
            m_progress.totalConnections += static_cast<int>( net.connections.size() );

        m_progress.stage = "Preparing SMD fanout";
    }
}


AUTOROUTER_JOB::~AUTOROUTER_JOB()
{
    Cancel();
    Join();
}


void AUTOROUTER_JOB::Start()
{
    bool expected = false;

    if( !m_started.compare_exchange_strong( expected, true ) )
        return;

    m_thread = std::thread( &AUTOROUTER_JOB::run, this );
}


void AUTOROUTER_JOB::Cancel()
{
    m_cancel.store( true );
}


void AUTOROUTER_JOB::Join()
{
    if( m_thread.joinable() )
        m_thread.join();
}


ROUTER_PROGRESS AUTOROUTER_JOB::GetProgress() const
{
    std::lock_guard lock( m_mutex );
    return m_progress;
}


std::optional<ROUTING_RESULT> AUTOROUTER_JOB::GetResult() const
{
    std::lock_guard lock( m_mutex );

    if( !m_result )
        return std::nullopt;

    return *m_result;
}


void AUTOROUTER_JOB::run()
{
    ROUTING_RESULT result;

    if( !m_snapshot )
    {
        result.message = "Autorouter received no board snapshot";
    }
    else
    {
        try
        {
            const ROUTER_CANCEL_CALLBACK cancel = [this] { return m_cancel.load(); };
            const ROUTER_PROGRESS_CALLBACK progress = [this]( const ROUTER_PROGRESS& state )
            {
                std::lock_guard lock( m_mutex );
                m_progress = state;
            };
            result = m_hostSession ? m_hostSession->Run( m_settings, cancel, progress )
                    : ROUTING_PIPELINE().Run( *m_snapshot, m_settings, cancel, progress );
        }
        catch( const std::exception& e )
        {
            result.message = std::string( "Autorouter failed: " ) + e.what();
        }
        catch( ... )
        {
            result.message = "Autorouter failed with an unknown exception";
        }
    }

    if( m_cancel.load() )
    {
        // Cancellation can arrive just after validation. Do not retain the
        // validation flag or any materialized proposal from that race.
        result = ROUTING_RESULT{};
        result.cancelled = true;
        result.message = "Autorouter cancelled";
    }

    {
        std::lock_guard lock( m_mutex );
        m_result = std::move( result );
    }

    m_finished.store( true );
}

} // namespace KICAD_AUTOROUTER
