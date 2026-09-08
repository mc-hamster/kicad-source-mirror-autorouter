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
 * Worker orchestration for the native autorouter.
 *
 * The algorithm receives a data-only snapshot. The optional host session owns
 * an isolated KiCad board for refill/validation; neither path accesses the live
 * editor board, views or UI objects.
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "pipeline/RoutingPipeline.h"


namespace KICAD_AUTOROUTER
{

class KICAD_ROUTING_SESSION;

class AUTOROUTER_JOB
{
public:
    AUTOROUTER_JOB( std::shared_ptr<const BOARD_SNAPSHOT> aSnapshot,
                    AUTOROUTER_SETTINGS aSettings,
                    std::unique_ptr<KICAD_ROUTING_SESSION> aHostSession = nullptr );
    ~AUTOROUTER_JOB();

    AUTOROUTER_JOB( const AUTOROUTER_JOB& ) = delete;
    AUTOROUTER_JOB& operator=( const AUTOROUTER_JOB& ) = delete;

    void Start();
    void Cancel();
    void Join();

    bool IsStarted() const { return m_started.load(); }
    bool IsFinished() const { return m_finished.load(); }
    bool IsCancellationRequested() const { return m_cancel.load(); }

    ROUTER_PROGRESS GetProgress() const;
    std::optional<ROUTING_RESULT> GetResult() const;

private:
    void run();

private:
    std::unique_ptr<KICAD_ROUTING_SESSION> m_hostSession;
    std::shared_ptr<const BOARD_SNAPSHOT> m_snapshot;
    AUTOROUTER_SETTINGS                    m_settings;
    std::atomic<bool>                      m_cancel{ false };
    std::atomic<bool>                      m_started{ false };
    std::atomic<bool>                      m_finished{ false };
    mutable std::mutex                     m_mutex;
    ROUTER_PROGRESS                        m_progress;
    std::optional<ROUTING_RESULT>          m_result;
    std::thread                            m_thread;
};

} // namespace KICAD_AUTOROUTER
