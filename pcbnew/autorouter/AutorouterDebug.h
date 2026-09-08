/*
 * This program source code file is part of KiCad, a free EDA application.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#pragma once

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <utility>

namespace KICAD_AUTOROUTER
{

/**
 * Opt-in diagnostics for long-running native autorouter jobs.
 *
 * The editor must not emit a log line for every A* expansion, so the normal
 * path is silent.  Set KICAD_AUTOROUTER_DEBUG=1 when running a diagnostic
 * build or the headless parity harness to get timestamped pipeline/search
 * checkpoints on stderr.
 */
inline bool autorouterDebugEnabled()
{
    static const bool enabled = []
    {
        const char* value = std::getenv( "KICAD_AUTOROUTER_DEBUG" );
        return value && *value && std::string( value ) != "0";
    }();

    return enabled;
}


inline void autorouterDebugLog( const std::string& aMessage )
{
    if( !autorouterDebugEnabled() )
        return;

    static const auto started = std::chrono::steady_clock::now();
    static std::mutex  mutex;
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - started )
                                  .count();

    std::lock_guard lock( mutex );
    std::cerr << "[autorouter " << elapsed << " ms] " << aMessage << '\n';
}


class AUTOROUTER_DEBUG_SCOPE
{
public:
    explicit AUTOROUTER_DEBUG_SCOPE( std::string aName ) :
            m_name( std::move( aName ) ),
            m_started( std::chrono::steady_clock::now() )
    {
        autorouterDebugLog( "BEGIN " + m_name );
    }

    ~AUTOROUTER_DEBUG_SCOPE()
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - m_started )
                                      .count();
        autorouterDebugLog( "END " + m_name + " elapsed=" + std::to_string( elapsed ) + " ms" );
    }

private:
    std::string                           m_name;
    std::chrono::steady_clock::time_point m_started;
};

} // namespace KICAD_AUTOROUTER
