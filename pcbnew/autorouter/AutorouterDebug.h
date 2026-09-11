/*
 * This program source code file is part of KiCad, a free EDA application.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace KICAD_AUTOROUTER
{

inline std::vector<std::pair<std::string, std::string>>& autorouterDecisionContext()
{
    static thread_local std::vector<std::pair<std::string, std::string>> context;
    return context;
}

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


/**
 * Emit one deterministic, machine-readable routing decision.
 *
 * KICAD_AUTOROUTER_DEBUG is intentionally human oriented and contains timing
 * noise.  Set KICAD_AUTOROUTER_DECISION_TRACE to a JSONL filename when doing
 * a source/native differential run.  Every field is encoded as a string so
 * exact integer geometry and source floating values survive without a JSON
 * library or a loss through double conversion.
 */
inline void autorouterDecisionLog(
        const std::string& aEvent,
        const std::vector<std::pair<std::string, std::string>>& aFields )
{
    struct TRACE_STATE
    {
        std::mutex    mutex;
        std::ofstream stream;
        std::uint64_t sequence = 0;
        std::uint64_t maximumEvents = 0;

        TRACE_STATE()
        {
            const char* path = std::getenv( "KICAD_AUTOROUTER_DECISION_TRACE" );

            if( path && *path )
                stream.open( path, std::ios::out | std::ios::trunc );

            const char* maximum = std::getenv( "KICAD_AUTOROUTER_DECISION_MAX_EVENTS" );

            if( maximum && *maximum )
                maximumEvents = std::strtoull( maximum, nullptr, 10 );
        }
    };

    static TRACE_STATE state;

    if( !state.stream.is_open() )
        return;

    std::vector<std::pair<std::string, std::string>> fields = autorouterDecisionContext();

    for( const auto& field : aFields )
    {
        const auto existing = std::find_if( fields.begin(), fields.end(), [&]( const auto& current )
        {
            return current.first == field.first;
        } );

        if( existing == fields.end() )
            fields.push_back( field );
        else
            existing->second = field.second;
    }

    const auto quote = []( const std::string& aValue )
    {
        std::string result;
        result.reserve( aValue.size() + 2 );
        result.push_back( '"' );

        constexpr char digits[] = "0123456789abcdef";
        for( const unsigned char current : aValue )
        {
            switch( current )
            {
            case '"': result += "\\\""; break;
            case '\\': result += "\\\\"; break;
            case '\b': result += "\\b"; break;
            case '\f': result += "\\f"; break;
            case '\n': result += "\\n"; break;
            case '\r': result += "\\r"; break;
            case '\t': result += "\\t"; break;
            default:
                if( current < 0x20 )
                {
                    result += "\\u00";
                    result.push_back( digits[current >> 4] );
                    result.push_back( digits[current & 0xf] );
                }
                else
                {
                    result.push_back( static_cast<char>( current ) );
                }
            }
        }

        result.push_back( '"' );
        return result;
    };

    std::lock_guard lock( state.mutex );

    if( state.maximumEvents > 0 && state.sequence >= state.maximumEvents )
        return;

    state.stream << "{\"sequence\":" << state.sequence++ << ",\"event\":"
                 << quote( aEvent ) << ",\"fields\":{";

    for( std::size_t index = 0; index < fields.size(); ++index )
    {
        if( index > 0 )
            state.stream << ',';

        state.stream << quote( fields[index].first ) << ':'
                     << quote( fields[index].second );
    }

    state.stream << "}}\n";
    state.stream.flush();
}


class AUTOROUTER_DECISION_CONTEXT_SCOPE
{
public:
    explicit AUTOROUTER_DECISION_CONTEXT_SCOPE(
            std::vector<std::pair<std::string, std::string>> aFields ) :
            m_previous( std::move( autorouterDecisionContext() ) )
    {
        autorouterDecisionContext() = m_previous;

        for( auto& field : aFields )
        {
            const auto existing = std::find_if(
                    autorouterDecisionContext().begin(), autorouterDecisionContext().end(),
                    [&]( const auto& current ) { return current.first == field.first; } );

            if( existing == autorouterDecisionContext().end() )
                autorouterDecisionContext().push_back( std::move( field ) );
            else
                existing->second = std::move( field.second );
        }
    }

    ~AUTOROUTER_DECISION_CONTEXT_SCOPE()
    {
        autorouterDecisionContext() = std::move( m_previous );
    }

    AUTOROUTER_DECISION_CONTEXT_SCOPE( const AUTOROUTER_DECISION_CONTEXT_SCOPE& ) = delete;
    AUTOROUTER_DECISION_CONTEXT_SCOPE& operator=( const AUTOROUTER_DECISION_CONTEXT_SCOPE& ) = delete;

private:
    std::vector<std::pair<std::string, std::string>> m_previous;
};


template<typename BOX>
inline std::string autorouterDecisionBounds( const BOX& aBounds )
{
    return std::to_string( aBounds.minX ) + ',' + std::to_string( aBounds.minY ) + ','
           + std::to_string( aBounds.maxX ) + ',' + std::to_string( aBounds.maxY );
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
