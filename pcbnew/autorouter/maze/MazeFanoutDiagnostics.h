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

#include <functional>
#include <string>
#include <utility>

#include "../AutorouteDiagnostic.h"


namespace KICAD_AUTOROUTER
{

/** Optional diagnostics seam corresponding to Freerouting's fanout logger. */
class MAZE_FANOUT_DIAGNOSTICS
{
public:
    MAZE_FANOUT_DIAGNOSTICS( int aNetCode = 0, std::string aPinName = {},
                             AUTOROUTE_DIAGNOSTIC_SINK aSink = {} ) :
            m_netCode( aNetCode ),
            m_pinName( std::move( aPinName ) ),
            m_sink( std::move( aSink ) )
    {
    }

    bool Enabled() const { return static_cast<bool>( m_sink ); }
    int  NetCode() const { return m_netCode; }
    const std::string& PinName() const { return m_pinName; }

    void Trace( const ROUTER_BOX& aShape, int aLayer, double aIntensity ) const
    {
        if( !m_sink )
            return;

        m_sink( { AUTOROUTE_DIAGNOSTIC::KIND::FREE_SPACE_ROOM, aShape, aLayer, aIntensity } );
    }

private:
    int                       m_netCode = 0;
    std::string               m_pinName;
    AUTOROUTE_DIAGNOSTIC_SINK m_sink;
};

} // namespace KICAD_AUTOROUTER
