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
 * Freerouting equivalent: autoroute/maze/AutorouteControl.java.
 */

#pragma once

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

/** Immutable cost and policy values for one maze-search attempt. */
class AUTOROUTE_CONTROL
{
public:
    // Direct counterpart of AutorouteControl.ExpansionCostFactor. Room costs
    // use geometric units, independently of the legacy grid cost methods below.
    struct EXPANSION_COST_FACTOR
    {
        double horizontal;
        double vertical;
    };

    AUTOROUTE_CONTROL( const AUTOROUTER_SETTINGS& aSettings, int aNetCode, int aRetry,
                       bool aTargetIsPlane = false ) :
            m_settings( aSettings ),
            m_netCode( aNetCode ),
            m_retry( aRetry ),
            m_targetIsPlane( aTargetIsPlane )
    {
    }

    const AUTOROUTER_SETTINGS& Settings() const { return m_settings; }
    int                        NetCode() const { return m_netCode; }
    int                        Retry() const { return m_retry; }

    double ViaCost() const;
    double TraceCost( double aLength ) const;
    double CongestionCost( int aUsage ) const;
    double DirectionCost( int aLayer, const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd ) const;

private:
    const AUTOROUTER_SETTINGS& m_settings;
    int                        m_netCode;
    int                        m_retry;
    bool                       m_targetIsPlane;
};

} // namespace KICAD_AUTOROUTER
