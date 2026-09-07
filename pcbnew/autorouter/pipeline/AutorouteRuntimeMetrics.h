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

#include "../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

/** Runtime metrics corresponding to Freerouting's AutorouteRuntimeMetrics. */
class AUTOROUTE_RUNTIME_METRICS
{
public:
    void AddExpandedNodes( int aCount ) { m_expandedNodes += aCount; }
    void AddRetry() { ++m_retries; }
    void AddRipup() { ++m_ripups; }
    void AddPass() { ++m_passes; }

    void Apply( ROUTER_METRICS& aMetrics ) const
    {
        aMetrics.expandedNodes = m_expandedNodes;
        aMetrics.retries = m_retries;
        aMetrics.ripups = m_ripups;
        aMetrics.passes = m_passes;
    }

private:
    int m_expandedNodes = 0;
    int m_retries = 0;
    int m_ripups = 0;
    int m_passes = 0;
};

} // namespace KICAD_AUTOROUTER
