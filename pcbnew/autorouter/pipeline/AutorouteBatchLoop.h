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

namespace KICAD_AUTOROUTER
{

/**
 * Pass-convergence guard corresponding to Freerouting's batch loop.  It keeps
 * a hopeless board from spending every configured pass on an unchanged state,
 * while rip-up activity resets the stagnation counter.
 */
class AUTOROUTE_BATCH_LOOP
{
public:
    bool Observe( int aRoutedConnections, int aRipups )
    {
        if( m_seen && aRoutedConnections <= m_lastRouted && aRipups == m_lastRipups )
            ++m_stagnantPasses;
        else
            m_stagnantPasses = 0;

        m_lastRouted = aRoutedConnections;
        m_lastRipups = aRipups;
        m_seen = true;
        return m_stagnantPasses >= 2;
    }

    int StagnantPasses() const { return m_stagnantPasses; }

private:
    int  m_lastRouted = 0;
    int  m_lastRipups = 0;
    int  m_stagnantPasses = 0;
    bool m_seen = false;
};

} // namespace KICAD_AUTOROUTER
