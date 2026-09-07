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

#include "DrillPage.h"

#include <algorithm>

namespace KICAD_AUTOROUTER
{

void DRILL_PAGE::AddCandidate( const ROUTER_POINT& aPoint )
{
    if( Contains( aPoint )
        && std::find( m_candidates.begin(), m_candidates.end(), aPoint ) == m_candidates.end() )
    {
        m_candidates.push_back( aPoint );
    }
}


ROUTER_POINT DRILL_PAGE::Center() const
{
    return { ( m_shape.minX + m_shape.maxX ) / 2, ( m_shape.minY + m_shape.maxY ) / 2 };
}

} // namespace KICAD_AUTOROUTER
