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

/** Per-connection optimization result, mirroring Freerouting's route ranking. */
class ITEM_ROUTE_RESULT
{
public:
    ITEM_ROUTE_RESULT( int aItemId, int aViaBefore, int aViaAfter, double aLengthBefore,
                       double aLengthAfter, int aUnroutedBefore, int aUnroutedAfter ) :
            m_itemId( aItemId ),
            m_viaBefore( aViaBefore ),
            m_viaAfter( aViaAfter ),
            m_lengthBefore( aLengthBefore ),
            m_lengthAfter( aLengthAfter ),
            m_unroutedBefore( aUnroutedBefore ),
            m_unroutedAfter( aUnroutedAfter )
    {
        // Match Freerouting's lexicographic improvement rule: completion
        // comes first, then vias, then copper length.  This is intentionally
        // independent from the percentage score because a small completion
        // improvement must beat a large length change.
        m_improved = m_unroutedAfter < m_unroutedBefore
                     || ( m_unroutedAfter == m_unroutedBefore
                          && ( m_viaAfter < m_viaBefore
                               || ( m_viaAfter == m_viaBefore
                                    && m_lengthAfter < m_lengthBefore ) ) );
    }

    int    ItemId() const { return m_itemId; }
    int    ViaCount() const { return m_viaAfter; }
    double TraceLength() const { return m_lengthAfter; }
    int    UnroutedCount() const { return m_unroutedAfter; }

    bool Improved() const { return m_improved; }
    void SetImproved( bool aValue ) { m_improved = aValue; }

    int CompareTo( const ITEM_ROUTE_RESULT& aOther ) const
    {
        if( m_unroutedAfter != aOther.m_unroutedAfter )
            return m_unroutedAfter < aOther.m_unroutedAfter ? -1 : 1;
        if( m_viaAfter != aOther.m_viaAfter )
            return m_viaAfter < aOther.m_viaAfter ? -1 : 1;
        if( m_lengthAfter != aOther.m_lengthAfter )
            return m_lengthAfter < aOther.m_lengthAfter ? -1 : 1;
        return 0;
    }

    double ImprovementPercentage() const
    {
        if( m_viaBefore == 0 || m_lengthBefore == 0 )
            return 0.0;
        return 1.0 - ( ( static_cast<double>( m_viaAfter ) / m_viaBefore
                         + m_lengthAfter / m_lengthBefore )
                       / 2.0 );
    }

private:
    int    m_itemId;
    int    m_viaBefore;
    int    m_viaAfter;
    double m_lengthBefore;
    double m_lengthAfter;
    int    m_unroutedBefore;
    int    m_unroutedAfter;
    bool   m_improved = false;
};

} // namespace KICAD_AUTOROUTER
