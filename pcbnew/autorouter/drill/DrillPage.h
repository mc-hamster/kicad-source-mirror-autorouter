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

#include <vector>

#include "../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

/** Freerouting equivalent: autoroute/drill/DrillPage. */
class DRILL_PAGE
{
public:
    explicit DRILL_PAGE( ROUTER_BOX aShape ) : m_shape( aShape ) {}

    const ROUTER_BOX& Shape() const { return m_shape; }
    bool              Contains( const ROUTER_POINT& aPoint ) const
    {
        return m_shape.Contains( aPoint );
    }

    void AddCandidate( const ROUTER_POINT& aPoint );
    void Invalidate() { m_valid = false; }
    void Reset()
    {
        m_valid = true;
        m_candidates.clear();
    }

    bool                            IsValid() const { return m_valid; }
    const std::vector<ROUTER_POINT>& Candidates() const { return m_candidates; }
    ROUTER_POINT                    Center() const;

private:
    ROUTER_BOX             m_shape;
    bool                   m_valid = true;
    std::vector<ROUTER_POINT> m_candidates;
};

} // namespace KICAD_AUTOROUTER
