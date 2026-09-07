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

#include <cstddef>
#include <vector>

#include "DrillPage.h"

namespace KICAD_AUTOROUTER
{

/**
 * Freerouting equivalent: autoroute/drill/DrillPageArray.
 *
 * The native router uses the pages as a deterministic spatial index for via
 * candidates.  It deliberately has no BOARD dependency, so the worker can
 * construct and invalidate it from a snapshot.
 */
class DRILL_PAGE_ARRAY
{
public:
    DRILL_PAGE_ARRAY( const ROUTER_BOX& aBounds, std::int64_t aMaxPageWidth );

    const std::vector<DRILL_PAGE>& Pages() const { return m_pages; }
    std::vector<DRILL_PAGE*>       OverlappingPages( const ROUTER_BOX& aShape );
    std::vector<ROUTER_POINT>      LandmarkCenters() const;
    void                           Invalidate( const ROUTER_BOX& aShape );
    void                           Reset();

private:
    ROUTER_BOX             m_bounds;
    int                    m_columns = 0;
    int                    m_rows = 0;
    std::int64_t           m_pageWidth = 1;
    std::int64_t           m_pageHeight = 1;
    std::vector<DRILL_PAGE> m_pages;
};

} // namespace KICAD_AUTOROUTER
