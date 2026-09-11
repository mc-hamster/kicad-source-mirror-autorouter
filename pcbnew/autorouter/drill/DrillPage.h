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
#include "ExpansionDrill.h"
#include "../datastructures/MinAreaTree.h"

namespace KICAD_AUTOROUTER
{

struct DRILL_PIN
{
    ROUTER_POINT position;
    int layer;
    bool drillAllowed;
};

/** Freerouting drill-page cache, rectangular centre-space geometry subset. */
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
    void Invalidate() { m_valid = false; m_drills.reset(); }
    void Reset()
    {
        m_candidates.clear();
        if( m_drills )
            for( auto& drill : *m_drills )
                drill.Reset();
    }

    bool                            IsValid() const { return m_valid; }
    const std::vector<ROUTER_POINT>& Candidates() const { return m_candidates; }
    ROUTER_POINT                    Center() const;
    int GetId() const;

    // Entries are in physical object/shape order. Obstacles span ALL drill
    // layers, including trace-inactive layers. Call Invalidate after edits.
    // Host inputs have via radius/clearance already applied; do not shrink twice.
    std::vector<EXPANSION_DRILL>* GetDrills(
            const std::vector<SHAPE_TREE_ENTRY>& aObstacles, int aNet, int aLayerCount,
            bool aAttachSmd = false, const std::vector<DRILL_PIN>& aPins = {},
            const ROUTER_CANCEL_CALLBACK& aCancel = {},
            std::size_t aMaxPieces = std::numeric_limits<std::size_t>::max(),
            bool aAnyAngle = false );

private:
    ROUTER_BOX             m_shape;
    bool                   m_valid = false;
    std::vector<ROUTER_POINT> m_candidates;
    std::optional<std::vector<EXPANSION_DRILL>> m_drills;
    int m_net = -1;
    int m_layerCount = 0;
    bool m_attachSmd = false;
    bool m_anyAngle = false;
};

} // namespace KICAD_AUTOROUTER
