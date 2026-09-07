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
 * Lightweight view-only representation of a proposed autoroute.
 *
 * Proposal objects are deliberately not BOARD_ITEMs: showing a result must
 * not change the board, connectivity, or undo history before the user chooses
 * Accept.  The corresponding PCB_TRACK/PCB_VIA objects are kept separately
 * and are committed only after acceptance.
 */

#pragma once

#include <eda_item.h>

#include "AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

class AUTOROUTER_PREVIEW_ITEM : public EDA_ITEM
{
public:
    explicit AUTOROUTER_PREVIEW_ITEM( const ROUTING_SEGMENT& aSegment );
    explicit AUTOROUTER_PREVIEW_ITEM( const ROUTING_VIA& aVia );
    ~AUTOROUTER_PREVIEW_ITEM() override = default;

    wxString GetClass() const override { return wxT( "AUTOROUTER_PREVIEW_ITEM" ); }
    const BOX2I ViewBBox() const override;
    void ViewDraw( int aLayer, KIGFX::VIEW* aView ) const override;
    std::vector<int> ViewGetLayers() const override;

private:
    bool           m_isVia = false;
    ROUTING_SEGMENT m_segment;
    ROUTING_VIA     m_via;
};

} // namespace KICAD_AUTOROUTER
