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
 * KiCad host adapter for the Freerouting-derived engine.
 *
 * Freerouting equivalent: board/facade/RoutingBoard plus
 * board/facade/RoutingBoardSearchFacade and RoutingBoardOperations.  This
 * class is the only routing component that knows about BOARD, PAD, PCB_TRACK,
 * PCB_VIA, DRC rule values, or KiCad UUIDs.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <board.h>
#include <board_item.h>

#include "RoutingBoardInterface.h"


class BOARD;

namespace KICAD_AUTOROUTER
{

class KICAD_BOARD_ADAPTER : public ROUTING_BOARD_INTERFACE
{
public:
    explicit KICAD_BOARD_ADAPTER( BOARD* aBoard ) :
            m_board( aBoard )
    {
    }

    std::shared_ptr<const BOARD_SNAPSHOT>
            CreateSnapshot( const AUTOROUTER_SETTINGS& aSettings ) const override;

    AUTOROUTER_SETTINGS CreateDefaultSettings() const;

    /** Convert a result to unattached, transient KiCad objects for proposal review. */
    std::vector<std::unique_ptr<BOARD_ITEM>> CreatePreviewItems( const ROUTING_RESULT& aResult ) const;

    BOARD_ITEM* FindBoardItem( const std::string& aUuid ) const;

    BOARD* GetBoard() const { return m_board; }

private:
    bool netClassIncluded( const std::string& aNetClass,
                           const AUTOROUTER_SETTINGS& aSettings ) const;
    bool netIncluded( const std::string& aNetName,
                      const AUTOROUTER_SETTINGS& aSettings ) const;
    void addBoardOutline( BOARD_SNAPSHOT& aSnapshot ) const;
    void addPads( BOARD_SNAPSHOT& aSnapshot, const AUTOROUTER_SETTINGS& aSettings ) const;
    void addExistingCopper( BOARD_SNAPSHOT& aSnapshot,
                            const AUTOROUTER_SETTINGS& aSettings ) const;
    void addKeepouts( BOARD_SNAPSHOT& aSnapshot ) const;
    void addClearanceRules( BOARD_SNAPSHOT& aSnapshot,
                            const AUTOROUTER_SETTINGS& aSettings ) const;
    void addNet( BOARD_SNAPSHOT& aSnapshot, int aNetCode, const wxString& aNetName,
                 const wxString& aNetClass, int aPriority, std::int64_t aClearance,
                 std::int64_t aViaDiameter, std::int64_t aViaDrill ) const;

    static ROUTER_POINT point( const VECTOR2I& aPoint );
    static ROUTER_BOX box( const BOX2I& aBox );
    static void appendPolygon( std::vector<ROUTER_POINT>& aDestination,
                               const SHAPE_LINE_CHAIN& aChain );

private:
    BOARD* m_board;
};

} // namespace KICAD_AUTOROUTER
