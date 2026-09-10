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
#include <cstdint>
#include <optional>
#include <vector>

#include "../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

class ROUTING_BOARD;

/**
 * Freerouting equivalent: autoroute/path/Connection.
 *
 * The worker's ROUTING_CONNECTION owns the path nodes.  This value object
 * exposes the connection-level measurements used by ordering, rip-up and
 * optimization without coupling those decisions to KiCad BOARD items.
 */
class CONNECTION
{
public:
    static CONNECTION FromRoute( const ROUTING_CONNECTION& aRoute );
    /** Direct translation of Freerouting Connection.get(Item). */
    static std::optional<CONNECTION> Get( const ROUTING_BOARD& aBoard,
                                          std::uint64_t aItem );

    bool IsComplete() const { return m_complete; }
    int  NetCode() const { return m_netCode; }
    bool HasStartPoint() const { return m_hasStartPoint; }
    bool HasEndPoint() const { return m_hasEndPoint; }
    const ROUTER_POINT& StartPoint() const { return m_startPoint; }
    const ROUTER_POINT& EndPoint() const { return m_endPoint; }
    int                 StartLayer() const { return m_startLayer; }
    int                 EndLayer() const { return m_endLayer; }
    std::size_t         ItemCount() const { return m_itemCount; }
    const std::vector<std::uint64_t>& Items() const { return m_items; }
    double              TraceLength() const { return m_traceLength; }
    double              Detour() const;

private:
    int          m_netCode = 0;
    ROUTER_POINT m_startPoint;
    ROUTER_POINT m_endPoint;
    int          m_startLayer = -1;
    int          m_endLayer = -1;
    std::size_t  m_itemCount = 0;
    std::vector<std::uint64_t> m_items;
    double       m_traceLength = 0.0;
    bool         m_complete = false;
    bool         m_hasStartPoint = false;
    bool         m_hasEndPoint = false;
};

} // namespace KICAD_AUTOROUTER
