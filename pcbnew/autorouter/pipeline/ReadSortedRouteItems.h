/*
 * This program source code file is part of KiCad, a free EDA application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#pragma once

#include <optional>
#include <vector>

#include "../AutorouterTypes.h"
#include "../board/facade/RoutingBoard.h"


namespace KICAD_AUTOROUTER
{

/** Direct translation of BatchOptimizer.ReadSortedRouteItems' mutable scan. */
class READ_SORTED_ROUTE_ITEMS
{
public:
    struct KEY
    {
        ROUTER_POINT point;
        int          layer = -1;
        int          kind = 0; // 0 = via, 1 = trace
        ROUTING_BOARD::ITEM_ID item = 0;

        bool operator<( const KEY& aOther ) const;
    };

    struct ENTRY
    {
        KEY         key;
        std::size_t connectionIndex = 0;
    };

    READ_SORTED_ROUTE_ITEMS();

    /**
     * Return the next mutable Via/PolylineTrace item in source x/y/layer order.
     *
     * The worker board is deliberately scanned from scratch for every call.
     * Accepted optimization edits can therefore contribute a later item in
     * the same pass, while items at or before the monotonic cursor are not
     * revisited.  Vias are scanned before traces, so a via wins an exact
     * x/y/layer tie; every other item at that exact key is skipped just as in
     * the Java source.
     */
    std::optional<ENTRY> Next( const ROUTING_BOARD& aBoard,
                               const std::vector<ROUTING_CONNECTION>& aConnections );

    const ROUTER_POINT& CurrentPosition() const { return m_minItemCoor; }
    int CurrentLayer() const { return m_minItemLayer; }

    static std::optional<KEY> Key( const ROUTING_BOARD& aBoard,
                                   const ROUTING_CONNECTION& aConnection );

private:
    static bool PositionLess( const KEY& aLeft, const KEY& aRight );
    bool IsAfterCursor( const KEY& aKey ) const;

private:
    ROUTER_POINT m_minItemCoor;
    int          m_minItemLayer = -1;
};

} // namespace KICAD_AUTOROUTER
