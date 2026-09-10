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

#include "ReadSortedRouteItems.h"

#include <algorithm>
#include <limits>


namespace KICAD_AUTOROUTER
{

bool READ_SORTED_ROUTE_ITEMS::KEY::operator<( const KEY& aOther ) const
{
    if( point.x != aOther.point.x )
        return point.x < aOther.point.x;
    if( point.y != aOther.point.y )
        return point.y < aOther.point.y;
    if( layer != aOther.layer )
        return layer < aOther.layer;
    if( kind != aOther.kind )
        return kind < aOther.kind;
    return item < aOther.item;
}


std::optional<READ_SORTED_ROUTE_ITEMS::KEY> READ_SORTED_ROUTE_ITEMS::Key(
        const ROUTING_BOARD& aBoard, const ROUTING_CONNECTION& aConnection )
{
    std::optional<KEY> result;
    for( ROUTING_BOARD::ITEM_ID itemId : aBoard.RouteItems( aConnection ) )
    {
        const auto item = aBoard.GetItemInfo( itemId );
        if( !item || !item->routable )
            continue;

        KEY candidate;
        candidate.item = itemId;
        candidate.layer = item->layers.empty()
                ? std::numeric_limits<int>::max()
                : *std::min_element( item->layers.begin(), item->layers.end() );

        if( item->kind == ROUTING_BOARD::ITEM_KIND::DRILL )
        {
            candidate.point = item->first;
            candidate.kind = 0;
        }
        else if( item->kind == ROUTING_BOARD::ITEM_KIND::TRACE )
        {
            // ReadSortedRouteItems deliberately leaves a trace for its
            // connected unfixed Via item, which is read first at that point.
            const auto contacts = aBoard.GetNormalContacts( itemId );
            const bool connectedToMovableVia = std::any_of(
                    contacts.begin(), contacts.end(),
                    [&]( ROUTING_BOARD::ITEM_ID aContact )
                    {
                        const auto contact = aBoard.GetItemInfo( aContact );
                        return contact && contact->routable
                               && contact->kind == ROUTING_BOARD::ITEM_KIND::DRILL;
                    } );
            if( connectedToMovableVia )
                continue;

            candidate.point = item->first.x < item->last.x
                                      || ( item->first.x == item->last.x
                                           && item->first.y < item->last.y )
                    ? item->last : item->first;
            candidate.kind = 1;
        }
        else
        {
            continue;
        }

        if( !result || candidate < *result )
            result = candidate;
    }

    return result;
}


void READ_SORTED_ROUTE_ITEMS::SortConnections(
        const ROUTING_BOARD& aBoard, std::vector<ROUTING_CONNECTION>& aConnections )
{
    std::stable_sort(
            aConnections.begin(), aConnections.end(),
            [&]( const ROUTING_CONNECTION& aLeft, const ROUTING_CONNECTION& aRight )
            {
                const auto left = Key( aBoard, aLeft );
                const auto right = Key( aBoard, aRight );
                if( left && right )
                    return *left < *right;
                return left.has_value() && !right.has_value();
            } );
}

} // namespace KICAD_AUTOROUTER
