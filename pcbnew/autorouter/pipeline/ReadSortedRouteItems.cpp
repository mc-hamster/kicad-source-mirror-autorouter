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
#include <set>
#include <tuple>


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


READ_SORTED_ROUTE_ITEMS::READ_SORTED_ROUTE_ITEMS() :
        m_minItemCoor{ std::numeric_limits<std::int64_t>::min(),
                       std::numeric_limits<std::int64_t>::min() }
{
}


bool READ_SORTED_ROUTE_ITEMS::PositionLess( const KEY& aLeft, const KEY& aRight )
{
    return std::tie( aLeft.point.x, aLeft.point.y, aLeft.layer )
           < std::tie( aRight.point.x, aRight.point.y, aRight.layer );
}


bool READ_SORTED_ROUTE_ITEMS::IsAfterCursor( const KEY& aKey ) const
{
    return std::tie( m_minItemCoor.x, m_minItemCoor.y, m_minItemLayer )
           < std::tie( aKey.point.x, aKey.point.y, aKey.layer );
}


std::optional<READ_SORTED_ROUTE_ITEMS::ENTRY> READ_SORTED_ROUTE_ITEMS::Next(
        const ROUTING_BOARD& aBoard,
        const std::vector<ROUTING_CONNECTION>& aConnections )
{
    struct CANDIDATE
    {
        KEY         key;
        std::size_t connectionIndex = 0;
    };

    std::vector<CANDIDATE> candidates;
    std::set<ROUTING_BOARD::ITEM_ID> seenItems;
    for( std::size_t connectionIndex = 0; connectionIndex < aConnections.size();
         ++connectionIndex )
    {
        for( ROUTING_BOARD::ITEM_ID itemId :
             aBoard.RouteItems( aConnections[connectionIndex] ) )
        {
            // Several host insertion records can name the one surviving
            // PolylineTrace after source-style combine.  UndoableObjects has
            // only that item, so expose it to the optimizer exactly once.
            if( !seenItems.insert( itemId ).second )
                continue;
            const auto item = aBoard.GetItemInfo( itemId );
            if( !item || !item->routable )
                continue;

            KEY key;
            key.item = itemId;
            key.layer = item->layers.empty()
                    ? std::numeric_limits<int>::max()
                    : *std::min_element( item->layers.begin(), item->layers.end() );

            if( item->kind == ROUTING_BOARD::ITEM_KIND::DRILL )
            {
                key.point = item->first;
                key.kind = 0;
            }
            else if( item->kind == ROUTING_BOARD::ITEM_KIND::TRACE )
            {
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

                key.point = item->first.x < item->last.x
                                    || ( item->first.x == item->last.x
                                         && item->first.y < item->last.y )
                        ? item->last : item->first;
                key.kind = 1;
            }
            else
            {
                continue;
            }

            candidates.push_back( { key, connectionIndex } );
        }
    }

    // UndoableObjects is traversed in insertion order. Worker item IDs are
    // allocated in that same order (and the first split piece keeps its ID),
    // so this is the stable native equivalent for exact coordinate ties.
    std::stable_sort( candidates.begin(), candidates.end(),
                      []( const CANDIDATE& aLeft, const CANDIDATE& aRight )
                      { return aLeft.key.item < aRight.key.item; } );

    std::optional<CANDIDATE> result;
    const auto scanKind = [&]( int aKind )
    {
        for( const CANDIDATE& candidate : candidates )
        {
            if( candidate.key.kind != aKind || !IsAfterCursor( candidate.key ) )
                continue;

            if( !result || PositionLess( candidate.key, result->key ) )
                result = candidate;
        }
    };

    // The Java implementation performs two full scans.  The strict-less
    // update in the trace scan leaves a via selected on an exact tie.
    scanKind( 0 );
    scanKind( 1 );

    if( !result )
    {
        m_minItemCoor = { std::numeric_limits<std::int64_t>::max(),
                          std::numeric_limits<std::int64_t>::max() };
        m_minItemLayer = std::numeric_limits<int>::max();
        return std::nullopt;
    }

    m_minItemCoor = result->key.point;
    m_minItemLayer = result->key.layer;
    return ENTRY{ result->key, result->connectionIndex };
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


} // namespace KICAD_AUTOROUTER
