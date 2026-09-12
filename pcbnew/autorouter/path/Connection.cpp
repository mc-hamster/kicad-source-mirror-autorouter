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

#include "Connection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include "../board/facade/RoutingBoard.h"

namespace KICAD_AUTOROUTER
{

namespace
{

double distance( const ROUTER_POINT& aLeft, const ROUTER_POINT& aRight )
{
    const long double dx = static_cast<long double>( aLeft.x ) - aRight.x;
    const long double dy = static_cast<long double>( aLeft.y ) - aRight.y;
    return std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
}


bool sameTraceItemStyle( const ROUTING_CONNECTION& aRoute, std::size_t aLeft,
                         std::size_t aRight )
{
    const ROUTING_EDGE_STYLE& left = EdgeStyle( aRoute, aLeft );
    const ROUTING_EDGE_STYLE& right = EdgeStyle( aRoute, aRight );
    return left.trackWidth == right.trackWidth && left.clearance == right.clearance
           && left.fixedState == right.fixedState;
}

} // namespace


CONNECTION CONNECTION::FromRoute( const ROUTING_CONNECTION& aRoute )
{
    CONNECTION result;
    result.m_netCode = aRoute.netCode;
    result.m_complete = aRoute.complete;
    if( !aRoute.nodes.empty() )
    {
        result.m_startPoint = aRoute.nodes.front().point;
        result.m_startLayer = aRoute.nodes.front().layer;
        result.m_endPoint = aRoute.nodes.back().point;
        result.m_endLayer = aRoute.nodes.back().layer;
        result.m_hasStartPoint = true;
        result.m_hasEndPoint = true;
    }

    for( std::size_t index = 1; index < aRoute.nodes.size(); ++index )
    {
        const bool trace = aRoute.nodes[index - 1].layer == aRoute.nodes[index].layer;
        if( trace )
            result.m_traceLength += distance( aRoute.nodes[index - 1].point,
                                              aRoute.nodes[index].point );

        // Freerouting inserts one PolylineTrace for a contiguous same-layer,
        // same-width run, not one Item for each corner-to-corner edge.  A via
        // and each width/clearance transition start a new source item.
        const bool continuesTrace = trace && index > 1
                && aRoute.nodes[index - 2].layer == aRoute.nodes[index - 1].layer
                && sameTraceItemStyle( aRoute, index - 2, index - 1 );
        if( !continuesTrace )
            ++result.m_itemCount;
    }

    return result;
}


std::optional<CONNECTION> CONNECTION::Get( const ROUTING_BOARD& aBoard,
                                            std::uint64_t aItem )
{
    const auto item = aBoard.GetItemInfo( aItem );
    if( !item || !item->routable )
        return std::nullopt;

    CONNECTION result;
    result.m_netCode = item->netCode;
    std::set<std::uint64_t, std::greater<std::uint64_t>> connectionItems{ aItem };
    const ROUTING_BOARD::ITEM_ID_SET contacts = aBoard.GetNormalContacts( aItem );

    for( std::uint64_t currentItem : contacts )
    {
        std::optional<ROUTER_POINT> previousPoint =
                aBoard.NormalContactPoint( aItem, currentItem );
        if( !previousPoint )
            continue;

        int  previousLayer = aBoard.FirstCommonLayer( aItem, currentItem );
        bool forkFound = false;

        if( item->kind == ROUTING_BOARD::ITEM_KIND::TRACE
            && aBoard.NormalContactsAt( aItem, *previousPoint ).size() != 1 )
        {
            forkFound = true;
        }

        // Search along unique normal contacts until the next fixed terminal,
        // stub or fork.  ITEM_ID_SET iterates in the reference's reverse-ID
        // TreeSet order, so ambiguous topology terminates deterministically.
        for( ;; )
        {
            const auto current = aBoard.GetItemInfo( currentItem );
            if( !current || !current->routable || forkFound )
            {
                if( !result.m_hasStartPoint )
                {
                    result.m_startPoint = *previousPoint;
                    result.m_startLayer = previousLayer;
                    result.m_hasStartPoint = true;
                }
                else if( *previousPoint != result.m_startPoint )
                {
                    result.m_endPoint = *previousPoint;
                    result.m_endLayer = previousLayer;
                    result.m_hasEndPoint = true;
                }
                break;
            }

            connectionItems.insert( currentItem );
            std::optional<ROUTER_POINT> nextPoint;
            int                          nextLayer = -1;
            std::optional<std::uint64_t> nextContact;

            for( std::uint64_t candidate : aBoard.GetNormalContacts( currentItem ) )
            {
                const int candidateLayer = aBoard.FirstCommonLayer( currentItem, candidate );
                if( candidateLayer < 0 )
                    continue;

                const auto candidatePoint =
                        aBoard.NormalContactPoint( currentItem, candidate );
                if( !candidatePoint )
                {
                    forkFound = true;
                    break;
                }

                if( candidateLayer != previousLayer || *candidatePoint != *previousPoint )
                {
                    if( nextContact )
                    {
                        forkFound = true;
                        break;
                    }

                    nextContact = candidate;
                    nextPoint = candidatePoint;
                    nextLayer = candidateLayer;
                }
            }

            if( !nextContact )
                break;

            // The Java item graph is normalized before this traversal and
            // cannot loop through a connection cycle.  Fail closed if a host
            // graph violates that invariant instead of hanging the worker.
            if( connectionItems.contains( *nextContact ) )
                break;

            currentItem = *nextContact;
            previousPoint = nextPoint;
            previousLayer = nextLayer;
        }
    }

    result.m_items.assign( connectionItems.begin(), connectionItems.end() );
    result.m_itemCount = result.m_items.size();
    for( std::uint64_t id : result.m_items )
    {
        const auto current = aBoard.GetItemInfo( id );
        if( current && current->kind == ROUTING_BOARD::ITEM_KIND::TRACE )
            result.m_traceLength += current->traceLength;
    }
    result.m_complete = result.m_hasStartPoint && result.m_hasEndPoint;
    return result;
}


double CONNECTION::Detour( double aCoordinateScale ) const
{
    if( !m_hasStartPoint || !m_hasEndPoint )
        return static_cast<double>( std::numeric_limits<int>::max() );

    if( m_itemCount == 0 )
        return 0.0;

    constexpr double sourceDetourAdd = 100.0;
    constexpr double detourItemCost = 0.1;
    const double coordinateScale =
            std::isfinite( aCoordinateScale ) && aCoordinateScale > 0
                    ? aCoordinateScale : 1.0;
    const double detourAdd = sourceDetourAdd * coordinateScale;
    const double minimum = distance( m_startPoint, m_endPoint );
    return ( m_traceLength + detourAdd ) / ( minimum + detourAdd )
           + detourItemCost * static_cast<double>( m_itemCount - 1 );
}

} // namespace KICAD_AUTOROUTER
