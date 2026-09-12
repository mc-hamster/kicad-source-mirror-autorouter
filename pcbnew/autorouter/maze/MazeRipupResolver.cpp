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
 * This program source code file is part of KiCad, a free EDA application.
 */

#include "MazeRipupResolver.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../board/facade/RoutingBoard.h"
#include "../path/Connection.h"


namespace KICAD_AUTOROUTER
{

double MAZE_RIPUP_RESOLVER::FanoutViaRipupCostFactor(
        std::int64_t aTraceHalfWidth, double aTraceLength )
{
    if( aTraceHalfWidth <= 0 || aTraceLength <= 0 )
        return 1.0;

    constexpr double fanoutCostConstant = 20000.0;
    const double ratio = static_cast<double>( aTraceHalfWidth ) / aTraceLength;
    return std::max( ratio * ratio * fanoutCostConstant, 1.0 );
}


double MAZE_RIPUP_RESOLVER::FanoutViaRipupCostFactor(
        const ROUTING_BOARD& aBoard, std::uint64_t aTraceItem,
        std::int64_t aFallbackTraceHalfWidth )
{
    const auto trace = aBoard.GetItemInfo( aTraceItem );
    if( !trace || trace->kind != ROUTING_BOARD::ITEM_KIND::TRACE )
        return 1.0;

    // MazeRipupResolver.calcFanoutViaRipupCostFactor tests each PolylineTrace
    // endpoint independently and protects only an endpoint with exactly one
    // normal contact.  A route-level isFanoutConnection flag is intentionally
    // insufficient: ordinary traces immediately adjacent to a one-layer pin
    // receive the same source protection.
    for( const ROUTER_POINT endpoint : { trace->first, trace->last } )
    {
        const ROUTING_BOARD::ITEM_ID_SET contacts =
                aBoard.NormalContactsAt( aTraceItem, endpoint );
        if( contacts.size() != 1 )
            continue;

        const auto contact = aBoard.GetItemInfo( *contacts.begin() );
        if( !contact )
            continue;

        const bool singleLayerPin = contact->pin && !contact->layers.empty()
                && contact->layers.front() == contact->layers.back();
        const bool shoveFixedTwoCornerTrace =
                contact->kind == ROUTING_BOARD::ITEM_KIND::TRACE
                && contact->fixedState == ROUTER_FIXED_STATE::SHOVE_FIXED
                && contact->traceCornerCount == 2;
        if( singleLayerPin || shoveFixedTwoCornerTrace )
        {
            const std::int64_t halfWidth = trace->traceHalfWidth > 0
                    ? trace->traceHalfWidth : aFallbackTraceHalfWidth;
            return FanoutViaRipupCostFactor( halfWidth, trace->traceLength );
        }
    }

    return 1.0;
}


int MAZE_RIPUP_RESOLVER::CheckRipup( const ROUTING_CONNECTION& aConnection,
                                     std::size_t aEdgeIndex,
                                     std::int64_t aFallbackTraceHalfWidth,
                                     const CONTEXT& aContext,
                                     double aRandomNumber,
                                     const std::vector<std::int64_t>&
                                             aAdditionalViaTraceHalfWidths,
                                     const CONNECTION* aTopologyConnection,
                                     double aCoordinateScale,
                                     const ROUTING_BOARD* aBoard,
                                     std::uint64_t aObstacleItem ) const
{
    if( !aConnection.complete || !HasValidEdgeStyles( aConnection )
        || aEdgeIndex + 1 >= aConnection.nodes.size() )
    {
        return -1;
    }

    const ROUTER_NODE& from = aConnection.nodes[aEdgeIndex];
    const ROUTER_NODE& to = aConnection.nodes[aEdgeIndex + 1];
    const bool isVia = from.layer != to.layer;
    const auto edgeHalfWidth = [&]( std::size_t aEdge )
    {
        const ROUTING_EDGE_STYLE& edgeStyle = EdgeStyle( aConnection, aEdge );
        return std::max<std::int64_t>(
                1, edgeStyle.trackWidth > 0 ? edgeStyle.trackWidth / 2
                                            : aFallbackTraceHalfWidth );
    };

    const double coordinateScale =
            std::isfinite( aCoordinateScale ) && aCoordinateScale > 0
                    ? aCoordinateScale : 1.0;
    double costFactor = 1.0;
    double fanoutFactor = 1.0;
    const bool preserveFanoutProtection =
            !aContext.removeUnconnectedVias
            && aContext.ripupCosts <= aContext.startRipupCosts * 2;
    const auto sourceItem = aBoard && aObstacleItem != 0
            ? aBoard->GetItemInfo( aObstacleItem ) : std::nullopt;
    if( !isVia )
    {
        const std::int64_t traceHalfWidth = sourceItem
                        && sourceItem->kind == ROUTING_BOARD::ITEM_KIND::TRACE
                        && sourceItem->traceHalfWidth > 0
                ? sourceItem->traceHalfWidth : edgeHalfWidth( aEdgeIndex );
        costFactor = static_cast<double>( traceHalfWidth )
                     / coordinateScale;
        if( preserveFanoutProtection && aBoard && sourceItem )
        {
            fanoutFactor = FanoutViaRipupCostFactor(
                    *aBoard, aObstacleItem, traceHalfWidth );
        }
        else if( preserveFanoutProtection && aConnection.isFanoutConnection )
        {
            const long double dx = static_cast<long double>( to.point.x ) - from.point.x;
            const long double dy = static_cast<long double>( to.point.y ) - from.point.y;
            fanoutFactor = FanoutViaRipupCostFactor(
                    edgeHalfWidth( aEdgeIndex ),
                    std::sqrt( static_cast<double>( dx * dx + dy * dy ) ) );
        }
    }
    else
    {
        int contactCount = 0;
        if( aBoard && sourceItem
            && sourceItem->kind == ROUTING_BOARD::ITEM_KIND::DRILL )
        {
            bool lookIfFanoutVia = preserveFanoutProtection;
            for( std::uint64_t contactId : aBoard->GetNormalContacts( aObstacleItem ) )
            {
                const auto contact = aBoard->GetItemInfo( contactId );
                if( !contact || contact->kind != ROUTING_BOARD::ITEM_KIND::TRACE
                    || contact->fixedState >= ROUTER_FIXED_STATE::USER_FIXED )
                {
                    return -1;
                }

                ++contactCount;
                costFactor = std::max(
                        costFactor,
                        static_cast<double>( std::max<std::int64_t>(
                                1, contact->traceHalfWidth ) ) / coordinateScale );
                if( lookIfFanoutVia && !aContext.isFanout )
                {
                    const double currentFactor = FanoutViaRipupCostFactor(
                            *aBoard, contactId, contact->traceHalfWidth );
                    if( currentFactor > 1.0 )
                    {
                        fanoutFactor = currentFactor;
                        lookIfFanoutVia = false;
                    }
                }
            }
        }
        else if( aEdgeIndex > 0
            && aConnection.nodes[aEdgeIndex - 1].layer
                       == aConnection.nodes[aEdgeIndex].layer )
        {
            ++contactCount;
            costFactor = std::max(
                    costFactor,
                    static_cast<double>( edgeHalfWidth( aEdgeIndex - 1 ) )
                            / coordinateScale );
            if( preserveFanoutProtection && aConnection.isFanoutConnection
                && !aContext.isFanout )
            {
                const CONNECTION connection = CONNECTION::FromRoute( aConnection );
                fanoutFactor = FanoutViaRipupCostFactor(
                        edgeHalfWidth( aEdgeIndex - 1 ), connection.TraceLength() );
            }
        }
        if( !( aBoard && sourceItem
               && sourceItem->kind == ROUTING_BOARD::ITEM_KIND::DRILL )
            && aEdgeIndex + 1 < aConnection.nodes.size() - 1
            && aConnection.nodes[aEdgeIndex + 1].layer
                       == aConnection.nodes[aEdgeIndex + 2].layer )
        {
            ++contactCount;
            costFactor = std::max(
                    costFactor,
                    static_cast<double>( edgeHalfWidth( aEdgeIndex + 1 ) )
                            / coordinateScale );
        }
        if( !( aBoard && sourceItem
               && sourceItem->kind == ROUTING_BOARD::ITEM_KIND::DRILL ) )
        {
            for( const std::int64_t halfWidth : aAdditionalViaTraceHalfWidths )
            {
                ++contactCount;
                costFactor = std::max(
                        costFactor,
                        static_cast<double>( std::max<std::int64_t>( 1, halfWidth ) )
                                / coordinateScale );
            }
        }
        if( fanoutFactor <= 1.0 )
            costFactor *= 0.5 * std::max( contactCount - 1, 0 );
    }

    double detour = 1.0;
    if( fanoutFactor <= 1.0 && !aContext.isFanout )
    {
        const CONNECTION routeConnection = CONNECTION::FromRoute( aConnection );
        const CONNECTION& connection = aTopologyConnection ? *aTopologyConnection
                                                           : routeConnection;
        detour = std::max( connection.Detour( coordinateScale ), 1e-12 );
    }

    if( aContext.ripupPassNo >= 4 && aContext.ripupPassNo % 3 != 0 )
    {
        const double random = std::clamp( aRandomNumber, 0.0, 1.0 );
        detour *= 0.5 + random * random;
    }

    // MazeRipupResolver performs the narrowing cast in Freerouting's source
    // coordinate space. Casting only after multiplying by the KiCad-IU scale
    // retains a fractional source cost and changes equal-cost queue ordering.
    const double sourceCost = std::max( 0, aContext.ripupCosts )
                              * costFactor / detour * fanoutFactor;
    constexpr int maximumSourceCost = std::numeric_limits<int>::max() / 100;
    const int sourceResult = !std::isfinite( sourceCost )
            || sourceCost >= maximumSourceCost
            ? maximumSourceCost
            : std::clamp( static_cast<int>( sourceCost ), 1, maximumSourceCost );
    const long double nativeResult = static_cast<long double>( sourceResult )
                                     * coordinateScale;
    return nativeResult >= std::numeric_limits<int>::max()
            ? std::numeric_limits<int>::max()
            : std::max( static_cast<int>( nativeResult ), 1 );
}

} // namespace KICAD_AUTOROUTER
