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

#include "ViaOptimizer.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>

#include "../facade/RoutingBoard.h"
#include "../../geometry/planar/ContactGeometry.h"
#include "../../maze/MazeSearchEngine.h"
#include "../../rules/ViaRule.h"


namespace KICAD_AUTOROUTER
{
namespace
{

struct COST_FACTOR
{
    double horizontal = 1.0;
    double vertical = 1.0;
};


COST_FACTOR traceCosts( const AUTOROUTER_SETTINGS& aSettings, int aLayer )
{
    const auto layer = std::find_if(
            aSettings.layers.begin(), aSettings.layers.end(),
            [&]( const ROUTER_LAYER_SETTINGS& aCandidate )
            { return aCandidate.layerId == aLayer; } );
    if( layer == aSettings.layers.end() || layer->preferredDirection == 0 )
    {
        const double preferred = std::max( 0, aSettings.traceLengthCost );
        return { preferred, preferred };
    }

    const auto [horizontal, vertical] = layer->TraceCosts( aSettings.traceLengthCost );
    return { horizontal, vertical };
}


double distance( const ROUTER_POINT& aLeft, const ROUTER_POINT& aRight )
{
    const long double dx = static_cast<long double>( aLeft.x ) - aRight.x;
    const long double dy = static_cast<long double>( aLeft.y ) - aRight.y;
    return std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
}


double weightedDistance( const ROUTER_POINT& aLeft, const ROUTER_POINT& aRight,
                         const COST_FACTOR& aCost )
{
    const long double dx = ( static_cast<long double>( aLeft.x ) - aRight.x )
                           * aCost.horizontal;
    const long double dy = ( static_cast<long double>( aLeft.y ) - aRight.y )
                           * aCost.vertical;
    return std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
}


bool collinear( const ROUTER_POINT& aFirst, const ROUTER_POINT& aSecond,
                const ROUTER_POINT& aOrigin )
{
    using CONTACT_GEOMETRY::WIDE;
    const WIDE firstX = WIDE( aFirst.x ) - aOrigin.x;
    const WIDE firstY = WIDE( aFirst.y ) - aOrigin.y;
    const WIDE secondX = WIDE( aSecond.x ) - aOrigin.x;
    const WIDE secondY = WIDE( aSecond.y ) - aOrigin.y;
    return firstX * secondY == firstY * secondX;
}


long double scalarProduct( const ROUTER_POINT& aFirst, const ROUTER_POINT& aSecond,
                           const ROUTER_POINT& aOrigin )
{
    return ( static_cast<long double>( aFirst.x ) - aOrigin.x )
                   * ( static_cast<long double>( aSecond.x ) - aOrigin.x )
           + ( static_cast<long double>( aFirst.y ) - aOrigin.y )
                   * ( static_cast<long double>( aSecond.y ) - aOrigin.y );
}


std::optional<ROUTER_POINT> changeLength( const ROUTER_POINT& aFrom,
                                          const ROUTER_POINT& aTo, double aLength )
{
    const long double dx = static_cast<long double>( aTo.x ) - aFrom.x;
    const long double dy = static_cast<long double>( aTo.y ) - aFrom.y;
    const long double oldLength = std::hypotl( dx, dy );
    if( oldLength <= 0.0L || !std::isfinite( aLength ) || aLength < 0.0 )
        return std::nullopt;

    const long double x = aFrom.x + dx * aLength / oldLength;
    const long double y = aFrom.y + dy * aLength / oldLength;
    constexpr long double minimum = static_cast<long double>(
            std::numeric_limits<std::int64_t>::min() );
    constexpr long double maximum = static_cast<long double>(
            std::numeric_limits<std::int64_t>::max() );
    if( !std::isfinite( x ) || !std::isfinite( y ) || x <= minimum || x >= maximum
        || y <= minimum || y >= maximum )
    {
        return std::nullopt;
    }

    return ROUTER_POINT{ static_cast<std::int64_t>( std::llround( x ) ),
                         static_cast<std::int64_t>( std::llround( y ) ) };
}


bool insertable( const ROUTING_CONNECTION& aCandidate, const MAZE_SEARCH_ENGINE& aSearch )
{
    if( !aCandidate.complete || !HasValidEdgeStyles( aCandidate ) )
        return false;

    for( std::size_t edge = 1; edge < aCandidate.nodes.size(); ++edge )
    {
        if( !aSearch.CanInsertSegment( aCandidate.netCode, aCandidate.nodes[edge - 1],
                                       aCandidate.nodes[edge],
                                       &aCandidate.edgeStyles[edge - 1] ) )
        {
            return false;
        }
    }

    return true;
}


bool profileAllowsSmdAttachment( const ROUTING_CONNECTION& aConnection,
                                 std::size_t aViaEdge,
                                 const BOARD_SNAPSHOT& aBoard,
                                 const AUTOROUTER_SETTINGS& aSettings )
{
    if( aSettings.allowViaInSmdPad )
        return true;

    const ROUTING_EDGE_STYLE& style = EdgeStyle( aConnection, aViaEdge - 1 );
    const ROUTING_NET* net = nullptr;
    for( const ROUTING_NET& candidate : aBoard.nets )
        if( candidate.netCode == aConnection.netCode )
        {
            net = &candidate;
            break;
        }

    if( !net )
        return false;

    return std::any_of(
            net->viaProfiles.begin(), net->viaProfiles.end(),
            [&]( const ROUTING_VIA_PROFILE& aProfile )
            {
                if( !aProfile.attachSmdAllowed )
                    return false;
                if( aProfile.diameter > 0 && style.viaDiameter > 0
                    && aProfile.diameter != style.viaDiameter )
                {
                    return false;
                }
                if( aProfile.drill > 0 && style.viaDrill > 0
                    && aProfile.drill != style.viaDrill )
                {
                    return false;
                }
                ROUTING_EDGE_STYLE profileStyle;
                profileStyle.viaDiameter = aProfile.diameter;
                profileStyle.viaDrill = aProfile.drill;
                profileStyle.viaLayers = aProfile.layers;
                profileStyle.viaType = aProfile.type;
                profileStyle.viaLayerGeometry = aProfile.layerGeometry;
                const std::vector<int> span = VIA_RULE::LayersFor(
                        aSettings, aConnection.nodes[aViaEdge - 1].layer,
                        aConnection.nodes[aViaEdge].layer, &profileStyle );
                if( !style.viaLayers.empty() && span != style.viaLayers )
                    return false;

                for( int layer : span )
                {
                    if( ViaStyleDiameterOnLayer( profileStyle, layer, aProfile.diameter )
                        != ViaStyleDiameterOnLayer( style, layer, style.viaDiameter ) )
                    {
                        return false;
                    }

                    if( ViaClearanceOnLayer( profileStyle.viaLayerGeometry, layer )
                        != ViaClearanceOnLayer( style.viaLayerGeometry, layer ) )
                    {
                        return false;
                    }
                }

                return true;
            } );
}


bool avoidsForbiddenSmdAttachment( const ROUTING_CONNECTION& aCandidate,
                                   std::size_t aViaEdge,
                                   const BOARD_SNAPSHOT& aBoard,
                                   const AUTOROUTER_SETTINGS& aSettings )
{
    if( profileAllowsSmdAttachment( aCandidate, aViaEdge, aBoard, aSettings ) )
        return true;

    const ROUTER_POINT& point = aCandidate.nodes[aViaEdge - 1].point;
    const ROUTING_EDGE_STYLE& style = EdgeStyle( aCandidate, aViaEdge - 1 );
    std::int64_t viaDiameter = style.viaDiameter;
    if( viaDiameter <= 0 )
    {
        const auto net = std::find_if(
                aBoard.nets.begin(), aBoard.nets.end(), [&]( const ROUTING_NET& aNet )
                { return aNet.netCode == aCandidate.netCode; } );
        if( net != aBoard.nets.end() )
            viaDiameter = net->viaDiameter;
    }
    const std::vector<int> viaLayers = !style.viaLayers.empty()
            ? style.viaLayers
            : std::vector<int>{ aCandidate.nodes[aViaEdge - 1].layer,
                                aCandidate.nodes[aViaEdge].layer };

    for( const ROUTING_PAD& pad : aBoard.pads )
    {
        if( pad.netCode != aCandidate.netCode || !pad.isSmd
            || pad.isFanoutTarget || pad.isPlaneTarget )
        {
            continue;
        }

        std::int64_t sharedLayerRadius = 0;
        for( int layer : viaLayers )
        {
            if( std::find( pad.layers.begin(), pad.layers.end(), layer ) == pad.layers.end() )
            {
                continue;
            }

            sharedLayerRadius =
                    std::max( sharedLayerRadius,
                              std::max<std::int64_t>( 1, ViaStyleDiameterOnLayer( style, layer, viaDiameter ) / 2 ) );
        }
        if( sharedLayerRadius <= 0 )
            continue;

        using CONTACT_GEOMETRY::WIDE;
        const WIDE dx = WIDE( point.x ) - pad.position.x;
        const WIDE dy = WIDE( point.y ) - pad.position.y;
        const WIDE minimum = WIDE( std::max<std::int64_t>( 1, pad.radius ) ) + sharedLayerRadius;
        if( dx * dx + dy * dy <= minimum * minimum )
            return false;
    }

    return true;
}


std::optional<ROUTING_CONNECTION> moveVia( const ROUTING_CONNECTION& aConnection,
                                            std::size_t aViaEdge,
                                            const ROUTER_POINT& aLocation,
                                            bool aAllowMissingBefore,
                                            bool aAllowMissingAfter,
                                            const MAZE_SEARCH_ENGINE& aSearch )
{
    if( aViaEdge == 0 || aViaEdge >= aConnection.nodes.size() )
    {
        return std::nullopt;
    }

    ROUTING_CONNECTION result = aConnection;
    EnsureEdgeStyles( result );
    const std::size_t viaStart = aViaEdge - 1;
    if( result.nodes[viaStart].layer == result.nodes[aViaEdge].layer
        || result.nodes[viaStart].point != result.nodes[aViaEdge].point
        || result.nodes[viaStart].point == aLocation )
    {
        return std::nullopt;
    }

    const bool hasBefore = viaStart > 0
                           && result.nodes[viaStart - 1].layer
                                      == result.nodes[viaStart].layer;
    const bool hasAfter = aViaEdge + 1 < result.nodes.size()
                          && result.nodes[aViaEdge + 1].layer
                                     == result.nodes[aViaEdge].layer;
    if( ( !hasBefore && !aAllowMissingBefore ) || ( !hasAfter && !aAllowMissingAfter )
        || ( !hasBefore && !hasAfter ) )
    {
        return std::nullopt;
    }

    result.nodes[viaStart].point = aLocation;
    result.nodes[aViaEdge].point = aLocation;
    if( !insertable( result, aSearch ) )
        return std::nullopt;

    return result;
}


std::optional<ROUTING_CONNECTION> furthestLegalMove(
        const ROUTING_CONNECTION& aConnection, std::size_t aViaEdge,
        const ROUTER_POINT& aTarget, std::int64_t aMinimumStep,
        bool aAllowMissingBefore, bool aAllowMissingAfter,
        const MAZE_SEARCH_ENGINE& aSearch,
        const std::function<bool( const ROUTING_CONNECTION& )>& aPlacementFilter = {} )
{
    const ROUTER_POINT from = aConnection.nodes[aViaEdge - 1].point;
    const double fullLength = distance( from, aTarget );
    if( fullLength <= 0.0 )
        return std::nullopt;

    if( auto exact = moveVia( aConnection, aViaEdge, aTarget, aAllowMissingBefore,
                              aAllowMissingAfter, aSearch );
        exact && ( !aPlacementFilter || aPlacementFilter( *exact ) ) )
    {
        return exact;
    }

    // ViaOptimizer.repositionVia() halves the attempted distance until a
    // legal DrillItemMover position is found, then accumulates legal halves.
    // Keep the same bounded monotone search, validating the complete native
    // replacement chain at every point rather than only its drill.
    double acceptedLength = 0.0;
    double increment = fullLength / 2.0;
    std::optional<ROUTING_CONNECTION> result;
    while( increment >= std::max<std::int64_t>( 1, aMinimumStep ) )
    {
        const auto point = changeLength( from, aTarget, acceptedLength + increment );
        if( point )
        {
            if( auto candidate = moveVia( aConnection, aViaEdge, *point,
                                          aAllowMissingBefore, aAllowMissingAfter,
                                          aSearch );
                candidate && ( !aPlacementFilter || aPlacementFilter( *candidate ) ) )
            {
                acceptedLength += increment;
                result = std::move( candidate );
            }
        }
        increment /= 2.0;
    }

    return result;
}

} // namespace


VIA_OPTIMIZER::VIA_EDGE_SET VIA_OPTIMIZER::MovableViaEdges(
        const ROUTING_CONNECTION& aConnection, const BOARD_SNAPSHOT& aBoard,
        const ROUTING_BOARD& aRoutingBoard )
{
    VIA_EDGE_SET result;
    if( !aConnection.complete || aConnection.nodes.size() < 3 )
        return result;

    const std::vector<ROUTING_BOARD::ITEM_ID> routeItems =
            aRoutingBoard.RouteItems( aConnection );
    const std::set<ROUTING_BOARD::ITEM_ID> routeItemSet( routeItems.begin(),
                                                         routeItems.end() );

    for( std::size_t viaEdge = 1; viaEdge < aConnection.nodes.size(); ++viaEdge )
    {
        const ROUTER_NODE& viaStart = aConnection.nodes[viaEdge - 1];
        const ROUTER_NODE& viaEnd = aConnection.nodes[viaEdge];
        if( viaStart.layer == viaEnd.layer || viaStart.point != viaEnd.point )
            continue;

        const bool hasBefore = viaEdge >= 2
                               && aConnection.nodes[viaEdge - 2].layer == viaStart.layer;
        const bool hasAfter = viaEdge + 1 < aConnection.nodes.size()
                              && aConnection.nodes[viaEdge + 1].layer == viaEnd.layer;
        const bool missingBeforeIsSynthetic = !hasBefore && viaEdge == 1
                && ( ( aConnection.fromPadIndex < aBoard.pads.size()
                       && ( aBoard.pads[aConnection.fromPadIndex].isFanoutTarget
                            || aBoard.pads[aConnection.fromPadIndex].isPlaneTarget ) )
                     || ( aConnection.isFanoutConnection
                          && aConnection.fromPadIndex >= aBoard.pads.size() ) );
        const bool missingAfterIsSynthetic = !hasAfter
                && viaEdge + 1 == aConnection.nodes.size()
                && ( ( aConnection.toPadIndex < aBoard.pads.size()
                       && ( aBoard.pads[aConnection.toPadIndex].isFanoutTarget
                            || aBoard.pads[aConnection.toPadIndex].isPlaneTarget ) )
                     || ( aConnection.isFanoutConnection
                          && aConnection.toPadIndex >= aBoard.pads.size() ) );
        if( ( !hasBefore && !missingBeforeIsSynthetic )
            || ( !hasAfter && !missingAfterIsSynthetic ) || ( !hasBefore && !hasAfter ) )
        {
            continue;
        }

        std::optional<ROUTING_BOARD::ITEM_ID> viaItem;
        for( ROUTING_BOARD::ITEM_ID itemId : routeItems )
        {
            const auto item = aRoutingBoard.GetItemInfo( itemId );
            if( item && item->kind == ROUTING_BOARD::ITEM_KIND::DRILL
                && item->first == viaStart.point )
            {
                viaItem = itemId;
                break;
            }
        }
        if( !viaItem )
            continue;

        int ownedTraceContacts = 0;
        int areaContacts = 0;
        bool unsupportedContact = false;
        const auto viaContacts = aRoutingBoard.GetNormalContacts( *viaItem );
        for( ROUTING_BOARD::ITEM_ID contactId : viaContacts )
        {
            const auto contact = aRoutingBoard.GetItemInfo( contactId );
            if( !contact )
            {
                unsupportedContact = true;
                break;
            }
            if( contact->kind == ROUTING_BOARD::ITEM_KIND::TRACE
                && routeItemSet.contains( contactId ) )
            {
                ++ownedTraceContacts;
            }
            else if( contact->kind == ROUTING_BOARD::ITEM_KIND::AREA )
            {
                ++areaContacts;
            }
            else
            {
                unsupportedContact = true;
                break;
            }
        }

        const bool twoTraceContactSet = hasBefore && hasAfter && !unsupportedContact
                                        && ownedTraceContacts == 2 && areaContacts == 0
                                        && viaContacts.size() == 2;
        const bool fanoutContactSet = hasBefore != hasAfter && aConnection.isFanoutConnection
                                      && !unsupportedContact && ownedTraceContacts == 1
                                      && areaContacts == 0 && viaContacts.size() == 1;
        const bool planeContactSet = hasBefore != hasAfter && aConnection.isPlaneConnection
                                     && !unsupportedContact && ownedTraceContacts == 1
                                     && areaContacts == 1 && viaContacts.size() == 2;
        if( twoTraceContactSet || fanoutContactSet || planeContactSet )
            result.insert( viaEdge );
    }

    return result;
}


std::vector<ROUTING_CONNECTION> VIA_OPTIMIZER::Candidates(
        const ROUTING_CONNECTION& aConnection, const BOARD_SNAPSHOT& aBoard,
        const AUTOROUTER_SETTINGS& aSettings, const ROUTING_BOARD& aRoutingBoard,
        const MAZE_SEARCH_ENGINE& aSearch, const VIA_EDGE_SET& aMovableViaEdges,
        const ROUTER_CANCEL_CALLBACK& aCancel )
{
    std::vector<ROUTING_CONNECTION> result;
    if( !aConnection.complete || aConnection.nodes.size() < 3
        || !HasValidEdgeStyles( aConnection ) )
    {
        return result;
    }

    ROUTING_CONNECTION source = aConnection;
    EnsureEdgeStyles( source );

    for( std::size_t viaEdge = 1; viaEdge < source.nodes.size(); ++viaEdge )
    {
        if( aCancel && aCancel() )
            return result;

        const ROUTER_NODE& viaStart = source.nodes[viaEdge - 1];
        const ROUTER_NODE& viaEnd = source.nodes[viaEdge];
        if( viaStart.layer == viaEnd.layer || viaStart.point != viaEnd.point )
        {
            continue;
        }

        const bool hasBefore = viaEdge >= 2
                               && source.nodes[viaEdge - 2].layer == viaStart.layer;
        const bool hasAfter = viaEdge + 1 < source.nodes.size()
                              && source.nodes[viaEdge + 1].layer == viaEnd.layer;
        const bool missingBeforeIsSynthetic = !hasBefore && viaEdge == 1
                && ( ( source.fromPadIndex < aBoard.pads.size()
                       && ( aBoard.pads[source.fromPadIndex].isFanoutTarget
                            || aBoard.pads[source.fromPadIndex].isPlaneTarget ) )
                     || ( source.isFanoutConnection
                          && source.fromPadIndex >= aBoard.pads.size() ) );
        const bool missingAfterIsSynthetic = !hasAfter && viaEdge + 1 == source.nodes.size()
                && ( ( source.toPadIndex < aBoard.pads.size()
                       && ( aBoard.pads[source.toPadIndex].isFanoutTarget
                            || aBoard.pads[source.toPadIndex].isPlaneTarget ) )
                     || ( source.isFanoutConnection
                          && source.toPadIndex >= aBoard.pads.size() ) );

        if( ( !hasBefore && !missingBeforeIsSynthetic )
            || ( !hasAfter && !missingAfterIsSynthetic ) || ( !hasBefore && !hasAfter ) )
        {
            continue;
        }
        // Captured before removal from the exact normal-contact set, matching
        // the source ViaOptimizer dispatch and preserving external branches.
        if( !aMovableViaEdges.contains( viaEdge ) )
            continue;

        // A one-trace terminal drill is the source's plane/fanout case.  It
        // may move only when its non-trace endpoint is an explicit planning
        // terminal; an ordinary pad contact is fixed and fails closed.
        if( hasBefore != hasAfter )
        {
            const ROUTER_NODE& corner = hasBefore ? source.nodes[viaEdge - 2]
                                                  : source.nodes[viaEdge + 1];
            const ROUTING_EDGE_STYLE& traceStyle = hasBefore
                    ? source.edgeStyles[viaEdge - 2] : source.edgeStyles[viaEdge];
            const std::int64_t halfWidth = std::max<std::int64_t>(
                    1, aSearch.ResolveTrackWidth( source.netCode, traceStyle ) / 2 );
            const std::int64_t minimumStep = static_cast<std::int64_t>(
                    0.3 * halfWidth ) + 1;

            std::function<bool( const ROUTING_CONNECTION& )> placementFilter;
            if( source.isFanoutConnection )
            {
                placementFilter = [&, viaEdge]( const ROUTING_CONNECTION& aCandidate )
                {
                    return avoidsForbiddenSmdAttachment( aCandidate, viaEdge, aBoard,
                                                         aSettings );
                };
            }
            if( source.isPlaneConnection )
            {
                const ROUTER_NODE originalPlanePoint = hasBefore ? viaEnd : viaStart;
                const auto contactedAreas = aRoutingBoard.ConductionAreaContactsAt(
                        source.netCode, originalPlanePoint );
                // The source accepts precisely one contact plane.  More or
                // fewer areas are ambiguous and must not be substituted by a
                // same-net area at another island.
                if( contactedAreas.size() != 1 )
                    continue;
                const auto plane = *contactedAreas.begin();
                const auto previousFilter = placementFilter;
                placementFilter = [&, plane, hasBefore, viaEdge, previousFilter](
                                          const ROUTING_CONNECTION& aCandidate )
                {
                    if( previousFilter && !previousFilter( aCandidate ) )
                        return false;
                    const ROUTER_NODE& movedPoint = hasBefore ? aCandidate.nodes[viaEdge]
                                                             : aCandidate.nodes[viaEdge - 1];
                    return aRoutingBoard.ConductionAreaContactsAt(
                                   aCandidate.netCode, movedPoint ).contains( plane );
                };
            }

            auto candidate = furthestLegalMove(
                    source, viaEdge, corner.point, minimumStep,
                    missingBeforeIsSynthetic, missingAfterIsSynthetic,
                    aSearch, placementFilter );
            if( candidate )
                result.push_back( std::move( *candidate ) );
            continue;
        }

        const ROUTER_NODE& firstCorner = source.nodes[viaEdge - 2];
        const ROUTER_NODE& secondCorner = source.nodes[viaEdge + 1];

        const COST_FACTOR firstCosts = traceCosts( aSettings, firstCorner.layer );
        const COST_FACTOR secondCosts = traceCosts( aSettings, secondCorner.layer );
        const double firstDistance = distance( viaStart.point, firstCorner.point );
        const double secondDistance = distance( viaStart.point, secondCorner.point );
        const long double scalar = scalarProduct( firstCorner.point, secondCorner.point,
                                                  viaStart.point );
        std::vector<ROUTER_POINT> targets;
        const auto addTarget = [&]( ROUTER_POINT aPoint )
        {
            if( aPoint != viaStart.point
                && std::find( targets.begin(), targets.end(), aPoint ) == targets.end() )
            {
                targets.push_back( aPoint );
            }
        };

        // Direct translation of ViaOptimizer.repositionVia's candidate order.
        // The weighted comparisons intentionally keep the source's asymmetric
        // layer-cost test: moving the drill transfers trace length from one
        // layer to the other.
        if( collinear( firstCorner.point, secondCorner.point, viaStart.point ) && scalar > 0.0L )
        {
            addTarget( secondDistance < firstDistance ? secondCorner.point
                                                      : firstCorner.point );
        }
        else
        {
            if( weightedDistance( viaStart.point, firstCorner.point, firstCosts )
                > weightedDistance( viaStart.point, firstCorner.point, secondCosts ) )
            {
                addTarget( firstCorner.point );
            }

            if( weightedDistance( viaStart.point, secondCorner.point, secondCosts )
                > weightedDistance( viaStart.point, secondCorner.point, firstCosts ) )
            {
                addTarget( secondCorner.point );
            }

            if( scalar > 0.0L )
            {
                ROUTER_POINT toPoint1;
                ROUTER_POINT toPoint2;
                if( firstDistance < secondDistance )
                {
                    toPoint1 = firstCorner.point;
                    const auto point = changeLength( viaStart.point, secondCorner.point,
                                                     firstDistance );
                    if( !point )
                        continue;
                    toPoint2 = *point;
                }
                else
                {
                    const auto point = changeLength( viaStart.point, firstCorner.point,
                                                     secondDistance );
                    if( !point )
                        continue;
                    toPoint1 = *point;
                    toPoint2 = secondCorner.point;
                }

                if( weightedDistance( toPoint1, toPoint2, firstCosts )
                    > weightedDistance( toPoint1, toPoint2, secondCosts ) )
                {
                    addTarget( toPoint1 );
                    addTarget( toPoint2 );
                }
                else
                {
                    addTarget( toPoint2 );
                    addTarget( toPoint1 );
                }
            }

            const auto addAxisDecomposition = [&]( const ROUTER_POINT& aCorner,
                                                    const COST_FACTOR& aCornerCosts,
                                                    const COST_FACTOR& aOtherCosts )
            {
                const bool orthogonal = aCorner.x == viaStart.point.x
                                        || aCorner.y == viaStart.point.y;
                if( orthogonal )
                    return;

                const double direct = weightedDistance( viaStart.point, aCorner,
                                                        aCornerCosts );
                for( ROUTER_POINT check :
                     { ROUTER_POINT{ viaStart.point.x, aCorner.y },
                       ROUTER_POINT{ aCorner.x, viaStart.point.y } } )
                {
                    const double decomposed = weightedDistance( viaStart.point, check,
                                                                aOtherCosts )
                                              + weightedDistance( check, aCorner,
                                                                  aCornerCosts );
                    if( direct > decomposed )
                        addTarget( check );
                }
            };

            addAxisDecomposition( firstCorner.point, firstCosts, secondCosts );
            addAxisDecomposition( secondCorner.point, secondCosts, firstCosts );
        }

        const ROUTING_EDGE_STYLE& firstStyle = source.edgeStyles[viaEdge - 2];
        const ROUTING_EDGE_STYLE& secondStyle = source.edgeStyles[viaEdge];
        const std::int64_t firstHalfWidth = std::max<std::int64_t>(
                1, aSearch.ResolveTrackWidth( source.netCode, firstStyle ) / 2 );
        const std::int64_t secondHalfWidth = std::max<std::int64_t>(
                1, aSearch.ResolveTrackWidth( source.netCode, secondStyle ) / 2 );
        const std::int64_t minimumStep = static_cast<std::int64_t>(
                0.3 * std::min( firstHalfWidth, secondHalfWidth ) ) + 1;

        for( const ROUTER_POINT& target : targets )
        {
            if( aCancel && aCancel() )
                return result;
            auto candidate = furthestLegalMove( source, viaEdge, target, minimumStep,
                                                false, false, aSearch );
            if( !candidate )
                continue;

            if( std::none_of( result.begin(), result.end(), [&]( const ROUTING_CONNECTION& aOther )
                              { return SameRouteGeometry( aOther, *candidate ); } ) )
            {
                result.push_back( std::move( *candidate ) );
            }
        }
    }

    return result;
}

} // namespace KICAD_AUTOROUTER
