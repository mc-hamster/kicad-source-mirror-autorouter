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

#include "BatchFanout.h"
#include "../rules/ViaRule.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

namespace KICAD_AUTOROUTER
{

namespace
{

int layerOrdinal( const AUTOROUTER_SETTINGS& aSettings, int aLayer )
{
    const auto it = std::find_if( aSettings.layers.begin(), aSettings.layers.end(),
                                  [aLayer]( const ROUTER_LAYER_SETTINGS& aSetting )
                                  {
                                      return aSetting.layerId == aLayer;
                                  } );

    if( it == aSettings.layers.end() || it->layerOrdinal < 0 )
        return it == aSettings.layers.end()
                       ? aLayer
                       : static_cast<int>( std::distance( aSettings.layers.begin(), it ) );

    return it->layerOrdinal;
}


std::vector<int> fanoutLayers( const AUTOROUTER_SETTINGS& aSettings, int aSourceLayer )
{
    std::vector<ROUTER_LAYER_SETTINGS> enabled;
    for( const ROUTER_LAYER_SETTINGS& layer : aSettings.layers )
    {
        if( layer.enabled && layer.layerId != aSourceLayer )
            enabled.push_back( layer );
    }

    if( enabled.empty() )
        return {};

    // Prefer a nearby trace landing layer. The manufactured through-via
    // still occupies the entire stack; this is not a partial-via mask.
    std::stable_sort( enabled.begin(), enabled.end(),
                      [&]( const ROUTER_LAYER_SETTINGS& aLeft,
                            const ROUTER_LAYER_SETTINGS& aRight )
                      {
                          const int leftDistance = std::abs(
                                  layerOrdinal( aSettings, aLeft.layerId )
                                  - layerOrdinal( aSettings, aSourceLayer ) );
                          const int rightDistance = std::abs(
                                  layerOrdinal( aSettings, aRight.layerId )
                                  - layerOrdinal( aSettings, aSourceLayer ) );
                          if( leftDistance != rightDistance )
                              return leftDistance < rightDistance;
                          return aLeft.layerId < aRight.layerId;
                      } );

    std::vector<int> result;
    result.reserve( enabled.size() );
    for( const ROUTER_LAYER_SETTINGS& layer : enabled )
        result.push_back( layer.layerId );
    return result;
}


std::vector<ROUTING_VIA_PROFILE> fanoutViasForNet(
        const BOARD_SNAPSHOT& aBoard, int aNetCode,
        const AUTOROUTER_SETTINGS& aSettings )
{
    std::vector<ROUTING_VIA_PROFILE> result;
    const auto net = std::find_if( aBoard.nets.begin(), aBoard.nets.end(),
                                   [aNetCode]( const ROUTING_NET& aCandidate )
                                   {
                                       return aCandidate.netCode == aNetCode;
                                   } );

    if( net == aBoard.nets.end() )
        return result;

    const auto append = [&]( ROUTING_VIA_PROFILE aProfile )
    {
        if( aProfile.diameter <= 0 || aProfile.drill <= 0
            || std::find( result.begin(), result.end(), aProfile ) != result.end() )
        {
            return;
        }

        result.push_back( std::move( aProfile ) );
    };

    // A non-empty netclass via rule remains the first choice.  When enabled,
    // RoutingBoard.fanout() appends every board ViaRule alternative after the
    // net rule, not merely the first one.  Keeping that ordered candidate set
    // lets a small padstack escape a channel that rejects the default large
    // board via instead of declaring the pin impossible prematurely.
    if( !net->viaProfiles.empty() )
    {
        for( const ROUTING_VIA_PROFILE& profile : net->viaProfiles )
            append( profile );
    }
    else if( net->viaDiameter > 0 )
    {
        append( { net->viaDiameter,
                  net->viaDrill > 0 ? net->viaDrill : 300000,
                  {},
                  false,
                  ROUTER_VIA_TYPE::THROUGH,
                  {} } );
    }

    if( !aSettings.fanoutFallbackToBoardVias )
        return result;

    // Board preset order is part of the rule: this is deliberately not a
    // smallest-via heuristic.  The search evaluates the first legal profile
    // in that declared order, matching the source ViaRule traversal.
    for( const ROUTING_VIA_DIMENSION& profile : aBoard.boardViaDimensions )
    {
        append( { profile.diameter, profile.drill, {}, false, ROUTER_VIA_TYPE::THROUGH, {} } );
    }

    return result;
}


} // namespace


std::vector<std::size_t> BATCH_FANOUT::OrderedPins(
        const BOARD_SNAPSHOT& board, FANOUT_PIN_ORDER order, const ROUTER_CANCEL_CALLBACK& cancel )
{
#if defined( __clang__ )
#pragma clang fp contract(off)
#endif
    struct COMPONENT
    {
        std::int64_t id;
        std::vector<std::size_t> pins;
    };
    std::map<std::int64_t, COMPONENT> components;
    std::int64_t maxComponent = 0;
    for( const auto& pad : board.pads )
        maxComponent = std::max<std::int64_t>( maxComponent, pad.componentId );
    for( std::size_t i = 0; i < board.pads.size(); ++i )
    {
        const auto& pad = board.pads[i];
        if( !pad.isSmd || pad.layers.size() != 1 || pad.netCode <= 0
            || pad.isFanoutTarget || pad.isPlaneTarget )
            continue;
        const auto id = pad.componentId >= 0 ? pad.componentId : maxComponent + 1 + i;
        components[id].id = id;
        components[id].pins.push_back( i );
    }
    std::vector<COMPONENT> sorted;
    for( auto& [id, component] : components )
        sorted.push_back( std::move( component ) );
    std::sort( sorted.begin(), sorted.end(), []( const auto& a, const auto& b )
    { return a.pins.size() != b.pins.size() ? a.pins.size() > b.pins.size() : a.id < b.id; } );
    std::vector<std::size_t> result;
    auto distance = []( ROUTER_POINT a, ROUTER_POINT b )
    {
        const double x = static_cast<double>( a.x ) - b.x, y = static_cast<double>( a.y ) - b.y;
        return std::sqrt( x * x + y * y );
    };
    for( auto& component : sorted )
    {
        if( cancel && cancel() )
            return {};
        double x = 0, y = 0;
        for( auto index : component.pins )
        {
            x += board.pads[index].position.x;
            y += board.pads[index].position.y;
        }
        x /= component.pins.size(); y /= component.pins.size();
        std::map<std::size_t, double> scores;
        for( auto index : component.pins )
        {
            const auto& pin = board.pads[index];
            const double dx = pin.position.x - x, dy = pin.position.y - y;
            double score = std::sqrt( dx * dx + dy * dy );
            if( order == FANOUT_PIN_ORDER::OUTER_FIRST )
                score = -score;
            else if( order == FANOUT_PIN_ORDER::PIN_INDEX )
                score = 0;
            else if( order == FANOUT_PIN_ORDER::CLOSEST_ON_NET || order == FANOUT_PIN_ORDER::DENSEST_FIRST )
            {
                score = order == FANOUT_PIN_ORDER::CLOSEST_ON_NET ? std::numeric_limits<double>::max() : 0;
                for( std::size_t j = 0; j < board.pads.size(); ++j )
                {
                    if( cancel && cancel() )
                        return {};
                    const auto& other = board.pads[j];
                    if( j == index || other.isFanoutTarget || other.isPlaneTarget )
                        continue;
                    const double d = distance( pin.position, other.position );
                    if( order == FANOUT_PIN_ORDER::CLOSEST_ON_NET && other.netCode == pin.netCode )
                        score = std::min( score, d );
                    // Source uses 20 mm and only net-assigned SMD pins here.
                    if( order == FANOUT_PIN_ORDER::DENSEST_FIRST && other.netCode > 0
                        && other.isSmd && other.layers.size() == 1 && d <= 20000000.0 )
                        score -= 1;
                }
            }
            scores[index] = score;
        }
        std::stable_sort( component.pins.begin(), component.pins.end(), [&]( auto a, auto b )
        {
            if( scores[a] != scores[b] )
                return scores[a] < scores[b];
            const auto pa = board.pads[a].pinIndex >= 0 ? board.pads[a].pinIndex : a;
            const auto pb = board.pads[b].pinIndex >= 0 ? board.pads[b].pinIndex : b;
            return pa < pb;
        } );
        result.insert( result.end(), component.pins.begin(), component.pins.end() );
    }
    return result;
}


std::vector<ROUTING_VIA_PROFILE> BATCH_FANOUT::ViaProfilesFor(
        const BOARD_SNAPSHOT& aBoard, int aNetCode,
        const AUTOROUTER_SETTINGS& aSettings )
{
    return fanoutViasForNet( aBoard, aNetCode, aSettings );
}


BOARD_SNAPSHOT BATCH_FANOUT::PrepareSnapshot( const BOARD_SNAPSHOT& aBoard,
                                              const AUTOROUTER_SETTINGS& aSettings,
                                              const ROUTER_CANCEL_CALLBACK& aCancel )
{
    if( !aSettings.enableFanout || !aSettings.allowVias || aSettings.maxFanoutPasses <= 0
        || aSettings.layers.size() < 2 )
    {
        return aBoard;
    }

    BOARD_SNAPSHOT result = aBoard;
    // BatchFanout constructs its component list from every net-assigned SMD
    // pin, not from the current ratsnest edges.  Create one immutable control
    // terminal for every such pin.  The live worker item graph below decides
    // whether the pin is already connected or has no unconnected source item,
    // so this remains safe for route-only-unconnected jobs.  Synthetic control
    // pads are not added to ROUTING_NET::padIndices because that list describes
    // real board pads and is used for widths, ordering and reporting.
    const auto orderedPins = OrderedPins( aBoard, aSettings.fanoutPinOrder, aCancel );
    for( auto orderedPin : orderedPins )
    {
        const auto netIt = std::find_if( result.nets.begin(), result.nets.end(), [&]( const auto& net )
        { return net.netCode == aBoard.pads[orderedPin].netCode; } );
        if( netIt == result.nets.end() )
            continue;
        auto& net = *netIt;
        if( aCancel && aCancel() )
            return result;

        // The control target below does not select a landing.  It only gives
        // the host's immutable ratsnest adapter an item identity for the
        // source-equivalent RoutingBoard.fanout() call.  The active room/drill
        // frontier chooses the first legal drill from the pin's connected set
        // toward its real unconnected item set.
        for( std::size_t padIndex : { orderedPin } )
        {
            if( aCancel && aCancel() )
                return result;

            if( padIndex >= result.pads.size() )
                continue;

            const ROUTING_PAD& pad = result.pads[padIndex];
            if( !pad.isSmd || pad.isPlaneTarget || pad.layers.size() != 1 )
                continue;

            // Match RoutingBoard.fanout's ViaRule traversal.  Its maze can
            // evaluate every rule alternative, so a synthetic native escape
            // must try each netclass/board profile in the same declaration
            // order rather than making the first profile a hard gate.
            const std::vector<ROUTING_VIA_PROFILE> fanoutVias =
                    fanoutViasForNet( result, net.netCode, aSettings );
            if( fanoutVias.empty() )
                continue;

            const std::vector<int> candidateLayers =
                    fanoutLayers( aSettings, pad.layers.front() );
            int controlTargetLayer = -1;

            // A control item is useful only when at least one declared ViaRule
            // can leave the source layer.  This is a topology check, not a
            // geometric preflight: padstack geometry, obstacles, ordering and
            // the actual target layer are resolved by the drill frontier.
            for( const int layer : candidateLayers )
            {
                const bool supported = std::any_of(
                        fanoutVias.begin(), fanoutVias.end(),
                        [&]( const ROUTING_VIA_PROFILE& aVia )
                        {
                            ROUTING_EDGE_STYLE style;
                            style.viaDiameter = aVia.diameter;
                            style.viaDrill = aVia.drill;
                            style.viaLayers = aVia.layers;
                            style.viaType = aVia.type;
                            style.viaLayerGeometry = aVia.layerGeometry;
                            return !VIA_RULE::LayersFor( aSettings, pad.layers.front(), layer,
                                                         &style ).empty();
                        } );
                if( supported )
                {
                    controlTargetLayer = layer;
                    break;
                }
            }

            if( controlTargetLayer < 0 )
                continue;

            ROUTING_PAD landing = pad;
            landing.position = pad.position;
            landing.layers = { controlTargetLayer };
            landing.isPlaneTarget = false;
            landing.isSmd = false;
            landing.isFanoutTarget = true;
            landing.fanoutSourceLayer = pad.layers.front();
            landing.fanoutTargetLayer = controlTargetLayer;
            landing.fanoutSourcePadIndex = padIndex;
            landing.fanoutViaDiameter = 0;
            landing.fanoutViaDrill = 0;
            landing.fanoutViaLayers.clear();
            landing.fanoutViaAttachSmdAllowed = false;
            landing.fanoutViaType = ROUTER_VIA_TYPE::AUTO;
            landing.fanoutMinEscapeLength = aSettings.allowViaInSmdPad
                    ? 0
                    : std::max(
                              std::max<std::int64_t>(
                                      1, pad.radius + pad.clearance + pad.trackWidth / 2 ),
                              std::max<std::int64_t>(
                                      0, aSettings.fanoutMinEscapeLengthIU ) );
            landing.fanoutMaxEscapeLength = aSettings.fanoutMaxEscapeLengthIU > 0
                    ? aSettings.fanoutMaxEscapeLengthIU
                    : landing.fanoutMinEscapeLength
                              + 8 * std::max<std::int64_t>( 1, aSettings.gridStepIU );
            if( landing.fanoutMaxEscapeLength < landing.fanoutMinEscapeLength )
                continue;
            landing.fanoutEscapePath.clear();
            landing.radius = 0;

            result.pads.push_back( std::move( landing ) );
        }

    }

    // RoutingBoard.fanout() never rewrites the design ratsnest through a
    // synthetic landing.  The control pads above exist only so the immutable
    // native search API can name one fanout attempt; BatchAutorouter builds
    // that temporary task graph separately.  Keep the real net graph intact
    // so a removed/redundant escape cannot strand later ordinary routing on a
    // virtual endpoint.

    return result;
}

std::vector<std::size_t> BATCH_FANOUT::PlaneTargetsFor( const BOARD_SNAPSHOT&,
                                                        const ROUTING_NET& aNet )
{
    return aNet.planeTargetIndices;
}


bool BATCH_FANOUT::HasFanoutWork( const BOARD_SNAPSHOT& aBoard, const ROUTING_NET& aNet )
{
    for( std::size_t index : aNet.planeTargetIndices )
    {
        if( index < aBoard.pads.size() && aBoard.pads[index].isPlaneTarget )
            return true;
    }

    for( std::size_t index : aNet.padIndices )
    {
        if( index < aBoard.pads.size() && aBoard.pads[index].isSmd
            && aBoard.pads[index].layers.size() == 1 )
        {
            return true;
        }
    }

    return false;
}

} // namespace KICAD_AUTOROUTER
