/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include "../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

/** Data-only Freerouting rules.ViaRule span and transition semantics. */
class VIA_RULE
{
public:
    static std::vector<int> ThroughLayers( const AUTOROUTER_SETTINGS& aSettings )
    {
        std::vector<std::pair<int, int>> ordered;
        for( std::size_t index = 0; index < aSettings.layers.size(); ++index )
        {
            const auto& layer = aSettings.layers[index];
            ordered.emplace_back( layer.layerOrdinal >= 0 ? layer.layerOrdinal
                                                           : static_cast<int>( index ),
                                  layer.layerId );
        }
        std::stable_sort( ordered.begin(), ordered.end() );
        std::vector<int> result;
        for( const auto& [ordinal, layer] : ordered )
        {
            if( std::find( result.begin(), result.end(), layer ) == result.end() )
                result.push_back( layer );
        }
        return result;
    }

    static bool AllowsTransition( const AUTOROUTER_SETTINGS& aSettings, int aFrom, int aTo )
    {
        const auto enabled = [&]( int aLayer )
        {
            return std::any_of( aSettings.layers.begin(), aSettings.layers.end(),
                               [aLayer]( const auto& layer )
                               { return layer.layerId == aLayer && layer.enabled; } );
        };
        return aSettings.allowVias && aFrom != aTo && enabled( aFrom ) && enabled( aTo );
    }

    /** Return every physical copper layer occupied by the selected padstack.
     *
     * aFrom/aTo are the layers joined by this search edge, not necessarily
     * the selected padstack's manufactured endpoints.  Freerouting accepts a
     * ViaInfo when its from/to span contains that transition, then inserts the
     * complete selected Padstack.  Invalid or incompatible declarations fail
     * closed.
     */
    static std::vector<int> LayersFor( const AUTOROUTER_SETTINGS& aSettings, int aFrom,
                                       int aTo, const ROUTING_EDGE_STYLE* aStyle = nullptr )
    {
        const std::vector<int> physical = ThroughLayers( aSettings );
        const auto from = std::find( physical.begin(), physical.end(), aFrom );
        const auto to = std::find( physical.begin(), physical.end(), aTo );

        if( from == physical.end() || to == physical.end() )
            return {};

        if( !aStyle )
            return physical;

        if( aStyle->viaLayers.empty() )
        {
            return aStyle->viaType == ROUTER_VIA_TYPE::AUTO
                           || aStyle->viaType == ROUTER_VIA_TYPE::THROUGH
                    ? physical : std::vector<int>{};
        }

        if( aStyle->viaLayers.size() < 2 )
            return {};

        int firstOrdinal = std::numeric_limits<int>::max();
        int lastOrdinal = std::numeric_limits<int>::min();
        for( int declaredLayer : aStyle->viaLayers )
        {
            const auto declared = std::find( physical.begin(), physical.end(), declaredLayer );
            if( declared == physical.end() )
                return {};

            const int ordinal = static_cast<int>( std::distance( physical.begin(), declared ) );
            firstOrdinal = std::min( firstOrdinal, ordinal );
            lastOrdinal = std::max( lastOrdinal, ordinal );
        }

        if( firstOrdinal >= lastOrdinal )
            return {};

        const int transitionFirst = static_cast<int>(
                std::min( std::distance( physical.begin(), from ),
                          std::distance( physical.begin(), to ) ) );
        const int transitionLast = static_cast<int>(
                std::max( std::distance( physical.begin(), from ),
                          std::distance( physical.begin(), to ) ) );

        if( transitionFirst < firstOrdinal || transitionLast > lastOrdinal )
            return {};

        if( aStyle->viaType == ROUTER_VIA_TYPE::THROUGH
            && ( firstOrdinal != 0
                 || lastOrdinal + 1 != static_cast<int>( physical.size() ) ) )
        {
            return {};
        }

        if( aStyle->viaType == ROUTER_VIA_TYPE::MICROVIA )
        {
            const bool adjacent = lastOrdinal == firstOrdinal + 1;
            const bool touchesOuter = firstOrdinal == 0
                                      || lastOrdinal + 1 == static_cast<int>( physical.size() );
            if( !adjacent || !touchesOuter )
                return {};
        }

        return std::vector<int>( physical.begin() + firstOrdinal,
                                 physical.begin() + lastOrdinal + 1 );
    }

    static bool AllowsTransition( const AUTOROUTER_SETTINGS& aSettings, int aFrom, int aTo,
                                  const ROUTING_EDGE_STYLE* aStyle )
    {
        return AllowsTransition( aSettings, aFrom, aTo )
               && !LayersFor( aSettings, aFrom, aTo, aStyle ).empty();
    }

    static bool SpansLayer( const AUTOROUTER_SETTINGS& aSettings, int aFrom, int aTo,
                            const ROUTING_EDGE_STYLE& aStyle, int aLayer )
    {
        const std::vector<int> layers = LayersFor( aSettings, aFrom, aTo, &aStyle );
        return std::find( layers.begin(), layers.end(), aLayer ) != layers.end();
    }
};

} // namespace KICAD_AUTOROUTER
