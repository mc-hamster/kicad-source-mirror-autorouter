/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include "../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

/**
 * Supported subset of Freerouting's rules/ViaRule: one through-hole padstack.
 *
 * Search entry/exit layers are NOT the manufactured drill span. Until the
 * adapter supplies explicit blind/buried padstacks, every new via crosses the
 * entire physical stack, including layers disabled for trace routing.
 */
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

    /**
     * Return every physical copper layer occupied by a via edge.  A custom
     * layer list is a span declaration, not permission to omit an
     * intermediate copper layer: a blind/buried via from layer A to C still
     * has copper on B.  Invalid custom masks fail closed by returning an
     * empty list.
     */
    static std::vector<int> LayersFor( const AUTOROUTER_SETTINGS& aSettings, int aFrom,
                                       int aTo, const ROUTING_EDGE_STYLE* aStyle = nullptr )
    {
        const std::vector<int> physical = ThroughLayers( aSettings );
        const auto from = std::find( physical.begin(), physical.end(), aFrom );
        const auto to = std::find( physical.begin(), physical.end(), aTo );

        if( from == physical.end() || to == physical.end() )
            return {};

        if( !aStyle || aStyle->viaLayers.empty() )
            return physical;

        const auto has = [&]( int aLayer )
        {
            return std::find( aStyle->viaLayers.begin(), aStyle->viaLayers.end(), aLayer )
                   != aStyle->viaLayers.end();
        };

        if( !has( aFrom ) || !has( aTo ) )
            return {};

        const auto first = std::min( from, to );
        const auto last = std::max( from, to );
        return std::vector<int>( first, last + 1 );
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
