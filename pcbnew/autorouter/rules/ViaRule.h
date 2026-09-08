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
};

} // namespace KICAD_AUTOROUTER
