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

#include "DestinationDistance.h"

#include <algorithm>
#include <cmath>
#include <iterator>


namespace KICAD_AUTOROUTER
{

namespace
{

ROUTER_BOX unionBox( const ROUTER_BOX& aLeft, const ROUTER_BOX& aRight )
{
    return { std::min( aLeft.minX, aRight.minX ), std::min( aLeft.minY, aRight.minY ),
             std::max( aLeft.maxX, aRight.maxX ), std::max( aLeft.maxY, aRight.maxY ) };
}


double axisDistance( const ROUTER_POINT& aPoint, const ROUTER_BOX& aBox )
{
    const double dx = aPoint.x < aBox.minX ? static_cast<double>( aBox.minX - aPoint.x )
                                             : aPoint.x > aBox.maxX
                                                       ? static_cast<double>( aPoint.x - aBox.maxX )
                                                       : 0.0;
    const double dy = aPoint.y < aBox.minY ? static_cast<double>( aBox.minY - aPoint.y )
                                             : aPoint.y > aBox.maxY
                                                       ? static_cast<double>( aPoint.y - aBox.maxY )
                                                       : 0.0;
    return std::sqrt( dx * dx + dy * dy );
}


double minimumTraceCost( const AUTOROUTER_SETTINGS& aSettings )
{
    return static_cast<double>( std::max( 0, aSettings.traceLengthCost ) );
}

} // namespace


void DESTINATION_DISTANCE::Configure( const AUTOROUTER_SETTINGS& aSettings,
                                      const ROUTING_PAD& aTarget )
{
    m_settings = &aSettings;
    m_componentBox = { aTarget.position.x - aTarget.radius, aTarget.position.y - aTarget.radius,
                       aTarget.position.x + aTarget.radius, aTarget.position.y + aTarget.radius };
    m_innerBox = {};
    m_solderBox = {};
    m_firstOrdinal = 0;
    m_lastOrdinal = 0;
    m_hasComponentBox = false;
    m_hasInnerBox = false;
    m_hasSolderBox = false;

    if( !aSettings.layers.empty() )
    {
        m_firstOrdinal = std::numeric_limits<int>::max();
        m_lastOrdinal = std::numeric_limits<int>::min();

        for( std::size_t index = 0; index < aSettings.layers.size(); ++index )
        {
            const ROUTER_LAYER_SETTINGS& layer = aSettings.layers[index];
            const int ordinal = layer.layerOrdinal >= 0 ? layer.layerOrdinal
                                                         : static_cast<int>( index );
            m_firstOrdinal = std::min( m_firstOrdinal, ordinal );
            m_lastOrdinal = std::max( m_lastOrdinal, ordinal );
        }
    }

    for( int layer : aTarget.layers )
        Join( m_componentBox, layer );
}


void DESTINATION_DISTANCE::Join( const ROUTER_BOX& aBox, int aLayer )
{
    if( !m_settings || m_settings->layers.empty() )
    {
        if( !m_hasInnerBox )
        {
            m_innerBox = aBox;
            m_hasInnerBox = true;
        }
        else
        {
            m_innerBox = unionBox( m_innerBox, aBox );
        }

        return;
    }

    auto it = std::find_if( m_settings->layers.begin(), m_settings->layers.end(),
                            [aLayer]( const ROUTER_LAYER_SETTINGS& aSetting )
                            {
                                return aSetting.layerId == aLayer;
                            } );
    const int ordinal = it != m_settings->layers.end() && it->layerOrdinal >= 0
                                ? it->layerOrdinal
                                : it == m_settings->layers.end()
                                          ? aLayer
                                          : static_cast<int>(
                                                    std::distance( m_settings->layers.begin(), it ) );
    const int firstOrdinal = m_firstOrdinal;
    const int lastOrdinal = m_lastOrdinal;

    bool* hasBox = &m_hasInnerBox;
    ROUTER_BOX* box = &m_innerBox;

    if( ordinal <= firstOrdinal )
    {
        hasBox = &m_hasComponentBox;
        box = &m_componentBox;
    }
    else if( ordinal >= lastOrdinal )
    {
        hasBox = &m_hasSolderBox;
        box = &m_solderBox;
    }

    if( !*hasBox )
    {
        *box = aBox;
        *hasBox = true;
    }
    else
    {
        *box = unionBox( *box, aBox );
    }
}


double DESTINATION_DISTANCE::Calculate( const ROUTER_POINT& aPoint, int aLayer ) const
{
    if( !m_settings )
        return std::numeric_limits<double>::max();

    auto it = std::find_if( m_settings->layers.begin(), m_settings->layers.end(),
                            [aLayer]( const ROUTER_LAYER_SETTINGS& aSetting )
                            {
                                return aSetting.layerId == aLayer;
                            } );
    const int ordinal = it != m_settings->layers.end() && it->layerOrdinal >= 0
                                ? it->layerOrdinal
                                : it == m_settings->layers.end()
                                          ? aLayer
                                          : static_cast<int>(
                                                    std::distance( m_settings->layers.begin(), it ) );

    const ROUTER_BOX* bestBox = nullptr;
    if( ordinal <= m_firstOrdinal && m_hasComponentBox )
        bestBox = &m_componentBox;
    else if( ordinal >= m_lastOrdinal && m_hasSolderBox )
        bestBox = &m_solderBox;
    else if( m_hasInnerBox )
        bestBox = &m_innerBox;

    if( !bestBox )
    {
        if( m_hasComponentBox )
            bestBox = &m_componentBox;
        else if( m_hasSolderBox )
            bestBox = &m_solderBox;
        else if( m_hasInnerBox )
            bestBox = &m_innerBox;
    }

    if( !bestBox )
        return std::numeric_limits<double>::max();

    const double grid = std::max( 1.0, static_cast<double>( m_settings->gridStepIU ) );
    const double traceCost = minimumTraceCost( *m_settings );
    const double viaCost = std::max( 0, m_settings->viaCost );
    return axisDistance( aPoint, *bestBox ) / grid * traceCost + viaCost *
           ( ( bestBox == &m_componentBox && ordinal != m_firstOrdinal )
             || ( bestBox == &m_solderBox
                          && ordinal != m_lastOrdinal ) );
}

} // namespace KICAD_AUTOROUTER
