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

#pragma once

#include <limits>

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

/**
 * Lower-bound destination estimate used by the maze frontier.
 *
 * This is the native equivalent of Freerouting's DestinationDistance.  It is
 * intentionally kept independent of the KiCad board model: the search can
 * join several target boxes (for example a pad and a plane landing point)
 * and calculate an admissible, layer-aware lower bound for each frontier
 * element.
 */
class DESTINATION_DISTANCE
{
public:
    DESTINATION_DISTANCE() = default;

    void Configure( const AUTOROUTER_SETTINGS& aSettings, const ROUTING_PAD& aTarget );
    void Join( const ROUTER_BOX& aBox, int aLayer );
    double Calculate( const ROUTER_POINT& aPoint, int aLayer ) const;

private:
    const AUTOROUTER_SETTINGS* m_settings = nullptr;
    ROUTER_BOX                 m_componentBox;
    ROUTER_BOX                 m_innerBox;
    ROUTER_BOX                 m_solderBox;
    int                        m_firstOrdinal = 0;
    int                        m_lastOrdinal = 0;
    bool                       m_hasComponentBox = false;
    bool                       m_hasInnerBox = false;
    bool                       m_hasSolderBox = false;
};

} // namespace KICAD_AUTOROUTER
