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
 * Experimental grid-normalized estimate used ONLY by the raster fallback.
 * This is not a translation of Freerouting. The source-equivalent box estimate
 * lives in DestinationDistance and runs in the room/drill frontier. Keep the
 * legacy costs isolated until the fallback itself is replaced, not just renamed.
 */
class LEGACY_DESTINATION_DISTANCE
{
public:
    LEGACY_DESTINATION_DISTANCE() = default;

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
