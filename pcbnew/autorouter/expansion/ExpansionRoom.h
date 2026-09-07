/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#pragma once

#include <algorithm>
#include <vector>

#include "ExpandableObject.h"
#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

class EXPANSION_DOOR;

/** Freerouting equivalent: autoroute/expansion/ExpansionRoom. */
class EXPANSION_ROOM : public EXPANDABLE_OBJECT
{
public:
    EXPANSION_ROOM( int aId, int aLayer, ROUTER_BOX aShape, bool aObstacle = false ) :
            m_id( aId ),
            m_layer( aLayer ),
            m_shape( aShape ),
            m_obstacle( aObstacle )
    {
    }

    void AddDoor( EXPANSION_DOOR* aDoor );
    const std::vector<EXPANSION_DOOR*>& GetDoors() const { return m_doors; }
    void ClearDoors() { m_doors.clear(); }
    void ResetDoors() { m_doors.clear(); }

    bool DoorExists( const EXPANSION_ROOM* aOther ) const;
    bool RemoveDoor( EXPANSION_DOOR* aDoor );

    const ROUTER_BOX& GetShape() const { return m_shape; }
    int               GetLayer() const { return m_layer; }
    int               GetId() const override { return m_id; }
    bool              IsObstacle() const { return m_obstacle; }
    bool              Contains( const ROUTER_POINT& aPoint ) const
    {
        return m_shape.Contains( aPoint );
    }

    void Reset() override { ResetDoors(); }

private:
    int                         m_id;
    int                         m_layer;
    ROUTER_BOX                  m_shape;
    bool                        m_obstacle;
    std::vector<EXPANSION_DOOR*> m_doors;
};

} // namespace KICAD_AUTOROUTER
