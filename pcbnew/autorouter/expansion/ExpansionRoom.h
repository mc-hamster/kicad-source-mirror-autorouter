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
#include "../geometry/planar/IntOctagon.h"
#include "../geometry/planar/Simplex.h"


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
            m_octagon( PLANAR::INT_OCTAGON::FromBox( aShape ) ),
            m_simplex( PLANAR::SIMPLEX::FromBox( aShape ) ),
            m_generalShape( false ),
            m_obstacle( aObstacle )
    {
    }

    EXPANSION_ROOM( int aId, int aLayer, PLANAR::INT_OCTAGON aShape,
                    bool aObstacle = false ) :
            m_id( aId ),
            m_layer( aLayer ),
            m_shape( aShape.BoundingBox() ),
            m_octagon( std::move( aShape ) ),
            m_simplex( m_octagon.ToSimplex().value_or( PLANAR::SIMPLEX::Empty() ) ),
            m_generalShape( false ),
            m_obstacle( aObstacle )
    {
    }

    EXPANSION_ROOM( int aId, int aLayer, PLANAR::SIMPLEX aShape,
                    bool aObstacle = false ) :
            m_id( aId ),
            m_layer( aLayer ),
            m_shape( aShape.BoundingBox().value_or( INT_BOX::Empty() ) ),
            m_octagon( aShape.BoundingOctagon().value_or(
                    PLANAR::INT_OCTAGON::Empty() ) ),
            m_simplex( std::move( aShape ) ),
            m_generalShape( true ),
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
    const PLANAR::INT_OCTAGON& GetOctagon() const { return m_octagon; }
    const PLANAR::SIMPLEX& GetSimplex() const { return m_simplex; }
    bool UsesGeneralShape() const { return m_generalShape; }
    int               GetLayer() const { return m_layer; }
    int               GetId() const override { return m_id; }
    bool              IsObstacle() const { return m_obstacle; }
    virtual bool      IsCompleteFreeSpace() const { return false; }
    bool              Contains( const ROUTER_POINT& aPoint ) const
    {
        return m_generalShape
                       ? m_simplex.Contains( PLANAR::POINT( aPoint ) )
                       : m_octagon.Contains( aPoint );
    }

    void SetShape( ROUTER_BOX aShape )
    {
        m_shape = aShape;
        m_octagon = PLANAR::INT_OCTAGON::FromBox( aShape );
        m_simplex = PLANAR::SIMPLEX::FromBox( aShape );
        m_generalShape = false;
    }
    void SetShape( PLANAR::INT_OCTAGON aShape )
    {
        m_shape = aShape.BoundingBox();
        m_octagon = std::move( aShape );
        m_simplex = m_octagon.ToSimplex().value_or( PLANAR::SIMPLEX::Empty() );
        m_generalShape = false;
    }
    void SetShape( PLANAR::SIMPLEX aShape )
    {
        m_shape = aShape.BoundingBox().value_or( INT_BOX::Empty() );
        m_octagon = aShape.BoundingOctagon().value_or(
                PLANAR::INT_OCTAGON::Empty() );
        m_simplex = std::move( aShape );
        m_generalShape = true;
    }

    void Reset() override { ResetDoors(); }

private:
    int                         m_id;
    int                         m_layer;
    ROUTER_BOX                  m_shape;
    PLANAR::INT_OCTAGON         m_octagon;
    PLANAR::SIMPLEX             m_simplex;
    bool                        m_generalShape;
    bool                        m_obstacle;
    std::vector<EXPANSION_DOOR*> m_doors;
};

} // namespace KICAD_AUTOROUTER
