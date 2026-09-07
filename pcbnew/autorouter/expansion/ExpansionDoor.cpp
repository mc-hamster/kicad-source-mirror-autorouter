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

#include "ExpansionDoor.h"

#include <algorithm>


namespace KICAD_AUTOROUTER
{

EXPANSION_DOOR::EXPANSION_DOOR( EXPANSION_ROOM* aFirstRoom,
                                EXPANSION_ROOM* aSecondRoom, int aDimension ) :
        m_firstRoom( aFirstRoom ),
        m_secondRoom( aSecondRoom ),
        m_dimension( aDimension )
{
    if( m_dimension < 0 )
    {
        const ROUTER_BOX shape = GetShape();
        const bool hasArea = shape.maxX > shape.minX && shape.maxY > shape.minY;
        const bool hasLine = shape.maxX > shape.minX || shape.maxY > shape.minY;
        m_dimension = hasArea ? 2 : ( hasLine ? 1 : 0 );
    }

    if( m_firstRoom )
        m_firstRoom->AddDoor( this );
    if( m_secondRoom )
        m_secondRoom->AddDoor( this );
}


EXPANSION_ROOM* EXPANSION_DOOR::OtherRoom( EXPANSION_ROOM* aRoom ) const
{
    if( aRoom == m_firstRoom )
        return m_secondRoom;
    if( aRoom == m_secondRoom )
        return m_firstRoom;
    return nullptr;
}


bool EXPANSION_DOOR::Connects( const EXPANSION_ROOM* aRoom ) const
{
    return aRoom && ( aRoom == m_firstRoom || aRoom == m_secondRoom );
}


ROUTER_BOX EXPANSION_DOOR::GetShape() const
{
    if( !m_firstRoom || !m_secondRoom )
        return {};

    const ROUTER_BOX& first = m_firstRoom->GetShape();
    const ROUTER_BOX& second = m_secondRoom->GetShape();
    return { std::max( first.minX, second.minX ), std::max( first.minY, second.minY ),
             std::min( first.maxX, second.maxX ), std::min( first.maxY, second.maxY ) };
}


int EXPANSION_DOOR::GetId() const
{
    if( !m_firstRoom || !m_secondRoom )
        return 0;

    const int first = std::min( m_firstRoom->GetId(), m_secondRoom->GetId() );
    const int second = std::max( m_firstRoom->GetId(), m_secondRoom->GetId() );
    return first * 31 + second;
}

} // namespace KICAD_AUTOROUTER
