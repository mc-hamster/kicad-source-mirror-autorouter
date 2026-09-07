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

#include "ExpansionRoom.h"

#include <algorithm>

#include "ExpansionDoor.h"


namespace KICAD_AUTOROUTER
{

void EXPANSION_ROOM::AddDoor( EXPANSION_DOOR* aDoor )
{
    if( aDoor && std::find( m_doors.begin(), m_doors.end(), aDoor ) == m_doors.end() )
        m_doors.push_back( aDoor );
}


bool EXPANSION_ROOM::DoorExists( const EXPANSION_ROOM* aOther ) const
{
    return std::any_of( m_doors.begin(), m_doors.end(),
                        [aOther]( const EXPANSION_DOOR* aDoor )
                        {
                            return aDoor && aDoor->Connects( aOther );
                        } );
}


bool EXPANSION_ROOM::RemoveDoor( EXPANSION_DOOR* aDoor )
{
    const auto it = std::find( m_doors.begin(), m_doors.end(), aDoor );
    if( it == m_doors.end() )
        return false;

    m_doors.erase( it );
    return true;
}

} // namespace KICAD_AUTOROUTER
