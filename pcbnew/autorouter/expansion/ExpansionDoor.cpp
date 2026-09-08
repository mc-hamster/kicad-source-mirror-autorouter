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
        m_dimension = INT_BOX::Dimension( shape );
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
        return INT_BOX::Empty();

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

std::vector<FLOAT_LINE> EXPANSION_DOOR::GetSectionSegments( double aOffset,
                                                          double aTolerance,
                                                          double aMaxSectionWidth,
                                                          std::size_t aMaxSections ) const
{
    const double offset = aOffset + aTolerance;
    const double sectionWidth = aMaxSectionWidth > 0 ? aMaxSectionWidth : 10 * offset;
    if( !std::isfinite( offset ) || !std::isfinite( sectionWidth )
        || offset < 0 || sectionWidth <= 0 || m_dimension < 1 || !m_firstRoom || !m_secondRoom )
        return {};
    const auto box = GetShape();
    FLOAT_LINE line{ { static_cast<double>( box.minX ), static_cast<double>( box.minY ) },
                     { static_cast<double>( box.maxX ), static_cast<double>( box.maxY ) } };
    if( m_dimension == 2 && !m_firstRoom->IsObstacle() && !m_secondRoom->IsObstacle() )
    {
        const ROUTER_POINT corners[] = { { box.minX, box.minY }, { box.maxX, box.minY },
                                         { box.maxX, box.maxY }, { box.minX, box.maxY } };
        auto inside = []( const ROUTER_BOX& b, ROUTER_POINT p )
        { return p.x > b.minX && p.x < b.maxX && p.y > b.minY && p.y < b.maxY; };
        std::vector<FLOAT_POINT> shared;
        for( auto corner : corners )
        {
            if( !inside( m_firstRoom->GetShape(), corner )
                && !inside( m_secondRoom->GetShape(), corner ) )
                shared.push_back( { static_cast<double>( corner.x ), static_cast<double>( corner.y ) } );
            if( shared.size() == 2 )
                break;
        }
        if( shared.size() != 2 )
            return {};
        line = { shared[0], shared[1] };
        if( std::hypot( line.b.x - line.a.x, line.b.y - line.a.y ) < 2 * offset )
            return {};
    }
    else if( m_dimension == 2 )
        line = { line.Middle(), line.Middle() };

    const double length = std::hypot( line.b.x - line.a.x, line.b.y - line.a.y );
    // Refuse an over-budget door rather than silently coarsening its sections
    // or allocating unbounded memory before the caller can check cancellation.
    if( length / sectionWidth >= static_cast<double>( aMaxSections ) )
        return {};
    const std::size_t count = static_cast<std::size_t>( length / sectionWidth ) + 1;
    const double fraction = length == 0 ? 0 : std::min( offset / length, 0.5 );
    const FLOAT_POINT start{ line.a.x + ( line.b.x - line.a.x ) * fraction,
                             line.a.y + ( line.b.y - line.a.y ) * fraction };
    const FLOAT_POINT end{ line.b.x - ( line.b.x - line.a.x ) * fraction,
                           line.b.y - ( line.b.y - line.a.y ) * fraction };
    std::vector<FLOAT_LINE> result;
    result.reserve( count );
    FLOAT_POINT current = start;
    for( std::size_t i = 0; i < count; ++i )
    {
        const double part = static_cast<double>( i + 1 ) / count;
        const FLOAT_POINT next{ start.x + ( end.x - start.x ) * part,
                                start.y + ( end.y - start.y ) * part };
        result.push_back( { current, next } );
        current = next;
    }
    return result;
}

} // namespace KICAD_AUTOROUTER
