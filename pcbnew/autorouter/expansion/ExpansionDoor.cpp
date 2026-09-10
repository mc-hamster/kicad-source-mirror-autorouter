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
        m_dimension = m_firstRoom && m_secondRoom
                              && !m_firstRoom->UsesGeneralShape()
                              && !m_secondRoom->UsesGeneralShape()
                      ? GetOctagonShape().Dimension()
                      : GetSimplexShape().Dimension();
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
    if( m_firstRoom && m_secondRoom && !m_firstRoom->UsesGeneralShape()
        && !m_secondRoom->UsesGeneralShape() )
    {
        return GetOctagonShape().BoundingBox();
    }
    return GetSimplexShape().BoundingBox().value_or( INT_BOX::Empty() );
}


PLANAR::INT_OCTAGON EXPANSION_DOOR::GetOctagonShape() const
{
    if( !m_firstRoom || !m_secondRoom )
        return PLANAR::INT_OCTAGON::Empty();
    if( !m_firstRoom->UsesGeneralShape() && !m_secondRoom->UsesGeneralShape() )
    {
        return m_firstRoom->GetOctagon().Intersection(
                m_secondRoom->GetOctagon() );
    }
    return GetSimplexShape().BoundingOctagon().value_or(
            PLANAR::INT_OCTAGON::Empty() );
}


PLANAR::SIMPLEX EXPANSION_DOOR::GetSimplexShape() const
{
    if( !m_firstRoom || !m_secondRoom )
        return PLANAR::SIMPLEX::Empty();

    // TileShape's double-dispatch calls other.intersection(this).  With two
    // Simplex operands that means second-room supports are concatenated first;
    // stable equal-direction normalization keeps that anchor identity.
    return m_secondRoom->GetSimplex().Intersection( m_firstRoom->GetSimplex() );
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
        || offset < 0 || sectionWidth <= 0 || !m_firstRoom || !m_secondRoom )
        return {};

    // Preserve the source's physical IntBox/IntOctagon behavior for the
    // fixed-direction frontiers.  In particular, IntOctagon gravity averages
    // all eight (possibly duplicate) indexed corners, whereas converting it
    // to a simplified Simplex changes that observable point.  General rooms
    // take the exact Simplex branch below.
    if( !m_firstRoom->UsesGeneralShape() && !m_secondRoom->UsesGeneralShape() )
    {
        const PLANAR::INT_OCTAGON doorShape = GetOctagonShape();
        if( doorShape.IsEmpty() )
            return {};

        FLOAT_LINE line;
        if( m_dimension == 1 )
        {
            const ROUTER_POINT first = doorShape.Corner( 0 );
            const ROUTER_POINT last = doorShape.Corner( 4 );
            line = { { static_cast<double>( first.x ), static_cast<double>( first.y ) },
                     { static_cast<double>( last.x ), static_cast<double>( last.y ) } };
        }
        if( m_dimension == 2 && m_firstRoom->IsCompleteFreeSpace()
            && m_secondRoom->IsCompleteFreeSpace() )
        {
            auto inside = []( const PLANAR::INT_OCTAGON& aShape,
                              ROUTER_POINT aPoint )
            {
                if( !aShape.Contains( aPoint ) )
                    return false;
                for( int edge = 0; edge < 8; ++edge )
                    if( aShape.SideOfBorderLine( aPoint.x, aPoint.y, edge ) == 0 )
                        return false;
                return true;
            };
            std::vector<FLOAT_POINT> shared;
            for( int cornerIndex = 0; cornerIndex < 8; ++cornerIndex )
            {
                const ROUTER_POINT corner = doorShape.Corner( cornerIndex );
                if( !inside( m_firstRoom->GetOctagon(), corner )
                    && !inside( m_secondRoom->GetOctagon(), corner )
                    && ( shared.empty()
                         || shared.front().x != static_cast<double>( corner.x )
                         || shared.front().y != static_cast<double>( corner.y ) ) )
                {
                    shared.push_back( { static_cast<double>( corner.x ),
                                        static_cast<double>( corner.y ) } );
                }
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
        {
            FLOAT_POINT centre;
            for( int cornerIndex = 0; cornerIndex < 8; ++cornerIndex )
            {
                const ROUTER_POINT corner = doorShape.Corner( cornerIndex );
                centre.x += static_cast<double>( corner.x );
                centre.y += static_cast<double>( corner.y );
            }
            centre.x /= 8;
            centre.y /= 8;
            line = { centre, centre };
        }
        else if( m_dimension != 1 )
        {
            FLOAT_POINT centre;
            for( int cornerIndex = 0; cornerIndex < 8; ++cornerIndex )
            {
                const ROUTER_POINT corner = doorShape.Corner( cornerIndex );
                centre.x += static_cast<double>( corner.x );
                centre.y += static_cast<double>( corner.y );
            }
            centre.x /= 8;
            centre.y /= 8;
            line = { centre, centre };
        }

        const double length = std::hypot( line.b.x - line.a.x,
                                          line.b.y - line.a.y );
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

    const PLANAR::SIMPLEX doorShape = GetSimplexShape();
    if( doorShape.IsEmpty() )
        return {};

    FLOAT_LINE line;
    if( m_dimension == 1 )
    {
        const auto diagonal = doorShape.DiagonalCornerSegment();
        if( !diagonal )
            return {};
        line = *diagonal;
    }
    if( m_dimension == 2 && m_firstRoom->IsCompleteFreeSpace()
        && m_secondRoom->IsCompleteFreeSpace() )
    {
        std::vector<FLOAT_POINT> shared;
        for( std::size_t cornerIndex = 0;
             cornerIndex < doorShape.Borders().size(); ++cornerIndex )
        {
            if( !doorShape.CornerIsBounded( cornerIndex ) )
                continue;
            const PLANAR::POINT& corner = doorShape.Corner( cornerIndex );
            if( !m_firstRoom->GetSimplex().ContainsInside( corner )
                && !m_secondRoom->GetSimplex().ContainsInside( corner ) )
            {
                const FLOAT_POINT approximate{ corner.X(), corner.Y() };
                if( shared.empty() || shared.front().x != approximate.x
                    || shared.front().y != approximate.y )
                {
                    shared.push_back( approximate );
                }
            }
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
    {
        const auto gravity = doorShape.CentreOfGravity();
        const FLOAT_POINT centre{ gravity.first, gravity.second };
        line = { centre, centre };
    }
    else if( m_dimension != 1 )
    {
        const auto gravity = doorShape.CentreOfGravity();
        const FLOAT_POINT centre{ gravity.first, gravity.second };
        line = { centre, centre };
    }

    const double length = std::hypot( line.b.x - line.a.x, line.b.y - line.a.y );
    // Refuse an over-budget door rather than silently coarsening its sections
    // or allocating unbounded memory before the caller can check cancellation.
    if( length / sectionWidth >= static_cast<double>( aMaxSections ) )
        return {};
    const int count = static_cast<int>( length / sectionWidth ) + 1;
    return line.ShrinkSegment( offset ).DivideSegmentIntoSections( count );
}

} // namespace KICAD_AUTOROUTER
