/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Freerouting SortedOrthogonalRoomNeighbours.java, a11c0a42 (GPL-3.0).
 */
#include "SortedOrthogonalRoomNeighbours.h"
#include <tuple>

namespace KICAD_AUTOROUTER
{
SORTED_ORTHOGONAL_ROOM_NEIGHBOURS::SORTED_ORTHOGONAL_ROOM_NEIGHBOURS(
        ROUTER_BOX aRoom, const std::vector<SHAPE_TREE_ENTRY>& aEntries ) : m_room( aRoom )
{
    for( const auto& entry : aEntries )
    {
        const auto box = INT_BOX::Intersection( aRoom, entry.shape );
        if( INT_BOX::Dimension( box ) < 0 )
            continue;
        m_edgeTouches[0] |= box.minY == aRoom.minY && box.maxX > aRoom.minX && box.minX < aRoom.maxX;
        m_edgeTouches[1] |= box.maxX == aRoom.maxX && box.maxY > aRoom.minY && box.minY < aRoom.maxY;
        m_edgeTouches[2] |= box.maxY == aRoom.maxY && box.maxX > aRoom.minX && box.minX < aRoom.maxX;
        m_edgeTouches[3] |= box.minX == aRoom.minX && box.maxY > aRoom.minY && box.minY < aRoom.maxY;
        const int first = box.minY == aRoom.minY && box.minX > aRoom.minX ? 0
                          : box.maxX == aRoom.maxX && box.minY > aRoom.minY ? 1
                          : box.maxY == aRoom.maxY ? 2 : box.minX == aRoom.minX ? 3 : -1;
        const int last = box.minX == aRoom.minX && box.minY > aRoom.minY ? 3
                         : box.maxY == aRoom.maxY && box.minX > aRoom.minX ? 2
                         : box.maxX == aRoom.maxX ? 1 : box.minY == aRoom.minY ? 0 : -1;
        if( first >= 0 && last >= 0 )
            m_neighbours.push_back( { entry, box, first, last } );
    }
    auto key = []( const NEIGHBOUR& n )
    {
        const auto& b = n.intersection;
        const auto first = n.firstSide == 0 ? b.minX : n.firstSide == 1 ? b.minY
                           : n.firstSide == 2 ? -b.maxX : -b.maxY;
        const auto last = n.lastSide == 0 ? b.maxX : n.lastSide == 1 ? b.maxY
                          : n.lastSide == 2 ? -b.minX : -b.minY;
        return std::tuple{ n.firstSide, first, ( n.lastSide - n.firstSide + 4 ) % 4,
                           last, n.entry.objectId };
    };
    std::stable_sort( m_neighbours.begin(), m_neighbours.end(),
                      [&]( const auto& left, const auto& right ) { return key( left ) < key( right ); } );
    m_neighbours.erase( std::unique( m_neighbours.begin(), m_neighbours.end(),
                         [&]( const auto& left, const auto& right ) { return key( left ) == key( right ); } ),
                        m_neighbours.end() );
}

int SORTED_ORTHOGONAL_ROOM_NEIGHBOURS::FirstUnrestrainedSide() const
{
    for( int side = 0; side < 4; ++side )
        if( !m_edgeTouches[side] )
            return side;
    return -1;
}

std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM>
SORTED_ORTHOGONAL_ROOM_NEIGHBOURS::IncompleteRooms( ROUTER_BOX aBounds, int aLayer ) const
{
    std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM> result;
    auto insert = [&]( ROUTER_BOX box )
    {
        const auto contained = INT_BOX::Intersection( m_room, box );
        if( INT_BOX::Dimension( box ) == 2 && INT_BOX::Dimension( contained ) > 0 )
            result.emplace_back( box, aLayer, contained );
    };
    if( m_neighbours.empty() )
        return result;
    const NEIGHBOUR* prev = &m_neighbours.back();
    for( const auto& next : m_neighbours )
    {
        const auto& p = prev->intersection;
        const auto& n = next.intersection;
        if( !INT_BOX::Intersects( p, n ) )
        {
            switch( next.firstSide )
            {
            case 0:
                if( prev->lastSide == 0 )
                {
                    if( p.maxX < n.minX )
                        insert( { p.maxX, aBounds.minY, n.minX, m_room.minY } );
                }
                else if( p.minY > m_room.minY || n.minX > m_room.minX )
                    insert( { aBounds.minX, aBounds.minY, n.minX, p.minY } );
                break;
            case 1:
                if( prev->lastSide == 1 )
                {
                    if( p.maxY < n.minY )
                        insert( { m_room.maxX, p.maxY, aBounds.maxX, n.minY } );
                }
                else if( p.maxX < m_room.maxX || n.minY > m_room.minY )
                    insert( { p.maxX, aBounds.minY, aBounds.maxX, n.minY } );
                break;
            case 2:
                if( prev->lastSide == 2 )
                {
                    if( p.minX > n.maxX )
                        insert( { n.maxX, m_room.maxY, p.minX, aBounds.maxY } );
                }
                else if( p.maxY < m_room.maxY || n.maxX < m_room.maxX )
                    insert( { n.maxX, p.maxY, aBounds.maxX, aBounds.maxY } );
                break;
            case 3:
                if( prev->lastSide == 3 )
                {
                    if( p.minY > n.maxY )
                        insert( { aBounds.minX, n.maxY, m_room.minX, p.minY } );
                }
                else if( n.maxY < m_room.maxY || p.minX > m_room.minX )
                    insert( { aBounds.minX, n.maxY, p.minX, aBounds.maxY } );
                break;
            }
        }
        prev = &next;
    }
    return result;
}


std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM>
SORTED_ORTHOGONAL_ROOM_NEIGHBOURS::ObstacleIncompleteRooms(
        ROUTER_BOX aBounds, int aLayer ) const
{
    std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM> result;
    auto insert = [&]( ROUTER_BOX box )
    {
        const ROUTER_BOX contained = INT_BOX::Intersection( m_room, box );
        if( INT_BOX::Dimension( box ) == 2 && INT_BOX::Dimension( contained ) > 0 )
            result.emplace_back( box, aLayer, contained );
    };

    // Direct counterpart of
    // calculateIncompleteRoomsWithEmptyNeighbours(ObstacleExpansionRoom).
    if( m_neighbours.empty() )
    {
        insert( { aBounds.minX, aBounds.minY, aBounds.maxX, m_room.minY } );
        insert( { m_room.maxX, aBounds.minY, aBounds.maxX, aBounds.maxY } );
        insert( { aBounds.minX, m_room.maxY, aBounds.maxX, aBounds.maxY } );
        insert( { aBounds.minX, aBounds.minY, m_room.minX, aBounds.maxY } );
        return result;
    }

    const NEIGHBOUR* previous = &m_neighbours.back();
    for( const NEIGHBOUR& next : m_neighbours )
    {
        const ROUTER_BOX& p = previous->intersection;
        const ROUTER_BOX& n = next.intersection;
        if( !INT_BOX::Intersects( p, n ) )
        {
            switch( next.firstSide )
            {
            case 0:
                if( previous->lastSide == 0 )
                {
                    if( p.maxX < n.minX )
                        insert( { p.maxX, aBounds.minY, n.minX, m_room.minY } );
                }
                else if( p.minY > m_room.minY || n.minX > m_room.minX )
                {
                    // Obstacle-to-free-space doors must be one-dimensional.
                    if( previous->lastSide == 3 )
                        insert( { aBounds.minX, m_room.minY, m_room.minX, p.minY } );
                    insert( { m_room.minX, aBounds.minY, n.minX, m_room.minY } );
                }
                break;
            case 1:
                if( previous->lastSide == 1 )
                {
                    if( p.maxY < n.minY )
                        insert( { m_room.maxX, p.maxY, aBounds.maxX, n.minY } );
                }
                else if( p.maxX < m_room.maxX || n.minY > m_room.minY )
                {
                    if( previous->lastSide == 0 )
                        insert( { p.maxX, aBounds.minY, m_room.maxX, m_room.minY } );
                    insert( { m_room.maxX, m_room.minY, aBounds.maxX, n.minY } );
                }
                break;
            case 2:
                if( previous->lastSide == 2 )
                {
                    if( p.minX > n.maxX )
                        insert( { n.maxX, m_room.maxY, p.minX, aBounds.maxY } );
                }
                else if( p.maxY < m_room.maxY || n.maxX < m_room.maxX )
                {
                    if( previous->lastSide == 1 )
                        insert( { m_room.maxX, p.maxY, aBounds.maxX, m_room.maxY } );
                    insert( { n.maxX, m_room.maxY, m_room.maxX, aBounds.maxY } );
                }
                break;
            case 3:
                if( previous->lastSide == 3 )
                {
                    if( p.minY > n.maxY )
                        insert( { aBounds.minX, n.maxY, m_room.minX, p.minY } );
                }
                else if( n.maxY < m_room.maxY || p.minX > m_room.minX )
                {
                    if( previous->lastSide == 2 )
                        insert( { m_room.minX, m_room.maxY, p.minX, aBounds.maxY } );
                    insert( { aBounds.minX, n.maxY, m_room.minX, m_room.maxY } );
                }
                break;
            }
        }
        previous = &next;
    }
    return result;
}
} // namespace KICAD_AUTOROUTER
