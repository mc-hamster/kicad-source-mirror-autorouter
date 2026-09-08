/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct translation of Freerouting ShapeSearchTree90Degree.restrainShape,
 * revision a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 * Preserve cut order and strict comparisons: they determine room topology.
 */
#include "ShapeSearchTree90Degree.h"

#include <optional>

namespace KICAD_AUTOROUTER
{
namespace
{
int dimension( const ROUTER_BOX& box )
{
    if( box.minX > box.maxX || box.minY > box.maxY )
        return -1;
    return ( box.minX < box.maxX ? 1 : 0 ) + ( box.minY < box.maxY ? 1 : 0 );
}

ROUTER_BOX intersection( const ROUTER_BOX& left, const ROUTER_BOX& right )
{
    return { std::max( left.minX, right.minX ), std::max( left.minY, right.minY ),
             std::min( left.maxX, right.maxX ), std::min( left.maxY, right.maxY ) };
}
} // namespace

std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM> SHAPE_SEARCH_TREE_90_DEGREE::CompleteShape(
        const INCOMPLETE_FREE_SPACE_EXPANSION_ROOM& aRoom, int aNet,
        std::optional<int> aIgnoreObject, std::optional<ROUTER_BOX> aIgnoreShape,
        const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM> result;
    if( m_tree.Size() == 0 || dimension( aRoom.GetContainedShape() ) < 0 )
        return result;
    ROUTER_BOX bounding = intersection( m_bounds, aRoom.GetShape() );
    result.emplace_back( bounding, aRoom.GetLayer(), aRoom.GetContainedShape() );
    const bool finished = m_tree.Visit( bounding, [&]( const SHAPE_TREE_ENTRY& entry )
    {
        if( aCancel && aCancel() )
            return false;
        if( !entry.IsTraceObstacle( aNet ) || entry.layer != aRoom.GetLayer()
            || ( aIgnoreObject && entry.objectId == *aIgnoreObject ) )
            return true;
        std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM> next;
        for( const auto& room : result )
        {
            if( INT_BOX::Overlaps( room.GetShape(), entry.shape ) )
            {
                // Preserve the upstream skip semantics; it drops this candidate,
                // rather than retaining a shape hidden by a previous overlap door.
                if( entry.isRoom && aIgnoreShape
                    && INT_BOX::Contains( *aIgnoreShape,
                                          intersection( room.GetShape(), entry.shape ) ) )
                    continue;
                const auto restrained = RestrainShape( room, entry.shape );
                next.insert( next.end(), restrained.begin(), restrained.end() );
            }
            else
                next.push_back( room );
        }
        result = std::move( next );
        bounding = INT_BOX::Empty();
        for( const auto& room : result )
            bounding = INT_BOX::Union( bounding, room.GetShape() );
        return true;
    } );
    if( !finished )
        result.clear();
    return result;
}

std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM> SHAPE_SEARCH_TREE_90_DEGREE::RestrainShape(
        const INCOMPLETE_FREE_SPACE_EXPANSION_ROOM& aRoom, const ROUTER_BOX& obstacle )
{
    std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM> result;
    const ROUTER_BOX& contained = aRoom.GetContainedShape();
    if( dimension( contained ) < 0 )
        return result;
    const ROUTER_BOX& room = aRoom.GetShape();
    std::int64_t cutDistance = 0;
    std::optional<ROUTER_BOX> restrained;

    // Pick the obstacle edge furthest from the contained shape. Tie order
    // is right, left, lower, upper, exactly as in the reference.
    if( room.minX < obstacle.maxX && room.maxX > obstacle.maxX
        && room.maxY > obstacle.minY && room.minY < obstacle.maxY )
    {
        const auto currentDistance = contained.minX - obstacle.maxX;
        if( currentDistance > cutDistance )
        {
            cutDistance = currentDistance;
            restrained = { obstacle.maxX, room.minY, room.maxX, room.maxY };
        }
    }
    if( room.minX < obstacle.minX && room.maxX > obstacle.minX
        && room.maxY > obstacle.minY && room.minY < obstacle.maxY )
    {
        const auto currentDistance = obstacle.minX - contained.maxX;
        if( currentDistance > cutDistance )
        {
            cutDistance = currentDistance;
            restrained = { room.minX, room.minY, obstacle.minX, room.maxY };
        }
    }
    if( room.minY < obstacle.minY && room.maxY > obstacle.minY
        && room.maxX > obstacle.minX && room.minX < obstacle.maxX )
    {
        const auto currentDistance = obstacle.minY - contained.maxY;
        if( currentDistance > cutDistance )
        {
            cutDistance = currentDistance;
            restrained = { room.minX, room.minY, room.maxX, obstacle.minY };
        }
    }
    if( room.minY < obstacle.maxY && room.maxY > obstacle.maxY
        && room.maxX > obstacle.minX && room.minX < obstacle.maxX )
    {
        const auto currentDistance = contained.minY - obstacle.maxY;
        if( currentDistance > cutDistance )
        {
            restrained = { room.minX, obstacle.maxY, room.maxX, room.maxY };
        }
    }
    if( restrained )
    {
        result.emplace_back( *restrained, aRoom.GetLayer(), contained );
        return result;
    }

    // The contained shape intersects the obstacle. Split it in the same
    // order as upstream and recursively restrain the remaining side.
    const ROUTER_BOX overlap = intersection( contained, obstacle );
    if( dimension( overlap ) < 0 )
        return result;
    std::optional<ROUTER_BOX> first;
    ROUTER_BOX second;
    if( overlap.minX > room.minX && overlap.minX == obstacle.minX && overlap.minX < room.maxX )
    {
        first = { room.minX, room.minY, overlap.minX, room.maxY };
        second = { overlap.minX, room.minY, room.maxX, room.maxY };
    }
    else if( overlap.maxX > room.minX && overlap.maxX == obstacle.maxX && overlap.maxX < room.maxX )
    {
        second = { room.minX, room.minY, overlap.maxX, room.maxY };
        first = { overlap.maxX, room.minY, room.maxX, room.maxY };
    }
    else if( overlap.minY > room.minY && overlap.minY == obstacle.minY && overlap.minY < room.maxY )
    {
        first = { room.minX, room.minY, room.maxX, overlap.minY };
        second = { room.minX, overlap.minY, room.maxX, room.maxY };
    }
    else if( overlap.maxY > room.minY && overlap.maxY == obstacle.maxY && overlap.maxY < room.maxY )
    {
        second = { room.minX, room.minY, room.maxX, overlap.maxY };
        first = { room.minX, overlap.maxY, room.maxX, room.maxY };
    }
    if( first )
    {
        const ROUTER_BOX newContained = intersection( contained, *first );
        if( dimension( newContained ) > 0 )
        {
            result.emplace_back( *first, aRoom.GetLayer(), newContained );
            auto rest = RestrainShape( { second, aRoom.GetLayer(), intersection( contained, second ) },
                                       obstacle );
            result.insert( result.end(), rest.begin(), rest.end() );
        }
    }
    return result;
}

} // namespace KICAD_AUTOROUTER
