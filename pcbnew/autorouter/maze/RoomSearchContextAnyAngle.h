/* This file is part of KiCad, licensed under GPL version 3 or later.
 * Unrestricted-angle room lifecycle translated from Freerouting a11c0a42
 * AutorouteEngine, ShapeSearchTree and SortedRoomNeighbours.
 */
#pragma once

#include "RoomSearchContext.h"
#include "../board/searchtree/ShapeSearchTree.h"
#include "../expansion/CompleteFreeSpaceExpansionRoom.h"
#include "../expansion/ExpansionDoor.h"
#include "../expansion/ObstacleExpansionRoom.h"
#include "../expansion/SortedRoomNeighbours.h"

#include <map>
#include <memory>
#include <unordered_map>

namespace KICAD_AUTOROUTER::DETAIL
{

/** Exact rational room database for Freerouting's ANY_ANGLE calculation mode.
 *
 * MIN_AREA_TREE remains a broad phase only.  Completion, neighbour ordering,
 * doors, corner smoothing, and ignored overlap shapes retain SIMPLEX support
 * lines all the way through the lifecycle.
 */
class ROOM_SEARCH_ANY_ANGLE
{
public:
    ROOM_SEARCH_ANY_ANGLE(
            ROUTER_BOX aBounds, const std::vector<SHAPE_TREE_ENTRY>& aObstacles,
            int aLayer, int aNet, double aOffset, int aMaxExpanded, int& aExpanded,
            ROOM_SEARCH_METRICS& aMetrics, const ROUTER_CANCEL_CALLBACK& aCancel,
            const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress,
            const std::vector<ROOM_RIPUP_OBSTACLE>& aRipupObstacles = {},
            int* aSharedNextId = nullptr ) :
            tree( aBounds ), layer( aLayer ), net( aNet ), offset( aOffset ),
            maxExpanded( aMaxExpanded ), expanded( aExpanded ), metrics( aMetrics ),
            cancel( aCancel ), progress( aProgress ),
            nextId( aSharedNextId ? *aSharedNextId : localNextId )
    {
        for( const SHAPE_TREE_ENTRY& entry : aObstacles )
        {
            if( cancel && cancel() )
                break;
            tree.Insert( entry );
            nextId = std::max( nextId, entry.objectId + 1 );
        }

        for( const ROOM_RIPUP_OBSTACLE& obstacle : aRipupObstacles )
        {
            if( cancel && cancel() )
                break;
            const int id = nextId++;
            const PLANAR::SIMPLEX shape = obstacle.shape.BoundingSimplex();
            if( shape.Dimension() < 0 )
                continue;
            auto room = std::make_unique<ROOM>();
            room->shape = std::make_unique<OBSTACLE_EXPANSION_ROOM>(
                    id, layer, shape, obstacle.group,
                    std::max( obstacle.ripupCost, 1 ), obstacle.shape.shapeIndex,
                    obstacle.traceInfo, obstacle.netCode );
            room->complete = true;
            ROOM* raw = room.get();
            byId.emplace( id, raw );
            byShape.emplace( room->shape.get(), raw );
            rooms.push_back( std::move( room ) );

            SHAPE_TREE_ENTRY entry = obstacle.shape;
            entry.objectId = id;
            entry.layer = layer;
            entry.isRoom = true;
            entry.obstacle = true;
            entry.simplex = shape;
            entry.shape = shape.BoundingBox().value_or( INT_BOX::Empty() );
            tree.Insert( entry );
            ++metrics.rooms;
            ++metrics.obstacleRooms;
        }

        // Consecutive shapes belonging to one movable trace share an overlap
        // door, so paying the item's rip-up cost exposes the whole chain.
        for( std::size_t first = 0; first < rooms.size(); ++first )
        {
            auto* firstObstacle = dynamic_cast<OBSTACLE_EXPANSION_ROOM*>(
                    rooms[first]->shape.get() );
            if( !firstObstacle )
                continue;
            for( std::size_t second = first + 1; second < rooms.size(); ++second )
            {
                auto* secondObstacle = dynamic_cast<OBSTACLE_EXPANSION_ROOM*>(
                        rooms[second]->shape.get() );
                if( !secondObstacle
                    || firstObstacle->GetGroup() != secondObstacle->GetGroup()
                    || std::abs( firstObstacle->GetShapeIndex()
                                 - secondObstacle->GetShapeIndex() ) != 1
                    || secondObstacle->GetSimplex().Intersection(
                               firstObstacle->GetSimplex() ).Dimension() != 2 )
                {
                    continue;
                }
                door( rooms[first].get(), rooms[second].get() );
            }
        }

        // Obstacle rooms are complete immediately.  Their surrounding free
        // rooms are only seeds and are completed when the frontier reaches
        // the obstacle, matching AutorouteEngine.completeNeighbourRooms.
        std::vector<ROOM*> obstacleRooms;
        for( const auto& room : rooms )
            if( dynamic_cast<OBSTACLE_EXPANSION_ROOM*>( room->shape.get() ) )
                obstacleRooms.push_back( room.get() );

        for( ROOM* obstacleRoom : obstacleRooms )
        {
            auto entries = neighbours( obstacleRoom->shape->GetSimplex() );
            std::erase_if( entries, [&]( const SHAPE_TREE_ENTRY& entry )
            {
                return entry.objectId == obstacleRoom->shape->GetId()
                       || entry.BoundingSimplex().Intersection(
                                  obstacleRoom->shape->GetSimplex() ).Dimension() > 1;
            } );
            SORTED_ROOM_NEIGHBOURS sorted(
                    obstacleRoom->shape->GetSimplex(), entries );
            for( const auto& gap : sorted.ObstacleIncompleteRooms( layer ) )
                door( obstacleRoom, incomplete( gap ) );
        }

        // Freerouting's board outline participates in neighbour topology.
        // The four degenerate boxes preserve the boundary support lines while
        // SHAPE_SEARCH_TREE clips unbounded rooms against the board box.
        const ROUTER_BOX bounds = tree.Bounds();
        const ROUTER_BOX borders[] = {
            { bounds.minX, bounds.minY, bounds.maxX, bounds.minY },
            { bounds.maxX, bounds.minY, bounds.maxX, bounds.maxY },
            { bounds.minX, bounds.maxY, bounds.maxX, bounds.maxY },
            { bounds.minX, bounds.minY, bounds.minX, bounds.maxY }
        };
        for( const ROUTER_BOX& border : borders )
            tree.Insert( { border, nextId++, 0, layer, 0, false, true } );
    }

    bool stopped() const
    {
        return expanded >= maxExpanded || ( cancel && cancel() );
    }

    bool step()
    {
        if( stopped() )
            return false;
        ++expanded;
        if( progress && expanded % 64 == 0 )
            progress( expanded );
        return true;
    }

    ROOM* incomplete( const INCOMPLETE_GENERAL_EXPANSION_ROOM& aSeed )
    {
        auto room = std::make_unique<ROOM>();
        room->shape = std::make_unique<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM>(
                aSeed.shape, aSeed.layer, aSeed.containedShape );
        ROOM* result = room.get();
        byShape.emplace( room->shape.get(), result );
        rooms.push_back( std::move( room ) );
        return result;
    }

    void door( ROOM* aFirst, ROOM* aSecond )
    {
        if( aFirst == aSecond || aFirst->shape->DoorExists( aSecond->shape.get() )
            || aSecond->shape->GetSimplex().Intersection(
                       aFirst->shape->GetSimplex() ).Dimension() <= 0 )
        {
            return;
        }
        auto created = std::make_unique<EXPANSION_DOOR>(
                aFirst->shape.get(), aSecond->shape.get() );
        doors.emplace( created.get(), std::move( created ) );
        ++metrics.doors;
    }

    std::vector<SHAPE_TREE_ENTRY> neighbours(
            const PLANAR::SIMPLEX& aShape ) const
    {
        auto entries = tree.Overlaps( aShape );
        std::erase_if( entries, [&]( const SHAPE_TREE_ENTRY& entry )
        {
            return entry.layer != layer || !entry.IsTraceObstacle( net );
        } );
        return entries;
    }

    ROOM* addComplete( INCOMPLETE_GENERAL_EXPANSION_ROOM aSeed )
    {
        while( step() )
        {
            const auto entries = neighbours( aSeed.shape );
            SORTED_ROOM_NEIGHBOURS sorted( aSeed.shape, entries );
            const int side = sorted.FirstUnrestrainedSide();
            if( side >= 0 )
            {
                const PLANAR::SIMPLEX enlarged = aSeed.shape.RemoveBorderLine( side );
                const auto replacements = tree.CompleteShape(
                        { enlarged, layer, aSeed.containedShape }, net,
                        {}, {}, cancel );
                if( replacements.size() == 1
                    && replacements.front().shape.Area() > aSeed.shape.Area() )
                {
                    aSeed = replacements.front();
                    continue;
                }
            }

            if( stopped() || aSeed.shape.Dimension() != 2 )
                return nullptr;

            PLANAR::SIMPLEX completedShape = aSeed.shape;
            const auto gaps = sorted.IncompleteRooms(
                    layer, aSeed.containedShape, &completedShape );
            if( completedShape.Dimension() != 2 )
                return nullptr;

            auto room = std::make_unique<ROOM>();
            const int id = nextId++;
            room->shape = std::make_unique<COMPLETE_FREE_SPACE_EXPANSION_ROOM>(
                    id, layer, completedShape );
            room->complete = true;
            ROOM* result = room.get();
            byId.emplace( id, result );
            byShape.emplace( room->shape.get(), result );
            rooms.push_back( std::move( room ) );

            for( const auto& neighbour : sorted.Neighbours() )
            {
                if( neighbour.entry.isRoom )
                {
                    const auto found = byId.find( neighbour.entry.objectId );
                    if( found != byId.end()
                        && SORTED_ROOM_NEIGHBOURS::InsertDoorOk(
                                result->shape.get(), found->second->shape.get(),
                                neighbour.intersection ) )
                    {
                        door( result, found->second );
                    }
                }
            }
            for( const auto& gap : gaps )
                door( result, incomplete( gap ) );

            SHAPE_TREE_ENTRY entry(
                    completedShape.BoundingBox().value_or( INT_BOX::Empty() ),
                    id, 0, layer, 0, true, true, {}, completedShape );
            tree.Insert( std::move( entry ) );
            ++metrics.rooms;
            return result;
        }
        return nullptr;
    }

    std::vector<ROOM*> complete( ROOM* aRoom )
    {
        std::vector<ROOM*> result;
        if( !aRoom->active || stopped() )
            return result;
        auto* incompleteRoom = dynamic_cast<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM*>(
                aRoom->shape.get() );
        if( !incompleteRoom )
            return result;

        std::optional<int> ignored;
        std::optional<PLANAR::SIMPLEX> ignoredShape;
        for( EXPANSION_DOOR* currentDoor : aRoom->shape->GetDoors() )
        {
            ROOM* other = byShape.at( currentDoor->OtherRoom( aRoom->shape.get() ) );
            if( other->complete && currentDoor->GetDimension() == 2 )
            {
                ignored = other->shape->GetId();
                ignoredShape = currentDoor->GetSimplexShape();
                break;
            }
        }

        const auto candidates = tree.CompleteShape(
                { incompleteRoom->GetSimplex(), layer,
                  incompleteRoom->GetContainedSimplex() },
                net, ignored, ignoredShape, cancel );

        const auto oldDoors = aRoom->shape->GetDoors();
        for( EXPANSION_DOOR* currentDoor : oldDoors )
        {
            currentDoor->OtherRoom( aRoom->shape.get() )->RemoveDoor( currentDoor );
            doors.erase( currentDoor );
        }
        aRoom->shape->ClearDoors();
        aRoom->active = false;

        bool first = true;
        for( const auto& candidate : candidates )
        {
            if( candidate.shape.Dimension() != 2 )
                continue;
            const auto current = first
                    ? std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM>{ candidate }
                    : tree.CompleteShape(
                            candidate, net, ignored, ignoredShape, cancel );
            first = false;
            for( const auto& recalculated : current )
                if( ROOM* added = addComplete( recalculated ) )
                    result.push_back( added );
        }
        return result;
    }

    void completeNeighbours( ROOM* aRoom )
    {
        while( !stopped() )
        {
            ROOM* pending = nullptr;
            for( EXPANSION_DOOR* currentDoor : aRoom->shape->GetDoors() )
            {
                ROOM* other = byShape.at(
                        currentDoor->OtherRoom( aRoom->shape.get() ) );
                if( !other->complete && other->active )
                {
                    pending = other;
                    break;
                }
            }
            if( !pending )
                break;
            complete( pending );
        }
    }

    SHAPE_SEARCH_TREE tree;
    int layer;
    int net;
    double offset;
    int maxExpanded;
    int& expanded;
    ROOM_SEARCH_METRICS& metrics;
    const ROUTER_CANCEL_CALLBACK& cancel;
    const ROUTER_SEARCH_PROGRESS_CALLBACK& progress;
    int localNextId = 1;
    int& nextId;
    std::vector<std::unique_ptr<ROOM>> rooms;
    std::map<int, ROOM*> byId;
    std::unordered_map<EXPANSION_ROOM*, ROOM*> byShape;
    std::unordered_map<EXPANSION_DOOR*, std::unique_ptr<EXPANSION_DOOR>> doors;
};

} // namespace KICAD_AUTOROUTER::DETAIL
