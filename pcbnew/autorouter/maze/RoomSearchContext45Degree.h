/* KiCad, GPL-3.0-or-later. Exact octagonal room lifecycle translated from
 * Freerouting a11c0a42 AutorouteEngine and Sorted45DegreeRoomNeighbours.
 * Internal context for the 45-degree frontier; no host dependencies.
 */
#pragma once

#include "MazeSearchEngine90Degree.h"
#include "RoomSearchContext.h"
#include "../board/searchtree/ShapeSearchTree45Degree.h"
#include "../expansion/CompleteFreeSpaceExpansionRoom.h"
#include "../expansion/ExpansionDoor.h"
#include "../expansion/ObstacleExpansionRoom.h"
#include "../expansion/Sorted45DegreeRoomNeighbours.h"

#include <map>
#include <memory>
#include <unordered_map>

namespace KICAD_AUTOROUTER::DETAIL
{
class ROOM_SEARCH_45_DEGREE
{
public:
    ROOM_SEARCH_45_DEGREE(
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
            auto room = std::make_unique<ROOM>();
            room->shape = std::make_unique<OBSTACLE_EXPANSION_ROOM>(
                    id, layer, obstacle.shape.BoundingOctagon(), obstacle.group,
                    std::max( obstacle.ripupCost, 1 ), obstacle.shape.shapeIndex );
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
            entry.octagon = raw->shape->GetOctagon();
            entry.shape = entry.octagon->BoundingBox();
            tree.Insert( entry );
            ++metrics.rooms;
            ++metrics.obstacleRooms;
        }

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
                if( !secondObstacle || firstObstacle->GetGroup() != secondObstacle->GetGroup()
                    || std::abs( firstObstacle->GetShapeIndex()
                                 - secondObstacle->GetShapeIndex() ) != 1
                    || firstObstacle->GetOctagon().Intersection(
                               secondObstacle->GetOctagon() ).Dimension() != 2 )
                {
                    continue;
                }
                door( rooms[first].get(), rooms[second].get() );
            }
        }

        std::vector<ROOM*> obstacleRooms;
        for( const auto& room : rooms )
            if( dynamic_cast<OBSTACLE_EXPANSION_ROOM*>( room->shape.get() ) )
                obstacleRooms.push_back( room.get() );
        for( ROOM* obstacleRoom : obstacleRooms )
        {
            auto entries = neighbours( obstacleRoom->shape->GetOctagon() );
            std::erase_if( entries, [&]( const SHAPE_TREE_ENTRY& entry )
            {
                return entry.objectId == obstacleRoom->shape->GetId()
                       || obstacleRoom->shape->GetOctagon().Intersection(
                                  entry.BoundingOctagon() ).Dimension() > 1;
            } );
            SORTED_45_DEGREE_ROOM_NEIGHBOURS sorted(
                    obstacleRoom->shape->GetOctagon(), entries );
            for( const auto& gap : sorted.ObstacleIncompleteRooms(
                         PLANAR::INT_OCTAGON::FromBox( tree.Bounds() ), layer ) )
            {
                door( obstacleRoom, incomplete( gap ) );
            }
        }

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

    ROOM* incomplete( const INCOMPLETE_45_DEGREE_EXPANSION_ROOM& aSeed )
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
            || aFirst->shape->GetOctagon().Intersection(
                       aSecond->shape->GetOctagon() ).Dimension() <= 0 )
        {
            return;
        }
        auto created = std::make_unique<EXPANSION_DOOR>(
                aFirst->shape.get(), aSecond->shape.get() );
        doors.emplace( created.get(), std::move( created ) );
        ++metrics.doors;
    }

    std::vector<SHAPE_TREE_ENTRY> neighbours(
            const PLANAR::INT_OCTAGON& aShape ) const
    {
        auto entries = tree.Overlaps( aShape );
        std::erase_if( entries, [&]( const SHAPE_TREE_ENTRY& entry )
        {
            return entry.layer != layer || !entry.IsTraceObstacle( net );
        } );
        return entries;
    }

    ROOM* addComplete( INCOMPLETE_45_DEGREE_EXPANSION_ROOM aSeed )
    {
        while( step() )
        {
            const auto entries = neighbours( aSeed.shape );
            SORTED_45_DEGREE_ROOM_NEIGHBOURS sorted( aSeed.shape, entries );
            bool enlarge = false;
            const auto& touches = sorted.EdgeInteriorTouchesObstacle();
            for( int edge = 0; edge < 8; ++edge )
            {
                if( touches[edge] )
                    continue;
                const ROUTER_POINT first = aSeed.shape.Corner( edge );
                const ROUTER_POINT second = aSeed.shape.Corner( ( edge + 1 ) % 8 );
                const long double dx = first.x - second.x;
                const long double dy = first.y - second.y;
                if( dx * dx + dy * dy > 1 )
                {
                    enlarge = true;
                    break;
                }
            }
            if( enlarge )
            {
                const PLANAR::INT_OCTAGON enlarged =
                        SORTED_45_DEGREE_ROOM_NEIGHBOURS::RemoveNotTouchingBorderLines(
                                aSeed.shape, touches );
                std::optional<int> ignored;
                std::optional<PLANAR::INT_OCTAGON> ignoredShape;
                double maxArea = 0;
                for( const SHAPE_TREE_ENTRY& entry : entries )
                {
                    const PLANAR::INT_OCTAGON overlap = aSeed.shape.Intersection(
                            entry.BoundingOctagon() );
                    if( entry.isRoom && overlap.Area() > maxArea )
                    {
                        ignored = entry.objectId;
                        ignoredShape = overlap;
                        maxArea = overlap.Area();
                    }
                }
                const auto replacements = tree.CompleteShape(
                        { enlarged, layer, aSeed.containedShape }, net,
                        ignored, ignoredShape, cancel );
                if( replacements.size() == 1
                    && replacements.front().shape.Area() > aSeed.shape.Area() )
                {
                    aSeed = replacements.front();
                    continue;
                }
            }

            if( stopped() || aSeed.shape.Dimension() != 2 )
                return nullptr;
            auto room = std::make_unique<ROOM>();
            const int id = nextId++;
            room->shape = std::make_unique<COMPLETE_FREE_SPACE_EXPANSION_ROOM>(
                    id, layer, aSeed.shape );
            room->complete = true;
            ROOM* result = room.get();
            byId.emplace( id, result );
            byShape.emplace( room->shape.get(), result );
            rooms.push_back( std::move( room ) );
            for( const SHAPE_TREE_ENTRY& entry : entries )
                if( entry.isRoom )
                    door( result, byId.at( entry.objectId ) );
            for( const auto& gap : sorted.IncompleteRooms(
                         PLANAR::INT_OCTAGON::FromBox( tree.Bounds() ), layer ) )
            {
                door( result, incomplete( gap ) );
            }
            tree.Insert( { aSeed.shape.BoundingBox(), id, 0, layer, 0,
                           true, true, aSeed.shape } );
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
        std::optional<PLANAR::INT_OCTAGON> ignoredShape;
        for( EXPANSION_DOOR* currentDoor : aRoom->shape->GetDoors() )
        {
            ROOM* other = byShape.at( currentDoor->OtherRoom( aRoom->shape.get() ) );
            if( other->complete && currentDoor->GetDimension() == 2 )
            {
                ignored = other->shape->GetId();
                ignoredShape = currentDoor->GetOctagonShape();
                break;
            }
        }
        const auto candidates = tree.CompleteShape(
                { incompleteRoom->GetOctagon(), layer,
                  incompleteRoom->GetContainedOctagon() },
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
                    ? std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>{ candidate }
                    : tree.CompleteShape( candidate, net, ignored, ignoredShape, cancel );
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
                ROOM* other = byShape.at( currentDoor->OtherRoom( aRoom->shape.get() ) );
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

    SHAPE_SEARCH_TREE_45_DEGREE tree;
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
