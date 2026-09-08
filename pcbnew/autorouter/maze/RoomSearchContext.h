/* KiCad, GPL-3.0-or-later. Shared rectangular room lifecycle translated from
 * Freerouting a11c0a42 AutorouteEngine and SortedOrthogonalRoomNeighbours.
 * Internal context for single- and multilayer frontiers; no host dependencies.
 */
#pragma once
#include "MazeSearchEngine90Degree.h"
#include "../expansion/CompleteFreeSpaceExpansionRoom.h"
#include "../expansion/ExpansionDoor.h"
#include "../expansion/SortedOrthogonalRoomNeighbours.h"
#include <map>
#include <memory>
#include <unordered_map>
namespace KICAD_AUTOROUTER::DETAIL
{
struct ROOM
{
    std::unique_ptr<EXPANSION_ROOM> shape;
    bool complete = false;
    bool active = true;
};

class ROOM_SEARCH
{
public:
    ROOM_SEARCH( ROUTER_BOX bounds, const std::vector<SHAPE_TREE_ENTRY>& obstacles,
                 int layer, int net, double offset, int maxExpanded, int& expanded,
                 ROOM_SEARCH_METRICS& metrics, const ROUTER_CANCEL_CALLBACK& cancel,
                 const ROUTER_SEARCH_PROGRESS_CALLBACK& progress, int* sharedNextId = nullptr ) :
            tree( bounds ), layer( layer ), net( net ), offset( offset ),
            maxExpanded( maxExpanded ), expanded( expanded ), metrics( metrics ),
            cancel( cancel ), progress( progress ), nextId( sharedNextId ? *sharedNextId : localNextId )
    {
        for( const auto& entry : obstacles )
        {
            if( cancel && cancel() )
                break;
            tree.Insert( entry );
            nextId = std::max( nextId, entry.objectId + 1 );
        }
        // Model the four boundary restraints explicitly, as the upstream tree
        // does for the board outline. Merely clipping completion to bounds is
        // insufficient: neighbour gap generation needs these touching sides.
        const ROUTER_BOX borders[] = {
            { bounds.minX, bounds.minY, bounds.maxX, bounds.minY },
            { bounds.maxX, bounds.minY, bounds.maxX, bounds.maxY },
            { bounds.minX, bounds.maxY, bounds.maxX, bounds.maxY },
            { bounds.minX, bounds.minY, bounds.minX, bounds.maxY }
        };
        for( const auto& border : borders )
            tree.Insert( { border, nextId++, 0, layer, 0, false, true } );
    }

    bool stopped() const { return expanded >= maxExpanded || ( cancel && cancel() ); }
    bool step()
    {
        if( stopped() )
            return false;
        ++expanded;
        if( progress && expanded % 64 == 0 )
            progress( expanded );
        return true;
    }

    ROOM* incomplete( const INCOMPLETE_FREE_SPACE_EXPANSION_ROOM& seed )
    {
        auto room = std::make_unique<ROOM>();
        room->shape = std::make_unique<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM>( seed );
        ROOM* result = room.get();
        byShape.emplace( room->shape.get(), result );
        rooms.push_back( std::move( room ) );
        return result;
    }

    void door( ROOM* first, ROOM* second )
    {
        if( first == second || first->shape->DoorExists( second->shape.get() )
            || INT_BOX::Dimension( INT_BOX::Intersection( first->shape->GetShape(),
                                                          second->shape->GetShape() ) ) <= 0 )
            return;
        auto created = std::make_unique<EXPANSION_DOOR>( first->shape.get(), second->shape.get() );
        doors.emplace( created.get(), std::move( created ) );
        ++metrics.doors;
    }

    std::vector<SHAPE_TREE_ENTRY> neighbours( ROUTER_BOX box ) const
    {
        auto entries = tree.Overlaps( box );
        std::erase_if( entries, [&]( const auto& e )
        { return e.layer != layer || !e.IsTraceObstacle( net ); } );
        return entries;
    }

    ROOM* addComplete( INCOMPLETE_FREE_SPACE_EXPANSION_ROOM seed )
    {
        // Port tryRemoveEdge: first unconstrained side, greatest-area overlap
        // door (first wins ties), then restart only after strict area growth.
        while( step() )
        {
            const auto entries = neighbours( seed.GetShape() );
            SORTED_ORTHOGONAL_ROOM_NEIGHBOURS sorted( seed.GetShape(), entries );
            const int side = sorted.FirstUnrestrainedSide();
            if( side >= 0 )
            {
                auto enlarged = seed.GetShape();
                const auto& bounds = tree.Bounds();
                if( side == 0 ) enlarged.minY = bounds.minY;
                if( side == 1 ) enlarged.maxX = bounds.maxX;
                if( side == 2 ) enlarged.maxY = bounds.maxY;
                if( side == 3 ) enlarged.minX = bounds.minX;
                std::optional<int> ignored;
                std::optional<ROUTER_BOX> ignoredShape;
                double maxArea = 0;
                for( const auto& e : entries )
                {
                    const auto overlap = INT_BOX::Intersection( seed.GetShape(), e.shape );
                    if( e.isRoom && INT_BOX::Area( overlap ) > maxArea )
                    {
                        ignored = e.objectId;
                        ignoredShape = overlap;
                        maxArea = INT_BOX::Area( overlap );
                    }
                }
                const auto replacements = tree.CompleteShape(
                        { enlarged, layer, seed.GetContainedShape() }, net, ignored, ignoredShape, cancel );
                if( replacements.size() == 1
                    && INT_BOX::Area( replacements.front().GetShape() ) > INT_BOX::Area( seed.GetShape() ) )
                {
                    seed = replacements.front();
                    continue;
                }
            }
            if( stopped() || INT_BOX::Dimension( seed.GetShape() ) != 2 )
                return nullptr;
            auto room = std::make_unique<ROOM>();
            const int id = nextId++;
            room->shape = std::make_unique<COMPLETE_FREE_SPACE_EXPANSION_ROOM>( id, layer, seed.GetShape() );
            room->complete = true;
            ROOM* result = room.get();
            byId.emplace( id, result );
            byShape.emplace( room->shape.get(), result );
            rooms.push_back( std::move( room ) );
            // Existing complete-room doors are inserted in object-ID order,
            // before incomplete gaps, as in calculateNeighbours.
            for( const auto& e : entries )
                if( e.isRoom )
                    door( result, byId.at( e.objectId ) );
            for( const auto& gap : sorted.IncompleteRooms( tree.Bounds(), layer ) )
                door( result, incomplete( gap ) );
            tree.Insert( { seed.GetShape(), id, 0, layer, 0, true, true } );
            ++metrics.rooms;
            return result;
        }
        return nullptr;
    }

    std::vector<ROOM*> complete( ROOM* room )
    {
        std::vector<ROOM*> result;
        if( !room->active || stopped() )
            return result;
        std::optional<int> ignored;
        std::optional<ROUTER_BOX> ignoredShape;
        for( auto* d : room->shape->GetDoors() )
        {
            ROOM* other = byShape.at( d->OtherRoom( room->shape.get() ) );
            if( other->complete && d->GetDimension() == 2 )
            {
                ignored = other->shape->GetId();
                ignoredShape = d->GetShape();
                break;
            }
        }
        const auto candidates = tree.CompleteShape(
                static_cast<const INCOMPLETE_FREE_SPACE_EXPANSION_ROOM&>( *room->shape ),
                net, ignored, ignoredShape, cancel );
        const auto oldDoors = room->shape->GetDoors();
        for( auto* d : oldDoors )
        {
            d->OtherRoom( room->shape.get() )->RemoveDoor( d );
            doors.erase( d );
        }
        room->shape->ClearDoors();
        room->active = false;
        bool first = true;
        for( const auto& candidate : candidates )
        {
            if( INT_BOX::Dimension( candidate.GetShape() ) != 2 )
                continue;
            const auto current = first
                    ? std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM>{ candidate }
                    : tree.CompleteShape( candidate, net, ignored, ignoredShape, cancel );
            first = false;
            for( const auto& recalculated : current )
                if( ROOM* added = addComplete( recalculated ) )
                    result.push_back( added );
        }
        return result;
    }

    void completeNeighbours( ROOM* room )
    {
        // Completion mutates the door list: restart, never retain its iterator.
        while( !stopped() )
        {
            ROOM* pending = nullptr;
            for( auto* d : room->shape->GetDoors() )
            {
                ROOM* other = byShape.at( d->OtherRoom( room->shape.get() ) );
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

    SHAPE_SEARCH_TREE_90_DEGREE tree;
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
}
