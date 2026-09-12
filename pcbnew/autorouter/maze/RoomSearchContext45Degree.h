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
#include "../AutorouterDebug.h"

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
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
            int* aSharedNextId = nullptr,
            std::shared_ptr<MIN_AREA_TREE> aSharedTree = {},
            bool aPopulateTree = true,
            std::vector<MIN_AREA_TREE::HANDLE>* aSharedRoomHandles = nullptr ) :
            tree( aBounds, static_cast<std::int64_t>(
                                   FREEROUTING_COORDINATE_UNIT_IU ), true,
                  std::move( aSharedTree ) ),
            layer( aLayer ), net( aNet ), offset( aOffset ),
            maxExpanded( aMaxExpanded ), expanded( aExpanded ), metrics( aMetrics ),
            cancel( aCancel ), progress( aProgress ),
            nextId( aSharedNextId ? *aSharedNextId : localNextId ),
            sharedRoomHandles( aSharedRoomHandles )
    {
        // AutorouteEngine always searches a RoutingBoard whose BoardOutline is
        // present in ShapeSearchTree.  Lower-level native tests and snapshots
        // made without Edge.Cuts used to omit that SearchTreeObject entirely.
        // The completed room then had no neighbours on the board boundary, so
        // Sorted45DegreeRoomNeighbours could neither mark those sides as
        // closed nor create the free rooms between an obstacle and the edge.
        //
        // Preserve real imported BoardOutline leaves verbatim.  Only for the
        // bounds-only compatibility case, add four thin leaves immediately
        // outside the routing bounds.  A completed free room intersects each
        // as a one-dimensional boundary neighbour.  An item room extending
        // beyond the bounds overlaps it in two dimensions and therefore
        // follows BoardOutline's source branch (not a sorted door neighbour).
        // This distinction is essential for creating the free gap on the
        // opposite side of a trace that terminates at the board edge.
        const bool hasBoardOutline = std::any_of(
                aObstacles.begin(), aObstacles.end(),
                []( const SHAPE_TREE_ENTRY& aEntry )
                {
                    return aEntry.boardOutline;
                } );
        int largestObjectId = 0;
        for( const SHAPE_TREE_ENTRY& entry : aObstacles )
            largestObjectId = std::max( largestObjectId, entry.objectId );

        for( const SHAPE_TREE_ENTRY& entry : aObstacles )
        {
            if( cancel && cancel() )
                break;
            if( aPopulateTree )
                tree.Insert( entry );
            nextId = std::max( nextId, entry.objectId + 1 );
        }

        if( aPopulateTree && !hasBoardOutline
            && ( !aObstacles.empty() || !aRipupObstacles.empty() ) )
        {
            const int outlineObjectId = largestObjectId + 1;
            const std::int64_t unit = static_cast<std::int64_t>(
                    FREEROUTING_COORDINATE_UNIT_IU );
            const std::array<ROUTER_BOX, 4> boundaryShapes = {
                ROUTER_BOX{ aBounds.minX - unit, aBounds.minY - unit,
                            aBounds.maxX + unit, aBounds.minY },
                ROUTER_BOX{ aBounds.maxX, aBounds.minY - unit,
                            aBounds.maxX + unit, aBounds.maxY + unit },
                ROUTER_BOX{ aBounds.minX - unit, aBounds.maxY,
                            aBounds.maxX + unit, aBounds.maxY + unit },
                ROUTER_BOX{ aBounds.minX - unit, aBounds.minY - unit,
                            aBounds.minX, aBounds.maxY + unit }
            };
            for( std::size_t shapeIndex = 0;
                 shapeIndex < boundaryShapes.size(); ++shapeIndex )
            {
                SHAPE_TREE_ENTRY boundary{
                        boundaryShapes[shapeIndex], outlineObjectId,
                        static_cast<int>( shapeIndex ), layer, 0,
                        false, true,
                        PLANAR::INT_OCTAGON::FromBox(
                                boundaryShapes[shapeIndex] ) };
                boundary.boardOutline = true;
                tree.Insert( std::move( boundary ) );
            }
            nextId = std::max( nextId, outlineObjectId + 1 );
        }

        for( const ROOM_RIPUP_OBSTACLE& obstacle : aRipupObstacles )
        {
            if( cancel && cancel() )
                break;
            const int id = nextId++;
            auto room = std::make_unique<ROOM>();
            room->shape = std::make_unique<OBSTACLE_EXPANSION_ROOM>(
                    id, layer, obstacle.shape.BoundingOctagon(), obstacle.group,
                    std::max( obstacle.ripupCost, 1 ), obstacle.shape.shapeIndex,
                    obstacle.traceInfo, obstacle.netCode,
                    obstacle.sourceObjectId );
            room->complete = true;
            ROOM* raw = room.get();
            byId.emplace( id, raw );
            byShape.emplace( room->shape.get(), raw );
            rooms.push_back( std::move( room ) );
            physicalObstacleRooms.push_back( {
                    obstacle.shape.treeInsertionOrder,
                    obstacle.shape.shapeIndex,
                    obstacle.shape.layer,
                    obstacle.shape.BoundingOctagon(), raw } );
            autorouterDecisionLog(
                    "OBSTACLE_ROOM_CREATE",
                    { { "layer", std::to_string( layer ) },
                      { "room_id", std::to_string( id ) },
                      { "group", std::to_string( obstacle.group ) },
                      { "source_object_id",
                        std::to_string( obstacle.sourceObjectId ) },
                      { "tree_order",
                        std::to_string( obstacle.shape.treeInsertionOrder ) },
                      { "shape_index",
                        std::to_string( obstacle.shape.shapeIndex ) },
                      { "shape_bounds", autorouterDecisionBounds(
                                                obstacle.shape.shape ) } } );
            SHAPE_TREE_ENTRY entry = obstacle.shape;
            entry.objectId = id;
            entry.layer = layer;
            // The source search tree stores the PolylineTrace/Via item, not
            // its graph-only ObstacleExpansionRoom wrapper.
            entry.isRoom = false;
            entry.obstacle = true;
            entry.octagon = raw->shape->GetOctagon();
            entry.shape = entry.octagon->BoundingBox();
            if( aPopulateTree )
                tree.Insert( entry );
            ++metrics.rooms;
            ++metrics.obstacleRooms;
        }

        // Imported boards reach this point with their exact BoardOutline tree
        // shapes.  The compatibility leaves above are used only when the
        // caller supplied bounds but no outline object at all.
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

    ROOM* roomForId( int aId, const char* aOperation ) const
    {
        const auto found = byId.find( aId );
        if( found != byId.end() )
            return found->second;

        throw std::logic_error(
                std::string( "45-degree room id missing during " ) + aOperation
                + ": net=" + std::to_string( net )
                + " layer=" + std::to_string( layer )
                + " room_id=" + std::to_string( aId ) );
    }

    ROOM* roomForShape( EXPANSION_ROOM* aShape, const char* aOperation ) const
    {
        const auto found = byShape.find( aShape );
        if( found != byShape.end() )
            return found->second;

        throw std::logic_error(
                std::string( "45-degree room shape missing during " ) + aOperation
                + ": net=" + std::to_string( net )
                + " layer=" + std::to_string( layer )
                + " room_id=" + ( aShape ? std::to_string( aShape->GetId() ) : "null" ) );
    }

    ROOM* incomplete( const INCOMPLETE_45_DEGREE_EXPANSION_ROOM& aSeed )
    {
        auto room = std::make_unique<ROOM>();
        room->shape = std::make_unique<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM>(
                aSeed.shape, aSeed.layer, aSeed.containedShape );
        ROOM* result = room.get();
        byShape.emplace( room->shape.get(), result );
        rooms.push_back( std::move( room ) );
        autorouterDecisionLog(
                "ROOM_INCOMPLETE_CREATE",
                { { "layer", std::to_string( aSeed.layer ) },
                  { "shape_bounds", autorouterDecisionBounds(
                                            aSeed.shape.BoundingBox() ) },
                  { "contained_bounds", autorouterDecisionBounds(
                                                aSeed.containedShape.BoundingBox() ) },
                  { "contained_dimension", std::to_string(
                                                   aSeed.containedShape.Dimension() ) } } );
        return result;
    }

    void door( ROOM* aFirst, ROOM* aSecond )
    {
        if( aFirst == aSecond || aFirst->shape->DoorExists( aSecond->shape.get() )
            || aFirst->shape->GetOctagon().IntersectionOnGrid(
                       aSecond->shape->GetOctagon(),
                       static_cast<std::int64_t>(
                               FREEROUTING_COORDINATE_UNIT_IU ) ).Dimension() <= 0 )
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

    ROOM* obstacleRoomForEntry( const SHAPE_TREE_ENTRY& aEntry ) const
    {
        // ItemAutorouteInfo.getExpansionRoom() maps a physical Item tree leaf
        // to its graph-only ObstacleExpansionRoom.  The native minimum-area
        // tree likewise contains the physical trace leaf (isRoom == false),
        // not the wrapper allocated above.  treeInsertionOrder is the stable
        // source-item identity across the persistent-tree replay; layer,
        // shape index and exact geometry disambiguate its individual leaves.
        const auto exact = std::find_if(
                physicalObstacleRooms.begin(), physicalObstacleRooms.end(),
                [&]( const PHYSICAL_OBSTACLE_ROOM& aLink )
                {
                    return aLink.treeInsertionOrder == aEntry.treeInsertionOrder
                           && aLink.shapeIndex == aEntry.shapeIndex
                           && aLink.layer == aEntry.layer
                           && aLink.shape == aEntry.BoundingOctagon();
                } );
        if( exact != physicalObstacleRooms.end() )
            return exact->room;

        // A through-via contributes one tree shape per copper layer while the
        // host connection edge has one ordinal.  Its replayed source shape
        // index can therefore differ after all-layer grouping.  Geometry plus
        // source-item order and layer remains an exact physical-leaf key.
        const auto byGeometry = std::find_if(
                physicalObstacleRooms.begin(), physicalObstacleRooms.end(),
                [&]( const PHYSICAL_OBSTACLE_ROOM& aLink )
                {
                    return aLink.treeInsertionOrder == aEntry.treeInsertionOrder
                           && aLink.layer == aEntry.layer
                           && aLink.shape == aEntry.BoundingOctagon();
                } );
        return byGeometry == physicalObstacleRooms.end() ? nullptr
                                                         : byGeometry->room;
    }

    void sortBySourceObjectIdentity(
            std::vector<SHAPE_TREE_ENTRY>& aEntries ) const
    {
        // Sorted45DegreeRoomNeighbours.calculateNeighbours explicitly sorts
        // ShapeTree entries by SearchTreeObject id and shape index before it
        // creates doors.  Door insertion order is observable in equal-cost
        // maze expansion, so the minimum-area traversal order is not a valid
        // substitute here.
        std::stable_sort(
                aEntries.begin(), aEntries.end(),
                [&]( const SHAPE_TREE_ENTRY& aLeft,
                     const SHAPE_TREE_ENTRY& aRight )
                {
                    const auto sourceObjectId = [&](
                            const SHAPE_TREE_ENTRY& aEntry )
                    {
                        if( aEntry.isRoom )
                            return static_cast<std::uint64_t>( aEntry.objectId );
                        if( const ROOM* obstacleRoom =
                                    obstacleRoomForEntry( aEntry ) )
                        {
                            return obstacleRoom->shape->GetSearchObjectId();
                        }
                        return static_cast<std::uint64_t>( aEntry.objectId );
                    };
                    const std::uint64_t leftObjectId = sourceObjectId( aLeft );
                    const std::uint64_t rightObjectId = sourceObjectId( aRight );
                    if( leftObjectId != rightObjectId )
                        return leftObjectId < rightObjectId;
                    return aLeft.shapeIndex < aRight.shapeIndex;
                } );
    }

    ROOM* addComplete( INCOMPLETE_45_DEGREE_EXPANSION_ROOM aSeed )
    {
        autorouterDecisionLog(
                "ROOM_ADD_COMPLETE_BEGIN",
                { { "layer", std::to_string( aSeed.layer ) },
                  { "shape_bounds", autorouterDecisionBounds(
                                            aSeed.shape.BoundingBox() ) },
                  { "contained_bounds", autorouterDecisionBounds(
                                                aSeed.containedShape.BoundingBox() ) },
                  { "contained_dimension", std::to_string(
                                                   aSeed.containedShape.Dimension() ) } } );
        while( step() )
        {
            const auto entries = neighbours( aSeed.shape );
            auto sourceOrderedEntries = entries;
            sortBySourceObjectIdentity( sourceOrderedEntries );
            SORTED_45_DEGREE_ROOM_NEIGHBOURS sorted(
                    aSeed.shape, entries,
                    static_cast<std::int64_t>(
                            FREEROUTING_COORDINATE_UNIT_IU ) );
            bool enlarge = false;
            const auto touches =
                    sorted.EdgeInteriorTouchesObstacleForYDownCoordinates();
            std::string touchBits;
            touchBits.reserve( touches.size() );
            for( bool touch : touches )
                touchBits.push_back( touch ? '1' : '0' );
            autorouterDecisionLog(
                    "ROOM_EDGE_TOUCHES",
                    { { "layer", std::to_string( layer ) },
                      { "room_bounds", autorouterDecisionBounds(
                                                aSeed.shape.BoundingBox() ) },
                      { "entry_count", std::to_string( entries.size() ) },
                      { "touches", touchBits } } );
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
                        SORTED_45_DEGREE_ROOM_NEIGHBOURS::
                                RemoveNotTouchingBorderLinesWithinBounds(
                                        aSeed.shape, touches,
                                        PLANAR::INT_OCTAGON::FromBox(
                                                tree.Bounds() ),
                                        static_cast<std::int64_t>(
                                                FREEROUTING_COORDINATE_UNIT_IU ) );
                std::optional<int> ignored;
                std::optional<PLANAR::INT_OCTAGON> ignoredShape;
                double maxArea = 0;
                for( const SHAPE_TREE_ENTRY& entry : entries )
                {
                    const PLANAR::INT_OCTAGON overlap =
                            aSeed.shape.IntersectionOnGrid(
                                    entry.BoundingOctagon(),
                                    static_cast<std::int64_t>(
                                            FREEROUTING_COORDINATE_UNIT_IU ) );
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
            for( const SHAPE_TREE_ENTRY& entry : sourceOrderedEntries )
            {
                ROOM* neighbour = entry.isRoom
                                          ? roomForId( entry.objectId, "addComplete" )
                                          : obstacleRoomForEntry( entry );
                if( !neighbour )
                    continue;
                const PLANAR::INT_OCTAGON intersection =
                        aSeed.shape.IntersectionOnGrid(
                                neighbour->shape->GetOctagon(),
                                static_cast<std::int64_t>(
                                        FREEROUTING_COORDINATE_UNIT_IU ) );
                if( SORTED_ROOM_NEIGHBOURS::InsertDoorOk(
                            result->shape.get(), neighbour->shape.get(),
                            intersection ) )
                {
                    door( result, neighbour );
                }
            }
            const auto gaps = sorted.IncompleteRoomsForYDownCoordinates(
                    PLANAR::INT_OCTAGON::FromBox( tree.Bounds() ), layer );
            for( const auto& gap : gaps )
            {
                door( result, incomplete( gap ) );
            }
            const MIN_AREA_TREE::HANDLE roomHandle = tree.Insert(
                    { aSeed.shape.BoundingBox(), id, 0, layer, 0,
                      true, true, aSeed.shape } );
            if( sharedRoomHandles && roomHandle != MIN_AREA_TREE::NONE )
                sharedRoomHandles->push_back( roomHandle );
            autorouterDecisionLog(
                    "ROOM_COMPLETE_ADDED",
                    { { "layer", std::to_string( layer ) },
                      { "room_id", std::to_string( id ) },
                      { "room_bounds", autorouterDecisionBounds(
                                                aSeed.shape.BoundingBox() ) },
                      { "contained_bounds", autorouterDecisionBounds(
                                                    aSeed.containedShape.BoundingBox() ) },
                      { "contained_dimension", std::to_string(
                                                       aSeed.containedShape.Dimension() ) } } );
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
            ROOM* other = roomForShape(
                    currentDoor->OtherRoom( aRoom->shape.get() ), "complete" );
            if( other->complete && currentDoor->GetDimension() == 2 )
            {
                ignored = other->shape->GetId();
                ignoredShape = currentDoor->GetOctagonShape();
                break;
            }
        }
        autorouterDecisionLog(
                "ROOM_COMPLETE_REQUEST",
                { { "layer", std::to_string( layer ) },
                  { "shape_bounds", autorouterDecisionBounds(
                                            incompleteRoom->GetOctagon().BoundingBox() ) },
                  { "contained_bounds", autorouterDecisionBounds(
                                                incompleteRoom->GetContainedOctagon()
                                                        .BoundingBox() ) },
                  { "ignored_object", ignored ? std::to_string( *ignored ) : "" },
                  { "ignored_shape_bounds", ignoredShape
                                                       ? autorouterDecisionBounds(
                                                                 ignoredShape->BoundingBox() )
                                                       : "" } } );
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

    void calculateObstacleDoors( ROOM* aRoom )
    {
        auto* obstacle = dynamic_cast<OBSTACLE_EXPANSION_ROOM*>(
                aRoom->shape.get() );
        if( !obstacle || obstacle->AllDoorsCalculated() )
            return;

        auto entries = neighbours( obstacle->GetOctagon() );
        sortBySourceObjectIdentity( entries );
        std::vector<SHAPE_TREE_ENTRY> boundaryEntries;
        boundaryEntries.reserve( entries.size() );
        for( const SHAPE_TREE_ENTRY& entry : entries )
        {
            ROOM* neighbour = entry.isRoom
                                      ? roomForId( entry.objectId, "calculateObstacleDoors" )
                                      : obstacleRoomForEntry( entry );
            if( neighbour == aRoom )
                continue;

            const PLANAR::INT_OCTAGON intersection =
                    obstacle->GetOctagon().IntersectionOnGrid(
                            entry.BoundingOctagon(),
                            static_cast<std::int64_t>(
                                    FREEROUTING_COORDINATE_UNIT_IU ) );
            const int dimension = intersection.Dimension();
            if( dimension > 1 )
            {
                // ObstacleExpansionRoom.createOverlapDoor() is the sole legal
                // two-dimensional item-room transition.  Unlike a boundary
                // neighbour it is not used to generate incomplete free rooms.
                if( auto* other = neighbour
                                           ? dynamic_cast<OBSTACLE_EXPANSION_ROOM*>(
                                                     neighbour->shape.get() )
                                           : nullptr;
                    other && obstacle->CanCreateOverlapDoorWith( *other ) )
                {
                    door( aRoom, neighbour );
                }
                continue;
            }
            if( dimension < 0 )
                continue;

            boundaryEntries.push_back( entry );
            if( dimension > 0 && neighbour
                && SORTED_ROOM_NEIGHBOURS::InsertDoorOk(
                           aRoom->shape.get(), neighbour->shape.get(),
                           intersection ) )
            {
                door( aRoom, neighbour );
            }
        }
        SORTED_45_DEGREE_ROOM_NEIGHBOURS sorted(
                obstacle->GetOctagon(), boundaryEntries,
                static_cast<std::int64_t>(
                        FREEROUTING_COORDINATE_UNIT_IU ) );
        const auto gaps = sorted.ObstacleIncompleteRoomsForYDownCoordinates(
                PLANAR::INT_OCTAGON::FromBox( tree.Bounds() ), layer );
        autorouterDecisionLog(
                "OBSTACLE_ROOM_GAPS",
                { { "room_id", std::to_string( obstacle->GetId() ) },
                  { "entry_count", std::to_string( entries.size() ) },
                  { "boundary_entry_count",
                    std::to_string( boundaryEntries.size() ) },
                  { "neighbour_count",
                    std::to_string( sorted.Neighbours().size() ) },
                  { "gap_count", std::to_string( gaps.size() ) } } );
        for( const auto& gap : gaps )
        {
            door( aRoom, incomplete( gap ) );
        }
        obstacle->SetDoorsCalculated( true );
    }

    void completeNeighbours( ROOM* aRoom )
    {
        if( auto* obstacle = dynamic_cast<OBSTACLE_EXPANSION_ROOM*>(
                    aRoom->shape.get() );
            obstacle && !obstacle->AllDoorsCalculated() )
        {
            calculateObstacleDoors( aRoom );
        }

        while( !stopped() )
        {
            bool topologyChanged = false;
            const auto doorsSnapshot = aRoom->shape->GetDoors();
            for( EXPANSION_DOOR* currentDoor : doorsSnapshot )
            {
                ROOM* other = roomForShape(
                        currentDoor->OtherRoom( aRoom->shape.get() ), "completeNeighbours" );
                if( !other->complete && other->active )
                {
                    complete( other );
                    topologyChanged = true;
                    break;
                }
                if( auto* otherObstacle = dynamic_cast<
                            OBSTACLE_EXPANSION_ROOM*>( other->shape.get() );
                    otherObstacle && !otherObstacle->AllDoorsCalculated() )
                {
                    calculateObstacleDoors( other );
                    topologyChanged = true;
                    break;
                }
            }
            if( !topologyChanged )
                break;
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
    struct PHYSICAL_OBSTACLE_ROOM
    {
        std::uint64_t treeInsertionOrder;
        int shapeIndex;
        int layer;
        PLANAR::INT_OCTAGON shape;
        ROOM* room;
    };
    std::vector<PHYSICAL_OBSTACLE_ROOM> physicalObstacleRooms;
    std::vector<MIN_AREA_TREE::HANDLE>* sharedRoomHandles = nullptr;
    std::vector<std::unique_ptr<ROOM>> rooms;
    std::map<int, ROOM*> byId;
    std::unordered_map<EXPANSION_ROOM*, ROOM*> byShape;
    std::unordered_map<EXPANSION_DOOR*, std::unique_ptr<EXPANSION_DOOR>> doors;
};
} // namespace KICAD_AUTOROUTER::DETAIL
