/*
 * KiCad, GPL-3.0-or-later. Freerouting a11c0a42 room -> drill-page -> drill ->
 * layer expansion with exact-octagonal centre-space rooms. Drill pages retain
 * their source IntBox partition; general-convex free-drill cutouts remain open.
 */
#include "MazeSearchEngine45Degree.h"
#include "RoomSearchContext45Degree.h"
#include "MazeExpansionEngine.h"
#include "MazeListElement.h"
#include "RoomCostSpace.h"
#include "../AutorouterDebug.h"
#include "../drill/DrillPageArray.h"
#include "../expansion/TargetItemExpansionDoor.h"
#include "../path/FoundConnectionLocator45Degree.h"
#include "../board/model/items/Pin.h"
#include <deque>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

namespace KICAD_AUTOROUTER
{
namespace
{
std::optional<ROUTER_POINT> nearestInRoom45Multilayer(
        const ROOM_TERMINAL& aTerminal, ROUTER_POINT aPoint,
        const PLANAR::INT_OCTAGON& aRoom )
{
    if( aTerminal.connectionArea )
        return TARGET_ITEM_EXPANSION_DOOR::NearestIntegralPointInRoom(
                *aTerminal.connectionArea, aTerminal.areaInset, aPoint, aRoom );
    return TARGET_ITEM_EXPANSION_DOOR::NearestIntegralPointInRoom(
            aTerminal.start, aTerminal.end, aPoint, aRoom );
}


std::optional<FLOAT_POINT> connectionCentreInRoom45Multilayer(
        const ROOM_TERMINAL& aTerminal, const PLANAR::INT_OCTAGON& aRoom )
{
    if( aTerminal.connectionArea || aRoom.Dimension() < 0 )
        return std::nullopt;

    const long double x0 = aTerminal.start.x;
    const long double y0 = aTerminal.start.y;
    const long double dx = static_cast<long double>( aTerminal.end.x ) - x0;
    const long double dy = static_cast<long double>( aTerminal.end.y ) - y0;
    long double first = 0;
    long double last = 1;
    const auto constrain = [&]( long double aOrigin, long double aStep,
                                long double aMinimum, long double aMaximum )
    {
        if( aStep == 0 )
            return aOrigin >= aMinimum && aOrigin <= aMaximum;

        long double low = ( aMinimum - aOrigin ) / aStep;
        long double high = ( aMaximum - aOrigin ) / aStep;
        if( low > high )
            std::swap( low, high );
        first = std::max( first, low );
        last = std::min( last, high );
        return first <= last;
    };

    if( !constrain( x0, dx, aRoom.leftX, aRoom.rightX )
        || !constrain( y0, dy, aRoom.bottomY, aRoom.topY )
        || !constrain( x0 - y0, dx - dy,
                       aRoom.upperLeftDiagonalX,
                       aRoom.lowerRightDiagonalX )
        || !constrain( x0 + y0, dx + dy,
                       aRoom.lowerLeftDiagonalX,
                       aRoom.upperRightDiagonalX ) )
    {
        return std::nullopt;
    }

    // PolylineTrace.getTraceConnectionShape() is the segment centre-line.
    // Its intersection with a convex room has its centre of gravity at the
    // midpoint of this clipped interval.  It is deliberately not rounded:
    // the fractional point participates in the first A* queue tie-break.
    const long double middle = ( first + last ) / 2;
    return FLOAT_POINT{ static_cast<double>( x0 + middle * dx ),
                        static_cast<double>( y0 + middle * dy ) };
}


ROUTER_POINT sourceGridPoint45( ROUTER_POINT aPoint )
{
    return FLOAT_POINT{ static_cast<double>( aPoint.x ),
                        static_cast<double>( aPoint.y ) }
            .RoundToSourceGridYDown();
}


PLANAR::INT_OCTAGON traceTreeShape45( ROUTER_POINT aStart,
                                      ROUTER_POINT aEnd,
                                      std::int64_t aExpansion )
{
    const ROUTER_POINT start = sourceGridPoint45( aStart );
    const ROUTER_POINT end = sourceGridPoint45( aEnd );
    const std::int64_t left = std::min( start.x, end.x );
    const std::int64_t right = std::max( start.x, end.x );
    const std::int64_t bottom = std::min( start.y, end.y );
    const std::int64_t top = std::max( start.y, end.y );
    const std::int64_t upperLeft = std::min( start.x - start.y,
                                             end.x - end.y );
    const std::int64_t lowerRight = std::max( start.x - start.y,
                                              end.x - end.y );
    const std::int64_t lowerLeft = std::min( start.x + start.y,
                                             end.x + end.y );
    const std::int64_t upperRight = std::max( start.x + start.y,
                                              end.x + end.y );
    const auto unit = static_cast<std::int64_t>(
            FREEROUTING_COORDINATE_UNIT_IU );
    return PLANAR::INT_OCTAGON( left, bottom, right, top, upperLeft,
                                lowerRight, lowerLeft, upperRight )
            .NormalizeOnGrid( unit )
            .OffsetOnGrid( aExpansion, unit );
}
}

std::optional<ROOM_MULTILAYER_PATH> MAZE_SEARCH_ENGINE_45_DEGREE::FindMultilayerConnection(
        const std::vector<ROOM_LAYER>& layers, int net, double sectionOffset,
        const ROOM_VIA_SETTINGS& via, int maxExpanded, int& expanded,
        ROOM_SEARCH_METRICS& metrics, const ROUTER_CANCEL_CALLBACK& cancel,
        const ROUTER_SEARCH_PROGRESS_CALLBACK& progress,
        bool sourceTraceRooms,
        PERSISTENT_45_DEGREE_TREE_STATE* persistentTree )
{
    using DETAIL::ROOM;
    using DETAIL::ROOM_SEARCH_45_DEGREE;
    using PLANAR::INT_OCTAGON;
    if( layers.size() < 2 || ( via.transitionsEnabled && !via.canDrill )
        || !std::isfinite( via.normalCost ) || via.normalCost < 0
        || !std::isfinite( sectionOffset ) || sectionOffset <= 0 || via.pageWidth <= 0
        || INT_BOX::Dimension( via.bounds ) != 2 || maxExpanded <= expanded )
        return std::nullopt;
    std::set<int> layerIds;
    std::vector<DESTINATION_DISTANCE::EXPANSION_COST_FACTOR> traceCosts;
    std::vector<bool> layerActive;
    ROUTER_BOX costBounds = via.bounds;
    std::size_t targetCount = 0;
    for( const auto& layer : layers )
    {
        if( !layerIds.insert( layer.id ).second || INT_BOX::Dimension( layer.bounds ) != 2
            || !std::isfinite( layer.horizontalCost ) || layer.horizontalCost <= 0
            || !std::isfinite( layer.verticalCost ) || layer.verticalCost <= 0
            || !std::isfinite( layer.bendCost ) || layer.bendCost < 0 )
            return std::nullopt;
        for( const auto* terminals : { &layer.starts, &layer.targets } )
            for( const auto& terminal : *terminals )
            {
                costBounds = INT_BOX::Union( costBounds, {
                        std::min( terminal.start.x, terminal.end.x ), std::min( terminal.start.y, terminal.end.y ),
                        std::max( terminal.start.x, terminal.end.x ), std::max( terminal.start.y, terminal.end.y ) } );
            }
        costBounds = INT_BOX::Union( costBounds, layer.bounds );
        traceCosts.push_back( { layer.horizontalCost, layer.verticalCost } );
        layerActive.push_back( layer.active );
        if( layer.active )
        {
            targetCount += layer.targets.size();
        }
    }
    if( targetCount == 0 )
        return std::nullopt;
    const ROOM_COST_SPACE costSpace( costBounds );
    DESTINATION_DISTANCE destinationDistance( traceCosts, layerActive,
            costSpace.ToReferenceCost( via.normalCost ), costSpace.ToReferenceCost( 0.8 * via.normalCost ) );
    // Java joins every destination tree shape, including inactive layers;
    // layerActive affects the estimate, not destination-box collection.
    for( std::size_t layer = 0; layer < layers.size(); ++layer )
        for( const auto& target : layers[layer].targets )
        {
            const ROUTER_BOX targetBounds = INT_BOX::Dimension( target.treeBounds ) >= 0
                    ? target.treeBounds
                    : ROUTER_BOX{ std::min( target.start.x, target.end.x ),
                                  std::min( target.start.y, target.end.y ),
                                  std::max( target.start.x, target.end.x ),
                                  std::max( target.start.y, target.end.y ) };
            destinationDistance.Join( costSpace.ToReference( targetBounds ), layer );
        }
    // Bound page allocation before construction, not after a potentially huge
    // array has already been allocated. This is an explicit resource failure.
    const double columns = std::ceil( ( static_cast<double>( via.bounds.maxX ) - via.bounds.minX ) / via.pageWidth );
    const double rows = std::ceil( ( static_cast<double>( via.bounds.maxY ) - via.bounds.minY ) / via.pageWidth );
    if( columns * rows > maxExpanded )
        return std::nullopt;
    DRILL_PAGE_ARRAY pages( via.bounds, via.pageWidth,
                            FREEROUTING_COORDINATE_UNIT_IU );
    if( via.stopAtFirstDrill )
        pages.AddFanoutCandidates( via.fanoutCenter, via.fanoutMinDistance,
                                   via.fanoutMaxDistance );
    // Freerouting has one compensated ShapeSearchTree for the whole board,
    // not one tree per routing layer.  Opposite-layer leaves are rejected by
    // completeShape's layer test, but they still determine MinAreaTree
    // topology and therefore the second-child-first visitation order.
    struct TREE_RECORD
    {
        SHAPE_TREE_ENTRY entry;
        std::size_t layerOrdinal = 0;
    };
    std::vector<TREE_RECORD> treeRecords;
    for( std::size_t layerOrdinal = 0; layerOrdinal < layers.size(); ++layerOrdinal )
    {
        for( const SHAPE_TREE_ENTRY& entry : layers[layerOrdinal].obstacles )
            treeRecords.push_back( { entry, layerOrdinal } );
        for( const ROOM_RIPUP_OBSTACLE& obstacle : layers[layerOrdinal].ripupObstacles )
            treeRecords.push_back( { obstacle.shape, layerOrdinal } );
    }

    // A production KiCad snapshot has a geometric BoardOutline item, just as
    // the source RoutingBoard always does.  Lower-level tests and defensive
    // callers can provide only finite layer bounds, however.  The old native
    // room database supplied four bounds leaves unconditionally; deleting
    // them for source parity left an actually empty MinAreaTree in this case,
    // and ShapeSearchTree45Degree.completeShape() correctly returned no room.
    // Supply the bounds leaves only when the real outline object is absent.
    // Group all layers under one insertion identity, matching the source
    // BoardOutline's one SearchTreeObject with one edge shape per layer.
    const bool hasBoardOutline = std::any_of(
            treeRecords.begin(), treeRecords.end(),
            []( const TREE_RECORD& aRecord )
            { return aRecord.entry.boardOutline; } );
    if( !hasBoardOutline )
    {
        std::uint64_t fallbackOrder = 0;
        for( const TREE_RECORD& record : treeRecords )
        {
            if( record.entry.treeInsertionOrder
                != std::numeric_limits<std::uint64_t>::max() )
            {
                fallbackOrder = std::max(
                        fallbackOrder, record.entry.treeInsertionOrder + 1 );
            }
        }

        for( std::size_t layerOrdinal = 0;
             layerOrdinal < layers.size(); ++layerOrdinal )
        {
            const ROUTER_BOX& bounds = layers[layerOrdinal].bounds;
            const ROUTER_BOX borders[] = {
                { bounds.minX, bounds.minY, bounds.maxX, bounds.minY },
                { bounds.maxX, bounds.minY, bounds.maxX, bounds.maxY },
                { bounds.minX, bounds.maxY, bounds.maxX, bounds.maxY },
                { bounds.minX, bounds.minY, bounds.minX, bounds.maxY }
            };
            for( const ROUTER_BOX& border : borders )
            {
                SHAPE_TREE_ENTRY entry{ border, 0, 0,
                                        layers[layerOrdinal].id, 0,
                                        false, true };
                entry.boardOutline = true;
                entry.treeInsertionOrder = fallbackOrder;
                treeRecords.push_back( { std::move( entry ), layerOrdinal } );
            }
        }
    }
    std::stable_sort( treeRecords.begin(), treeRecords.end(),
                      []( const TREE_RECORD& aLeft, const TREE_RECORD& aRight )
                      {
                          if( aLeft.entry.treeInsertionOrder
                              != aRight.entry.treeInsertionOrder )
                          {
                              return aLeft.entry.treeInsertionOrder
                                     < aRight.entry.treeInsertionOrder;
                          }
                          // BoardOutline.calculateTreeShapes loops layers
                          // outside contour edges.  The same ordering is used
                          // for multilayer DrillItem shapes.
                          if( aLeft.layerOrdinal != aRight.layerOrdinal )
                              return aLeft.layerOrdinal < aRight.layerOrdinal;
                          return aLeft.entry.shapeIndex < aRight.entry.shapeIndex;
                      } );

    auto sharedTree = persistentTree ? persistentTree->tree
                                     : std::make_shared<MIN_AREA_TREE>();
    SHAPE_SEARCH_TREE_45_DEGREE sourceTree(
            via.bounds,
            static_cast<std::int64_t>( FREEROUTING_COORDINATE_UNIT_IU ),
            true, sharedTree );
    int nextTreeObjectId = 1;
    std::uint64_t previousOrder = std::numeric_limits<std::uint64_t>::max();
    int currentObjectId = 0;
    int currentShapeIndex = 0;
    std::vector<SHAPE_TREE_ENTRY> desiredPhysicalLeaves;
    desiredPhysicalLeaves.reserve( treeRecords.size() );
    for( const TREE_RECORD& record : treeRecords )
    {
        SHAPE_TREE_ENTRY entry = record.entry;
        if( entry.treeInsertionOrder != previousOrder )
        {
            previousOrder = entry.treeInsertionOrder;
            currentObjectId = nextTreeObjectId++;
            currentShapeIndex = 0;
        }
        entry.objectId = currentObjectId;
        // treeShapeCount/shapeLayer index all shapes of one SearchTreeObject,
        // including every copper layer of a Pin and every BoardOutline edge.
        entry.shapeIndex = currentShapeIndex++;
        entry.isRoom = false;
        desiredPhysicalLeaves.push_back( std::move( entry ) );
    }

    const auto samePhysicalLeaf = []( const SHAPE_TREE_ENTRY& aLeft,
                                      const SHAPE_TREE_ENTRY& aRight )
    {
        return aLeft.treeInsertionOrder == aRight.treeInsertionOrder
               && aLeft.shapeIndex == aRight.shapeIndex
               && aLeft.layer == aRight.layer && aLeft.net == aRight.net
               && aLeft.obstacle == aRight.obstacle
               && aLeft.boardOutline == aRight.boardOutline
               && aLeft.BoundingOctagon() == aRight.BoundingOctagon();
    };

    if( persistentTree )
    {
        // Keep unchanged leaves in place.  Removing and reinserting the whole
        // board would produce the same geometry but a different MinAreaTree,
        // which is precisely the deterministic-parity defect this state fixes.
        std::vector<bool> desiredMatched( desiredPhysicalLeaves.size(), false );
        std::vector<bool> existingMatched(
                persistentTree->physicalLeaves.size(), false );
        for( std::size_t desired = 0; desired < desiredPhysicalLeaves.size(); ++desired )
        {
            for( std::size_t existing = 0;
                 existing < persistentTree->physicalLeaves.size(); ++existing )
            {
                if( !existingMatched[existing]
                    && samePhysicalLeaf( desiredPhysicalLeaves[desired],
                                         persistentTree->physicalLeaves[existing].entry ) )
                {
                    desiredMatched[desired] = true;
                    existingMatched[existing] = true;
                    break;
                }
            }
        }

        for( std::size_t existing = 0;
             existing < persistentTree->physicalLeaves.size(); ++existing )
        {
            if( !existingMatched[existing] )
                sharedTree->Remove(
                        persistentTree->physicalLeaves[existing].handle );
        }

        // A new routed PolylineTrace must not jump directly from no leaves to
        // its final leaves. FoundConnectionInserter inserted several temporary
        // trace objects, combined them by transferring Leaf handles, and then
        // changed only the tree-shape range affected by pullTight. That history
        // determines later equal-area descent and second-child-first traversal.
        // Replay it only when the final item is genuinely new in this tree;
        // unchanged items retain their existing handles above.
        struct REPLAY_TRACE
        {
            std::vector<SHAPE_TREE_ENTRY> entries;
            std::vector<MIN_AREA_TREE::HANDLE> handles;
        };

        std::map<std::size_t, MIN_AREA_TREE::HANDLE> replayedHandles;
        std::map<std::uint64_t, std::vector<std::size_t>> replayGroups;
        for( std::size_t desired = 0; desired < desiredPhysicalLeaves.size(); ++desired )
        {
            if( desiredPhysicalLeaves[desired].traceInsertionSteps )
                replayGroups[desiredPhysicalLeaves[desired].treeInsertionOrder]
                        .push_back( desired );
        }

        const auto sameTreeShape = []( const SHAPE_TREE_ENTRY& aLeft,
                                       const SHAPE_TREE_ENTRY& aRight )
        {
            return aLeft.layer == aRight.layer
                   && aLeft.BoundingOctagon() == aRight.BoundingOctagon();
        };
        const auto eraseTrace = [&]( REPLAY_TRACE& aTrace )
        {
            for( MIN_AREA_TREE::HANDLE handle : aTrace.handles )
                sharedTree->Remove( handle );
            aTrace.entries.clear();
            aTrace.handles.clear();
        };

        for( const auto& [treeOrder, desiredIndices] : replayGroups )
        {
            if( desiredIndices.empty()
                || std::any_of( desiredIndices.begin(), desiredIndices.end(),
                                [&]( std::size_t aIndex )
                                { return desiredMatched[aIndex]; } ) )
            {
                continue;
            }

            std::vector<SHAPE_TREE_ENTRY> finalEntries;
            finalEntries.reserve( desiredIndices.size() );
            for( std::size_t index : desiredIndices )
                finalEntries.push_back( desiredPhysicalLeaves[index] );
            std::sort( finalEntries.begin(), finalEntries.end(),
                       []( const auto& aLeft, const auto& aRight )
                       { return aLeft.shapeIndex < aRight.shapeIndex; } );

            const auto journal = finalEntries.front().traceInsertionSteps;
            const int finalLayer = finalEntries.front().layer;
            const bool oneTraceLayer = journal && !journal->empty()
                    && std::all_of(
                            finalEntries.begin(), finalEntries.end(),
                            [&]( const SHAPE_TREE_ENTRY& aEntry )
                            {
                                return aEntry.layer == finalLayer
                                       && aEntry.traceTreeExpansion >= 0;
                            } )
                    && std::all_of(
                            journal->begin(), journal->end(),
                            [&]( const ROUTING_TRACE_INSERTION_STEP& aStep )
                            {
                                const auto sameLayer = [&](
                                        const ROUTING_TRACE_GEOMETRY_SNAPSHOT& aGeometry )
                                {
                                    return aGeometry.nodes.size() >= 2
                                           && ( aGeometry.edgeStyles.empty()
                                                || aGeometry.edgeStyles.size() + 1
                                                           == aGeometry.nodes.size() )
                                           && std::all_of(
                                            aGeometry.nodes.begin(),
                                            aGeometry.nodes.end(),
                                            [&]( const ROUTER_NODE& aNode )
                                            { return aNode.layer == finalLayer; } );
                                };
                                return sameLayer( aStep.insertedSpan )
                                       && sameLayer( aStep.combinedBeforeTighten )
                                       && sameLayer( aStep.tightenedResult );
                            } );
            if( !oneTraceLayer )
            {
                autorouterDecisionLog(
                        "TRACE_TREE_REPLAY_SKIPPED",
                        { { "tree_order", std::to_string( treeOrder ) },
                          { "final_layer", std::to_string( finalLayer ) },
                          { "final_leaves", std::to_string(
                                                      finalEntries.size() ) },
                          { "steps", journal
                                               ? std::to_string( journal->size() )
                                               : "0" } } );
                continue;
            }

            const std::int64_t fallbackExpansion =
                    finalEntries.front().traceTreeExpansion;
            const auto geometryEntries = [&](
                    const ROUTING_TRACE_GEOMETRY_SNAPSHOT& aGeometry,
                    int aObjectId )
            {
                ROUTING_TRACE_GEOMETRY_SNAPSHOT geometry = aGeometry;
                if( geometry.edgeStyles.empty() && geometry.nodes.size() > 1 )
                {
                    geometry.edgeStyles.resize( geometry.nodes.size() - 1 );
                }

                // Polyline's constructor removes a redundant collinear line
                // before calculateTreeShapes. Mirror only source-safe cases:
                // same layer, same edge style and continued direction.
                bool changed = true;
                while( changed && geometry.nodes.size() > 2 )
                {
                    changed = false;
                    for( std::size_t node = 1; node + 1 < geometry.nodes.size(); ++node )
                    {
                        const ROUTER_NODE& first = geometry.nodes[node - 1];
                        const ROUTER_NODE& middle = geometry.nodes[node];
                        const ROUTER_NODE& last = geometry.nodes[node + 1];
                        using WIDE = __int128_t;
                        const WIDE ax = WIDE( middle.point.x ) - first.point.x;
                        const WIDE ay = WIDE( middle.point.y ) - first.point.y;
                        const WIDE bx = WIDE( last.point.x ) - middle.point.x;
                        const WIDE by = WIDE( last.point.y ) - middle.point.y;
                        if( first.layer == middle.layer && middle.layer == last.layer
                            && geometry.edgeStyles[node - 1]
                                       == geometry.edgeStyles[node]
                            && ax * by == ay * bx && ax * bx + ay * by >= 0 )
                        {
                            geometry.nodes.erase(
                                    geometry.nodes.begin()
                                            + static_cast<std::ptrdiff_t>( node ) );
                            geometry.edgeStyles.erase(
                                    geometry.edgeStyles.begin()
                                            + static_cast<std::ptrdiff_t>( node ) );
                            changed = true;
                            break;
                        }
                    }
                }

                std::vector<SHAPE_TREE_ENTRY> entries;
                for( std::size_t edge = 0; edge + 1 < geometry.nodes.size(); ++edge )
                {
                    const ROUTER_NODE& from = geometry.nodes[edge];
                    const ROUTER_NODE& to = geometry.nodes[edge + 1];
                    if( from.layer != to.layer || from.point == to.point )
                        continue;

                    std::int64_t expansion = fallbackExpansion;
                    if( edge < finalEntries.size()
                        && finalEntries[edge].traceTreeExpansion >= 0 )
                    {
                        expansion = finalEntries[edge].traceTreeExpansion;
                    }
                    else if( !geometry.edgeStyles.empty() )
                    {
                        const auto style = std::find_if(
                                finalEntries.begin(), finalEntries.end(),
                                [&]( const SHAPE_TREE_ENTRY& aEntry )
                                {
                                    const std::size_t finalEdge =
                                            static_cast<std::size_t>(
                                                    std::max( 0, aEntry.shapeIndex ) );
                                    const auto& finalGeometry =
                                            journal->back().tightenedResult;
                                    return finalEdge < finalGeometry.edgeStyles.size()
                                           && finalGeometry.edgeStyles[finalEdge]
                                                      == geometry.edgeStyles[edge];
                                } );
                        if( style != finalEntries.end()
                            && style->traceTreeExpansion >= 0 )
                        {
                            expansion = style->traceTreeExpansion;
                        }
                    }

                    SHAPE_TREE_ENTRY entry = finalEntries.front();
                    entry.objectId = aObjectId;
                    entry.shapeIndex = static_cast<int>( entries.size() );
                    entry.layer = from.layer;
                    entry.isRoom = false;
                    entry.boardOutline = false;
                    entry.traceInsertionSteps.reset();
                    entry.traceTreeExpansion = expansion;
                    entry.simplex.reset();
                    entry.octagon = traceTreeShape45( from.point, to.point,
                                                       expansion );
                    entry.shape = entry.octagon->BoundingBox();
                    entries.push_back( std::move( entry ) );
                }
                return entries;
            };

            const auto insertEntries = [&]( std::vector<SHAPE_TREE_ENTRY> aEntries )
            {
                REPLAY_TRACE result;
                result.entries = std::move( aEntries );
                result.handles.reserve( result.entries.size() );
                for( const SHAPE_TREE_ENTRY& entry : result.entries )
                    result.handles.push_back( sourceTree.Insert( entry ) );
                return result;
            };
            const auto sameTraceShapes = [&]( const REPLAY_TRACE& aTrace,
                                              const std::vector<SHAPE_TREE_ENTRY>& aEntries )
            {
                return aTrace.entries.size() == aEntries.size()
                       && std::equal( aTrace.entries.begin(), aTrace.entries.end(),
                                      aEntries.begin(), sameTreeShape );
            };

            const auto pointOnSegment = []( const ROUTER_POINT& aPoint,
                                            const ROUTER_POINT& aStart,
                                            const ROUTER_POINT& aEnd )
            {
                using WIDE = __int128_t;
                const WIDE dx = WIDE( aEnd.x ) - aStart.x;
                const WIDE dy = WIDE( aEnd.y ) - aStart.y;
                const WIDE px = WIDE( aPoint.x ) - aStart.x;
                const WIDE py = WIDE( aPoint.y ) - aStart.y;
                return dx * py == dy * px
                       && aPoint.x >= std::min( aStart.x, aEnd.x )
                       && aPoint.x <= std::max( aStart.x, aEnd.x )
                       && aPoint.y >= std::min( aStart.y, aEnd.y )
                       && aPoint.y <= std::max( aStart.y, aEnd.y );
            };

            const auto splitGeometryAt = [&](
                    ROUTING_TRACE_GEOMETRY_SNAPSHOT aGeometry,
                    const ROUTER_POINT& aPoint )
                    -> std::optional<std::pair<
                            ROUTING_TRACE_GEOMETRY_SNAPSHOT,
                            ROUTING_TRACE_GEOMETRY_SNAPSHOT>>
            {
                if( aGeometry.nodes.size() < 2 )
                    return std::nullopt;
                if( aGeometry.edgeStyles.empty() )
                    aGeometry.edgeStyles.resize( aGeometry.nodes.size() - 1 );
                if( aGeometry.edgeStyles.size() + 1 != aGeometry.nodes.size() )
                    return std::nullopt;

                std::size_t splitNode = aGeometry.nodes.size();
                for( std::size_t node = 0; node < aGeometry.nodes.size(); ++node )
                {
                    if( aGeometry.nodes[node].point == aPoint )
                    {
                        splitNode = node;
                        break;
                    }
                }

                if( splitNode == aGeometry.nodes.size() )
                {
                    for( std::size_t edge = 0; edge + 1 < aGeometry.nodes.size(); ++edge )
                    {
                        if( aGeometry.nodes[edge].layer != aGeometry.nodes[edge + 1].layer
                            || !pointOnSegment( aPoint, aGeometry.nodes[edge].point,
                                                aGeometry.nodes[edge + 1].point ) )
                        {
                            continue;
                        }
                        aGeometry.nodes.insert(
                                aGeometry.nodes.begin()
                                        + static_cast<std::ptrdiff_t>( edge + 1 ),
                                ROUTER_NODE{ aPoint, aGeometry.nodes[edge].layer } );
                        aGeometry.edgeStyles.insert(
                                aGeometry.edgeStyles.begin()
                                        + static_cast<std::ptrdiff_t>( edge + 1 ),
                                aGeometry.edgeStyles[edge] );
                        splitNode = edge + 1;
                        break;
                    }
                }

                if( splitNode == 0 || splitNode + 1 >= aGeometry.nodes.size() )
                    return std::nullopt;

                ROUTING_TRACE_GEOMETRY_SNAPSHOT first;
                first.nodes.assign( aGeometry.nodes.begin(),
                                    aGeometry.nodes.begin()
                                            + static_cast<std::ptrdiff_t>( splitNode + 1 ) );
                first.edgeStyles.assign(
                        aGeometry.edgeStyles.begin(),
                        aGeometry.edgeStyles.begin()
                                + static_cast<std::ptrdiff_t>( splitNode ) );

                ROUTING_TRACE_GEOMETRY_SNAPSHOT second;
                second.nodes.assign(
                        aGeometry.nodes.begin()
                                + static_cast<std::ptrdiff_t>( splitNode ),
                        aGeometry.nodes.end() );
                second.edgeStyles.assign(
                        aGeometry.edgeStyles.begin()
                                + static_cast<std::ptrdiff_t>( splitNode ),
                        aGeometry.edgeStyles.end() );
                return std::pair{ std::move( first ), std::move( second ) };
            };

            REPLAY_TRACE aggregate;
            ROUTING_TRACE_GEOMETRY_SNAPSHOT aggregateGeometry;
            for( std::size_t stepIndex = 0; stepIndex < journal->size(); ++stepIndex )
            {
                const ROUTING_TRACE_INSERTION_STEP& step = ( *journal )[stepIndex];
                const int insertedObjectId = nextTreeObjectId++;
                REPLAY_TRACE inserted = insertEntries(
                        geometryEntries( step.insertedSpan, insertedObjectId ) );
                std::vector<SHAPE_TREE_ENTRY> combinedEntries = geometryEntries(
                        step.combinedBeforeTighten, insertedObjectId );

                if( aggregate.entries.empty() )
                {
                    aggregate = std::move( inserted );
                    if( !sameTraceShapes( aggregate, combinedEntries ) )
                    {
                        eraseTrace( aggregate );
                        aggregate = insertEntries( std::move( combinedEntries ) );
                    }
                }
                else
                {
                    // A rewound forced insertion can start before the existing
                    // trace's current endpoint.  In that case
                    // insertForcedTracePolyline() does not pick the existing
                    // trace at the new span's first corner.  It inserts a
                    // second PolylineTrace, and normalize() then splits both
                    // traces at the common collinear interval before removing
                    // the duplicate piece.  Collapsing the two items directly
                    // to combinedBeforeTighten creates the same copper but a
                    // different MinAreaTree.  Reproduce the source split,
                    // duplicate-removal, pull-tight and final stub-combine
                    // lifecycle for the exact tail-overlap case emitted by
                    // FoundConnectionInserter's first rewind.
                    bool replayedNormalizedOverlap = false;
                    if( step.normalizedSplitNode
                        && stepIndex + 1 == journal->size()
                        && aggregateGeometry.nodes.size() >= 2
                        && step.insertedSpan.nodes.size() >= 2 )
                    {
                        const ROUTER_NODE& oldOverlapStart =
                                aggregateGeometry.nodes[aggregateGeometry.nodes.size() - 2];
                        const ROUTER_NODE& oldOverlapEnd = aggregateGeometry.nodes.back();
                        const ROUTER_NODE& insertedStart = step.insertedSpan.nodes.front();
                        const ROUTER_NODE& insertedOverlapEnd = step.insertedSpan.nodes[1];
                        const bool exactTailOverlap =
                                oldOverlapStart.layer == oldOverlapEnd.layer
                                && insertedStart.layer == insertedOverlapEnd.layer
                                && oldOverlapStart.layer == insertedStart.layer
                                && oldOverlapEnd == insertedOverlapEnd
                                && insertedStart.point != oldOverlapStart.point
                                && pointOnSegment( oldOverlapStart.point,
                                                   insertedStart.point,
                                                   insertedOverlapEnd.point );

                        const auto oldPieces = exactTailOverlap
                                ? splitGeometryAt( aggregateGeometry,
                                                   oldOverlapStart.point )
                                : std::nullopt;
                        const auto insertedAtStart = exactTailOverlap
                                ? splitGeometryAt( step.insertedSpan,
                                                   oldOverlapStart.point )
                                : std::nullopt;
                        const auto insertedAtEnd = insertedAtStart
                                ? splitGeometryAt( insertedAtStart->second,
                                                   oldOverlapEnd.point )
                                : std::nullopt;
                        const auto finalAtStart = insertedAtEnd
                                ? splitGeometryAt( step.tightenedResult,
                                                   oldOverlapStart.point )
                                : std::nullopt;

                        if( oldPieces && insertedAtStart && insertedAtEnd
                            && finalAtStart )
                        {
                            // split old aggregate -> retained prefix + old
                            // overlap; ShapeSearchTree.divideEntries removes
                            // and reinserts the complete source item.
                            eraseTrace( aggregate );
                            const int oldPrefixId = nextTreeObjectId++;
                            REPLAY_TRACE oldPrefix = insertEntries(
                                    geometryEntries( oldPieces->first,
                                                     oldPrefixId ) );
                            const int oldOverlapId = nextTreeObjectId++;
                            REPLAY_TRACE oldOverlap = insertEntries(
                                    geometryEntries( oldPieces->second,
                                                     oldOverlapId ) );

                            // split the just-inserted trace first at the start
                            // and then at the end of the overlap.
                            eraseTrace( inserted );
                            const int insertedPrefixId = nextTreeObjectId++;
                            REPLAY_TRACE insertedPrefix = insertEntries(
                                    geometryEntries( insertedAtStart->first,
                                                     insertedPrefixId ) );
                            const int insertedTailId = nextTreeObjectId++;
                            REPLAY_TRACE insertedTail = insertEntries(
                                    geometryEntries( insertedAtStart->second,
                                                     insertedTailId ) );
                            eraseTrace( insertedTail );
                            const int duplicateOverlapId = nextTreeObjectId++;
                            REPLAY_TRACE duplicateOverlap = insertEntries(
                                    geometryEntries( insertedAtEnd->first,
                                                     duplicateOverlapId ) );
                            const int suffixId = nextTreeObjectId++;
                            REPLAY_TRACE suffix = insertEntries(
                                    geometryEntries( insertedAtEnd->second,
                                                     suffixId ) );

                            // normalize removes one copy of the overlap and
                            // combines the surviving overlap item with the
                            // following suffix.  mergeEntriesAtEnd replaces
                            // the overlap and first suffix shapes, transferring
                            // every later suffix Leaf without reinsertion.
                            if( !oldOverlap.handles.empty()
                                && !duplicateOverlap.handles.empty()
                                && !suffix.handles.empty() )
                            {
                                sharedTree->Remove( oldOverlap.handles.front() );
                                sharedTree->Remove( duplicateOverlap.handles.front() );
                                sharedTree->Remove( suffix.handles.front() );

                                const auto selectedEntries = geometryEntries(
                                        insertedAtStart->second,
                                        duplicateOverlapId );
                                const std::size_t transferredSuffix =
                                        suffix.handles.size() - 1;
                                const std::size_t linkCount =
                                        selectedEntries.size() - transferredSuffix;
                                REPLAY_TRACE selected;
                                selected.entries = selectedEntries;
                                selected.handles.reserve( selectedEntries.size() );
                                for( std::size_t index = 0; index < linkCount; ++index )
                                {
                                    selected.handles.push_back( sourceTree.Insert(
                                            selectedEntries[index] ) );
                                }
                                for( std::size_t index = 0;
                                     index < transferredSuffix; ++index )
                                {
                                    const std::size_t oldIndex = index + 1;
                                    const std::size_t newIndex = linkCount + index;
                                    sharedTree->UpdateEntry(
                                            suffix.handles[oldIndex],
                                            selectedEntries[newIndex] );
                                    selected.handles.push_back(
                                            suffix.handles[oldIndex] );
                                }

                                // pullTight changes the selected suffix item.
                                // For the normalized-overlap path the keep
                                // point is its far endpoint, so every old shape
                                // is replaced in source order.
                                eraseTrace( selected );
                                REPLAY_TRACE tightened = insertEntries(
                                        geometryEntries( finalAtStart->second,
                                                         duplicateOverlapId ) );

                                // FoundConnectionInserter removes the leading
                                // split stub after all spans have been inserted.
                                eraseTrace( insertedPrefix );

                                // normalizeTraces then combines the retained
                                // old prefix with the tightened suffix.  Keep
                                // the non-boundary leaves in place and replace
                                // exactly the two link shapes, matching
                                // mergeEntriesInFront.
                                if( !oldPrefix.handles.empty()
                                    && !tightened.handles.empty() )
                                {
                                    sharedTree->Remove( oldPrefix.handles.back() );
                                    sharedTree->Remove( tightened.handles.front() );
                                    const auto finalStepEntries = geometryEntries(
                                            step.tightenedResult,
                                            duplicateOverlapId );
                                    const std::size_t retainedPrefix =
                                            oldPrefix.handles.size() - 1;
                                    const std::size_t retainedSuffix =
                                            tightened.handles.size() - 1;
                                    const std::size_t finalLinkCount =
                                            finalStepEntries.size()
                                            - retainedPrefix - retainedSuffix;
                                    REPLAY_TRACE finalized;
                                    finalized.entries = finalStepEntries;
                                    finalized.handles.reserve(
                                            finalStepEntries.size() );
                                    for( std::size_t index = 0;
                                         index < retainedPrefix; ++index )
                                    {
                                        sharedTree->UpdateEntry(
                                                oldPrefix.handles[index],
                                                finalStepEntries[index] );
                                        finalized.handles.push_back(
                                                oldPrefix.handles[index] );
                                    }
                                    for( std::size_t index = 0;
                                         index < finalLinkCount; ++index )
                                    {
                                        finalized.handles.push_back(
                                                sourceTree.Insert(
                                                        finalStepEntries[
                                                                retainedPrefix
                                                                + index] ) );
                                    }
                                    for( std::size_t index = 1;
                                         index < tightened.handles.size(); ++index )
                                    {
                                        const std::size_t targetIndex =
                                                retainedPrefix + finalLinkCount
                                                + index - 1;
                                        sharedTree->UpdateEntry(
                                                tightened.handles[index],
                                                finalStepEntries[targetIndex] );
                                        finalized.handles.push_back(
                                                tightened.handles[index] );
                                    }
                                    aggregate = std::move( finalized );
                                    replayedNormalizedOverlap = true;
                                    autorouterDecisionLog(
                                            "TRACE_TREE_NORMALIZED_OVERLAP_REPLAY",
                                            { { "tree_order",
                                                std::to_string( treeOrder ) },
                                              { "old_prefix_leaves",
                                                std::to_string(
                                                        oldPrefix.entries.size() ) },
                                              { "inserted_leaves",
                                                std::to_string(
                                                        insertedAtStart->first.nodes.size()
                                                        - 1 ) },
                                              { "final_leaves",
                                                std::to_string(
                                                        aggregate.handles.size() ) } } );
                                }
                            }
                        }
                    }

                    if( replayedNormalizedOverlap )
                    {
                        aggregateGeometry = step.tightenedResult;
                        continue;
                    }

                    const std::size_t prefix = aggregate.entries.size() - 1;
                    const std::size_t suffix = inserted.entries.size() - 1;
                    const bool canTransfer = !step.normalizedSplitNode
                            && combinedEntries.size() >= prefix + suffix + 1
                            && std::equal(
                                    aggregate.entries.begin(),
                                    aggregate.entries.begin()
                                            + static_cast<std::ptrdiff_t>( prefix ),
                                    combinedEntries.begin(), sameTreeShape )
                            && std::equal(
                                    inserted.entries.begin() + 1,
                                    inserted.entries.end(),
                                    combinedEntries.end()
                                            - static_cast<std::ptrdiff_t>( suffix ),
                                    sameTreeShape );
                    if( canTransfer )
                    {
                        sharedTree->Remove( aggregate.handles.back() );
                        sharedTree->Remove( inserted.handles.front() );
                        const std::size_t changedCount =
                                combinedEntries.size() - prefix - suffix;
                        REPLAY_TRACE combined;
                        combined.entries = combinedEntries;
                        combined.handles.reserve( combinedEntries.size() );
                        for( std::size_t index = 0; index < prefix; ++index )
                        {
                            sharedTree->UpdateEntry( aggregate.handles[index],
                                                     combinedEntries[index] );
                            combined.handles.push_back( aggregate.handles[index] );
                        }
                        for( std::size_t index = 0; index < changedCount; ++index )
                        {
                            combined.handles.push_back( sourceTree.Insert(
                                    combinedEntries[prefix + index] ) );
                        }
                        for( std::size_t index = 0; index < suffix; ++index )
                        {
                            const std::size_t sourceIndex = index + 1;
                            const std::size_t targetIndex =
                                    prefix + changedCount + index;
                            sharedTree->UpdateEntry( inserted.handles[sourceIndex],
                                                     combinedEntries[targetIndex] );
                            combined.handles.push_back(
                                    inserted.handles[sourceIndex] );
                        }
                        aggregate = std::move( combined );
                    }
                    else
                    {
                        eraseTrace( aggregate );
                        eraseTrace( inserted );
                        aggregate = insertEntries( std::move( combinedEntries ) );
                    }
                }

                std::vector<SHAPE_TREE_ENTRY> tightenedEntries = geometryEntries(
                        step.tightenedResult, insertedObjectId );
                if( sameTraceShapes( aggregate, tightenedEntries ) )
                {
                    for( std::size_t index = 0; index < aggregate.handles.size(); ++index )
                        sharedTree->UpdateEntry( aggregate.handles[index],
                                                 tightenedEntries[index] );
                    aggregate.entries = std::move( tightenedEntries );
                    aggregateGeometry = step.tightenedResult;
                    continue;
                }

                std::size_t keepStart = 0;
                std::size_t keepEnd = 0;
                if( !step.normalizedSplitNode )
                {
                    while( keepStart < aggregate.entries.size()
                           && keepStart < tightenedEntries.size()
                           && sameTreeShape( aggregate.entries[keepStart],
                                             tightenedEntries[keepStart] ) )
                    {
                        ++keepStart;
                    }
                    while( keepEnd + keepStart < aggregate.entries.size()
                           && keepEnd + keepStart < tightenedEntries.size()
                           && sameTreeShape(
                                   aggregate.entries[aggregate.entries.size()
                                                             - 1 - keepEnd],
                                   tightenedEntries[tightenedEntries.size()
                                                            - 1 - keepEnd] ) )
                    {
                        ++keepEnd;
                    }
                }

                for( std::size_t index = keepStart;
                     index + keepEnd < aggregate.handles.size(); ++index )
                {
                    sharedTree->Remove( aggregate.handles[index] );
                }
                REPLAY_TRACE tightened;
                tightened.entries = tightenedEntries;
                tightened.handles.reserve( tightenedEntries.size() );
                for( std::size_t index = 0; index < keepStart; ++index )
                {
                    sharedTree->UpdateEntry( aggregate.handles[index],
                                             tightenedEntries[index] );
                    tightened.handles.push_back( aggregate.handles[index] );
                }
                for( std::size_t index = keepStart;
                     index + keepEnd < tightenedEntries.size(); ++index )
                {
                    tightened.handles.push_back( sourceTree.Insert(
                            tightenedEntries[index] ) );
                }
                for( std::size_t index = keepEnd; index > 0; --index )
                {
                    const std::size_t oldIndex =
                            aggregate.handles.size() - index;
                    const std::size_t newIndex = tightenedEntries.size() - index;
                    sharedTree->UpdateEntry( aggregate.handles[oldIndex],
                                             tightenedEntries[newIndex] );
                    tightened.handles.push_back( aggregate.handles[oldIndex] );
                }
                aggregate = std::move( tightened );
                aggregateGeometry = step.tightenedResult;
            }

            const bool finalMatches = aggregate.entries.size()
                                              == finalEntries.size()
                    && std::equal( aggregate.entries.begin(),
                                   aggregate.entries.end(), finalEntries.begin(),
                                   sameTreeShape );
            if( !finalMatches )
            {
                eraseTrace( aggregate );
                aggregate = insertEntries( finalEntries );
            }
            for( std::size_t index = 0; index < finalEntries.size(); ++index )
            {
                sharedTree->UpdateEntry( aggregate.handles[index],
                                         finalEntries[index] );
                const auto desired = std::find_if(
                        desiredIndices.begin(), desiredIndices.end(),
                        [&]( std::size_t aIndex )
                        {
                            return desiredPhysicalLeaves[aIndex].shapeIndex
                                   == finalEntries[index].shapeIndex;
                        } );
                if( desired != desiredIndices.end() )
                    replayedHandles[*desired] = aggregate.handles[index];
            }

            if( autorouterDebugEnabled() )
            {
                std::ostringstream log;
                log << "TRACE_TREE_REPLAY order=" << treeOrder
                    << " steps=" << journal->size()
                    << " final_leaves=" << aggregate.handles.size();
                autorouterDebugLog( log.str() );
            }
            autorouterDecisionLog(
                    "TRACE_TREE_REPLAY",
                    { { "tree_order", std::to_string( treeOrder ) },
                      { "steps", std::to_string( journal->size() ) },
                      { "final_leaves", std::to_string(
                                                  aggregate.handles.size() ) } } );
        }

        std::vector<PERSISTENT_45_DEGREE_TREE_STATE::PHYSICAL_LEAF> synchronized;
        synchronized.reserve( desiredPhysicalLeaves.size() );
        for( std::size_t desired = 0; desired < desiredPhysicalLeaves.size(); ++desired )
        {
            if( desiredMatched[desired] )
            {
                const auto existing = std::find_if(
                        persistentTree->physicalLeaves.begin(),
                        persistentTree->physicalLeaves.end(),
                        [&]( const auto& aLeaf )
                        {
                            return samePhysicalLeaf( desiredPhysicalLeaves[desired],
                                                     aLeaf.entry );
                        } );
                synchronized.push_back( *existing );
            }
            else
            {
                const auto replayed = replayedHandles.find( desired );
                const MIN_AREA_TREE::HANDLE handle = replayed != replayedHandles.end()
                        ? replayed->second
                        : sourceTree.Insert( desiredPhysicalLeaves[desired] );
                synchronized.push_back( { desiredPhysicalLeaves[desired], handle } );
            }
        }
        persistentTree->physicalLeaves = std::move( synchronized );
    }
    else
    {
        for( const SHAPE_TREE_ENTRY& entry : desiredPhysicalLeaves )
            sourceTree.Insert( entry );
    }

    std::vector<std::unique_ptr<ROOM_SEARCH_45_DEGREE>> spaces;
    std::vector<MIN_AREA_TREE::HANDLE> temporaryRoomHandles;
    struct TEMPORARY_ROOM_CLEANUP
    {
        std::shared_ptr<MIN_AREA_TREE> tree;
        std::vector<MIN_AREA_TREE::HANDLE>& handles;
        ~TEMPORARY_ROOM_CLEANUP()
        {
            // AutorouteEngine.clear() removes CompleteFreeSpaceExpansionRooms
            // in creation order after every non-maintained connection.  The
            // removals are semantically visible because they leave the
            // surviving MinAreaTree with its mutation-derived topology.
            for( MIN_AREA_TREE::HANDLE handle : handles )
                tree->Remove( handle );
        }
    } roomCleanup{ sharedTree, temporaryRoomHandles };
    int nextRoomId = nextTreeObjectId;
    for( const auto& layer : layers )
        spaces.push_back( std::make_unique<ROOM_SEARCH_45_DEGREE>( layer.bounds, layer.obstacles,
                layer.id, net, sectionOffset, maxExpanded, expanded, metrics, cancel, progress,
                layer.ripupObstacles, &nextRoomId, sharedTree, false,
                &temporaryRoomHandles ) );

    // `active` controls whether a layer can carry a trace-room state; it
    // must not make that physical copper layer transparent to a manufactured
    // through via.  Production callers provide `via.obstacles` as the exact
    // all-layer drill preflight set.  Retain any inactive ROOM_LAYER entries
    // as well so this lower-level frontier cannot accidentally accept a via
    // through an inactive inner-layer plane when the caller's broad drill
    // list is incomplete.  The exact canDrill callback is still the final
    // authority for non-rectangular geometry.
    std::vector<SHAPE_TREE_ENTRY> drillObstacles = via.obstacles;
    const auto appendDrillObstacle = [&]( const SHAPE_TREE_ENTRY& aEntry )
    {
        const auto duplicate = [&]( const SHAPE_TREE_ENTRY& aKnown )
        {
            return aKnown.shape.minX == aEntry.shape.minX
                   && aKnown.shape.minY == aEntry.shape.minY
                   && aKnown.shape.maxX == aEntry.shape.maxX
                   && aKnown.shape.maxY == aEntry.shape.maxY
                   && aKnown.BoundingOctagon() == aEntry.BoundingOctagon()
                   && aKnown.objectId == aEntry.objectId
                   && aKnown.shapeIndex == aEntry.shapeIndex && aKnown.layer == aEntry.layer
                   && aKnown.net == aEntry.net && aKnown.isRoom == aEntry.isRoom
                   && aKnown.obstacle == aEntry.obstacle;
        };

        if( std::none_of( drillObstacles.begin(), drillObstacles.end(), duplicate ) )
            drillObstacles.push_back( aEntry );
    };
    for( const ROOM_LAYER& layer : layers )
        if( !layer.active )
            for( const SHAPE_TREE_ENTRY& obstacle : layer.obstacles )
                appendDrillObstacle( obstacle );

    auto stopped = [&]() { return expanded >= maxExpanded || ( cancel && cancel() ); };
    auto step = [&]() { return spaces.front()->step(); };
    auto remaining = [&]( FLOAT_POINT from, std::size_t fromLayer )
    {
        ++metrics.destinationQueries;
        return costSpace.ToNativeCost( destinationDistance.Calculate(
                costSpace.ToReference( from ), fromLayer ) );
    };
    auto roomAt = [&]( std::size_t layer, ROUTER_POINT point ) -> ROOM*
    {
        auto& space = *spaces[layer];
        const INT_OCTAGON seed = INT_OCTAGON::FromBox(
                { point.x, point.y, point.x, point.y } );
        for( const auto& item : space.tree.Overlaps( seed ) )
            if( item.isRoom && item.layer == layers[layer].id
                && item.BoundingOctagon().Contains( point ) )
                return space.roomForId( item.objectId, "roomAt" );
        auto rooms = space.complete( space.incomplete( {
                INT_OCTAGON::FromBox( layers[layer].bounds ), layers[layer].id, seed } ) );

        // ExpansionDrill.calculateExpansionRooms requires exactly one room
        // containing the drill location.  ShapeSearchTree45Degree may divide
        // an otherwise empty board-sized room into multiple sections; only
        // the section whose intersected contained-shape includes this point
        // is the drill room.  Counting every completed sibling made all
        // candidates in that page look ambiguous and forced fanout back to
        // the legacy grid search.
        ROOM* containing = nullptr;
        for( ROOM* room : rooms )
        {
            if( room->shape->GetOctagon().Contains( point ) )
            {
                if( containing )
                    return nullptr;
                containing = room;
            }
        }
        return containing;
    };
    enum class KIND
    {
        ROOM_ENTRY,
        PAGE,
        DRILL_ENTER,
        DRILL_EXIT,
        OWN_ITEM_TARGET,
        TARGET,
        FANOUT_TARGET
    };
    struct STATE
    {
        KIND kind = KIND::ROOM_ENTRY;
        ROOM* room = nullptr;
        std::size_t layer = 0;
        EXPANSION_DOOR* door = nullptr;
        DRILL_PAGE* page = nullptr;
        EXPANSION_DRILL* drill = nullptr;
        std::size_t section = 0;
        FLOAT_LINE entry;
        double g = 0;
        double f = 0;
        std::size_t parent = std::numeric_limits<std::size_t>::max();
        std::size_t owner = 0;
        std::size_t targetOwner = 0;
        int itemId = 0;
        int ripupCost = 0;
        std::optional<std::size_t> rippedGroup;
        bool roomRipped = false;
        MAZE_ADJUSTMENT adjustment = MAZE_ADJUSTMENT::NONE;
        bool alreadyChecked = false;
        std::optional<ROUTING_EDGE_STYLE> viaStyle;
        const ROOM_TERMINAL* backtrackPin = nullptr;
    };
    constexpr auto NONE = std::numeric_limits<std::size_t>::max();
    std::deque<STATE> states;
    std::map<decltype( MAZE_LIST_ELEMENT{}.SortKey() ), std::size_t> open;
    std::set<std::pair<EXPANSION_DOOR*, std::size_t>> occupied;
    // TargetItemExpansionDoor instances are one-sided frontier objects, not
    // ordinary room-neighbour doors.  Keep them alive for every queued start
    // state and its complete parent chain.
    std::vector<std::unique_ptr<TARGET_ITEM_EXPANSION_DOOR>> startDoors;
    bool allocationLimit = false;
    auto push = [&]( const STATE& state )
    {
        // MazeSearchEngine's fanout frontier is deliberately local.  Its
        // TreeSet.add override rejects every element whose next room is on
        // the source layer once that element's shape-entry midpoint leaves
        // the configured escape envelope.  ExpansionDrill elements are also
        // rejected while they are inside the minimum escape radius.  These
        // are queue semantics, not merely final-via validation: allowing an
        // out-of-envelope room element to enter the frontier changes which
        // lower-cost door is occupied next and therefore the complete maze
        // expansion order.
        if( via.stopAtFirstDrill )
        {
            const bool hasReferenceNextRoom =
                    state.room != nullptr
                    && state.kind != KIND::DRILL_ENTER
                    && state.kind != KIND::TARGET
                    && state.kind != KIND::OWN_ITEM_TARGET;
            if( hasReferenceNextRoom
                && layers[state.layer].id == via.fanoutSourceLayer
                && via.fanoutMaxDistance > 0 )
            {
                const FLOAT_POINT entry = state.entry.Middle();
                const long double dx = static_cast<long double>( entry.x )
                                       - via.fanoutCenter.x;
                const long double dy = static_cast<long double>( entry.y )
                                       - via.fanoutCenter.y;
                if( std::hypotl( dx, dy )
                    > static_cast<long double>( via.fanoutMaxDistance ) )
                {
                    return false;
                }
            }

            if( state.drill && via.fanoutMinDistance > 0 )
            {
                const long double dx =
                        static_cast<long double>( state.drill->location.x )
                        - via.fanoutCenter.x;
                const long double dy =
                        static_cast<long double>( state.drill->location.y )
                        - via.fanoutCenter.y;
                if( std::hypotl( dx, dy )
                    < static_cast<long double>( via.fanoutMinDistance ) )
                {
                    return false;
                }
            }
        }

        const int id = state.door ? state.door->GetId() : state.page ? state.page->GetId()
                      : state.drill ? state.drill->GetId()
                      : static_cast<std::int32_t>( 31u * static_cast<std::uint32_t>( state.itemId )
                                                   + state.room->shape->GetId() );
        const MAZE_LIST_ELEMENT key{ state.g, state.f, id, state.section };
        if( open.contains( key.SortKey() ) )
            return false;
        // Bound pending work, not the append-only parent store.  A parent
        // state must remain alive for backtracking after it has left the
        // queue; counting those historical states as pending work could stop
        // the search immediately after one door contributed many sections.
        // At most maxExpanded states can be popped and at most maxExpanded
        // remain queued, so the retained chain store is still O(maxExpanded).
        if( open.size() >= static_cast<std::size_t>( maxExpanded ) )
        {
            allocationLimit = true;
            return false;
        }
        open.emplace( key.SortKey(), states.size() );
        states.push_back( state );
        return true;
    };
    const auto adjustmentName = []( MAZE_ADJUSTMENT aAdjustment )
    {
        switch( aAdjustment )
        {
        case MAZE_ADJUSTMENT::LEFT: return "LEFT";
        case MAZE_ADJUSTMENT::RIGHT: return "RIGHT";
        default: return "NONE";
        }
    };
    struct START_ITEM_SHAPE
    {
        std::size_t          layer = 0;
        const ROOM_TERMINAL* terminal = nullptr;
        int                  itemId = 0;
    };
    std::vector<START_ITEM_SHAPE> startShapes;
    for( std::size_t layer = 0; layer < layers.size(); ++layer )
        if( layers[layer].active )
            for( const ROOM_TERMINAL& terminal : layers[layer].starts )
                startShapes.push_back( { layer, &terminal, 0 } );

    // Item.compareTo() orders Freerouting's board items by descending
    // insertion ID.  Each item's tree shapes are then visited in physical
    // layer order.  Completing layer-first changed room IDs and, more
    // importantly, which already-complete room supplies the first 2-D door.
    const auto fallbackItemKey = []( const ROOM_TERMINAL& aTerminal )
    {
        return std::tuple{ aTerminal.owner, aTerminal.start.x, aTerminal.start.y,
                           aTerminal.end.x, aTerminal.end.y };
    };
    std::stable_sort( startShapes.begin(), startShapes.end(),
                      [&]( const START_ITEM_SHAPE& aLeft,
                           const START_ITEM_SHAPE& aRight )
                      {
                          const ROOM_TERMINAL& left = *aLeft.terminal;
                          const ROOM_TERMINAL& right = *aRight.terminal;
                          if( left.itemId != 0 || right.itemId != 0 )
                          {
                              if( left.itemId != right.itemId )
                                  return left.itemId > right.itemId;
                          }
                          else
                          {
                              const auto leftKey = fallbackItemKey( left );
                              const auto rightKey = fallbackItemKey( right );
                              if( leftKey != rightKey )
                                  return leftKey > rightKey;
                          }
                          if( left.treeEntryIndex != right.treeEntryIndex )
                              return left.treeEntryIndex < right.treeEntryIndex;
                          return aLeft.layer < aRight.layer;
                      } );

    // TargetItemExpansionDoor.getId() is based on the board Item insertion
    // id, not on the order in which this search happens to visit the start
    // set.  Existing traces are inserted after pins in both source and native
    // worker boards; retaining that identity is what makes an equal-cost pin
    // door win before a trace door, as it does in Freerouting.
    const auto mazeItemId = []( std::uint64_t aItemId )
    {
        return static_cast<std::int32_t>( static_cast<std::uint32_t>( aItemId ) );
    };
    int nextItemId = 1;
    for( const ROOM_LAYER& layer : layers )
    {
        for( const ROOM_TERMINAL& terminal : layer.starts )
            if( terminal.itemId != 0 )
                nextItemId = std::max( nextItemId, mazeItemId( terminal.itemId ) + 1 );
        for( const ROOM_TERMINAL& terminal : layer.targets )
            if( terminal.itemId != 0 )
                nextItemId = std::max( nextItemId, mazeItemId( terminal.itemId ) + 1 );
    }
    const ROOM_TERMINAL* previousTerminal = nullptr;
    int previousShapeItemId = 0;
    for( START_ITEM_SHAPE& shape : startShapes )
    {
        const auto sameItem = [&]( const ROOM_TERMINAL& aLeft,
                                   const ROOM_TERMINAL& aRight )
        {
            if( aLeft.itemId != 0 || aRight.itemId != 0 )
                return aLeft.itemId != 0 && aLeft.itemId == aRight.itemId;
            return fallbackItemKey( aLeft ) == fallbackItemKey( aRight );
        };
        if( !previousTerminal || !sameItem( *shape.terminal, *previousTerminal ) )
        {
            shape.itemId = shape.terminal->itemId != 0
                                   ? mazeItemId( shape.terminal->itemId )
                                   : nextItemId++;
        }
        else
        {
            shape.itemId = previousShapeItemId;
        }
        previousShapeItemId = shape.itemId;
        previousTerminal = shape.terminal;

        if( stopped() )
            return std::nullopt;
        auto& space = *spaces[shape.layer];
        const ROOM_TERMINAL& start = *shape.terminal;
        // MazeSearchEngine.init() creates exactly one incomplete room for
        // each search-tree entry, containing Connectable's complete trace
        // connection shape.  In particular, PolylineTrace contributes its
        // full centre-line segment.  Sampling that segment into uncovered
        // points changed which room an earlier trace claimed and therefore
        // changed every later room/door id and queue tie-break.
        if( !start.connectionArea )
        {
            const INT_OCTAGON contained = INT_OCTAGON::FromSegment(
                    start.start, start.end );
            space.complete( space.incomplete( {
                    INT_OCTAGON::FromBox( layers[shape.layer].bounds ),
                    layers[shape.layer].id, contained } ) );
        }
        else
        {
            // The native area terminal still carries its exact polygon in
            // connectionArea rather than a TileShape value.  Preserve the
            // existing safe on-copper seeding until that polygon is promoted
            // to the common convex-shape terminal representation.
            std::vector<INT_OCTAGON> cuts;
            cuts.reserve( layers[shape.layer].obstacles.size() + 1 );
            cuts.push_back( INT_OCTAGON::FromBox(
                    layers[shape.layer].bounds ) );
            for( const SHAPE_TREE_ENTRY& obstacle :
                 layers[shape.layer].obstacles )
            {
                if( obstacle.layer == layers[shape.layer].id
                    && obstacle.IsTraceObstacle( net ) )
                {
                    cuts.push_back( obstacle.BoundingOctagon() );
                }
            }

            for( const ROUTER_POINT& seedPoint :
                 TARGET_ITEM_EXPANSION_DOOR::IntegralRoomSeedPoints(
                         start.start, start.end, cuts ) )
            {
                const INT_OCTAGON seed = INT_OCTAGON::FromSegment(
                        seedPoint, seedPoint );
                space.complete( space.incomplete( {
                        INT_OCTAGON::FromBox( layers[shape.layer].bounds ),
                        layers[shape.layer].id, seed } ) );
            }
        }
    }

    // The source creates every start room before it inserts any target-item
    // door into the queue.  Keep attachment/queue seeding as a distinct pass.
    for( const START_ITEM_SHAPE& shape : startShapes )
    {
        auto& space = *spaces[shape.layer];
        const ROOM_TERMINAL& start = *shape.terminal;
        for( const auto& [id, room] : space.byId )
        {
            if( room->shape->IsObstacle() )
                continue;

            std::optional<FLOAT_POINT> attachment =
                    connectionCentreInRoom45Multilayer(
                            start, room->shape->GetOctagon() );
            double bestDistance = attachment
                                          ? remaining( *attachment, shape.layer )
                                          : std::numeric_limits<double>::infinity();
            // ConductionArea.centreOfGravity() needs the exact clipped
            // polygon.  Until that region is carried into ROOM_TERMINAL,
            // preserve the exact, safe on-copper area attachment.
            if( !attachment )
                for( const ROOM_LAYER& targetLayer : layers )
                    for( const ROOM_TERMINAL& target : targetLayer.targets )
                        for( const ROUTER_POINT& toward :
                             { target.start, target.end } )
                        {
                            const auto candidate = nearestInRoom45Multilayer(
                                    start, toward,
                                    room->shape->GetOctagon() );
                            if( !candidate )
                                continue;
                            const FLOAT_POINT point{
                                    static_cast<double>( candidate->x ),
                                    static_cast<double>( candidate->y ) };
                            const double candidateDistance =
                                    remaining( point, shape.layer );
                            if( !attachment
                                || candidateDistance < bestDistance )
                            {
                                attachment = point;
                                bestDistance = candidateDistance;
                            }
                        }
            if( !attachment )
                continue;
            const FLOAT_POINT p = *attachment;
            STATE state;
            state.room = room; state.layer = shape.layer; state.entry = { p, p };
            state.f = bestDistance; state.owner = start.owner; state.itemId = shape.itemId;
            state.backtrackPin = &start;
            const ROUTER_BOX treeBounds = INT_BOX::Dimension( start.treeBounds ) >= 0
                    ? start.treeBounds
                    : ROUTER_BOX{ std::min( start.start.x, start.end.x ),
                                  std::min( start.start.y, start.end.y ),
                                  std::max( start.start.x, start.end.x ),
                                  std::max( start.start.y, start.end.y ) };
            auto startDoor = std::make_unique<TARGET_ITEM_EXPANSION_DOOR>(
                    room->shape.get(), shape.itemId, start.owner,
                    start.start, start.end, treeBounds,
                    start.treeEntryIndex, start.treeOctagon,
                    FREEROUTING_COORDINATE_UNIT_IU, true );
            state.door = startDoor.get();
            startDoors.push_back( std::move( startDoor ) );
            const ROUTER_BOX roomBounds = room->shape->GetShape();
            autorouterDecisionLog(
                    "START_ROOM_SEED",
                    { { "layer", std::to_string( shape.layer ) },
                      { "owner", std::to_string( start.owner ) },
                      { "item_id", std::to_string( shape.itemId ) },
                      { "room_id", std::to_string( room->shape->GetId() ) },
                      { "room_bounds", autorouterDecisionBounds( roomBounds ) },
                      { "door_dimension", "2" },
                      { "door_bounds",
                        autorouterDecisionBounds( state.door->GetShape() ) },
                      { "attachment", std::to_string( attachment->x ) + ','
                                              + std::to_string( attachment->y ) },
                      { "sorting_value", std::to_string( bestDistance ) } } );
            push( state );
        }
    }
    const int targetIdBase = nextItemId;
    int fanoutEnvelopeRejects = 0;
    int drillRoomFailures = 0;
    int drillRoomMismatches = 0;
    int viaStyleRejects = 0;
    bool loggedRoomMismatch = false;
    while( !open.empty() && step() )
    {
        const auto index = open.begin()->second;
        open.erase( open.begin() );
        const auto current = states[index];
        auto& space = *spaces[current.layer];
        const auto& layer = layers[current.layer];
        const auto from = current.entry.Middle();
        if( current.room && current.door )
        {
            autorouterDecisionLog(
                    "ROOM_ENTRY_POP_RAW",
                    { { "net", std::to_string( net ) },
                      { "layer", std::to_string( current.layer ) },
                      { "section", std::to_string( current.section ) },
                      { "from_section",
                        current.parent == NONE
                                ? "-1"
                                : std::to_string( states[current.parent].section ) },
                      { "room_id", std::to_string( current.room->shape->GetId() ) },
                      { "room_bounds",
                        autorouterDecisionBounds( current.room->shape->GetShape() ) },
                      { "door_id", std::to_string( current.door->GetId() ) },
                      { "door_dimension",
                        std::to_string( current.door->GetDimension() ) },
                      { "door_bounds",
                        autorouterDecisionBounds( current.door->GetShape() ) },
                      { "shape_entry",
                        std::to_string( current.entry.a.x ) + ','
                                + std::to_string( current.entry.a.y ) + ','
                                + std::to_string( current.entry.b.x ) + ','
                                + std::to_string( current.entry.b.y ) },
                      { "expansion_value", std::to_string( current.g ) },
                      { "sorting_value", std::to_string( current.f ) } } );
        }
        if( via.transitionsEnabled && current.kind == KIND::PAGE )
        {
            ++metrics.drillPages;
            const bool wasCached = current.page->IsValid();
            auto* drills = current.page->GetDrills( drillObstacles, net, layers.size(), via.attachSmd, via.pins,
                    cancel, static_cast<std::size_t>( std::max( 0, maxExpanded - expanded ) ) );
            if( !drills )
                continue;
            if( !wasCached ) metrics.drills += drills->size();
            for( auto& drill : *drills )
            {
                if( stopped() )
                    return std::nullopt;
                if( !drill.valid || drill.occupied[current.layer] )
                    continue;
                if( via.stopAtFirstDrill
                    && layers[current.layer].id == via.fanoutSourceLayer )
                {
                    const long double dx = static_cast<long double>( drill.location.x )
                                                   - via.fanoutCenter.x;
                    const long double dy = static_cast<long double>( drill.location.y )
                                                   - via.fanoutCenter.y;
                    const long double distance = std::hypotl( dx, dy );
                    if( distance < static_cast<long double>( via.fanoutMinDistance )
                        || ( via.fanoutMaxDistance > 0
                             && distance
                                        > static_cast<long double>( via.fanoutMaxDistance ) ) )
                    {
                        ++fanoutEnvelopeRejects;
                        continue;
                    }
                }
                bool roomsReady = true;
                for( std::size_t i = 0; i < layers.size(); ++i )
                    if( layers[i].active && !drill.rooms[i] )
                    {
                        roomsReady = false;
                        break;
                    }
                if( !roomsReady )
                {
                    if( !via.canDrill( drill.location ) )
                    { drill.valid = false; continue; }
                    for( std::size_t i = 0; i < layers.size(); ++i )
                    {
                        if( !layers[i].active )
                            continue;
                        ROOM* room = roomAt( i, drill.location );
                        if( !room )
                        {
                            ++drillRoomFailures;
                            if( autorouterDebugEnabled() && drillRoomFailures == 1 )
                            {
                                std::ostringstream message;
                                message << "ROOM45_DRILL_ROOM_MISSING net=" << net
                                        << " drill=(" << drill.location.x << ','
                                        << drill.location.y << ") ordinal=" << i
                                        << " layer=" << layers[i].id;
                                autorouterDebugLog( message.str() );
                            }
                            drill.valid = false;
                            break;
                        }
                        drill.rooms[i] = room->shape.get();
                    }
                }
                if( !drill.valid || drill.rooms[current.layer] != current.room->shape.get() )
                {
                    if( drill.valid )
                    {
                        ++drillRoomMismatches;
                        if( !loggedRoomMismatch && autorouterDebugEnabled() )
                        {
                            loggedRoomMismatch = true;
                            std::ostringstream message;
                            message << "ROOM45_DRILL_ROOM_MISMATCH net=" << net
                                    << " drill=(" << drill.location.x << ','
                                    << drill.location.y << ") layer="
                                    << layers[current.layer].id << " current_room="
                                    << current.room->shape->GetId() << " drill_room="
                                    << ( drill.rooms[current.layer]
                                                 ? drill.rooms[current.layer]->GetId() : -1 );
                            autorouterDebugLog( message.str() );
                        }
                    }
                    continue;
                }
                FLOAT_POINT compareCorner = from;
                if( current.backtrackPin )
                {
                    if( const auto exit = PIN::NearestTraceExitCorner(
                                current.backtrackPin->start,
                                current.backtrackPin->traceExitRestrictions,
                                drill.location,
                                current.backtrackPin->traceExitOffset ) )
                    {
                        compareCorner = { static_cast<double>( exit->x ),
                                          static_cast<double>( exit->y ) };
                    }
                }
                const auto nearest = MAZE_EXPANSION_ENGINE::Nearest(
                        drill.freeShape, compareCorner );
                const auto cost = MAZE_EXPANSION_ENGINE::ToDrill(
                        drill.freeShape, compareCorner, current.g,
                        via.normalCost, true, layer.horizontalCost, layer.verticalCost,
                        remaining( nearest, current.layer ) );
                STATE state = current;
                state.kind = KIND::DRILL_ENTER; state.page = nullptr; state.drill = &drill;
                state.section = current.layer; state.entry = { cost.entry, cost.entry };
                state.g = cost.expansion; state.f = cost.sorting;
                // Pages are only lazy expansion work, not physical backtrack doors.
                state.parent = current.parent;
                push( state );
            }
            continue; // Reference page sections are deliberately not occupied.
        }
        if( current.drill )
        {
            if( current.drill->occupied[current.layer] )
                continue;
            current.drill->occupied[current.layer] = true;
            if( current.kind == KIND::DRILL_ENTER )
            {
                const bool fanoutDrill = via.stopAtFirstDrill
                                         && layers[current.layer].id
                                                    == via.fanoutSourceLayer;
                for( std::size_t to = 0; to < layers.size(); ++to )
                {
                    if( to == current.layer || !layers[to].active || current.drill->occupied[to] )
                        continue;
                    std::optional<ROUTING_EDGE_STYLE> selectedStyle;
                    if( via.selectViaStyle )
                    {
                        selectedStyle = via.selectViaStyle(
                                current.drill->location, layers[current.layer].id,
                                layers[to].id );
                        if( !selectedStyle )
                        {
                            ++viaStyleRejects;
                            continue;
                        }
                    }
                    STATE state = current;
                    state.kind = fanoutDrill ? KIND::FANOUT_TARGET : KIND::DRILL_EXIT;
                    state.layer = to; state.section = to;
                    state.room = spaces[to]->roomForShape(
                            current.drill->rooms[to], "drill transition" );
                    state.parent = index;
                    state.viaStyle = std::move( selectedStyle );
                    state.ripupCost = 0;
                    state.rippedGroup.reset();
                    state.roomRipped = false;
                    state.adjustment = MAZE_ADJUSTMENT::NONE;
                    state.alreadyChecked = false;
                    if( const auto* obstacle =
                                dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>(
                                        state.room->shape.get() ) )
                    {
                        state.ripupCost = obstacle->GetRipupCost();
                        state.rippedGroup = obstacle->GetGroup();
                        state.roomRipped = true;
                        state.alreadyChecked = true;
                        state.g += state.ripupCost;
                    }
                    // Fanout changes the stopping condition, not the A* lower bound.
                    // Freerouting's MazeExpansionEngine.expandToOtherLayers()
                    // always adds DestinationDistance on the destination layer;
                    // MazeSearchEngine stops only when that queued drill-layer
                    // element is later removed from the frontier.  Zeroing the
                    // heuristic here made every reachable drill outrank a nearby
                    // real target and produced a via for almost every SMD pin.
                    state.f = state.g + remaining( from, to );
                    push( state );
                    ++metrics.layerTransitions;
                }
                continue;
            }
        }
        if( current.kind == KIND::OWN_ITEM_TARGET )
        {
            // A source TargetItemExpansionDoor for another member of the
            // start set is a real frontier element, but it is not a route
            // destination and has no next room.  Occupying it suppresses the
            // separately seeded copy of the same item/room door.
            if( !current.door
                || occupied.contains( { current.door, current.section } ) )
            {
                continue;
            }
            occupied.emplace( current.door, current.section );
            continue;
        }
        if( current.kind == KIND::TARGET || current.kind == KIND::FANOUT_TARGET )
        {
            ROOM_MULTILAYER_PATH result{
                    {}, current.owner,
                    current.kind == KIND::FANOUT_TARGET
                            ? std::numeric_limits<std::size_t>::max()
                            : current.targetOwner,
                    {}, 0, {} };
            std::vector<std::size_t> chain;
            for( auto i = index; i != NONE; i = states[i].parent )
                chain.push_back( i );
            std::reverse( chain.begin(), chain.end() );
            std::set<std::size_t> rippedGroups;
            for( std::size_t entry : chain )
            {
                if( states[entry].rippedGroup && states[entry].ripupCost > 0 )
                {
                    rippedGroups.insert( *states[entry].rippedGroup );
                    result.ripupCost += states[entry].ripupCost;
                }
            }
            result.rippedObstacleGroups.assign( rippedGroups.begin(), rippedGroups.end() );
            result.nodes.push_back( { states[chain.front()].entry.Middle().Round(),
                                      layers[states[chain.front()].layer].id } );
            bool valid = true;
            std::vector<OCTAGONAL_CORRIDOR_STEP> corridor;
            std::size_t corridorLayer = NONE;
            const ROOM_TERMINAL* startTerminal =
                    states[chain.front()].backtrackPin;
            bool initialCorridor = true;
            auto locateCorridor = [&]()
            {
                if( corridor.empty() )
                    return true;

                const auto located = FOUND_CONNECTION_LOCATOR_45_DEGREE::LocateOctagonal(
                        result.nodes.back().point, corridor,
                        sourceTraceRooms ? sectionOffset : 0,
                        FREEROUTING_TRACE_WIDTH_TOLERANCE_IU,
                        false, static_cast<std::int64_t>(
                                       FREEROUTING_COORDINATE_UNIT_IU ),
                        initialCorridor
                                ? FOUND_CONNECTION_LOCATOR_45_DEGREE::START_ENDPOINT_LOCATOR(
                                          [startTerminal](
                                                  ROUTER_POINT aFrom,
                                                  const INT_OCTAGON& aRoom )
                                                  -> std::optional<ROUTER_POINT>
                                          {
                                              if( !startTerminal )
                                                  return std::nullopt;
                                              return nearestInRoom45Multilayer(
                                                      *startTerminal, aFrom,
                                                      aRoom );
                                          } )
                                : FOUND_CONNECTION_LOCATOR_45_DEGREE::START_ENDPOINT_LOCATOR{} );
                if( !located )
                {
                    if( autorouterDebugEnabled() )
                    {
                        std::ostringstream message;
                        message << "ROOM45_DRILL_LOCATOR_REJECTED net=" << net
                                << " chain=" << chain.size()
                                << " corridor=" << corridor.size()
                                << " layer=" << layers[corridorLayer].id
                                << " from=(" << result.nodes.back().point.x << ','
                                << result.nodes.back().point.y << ')';
                        autorouterDebugLog( message.str() );
                    }
                    return false;
                }

                // The seed is a queue-ordering centroid.  On the first
                // same-layer corridor the source locator replaces it with
                // the point on the exact start-item connection shape nearest
                // to the backtracked route.  Subsequent corridor starts are
                // physical drill locations and must remain fixed.
                if( initialCorridor )
                    result.nodes.back().point = located->front();

                for( std::size_t point = 1; point < located->size(); ++point )
                {
                    result.nodes.push_back(
                            { ( *located )[point], layers[corridorLayer].id } );
                    result.edgeStyles.emplace_back();
                }

                std::ostringstream locatedPoints;
                for( std::size_t point = 0; point < located->size(); ++point )
                {
                    if( point > 0 )
                        locatedPoints << ';';
                    locatedPoints << ( *located )[point].x << ','
                                  << ( *located )[point].y;
                }
                autorouterDecisionLog(
                        "LOCATED_PATH_45",
                        { { "net", std::to_string( net ) },
                          { "layer", std::to_string( layers[corridorLayer].id ) },
                          { "section_offset", std::to_string( sectionOffset ) },
                          { "corridor_steps", std::to_string( corridor.size() ) },
                          { "points", locatedPoints.str() } } );
                corridor.clear();
                corridorLayer = NONE;
                initialCorridor = false;
                return true;
            };
            for( std::size_t i = 1; i < chain.size() && valid; ++i )
            {
                const auto& before = states[chain[i - 1]];
                const auto& after = states[chain[i]];
                if( after.kind == KIND::DRILL_EXIT || after.kind == KIND::FANOUT_TARGET )
                {
                    if( !corridor.empty() )
                    {
                        valid = false;
                        break;
                    }
                    if( result.nodes.back().point != after.drill->location )
                    { valid = false; break; }
                    result.nodes.push_back( { after.drill->location, layers[after.layer].id } );
                    result.edgeStyles.push_back( after.viaStyle.value_or(
                            ROUTING_EDGE_STYLE{} ) );
                    continue;
                }
                FLOAT_LINE entry = after.entry;
                if( after.kind == KIND::DRILL_ENTER )
                {
                    const auto p = after.drill->location;
                    entry = { { static_cast<double>( p.x ), static_cast<double>( p.y ) },
                              { static_cast<double>( p.x ), static_cast<double>( p.y ) } };
                }
                if( corridor.empty() )
                    corridorLayer = before.layer;
                if( before.layer != corridorLayer || after.layer != corridorLayer )
                {
                    valid = false;
                    break;
                }
                if( after.door )
                {
                    // Source backtracking stores the selected section number,
                    // not the projected shape entry used by the A* state.  The
                    // locator recalculates the raw door section before choosing
                    // its nearest point.
                    const auto rawSections = after.door->GetSectionSegments(
                            sourceTraceRooms ? sectionOffset : 0,
                            sourceTraceRooms
                                    ? FREEROUTING_TRACE_WIDTH_TOLERANCE_IU : 0,
                            sourceTraceRooms ? 0 : 10 * sectionOffset,
                            std::numeric_limits<std::size_t>::max(), true );
                    if( after.section >= rawSections.size() )
                    {
                        valid = false;
                        break;
                    }
                    entry = rawSections[after.section];
                }
                const auto* beforeObstacle =
                        dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>(
                                before.room->shape.get() );
                const bool obstacleRipped = beforeObstacle
                        && ( before.roomRipped
                             || ( after.rippedGroup
                                  && *after.rippedGroup
                                             == beforeObstacle->GetGroup() ) );
                corridor.push_back(
                        { before.room->shape->GetOctagon(),
                          after.door
                                  ? std::optional<INT_OCTAGON>(
                                            after.door->GetOctagonShape() )
                                  : std::nullopt,
                          entry,
                          obstacleRipped } );

                if( after.kind == KIND::DRILL_ENTER || after.kind == KIND::TARGET )
                    valid = locateCorridor();
            }
            if( valid )
                valid = locateCorridor();
            if( valid && !( cancel && cancel() ) )
            {
                metrics.rippedRooms += static_cast<int>( result.rippedObstacleGroups.size() );
                metrics.ripupCost += result.ripupCost;
                metrics.routed = true;
                return result;
            }
            continue;
        }
        // Freerouting only occupies the section after expandToRoomDoors()
        // reports that something was expanded.  In particular, entering a
        // room through a small or thin door may deliberately produce no work;
        // that section must remain available to a cheaper/later frontier
        // entry.  Occupying it here, at pop time, suppressed that retry and
        // changed deterministic maze ordering.
        if( current.door && occupied.contains( { current.door, current.section } ) )
            continue;
        ++metrics.sections;
        const std::size_t doorsBeforeCompletion =
                current.room->shape->GetDoors().size();
        autorouterDecisionLog(
                "ROOM_ENTRY_POP",
                { { "net", std::to_string( net ) },
                  { "layer", std::to_string( current.layer ) },
                  { "section", std::to_string( current.section ) },
                  { "room_id", std::to_string( current.room->shape->GetId() ) },
                  { "room_bounds",
                    autorouterDecisionBounds( current.room->shape->GetShape() ) },
                  { "from_section",
                    current.parent == NONE
                            ? "-1"
                            : std::to_string( states[current.parent].section ) },
                  { "door_id",
                    current.door ? std::to_string( current.door->GetId() ) : "-1" },
                  { "door_dimension",
                    current.door ? std::to_string( current.door->GetDimension() ) : "-1" },
                  { "door_bounds",
                    current.door ? autorouterDecisionBounds( current.door->GetShape() ) : "" },
                  { "doors_before", std::to_string( doorsBeforeCompletion ) },
                  { "expansion_value", std::to_string( current.g ) },
                  { "sorting_value", std::to_string( current.f ) } } );
        space.completeNeighbours( current.room );
        autorouterDecisionLog(
                "ROOM_COMPLETE_SYNC",
                { { "net", std::to_string( net ) },
                  { "layer", std::to_string( current.layer ) },
                  { "section", std::to_string( current.section ) },
                  { "room_id", std::to_string( current.room->shape->GetId() ) },
                  { "room_bounds",
                    autorouterDecisionBounds( current.room->shape->GetShape() ) },
                  { "doors_before", std::to_string( doorsBeforeCompletion ) },
                  { "doors_after",
                    std::to_string( current.room->shape->GetDoors().size() ) } } );
        if( stopped() )
            return std::nullopt;

        // MazeSearchEngine.expandToRoomDoors does not leave a free-space room
        // through a door narrower than the compensated trace diameter.  It
        // completes the room first (the topology mutation above is still
        // required), then leaves that entry unexpanded so another section can
        // reach it.  Obstacle entries are handled by the shove/rip-up path and
        // therefore are not rejected by this free-space guard.
        const bool currentDoorIsSmall = current.door
                && current.door->IsSmallFor45DegreeTrace(
                        2 * ( sectionOffset
                              + FREEROUTING_TRACE_WIDTH_TOLERANCE_IU ) );
        auto* currentObstacle = dynamic_cast<OBSTACLE_EXPANSION_ROOM*>(
                current.room->shape.get() );
        if( currentDoorIsSmall )
        {
            EXPANSION_ROOM* fromRoom =
                    current.door->OtherRoom( current.room->shape.get() );
            // The source applies its early small-door rejection only while
            // entering ordinary FreeSpaceExpansionRoom.  An obstacle room
            // must still run checkRipup through a narrow door; conversely a
            // free room may be left through that door after its source
            // obstacle has been ripped.  Testing only fromRoom accidentally
            // rejected every narrow free-to-obstacle entry before it could
            // enqueue the paid retry.
            if( !currentObstacle
                && !dynamic_cast<OBSTACLE_EXPANSION_ROOM*>( fromRoom ) )
                continue;
        }

        bool somethingExpanded = false;
        const bool nextRoomIsThick = DETAIL::RoomIsThick(
                *current.room->shape, sectionOffset, current.door, from,
                currentDoorIsSmall );
        int currentRoomRipupCost = 0;
        if( currentObstacle && !current.alreadyChecked )
        {
            const auto* previousObstacle = current.door
                    ? dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>(
                              current.door->OtherRoom( current.room->shape.get() ) )
                    : nullptr;
            currentRoomRipupCost = previousObstacle
                                           && current.adjustment == MAZE_ADJUSTMENT::NONE
                                           && previousObstacle->GetGroup()
                                                      == currentObstacle->GetGroup()
                                   ? 1 : currentObstacle->GetRipupCost();
        }

        auto expandToDoorSection = [&]( EXPANSION_DOOR* aDoor,
                                        std::size_t aSection,
                                        const FLOAT_LINE& aShapeEntry,
                                        int aAddCost,
                                        MAZE_ADJUSTMENT aAdjustment )
        {
            if( !aDoor || occupied.contains( { aDoor, aSection } ) )
                return false;

            ROOM* next = space.roomForShape(
                    aDoor->OtherRoom( current.room->shape.get() ),
                    "door-section expansion" );
            if( !next->complete || !next->active )
                return false;

            const FLOAT_POINT to = aShapeEntry.Middle();
            double bend = 0;
            if( current.parent != NONE
                && states[current.parent].layer == current.layer )
            {
                const STATE& previous = states[current.parent];
                FLOAT_POINT previousDoorCentre = previous.entry.Middle();
                if( previous.door )
                {
                    const auto gravity =
                            previous.door->GetOctagonShape().CentreOfGravity();
                    previousDoorCentre = { gravity.first, gravity.second };
                }
                else if( previous.drill )
                {
                    previousDoorCentre = {
                            static_cast<double>( previous.drill->location.x ),
                            static_cast<double>( previous.drill->location.y ) };
                }
                else if( previous.page )
                {
                    const ROUTER_POINT centre = previous.page->Center();
                    previousDoorCentre = { static_cast<double>( centre.x ),
                                           static_cast<double>( centre.y ) };
                }
                bend = MAZE_LIST_ELEMENT::BendPenalty(
                        previousDoorCentre, from, to, layer.bendCost );
            }
            const double g = current.g
                    + from.WeightedDistance(
                            to, layer.horizontalCost, layer.verticalCost )
                    + bend + aAddCost;
            const bool roomRipped =
                    ( aAddCost > 0 && aAdjustment == MAZE_ADJUSTMENT::NONE )
                    || ( current.alreadyChecked && current.roomRipped );
            const std::optional<std::size_t> rippedGroup =
                    aAddCost > 0 && aAdjustment == MAZE_ADJUSTMENT::NONE
                            && currentObstacle
                    ? std::optional<std::size_t>( currentObstacle->GetGroup() )
                    : std::nullopt;

            STATE state;
            state.room = next;
            state.layer = current.layer;
            state.door = aDoor;
            state.section = aSection;
            state.entry = aShapeEntry;
            state.g = g;
            state.f = g + remaining( to, current.layer );
            state.parent = index;
            state.owner = current.owner;
            state.ripupCost = aAddCost;
            state.rippedGroup = rippedGroup;
            state.roomRipped = roomRipped;
            state.adjustment = aAdjustment;

            const ROUTER_BOX doorBounds = aDoor->GetShape();
            const ROUTER_BOX fromDoorBounds = current.door
                                                    ? current.door->GetShape()
                                                    : ROUTER_BOX{};
            autorouterDecisionLog(
                    "RAW_SECTION_ASSIGN",
                    { { "net", std::to_string( net ) },
                      { "layer", std::to_string( current.layer ) },
                      { "selected_section", std::to_string( aSection ) },
                      { "from_section", std::to_string( current.section ) },
                      { "backtrack_section",
                        current.parent == NONE
                                ? "0"
                                : std::to_string( states[current.parent].section ) },
                      { "add_costs", std::to_string( aAddCost ) },
                      { "adjustment", adjustmentName( aAdjustment ) },
                      { "room_ripped", roomRipped ? "true" : "false" },
                      { "door_dimension", std::to_string( aDoor->GetDimension() ) },
                      { "door_bounds", autorouterDecisionBounds( doorBounds ) },
                      { "from_door_dimension",
                        current.door ? std::to_string( current.door->GetDimension() ) : "-1" },
                      { "from_door_bounds",
                        current.door ? autorouterDecisionBounds( fromDoorBounds ) : "" },
                      { "shape_entry",
                        std::to_string( aShapeEntry.a.x ) + ','
                                + std::to_string( aShapeEntry.a.y ) + ','
                                + std::to_string( aShapeEntry.b.x ) + ','
                                + std::to_string( aShapeEntry.b.y ) },
                      { "expansion_value", std::to_string( state.g ) },
                      { "sorting_value", std::to_string( state.f ) } } );
            // expandToDoorSection() reports that the section was expanded
            // after constructing the MazeListElement, regardless of whether
            // TreeSet.add() retains it.  The fanout TreeSet override and the
            // normal comparator may reject an out-of-envelope or duplicate
            // element, but the current entry is still occupied by
            // occupyNextElement().  Returning push(state) here left it
            // unoccupied and allowed a later frontier entry to expand the
            // same room/door section a second time.
            push( state );
            return true;
        };
        int targetId = targetIdBase;
        for( std::size_t i = 0; i < current.layer; ++i )
            targetId += layers[i].targets.size();
        const auto enqueueTarget = [&]( const ROOM_TERMINAL& target,
                                        bool aDestination )
        {
            const auto targetPoint = nearestInRoom45Multilayer(
                    target, from.Round(), current.room->shape->GetOctagon() );
            if( !targetPoint )
            {
                if( autorouterDebugEnabled() )
                {
                    const ROUTER_BOX roomBounds =
                            current.room->shape->GetOctagon().BoundingBox();
                    std::ostringstream message;
                    message << "ROOM45_TARGET_REJECT net=" << net
                            << " layer=" << layer.id
                            << " room=" << current.room->shape->GetId()
                            << " bounds=(" << roomBounds.minX << ',' << roomBounds.minY
                            << ")-(" << roomBounds.maxX << ',' << roomBounds.maxY << ')'
                            << " target=(" << target.start.x << ',' << target.start.y
                            << ")-(" << target.end.x << ',' << target.end.y << ')';
                    autorouterDebugLog( message.str() );
                }
                return;
            }
            const FLOAT_POINT to{ static_cast<double>( targetPoint->x ),
                                  static_cast<double>( targetPoint->y ) };
            STATE state;
            state.kind = aDestination ? KIND::TARGET : KIND::OWN_ITEM_TARGET;
            state.room = aDestination ? current.room : nullptr;
            state.layer = current.layer;
            state.entry = { to, to }; state.g = current.g + from.WeightedDistance( to, layer.horizontalCost, layer.verticalCost );
            state.f = aDestination
                              ? state.g
                              : state.g + remaining( to, current.layer );
            state.parent = index; state.owner = current.owner;
            state.targetOwner = target.owner;
            state.itemId = target.itemId != 0
                                   ? mazeItemId( target.itemId )
                                   : targetId;
            state.roomRipped = current.roomRipped;
            if( !aDestination )
            {
                const auto matchingDoor = std::find_if(
                        startDoors.begin(), startDoors.end(),
                        [&]( const auto& door )
                        {
                            return door->FirstRoom() == current.room->shape.get()
                                   && door->ItemId() == state.itemId
                                   && door->TreeEntryIndex()
                                              == target.treeEntryIndex;
                        } );
                if( matchingDoor == startDoors.end()
                    || matchingDoor->get() == current.door )
                {
                    return;
                }
                state.door = matchingDoor->get();
            }
            push( state );
            // Java's expandToTargetDoors() reports expansion after assigning
            // an otherwise valid, unoccupied target section; TreeSet
            // deduplication does not change that return value.
            somethingExpanded = true;
        };

        // Every own-net item has a TargetItemExpansionDoor in a completed
        // room.  Doors belonging to the start set are queued and occupied but
        // do not terminate the search; only destination-set doors do.
        for( const auto& target : layer.starts )
            enqueueTarget( target, false );

        if( !via.stopAtFirstDrill || via.allowDirectFanoutTarget )
        for( const auto& target : layer.targets )
        {
            ++targetId;
            enqueueTarget( target, true );
        }

        if( sourceTraceRooms && currentObstacle && current.door
            && !current.alreadyChecked && currentRoomRipupCost != 1
            && nextRoomIsThick && !currentDoorIsSmall
            && currentObstacle->GetTraceInfo() )
        {
            const auto fromSections = current.door->GetSectionSegments(
                    sectionOffset, FREEROUTING_TRACE_WIDTH_TOLERANCE_IU, 0,
                    std::numeric_limits<std::size_t>::max(), true );
            const bool outerSection = !fromSections.empty()
                    && ( current.section == 0
                         || current.section + 1 == fromSections.size() );
            if( outerSection )
            {
                bool shoveCompleted = false;
                if( current.adjustment != MAZE_ADJUSTMENT::RIGHT )
                {
                    std::vector<MAZE_SHOVE_DOOR_SECTION> doors;
                    shoveCompleted = MAZE_TRACE_SHOVER::CheckShoveTraceLine(
                            *current.door, current.section, current.entry,
                            *currentObstacle, sectionOffset, false, doors,
                            FREEROUTING_TRACE_WIDTH_TOLERANCE_IU );
                    autorouterDecisionLog(
                            "TRACE_SHOVE_RESULT",
                            { { "side", "RIGHT" },
                              { "completed", shoveCompleted ? "true" : "false" },
                              { "door_count", std::to_string( doors.size() ) },
                              { "room_id",
                                std::to_string( current.room->shape->GetId() ) },
                              { "from_door_id",
                                std::to_string( current.door->GetId() ) },
                              { "from_section",
                                std::to_string( current.section ) } } );
                    for( const MAZE_SHOVE_DOOR_SECTION& door : doors )
                    {
                        const MAZE_ADJUSTMENT adjustment =
                                door.door->GetDimension() == 2
                                        ? MAZE_ADJUSTMENT::LEFT
                                        : MAZE_ADJUSTMENT::NONE;
                        somethingExpanded = expandToDoorSection(
                                door.door, door.section, door.line, 0,
                                adjustment ) || somethingExpanded;
                    }
                }

                if( current.adjustment != MAZE_ADJUSTMENT::LEFT )
                {
                    std::vector<MAZE_SHOVE_DOOR_SECTION> doors;
                    const bool rightCompleted =
                            MAZE_TRACE_SHOVER::CheckShoveTraceLine(
                            *current.door, current.section, current.entry,
                            *currentObstacle, sectionOffset, true, doors,
                            FREEROUTING_TRACE_WIDTH_TOLERANCE_IU );
                    autorouterDecisionLog(
                            "TRACE_SHOVE_RESULT",
                            { { "side", "LEFT" },
                              { "completed", rightCompleted ? "true" : "false" },
                              { "door_count", std::to_string( doors.size() ) },
                              { "room_id",
                                std::to_string( current.room->shape->GetId() ) },
                              { "from_door_id",
                                std::to_string( current.door->GetId() ) },
                              { "from_section",
                                std::to_string( current.section ) } } );
                    shoveCompleted = rightCompleted || shoveCompleted;
                    for( const MAZE_SHOVE_DOOR_SECTION& door : doors )
                    {
                        const MAZE_ADJUSTMENT adjustment =
                                door.door->GetDimension() == 2
                                        ? MAZE_ADJUSTMENT::RIGHT
                                        : MAZE_ADJUSTMENT::NONE;
                        somethingExpanded = expandToDoorSection(
                                door.door, door.section, door.line, 0,
                                adjustment ) || somethingExpanded;
                    }
                }

                if( !shoveCompleted )
                {
                    if( currentRoomRipupCost > 0 )
                    {
                        STATE retry = current;
                        retry.g += currentRoomRipupCost;
                        retry.f += currentRoomRipupCost;
                        retry.ripupCost = currentRoomRipupCost;
                        retry.rippedGroup = currentObstacle->GetGroup();
                        retry.roomRipped = true;
                        retry.alreadyChecked = true;
                        push( retry );
                    }

                    if( current.door && somethingExpanded )
                        occupied.emplace( current.door, current.section );
                    continue;
                }
            }
        }

        const auto& roomDoors = current.room->shape->GetDoors();
        for( auto doorIt = roomDoors.begin(); doorIt != roomDoors.end(); ++doorIt )
        {
            auto* door = *doorIt;
            // AutorouteEngine.occupyNextElement() never expands the door by
            // which the current room was entered.  Apart from being redundant,
            // putting that door back into the queue changes equal-cost ordering
            // and can create search cycles before its sections become occupied.
            if( door == current.door )
                continue;

            ROOM* next = space.roomForShape(
                    door->OtherRoom( current.room->shape.get() ),
                    "room-door expansion" );
            if( !next->complete || !next->active )
                continue;
            autorouterDecisionLog(
                    "ROOM_DOOR_CANDIDATE",
                    { { "net", std::to_string( net ) },
                      { "layer", std::to_string( current.layer ) },
                      { "from_section", std::to_string( current.section ) },
                      { "room_id", std::to_string( current.room->shape->GetId() ) },
                      { "room_bounds",
                        autorouterDecisionBounds( current.room->shape->GetShape() ) },
                      { "door_dimension", std::to_string( door->GetDimension() ) },
                      { "door_bounds", autorouterDecisionBounds( door->GetShape() ) },
                      { "next_room_id", std::to_string( next->shape->GetId() ) },
                      { "next_room_bounds",
                        autorouterDecisionBounds( next->shape->GetShape() ) },
                      { "is_backtrack", door == current.door ? "true" : "false" } } );
            const auto sections = door->GetSectionSegments(
                    sourceTraceRooms ? sectionOffset : 0,
                    sourceTraceRooms ? FREEROUTING_TRACE_WIDTH_TOLERANCE_IU : 0,
                    sourceTraceRooms ? 0 : 10 * sectionOffset,
                    static_cast<std::size_t>( std::max( 0, maxExpanded - expanded ) ),
                    true );
            autorouterDecisionLog(
                    "ROOM_DOOR_SECTIONS",
                    { { "door_id", std::to_string( door->GetId() ) },
                      { "section_count", std::to_string( sections.size() ) },
                      { "first_occupied",
                        sections.empty() || !occupied.contains( { door, 0 } )
                                ? "false" : "true" },
                      { "next_room_thick", nextRoomIsThick ? "true" : "false" },
                      { "ripup_cost", std::to_string( currentRoomRipupCost ) } } );
            if( nextRoomIsThick
                && !DETAIL::DoorEntryIsThick(
                        *current.room->shape, *door, sections,
                        sectionOffset ) )
            {
                continue;
            }
            for( std::size_t section = 0; section < sections.size(); ++section )
            {
                if( occupied.contains( { door, section } ) )
                    continue;
                FLOAT_LINE shapeEntry = sections[section];
                if( !nextRoomIsThick )
                {
                    if( door->GetDimension() == 1 && section == 0
                        && sections.size() == 1
                        && sections.front().a.DistanceSquared(
                                   sections.front().b ) < 1 )
                    {
                        continue;
                    }

                    const auto projected = DETAIL::SegmentProjection(
                            current.entry, sections[section] );
                    const FLOAT_LINE checkSegment =
                            current.entry.AdjustDirection( sections[section] );
                    const auto firstProjection =
                            sections[section].SegmentProjection( checkSegment );
                    const auto secondProjection =
                            sections[section].SegmentProjection2( checkSegment );
                    const auto lineFields = []( const FLOAT_LINE& aLine )
                    {
                        return std::to_string( aLine.a.x ) + ','
                                + std::to_string( aLine.a.y ) + ','
                                + std::to_string( aLine.b.x ) + ','
                                + std::to_string( aLine.b.y );
                    };
                    autorouterDecisionLog(
                            "ROOM_DOOR_PROJECTION",
                            { { "door_id", std::to_string( door->GetId() ) },
                              { "section", std::to_string( section ) },
                              { "from_entry", lineFields( current.entry ) },
                              { "door_section", lineFields( sections[section] ) },
                              { "first_projection",
                                firstProjection ? lineFields( *firstProjection ) : "" },
                              { "second_projection",
                                secondProjection ? lineFields( *secondProjection ) : "" },
                              { "projection",
                                projected ? lineFields( *projected ) : "" } } );
                    if( !projected )
                        continue;
                    shapeEntry = *projected;
                }
                somethingExpanded = expandToDoorSection(
                        door, section, shapeEntry, currentRoomRipupCost,
                        MAZE_ADJUSTMENT::NONE ) || somethingExpanded;
            }
        }
        // The reference normally reaches the next page through a room door.
        // A completely empty alternate layer has no such door: permit pages
        // there too, otherwise a legitimate two-via crossing is unreachable.
        // Per-drill/per-layer occupation still prevents cycling back through it.
        if( via.transitionsEnabled && !current.drill
            && current.room->shape->IsCompleteFreeSpace()
            && ( somethingExpanded || nextRoomIsThick ) )
            for( auto* page : pages.OverlappingPages(
                         current.room->shape->GetOctagon().BoundingBox() ) )
            {
                const auto nearest = MAZE_EXPANSION_ENGINE::Nearest( page->Shape(), from );
                const auto cost = MAZE_EXPANSION_ENGINE::ToPage( page->Shape(), from, current.g,
                        via.normalCost, layer.horizontalCost, layer.verticalCost, remaining( nearest, current.layer ) );
                STATE state;
                state.kind = KIND::PAGE; state.room = current.room; state.layer = current.layer;
                state.page = page; state.section = current.layer; state.entry = current.entry;
                state.g = cost.expansion; state.f = cost.sorting; state.parent = index; state.owner = current.owner;
                state.backtrackPin = current.parent == NONE ? current.backtrackPin : nullptr;
                push( state );
                somethingExpanded = true;
            }

        if( current.door && somethingExpanded )
            occupied.emplace( current.door, current.section );
    }
    if( autorouterDebugEnabled() )
    {
        std::ostringstream message;
        message << "ROOM45_DRILL_FRONTIER_EXHAUSTED net=" << net
                << " states=" << states.size() << " open=" << open.size()
                << " rooms=" << metrics.rooms << " sections=" << metrics.sections
                << " pages=" << metrics.drillPages << " drills=" << metrics.drills
                << " transitions=" << metrics.layerTransitions
                << " envelope_rejects=" << fanoutEnvelopeRejects
                << " room_failures=" << drillRoomFailures
                << " room_mismatches=" << drillRoomMismatches
                << " style_rejects=" << viaStyleRejects
                << " expanded=" << expanded << " allocation_limit=" << allocationLimit;
        autorouterDebugLog( message.str() );
    }
    return std::nullopt;
}
} // namespace KICAD_AUTOROUTER
