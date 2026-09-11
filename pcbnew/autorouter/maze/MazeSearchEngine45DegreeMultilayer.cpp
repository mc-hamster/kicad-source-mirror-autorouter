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
}

std::optional<ROOM_MULTILAYER_PATH> MAZE_SEARCH_ENGINE_45_DEGREE::FindMultilayerConnection(
        const std::vector<ROOM_LAYER>& layers, int net, double sectionOffset,
        const ROOM_VIA_SETTINGS& via, int maxExpanded, int& expanded,
        ROOM_SEARCH_METRICS& metrics, const ROUTER_CANCEL_CALLBACK& cancel,
        const ROUTER_SEARCH_PROGRESS_CALLBACK& progress,
        bool sourceTraceRooms )
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
    DRILL_PAGE_ARRAY pages( via.bounds, via.pageWidth );
    if( via.stopAtFirstDrill )
        pages.AddFanoutCandidates( via.fanoutCenter, via.fanoutMinDistance,
                                   via.fanoutMaxDistance );
    std::vector<std::unique_ptr<ROOM_SEARCH_45_DEGREE>> spaces;
    int nextRoomId = 1;
    for( const auto& layer : layers )
        spaces.push_back( std::make_unique<ROOM_SEARCH_45_DEGREE>( layer.bounds, layer.obstacles,
                layer.id, net, sectionOffset, maxExpanded, expanded, metrics, cancel, progress,
                layer.ripupObstacles, &nextRoomId ) );

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
            if( item.isRoom && item.BoundingOctagon().Contains( point ) )
                return space.byId.at( item.objectId );
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
    enum class KIND { ROOM_ENTRY, PAGE, DRILL_ENTER, DRILL_EXIT, TARGET, FANOUT_TARGET };
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
        // A completed free room may be divided into source-mandated page-size
        // sections. Rejecting the artificial inter-section door by its
        // midpoint can hide a real same-layer TargetItemExpansionDoor beyond
        // the fanout envelope. The envelope constrains the physical drill,
        // not decomposition-only room travel, so it is applied below when
        // concrete drill candidates are enumerated.

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
    const auto itemKey = []( const ROOM_TERMINAL& aTerminal )
    {
        return std::tuple{ aTerminal.owner, aTerminal.start.x, aTerminal.start.y,
                           aTerminal.end.x, aTerminal.end.y };
    };
    std::stable_sort( startShapes.begin(), startShapes.end(),
                      [&]( const START_ITEM_SHAPE& aLeft,
                           const START_ITEM_SHAPE& aRight )
                      {
                          const auto left = itemKey( *aLeft.terminal );
                          const auto right = itemKey( *aRight.terminal );
                          if( left != right )
                              return left > right;
                          return aLeft.layer < aRight.layer;
                      } );

    int nextItemId = 1;
    std::optional<decltype( itemKey( ROOM_TERMINAL{} ) )> previousItem;
    for( START_ITEM_SHAPE& shape : startShapes )
    {
        const auto key = itemKey( *shape.terminal );
        if( !previousItem || key != *previousItem )
        {
            shape.itemId = nextItemId++;
            previousItem = key;
        }
        else
        {
            shape.itemId = nextItemId - 1;
        }

        if( stopped() )
            return std::nullopt;
        auto& space = *spaces[shape.layer];
        const ROOM_TERMINAL& start = *shape.terminal;
        std::vector<INT_OCTAGON> cuts;
        cuts.reserve( layers[shape.layer].obstacles.size() + 1 );
        cuts.push_back( INT_OCTAGON::FromBox( layers[shape.layer].bounds ) );
        for( const SHAPE_TREE_ENTRY& obstacle : layers[shape.layer].obstacles )
            if( obstacle.layer == layers[shape.layer].id
                && obstacle.IsTraceObstacle( net ) )
                cuts.push_back( obstacle.BoundingOctagon() );

        for( const ROUTER_POINT& seedPoint :
             TARGET_ITEM_EXPANSION_DOOR::IntegralRoomSeedPoints(
                     start.start, start.end, cuts ) )
        {
            const bool covered = std::any_of(
                    space.byId.begin(), space.byId.end(),
                    [&]( const auto& entry )
                    {
                        const ROOM* room = entry.second;
                        return !room->shape->IsObstacle()
                               && room->shape->GetOctagon().Contains( seedPoint );
                    } );
            if( covered )
                continue;

            const INT_OCTAGON seed = INT_OCTAGON::FromBox(
                    { seedPoint.x, seedPoint.y, seedPoint.x, seedPoint.y } );
            space.complete( space.incomplete( {
                    INT_OCTAGON::FromBox( layers[shape.layer].bounds ),
                    layers[shape.layer].id, seed } ) );
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

            std::optional<ROUTER_POINT> attachment;
            double bestDistance = std::numeric_limits<double>::infinity();
            for( const ROOM_LAYER& targetLayer : layers )
                for( const ROOM_TERMINAL& target : targetLayer.targets )
                    for( const ROUTER_POINT& toward : { target.start, target.end } )
                    {
                        const auto candidate = nearestInRoom45Multilayer(
                                start, toward, room->shape->GetOctagon() );
                        if( !candidate )
                            continue;
                        const FLOAT_POINT point{ static_cast<double>( candidate->x ),
                                                 static_cast<double>( candidate->y ) };
                        const double candidateDistance = remaining( point, shape.layer );
                        if( !attachment || candidateDistance < bestDistance )
                        {
                            attachment = candidate;
                            bestDistance = candidateDistance;
                        }
                    }
            if( !attachment )
                continue;
            const FLOAT_POINT p{ static_cast<double>( attachment->x ),
                                 static_cast<double>( attachment->y ) };
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
                    start.start, start.end, treeBounds );
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
                    state.room = spaces[to]->byShape.at( current.drill->rooms[to] );
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
            auto locateCorridor = [&]()
            {
                if( corridor.empty() )
                    return true;

                const auto located = FOUND_CONNECTION_LOCATOR_45_DEGREE::LocateOctagonal(
                        result.nodes.back().point, corridor,
                        sourceTraceRooms ? sectionOffset : 0,
                        FREEROUTING_TRACE_WIDTH_TOLERANCE_IU );
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
        if( currentDoorIsSmall )
        {
            EXPANSION_ROOM* fromRoom =
                    current.door->OtherRoom( current.room->shape.get() );
            if( !dynamic_cast<OBSTACLE_EXPANSION_ROOM*>( fromRoom ) )
                continue;
        }

        bool somethingExpanded = false;
        const bool nextRoomIsThick = DETAIL::RoomIsThick(
                *current.room->shape, sectionOffset, current.door, from,
                currentDoorIsSmall );
        auto* currentObstacle = dynamic_cast<OBSTACLE_EXPANSION_ROOM*>(
                current.room->shape.get() );
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

            ROOM* next = space.byShape.at(
                    aDoor->OtherRoom( current.room->shape.get() ) );
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
            return push( state );
        };
        int targetId = targetIdBase;
        for( std::size_t i = 0; i < current.layer; ++i )
            targetId += layers[i].targets.size();
        if( !via.stopAtFirstDrill || via.allowDirectFanoutTarget )
        for( const auto& target : layer.targets )
        {
            ++targetId;
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
                continue;
            }
            const FLOAT_POINT to{ static_cast<double>( targetPoint->x ),
                                  static_cast<double>( targetPoint->y ) };
            STATE state;
            state.kind = KIND::TARGET; state.room = current.room; state.layer = current.layer;
            state.entry = { to, to }; state.g = current.g + from.WeightedDistance( to, layer.horizontalCost, layer.verticalCost );
            state.f = state.g; state.parent = index; state.owner = current.owner;
            state.targetOwner = target.owner; state.itemId = targetId;
            state.roomRipped = current.roomRipped;
            push( state );
            // Java's expandToTargetDoors() reports expansion after assigning
            // an otherwise valid, unoccupied target section; TreeSet
            // deduplication does not change that return value.
            somethingExpanded = true;
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
                    shoveCompleted = MAZE_TRACE_SHOVER::CheckShoveTraceLine(
                            *current.door, current.section, current.entry,
                            *currentObstacle, sectionOffset, true, doors,
                            FREEROUTING_TRACE_WIDTH_TOLERANCE_IU )
                            || shoveCompleted;
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

            ROOM* next = space.byShape.at( door->OtherRoom( current.room->shape.get() ) );
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
