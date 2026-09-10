/*
 * KiCad, GPL-3.0-or-later. Freerouting a11c0a42 room -> drill-page -> drill ->
 * layer expansion, with the native rectangular centre-space shape adapter.
 * Forced pad shove, obstacle rooms and non-through padstacks are not emulated.
 */
#include "MazeSearchEngine90Degree.h"
#include "RoomSearchContext.h"
#include "MazeExpansionEngine.h"
#include "MazeListElement.h"
#include "RoomCostSpace.h"
#include "../drill/DrillPageArray.h"
#include "../path/FoundConnectionLocator45Degree.h"
#include <deque>
#include <map>
#include <set>

namespace KICAD_AUTOROUTER
{
std::optional<ROOM_MULTILAYER_PATH> MAZE_SEARCH_ENGINE_90_DEGREE::FindMultilayerConnection(
        const std::vector<ROOM_LAYER>& layers, int net, double sectionOffset,
        const ROOM_VIA_SETTINGS& via, int maxExpanded, int& expanded,
        ROOM_SEARCH_METRICS& metrics, const ROUTER_CANCEL_CALLBACK& cancel,
        const ROUTER_SEARCH_PROGRESS_CALLBACK& progress, bool orthogonal )
{
    using DETAIL::ROOM;
    using DETAIL::ROOM_SEARCH;
    if( layers.size() < 2 || !via.canDrill || !std::isfinite( via.normalCost ) || via.normalCost < 0
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
                if( terminal.start.x != terminal.end.x && terminal.start.y != terminal.end.y )
                    return std::nullopt;
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
            destinationDistance.Join( costSpace.ToReference( ROUTER_BOX{
                    std::min( target.start.x, target.end.x ), std::min( target.start.y, target.end.y ),
                    std::max( target.start.x, target.end.x ), std::max( target.start.y, target.end.y ) } ), layer );
    // Bound page allocation before construction, not after a potentially huge
    // array has already been allocated. This is an explicit resource failure.
    const double columns = std::ceil( ( static_cast<double>( via.bounds.maxX ) - via.bounds.minX ) / via.pageWidth );
    const double rows = std::ceil( ( static_cast<double>( via.bounds.maxY ) - via.bounds.minY ) / via.pageWidth );
    if( columns * rows > maxExpanded )
        return std::nullopt;
    DRILL_PAGE_ARRAY pages( via.bounds, via.pageWidth );
    std::vector<std::unique_ptr<ROOM_SEARCH>> spaces;
    int nextRoomId = 1;
    for( const auto& layer : layers )
        spaces.push_back( std::make_unique<ROOM_SEARCH>( layer.bounds, layer.obstacles,
                layer.id, net, sectionOffset, maxExpanded, expanded, metrics, cancel, progress,
                &nextRoomId, layer.ripupObstacles ) );

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
    auto nearestTerminal = []( const ROOM_TERMINAL& terminal, FLOAT_POINT point )
    {
        return MAZE_EXPANSION_ENGINE::Nearest( {
                std::min( terminal.start.x, terminal.end.x ), std::min( terminal.start.y, terminal.end.y ),
                std::max( terminal.start.x, terminal.end.x ), std::max( terminal.start.y, terminal.end.y ) }, point );
    };
    auto remaining = [&]( FLOAT_POINT from, std::size_t fromLayer )
    {
        ++metrics.destinationQueries;
        return costSpace.ToNativeCost( destinationDistance.Calculate(
                costSpace.ToReference( from ), fromLayer ) );
    };
    auto roomAt = [&]( std::size_t layer, ROUTER_POINT point ) -> ROOM*
    {
        auto& space = *spaces[layer];
        const ROUTER_BOX seed{ point.x, point.y, point.x, point.y };
        for( const auto& item : space.tree.Overlaps( seed ) )
            if( item.isRoom )
                return space.byId.at( item.objectId );
        auto rooms = space.complete( space.incomplete( { layers[layer].bounds, layers[layer].id, seed } ) );
        return rooms.size() == 1 ? rooms.front() : nullptr;
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
    };
    constexpr auto NONE = std::numeric_limits<std::size_t>::max();
    std::deque<STATE> states;
    std::map<decltype( MAZE_LIST_ELEMENT{}.SortKey() ), std::size_t> open;
    std::set<std::pair<EXPANSION_DOOR*, std::size_t>> occupied;
    bool allocationLimit = false;
    auto push = [&]( STATE state )
    {
        if( via.stopAtFirstDrill && layers[state.layer].id == via.fanoutSourceLayer
            && via.fanoutMaxDistance > 0 )
        {
            const FLOAT_POINT point = state.entry.Middle();
            const long double dx = static_cast<long double>( point.x )
                                           - via.fanoutCenter.x;
            const long double dy = static_cast<long double>( point.y )
                                           - via.fanoutCenter.y;
            if( std::hypotl( dx, dy )
                > static_cast<long double>( via.fanoutMaxDistance ) )
            {
                return;
            }
        }

        const int id = state.door ? state.door->GetId() : state.page ? state.page->GetId()
                      : state.drill ? state.drill->GetId()
                      : static_cast<std::int32_t>( 31u * static_cast<std::uint32_t>( state.itemId )
                                                   + state.room->shape->GetId() );
        const MAZE_LIST_ELEMENT key{ state.g, state.f, id, state.section };
        if( open.contains( key.SortKey() ) )
            return;
        if( states.size() >= static_cast<std::size_t>( maxExpanded ) )
        { allocationLimit = true; return; }
        open.emplace( key.SortKey(), states.size() );
        states.push_back( state );
    };
    int nextItemId = 1;
    for( std::size_t layer = 0; layer < layers.size(); ++layer )
    {
        if( !layers[layer].active )
            continue;
        auto& space = *spaces[layer];
        for( const auto& start : layers[layer].starts )
        {
            if( stopped() )
                return std::nullopt;
            const ROUTER_BOX contained{
                std::min( start.start.x, start.end.x ), std::min( start.start.y, start.end.y ),
                std::max( start.start.x, start.end.x ), std::max( start.start.y, start.end.y ) };
            space.complete( space.incomplete( { layers[layer].bounds, layers[layer].id, contained } ) );
            const int itemId = nextItemId++;
            for( const auto& [id, room] : space.byId )
            {
                auto overlap = INT_BOX::Intersection( contained, room->shape->GetShape() );
                if( INT_BOX::Dimension( overlap ) < 0 )
                    continue;
                const FLOAT_POINT p{ ( static_cast<double>( overlap.minX ) + overlap.maxX ) / 2,
                                     ( static_cast<double>( overlap.minY ) + overlap.maxY ) / 2 };
                STATE state;
                state.room = room; state.layer = layer; state.entry = { p, p };
                state.f = remaining( p, layer ); state.owner = start.owner; state.itemId = itemId;
                push( state );
            }
        }
    }
    const int targetIdBase = nextItemId;
    while( !open.empty() && !allocationLimit && step() )
    {
        const auto index = open.begin()->second;
        open.erase( open.begin() );
        const auto current = states[index];
        auto& space = *spaces[current.layer];
        const auto& layer = layers[current.layer];
        const auto from = current.entry.Middle();
        if( current.kind == KIND::PAGE )
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
                if( stopped() || allocationLimit )
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
                        if( !room ) { drill.valid = false; break; }
                        drill.rooms[i] = room->shape.get();
                    }
                }
                if( !drill.valid || drill.rooms[current.layer] != current.room->shape.get() )
                    continue;
                const auto nearest = MAZE_EXPANSION_ENGINE::Nearest( drill.freeShape, from );
                const auto cost = MAZE_EXPANSION_ENGINE::ToDrill( drill.freeShape, from, current.g,
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
                    STATE state = current;
                    state.kind = fanoutDrill ? KIND::FANOUT_TARGET : KIND::DRILL_EXIT;
                    state.layer = to; state.section = to;
                    state.room = spaces[to]->byShape.at( current.drill->rooms[to] );
                    state.parent = index;
                    if( const auto* obstacle =
                                dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>(
                                        state.room->shape.get() ) )
                    {
                        state.ripupCost = obstacle->GetRipupCost();
                        state.g += state.ripupCost;
                    }
                    state.f = fanoutDrill ? state.g : state.g + remaining( from, to );
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
                    {}, 0 };
            std::vector<std::size_t> chain;
            for( auto i = index; i != NONE; i = states[i].parent )
                chain.push_back( i );
            std::reverse( chain.begin(), chain.end() );
            std::set<std::size_t> rippedGroups;
            for( std::size_t entry : chain )
            {
                const auto* obstacle = dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>(
                        states[entry].room ? states[entry].room->shape.get() : nullptr );
                if( obstacle && states[entry].ripupCost > 0 )
                {
                    rippedGroups.insert( obstacle->GetGroup() );
                    result.ripupCost += states[entry].ripupCost;
                }
            }
            result.rippedObstacleGroups.assign( rippedGroups.begin(), rippedGroups.end() );
            result.nodes.push_back( { states[chain.front()].entry.Middle().Round(),
                                      layers[states[chain.front()].layer].id } );
            bool valid = true;
            for( std::size_t i = 1; i < chain.size() && valid; ++i )
            {
                const auto& before = states[chain[i - 1]];
                const auto& after = states[chain[i]];
                if( after.kind == KIND::DRILL_EXIT || after.kind == KIND::FANOUT_TARGET )
                {
                    if( result.nodes.back().point != after.drill->location )
                    { valid = false; break; }
                    result.nodes.push_back( { after.drill->location, layers[after.layer].id } );
                    continue;
                }
                FLOAT_LINE entry = after.entry;
                if( after.kind == KIND::DRILL_ENTER )
                {
                    const auto p = after.drill->location;
                    entry = { { static_cast<double>( p.x ), static_cast<double>( p.y ) },
                              { static_cast<double>( p.x ), static_cast<double>( p.y ) } };
                }
                const auto located = FOUND_CONNECTION_LOCATOR_45_DEGREE::LocateRectangular(
                        result.nodes.back().point, { { before.room->shape->GetShape(),
                            after.door ? std::optional<ROUTER_BOX>( after.door->GetShape() ) : std::nullopt,
                            entry } }, orthogonal );
                if( !located ) { valid = false; break; }
                for( std::size_t j = 1; j < located->size(); ++j )
                    result.nodes.push_back( { ( *located )[j], layers[after.layer].id } );
            }
            if( valid && !( cancel && cancel() ) )
            {
                metrics.rippedRooms += static_cast<int>( result.rippedObstacleGroups.size() );
                metrics.ripupCost += result.ripupCost;
                metrics.routed = true;
                return result;
            }
            continue;
        }
        if( current.door && !occupied.emplace( current.door, current.section ).second )
            continue;
        ++metrics.sections;
        space.completeNeighbours( current.room );
        if( stopped() )
            return std::nullopt;
        int targetId = targetIdBase;
        for( std::size_t i = 0; i < current.layer; ++i )
            targetId += layers[i].targets.size();
        for( const auto& target : layer.targets )
        {
            const auto to = nearestTerminal( target, from );
            ++targetId;
            if( !current.room->shape->Contains( to.Round() ) )
                continue;
            STATE state;
            state.kind = KIND::TARGET; state.room = current.room; state.layer = current.layer;
            state.entry = { to, to }; state.g = current.g + from.WeightedDistance( to, layer.horizontalCost, layer.verticalCost );
            state.f = state.g; state.parent = index; state.owner = current.owner;
            state.targetOwner = target.owner; state.itemId = targetId;
            push( state );
        }
        for( auto* door : current.room->shape->GetDoors() )
        {
            ROOM* next = space.byShape.at( door->OtherRoom( current.room->shape.get() ) );
            if( !next->complete || !next->active )
                continue;
            const auto sections = door->GetSectionSegments( 0, 0, 10 * sectionOffset,
                    static_cast<std::size_t>( std::max( 0, maxExpanded - expanded ) ) );
            for( std::size_t section = 0; section < sections.size(); ++section )
            {
                if( occupied.contains( { door, section } ) )
                    continue;
                const auto to = sections[section].Middle();
                double bend = 0;
                if( current.parent != NONE && states[current.parent].layer == current.layer )
                    bend = MAZE_LIST_ELEMENT::BendPenalty( states[current.parent].entry.Middle(), from, to, layer.bendCost );
                int ripupCost = 0;
                if( const auto* obstacle = dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>(
                            next->shape.get() ) )
                {
                    const auto* currentObstacle =
                            dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>(
                                    current.room->shape.get() );
                    ripupCost = currentObstacle
                                            && currentObstacle->GetGroup()
                                                       == obstacle->GetGroup()
                                        ? 1 : obstacle->GetRipupCost();
                }
                STATE state;
                state.room = next; state.layer = current.layer; state.door = door; state.section = section;
                state.entry = sections[section]; state.g = current.g + from.WeightedDistance( to, layer.horizontalCost, layer.verticalCost ) + bend + ripupCost;
                state.f = state.g + remaining( to, current.layer ); state.parent = index; state.owner = current.owner;
                state.ripupCost = ripupCost;
                push( state );
            }
        }
        // The reference normally reaches the next page through a room door.
        // A completely empty alternate layer has no such door: permit pages
        // there too, otherwise a legitimate two-via crossing is unreachable.
        // Per-drill/per-layer occupation still prevents cycling back through it.
        if( !current.drill || current.room->shape->GetDoors().empty() )
            for( auto* page : pages.OverlappingPages( current.room->shape->GetShape() ) )
            {
                const auto nearest = MAZE_EXPANSION_ENGINE::Nearest( page->Shape(), from );
                const auto cost = MAZE_EXPANSION_ENGINE::ToPage( page->Shape(), from, current.g,
                        via.normalCost, layer.horizontalCost, layer.verticalCost, remaining( nearest, current.layer ) );
                STATE state;
                state.kind = KIND::PAGE; state.room = current.room; state.layer = current.layer;
                state.page = page; state.section = current.layer; state.entry = current.entry;
                state.g = cost.expansion; state.f = cost.sorting; state.parent = index; state.owner = current.owner;
                push( state );
            }
    }
    return std::nullopt;
}
} // namespace KICAD_AUTOROUTER
