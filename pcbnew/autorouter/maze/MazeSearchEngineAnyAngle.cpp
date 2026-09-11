/* This file is part of KiCad, licensed under GPL version 3 or later.
 * Derived from Freerouting a11c0a42 (GPL-3.0): ShapeSearchTree,
 * AutorouteEngine room completion, ExpansionDoor and MazeSearchEngine.
 */
#include "MazeSearchEngineAnyAngle.h"

#include "DestinationDistance.h"
#include "MazeListElement.h"
#include "RoomCostSpace.h"
#include "RoomSearchContextAnyAngle.h"
#include "../AutorouterDebug.h"
#include "../expansion/TargetItemExpansionDoor.h"
#include "../path/FoundConnectionLocatorAnyAngle.h"

#include <deque>
#include <map>
#include <set>
#include <sstream>

namespace KICAD_AUTOROUTER
{
namespace
{
using DETAIL::ROOM;
using DETAIL::ROOM_SEARCH_ANY_ANGLE;
using PLANAR::POINT;
using PLANAR::SIMPLEX;

std::optional<ROUTER_POINT> nearestInGeneralRoom(
        const ROOM_TERMINAL& aTerminal, ROUTER_POINT aPoint,
        const SIMPLEX& aRoom )
{
    return TARGET_ITEM_EXPANSION_DOOR::NearestIntegralPointInRoom(
            aTerminal.start, aTerminal.end, aPoint, aRoom );
}
} // namespace


std::optional<ROOM_PATH> MAZE_SEARCH_ENGINE_ANY_ANGLE::FindConnection(
        ROUTER_BOX aBounds, const std::vector<SHAPE_TREE_ENTRY>& aObstacles,
        int aLayer, int aNet, const std::vector<ROOM_TERMINAL>& aStarts,
        const std::vector<ROOM_TERMINAL>& aTargets, double aSectionOffset,
        double aHorizontalCost, double aVerticalCost, int aMaxExpanded,
        int& aExpanded, ROOM_SEARCH_METRICS& aMetrics,
        const ROUTER_CANCEL_CALLBACK& aCancel,
        const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress,
        double aBendCost, const std::vector<ROOM_RIPUP_OBSTACLE>& aRipupObstacles )
{
    if( aStarts.empty() || aTargets.empty()
        || INT_BOX::Dimension( aBounds ) != 2
        || !std::isfinite( aSectionOffset ) || aSectionOffset <= 0
        || !std::isfinite( aHorizontalCost ) || aHorizontalCost <= 0
        || !std::isfinite( aVerticalCost ) || aVerticalCost <= 0
        || !std::isfinite( aBendCost ) || aBendCost < 0 )
    {
        return std::nullopt;
    }

    ROUTER_BOX costBounds = aBounds;
    for( const auto* terminals : { &aStarts, &aTargets } )
    {
        for( const ROOM_TERMINAL& terminal : *terminals )
        {
            costBounds = INT_BOX::Union( costBounds, {
                    std::min( terminal.start.x, terminal.end.x ),
                    std::min( terminal.start.y, terminal.end.y ),
                    std::max( terminal.start.x, terminal.end.x ),
                    std::max( terminal.start.y, terminal.end.y ) } );
        }
    }
    const ROOM_COST_SPACE costSpace( costBounds );
    DESTINATION_DISTANCE destinationDistance(
            { { aHorizontalCost, aVerticalCost } }, { true }, 0, 0 );
    for( const ROOM_TERMINAL& target : aTargets )
    {
        const ROUTER_BOX targetBounds = INT_BOX::Dimension( target.treeBounds ) >= 0
                ? target.treeBounds
                : ROUTER_BOX{ std::min( target.start.x, target.end.x ),
                              std::min( target.start.y, target.end.y ),
                              std::max( target.start.x, target.end.x ),
                              std::max( target.start.y, target.end.y ) };
        destinationDistance.Join( costSpace.ToReference( targetBounds ), 0 );
    }

    ROOM_SEARCH_ANY_ANGLE search(
            aBounds, aObstacles, aLayer, aNet, aSectionOffset,
            aMaxExpanded, aExpanded, aMetrics, aCancel, aProgress,
            aRipupObstacles );
    struct STATE
    {
        ROOM* room = nullptr;
        EXPANSION_DOOR* door = nullptr;
        std::size_t section = 0;
        FLOAT_LINE entry;
        double g = 0;
        double f = 0;
        std::size_t parent = std::numeric_limits<std::size_t>::max();
        std::size_t owner = std::numeric_limits<std::size_t>::max();
        std::optional<std::size_t> target;
        std::uint32_t targetItemId = 0;
        int ripupCost = 0;
    };
    constexpr std::size_t NONE = std::numeric_limits<std::size_t>::max();
    std::deque<STATE> states;
    std::map<decltype( MAZE_LIST_ELEMENT{}.SortKey() ), std::size_t> open;
    std::set<std::pair<EXPANSION_DOOR*, std::size_t>> occupied;
    auto cost = [&]( FLOAT_POINT aFrom, FLOAT_POINT aTo )
    {
        return aFrom.WeightedDistance( aTo, aHorizontalCost, aVerticalCost );
    };
    auto distance = [&]( FLOAT_POINT aFrom )
    {
        ++aMetrics.destinationQueries;
        return costSpace.ToNativeCost(
                destinationDistance.Calculate( costSpace.ToReference( aFrom ), 0 ) );
    };
    auto startAttachment = [&]( const ROOM_TERMINAL& aStart,
                                const SIMPLEX& aRoom )
            -> std::optional<ROUTER_POINT>
    {
        std::optional<ROUTER_POINT> best;
        double bestDistance = std::numeric_limits<double>::infinity();
        for( const ROOM_TERMINAL& target : aTargets )
        {
            for( const ROUTER_POINT& toward : { target.start, target.end } )
            {
                const auto candidate = nearestInGeneralRoom( aStart, toward, aRoom );
                if( !candidate )
                    continue;
                const FLOAT_POINT point{ static_cast<double>( candidate->x ),
                                         static_cast<double>( candidate->y ) };
                const double candidateDistance = distance( point );
                if( !best || candidateDistance < bestDistance )
                {
                    best = candidate;
                    bestDistance = candidateDistance;
                }
            }
        }
        return best;
    };
    auto push = [&]( const STATE& aState )
    {
        const int roomId = aState.room ? aState.room->shape->GetId()
                                      : states[aState.parent].room->shape->GetId();
        const int id = aState.door ? aState.door->GetId()
                                   : static_cast<std::int32_t>(
                                             31u * aState.targetItemId + roomId );
        const MAZE_LIST_ELEMENT key{ aState.g, aState.f, id, aState.section };
        if( open.emplace( key.SortKey(), states.size() ).second )
            states.push_back( aState );
    };

    std::vector<SIMPLEX> cuts{ SIMPLEX::Box( aBounds ) };
    cuts.reserve( aObstacles.size() + 1 );
    for( const SHAPE_TREE_ENTRY& obstacle : aObstacles )
    {
        if( obstacle.layer == aLayer && obstacle.IsTraceObstacle( aNet ) )
            cuts.push_back( obstacle.BoundingSimplex() );
    }

    for( std::size_t startIndex = 0; startIndex < aStarts.size(); ++startIndex )
    {
        const ROOM_TERMINAL& start = aStarts[startIndex];
        const auto seedPoints = TARGET_ITEM_EXPANSION_DOOR::IntegralRoomSeedPoints(
                start.start, start.end, cuts );
        for( const ROUTER_POINT& seedPoint : seedPoints )
        {
            const bool covered = std::any_of(
                    search.byId.begin(), search.byId.end(),
                    [&]( const auto& entry )
                    {
                        const ROOM* room = entry.second;
                        return !room->shape->IsObstacle()
                               && room->shape->GetSimplex().Contains( POINT( seedPoint ) );
                    } );
            if( covered )
                continue;
            const SIMPLEX seed = SIMPLEX::FromBox(
                    { seedPoint.x, seedPoint.y, seedPoint.x, seedPoint.y } );
            search.complete( search.incomplete(
                    { SIMPLEX::Box( aBounds ), aLayer, seed } ) );
        }

        for( const auto& [id, room] : search.byId )
        {
            if( room->shape->IsObstacle() )
                continue;
            const auto attachment = startAttachment(
                    start, room->shape->GetSimplex() );
            if( !attachment )
                continue;
            const FLOAT_POINT point{ static_cast<double>( attachment->x ),
                                     static_cast<double>( attachment->y ) };
            push( { room, nullptr, 0, { point, point }, 0, distance( point ),
                    NONE, start.owner, {},
                    static_cast<std::uint32_t>( startIndex + 1 ) } );
        }
    }

    while( !open.empty() && search.step() )
    {
        const std::size_t index = open.begin()->second;
        open.erase( open.begin() );
        const STATE current = states[index];
        if( current.target )
        {
            ROOM_PATH path{ {}, current.owner, *current.target, {}, 0 };
            std::vector<std::size_t> entries;
            for( std::size_t currentIndex = index; currentIndex != NONE;
                 currentIndex = states[currentIndex].parent )
            {
                entries.push_back( currentIndex );
            }
            std::reverse( entries.begin(), entries.end() );
            std::set<std::size_t> rippedGroups;
            for( const std::size_t entry : entries )
            {
                const auto* obstacle = dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>(
                        states[entry].room ? states[entry].room->shape.get() : nullptr );
                if( obstacle && states[entry].ripupCost > 0 )
                {
                    rippedGroups.insert( obstacle->GetGroup() );
                    path.ripupCost += states[entry].ripupCost;
                }
            }
            path.rippedObstacleGroups.assign( rippedGroups.begin(), rippedGroups.end() );

            std::vector<GENERAL_CORRIDOR_STEP> corridor;
            for( std::size_t entry = 1; entry < entries.size(); ++entry )
            {
                const STATE& from = states[entries[entry - 1]];
                const STATE& to = states[entries[entry]];
                corridor.push_back( {
                        from.room->shape->GetSimplex(),
                        to.door ? std::optional<SIMPLEX>(
                                          to.door->GetSimplexShape() )
                                : std::nullopt,
                        to.entry } );
            }
            const auto located = FOUND_CONNECTION_LOCATOR_ANY_ANGLE::Locate(
                    states[entries.front()].entry.Middle().Round(), corridor );
            if( !located )
                continue;
            path.points = *located;
            aMetrics.rippedRooms += static_cast<int>( path.rippedObstacleGroups.size() );
            aMetrics.ripupCost += path.ripupCost;
            aMetrics.routed = true;
            return path;
        }

        // Match MazeSearchEngine.occupyNextElement(): an entry becomes
        // occupied only after room expansion actually produces work.
        if( current.door && occupied.contains( { current.door, current.section } ) )
            continue;
        ++aMetrics.sections;
        search.completeNeighbours( current.room );
        if( search.stopped() )
            return std::nullopt;

        const bool currentDoorIsSmall = current.door
                && current.door->IsSmallForAnyAngleTrace(
                        2 * ( aSectionOffset
                              + FREEROUTING_TRACE_WIDTH_TOLERANCE_IU ) );
        if( currentDoorIsSmall )
        {
            EXPANSION_ROOM* fromRoom =
                    current.door->OtherRoom( current.room->shape.get() );
            if( !dynamic_cast<OBSTACLE_EXPANSION_ROOM*>( fromRoom ) )
                continue;
        }

        bool somethingExpanded = false;
        const FLOAT_POINT from = current.entry.Middle();
        for( std::size_t targetIndex = 0; targetIndex < aTargets.size(); ++targetIndex )
        {
            const ROOM_TERMINAL& target = aTargets[targetIndex];
            const auto point = nearestInGeneralRoom(
                    target, from.Round(), current.room->shape->GetSimplex() );
            if( !point )
                continue;
            const FLOAT_POINT to{ static_cast<double>( point->x ),
                                  static_cast<double>( point->y ) };
            const double g = current.g + cost( from, to );
            push( { nullptr, nullptr, 0, { to, to }, g, g, index,
                    current.owner, target.owner,
                    static_cast<std::uint32_t>( aStarts.size() + targetIndex + 1 ) } );
            somethingExpanded = true;
        }

        for( EXPANSION_DOOR* door : current.room->shape->GetDoors() )
        {
            if( door == current.door )
                continue;

            ROOM* next = search.byShape.at(
                    door->OtherRoom( current.room->shape.get() ) );
            if( !next->complete || !next->active )
                continue;
            const auto sections = door->GetSectionSegments(
                    0, 0, 10 * aSectionOffset,
                    static_cast<std::size_t>(
                            std::max( 0, aMaxExpanded - aExpanded ) ) );
            for( std::size_t section = 0; section < sections.size(); ++section )
            {
                if( occupied.contains( { door, section } ) )
                    continue;
                const FLOAT_POINT to = sections[section].Middle();
                double bend = 0;
                if( current.parent != NONE )
                {
                    const STATE& previous = states[current.parent];
                    FLOAT_POINT centre = previous.entry.Middle();
                    if( previous.door )
                    {
                        const auto gravity = previous.door->GetSimplexShape().CentreOfGravity();
                        centre = { gravity.first, gravity.second };
                    }
                    bend = MAZE_LIST_ELEMENT::BendPenalty(
                            centre, from, to, aBendCost );
                }
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
                const double g = current.g + cost( from, to ) + bend + ripupCost;
                const ROUTER_BOX doorBounds = door->GetShape();
                const ROUTER_BOX fromDoorBounds = current.door
                                                        ? current.door->GetShape()
                                                        : ROUTER_BOX{};
                autorouterDecisionLog(
                        "RAW_SECTION_ASSIGN",
                        { { "net", std::to_string( aNet ) },
                          { "layer", "0" },
                          { "selected_section", std::to_string( section ) },
                          { "from_section", std::to_string( current.section ) },
                          { "backtrack_section",
                            current.parent == NONE
                                    ? "0"
                                    : std::to_string( states[current.parent].section ) },
                          { "add_costs", std::to_string( ripupCost ) },
                          { "adjustment", "NONE" },
                          { "room_ripped", ripupCost > 0 ? "true" : "false" },
                          { "door_dimension", std::to_string( door->GetDimension() ) },
                          { "door_bounds", autorouterDecisionBounds( doorBounds ) },
                          { "from_door_dimension",
                            current.door ? std::to_string( current.door->GetDimension() ) : "-1" },
                          { "from_door_bounds",
                            current.door ? autorouterDecisionBounds( fromDoorBounds ) : "" },
                          { "expansion_value", std::to_string( g ) },
                          { "sorting_value", std::to_string( g + distance( to ) ) } } );
                push( { next, door, section, sections[section], g,
                        g + distance( to ), index, current.owner, {}, 0, ripupCost } );
                somethingExpanded = true;
            }
        }

        if( current.door && somethingExpanded )
            occupied.emplace( current.door, current.section );
    }

    if( autorouterDebugEnabled() )
    {
        std::ostringstream message;
        message << "ROOM_ANY_FRONTIER_EXHAUSTED layer=" << aLayer
                << " rooms=" << search.rooms.size()
                << " expanded=" << aExpanded;
        autorouterDebugLog( message.str() );
    }
    return std::nullopt;
}

} // namespace KICAD_AUTOROUTER
