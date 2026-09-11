/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Derived from Freerouting a11c0a42 (GPL-3.0): AutorouteEngine's free-room
 * lifecycle, SortedOrthogonalRoomNeighbours, ExpansionDoor, MazeSearchEngine.
 *
 * Host adaptation: inputs are trace-CENTRE free rectangles (full clearance and
 * radius already applied), not upstream's half-clearance compensated shapes.
 * Consequently the locator does not shrink them a second time. It emits a
 * 90/45-degree polyline through the backtracked corridor inside convex rooms.
 * Paid rectangular obstacle rooms participate in this same frontier. The full
 * upstream locator, thin-room/shove geometry and general-convex frontier
 * remain separate work; this class is not a claim of complete engine parity.
 */
#include "MazeSearchEngine90Degree.h"
#include "MazeListElement.h"
#include "RoomSearchContext.h"
#include "RoomCostSpace.h"
#include "../AutorouterDebug.h"
#include "../expansion/TargetItemExpansionDoor.h"

#include <deque>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <sstream>
#include <unordered_map>
#include "../expansion/CompleteFreeSpaceExpansionRoom.h"
#include "../expansion/ExpansionDoor.h"
#include "../expansion/SortedOrthogonalRoomNeighbours.h"
#include "../path/FoundConnectionLocator45Degree.h"

namespace KICAD_AUTOROUTER
{
namespace
{
std::optional<ROUTER_POINT> nearestInRoom( const ROOM_TERMINAL& terminal,
                                           ROUTER_POINT point,
                                           const ROUTER_BOX& room )
{
    return TARGET_ITEM_EXPANSION_DOOR::NearestIntegralPointInRoom(
            terminal.start, terminal.end, point, room );
}

using DETAIL::ROOM;
using DETAIL::ROOM_SEARCH;

const char* adjustmentName( MAZE_ADJUSTMENT aAdjustment )
{
    switch( aAdjustment )
    {
    case MAZE_ADJUSTMENT::LEFT: return "LEFT";
    case MAZE_ADJUSTMENT::RIGHT: return "RIGHT";
    default: return "NONE";
    }
}
} // namespace


namespace DETAIL
{
std::optional<FLOAT_LINE> SegmentProjection(
        const FLOAT_LINE& aFromSegment, const FLOAT_LINE& aToSegment )
{
    // MazeSearchEngine.segmentProjection() intentionally evaluates both
    // projection directions.  In a thin room one direction can clip a valid
    // endpoint which the other direction retains.
    const FLOAT_LINE checkSegment = aFromSegment.AdjustDirection( aToSegment );
    const auto firstProjection = aToSegment.SegmentProjection( checkSegment );
    const auto secondProjection = aToSegment.SegmentProjection2( checkSegment );

    if( !firstProjection )
        return secondProjection;
    if( !secondProjection )
        return firstProjection;

    const auto equalPoint = []( FLOAT_POINT aLeft, FLOAT_POINT aRight )
    {
        return aLeft.x == aRight.x && aLeft.y == aRight.y;
    };

    FLOAT_POINT resultA;
    if( equalPoint( firstProjection->a, aToSegment.a )
        || equalPoint( secondProjection->a, aToSegment.a ) )
    {
        resultA = aToSegment.a;
    }
    else
    {
        resultA = firstProjection->a.DistanceSquared( aToSegment.a )
                                  <= secondProjection->a.DistanceSquared(
                                             aToSegment.a )
                          ? firstProjection->a
                          : secondProjection->a;
    }

    FLOAT_POINT resultB;
    if( equalPoint( firstProjection->b, aToSegment.b )
        || equalPoint( secondProjection->b, aToSegment.b ) )
    {
        resultB = aToSegment.b;
    }
    else
    {
        resultB = firstProjection->b.DistanceSquared( aToSegment.b )
                                  <= secondProjection->b.DistanceSquared(
                                             aToSegment.b )
                          ? firstProjection->b
                          : secondProjection->b;
    }

    return FLOAT_LINE{ resultA, resultB };
}


bool RoomIsThick( const EXPANSION_ROOM& aRoom,
                  double aCompensatedTraceHalfWidth,
                  const EXPANSION_DOOR* aEntryDoor,
                  FLOAT_POINT aEntryMiddle, bool aCurrentDoorIsSmall )
{
    if( !std::isfinite( aCompensatedTraceHalfWidth )
        || aCompensatedTraceHalfWidth < 0 )
    {
        return false;
    }

    // ObstacleExpansionRoom.roomShapeIsThick() compares the compensated
    // obstacle half-width with the incoming compensated trace half-width.
    // Native obstacle rooms already contain that compensation, so their
    // minimum full width is the equivalent data available at this boundary.
    if( aRoom.IsObstacle() )
    {
        const double width = aRoom.UsesGeneralShape()
                                     ? aRoom.GetSimplex().MinWidth()
                                     : aRoom.GetOctagon().MinWidth();
        return width >= 2 * aCompensatedTraceHalfWidth;
    }

    const double minimumWidth = aRoom.UsesGeneralShape()
                                        ? aRoom.GetSimplex().MinWidth()
                                        : aRoom.GetOctagon().MinWidth();
    if( minimumWidth < 2 * aCompensatedTraceHalfWidth )
        return false;

    if( !aEntryDoor || aEntryDoor->GetDimension() != 1
        || aCurrentDoorIsSmall )
    {
        return true;
    }

    const auto nearest = aRoom.GetSimplex().NearestBorderPointsApprox(
            aEntryMiddle, 2 );
    if( nearest.size() < 2 )
        return false;

    // Java's literal +1 is one Freerouting coordinate, not one KiCad IU.
    return nearest[1].Distance( aEntryMiddle )
           > aCompensatedTraceHalfWidth
                     + FREEROUTING_COORDINATE_UNIT_IU;
}


bool DoorEntryIsThick( const EXPANSION_ROOM& aRoom,
                       const EXPANSION_DOOR& aDoor,
                       const std::vector<FLOAT_LINE>& aSections,
                       double aCompensatedTraceHalfWidth )
{
    if( aDoor.GetDimension() != 1 || aSections.size() != 1
        || !aDoor.FirstRoom() || !aDoor.SecondRoom()
        || !aDoor.FirstRoom()->IsCompleteFreeSpace()
        || !aDoor.SecondRoom()->IsCompleteFreeSpace() )
    {
        return true;
    }

    const double minimumWidth = aRoom.UsesGeneralShape()
                                        ? aRoom.GetSimplex().MinWidth()
                                        : aRoom.GetOctagon().MinWidth();
    if( minimumWidth < 2 * aCompensatedTraceHalfWidth )
        return false;

    const FLOAT_POINT middle = aSections.front().Middle();
    const auto nearest = aRoom.GetSimplex().NearestBorderPointsApprox(
            middle, 2 );
    return nearest.size() >= 2
           && nearest[1].Distance( middle )
                      > aCompensatedTraceHalfWidth
                                + FREEROUTING_COORDINATE_UNIT_IU;
}
} // namespace DETAIL

std::optional<ROOM_PATH> MAZE_SEARCH_ENGINE_90_DEGREE::FindConnection(
        ROUTER_BOX aBounds, const std::vector<SHAPE_TREE_ENTRY>& aObstacles,
        int aLayer, int aNet, const std::vector<ROOM_TERMINAL>& aStarts,
        const std::vector<ROOM_TERMINAL>& aTargets, double aSectionOffset,
        double aHorizontalCost, double aVerticalCost, int aMaxExpanded,
        int& aExpanded, ROOM_SEARCH_METRICS& aMetrics,
        const ROUTER_CANCEL_CALLBACK& aCancel, const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress,
        bool aOrthogonal, double aBendCost,
        const std::vector<ROOM_RIPUP_OBSTACLE>& aRipupObstacles,
        bool aSourceTraceRooms )
{
    if( aStarts.empty() || aTargets.empty() || INT_BOX::Dimension( aBounds ) != 2
        || !std::isfinite( aSectionOffset ) || aSectionOffset <= 0
        || !std::isfinite( aHorizontalCost ) || aHorizontalCost <= 0
        || !std::isfinite( aVerticalCost ) || aVerticalCost <= 0
        || !std::isfinite( aBendCost ) || aBendCost < 0 )
        return std::nullopt;
    ROUTER_BOX costBounds = aBounds;
    for( const auto& terminals : { &aStarts, &aTargets } )
        for( const auto& terminal : *terminals )
        {
            costBounds = INT_BOX::Union( costBounds, {
                    std::min( terminal.start.x, terminal.end.x ), std::min( terminal.start.y, terminal.end.y ),
                    std::max( terminal.start.x, terminal.end.x ), std::max( terminal.start.y, terminal.end.y ) } );
        }
    const ROOM_COST_SPACE costSpace( costBounds );
    // This subproblem is strictly single-layer. Map its sole physical layer
    // to source ordinal zero, not the arbitrary KiCad layer ID.
    DESTINATION_DISTANCE destinationDistance( { { aHorizontalCost, aVerticalCost } }, { true }, 0, 0 );
    for( const auto& target : aTargets )
    {
        const ROUTER_BOX targetBounds = INT_BOX::Dimension( target.treeBounds ) >= 0
                ? target.treeBounds
                : ROUTER_BOX{ std::min( target.start.x, target.end.x ),
                              std::min( target.start.y, target.end.y ),
                              std::max( target.start.x, target.end.x ),
                              std::max( target.start.y, target.end.y ) };
        destinationDistance.Join( costSpace.ToReference( targetBounds ), 0 );
    }
    ROOM_SEARCH search( aBounds, aObstacles, aLayer, aNet, aSectionOffset,
                        aMaxExpanded, aExpanded, aMetrics, aCancel, aProgress, nullptr,
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
        std::optional<std::size_t> rippedGroup;
        bool roomRipped = false;
        MAZE_ADJUSTMENT adjustment = MAZE_ADJUSTMENT::NONE;
        bool alreadyChecked = false;
    };
    constexpr auto NONE = std::numeric_limits<std::size_t>::max();
    std::deque<STATE> states;
    // Reference TreeSet order: f, g, door ID, section. Equal keys keep the
    // first entry, rather than using insertion sequence as another tie-break.
    std::map<decltype( MAZE_LIST_ELEMENT{}.SortKey() ), std::size_t> open;
    std::set<std::pair<EXPANSION_DOOR*, std::size_t>> occupied;
    auto cost = [&]( FLOAT_POINT from, FLOAT_POINT to )
    {
        // FloatPoint.weightedDistance is weighted Euclidean, NOT Manhattan.
        return from.WeightedDistance( to, aHorizontalCost, aVerticalCost );
    };
    auto distance = [&]( FLOAT_POINT from )
    {
        ++aMetrics.destinationQueries;
        return costSpace.ToNativeCost( destinationDistance.Calculate( costSpace.ToReference( from ), 0 ) );
    };
    auto startAttachment = [&]( const ROOM_TERMINAL& aStart, const ROUTER_BOX& aRoom )
            -> std::optional<ROUTER_POINT>
    {
        std::optional<ROUTER_POINT> best;
        double bestDistance = std::numeric_limits<double>::infinity();
        for( const ROOM_TERMINAL& target : aTargets )
        {
            for( const ROUTER_POINT& toward : { target.start, target.end } )
            {
                const auto candidate = nearestInRoom( aStart, toward, aRoom );
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
    auto push = [&]( const STATE& state )
    {
        const auto roomId = state.room ? state.room->shape->GetId()
                                      : states[state.parent].room->shape->GetId();
        // Distinct physical terminals can share an electrical owner/pad index.
        // Give them separate stable IDs so queue deduplication cannot drop one.
        const std::uint32_t itemId = state.targetItemId;
        const int id = state.door ? state.door->GetId()
                                 : static_cast<std::int32_t>( 31u * itemId + roomId );
        const MAZE_LIST_ELEMENT key{ state.g, state.f, id, state.section };
        if( !open.emplace( key.SortKey(), states.size() ).second )
            return false;

        states.push_back( state );
        return true;
    };
    for( std::size_t startIndex = 0; startIndex < aStarts.size(); ++startIndex )
    {
        const auto& start = aStarts[startIndex];
        const ROUTER_BOX contained{ std::min( start.start.x, start.end.x ),
                                   std::min( start.start.y, start.end.y ),
                                   std::max( start.start.x, start.end.x ),
                                   std::max( start.start.y, start.end.y ) };

        if( start.start.x == start.end.x || start.start.y == start.end.y )
        {
            // A point or axis-aligned trace is itself an orthogonal contained
            // shape, so preserve the direct source-shaped completion path.
            search.complete( search.incomplete( { aBounds, aLayer, contained } ) );
        }
        else
        {
            // A diagonal trace's AABB contains two wedges which are not
            // copper.  Seed exact lattice points immediately around every
            // orthogonal obstacle cut instead.  This reaches every room
            // touched by the real centre-line with bounded work.
            std::vector<ROUTER_BOX> cuts;
            cuts.reserve( aObstacles.size() + 1 );
            cuts.push_back( aBounds );
            for( const SHAPE_TREE_ENTRY& obstacle : aObstacles )
                if( obstacle.layer == aLayer && obstacle.IsTraceObstacle( aNet ) )
                    cuts.push_back( obstacle.shape );

            for( const ROUTER_POINT& seedPoint :
                 TARGET_ITEM_EXPANSION_DOOR::IntegralRoomSeedPoints(
                         start.start, start.end, cuts ) )
            {
                const ROUTER_BOX seed{ seedPoint.x, seedPoint.y,
                                       seedPoint.x, seedPoint.y };
                const bool covered = std::any_of(
                        search.byId.begin(), search.byId.end(),
                        [&]( const auto& entry )
                        {
                            const ROOM* room = entry.second;
                            return !room->shape->IsObstacle()
                                   && room->shape->GetShape().Contains( seedPoint );
                        } );
                if( !covered )
                    search.complete( search.incomplete( { aBounds, aLayer, seed } ) );
            }
        }

        // Preserve every electrical source in every free room touched by its
        // exact shape, including rooms made by an earlier source seed.
        for( const auto& [id, room] : search.byId )
        {
            if( room->shape->IsObstacle() )
                continue;
            const auto attachment = startAttachment( start, room->shape->GetShape() );
            if( !attachment )
                continue;
            const FLOAT_POINT point{ static_cast<double>( attachment->x ),
                                     static_cast<double>( attachment->y ) };
            push( { room, nullptr, 0, { point, point }, 0, distance( point ), NONE,
                    start.owner, {}, static_cast<std::uint32_t>( startIndex + 1 ),
                    0, {}, false, MAZE_ADJUSTMENT::NONE, false } );
        }
    }

    while( !open.empty() && search.step() )
    {
        const auto index = open.begin()->second;
        open.erase( open.begin() );
        const STATE current = states[index];
        if( current.target )
        {
            ROOM_PATH path{ {}, current.owner, *current.target, {}, 0 };
            std::vector<std::size_t> entries;
            for( auto i = index; i != NONE; i = states[i].parent )
                entries.push_back( i );
            std::reverse( entries.begin(), entries.end() );
            std::set<std::size_t> rippedGroups;
            for( std::size_t entry : entries )
            {
                if( states[entry].rippedGroup && states[entry].ripupCost > 0 )
                {
                    rippedGroups.insert( *states[entry].rippedGroup );
                    path.ripupCost += states[entry].ripupCost;
                }
            }
            path.rippedObstacleGroups.assign( rippedGroups.begin(), rippedGroups.end() );
            std::optional<std::vector<ROUTER_POINT>> located;
            if( aSourceTraceRooms )
            {
                std::vector<OCTAGONAL_CORRIDOR_STEP> corridor;
                for( std::size_t i = 1; i < entries.size(); ++i )
                {
                    const auto& from = states[entries[i - 1]];
                    const auto& to = states[entries[i]];
                    const auto* obstacle = dynamic_cast<const OBSTACLE_EXPANSION_ROOM*>(
                            from.room ? from.room->shape.get() : nullptr );
                    const bool obstacleRipped = obstacle
                            && ( from.roomRipped
                                 || ( to.rippedGroup
                                      && *to.rippedGroup == obstacle->GetGroup() ) );
                    corridor.push_back( {
                            from.room->shape->GetOctagon(),
                            to.door ? std::optional<PLANAR::INT_OCTAGON>(
                                              to.door->GetOctagonShape() )
                                    : std::nullopt,
                            to.entry, obstacleRipped } );
                }
                located = FOUND_CONNECTION_LOCATOR_45_DEGREE::LocateOctagonal(
                        states[entries.front()].entry.Middle().Round(), corridor,
                        aSectionOffset, FREEROUTING_TRACE_WIDTH_TOLERANCE_IU,
                        aOrthogonal );
            }
            else
            {
                std::vector<RECTANGULAR_CORRIDOR_STEP> corridor;
                for( std::size_t i = 1; i < entries.size(); ++i )
                {
                    const auto& from = states[entries[i - 1]];
                    const auto& to = states[entries[i]];
                    corridor.push_back( { from.room->shape->GetShape(),
                            to.door ? std::optional<ROUTER_BOX>( to.door->GetShape() )
                                    : std::nullopt,
                            to.entry } );
                }
                located = FOUND_CONNECTION_LOCATOR_45_DEGREE::LocateRectangular(
                        states[entries.front()].entry.Middle().Round(), corridor,
                        aOrthogonal );
            }
            if( !located )
            {
                if( autorouterDebugEnabled() )
                    autorouterDebugLog( "ROOM_LOCATOR_REJECTED states="
                                        + std::to_string( entries.size() )
                                        + " corridors="
                                        + std::to_string( entries.size() - 1 ) );
                continue;
            }
            path.points = *located;
            aMetrics.rippedRooms += static_cast<int>( path.rippedObstacleGroups.size() );
            aMetrics.ripupCost += path.ripupCost;
            aMetrics.routed = true;
            return path;
        }
        // MazeSearchEngine.occupyNextElement() delays occupation until
        // expandToRoomDoors() reports that the entry produced work.  Thin or
        // small entries which expand nothing must remain retryable.
        if( current.door && occupied.contains( { current.door, current.section } ) )
            continue;
        ++aMetrics.sections;
        search.completeNeighbours( current.room );
        if( search.stopped() )
            return std::nullopt;

        const bool currentDoorIsSmall = current.door
                && current.door->IsSmallFor90DegreeTrace(
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
        const auto from = current.entry.Middle();
        const bool nextRoomIsThick = DETAIL::RoomIsThick(
                *current.room->shape, aSectionOffset, current.door, from,
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

            ROOM* next = search.byShape.at(
                    aDoor->OtherRoom( current.room->shape.get() ) );
            if( !next->complete || !next->active )
                return false;

            const FLOAT_POINT to = aShapeEntry.Middle();
            double bend = 0;
            if( current.parent != NONE )
            {
                const STATE& previous = states[current.parent];
                FLOAT_POINT centre = previous.entry.Middle();
                if( previous.door )
                {
                    const ROUTER_BOX shape = previous.door->GetShape();
                    centre = { ( static_cast<double>( shape.minX ) + shape.maxX ) / 2,
                               ( static_cast<double>( shape.minY ) + shape.maxY ) / 2 };
                }
                bend = MAZE_LIST_ELEMENT::BendPenalty(
                        centre, from, to, aBendCost );
            }

            const double g = current.g + cost( from, to ) + bend + aAddCost;
            const bool roomRipped =
                    ( aAddCost > 0 && aAdjustment == MAZE_ADJUSTMENT::NONE )
                    || ( current.alreadyChecked && current.roomRipped );
            const std::optional<std::size_t> rippedGroup =
                    aAddCost > 0 && aAdjustment == MAZE_ADJUSTMENT::NONE
                            && currentObstacle
                    ? std::optional<std::size_t>( currentObstacle->GetGroup() )
                    : std::nullopt;
            const ROUTER_BOX doorBounds = aDoor->GetShape();
            const ROUTER_BOX fromDoorBounds = current.door
                                                    ? current.door->GetShape()
                                                    : ROUTER_BOX{};
            autorouterDecisionLog(
                    "RAW_SECTION_ASSIGN",
                    { { "net", std::to_string( aNet ) },
                      { "layer", "0" },
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
                      { "expansion_value", std::to_string( g ) },
                      { "sorting_value", std::to_string( g + distance( to ) ) } } );
            return push( { next, aDoor, aSection, aShapeEntry, g,
                           g + distance( to ), index, current.owner, {}, 0,
                           aAddCost, rippedGroup, roomRipped, aAdjustment, false } );
        };
        for( std::size_t targetIndex = 0; targetIndex < aTargets.size(); ++targetIndex )
        {
            const auto& target = aTargets[targetIndex];
            const auto p = nearestInRoom( target, from.Round(),
                                          current.room->shape->GetShape() );
            if( !p )
                continue;
            const FLOAT_POINT to{ static_cast<double>( p->x ), static_cast<double>( p->y ) };
            const double g = current.g + cost( from, to );
            somethingExpanded = push( { nullptr, nullptr, 0, { to, to }, g, g,
                    index, current.owner, target.owner,
                    static_cast<std::uint32_t>( aStarts.size() + targetIndex + 1 ),
                    0, {}, current.roomRipped, MAZE_ADJUSTMENT::NONE, false } )
                    || somethingExpanded;
        }

        // In source-tree geometry, a matching PolylineTrace is first tried as
        // a zero-cost shove corridor.  Only failure from an outer section
        // requeues this exact state with the paid rip-up cost.
        if( aSourceTraceRooms && currentObstacle && current.door
            && !current.alreadyChecked && currentRoomRipupCost != 1
            && nextRoomIsThick && !currentDoorIsSmall
            && currentObstacle->GetTraceInfo() )
        {
            const auto fromSections = current.door->GetSectionSegments(
                    aSectionOffset, FREEROUTING_TRACE_WIDTH_TOLERANCE_IU );
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
                            *currentObstacle, aSectionOffset, false, doors,
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
                            *currentObstacle, aSectionOffset, true, doors,
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

        for( auto* door : current.room->shape->GetDoors() )
        {
            if( door == current.door )
                continue;

            ROOM* next = search.byShape.at( door->OtherRoom( current.room->shape.get() ) );
            if( !next->complete || !next->active )
                continue;
            const auto sections = door->GetSectionSegments(
                    aSourceTraceRooms ? aSectionOffset : 0,
                    aSourceTraceRooms ? FREEROUTING_TRACE_WIDTH_TOLERANCE_IU : 0,
                    aSourceTraceRooms ? 0 : 10 * aSectionOffset,
                    static_cast<std::size_t>( std::max( 0, aMaxExpanded - aExpanded ) ) );
            if( nextRoomIsThick
                && !DETAIL::DoorEntryIsThick(
                        *current.room->shape, *door, sections,
                        aSectionOffset ) )
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

        if( current.door && somethingExpanded )
            occupied.emplace( current.door, current.section );
    }
    if( autorouterDebugEnabled() )
    {
        std::ostringstream message;
        message << "ROOM_FRONTIER_EXHAUSTED layer=" << aLayer << " rooms=";
        for( const auto& room : search.rooms )
        {
            const auto& box = room->shape->GetShape();
            message << " {id=" << room->shape->GetId() << ",box=(" << box.minX << ','
                    << box.minY << ',' << box.maxX << ',' << box.maxY << "),obstacle="
                    << room->shape->IsObstacle() << ",complete=" << room->complete
                    << ",active=" << room->active << ",doors="
                    << room->shape->GetDoors().size() << '}';
        }
        autorouterDebugLog( message.str() );
    }
    return std::nullopt;
}
} // namespace KICAD_AUTOROUTER
