/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Derived from Freerouting a11c0a42 (GPL-3.0): AutorouteEngine's free-room
 * lifecycle, SortedOrthogonalRoomNeighbours, ExpansionDoor, MazeSearchEngine.
 *
 * Host adaptation: inputs are trace-CENTRE free rectangles (full clearance and
 * radius already applied), not upstream's half-clearance compensated shapes.
 * Consequently the locator does not shrink them a second time. It emits a
 * 90/45-degree polyline through the backtracked corridor inside convex rooms.
 * The full upstream locator, thin-room/shove logic and layer/drill frontier
 * remain separate work; this class is not a claim of complete engine parity.
 */
#include "MazeSearchEngine90Degree.h"
#include "MazeListElement.h"
#include "RoomSearchContext.h"

#include <deque>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <unordered_map>
#include "../expansion/CompleteFreeSpaceExpansionRoom.h"
#include "../expansion/ExpansionDoor.h"
#include "../expansion/SortedOrthogonalRoomNeighbours.h"
#include "../path/FoundConnectionLocator45Degree.h"

namespace KICAD_AUTOROUTER
{
namespace
{
ROUTER_POINT nearest( const ROOM_TERMINAL& terminal, ROUTER_POINT point )
{
    // The host splits diagonal terminal traces into endpoint seeds. All
    // terminals in this rectangular slice are points or axis-aligned lines.
    return { std::clamp( point.x, std::min( terminal.start.x, terminal.end.x ),
                                  std::max( terminal.start.x, terminal.end.x ) ),
             std::clamp( point.y, std::min( terminal.start.y, terminal.end.y ),
                                  std::max( terminal.start.y, terminal.end.y ) ) };
}

using DETAIL::ROOM;
using DETAIL::ROOM_SEARCH;
} // namespace

std::optional<ROOM_PATH> MAZE_SEARCH_ENGINE_90_DEGREE::FindConnection(
        ROUTER_BOX aBounds, const std::vector<SHAPE_TREE_ENTRY>& aObstacles,
        int aLayer, int aNet, const std::vector<ROOM_TERMINAL>& aStarts,
        const std::vector<ROOM_TERMINAL>& aTargets, double aSectionOffset,
        double aHorizontalCost, double aVerticalCost, int aMaxExpanded,
        int& aExpanded, ROOM_SEARCH_METRICS& aMetrics,
        const ROUTER_CANCEL_CALLBACK& aCancel, const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress,
        bool aOrthogonal, double aBendCost )
{
    if( aStarts.empty() || aTargets.empty() || INT_BOX::Dimension( aBounds ) != 2
        || !std::isfinite( aSectionOffset ) || aSectionOffset <= 0
        || !std::isfinite( aHorizontalCost ) || aHorizontalCost <= 0
        || !std::isfinite( aVerticalCost ) || aVerticalCost <= 0
        || !std::isfinite( aBendCost ) || aBendCost < 0 )
        return std::nullopt;
    for( const auto& terminals : { &aStarts, &aTargets } )
        for( const auto& terminal : *terminals )
            if( terminal.start.x != terminal.end.x && terminal.start.y != terminal.end.y )
                return std::nullopt; // A diagonal line is not its enclosing rectangle.
    ROOM_SEARCH search( aBounds, aObstacles, aLayer, aNet, aSectionOffset,
                        aMaxExpanded, aExpanded, aMetrics, aCancel, aProgress );
    struct STATE
    {
        ROOM* room;
        EXPANSION_DOOR* door;
        std::size_t section;
        FLOAT_LINE entry;
        double g;
        double f;
        std::size_t parent;
        std::size_t owner;
        std::optional<std::size_t> target;
        std::uint32_t targetItemId = 0;
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
        double best = std::numeric_limits<double>::infinity();
        for( const auto& target : aTargets )
        {
            const auto p = nearest( target, from.Round() );
            best = std::min( best, cost( from, { static_cast<double>( p.x ), static_cast<double>( p.y ) } ) );
        }
        return best;
    };
    auto push = [&]( STATE state )
    {
        const auto roomId = state.room ? state.room->shape->GetId()
                                      : states[state.parent].room->shape->GetId();
        // Distinct physical terminals can share an electrical owner/pad index.
        // Give them separate stable IDs so queue deduplication cannot drop one.
        const std::uint32_t itemId = state.targetItemId;
        const int id = state.door ? state.door->GetId()
                                 : static_cast<std::int32_t>( 31u * itemId + roomId );
        const MAZE_LIST_ELEMENT key{ state.g, state.f, id, state.section };
        if( open.emplace( key.SortKey(), states.size() ).second )
            states.push_back( state );
    };
    for( std::size_t startIndex = 0; startIndex < aStarts.size(); ++startIndex )
    {
        const auto& start = aStarts[startIndex];
        const ROUTER_BOX contained{ std::min( start.start.x, start.end.x ),
                                   std::min( start.start.y, start.end.y ),
                                   std::max( start.start.x, start.end.x ),
                                   std::max( start.start.y, start.end.y ) };
        const auto startRooms = search.complete( search.incomplete( { aBounds, aLayer, contained } ) );
        for( ROOM* room : startRooms )
        {
            const auto attachment = INT_BOX::Intersection( contained, room->shape->GetShape() );
            if( INT_BOX::Dimension( attachment ) < 0 )
                continue;
            const FLOAT_POINT p{ ( static_cast<double>( attachment.minX ) + attachment.maxX ) / 2,
                                 ( static_cast<double>( attachment.minY ) + attachment.maxY ) / 2 };
            push( { room, nullptr, 0, { p, p }, 0, distance( p ), NONE, start.owner, {},
                    static_cast<std::uint32_t>( startIndex + 1 ) } );
        }
        // A previous seed may have already made this same free room. Preserve
        // every electrical source instead of dropping those covered by it.
        for( const auto& [id, room] : search.byId )
        {
            const auto b = INT_BOX::Intersection( contained, room->shape->GetShape() );
            if( INT_BOX::Dimension( b ) >= 0
                && std::find( startRooms.begin(), startRooms.end(), room ) == startRooms.end() )
            {
                const FLOAT_POINT p{ ( static_cast<double>( b.minX ) + b.maxX ) / 2,
                                     ( static_cast<double>( b.minY ) + b.maxY ) / 2 };
                push( { room, nullptr, 0, { p, p }, 0, distance( p ), NONE, start.owner, {},
                        static_cast<std::uint32_t>( startIndex + 1 ) } );
            }
        }
    }

    while( !open.empty() && search.step() )
    {
        const auto index = open.begin()->second;
        open.erase( open.begin() );
        const STATE current = states[index];
        if( current.target )
        {
            ROOM_PATH path{ {}, current.owner, *current.target };
            std::vector<std::size_t> entries;
            for( auto i = index; i != NONE; i = states[i].parent )
                entries.push_back( i );
            std::reverse( entries.begin(), entries.end() );
            std::vector<RECTANGULAR_CORRIDOR_STEP> corridor;
            for( std::size_t i = 1; i < entries.size(); ++i )
            {
                const auto& from = states[entries[i - 1]];
                const auto& to = states[entries[i]];
                corridor.push_back( { from.room->shape->GetShape(),
                        to.door ? std::optional<ROUTER_BOX>( to.door->GetShape() ) : std::nullopt,
                        to.entry } );
            }
            const auto located = FOUND_CONNECTION_LOCATOR_45_DEGREE::LocateRectangular(
                    states[entries.front()].entry.Middle().Round(), corridor, aOrthogonal );
            if( !located )
                continue;
            path.points = *located;
            aMetrics.routed = true;
            return path;
        }
        if( current.door && !occupied.emplace( current.door, current.section ).second )
            continue;
        ++aMetrics.sections;
        search.completeNeighbours( current.room );
        if( search.stopped() )
            return std::nullopt;
        const auto from = current.entry.Middle();
        for( std::size_t targetIndex = 0; targetIndex < aTargets.size(); ++targetIndex )
        {
            const auto& target = aTargets[targetIndex];
            const auto p = nearest( target, from.Round() );
            if( !current.room->shape->Contains( p ) )
                continue;
            const FLOAT_POINT to{ static_cast<double>( p.x ), static_cast<double>( p.y ) };
            const double g = current.g + cost( from, to );
            push( { nullptr, nullptr, 0, { to, to }, g, g, index, current.owner, target.owner,
                    static_cast<std::uint32_t>( aStarts.size() + targetIndex + 1 ) } );
        }
        for( auto* door : current.room->shape->GetDoors() )
        {
            ROOM* next = search.byShape.at( door->OtherRoom( current.room->shape.get() ) );
            if( !next->complete || !next->active )
                continue;
            // Centre-space rectangles have no remaining geometric radius to
            // subtract. Section width remains trace-scaled; the midpoint stays
            // in both rooms even when a narrow door shrinks to its centre.
            const auto sections = door->GetSectionSegments( 0, 0, 10 * aSectionOffset,
                    static_cast<std::size_t>( std::max( 0, aMaxExpanded - aExpanded ) ) );
            for( std::size_t section = 0; section < sections.size(); ++section )
            {
                if( occupied.contains( { door, section } ) )
                    continue;
                const auto to = sections[section].Middle();
                double bend = 0;
                if( current.parent != NONE )
                {
                    const auto& previous = states[current.parent];
                    auto centre = previous.entry.Middle();
                    if( previous.door )
                    {
                        const auto b = previous.door->GetShape();
                        centre = { ( static_cast<double>( b.minX ) + b.maxX ) / 2,
                                   ( static_cast<double>( b.minY ) + b.maxY ) / 2 };
                    }
                    bend = MAZE_LIST_ELEMENT::BendPenalty( centre, from, to, aBendCost );
                }
                const double g = current.g + cost( from, to ) + bend;
                push( { next, door, section, sections[section], g, g + distance( to ),
                        index, current.owner, {} } );
            }
        }
    }
    return std::nullopt;
}
} // namespace KICAD_AUTOROUTER
