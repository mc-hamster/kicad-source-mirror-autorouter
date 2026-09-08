/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * This program source code file is part of KiCad, a free EDA application.
 */

#include "BatchOptimizer.h"

#include <algorithm>

#include "../maze/MazeTraceShover.h"


namespace KICAD_AUTOROUTER
{

void BATCH_OPTIMIZER::simplifyConnection( ROUTING_CONNECTION& aConnection,
                                          const MAZE_SEARCH_ENGINE& aSearch ) const
{
    MAZE_TRACE_SHOVER::Shorten( aConnection, aSearch );
}


void BATCH_OPTIMIZER::RemoveRedundantViaTails( std::vector<ROUTING_CONNECTION>& aConnections,
                                             const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    if( !m_occupancy.Board() )
        return;

    for( auto& connection : aConnections )
    {
        // Terminal via transitions may become redundant after a later trace
        // attaches on the source layer. Remove only if every real-pad/plane
        // contact component survives; synthetic fanout requests are not copper.
        auto groups = m_occupancy.Board()->ConnectedPadGroups( connection.netCode );
        for( auto& group : groups )
            std::erase_if( group, [&]( auto pad ) { return m_board.pads[pad].isFanoutTarget; } );
        // A fanout may be completely bypassed by subsequent same-layer
        // copper. Keeping its stub after deleting the via creates a dangling
        // track, so first try removing the whole redundant fanout item.
        if( connection.isFanoutConnection )
        {
            m_occupancy.Remove( connection );
            bool preservesContacts = true;
            for( const auto& group : groups )
                for( auto pad : group )
                    if( !m_occupancy.Board()->Connected( group.front(), pad ) )
                        preservesContacts = false;
            if( preservesContacts )
            {
                connection.nodes.clear();
                continue;
            }
            m_occupancy.Add( connection );
        }
        for( bool front : { true, false } )
        {
            if( aCancel && aCancel() )
                return;
            if( connection.nodes.size() < 2 )
                break;
            const auto& first = front ? connection.nodes[0]
                                      : connection.nodes[connection.nodes.size() - 2];
            const auto& second = front ? connection.nodes[1] : connection.nodes.back();
            if( first.layer == second.layer || first.point != second.point )
                continue;
            const ROUTING_CONNECTION original = connection;
            m_occupancy.Remove( original );
            connection.nodes.erase( front ? connection.nodes.begin() : connection.nodes.end() - 1 );
            m_occupancy.Add( connection );
            bool preservesContacts = true;
            for( const auto& group : groups )
                for( auto pad : group )
                    if( !m_occupancy.Board()->Connected( group.front(), pad ) )
                        preservesContacts = false;
            if( !preservesContacts )
            {
                m_occupancy.Remove( connection );
                connection = original;
                m_occupancy.Add( connection );
            }
        }
    }
    removeTraceTails( aConnections, aCancel );
    std::erase_if( aConnections, []( const auto& route ) { return route.nodes.empty(); } );
}


void BATCH_OPTIMIZER::removeTraceTails( std::vector<ROUTING_CONNECTION>& connections,
                                      const ROUTER_CANCEL_CALLBACK& cancel ) const
{
    // Freerouting removeTraceTails operates on normalized trace items, split
    // at their contacts. Native paths can still span a via/T junction, so
    // deleting the entire last segment would also delete the useful trunk.
    // Trim only toward exact centre-line junctions and retain every real
    // pad/plane component. Existing host copper is never a deletion candidate.
    bool changed;
    do
    {
        changed = false;
        for( auto& connection : connections )
        {
            if( cancel && cancel() )
                return;
            auto groups = m_occupancy.Board()->ConnectedPadGroups( connection.netCode );
            for( auto& group : groups )
                std::erase_if( group, [&]( auto pad ) { return m_board.pads[pad].isFanoutTarget; } );
            std::int64_t width = 0;
            for( const auto& net : m_board.nets )
                if( net.netCode == connection.netCode )
                    for( auto index : net.padIndices )
                        width = std::max( width, m_board.pads[index].trackWidth );
            const auto radius = ( width > 0 ? width : 150000 ) / 2;
            for( bool front : { true, false } )
                while( connection.nodes.size() > 1 )
                {
                    if( cancel && cancel() )
                        return;
                    const auto from = front ? connection.nodes.front() : connection.nodes.back();
                    const auto to = front ? connection.nodes[1] : connection.nodes[connection.nodes.size() - 2];
                    if( from.layer != to.layer )
                        break;
                    const auto original = connection;
                    m_occupancy.Remove( original );
                    const bool endHasContact = m_occupancy.Board()->HasCopperAt(
                            connection.netCode, from, radius );
                    auto junctions = m_occupancy.Board()->TraceJunctions( connection.netCode, from, to );
                    std::erase( junctions, from.point );
                    // Also collapse an overlapping end back to its first
                    // centre-line contact. A point inside another trace is
                    // electrically connected, but retaining a doubled stub
                    // past that junction still fails KiCad's dangling check.
                    if( endHasContact && junctions.empty() )
                    { m_occupancy.Add( original ); break; }
                    const auto next = junctions.empty() ? to.point : junctions.front();
                    if( next == from.point )
                    { m_occupancy.Add( original ); break; }
                    if( next == to.point )
                        connection.nodes.erase( front ? connection.nodes.begin() : connection.nodes.end() - 1 );
                    else
                        ( front ? connection.nodes.front() : connection.nodes.back() ).point = next;
                    m_occupancy.Add( connection );
                    bool preserves = true;
                    for( const auto& group : groups )
                        for( auto pad : group )
                            if( !m_occupancy.Board()->Connected( group.front(), pad ) )
                                preserves = false;
                    if( !preserves )
                    {
                        m_occupancy.Remove( connection );
                        connection = original;
                        m_occupancy.Add( original );
                        break;
                    }
                    changed = true;
                }
            if( connection.nodes.size() == 1 )
            {
                m_occupancy.Remove( connection );
                connection.nodes.clear();
            }
        }
    } while( changed ); // Strictly removes vertices/length; never grows copper.
}


int BATCH_OPTIMIZER::Optimize( std::vector<ROUTING_CONNECTION>& aConnections,
                               const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    if( !m_settings.optimizeAfterComplete )
        return 0;

    MAZE_SEARCH_ENGINE search( m_board, m_settings, m_occupancy );
    int completedPasses = 0;
    int optimizedItems = 0;

    for( int pass = 0; pass < std::max( 0, m_settings.optimizationPasses ); ++pass )
    {
        if( aCancel && aCancel() )
            break;

        for( ROUTING_CONNECTION& connection : aConnections )
        {
            if( aCancel && aCancel() )
                break;

            if( m_settings.maxOptimizationItems > 0
                && optimizedItems >= m_settings.maxOptimizationItems )
            {
                return completedPasses;
            }

            if( connection.complete )
            {
                // Shortening a trunk must not detach a branch that terminates
                // in its interior. Preserve every pre-existing pad component.
                const auto groups = m_occupancy.Board()
                        ? m_occupancy.Board()->ConnectedPadGroups( connection.netCode )
                        : std::vector<std::vector<std::size_t>>{};
                const ROUTING_CONNECTION oldConnection = connection;
                m_occupancy.Remove( oldConnection );
                simplifyConnection( connection, search );
                m_occupancy.Add( connection );
                bool preservesContacts = true;
                for( const auto& group : groups )
                    for( auto pad : group )
                        if( !m_occupancy.Board()->Connected( group.front(), pad ) )
                            preservesContacts = false;
                if( !preservesContacts )
                {
                    m_occupancy.Remove( connection );
                    connection = oldConnection;
                    m_occupancy.Add( connection );
                }
                ++optimizedItems;
            }
        }

        ++completedPasses;
    }

    return completedPasses;
}

} // namespace KICAD_AUTOROUTER
