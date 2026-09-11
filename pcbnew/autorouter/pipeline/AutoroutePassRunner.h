/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Batch item selection translated from Freerouting a11c0a42
 * autoroute/pipeline/AutoroutePassRunner.java and BatchAutorouter.java.
 */
#pragma once

#include "../AutorouterTypes.h"
#include "../board/facade/RoutingBoard.h"
#include "TaskState.h"

#include <algorithm>
#include <map>
#include <set>
#include <vector>

namespace KICAD_AUTOROUTER
{

struct AUTOROUTE_ITEM
{
    int         netCode = 0;
    std::size_t pad = std::numeric_limits<std::size_t>::max();
};


/** Source-order item selection for one routing pass.
 *
 * Freerouting snapshots the pass's to-do list before routing it and marks the
 * complete connected set of each encountered non-routable item as handled.
 * A native pad is the stable representative of that item set.  Rebuilding
 * this list before every pass is important: rip-up and successful insertion
 * change the connected components which the following pass must observe.
 */
class AUTOROUTE_PASS_RUNNER
{
public:
    void Begin() { m_state = TASK_STATE::RUNNING; }
    void BeginOptimization() { m_state = TASK_STATE::OPTIMIZING; }
    void Complete() { m_state = TASK_STATE::COMPLETED; }
    void Cancel() { m_state = TASK_STATE::CANCELLED; }
    void Fail() { m_state = TASK_STATE::FAILED; }
    TASK_STATE State() const { return m_state; }

    static std::vector<AUTOROUTE_ITEM> GetAutorouteItems(
            const BOARD_SNAPSHOT& aBoard, const ROUTING_BOARD& aRoutingBoard )
    {
        std::map<int, const ROUTING_NET*> nets;
        std::map<int, std::vector<std::vector<std::size_t>>> connectedGroups;
        std::set<int> exactRepairNets;
        for( const ROUTING_PAD& pad : aBoard.pads )
            if( pad.isExactTarget )
                exactRepairNets.insert( pad.netCode );
        for( const ROUTING_NET& net : aBoard.nets )
        {
            nets.emplace( net.netCode, &net );
            connectedGroups.emplace( net.netCode,
                                     aRoutingBoard.ConnectedPadGroups( net.netCode ) );
        }

        std::set<std::size_t> handledPads;
        std::vector<AUTOROUTE_ITEM> result;
        result.reserve( aBoard.pads.size() );

        // Item.compareTo() sorts Freerouting board items by descending
        // insertion id.  The adapter stores real pads in their DSN creation
        // order, so reverse traversal is the exact natural pass order.  The
        // synthetic terminals appended after real pads are not source Items.
        for( std::size_t next = aBoard.pads.size(); next > 0; --next )
        {
            const std::size_t padIndex = next - 1;
            const ROUTING_PAD& pad = aBoard.pads[padIndex];
            if( pad.isFanoutTarget || pad.isPlaneTarget || handledPads.contains( padIndex ) )
                continue;

            const auto netIt = nets.find( pad.netCode );
            if( netIt == nets.end() || netIt->second->connections.empty() )
                continue;

            handledPads.insert( padIndex );
            for( const auto& group : connectedGroups.at( pad.netCode ) )
            {
                if( std::find( group.begin(), group.end(), padIndex ) == group.end() )
                    continue;

                for( std::size_t member : group )
                {
                    // Exact refill terminals are synthetic search anchors,
                    // not source board Items. A real pad on the same island
                    // must not consume them from the repair pass's work list.
                    if( member >= aBoard.pads.size()
                        || !aBoard.pads[member].isExactTarget )
                    {
                        handledPads.insert( member );
                    }
                }
                break;
            }

            if( aRoutingBoard.CountMissing( *netIt->second ) == 0 )
                continue;

            // For a plane net, getAutorouteItems skips an item whose connected
            // set already reaches a ConductionArea.  Synthetic plane terminals
            // are the native representation of those areas.
            const bool connectedToPlane = std::any_of(
                    netIt->second->planeTargetIndices.begin(),
                    netIt->second->planeTargetIndices.end(),
                    [&]( std::size_t target )
                    {
                        return target < aBoard.pads.size()
                               && aRoutingBoard.Connected( padIndex, target );
                    } );
            // Exact refill terminals identify two disconnected islands, but
            // remain synthetic targets rather than source board items. Their
            // presence disables the normal plane false-work suppression so a
            // real pad representative can bridge those islands in KiCad's
            // stricter post-refill repair pass.
            if( !connectedToPlane || exactRepairNets.contains( pad.netCode ) )
                result.push_back( { pad.netCode, padIndex } );
        }

        return result;
    }

private:
    TASK_STATE m_state = TASK_STATE::IDLE;
};

} // namespace KICAD_AUTOROUTER
