/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Native routing-board boundary corresponding to Freerouting board/facade/RoutingBoard.
 */
#pragma once

#include <map>
#include <memory>
#include <set>
#include <vector>

#include "../../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{

/** Private worker-owned copper, contacts and reversible edits. Never owns a KiCad BOARD. */
class ROUTING_BOARD
{
public:
    using ITEM_ID = std::uint64_t;

    ROUTING_BOARD( const BOARD_SNAPSHOT& aSnapshot, const AUTOROUTER_SETTINGS& aSettings );
    ~ROUTING_BOARD();
    ROUTING_BOARD( const ROUTING_BOARD& ) = delete;
    ROUTING_BOARD& operator=( const ROUTING_BOARD& ) = delete;

    void AddRoute( const ROUTING_CONNECTION& aRoute );
    void RemoveRoute( const ROUTING_CONNECTION& aRoute );
    void ClearRoutes();

    bool Connected( std::size_t aFirstPad, std::size_t aSecondPad ) const;
    std::vector<std::vector<std::size_t>> ConnectedPadGroups( int aNetCode ) const;
    int CountMissing( const ROUTING_NET& aNet ) const;
    std::vector<ROUTING_TERMINAL> Terminals( std::size_t aPad ) const;
    /** Exact same-net copper contact, including real pads and filled areas. */
    bool HasCopperAt( int aNet, ROUTER_NODE aNode, std::int64_t aRadius = 0 ) const;
    /** Centre-line junctions in start-to-end order (no virtual fanout pads).
     * Rounded, off-line intersections are deliberately not used for trimming.
     */
    std::vector<ROUTER_POINT> TraceJunctions( int aNet, ROUTER_NODE aStart,
                                             ROUTER_NODE aEnd ) const;
    std::set<ITEM_ID> ConnectedSet( ITEM_ID aItem ) const;
    /** Routing topology: exact endpoint/centre contacts, not copper overlap.
     * Kept separate from ConnectedSet(), which answers host physical connectivity.
     */
    std::set<ITEM_ID> GetNormalContacts( ITEM_ID aItem ) const;
    std::optional<ROUTER_POINT> NormalContactPoint( ITEM_ID aFirst, ITEM_ID aSecond ) const;
    std::set<ITEM_ID> NormalConnectedSet( ITEM_ID aItem ) const;
    std::vector<ITEM_ID> RouteItems( const ROUTING_CONNECTION& aRoute ) const;
    std::optional<ITEM_ID> PadItem( std::size_t aPad ) const;
    std::size_t ItemCount() const;
    std::uint64_t Revision() const;

    /** Speculative copper edits are rolled back unless explicitly committed. */
    class TRANSACTION
    {
    public:
        explicit TRANSACTION( ROUTING_BOARD& aBoard );
        ~TRANSACTION();
        TRANSACTION( const TRANSACTION& ) = delete;
        TRANSACTION& operator=( const TRANSACTION& ) = delete;
        void Commit();
    private:
        struct STATE;
        ROUTING_BOARD& m_board;
        std::unique_ptr<STATE> m_before;
    };

private:
    struct IMPL;
    std::unique_ptr<IMPL> m_impl;
};

} // namespace KICAD_AUTOROUTER
