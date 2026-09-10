/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Native routing-board boundary corresponding to Freerouting board/facade/RoutingBoard.
 */
#pragma once

#include <map>
#include <limits>
#include <memory>
#include <optional>
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
    using ITEM_ID_SET = std::set<ITEM_ID, std::greater<ITEM_ID>>;

    enum class ITEM_KIND
    {
        UNKNOWN,
        TRACE,
        DRILL,
        AREA
    };

    /** Read-only source-item view used by Connection.get(). */
    struct ITEM_INFO
    {
        ITEM_ID                  id = 0;
        int                      netCode = 0;
        std::size_t              padIndex = std::numeric_limits<std::size_t>::max();
        ITEM_KIND                kind = ITEM_KIND::UNKNOWN;
        bool                     routable = false;
        ROUTER_POINT             first;
        ROUTER_POINT             last;
        std::vector<int>         layers;
        double                   traceLength = 0.0;
    };

    ROUTING_BOARD( const BOARD_SNAPSHOT& aSnapshot, const AUTOROUTER_SETTINGS& aSettings );
    ~ROUTING_BOARD();
    ROUTING_BOARD( const ROUTING_BOARD& ) = delete;
    ROUTING_BOARD& operator=( const ROUTING_BOARD& ) = delete;

    void AddRoute( const ROUTING_CONNECTION& aRoute );
    void RemoveRoute( const ROUTING_CONNECTION& aRoute );
    void ClearRoutes();
    /** Move a planning-only terminal to the drill selected by dynamic fanout. */
    void RelocateSyntheticPad( std::size_t aPad, ROUTER_POINT aPosition );

    bool Connected( std::size_t aFirstPad, std::size_t aSecondPad ) const;
    /** True when the pad's physical same-net component reaches another copper layer. */
    bool ConnectedSetTouchesOtherLayer( std::size_t aPad, int aLayer ) const;
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
    ITEM_ID_SET GetNormalContacts( ITEM_ID aItem ) const;
    std::optional<ROUTER_POINT> NormalContactPoint( ITEM_ID aFirst, ITEM_ID aSecond ) const;
    int FirstCommonLayer( ITEM_ID aFirst, ITEM_ID aSecond ) const;
    ITEM_ID_SET NormalContactsAt( ITEM_ID aTrace, ROUTER_POINT aPoint ) const;
    ITEM_ID_SET NormalConnectedSet( ITEM_ID aItem ) const;
    /** Direct translation of Item.getConnectionItems(StopConnectionOption.NONE).
     * Returns routable trace/drill items up to the next terminal, stub or fork.
     */
    ITEM_ID_SET GetConnectionItems( ITEM_ID aItem ) const;
    /** Exact conduction-area identities containing a same-net point. */
    ITEM_ID_SET ConductionAreaContactsAt( int aNetCode, ROUTER_NODE aPoint ) const;
    std::optional<ITEM_INFO> GetItemInfo( ITEM_ID aItem ) const;
    /** Exact standalone geometry and manufacturing style for a mutable item. */
    std::optional<ROUTING_CONNECTION> ItemRoute( ITEM_ID aItem ) const;
    /** Current mutable item geometry in source insertion-ID order. */
    std::vector<ROUTING_CONNECTION> ItemRoutes() const;
    /** Remove only the supplied mutable items. Fixed/unknown IDs reject the
     * complete operation; callers use TRANSACTION for speculative edits.
     */
    bool RemoveItems( const ITEM_ID_SET& aItems );
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
