/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Native worker counterpart of Freerouting board/optimize/TraceTightener.java
 * at a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include <vector>

#include "../state/ChangedArea.h"


namespace KICAD_AUTOROUTER
{

class ROUTING_OCCUPANCY;

/** Pulls mutable worker traces/vias tight only around explicitly marked edits.
 *
 * The source mutates PolylineTrace and Via items in its ShapeSearchTree.  The
 * native board stores equivalent item geometry behind ROUTING_OCCUPANCY, so
 * each accepted local replacement is an atomic occupancy/normal-contact
 * transaction and is checked by the same strict insertion predicates as a
 * routed connection.
 */
class TRACE_TIGHTENER
{
public:
    TRACE_TIGHTENER( const BOARD_SNAPSHOT& aBoard,
                     const AUTOROUTER_SETTINGS& aSettings,
                     ROUTING_OCCUPANCY& aOccupancy ) :
            m_board( aBoard ), m_settings( aSettings ), m_occupancy( aOccupancy )
    {
    }

    /** Source-shaped fixed point over all changed layers.
     * A positive net code restricts the operation to that net.  A zero time
     * limit means no deadline, matching TraceTightener's constructor.
     */
    bool OptChangedArea( CHANGED_AREA& aChangedArea,
                         std::vector<ROUTING_CONNECTION>& aConnections,
                         int aOnlyNetCode,
                         const ROUTER_CANCEL_CALLBACK& aCancel,
                         int aTimeLimitMilliseconds = 0 ) const;

    /** Mark the physical trace/via shapes of one inserted or removed route. */
    static void MarkConnection( CHANGED_AREA& aChangedArea,
                                const ROUTING_CONNECTION& aConnection,
                                const BOARD_SNAPSHOT& aBoard,
                                const AUTOROUTER_SETTINGS& aSettings );

    /** Number of layer slots needed by ChangedArea for this snapshot. */
    static int LayerCount( const BOARD_SNAPSHOT& aBoard,
                           const AUTOROUTER_SETTINGS& aSettings );

private:
    const BOARD_SNAPSHOT&      m_board;
    const AUTOROUTER_SETTINGS& m_settings;
    ROUTING_OCCUPANCY&         m_occupancy;
};

} // namespace KICAD_AUTOROUTER
