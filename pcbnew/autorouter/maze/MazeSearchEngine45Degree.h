/* This file is part of KiCad, licensed under GPL version 3 or later.
 * Exact-octagonal 45-degree room frontier derived from Freerouting a11c0a42.
 */
#pragma once

#include "MazeSearchEngine90Degree.h"

#include <memory>

namespace KICAD_AUTOROUTER
{
/** Persistent physical ShapeSearchTree state owned by one AutorouteEngine.
 *
 * Freerouting does not rebuild its MinAreaTree for every connection.  Complete
 * expansion rooms are temporary, but inserting and removing them changes the
 * surviving binary-tree topology.  Later searches consequently observe that
 * mutation history even when the set of physical leaves is unchanged.  Keep
 * the physical leaf handles beside the tree so board-item additions/removals
 * can be synchronized without throwing that history away.
 */
struct PERSISTENT_45_DEGREE_TREE_STATE
{
    struct PHYSICAL_LEAF
    {
        SHAPE_TREE_ENTRY entry;
        MIN_AREA_TREE::HANDLE handle = MIN_AREA_TREE::NONE;
    };

    std::shared_ptr<MIN_AREA_TREE> tree = std::make_shared<MIN_AREA_TREE>();
    std::vector<PHYSICAL_LEAF> physicalLeaves;
};

class MAZE_SEARCH_ENGINE_45_DEGREE
{
public:
    static std::optional<ROOM_PATH> FindConnection(
            ROUTER_BOX aBounds, const std::vector<SHAPE_TREE_ENTRY>& aObstacles,
            int aLayer, int aNet, const std::vector<ROOM_TERMINAL>& aStarts,
            const std::vector<ROOM_TERMINAL>& aTargets, double aSectionOffset,
            double aHorizontalCost, double aVerticalCost, int aMaxExpanded,
            int& aExpanded, ROOM_SEARCH_METRICS& aMetrics,
            const ROUTER_CANCEL_CALLBACK& aCancel = {},
            const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress = {},
            double aBendCost = 0,
            const std::vector<ROOM_RIPUP_OBSTACLE>& aRipupObstacles = {} );

    /** Exact-octagonal room/page/drill frontier. Layers retain physical stack
     * order; inactive layers still participate in full via preflight. */
    static std::optional<ROOM_MULTILAYER_PATH> FindMultilayerConnection(
            const std::vector<ROOM_LAYER>& aLayers, int aNet, double aSectionOffset,
            const ROOM_VIA_SETTINGS& aVia, int aMaxExpanded, int& aExpanded,
            ROOM_SEARCH_METRICS& aMetrics, const ROUTER_CANCEL_CALLBACK& aCancel = {},
            const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress = {},
            bool aSourceTraceRooms = false,
            PERSISTENT_45_DEGREE_TREE_STATE* aPersistentTree = nullptr );
};
} // namespace KICAD_AUTOROUTER
