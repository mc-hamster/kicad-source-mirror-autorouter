/* This file is part of KiCad, licensed under GPL version 3 or later.
 * Exact unrestricted-angle room frontier derived from Freerouting a11c0a42.
 */
#pragma once

#include "MazeSearchEngine90Degree.h"

namespace KICAD_AUTOROUTER
{

class MAZE_SEARCH_ENGINE_ANY_ANGLE
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
};

} // namespace KICAD_AUTOROUTER
