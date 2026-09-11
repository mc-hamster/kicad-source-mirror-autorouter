/* This file is part of KiCad, licensed under GPL version 3 or later.
 * Exact-octagonal 45-degree room frontier derived from Freerouting a11c0a42.
 */
#pragma once

#include "MazeSearchEngine90Degree.h"

namespace KICAD_AUTOROUTER
{
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
            const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress = {} );

private:
    /** Direct translations of MazeSearchEngine.segmentProjection and the
     * fixed-direction parts of expandToRoomDoors.  They live on the shared
     * engine type so the single- and multilayer frontiers cannot drift. */
    static std::optional<FLOAT_LINE> SegmentProjection(
            const FLOAT_LINE& aFromSegment, const FLOAT_LINE& aToSegment );
    static bool RoomIsThick( const EXPANSION_ROOM& aRoom,
                             double aCompensatedTraceHalfWidth,
                             const EXPANSION_DOOR* aEntryDoor,
                             FLOAT_POINT aEntryMiddle );
    static bool DoorEntryIsThick( const EXPANSION_ROOM& aRoom,
                                  const EXPANSION_DOOR& aDoor,
                                  const std::vector<FLOAT_LINE>& aSections,
                                  double aCompensatedTraceHalfWidth );
};
} // namespace KICAD_AUTOROUTER
