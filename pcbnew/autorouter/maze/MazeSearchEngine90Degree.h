/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Rectangular free-room slice of Freerouting a11c0a42. See the implementation
 * for the boundary between translated algorithms and the host adaptation.
 */
#pragma once

#include "../board/searchtree/ShapeSearchTree90Degree.h"
#include "../drill/DrillPage.h"
#include "MazeTraceShover.h"

namespace KICAD_AUTOROUTER
{
struct ROOM_SEARCH_METRICS
{
    int rooms = 0;
    int doors = 0;
    int sections = 0;
    int drillPages = 0;
    int drills = 0;
    int layerTransitions = 0;
    int destinationQueries = 0;
    int obstacleRooms = 0;
    int rippedRooms = 0;
    std::int64_t ripupCost = 0;
    bool routed = false;
};

/** A compensated shape belonging to one movable foreign connection item.
 *
 * The shape remains in the room tree so ordinary free rooms are restrained by
 * it, but it also owns an ObstacleExpansionRoom which the frontier may enter
 * after paying ripupCost.  group identifies all tree shapes belonging to the
 * same native connection item.
 */
struct ROOM_RIPUP_OBSTACLE
{
    SHAPE_TREE_ENTRY shape;
    std::size_t      group = std::numeric_limits<std::size_t>::max();
    int              ripupCost = 0;
    // Native occupancy connection that owns this item shape.  This is kept
    // separate from group: one source Connection may contain multiple trace
    // and via items with independently paid obstacle rooms.
    std::size_t      connectionIndex = std::numeric_limits<std::size_t>::max();
    // Present only for a source PolylineTrace-style item.  Drill rooms and
    // unsupported mutable geometry deliberately retain an empty pointer and
    // fall through to ordinary paid rip-up.
    std::shared_ptr<const MAZE_TRACE_ROOM_INFO> traceInfo;
};

struct ROOM_TERMINAL
{
    ROUTER_POINT start;
    ROUTER_POINT end;
    std::size_t owner = std::numeric_limits<std::size_t>::max();
    // Exact bounding region joined into DestinationDistance.  The routeable
    // connection shape remains start..end; Freerouting deliberately uses the
    // larger search-tree shape only for its admissible destination bound.
    ROUTER_BOX treeBounds{ 1, 1, 0, 0 };
};

struct ROOM_PATH
{
    std::vector<ROUTER_POINT> points;
    std::size_t startOwner;
    std::size_t targetOwner;
    std::vector<std::size_t> rippedObstacleGroups;
    std::int64_t ripupCost = 0;
};

struct ROOM_LAYER
{
    int id = 0;
    bool active = true;
    ROUTER_BOX bounds;
    std::vector<SHAPE_TREE_ENTRY> obstacles;
    std::vector<ROOM_RIPUP_OBSTACLE> ripupObstacles;
    std::vector<ROOM_TERMINAL> starts;
    std::vector<ROOM_TERMINAL> targets;
    double horizontalCost = 1;
    double verticalCost = 1;
    double bendCost = 0;
};

struct ROOM_VIA_SETTINGS
{
    ROUTER_BOX bounds;
    std::vector<SHAPE_TREE_ENTRY> obstacles;
    std::int64_t pageWidth = 10000;
    double normalCost = 0;
    // Freerouting keeps one global room queue across all enabled layers even
    // when the ViaRule is empty.  Disable only drill-page transitions; do not
    // fall back to independent per-layer searches with different ordering.
    bool transitionsEnabled = true;
    bool attachSmd = false;
    std::vector<DRILL_PIN> pins; // Physical layer ordinals, not host layer IDs.
    // Must validate the entire manufactured via, not only entry/exit layers.
    std::function<bool( ROUTER_POINT )> canDrill;
    // Select and validate the first ordered ViaRule entry which can perform
    // this exact layer transition at the candidate drill.  The returned
    // style carries the complete manufactured padstack span into
    // backtracking and insertion.  A missing callback retains the legacy
    // lower-level room-search behavior used by geometry-only tests.
    std::function<std::optional<ROUTING_EDGE_STYLE>( ROUTER_POINT, int, int )>
            selectViaStyle;
    // RoutingBoard.fanout terminates at the first drill reached from the
    // pin's one-layer connected set. These fields keep that state inside the
    // same room/drill frontier instead of falling back to a second grid maze.
    bool stopAtFirstDrill = false;
    bool allowDirectFanoutTarget = false;
    int fanoutSourceLayer = -1;
    ROUTER_POINT fanoutCenter;
    std::int64_t fanoutMinDistance = 0;
    std::int64_t fanoutMaxDistance = 0; // 0 = unbounded
};

struct ROOM_MULTILAYER_PATH
{
    std::vector<ROUTER_NODE> nodes;
    std::size_t startOwner;
    std::size_t targetOwner;
    std::vector<std::size_t> rippedObstacleGroups;
    std::int64_t ripupCost = 0;
    // One entry per node edge.  Via entries are selected while the drill
    // transition is queued rather than guessed after path reconstruction.
    std::vector<ROUTING_EDGE_STYLE> edgeStyles;
};


namespace DETAIL
{
/** Shared, angle-independent translations of the thin-room portion of
 * MazeSearchEngine.expandToRoomDoors(). */
std::optional<FLOAT_LINE> SegmentProjection(
        const FLOAT_LINE& aFromSegment, const FLOAT_LINE& aToSegment );
bool RoomIsThick( const EXPANSION_ROOM& aRoom,
                  double aCompensatedTraceHalfWidth,
                  const EXPANSION_DOOR* aEntryDoor,
                  FLOAT_POINT aEntryMiddle, bool aCurrentDoorIsSmall );
bool DoorEntryIsThick( const EXPANSION_ROOM& aRoom,
                       const EXPANSION_DOOR& aDoor,
                       const std::vector<FLOAT_LINE>& aSections,
                       double aCompensatedTraceHalfWidth );
} // namespace DETAIL

/**
 * Active rectangular room/door search, with a full-stack through-drill
 * frontier for multilayer attempts. Movable rectangular trace/via shapes are
 * represented by paid obstacle-room states; actual shove/rip-up and neckdown
 * remain insertion responsibilities. The caller supplies compensated
 * rectangles and independently checks every output edge.
 */
class MAZE_SEARCH_ENGINE_90_DEGREE
{
public:
    static std::optional<ROOM_PATH> FindConnection(
            ROUTER_BOX aBounds, const std::vector<SHAPE_TREE_ENTRY>& aObstacles,
            int aLayer, int aNet, const std::vector<ROOM_TERMINAL>& aStarts,
            const std::vector<ROOM_TERMINAL>& aTargets, double aSectionOffset,
            double aHorizontalCost, double aVerticalCost, int aMaxExpanded,
            int& aExpanded, ROOM_SEARCH_METRICS& aMetrics,
            const ROUTER_CANCEL_CALLBACK& aCancel = {},
            const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress = {}, bool aOrthogonal = true,
            double aBendCost = 0,
            const std::vector<ROOM_RIPUP_OBSTACLE>& aRipupObstacles = {},
            bool aSourceTraceRooms = false );

    // Layers are in physical stack order, with explicit (possibly nonordinal)
    // host IDs. Inactive layers are retained for full through-drill validation.
    static std::optional<ROOM_MULTILAYER_PATH> FindMultilayerConnection(
            const std::vector<ROOM_LAYER>& aLayers, int aNet, double aSectionOffset,
            const ROOM_VIA_SETTINGS& aVia, int aMaxExpanded, int& aExpanded,
            ROOM_SEARCH_METRICS& aMetrics, const ROUTER_CANCEL_CALLBACK& aCancel = {},
            const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress = {}, bool aOrthogonal = false );
};
} // namespace KICAD_AUTOROUTER
