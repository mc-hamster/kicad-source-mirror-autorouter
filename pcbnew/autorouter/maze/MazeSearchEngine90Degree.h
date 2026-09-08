/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Rectangular free-room slice of Freerouting a11c0a42. See the implementation
 * for the boundary between translated algorithms and the host adaptation.
 */
#pragma once

#include "../board/searchtree/ShapeSearchTree90Degree.h"
#include "../drill/DrillPage.h"

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
    bool routed = false;
};

struct ROOM_TERMINAL
{
    ROUTER_POINT start;
    ROUTER_POINT end;
    std::size_t owner = std::numeric_limits<std::size_t>::max();
};

struct ROOM_PATH
{
    std::vector<ROUTER_POINT> points;
    std::size_t startOwner;
    std::size_t targetOwner;
};

struct ROOM_LAYER
{
    int id = 0;
    bool active = true;
    ROUTER_BOX bounds;
    std::vector<SHAPE_TREE_ENTRY> obstacles;
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
    bool attachSmd = false;
    std::vector<DRILL_PIN> pins; // Physical layer ordinals, not host layer IDs.
    // Must validate the entire manufactured via, not only entry/exit layers.
    std::function<bool( ROUTER_POINT )> canDrill;
};

struct ROOM_MULTILAYER_PATH
{
    std::vector<ROUTER_NODE> nodes;
    std::size_t startOwner;
    std::size_t targetOwner;
};

/**
 * Active rectangular room/door search, with a full-stack through-drill
 * frontier for multilayer attempts. No shove, rip-up, neckdown or
 * obstacle-room traversal is claimed by this slice. The caller
 * supplies compensated rectangles and independently checks every output edge.
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
            double aBendCost = 0 );

    // Layers are in physical stack order, with explicit (possibly nonordinal)
    // host IDs. Inactive layers are retained for full through-drill validation.
    static std::optional<ROOM_MULTILAYER_PATH> FindMultilayerConnection(
            const std::vector<ROOM_LAYER>& aLayers, int aNet, double aSectionOffset,
            const ROOM_VIA_SETTINGS& aVia, int aMaxExpanded, int& aExpanded,
            ROOM_SEARCH_METRICS& aMetrics, const ROUTER_CANCEL_CALLBACK& aCancel = {},
            const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress = {}, bool aOrthogonal = false );
};
} // namespace KICAD_AUTOROUTER
