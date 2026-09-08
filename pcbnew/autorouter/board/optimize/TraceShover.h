/* KiCad, GPL-3.0-or-later. Freerouting board/optimize/TraceShover.java.
 * This slice implements recursive spring-over of fixed convex obstacles.
 * It does NOT implement check/insert of movable traces or via shoving.
 */
#pragma once
#include "../../geometry/planar/Simplex.h"

namespace KICAD_AUTOROUTER
{
class TRACE_SHOVER
{
public:
    struct OBSTACLE
    {
        std::uint64_t id;
        ROUTER_BOX bounds;              // original item bounds, for nested obstacles
        PLANAR::SIMPLEX checkShape;     // clearance-expanded centre space
        PLANAR::SIMPLEX offsetShape;    // source's additional one-unit wrap margin
        bool canSpringOver = true;     // outline/unsupported fixed shape fails closed
    };
    struct RESULT
    {
        std::optional<PLANAR::POLYLINE> polyline;
        bool changed = false;
        bool cancelled = false;
        int recursiveSteps = 0;
    };
    // Obstacles in reference item order (descending stable item IDs).
    static RESULT SpringOverObstacles( const PLANAR::POLYLINE& aPolyline,
                                       const std::vector<OBSTACLE>& aObstacles,
                                       const ROUTER_CANCEL_CALLBACK& aCancel = {},
                                       int aMaxRecursionDepth = 20 );
};
} // namespace KICAD_AUTOROUTER
