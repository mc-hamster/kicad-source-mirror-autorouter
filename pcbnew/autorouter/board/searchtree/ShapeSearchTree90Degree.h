/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Derived from Freerouting's board/searchtree/ShapeSearchTree90Degree.java
 * at a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "../../expansion/IncompleteFreeSpaceExpansionRoom.h"

namespace KICAD_AUTOROUTER
{

/**
 * Ported orthogonal room-restraint primitive. The board search-tree traversal
 * and 45-degree/general convex variants are not implemented by this class yet.
 */
class SHAPE_SEARCH_TREE_90_DEGREE
{
public:
    static std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM> RestrainShape(
            const INCOMPLETE_FREE_SPACE_EXPANSION_ROOM& aRoom, const ROUTER_BOX& aObstacle );
};

} // namespace KICAD_AUTOROUTER
