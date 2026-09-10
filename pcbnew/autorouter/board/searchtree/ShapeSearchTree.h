/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct translation of Freerouting board/searchtree/ShapeSearchTree.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "../../datastructures/MinAreaTree.h"

namespace KICAD_AUTOROUTER
{

struct INCOMPLETE_GENERAL_EXPANSION_ROOM
{
    PLANAR::SIMPLEX shape;
    int layer = 0;
    PLANAR::SIMPLEX containedShape;
};

/** General-convex room completion used by the unrestricted-angle router.
 *
 * The minimum-area tree is only a bounding-box broad phase.  Obstacle
 * intersection, containment, room splitting, and retained contained shapes
 * all use exact rational SIMPLEX geometry, matching the source TileShape path.
 */
class SHAPE_SEARCH_TREE
{
public:
    explicit SHAPE_SEARCH_TREE( ROUTER_BOX aBounds = {} ) : m_bounds( aBounds ) {}

    MIN_AREA_TREE::HANDLE Insert( SHAPE_TREE_ENTRY aEntry );
    bool Remove( MIN_AREA_TREE::HANDLE aHandle ) { return m_tree.Remove( aHandle ); }
    std::vector<SHAPE_TREE_ENTRY> Overlaps( const PLANAR::SIMPLEX& aShape ) const;
    const ROUTER_BOX& Bounds() const { return m_bounds; }

    std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> CompleteShape(
            const INCOMPLETE_GENERAL_EXPANSION_ROOM& aRoom, int aNet,
            std::optional<int> aIgnoreObject = {},
            std::optional<PLANAR::SIMPLEX> aIgnoreShape = {},
            const ROUTER_CANCEL_CALLBACK& aCancel = {} ) const;

    static std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> RestrainShape(
            const INCOMPLETE_GENERAL_EXPANSION_ROOM& aRoom,
            const PLANAR::SIMPLEX& aObstacle );

private:
    std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> divideLargeRoom(
            std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> aRooms ) const;

    ROUTER_BOX m_bounds;
    MIN_AREA_TREE m_tree;
};

} // namespace KICAD_AUTOROUTER
