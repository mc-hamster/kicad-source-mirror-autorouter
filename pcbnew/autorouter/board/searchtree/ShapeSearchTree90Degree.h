/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Derived from Freerouting's board/searchtree/ShapeSearchTree90Degree.java
 * at a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "../../expansion/IncompleteFreeSpaceExpansionRoom.h"
#include "../../datastructures/MinAreaTree.h"

namespace KICAD_AUTOROUTER
{

/**
 * Orthogonal shape completion, including the reference's dynamic traversal.
 * Entries must already be clearance compensated by the caller. This is not
 * the 45-degree/general-convex tree or its incremental trace-update machinery.
 */
class SHAPE_SEARCH_TREE_90_DEGREE
{
public:
    explicit SHAPE_SEARCH_TREE_90_DEGREE( ROUTER_BOX aBounds = {} ) : m_bounds( aBounds ) {}
    MIN_AREA_TREE::HANDLE Insert( const SHAPE_TREE_ENTRY& aEntry ) { return m_tree.Insert( aEntry ); }
    bool Remove( MIN_AREA_TREE::HANDLE aHandle ) { return m_tree.Remove( aHandle ); }
    std::vector<SHAPE_TREE_ENTRY> Overlaps( ROUTER_BOX aBox ) const { return m_tree.Overlaps( aBox ); }
    const ROUTER_BOX& Bounds() const { return m_bounds; }

    std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM> CompleteShape(
            const INCOMPLETE_FREE_SPACE_EXPANSION_ROOM& aRoom, int aNet,
            std::optional<int> aIgnoreObject = {}, std::optional<ROUTER_BOX> aIgnoreShape = {},
            const ROUTER_CANCEL_CALLBACK& aCancel = {} ) const;

    static std::vector<INCOMPLETE_FREE_SPACE_EXPANSION_ROOM> RestrainShape(
            const INCOMPLETE_FREE_SPACE_EXPANSION_ROOM& aRoom, const ROUTER_BOX& aObstacle );
private:
    ROUTER_BOX m_bounds;
    MIN_AREA_TREE m_tree;
};

} // namespace KICAD_AUTOROUTER
