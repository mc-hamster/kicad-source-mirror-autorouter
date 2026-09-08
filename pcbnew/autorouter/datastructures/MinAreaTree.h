/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Port of Freerouting datastructures/MinAreaTree.java at a11c0a42 (GPL-3.0).
 */
#pragma once

#include <functional>
#include "../geometry/planar/IntBox.h"

namespace KICAD_AUTOROUTER
{
/** A compensated shape, with identity independent of its bounding rectangle. */
struct SHAPE_TREE_ENTRY
{
    ROUTER_BOX shape;
    int objectId = 0;
    int shapeIndex = 0;
    int layer = 0;
    int net = 0;
    bool isRoom = false;
    bool obstacle = true;

    bool IsTraceObstacle( int aNet ) const
    {
        return obstacle && ( isRoom || net == 0 || net != aNet );
    }
};

/**
 * Minimum-area-increase binary tree. First-child insertion ties and second-first
 * traversal are intentional: changing either changes completed room topology.
 * Handles remain stable through inserts/removes; erased handles are not reused.
 */
class MIN_AREA_TREE
{
public:
    using HANDLE = std::size_t;
    static constexpr HANDLE NONE = std::numeric_limits<HANDLE>::max();

    HANDLE Insert( const SHAPE_TREE_ENTRY& aEntry );
    bool Remove( HANDLE aHandle );
    std::size_t Size() const { return m_leafCount; }

    // The callback may shrink aQuery during traversal, as completeShape does.
    // It must not mutate the tree. False stops traversal (e.g. cancellation).
    bool Visit( ROUTER_BOX& aQuery,
                const std::function<bool( const SHAPE_TREE_ENTRY& )>& aVisit ) const;
    std::vector<SHAPE_TREE_ENTRY> Overlaps( ROUTER_BOX aQuery ) const;

private:
    struct NODE
    {
        ROUTER_BOX bounds;
        HANDLE parent = NONE;
        HANDLE first = NONE;
        HANDLE second = NONE;
        std::optional<SHAPE_TREE_ENTRY> entry;
    };
    std::vector<NODE> m_nodes;
    HANDLE m_root = NONE;
    std::size_t m_leafCount = 0;
};
} // namespace KICAD_AUTOROUTER
