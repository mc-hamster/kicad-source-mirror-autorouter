/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Port of Freerouting datastructures/MinAreaTree.java at a11c0a42 (GPL-3.0).
 */
#pragma once

#include <functional>
#include "../geometry/planar/IntBox.h"
#include "../geometry/planar/IntOctagon.h"
#include "../geometry/planar/Simplex.h"

namespace KICAD_AUTOROUTER
{
/** A compensated shape, with identity independent of its bounding rectangle. */
struct SHAPE_TREE_ENTRY
{
    SHAPE_TREE_ENTRY() = default;
    SHAPE_TREE_ENTRY( ROUTER_BOX aShape, int aObjectId = 0, int aShapeIndex = 0,
                      int aLayer = 0, int aNet = 0, bool aIsRoom = false,
                      bool aObstacle = true,
                      std::optional<PLANAR::INT_OCTAGON> aOctagon = {},
                      std::optional<PLANAR::SIMPLEX> aSimplex = {} ) :
            shape( aShape ), objectId( aObjectId ), shapeIndex( aShapeIndex ),
            layer( aLayer ), net( aNet ), isRoom( aIsRoom ), obstacle( aObstacle ),
            octagon( std::move( aOctagon ) ), simplex( std::move( aSimplex ) )
    {
    }

    ROUTER_BOX shape;
    int objectId = 0;
    int shapeIndex = 0;
    int layer = 0;
    int net = 0;
    bool isRoom = false;
    bool obstacle = true;
    // Exact 45-degree shape.  The minimum-area tree deliberately indexes its
    // bounding box; ShapeSearchTree45Degree applies this shape as the exact
    // narrow phase after traversal reaches a leaf.
    std::optional<PLANAR::INT_OCTAGON> octagon;
    // Exact arbitrary-angle convex shape.  The general ShapeSearchTree keeps
    // this representation through room restraint instead of silently
    // replacing non-45-degree supports with their octagonal envelope.
    std::optional<PLANAR::SIMPLEX> simplex;

    bool IsTraceObstacle( int aNet ) const
    {
        return obstacle && ( isRoom || net == 0 || net != aNet );
    }

    PLANAR::INT_OCTAGON BoundingOctagon() const
    {
        return octagon ? *octagon : PLANAR::INT_OCTAGON::FromBox( shape );
    }

    PLANAR::SIMPLEX BoundingSimplex() const
    {
        if( simplex )
            return *simplex;
        if( octagon )
        {
            const auto converted = octagon->ToSimplex();
            return converted ? *converted : PLANAR::SIMPLEX::Empty();
        }
        return PLANAR::SIMPLEX::Box( shape );
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
