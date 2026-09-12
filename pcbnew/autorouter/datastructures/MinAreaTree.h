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
    // Exact 45-degree shape.  Freerouting's 45-degree minimum-area tree uses
    // octagonal unions for insertion and traversal as well as for leaf tests.
    std::optional<PLANAR::INT_OCTAGON> octagon;
    // Exact arbitrary-angle convex shape.  The general ShapeSearchTree keeps
    // this representation through room restraint instead of silently
    // replacing non-45-degree supports with their octagonal envelope.
    std::optional<PLANAR::SIMPLEX> simplex;
    // Source board-item insertion order.  Freerouting keeps one all-layer
    // tree, so shapes of a multilayer item must be replayed together rather
    // than grouped by the native layer context that consumes them.
    std::uint64_t treeInsertionOrder = std::numeric_limits<std::uint64_t>::max();
    bool boardOutline = false;
    // Present only on final PolylineTrace leaves produced by the forced
    // inserter. All leaves of that item share the immutable journal. The
    // expansion is candidate-tree-specific and is attached per leaf because
    // a neckdown can change width within one inserted connection.
    std::shared_ptr<const std::vector<ROUTING_TRACE_INSERTION_STEP>>
            traceInsertionSteps;
    std::int64_t traceTreeExpansion = -1;

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
        return PLANAR::SIMPLEX::FromBox( shape );
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
    /** Change leaf identity/index without changing binary-tree topology.
     * ShapeSearchTree.mergeEntries* transfers surviving Leaf objects between
     * PolylineTrace instances in exactly this way. Geometry must remain
     * identical because ancestor bounds are intentionally not rebuilt. */
    bool UpdateEntry( HANDLE aHandle, const SHAPE_TREE_ENTRY& aEntry );
    std::size_t Size() const { return m_leafCount; }

    // The callback may shrink aQuery during traversal, as completeShape does.
    // It must not mutate the tree. False stops traversal (e.g. cancellation).
    bool Visit( ROUTER_BOX& aQuery,
                const std::function<bool( const SHAPE_TREE_ENTRY& )>& aVisit ) const;
    bool Visit( PLANAR::INT_OCTAGON& aQuery,
                const std::function<bool( const SHAPE_TREE_ENTRY& )>& aVisit ) const;
    std::vector<SHAPE_TREE_ENTRY> Overlaps( ROUTER_BOX aQuery ) const;
    std::vector<SHAPE_TREE_ENTRY> Overlaps( PLANAR::INT_OCTAGON aQuery ) const;

private:
    struct NODE
    {
        ROUTER_BOX bounds;
        HANDLE parent = NONE;
        HANDLE first = NONE;
        HANDLE second = NONE;
        std::optional<SHAPE_TREE_ENTRY> entry;
        // Freerouting's 45-degree MinAreaTree uses IntOctagon for both the
        // insertion-area heuristic and traversal pruning.  Keeping only its
        // rectangular envelope changes the tree topology and therefore the
        // observable obstacle visitation order during room completion.
        std::optional<PLANAR::INT_OCTAGON> octagonBounds;
    };
    std::vector<NODE> m_nodes;
    HANDLE m_root = NONE;
    std::size_t m_leafCount = 0;
};
} // namespace KICAD_AUTOROUTER
