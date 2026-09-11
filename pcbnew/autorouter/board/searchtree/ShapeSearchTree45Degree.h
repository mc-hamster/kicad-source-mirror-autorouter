/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Derived from Freerouting board/searchtree/ShapeSearchTree45Degree.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "../../datastructures/MinAreaTree.h"

namespace KICAD_AUTOROUTER
{

struct INCOMPLETE_45_DEGREE_EXPANSION_ROOM
{
    PLANAR::INT_OCTAGON shape;
    int layer = 0;
    PLANAR::INT_OCTAGON containedShape;
};

/** Exact octagonal room completion for the 45-degree autorouter.
 *
 * MIN_AREA_TREE remains a bounding-box broad phase, like the source tree's
 * inner-node bounds.  Every leaf decision and restraint uses the retained
 * INT_OCTAGON, so diagonal corner space is not silently collapsed to a box.
 */
class SHAPE_SEARCH_TREE_45_DEGREE
{
public:
    explicit SHAPE_SEARCH_TREE_45_DEGREE( ROUTER_BOX aBounds = {} ) : m_bounds( aBounds ) {}

    MIN_AREA_TREE::HANDLE Insert( SHAPE_TREE_ENTRY aEntry );
    bool Remove( MIN_AREA_TREE::HANDLE aHandle ) { return m_tree.Remove( aHandle ); }
    std::vector<SHAPE_TREE_ENTRY> Overlaps( const PLANAR::INT_OCTAGON& aShape ) const;
    const ROUTER_BOX& Bounds() const { return m_bounds; }

    std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> CompleteShape(
            const INCOMPLETE_45_DEGREE_EXPANSION_ROOM& aRoom, int aNet,
            std::optional<int> aIgnoreObject = {},
            std::optional<PLANAR::INT_OCTAGON> aIgnoreShape = {},
            const ROUTER_CANCEL_CALLBACK& aCancel = {} ) const;

    static std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> RestrainShape(
            const INCOMPLETE_45_DEGREE_EXPANSION_ROOM& aRoom,
            const PLANAR::INT_OCTAGON& aObstacle );

    static PLANAR::INT_OCTAGON CalcOutsideRestrainedShape(
            const PLANAR::INT_OCTAGON& aObstacle, int aObstacleLine,
            const PLANAR::INT_OCTAGON& aRoom );
    static PLANAR::INT_OCTAGON CalcInsideRestrainedShape(
            const PLANAR::INT_OCTAGON& aObstacle, int aObstacleLine,
            const PLANAR::INT_OCTAGON& aRoom );

    /** Offset an IntBox-backed DrillItem without chamfering its corners.
     *
     * ShapeSearchTree45Degree.calculateTreeShapes(DrillItem) deliberately
     * uses IntBox.offset() when the pad's bounding octagon is a box.  Keeping
     * this operation named at the tree boundary prevents generic obstacle
     * offsets from accidentally changing room topology around rectangular
     * pads.
     */
    static PLANAR::INT_OCTAGON OffsetDrillItemBox(
            const ROUTER_BOX& aBox, std::int64_t aDistance );

private:
    static bool obstacleSegmentTouchesInside( const PLANAR::INT_OCTAGON& aObstacle,
                                               int aObstacleLine,
                                               const PLANAR::INT_OCTAGON& aRoom );
    static double signedLineDistance( const PLANAR::INT_OCTAGON& aObstacle,
                                      int aObstacleLine,
                                      const PLANAR::INT_OCTAGON& aContained );
    std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> divideLargeRoom(
            std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> aRooms ) const;

    ROUTER_BOX m_bounds;
    MIN_AREA_TREE m_tree;
};

} // namespace KICAD_AUTOROUTER
