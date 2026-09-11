/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Board-outline tree geometry translated from Freerouting BoardOutline and
 * ShapeSearchTree at a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "../../../AutorouterTypes.h"
#include "../../../datastructures/MinAreaTree.h"

#include <algorithm>
#include <iterator>

namespace KICAD_AUTOROUTER
{

/** Source-shaped board boundary used by the routing search tree.
 *
 * KiCad's board bounding box includes the displayed Edge.Cuts stroke width.
 * Freerouting instead starts from the geometric contour, gives each outline
 * segment a fixed 10 um half width, compensates that obstacle against the
 * candidate trace class, and uses a box 100 um outside the contour only as a
 * finite room-construction bound.
 */
class BOARD_OUTLINE
{
public:
    static constexpr std::int64_t HALF_WIDTH_IU = 10000;
    static constexpr std::int64_t BOUNDING_MARGIN_IU = 100000;

    static ROUTER_BOX SearchBounds( const BOARD_SNAPSHOT& aBoard )
    {
        if( aBoard.boardOutline.empty() )
            return aBoard.bounds;

        ROUTER_BOX result{ aBoard.boardOutline.front().x,
                           aBoard.boardOutline.front().y,
                           aBoard.boardOutline.front().x,
                           aBoard.boardOutline.front().y };
        for( const ROUTER_POINT& point : aBoard.boardOutline )
        {
            result.minX = std::min( result.minX, point.x );
            result.minY = std::min( result.minY, point.y );
            result.maxX = std::max( result.maxX, point.x );
            result.maxY = std::max( result.maxY, point.y );
        }

        result.minX -= BOUNDING_MARGIN_IU;
        result.minY -= BOUNDING_MARGIN_IU;
        result.maxX += BOUNDING_MARGIN_IU;
        result.maxY += BOUNDING_MARGIN_IU;
        return result;
    }

    static std::vector<SHAPE_TREE_ENTRY> CalculateTreeShapes(
            const BOARD_SNAPSHOT& aBoard, int aLayer,
            std::int64_t aCandidateClearanceCompensation, int& aNextObjectId )
    {
        std::vector<SHAPE_TREE_ENTRY> result;
        if( aBoard.boardOutline.size() < 2 )
            return result;

        const int objectId = aNextObjectId++;
        int shapeIndex = 0;
        const std::int64_t expansion = HALF_WIDTH_IU
                + std::max<std::int64_t>(
                        0, aBoard.edgeClearance
                                   - aCandidateClearanceCompensation ) + 1;

        const auto appendContour = [&]( const std::vector<ROUTER_POINT>& aContour )
        {
            if( aContour.size() < 2 )
                return;

            for( std::size_t index = 0; index < aContour.size(); ++index )
            {
                const ROUTER_POINT& start = aContour[index];
                const ROUTER_POINT& end = aContour[( index + 1 ) % aContour.size()];
                if( start == end )
                    continue;

                PLANAR::INT_OCTAGON shape = PLANAR::INT_OCTAGON(
                        std::min( start.x, end.x ),
                        std::min( start.y, end.y ),
                        std::max( start.x, end.x ),
                        std::max( start.y, end.y ),
                        std::min( start.x - start.y, end.x - end.y ),
                        std::max( start.x - start.y, end.x - end.y ),
                        std::min( start.x + start.y, end.x + end.y ),
                        std::max( start.x + start.y, end.x + end.y ) )
                                                    .Normalize()
                                                    .Offset( expansion );
                if( shape.Dimension() < 0 )
                    continue;

                result.emplace_back(
                        shape.BoundingBox(), objectId, shapeIndex++, aLayer,
                        0, false, true, shape,
                        PLANAR::SIMPLEX::FromExpandedSegment(
                                start, end, expansion ) );
            }
        };

        appendContour( aBoard.boardOutline );
        for( const std::vector<ROUTER_POINT>& hole : aBoard.boardHoles )
            appendContour( hole );
        return result;
    }
};

} // namespace KICAD_AUTOROUTER
