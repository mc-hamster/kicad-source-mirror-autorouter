/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Board-outline tree geometry translated from Freerouting BoardOutline and
 * ShapeSearchTree at a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "../../../AutorouterTypes.h"
#include "../../../datastructures/MinAreaTree.h"
#include "../../../geometry/planar/Polyline.h"
#include "../../../geometry/planar/Simplex.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <limits>

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

    static std::int64_t SourceGridCoordinate( std::int64_t aCoordinate )
    {
        const long double grid = FREEROUTING_COORDINATE_UNIT_IU;
        return static_cast<std::int64_t>(
                std::floor( static_cast<long double>( aCoordinate ) / grid + 0.5L )
                * grid );
    }

    /** Reproduce KiCad POINT::Format("%.6g") followed by Freerouting's
     * `(resolution um 10)` transform and Java Math.round().
     *
     * This differs from merely rounding to the 0.1 um source grid.  At board
     * coordinates with six integer digits, Specctra keeps no fractional
     * micrometres; for example 228.0111 mm is serialized as 228011 um and
     * Freerouting therefore receives 228.0110 mm.  Arc-tessellation vertices
     * expose that loss and the resulting one-source-unit difference changes
     * BoardOutline tree shapes and maze queue ordering.
     */
    static std::int64_t DsnRoundTripCoordinate( std::int64_t aCoordinate )
    {
        char buffer[64];
        const double micrometres = static_cast<double>( aCoordinate ) / 1000.0;
        std::snprintf( buffer, sizeof( buffer ), "%.6g", micrometres );
        const long double serializedMicrometres = std::strtod( buffer, nullptr );
        const long double sourceCoordinate = std::floor(
                serializedMicrometres * 10.0L + 0.5L );
        return static_cast<std::int64_t>( sourceCoordinate )
               * sourceCoordinateUnit();
    }

    /** Apply the DSN y-down to y-up reflection before Java rounding, then
     * return the result in KiCad's y-down coordinate system. */
    static std::int64_t DsnRoundTripCoordinateYDown(
            std::int64_t aCoordinate )
    {
        return -DsnRoundTripCoordinate( -aCoordinate );
    }

    static ROUTER_POINT DsnRoundTripPointYDown( ROUTER_POINT aPoint )
    {
        return { DsnRoundTripCoordinate( aPoint.x ),
                 DsnRoundTripCoordinateYDown( aPoint.y ) };
    }

    static ROUTER_BOX SearchBounds( const BOARD_SNAPSHOT& aBoard )
    {
        if( aBoard.boardOutline.empty() )
            return aBoard.bounds;

        const ROUTER_POINT first = DsnRoundTripPointYDown(
                aBoard.boardOutline.front() );
        ROUTER_BOX result{ first.x, first.y, first.x, first.y };
        for( const ROUTER_POINT& rawPoint : aBoard.boardOutline )
        {
            const ROUTER_POINT point = DsnRoundTripPointYDown( rawPoint );
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
        const std::int64_t expansion = SourceGridCoordinate( HALF_WIDTH_IU
                + std::max<std::int64_t>(
                        0, aBoard.edgeClearance
                                   - aCandidateClearanceCompensation ) + 1 );
        const std::int64_t sourceExpansion = expansion / sourceCoordinateUnit();

        const auto appendContour = [&]( const std::vector<ROUTER_POINT>& aContour )
        {
            if( aContour.size() < 2 )
                return;

            std::vector<ROUTER_POINT> nativeContour;
            nativeContour.reserve( aContour.size() );
            for( const ROUTER_POINT& rawPoint : aContour )
            {
                const ROUTER_POINT point = DsnRoundTripPointYDown( rawPoint );
                if( nativeContour.empty() || nativeContour.back() != point )
                    nativeContour.push_back( point );
            }
            if( nativeContour.size() > 1
                && nativeContour.front() == nativeContour.back() )
            {
                nativeContour.pop_back();
            }
            if( nativeContour.size() < 2 )
                return;

            // DSN/Freerouting uses y-up coordinates.  Reflecting KiCad's
            // y-down contour reverses its winding; rotate the reversed walk
            // by one vertex so shape zero remains the same physical edge as
            // the source BoardOutline insertion stream.
            std::vector<ROUTER_POINT> sourceContour;
            sourceContour.reserve( nativeContour.size() );
            sourceContour.push_back( toSourcePoint( nativeContour[1] ) );
            sourceContour.push_back( toSourcePoint( nativeContour[0] ) );
            for( std::size_t index = nativeContour.size(); index-- > 2; )
                sourceContour.push_back( toSourcePoint( nativeContour[index] ) );

            std::vector<PLANAR::LINE> borderLines;
            borderLines.reserve( sourceContour.size() );
            for( std::size_t index = 0; index < sourceContour.size(); ++index )
            {
                const ROUTER_POINT start = sourceContour[index];
                const ROUTER_POINT end = sourceContour[
                        ( index + 1 ) % sourceContour.size()];
                if( start == end )
                    return;
                borderLines.emplace_back( start, end );
            }

            for( std::size_t index = 0; index < borderLines.size(); ++index )
            {
                // This is ShapeSearchTree.calculateTreeShapes(BoardOutline),
                // not merely an equivalent segment envelope.  The previous
                // and next contour supports clip the translated centre line;
                // then ShapeSearchTree45Degree takes the floating-point
                // bounding octagon.  Its directed floor/ceil is observable:
                // exact rational replacement changes a diagonal support by
                // one source coordinate and therefore changes tree descent.
                PLANAR::POLYLINE polyline(
                        { borderLines[( index + borderLines.size() - 1 )
                                      % borderLines.size()],
                          borderLines[index],
                          borderLines[( index + 1 ) % borderLines.size()] } );
                const auto sourceSimplex = polyline.OffsetShape(
                        static_cast<int>( sourceExpansion ), 0 );
                if( !sourceSimplex )
                    continue;
                const auto sourceShape = sourceBoundingOctagon( *sourceSimplex );
                const auto nativeSimplex = toNativeSimplex( *sourceSimplex );
                if( !sourceShape || !nativeSimplex )
                    continue;
                const PLANAR::INT_OCTAGON shape = toNativeOctagon( *sourceShape );
                if( shape.Dimension() < 0 )
                    continue;

                result.emplace_back( shape.BoundingBox(), objectId,
                                     shapeIndex++, aLayer, 0, false, true,
                                     shape, *nativeSimplex );
                result.back().boardOutline = true;
            }
        };

        appendContour( aBoard.boardOutline );
        for( const std::vector<ROUTER_POINT>& hole : aBoard.boardHoles )
            appendContour( hole );
        return result;
    }

private:
    static constexpr std::int64_t sourceCoordinateUnit()
    {
        return static_cast<std::int64_t>( FREEROUTING_COORDINATE_UNIT_IU );
    }

    static ROUTER_POINT toSourcePoint( ROUTER_POINT aNativePoint )
    {
        const std::int64_t unit = sourceCoordinateUnit();
        return { aNativePoint.x / unit, -aNativePoint.y / unit };
    }

    static std::optional<PLANAR::INT_OCTAGON> sourceBoundingOctagon(
            const PLANAR::SIMPLEX& aSimplex )
    {
        if( aSimplex.IsEmpty() || !aSimplex.IsBounded() )
            return {};

        double left = std::numeric_limits<std::int32_t>::max();
        double bottom = left;
        double right = std::numeric_limits<std::int32_t>::min();
        double top = right;
        double upperLeft = left;
        double lowerRight = right;
        double lowerLeft = left;
        double upperRight = right;
        for( std::size_t index = 0; index < aSimplex.Borders().size(); ++index )
        {
            const FLOAT_POINT corner = aSimplex.CornerApprox( index );
            left = std::min( left, corner.x );
            bottom = std::min( bottom, corner.y );
            right = std::max( right, corner.x );
            top = std::max( top, corner.y );
            upperLeft = std::min( upperLeft, corner.x - corner.y );
            lowerRight = std::max( lowerRight, corner.x - corner.y );
            lowerLeft = std::min( lowerLeft, corner.x + corner.y );
            upperRight = std::max( upperRight, corner.x + corner.y );
        }

        return PLANAR::INT_OCTAGON(
                static_cast<std::int64_t>( std::floor( left ) ),
                static_cast<std::int64_t>( std::floor( bottom ) ),
                static_cast<std::int64_t>( std::ceil( right ) ),
                static_cast<std::int64_t>( std::ceil( top ) ),
                static_cast<std::int64_t>( std::floor( upperLeft ) ),
                static_cast<std::int64_t>( std::ceil( lowerRight ) ),
                static_cast<std::int64_t>( std::floor( lowerLeft ) ),
                static_cast<std::int64_t>( std::ceil( upperRight ) ) );
    }

    static std::int64_t fromSourceCoordinate( std::int64_t aCoordinate )
    {
        const PLANAR::INTEGER result = PLANAR::INTEGER( aCoordinate )
                                       * sourceCoordinateUnit();
        if( result < std::numeric_limits<std::int64_t>::min()
            || result > std::numeric_limits<std::int64_t>::max() )
        {
            throw std::overflow_error( "board outline coordinate overflow" );
        }
        return result.convert_to<std::int64_t>();
    }

    static ROUTER_POINT toNativePoint( ROUTER_POINT aSourcePoint )
    {
        return { fromSourceCoordinate( aSourcePoint.x ),
                 -fromSourceCoordinate( aSourcePoint.y ) };
    }

    static std::optional<PLANAR::SIMPLEX> toNativeSimplex(
            const PLANAR::SIMPLEX& aSourceSimplex )
    {
        std::vector<PLANAR::LINE> borders;
        borders.reserve( aSourceSimplex.Borders().size() );
        for( const PLANAR::LINE& sourceLine : aSourceSimplex.Borders() )
        {
            // Reflection reverses handedness, so reverse every directed
            // support while mapping it back to KiCad's y-down coordinates.
            borders.emplace_back( toNativePoint( sourceLine.b ),
                                  toNativePoint( sourceLine.a ) );
        }
        const PLANAR::SIMPLEX result = PLANAR::SIMPLEX::GetInstance(
                std::move( borders ) );
        if( result.IsEmpty() || !result.IsBounded() )
            return {};
        return result;
    }

    static PLANAR::INT_OCTAGON toNativeOctagon(
            const PLANAR::INT_OCTAGON& aSource )
    {
        return PLANAR::INT_OCTAGON(
                fromSourceCoordinate( aSource.leftX ),
                -fromSourceCoordinate( aSource.topY ),
                fromSourceCoordinate( aSource.rightX ),
                -fromSourceCoordinate( aSource.bottomY ),
                fromSourceCoordinate( aSource.lowerLeftDiagonalX ),
                fromSourceCoordinate( aSource.upperRightDiagonalX ),
                fromSourceCoordinate( aSource.upperLeftDiagonalX ),
                fromSourceCoordinate( aSource.lowerRightDiagonalX ) );
    }
};

} // namespace KICAD_AUTOROUTER
