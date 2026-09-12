/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct C++ translation of Freerouting geometry/planar/IntOctagon.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "Line.h"
#include "Simplex.h"

#include <array>
#include <optional>

namespace KICAD_AUTOROUTER::PLANAR
{

/** Integer convex octagon bounded by the four orthogonal and four 45-degree
 * support directions used by Freerouting's 45-degree search tree.
 *
 * Diagonal coordinates are support-line intercepts: upper/lower-left and
 * lower/right constrain x-y, while lower/upper-right constrain x+y.  Public
 * immutable-looking fields deliberately retain the source representation and
 * constructor order so geometry oracle records can be compared directly.
 */
class INT_OCTAGON
{
public:
    /** Freerouting FortyfiveDegreeDirection declaration order. */
    enum class DIRECTION_45
    {
        RIGHT,
        RIGHT45,
        UP,
        UP45,
        LEFT,
        LEFT45,
        DOWN,
        DOWN45
    };

    static constexpr std::int64_t CRITICAL_COORDINATE = 33554432;

    std::int64_t leftX;
    std::int64_t bottomY;
    std::int64_t rightX;
    std::int64_t topY;
    std::int64_t upperLeftDiagonalX;
    std::int64_t lowerRightDiagonalX;
    std::int64_t lowerLeftDiagonalX;
    std::int64_t upperRightDiagonalX;

    INT_OCTAGON( std::int64_t aLeftX, std::int64_t aBottomY,
                 std::int64_t aRightX, std::int64_t aTopY,
                 std::int64_t aUpperLeftDiagonalX,
                 std::int64_t aLowerRightDiagonalX,
                 std::int64_t aLowerLeftDiagonalX,
                 std::int64_t aUpperRightDiagonalX ) :
            leftX( aLeftX ), bottomY( aBottomY ), rightX( aRightX ), topY( aTopY ),
            upperLeftDiagonalX( aUpperLeftDiagonalX ),
            lowerRightDiagonalX( aLowerRightDiagonalX ),
            lowerLeftDiagonalX( aLowerLeftDiagonalX ),
            upperRightDiagonalX( aUpperRightDiagonalX )
    {
    }

    static INT_OCTAGON Empty();
    static INT_OCTAGON FromBox( const ROUTER_BOX& aBox );
    /** Smallest octagonal envelope of an integral line segment.
     *
     * This is LineSegment.boundingOctagon() from Freerouting.  Keeping a
     * one-dimensional trace connection shape one-dimensional is observable:
     * start-room completion uses the whole segment, not a collection of
     * endpoint samples.
     */
    static INT_OCTAGON FromSegment( ROUTER_POINT aStart,
                                    ROUTER_POINT aEnd );

    bool IsEmpty() const;
    bool IsNormalized() const;
    bool IsIntBox() const;
    int Dimension() const;
    ROUTER_BOX BoundingBox() const;
    ROUTER_POINT Corner( int aIndex ) const;
    std::int64_t CornerX( int aIndex ) const;
    std::int64_t CornerY( int aIndex ) const;
    double Area() const;
    LINE BorderLine( int aIndex ) const;

    INT_OCTAGON TranslateBy( ROUTER_POINT aVector ) const;
    double MaxWidth() const;
    double MinWidth() const;
    INT_OCTAGON Offset( double aDistance ) const;
    /** Apply source Java rounding on a coarser host-coordinate lattice.
     *
     * Freerouting rounds both the orthogonal and sqrt(2)-scaled offset in its
     * integer board unit.  KiCad IU are finer than that unit, so doing the
     * irrational multiplication in IU produces a different support line.
     */
    INT_OCTAGON OffsetOnGrid( double aDistance, std::int64_t aGrid ) const;
    INT_OCTAGON Enlarge( double aOffset ) const { return Offset( aOffset ); }

    bool Contains( ROUTER_POINT aPoint ) const;
    bool ContainsInside( ROUTER_POINT aPoint ) const;
    bool IsContainedIn( const ROUTER_BOX& aBox ) const;
    bool IsContainedIn( const INT_OCTAGON& aOther ) const;
    bool Intersects( const INT_OCTAGON& aOther ) const;
    bool Overlaps( const INT_OCTAGON& aOther ) const;

    INT_OCTAGON Union( const INT_OCTAGON& aOther ) const;
    INT_OCTAGON Intersection( const INT_OCTAGON& aOther ) const;
    INT_OCTAGON Normalize() const;

    /** Normalize after evaluating Freerouting's integer arithmetic on a
     * coarser host-coordinate lattice.  Every support is first expressed in
     * source units, so ceil/floor at diagonal intersections happens before
     * scaling back to KiCad IU. */
    INT_OCTAGON NormalizeOnGrid( std::int64_t aGrid ) const;
    INT_OCTAGON IntersectionOnGrid( const INT_OCTAGON& aOther,
                                    std::int64_t aGrid ) const;

    /** Divide this outer octagon minus aCutout into Freerouting's ordered
     * convex pieces. A non-overlap returns only this shape; an area overlap
     * returns eight pieces, including empty/lower-dimensional intermediates,
     * so PolylineArea can apply TileShape's dimension filtering itself.
     */
    std::vector<INT_OCTAGON> Cutout( const INT_OCTAGON& aCutout ) const;

    /** Divide a rectangular outer shape minus this octagonal cutout.  This is
     * the specialised IntOctagon.cutoutFrom(IntBox) dispatch used by Java
     * before TileShape.simplify() changes later dispatch decisions.
     */
    std::vector<INT_OCTAGON> CutoutFromBox( const ROUTER_BOX& aOuter ) const;

    /** Arithmetic mean of the eight source octagon corners.  This deliberately
     * is not the polygon area centroid: PolylineShape.centreOfGravity() uses
     * the arithmetic corner mean and DrillPage rounds that point.
     */
    std::pair<double, double> CentreOfGravity() const;

    /** Source Side values encoded as -1=ON_THE_LEFT, 0=COLLINEAR,
     * +1=ON_THE_RIGHT. */
    int SideOfBorderLine( std::int64_t aX, std::int64_t aY, int aBorderIndex ) const;
    int Compare( const INT_OCTAGON& aOther, int aEdgeIndex ) const;

    std::int64_t LeftXValue( std::int64_t aY ) const;
    std::int64_t RightXValue( std::int64_t aY ) const;
    std::int64_t LowerYValue( std::int64_t aX ) const;
    std::int64_t UpperYValue( std::int64_t aX ) const;

    /** Nearest outside integral border point reached along one of the eight
     * source 45-degree directions.  The input is normally inside this shape;
     * the source operation itself intentionally does not enforce that guard.
     */
    ROUTER_POINT BorderPoint( ROUTER_POINT aPoint, DIRECTION_45 aDirection ) const;

    /** Sorted source-direction border projections. Equal distances preserve
     * FortyfiveDegreeDirection declaration order.
     */
    std::vector<ROUTER_POINT> NearestBorderProjections(
            ROUTER_POINT aPoint, int aMaximumResultPoints ) const;

    std::optional<SIMPLEX> ToSimplex() const;

    bool operator==( const INT_OCTAGON& aOther ) const;
    bool operator!=( const INT_OCTAGON& aOther ) const { return !( *this == aOther ); }
};

} // namespace KICAD_AUTOROUTER::PLANAR
