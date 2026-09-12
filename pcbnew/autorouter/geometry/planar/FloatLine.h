/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct C++ translation of Freerouting geometry/planar/{FloatPoint,
 * FloatLine}.java at a11c0a42d1b3827e5126429c5c9820c4ab5bec7c.
 */
#pragma once

#include "IntBox.h"

#include <optional>
#include <vector>

namespace KICAD_AUTOROUTER
{
namespace PLANAR
{
class INT_OCTAGON;
}

struct FLOAT_LINE;

struct FLOAT_POINT
{
    double x = 0;
    double y = 0;

    static PLANAR::INT_OCTAGON BoundingOctagon(
            const std::vector<FLOAT_POINT>& aPoints );

    double SizeSquared() const;
    double Size() const;
    double DistanceSquared( FLOAT_POINT aOther ) const;
    double Distance( FLOAT_POINT aOther ) const;
    double WeightedDistance( FLOAT_POINT aOther, double aHorizontal,
                             double aVertical ) const;
    ROUTER_POINT Round() const;
    ROUTER_POINT RoundToGridJava( std::int64_t aGrid ) const;
    /** Apply Java rounding after reflecting a y-down coordinate to the
     * source router's y-up system, then reflect the result back. */
    ROUTER_POINT RoundToGridJavaYDown( std::int64_t aGrid ) const;
    /** Round as Freerouting does after KiCad coordinates have been scaled
     * into native internal units. */
    ROUTER_POINT RoundToSourceGrid() const;
    /** Apply the same Java rounding after reflecting KiCad's y-down value to
     * Freerouting's y-up coordinate system, then reflect the result back. */
    ROUTER_POINT RoundToSourceGridYDown() const;
    ROUTER_POINT RoundToTheRight( ROUTER_POINT aDirection ) const;
    ROUTER_POINT RoundToGrid( std::int64_t aHorizontalGrid,
                              std::int64_t aVerticalGrid ) const;
    ROUTER_POINT RoundToTheLeft( ROUTER_POINT aDirection ) const;
    FLOAT_POINT Add( FLOAT_POINT aOther ) const;
    FLOAT_POINT Subtract( FLOAT_POINT aOther ) const;
    double ScalarProduct( FLOAT_POINT aFirst, FLOAT_POINT aSecond ) const;
    FLOAT_POINT ChangeSize( double aNewSize ) const;
    FLOAT_POINT ChangeLength( FLOAT_POINT aToPoint, double aNewLength ) const;
    FLOAT_POINT MiddlePoint( FLOAT_POINT aToPoint ) const;
    /** Source Side encoding: +1=ON_THE_LEFT, -1=ON_THE_RIGHT. */
    int SideOf( FLOAT_POINT aFirst, FLOAT_POINT aSecond ) const;
    FLOAT_POINT Rotate( double aAngle, FLOAT_POINT aPole ) const;
    FLOAT_POINT Turn90Degree( int aFactor ) const;
    FLOAT_POINT Turn90Degree( int aFactor, FLOAT_POINT aPole ) const;
    bool IsContainedInBox( FLOAT_POINT aFirst, FLOAT_POINT aSecond,
                           double aTolerance ) const;
    ROUTER_BOX BoundingBox() const;
    std::vector<FLOAT_POINT> TangentialPoints( FLOAT_POINT aToPoint,
                                               double aDistance ) const;
    std::optional<FLOAT_POINT> LeftTangentialPoint(
            FLOAT_POINT aToPoint, double aDistance ) const;
    std::optional<FLOAT_POINT> RightTangentialPoint(
            FLOAT_POINT aToPoint, double aDistance ) const;
    FLOAT_POINT CircleCenter( FLOAT_POINT aFirst, FLOAT_POINT aSecond ) const;
    bool InsideCircle( FLOAT_POINT aFirst, FLOAT_POINT aSecond,
                       FLOAT_POINT aThird ) const;
};


struct FLOAT_LINE
{
    FLOAT_POINT a;
    FLOAT_POINT b;

    FLOAT_POINT Middle() const;
    FLOAT_LINE Opposite() const;
    FLOAT_LINE AdjustDirection( const FLOAT_LINE& aOther ) const;
    std::optional<FLOAT_POINT> Intersection( const FLOAT_LINE& aOther ) const;
    FLOAT_LINE Translate( double aDistance ) const;
    double SignedDistance( FLOAT_POINT aPoint ) const;
    FLOAT_POINT PerpendicularProjection( FLOAT_POINT aPoint ) const;
    double SegmentDistance( FLOAT_POINT aPoint ) const;
    std::optional<FLOAT_LINE> SegmentProjection( const FLOAT_LINE& aLineSegment ) const;
    std::optional<FLOAT_LINE> SegmentProjection2( const FLOAT_LINE& aLineSegment ) const;
    FLOAT_LINE ShrinkSegment( double aOffset ) const;
    FLOAT_POINT NearestSegmentPoint( FLOAT_POINT aFromPoint ) const;
    std::vector<FLOAT_LINE> DivideSegmentIntoSections( int aCount ) const;
};

} // namespace KICAD_AUTOROUTER
