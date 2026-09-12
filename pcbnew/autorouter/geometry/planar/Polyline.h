/* KiCad, GPL-3.0-or-later. Freerouting Polyline line-array representation.
 * Lines remain integer; corners are exact rationals, including after cutout.
 */
#pragma once
#include "FloatLine.h"
#include "Line.h"
#include <cmath>
#include <memory>
#include <optional>

namespace KICAD_AUTOROUTER::PLANAR
{
class SIMPLEX;
class INT_OCTAGON;
class LINE_SEGMENT;

class POLYLINE
{
public:
    std::vector<LINE> lines;
    explicit POLYLINE( std::vector<LINE> aLines );
    static POLYLINE FromPoints( const std::vector<ROUTER_POINT>& aPoints );
    bool Empty() const { return lines.size() < 3; }
    POINT Corner( std::size_t i ) const { return lines.at( i ).Intersection( lines.at( i + 1 ) ).value(); }
    /** Freerouting Polyline.cornerApprox: intersect the two integer support
     * lines in double precision instead of converting the exact rational
     * corner.  The distinction is observable in outward floor/ceil bounds. */
    FLOAT_POINT CornerApprox( std::size_t aIndex ) const;
    POINT FirstCorner() const { return Corner( 0 ); }
    POINT LastCorner() const { return Corner( lines.size() - 2 ); }
    std::size_t CornerCount() const { return lines.empty() ? 0 : lines.size() - 1; }
    bool IsPoint() const;
    bool IsOrthogonal() const;
    bool IsMultipleOf45Degree() const;
    bool HasSameEndpoints( const POLYLINE& other ) const
    { return !Empty() && !other.Empty() && FirstCorner() == other.FirstCorner() && LastCorner() == other.LastCorner(); }
    POLYLINE Reverse() const;
    POLYLINE Combine( const POLYLINE& aOther ) const;
    std::vector<POLYLINE> Split( std::size_t aLineIndex, const LINE& aEndLine ) const;
    POLYLINE SkipLines( std::size_t aFrom, std::size_t aTo ) const;
    std::optional<POLYLINE> TranslateBy( ROUTER_POINT aVector ) const;
    std::optional<POLYLINE> Turn90Degree( int aFactor, ROUTER_POINT aPole ) const;
    std::optional<POLYLINE> RotateApprox( double aAngle, double aPoleX,
                                          double aPoleY ) const;
    std::optional<POLYLINE> MirrorVertical( ROUTER_POINT aPole ) const;
    std::optional<POLYLINE> MirrorHorizontal( ROUTER_POINT aPole ) const;
    double LengthApprox() const;
    double LengthApprox( int aRequestedFromCorner, int aRequestedToCorner ) const;
    std::optional<ROUTER_BOX> BoundingBox( int aRequestedFromCorner = 0,
                                           int aRequestedToCorner = -1 ) const;
    std::optional<INT_OCTAGON> BoundingOctagon( int aRequestedFromCorner = 0,
                                                int aRequestedToCorner = -1 ) const;
    std::optional<std::pair<double, double>> NearestPointApprox(
            double aX, double aY ) const;
    double Distance( double aX, double aY ) const;
    bool Contains( const POINT& aPoint ) const;
    std::vector<SIMPLEX> OffsetShapes( int aHalfWidth,
                                       int aRequestedFromLine = 0,
                                       int aRequestedToLine = -1 ) const;
    std::optional<SIMPLEX> OffsetShape( int aHalfWidth,
                                        std::size_t aSegmentIndex ) const;
    std::optional<ROUTER_BOX> OffsetBox( int aHalfWidth,
                                         std::size_t aSegmentIndex ) const;
    std::unique_ptr<LINE_SEGMENT> ProjectionLine( const POINT& aPoint ) const;
    std::optional<POLYLINE> Shorten( std::size_t aNewLineCount,
                                      double aLastSegmentLength ) const;
    std::optional<std::vector<ROUTER_POINT>> IntegralCorners() const;
};
} // namespace KICAD_AUTOROUTER::PLANAR
