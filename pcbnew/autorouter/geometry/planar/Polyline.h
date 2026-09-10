/* KiCad, GPL-3.0-or-later. Freerouting Polyline line-array representation.
 * Lines remain integer; corners are exact rationals, including after cutout.
 */
#pragma once
#include "Line.h"
#include <cmath>
#include <optional>

namespace KICAD_AUTOROUTER::PLANAR
{
class SIMPLEX;
class INT_OCTAGON;

class POLYLINE
{
public:
    std::vector<LINE> lines;
    explicit POLYLINE( std::vector<LINE> aLines );
    static POLYLINE FromPoints( const std::vector<ROUTER_POINT>& aPoints );
    bool Empty() const { return lines.size() < 3; }
    POINT Corner( std::size_t i ) const { return lines.at( i ).Intersection( lines.at( i + 1 ) ).value(); }
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
    double LengthApprox() const;
    double LengthApprox( int aRequestedFromCorner, int aRequestedToCorner ) const;
    std::optional<ROUTER_BOX> BoundingBox( int aRequestedFromCorner = 0,
                                           int aRequestedToCorner = -1 ) const;
    std::optional<INT_OCTAGON> BoundingOctagon( int aRequestedFromCorner = 0,
                                                int aRequestedToCorner = -1 ) const;
    std::optional<std::pair<double, double>> NearestPointApprox(
            double aX, double aY ) const;
    bool Contains( const POINT& aPoint ) const;
    std::vector<SIMPLEX> OffsetShapes( int aHalfWidth,
                                       int aRequestedFromLine = 0,
                                       int aRequestedToLine = -1 ) const;
    std::optional<std::vector<ROUTER_POINT>> IntegralCorners() const;
};
} // namespace KICAD_AUTOROUTER::PLANAR
