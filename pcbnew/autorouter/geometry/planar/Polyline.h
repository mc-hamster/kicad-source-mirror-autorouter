/* KiCad, GPL-3.0-or-later. Freerouting Polyline line-array representation.
 * Lines remain integer; corners are exact rationals, including after cutout.
 */
#pragma once
#include "Line.h"
#include <cmath>

namespace KICAD_AUTOROUTER::PLANAR
{
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
    bool HasSameEndpoints( const POLYLINE& other ) const
    { return !Empty() && !other.Empty() && FirstCorner() == other.FirstCorner() && LastCorner() == other.LastCorner(); }
    POLYLINE Reverse() const;
    POLYLINE Combine( const POLYLINE& aOther ) const;
    double LengthApprox() const;
    std::optional<std::vector<ROUTER_POINT>> IntegralCorners() const;
};
} // namespace KICAD_AUTOROUTER::PLANAR
