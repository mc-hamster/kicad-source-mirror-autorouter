/* KiCad, GPL-3.0-or-later. Bounded, full-dimensional convex support-line
 * slice of Freerouting Simplex/TileShape. Not an unbounded half-plane solver.
 */
#pragma once
#include "Polyline.h"

#include <optional>

namespace KICAD_AUTOROUTER::PLANAR
{
class SIMPLEX
{
public:
    // Borders must already be CCW, irredundant, bounded and convex.
    explicit SIMPLEX( std::vector<LINE> aBorders );
    static SIMPLEX Box( ROUTER_BOX aBox );
    /** Build the full-dimensional L-infinity sweep of an integer segment.
     * This is the conservative convex centre-space used for a trace moving
     * around another diagonal trace; unlike its axis-aligned bounding box it
     * retains the two free wedges beside the segment.  Zero-length segments
     * have no trace direction and must be represented by Box() by callers.
     */
    static std::optional<SIMPLEX> FromExpandedSegment(
            const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
            std::int64_t aChebyshevRadius );
    /** Build a bounded simplex from a strictly convex integer contour and
     * enlarge it by an L-infinity radius.  The square offset is deliberately
     * conservative for the worker's circular copper clearance; callers must
     * still reject non-integral resulting route corners rather than round. */
    static std::optional<SIMPLEX> FromConvexPolygon(
            const std::vector<ROUTER_POINT>& aPolygon,
            std::int64_t aChebyshevOffset = 0 );
    const std::vector<LINE>& Borders() const { return m_borders; }
    const POINT& Corner( std::size_t i ) const { return m_corners.at( i ); }
    bool Contains( const POINT& p ) const;
    bool ContainsInside( const POINT& p ) const;
    bool IntersectsSegment( const POLYLINE& aLine, std::size_t aIndex ) const;
    // Source side indices and touching rules; tuples are (polyline line, border).
    std::vector<std::size_t> BorderIntersections( const POLYLINE& aLine, std::size_t aIndex ) const;
    std::vector<std::pair<std::size_t, std::size_t>> EntrancePoints( const POLYLINE& aLine ) const;
    std::vector<POLYLINE> Cutout( const POLYLINE& aLine ) const;
private:
    std::vector<LINE> m_borders;
    std::vector<POINT> m_corners;
};
} // namespace KICAD_AUTOROUTER::PLANAR
