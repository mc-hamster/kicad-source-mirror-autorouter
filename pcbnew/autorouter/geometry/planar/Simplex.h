/* KiCad, GPL-3.0-or-later. Convex support-line geometry translated from
 * Freerouting Simplex/TileShape at a11c0a42.
 */
#pragma once
#include "Polyline.h"

#include <optional>

namespace KICAD_AUTOROUTER::PLANAR
{
class SIMPLEX
{
public:
    // Strict bounded constructor retained for existing production callers.
    // Use GetInstance when the source operation may normalize to an empty,
    // lower-dimensional, or unbounded simplex.
    explicit SIMPLEX( std::vector<LINE> aBorders );
    static SIMPLEX GetInstance( std::vector<LINE> aBorders );
    static SIMPLEX Empty();
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
    const POINT& Corner( std::size_t i ) const;
    bool CornerIsBounded( std::size_t aIndex ) const;
    bool IsEmpty() const { return m_borders.empty(); }
    bool IsBounded() const;
    int Dimension() const;
    bool IsIntBox() const;
    bool IsIntOctagon() const;
    double Area() const;
    double Circumference() const;
    double MaxWidth() const;
    double MinWidth() const;
    std::optional<ROUTER_BOX> BoundingBox() const;
    int BorderLineIndex( const LINE& aLine ) const;
    SIMPLEX RemoveBorderLine( std::size_t aIndex ) const;
    SIMPLEX Intersection( const SIMPLEX& aOther ) const;
    bool Intersects( const SIMPLEX& aOther ) const;
    /** Cut this (inner) simplex out of aOuter using the pinned source's
     * ordered minimum-distance division lines.  A null result has the same
     * meaning as Java's null return for a lower-dimensional inner shape. */
    std::optional<std::vector<SIMPLEX>> CutoutFrom( const SIMPLEX& aOuter ) const;
    std::optional<SIMPLEX> TranslateBy( ROUTER_POINT aVector ) const;
    std::optional<SIMPLEX> Offset( double aWidth ) const;
    std::optional<SIMPLEX> Enlarge( double aOffset ) const;
    std::pair<double, double> CentreOfGravity() const;
    std::pair<double, double> NearestPointApprox( double aX, double aY ) const;
    std::pair<double, double> NearestBorderPointApprox( double aX, double aY ) const;
    int EqualsCorner( const POINT& aPoint ) const;
    int ContainsOnBorderLineNo( const POINT& aPoint ) const;
    std::vector<int> TouchingSides( const SIMPLEX& aOther ) const;
    double DistanceToTheLeft( const LINE& aLine ) const;
    /** Source Side encoding: +1=ON_THE_LEFT, -1=ON_THE_RIGHT, 0=COLLINEAR. */
    int SideOf( const LINE& aLine ) const;
    bool IsIntersectedInteriorBy( const POINT& aStart, const POINT& aEnd,
                                  const LINE& aLine ) const;
    std::vector<SIMPLEX> DivideIntoSections( double aMaximumSectionWidth ) const;
    std::size_t NextNo( std::size_t aIndex ) const
    { return ( aIndex + 1 ) % m_borders.size(); }
    std::size_t PrevNo( std::size_t aIndex ) const
    { return ( aIndex + m_borders.size() - 1 ) % m_borders.size(); }
    int IndexOfRightMostCorner( const POINT& aFromPoint ) const;
    bool Contains( const POINT& p ) const;
    bool ContainsInside( const POINT& p ) const;
    bool IntersectsSegment( const POLYLINE& aLine, std::size_t aIndex ) const;
    // Source side indices and touching rules; tuples are (polyline line, border).
    std::vector<std::size_t> BorderIntersections( const POLYLINE& aLine, std::size_t aIndex ) const;
    std::vector<std::pair<std::size_t, std::size_t>> EntrancePoints( const POLYLINE& aLine ) const;
    std::vector<POLYLINE> Cutout( const POLYLINE& aLine ) const;
private:
    struct UNCHECKED_TAG {};
    SIMPLEX( std::vector<LINE> aBorders, UNCHECKED_TAG );
    void calculateCorners();

    std::vector<LINE> m_borders;
    std::vector<std::optional<POINT>> m_corners;
};
} // namespace KICAD_AUTOROUTER::PLANAR
