/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct C++ translation of Freerouting geometry/planar/LineSegment.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "IntOctagon.h"

namespace KICAD_AUTOROUTER::PLANAR
{

/** A finite segment represented by its support and two closing lines. */
class LINE_SEGMENT
{
public:
    LINE_SEGMENT( LINE aStartLine, LINE aMiddleLine, LINE aEndLine );

    static std::optional<LINE_SEGMENT> FromPolyline( const POLYLINE& aPolyline,
                                                      std::size_t aLineIndex );
    static std::optional<LINE_SEGMENT> FromShape( const SIMPLEX& aShape,
                                                  std::size_t aLineIndex );

    POINT StartPoint() const;
    POINT EndPoint() const;
    std::pair<double, double> StartPointApprox() const;
    std::pair<double, double> EndPointApprox() const;

    const LINE& GetLine() const { return m_middle; }
    const LINE& GetStartClosingLine() const { return m_start; }
    const LINE& GetEndClosingLine() const { return m_end; }

    LINE_SEGMENT Opposite() const;
    POLYLINE ToPolyline() const;
    SIMPLEX ToSimplex() const;
    bool Contains( const POINT& aPoint ) const;
    ROUTER_BOX BoundingBox() const;
    INT_OCTAGON BoundingOctagon() const;
    LINE_SEGMENT ChangeLengthApprox( double aNewLength ) const;

    std::vector<LINE> Intersection( const LINE_SEGMENT& aOther ) const;
    bool Intersects( const LINE_SEGMENT& aOther ) const;
    bool Overlaps( const LINE_SEGMENT& aOther ) const;

    std::vector<ROUTER_POINT> StairApproximation( double aWidth,
                                                  bool aToTheRight ) const;
    std::vector<ROUTER_POINT> StairApproximation45( double aWidth,
                                                    bool aToTheRight ) const;
    std::vector<int> BorderIntersections( const SIMPLEX& aShape ) const;
    LINE_SEGMENT SortEndpointsInXY() const;

private:
    LINE m_start;
    LINE m_middle;
    LINE m_end;
};

} // namespace KICAD_AUTOROUTER::PLANAR
