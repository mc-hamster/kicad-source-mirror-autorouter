/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct C++ translation of Freerouting board/state/ChangedArea.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include <cstddef>
#include <vector>

#include "../../geometry/planar/IntOctagon.h"


namespace KICAD_AUTOROUTER
{

/** Per-layer octagonal bounds of worker-board mutations.
 *
 * Coordinates are retained as doubles until GetArea() applies the source's
 * outward floor/ceil conversion.  This matters when a rational line
 * intersection is the only point which marks a changed trace corner.
 */
class CHANGED_AREA
{
public:
    explicit CHANGED_AREA( int aLayerCount );

    void Join( double aX, double aY, int aLayer );
    void Join( ROUTER_POINT aPoint, int aLayer );
    void Join( const PLANAR::INT_OCTAGON& aShape, int aLayer );

    PLANAR::INT_OCTAGON GetArea( int aLayer ) const;
    ROUTER_BOX SurroundingBox() const;
    void SetEmpty( int aLayer );

    int LayerCount() const { return static_cast<int>( m_areas.size() ); }

private:
    struct MUTABLE_OCTAGON
    {
        double leftX;
        double bottomY;
        double rightX;
        double topY;
        double upperLeftDiagonalX;
        double lowerRightDiagonalX;
        double lowerLeftDiagonalX;
        double upperRightDiagonalX;

        void SetEmpty();
        bool IsEmpty() const;
        PLANAR::INT_OCTAGON ToInt() const;
    };

    std::vector<MUTABLE_OCTAGON> m_areas;
};

} // namespace KICAD_AUTOROUTER
