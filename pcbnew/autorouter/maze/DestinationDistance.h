/*
 * KiCad, GPL-3.0-or-later. Direct translation of Freerouting
 * autoroute/maze/DestinationDistance.java at a11c0a42.
 */
#pragma once

#include "AutorouteControl.h"
#include "../geometry/planar/FloatLine.h"

namespace KICAD_AUTOROUTER
{
/** Source component/solder/inner-box estimate in REFERENCE routing coordinates.
 * The room adapter converts KiCad IU at the boundary; the raster fallback does
 * not use this class. See DESTINATION-DISTANCE-PARITY.md for tested scope.
 */
class DESTINATION_DISTANCE
{
public:
    using EXPANSION_COST_FACTOR = AUTOROUTE_CONTROL::EXPANSION_COST_FACTOR;
    static constexpr int REFERENCE_COORDINATE_LIMIT = 33554432; // Limits.CRIT_INT
    static constexpr double REFERENCE_MAX_COST = 2147483647.0; // Integer.MAX_VALUE

    DESTINATION_DISTANCE( const std::vector<EXPANSION_COST_FACTOR>& traceCosts,
                          const std::vector<bool>& layerActive,
                          double minNormalViaCost, double minCheapViaCost );
    void Join( ROUTER_BOX box, int layer );
    double Calculate( FLOAT_POINT point, int layer ) const;
    double Calculate( ROUTER_BOX box, int layer ) const;
    double CalculateCheapDistance( ROUTER_BOX box, int layer ) const;

private:
    // Passing the cost per invocation is equivalent to Java's save/swap/restore,
    // without mutable temporary state during a const query.
    double calculate( ROUTER_BOX box, int layer, double minNormalViaCost ) const;
    static constexpr ROUTER_BOX EMPTY{ REFERENCE_COORDINATE_LIMIT, REFERENCE_COORDINATE_LIMIT,
                                      -REFERENCE_COORDINATE_LIMIT, -REFERENCE_COORDINATE_LIMIT };
    std::vector<EXPANSION_COST_FACTOR> m_traceCosts;
    std::vector<bool> m_layerActive;
    int m_layerCount = 0;
    int m_activeLayerCount = 0;
    double m_minCheapViaCost = 0;
    double m_minNormalViaCost = 0;
    double m_minComponentSideTraceCost = 0;
    double m_maxComponentSideTraceCost = 0;
    double m_minSolderSideTraceCost = 0;
    double m_maxSolderSideTraceCost = 0;
    double m_maxInnerSideTraceCost = 0;
    double m_minComponentInnerTraceCost = 0;
    double m_minSolderInnerTraceCost = 0;
    double m_minComponentSolderInnerTraceCost = 0;
    ROUTER_BOX m_componentSideBox = EMPTY;
    ROUTER_BOX m_solderSideBox = EMPTY;
    ROUTER_BOX m_innerSideBox = EMPTY;
    bool m_boxIsEmpty = true;
    bool m_componentSideBoxIsEmpty = true;
    bool m_solderSideBoxIsEmpty = true;
    bool m_innerSideBoxIsEmpty = true;
};
} // namespace KICAD_AUTOROUTER
