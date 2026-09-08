/*
 * KiCad, GPL-3.0-or-later. Direct translation of Freerouting
 * autoroute/maze/DestinationDistance.java at a11c0a42.
 * Keep source branch and evaluation order, including inactive-layer defaults.
 */
#if defined( __clang__ )
// Java evaluates multiply and add separately. Contracting them to FMA changes
// frontier tie values (first reproduced by destination oracle record 26).
// Keep this before the inline geometry includes; CMake also covers GCC/MSVC.
#pragma clang fp contract(off)
#endif
#include "DestinationDistance.h"
#include <stdexcept>

namespace KICAD_AUTOROUTER
{
DESTINATION_DISTANCE::DESTINATION_DISTANCE( const std::vector<EXPANSION_COST_FACTOR>& traceCosts,
        const std::vector<bool>& layerActive, double minNormalViaCost, double minCheapViaCost )
{
    if( layerActive.empty() || traceCosts.size() != layerActive.size() )
        throw std::invalid_argument( "DestinationDistance requires matching nonempty layer arrays" );
    m_traceCosts = traceCosts;
    m_layerActive = layerActive;
    m_layerCount = m_layerActive.size();
    m_minNormalViaCost = minNormalViaCost;
    m_minCheapViaCost = minCheapViaCost;
    int currentActiveLayerCount = 0;
    for( int ind = 0; ind < m_layerCount; ind++ )
    {
        if( m_layerActive[ind] )
        {
            ++currentActiveLayerCount;
        }
    }
    m_activeLayerCount = currentActiveLayerCount;

    if( m_layerActive[0] )
    {
        if( m_traceCosts[0].horizontal < m_traceCosts[0].vertical )
        {
            m_minComponentSideTraceCost = m_traceCosts[0].horizontal;
            m_maxComponentSideTraceCost = m_traceCosts[0].vertical;
        }
        else
        {
            m_minComponentSideTraceCost = m_traceCosts[0].vertical;
            m_maxComponentSideTraceCost = m_traceCosts[0].horizontal;
        }
    }

    if( m_layerActive[m_layerCount - 1] )
    {
        const EXPANSION_COST_FACTOR& currentTraceCost = m_traceCosts[m_layerCount - 1];

        if( currentTraceCost.horizontal < currentTraceCost.vertical )
        {
            m_minSolderSideTraceCost = currentTraceCost.horizontal;
            m_maxSolderSideTraceCost = currentTraceCost.vertical;
        }
        else
        {
            m_minSolderSideTraceCost = currentTraceCost.vertical;
            m_maxSolderSideTraceCost = currentTraceCost.horizontal;
        }
    }

    // Note: for inner layers we assume, that cost in preferred direction is 1
    m_maxInnerSideTraceCost = std::min( m_maxComponentSideTraceCost, m_maxSolderSideTraceCost );
    for( int ind2 = 1; ind2 < m_layerCount - 1; ind2++ )
    {
        if( !m_layerActive[ind2] )
        {
            continue;
        }
        double currentMaxCost = std::max( m_traceCosts[ind2].horizontal, m_traceCosts[ind2].vertical );

        m_maxInnerSideTraceCost = std::min( m_maxInnerSideTraceCost, currentMaxCost );
    }
    m_minComponentInnerTraceCost = std::min( m_minComponentSideTraceCost, m_maxInnerSideTraceCost );
    m_minSolderInnerTraceCost = std::min( m_minSolderSideTraceCost, m_maxInnerSideTraceCost );
    m_minComponentSolderInnerTraceCost =
            std::min( m_minComponentInnerTraceCost, m_minSolderInnerTraceCost );
}

void DESTINATION_DISTANCE::Join( ROUTER_BOX box, int layer )
{
    if( layer < 0 || layer >= m_layerCount )
        throw std::out_of_range( "DestinationDistance layer" );
    // Raw min/max union deliberately matches IntBox.union, including EMPTY.
    auto join = [&]( ROUTER_BOX& target )
    {
        target = { std::min( target.minX, box.minX ), std::min( target.minY, box.minY ),
                   std::max( target.maxX, box.maxX ), std::max( target.maxY, box.maxY ) };
    };
    if( layer == 0 )
    {
        join( m_componentSideBox );
        m_componentSideBoxIsEmpty = false;
    }
    else if( layer == m_layerCount - 1 )
    {
        join( m_solderSideBox );
        m_solderSideBoxIsEmpty = false;
    }
    else
    {
        join( m_innerSideBox );
        m_innerSideBoxIsEmpty = false;
    }
    m_boxIsEmpty = false;
}

double DESTINATION_DISTANCE::Calculate( FLOAT_POINT point, int layer ) const
{
    return Calculate( point.BoundingBox(), layer );
}

double DESTINATION_DISTANCE::Calculate( ROUTER_BOX box, int layer ) const
{
    return calculate( box, layer, m_minNormalViaCost );
}

double DESTINATION_DISTANCE::CalculateCheapDistance( ROUTER_BOX box, int layer ) const
{
    return calculate( box, layer, m_minCheapViaCost );
}

double DESTINATION_DISTANCE::calculate( ROUTER_BOX box, int layer, double minNormalViaCost ) const
{
    if( layer < 0 || layer >= m_layerCount )
        throw std::out_of_range( "DestinationDistance layer" );

    if( m_boxIsEmpty )
    {
        return REFERENCE_MAX_COST;
    }

    double componentSideDeltaX;
    double componentSideDeltaY;

    if( box.minX > m_componentSideBox.maxX )
    {
        componentSideDeltaX = box.minX - m_componentSideBox.maxX;
    }
    else if( box.maxX < m_componentSideBox.minX )
    {
        componentSideDeltaX = m_componentSideBox.minX - box.maxX;
    }
    else
    {
        componentSideDeltaX = 0;
    }

    if( box.minY > m_componentSideBox.maxY )
    {
        componentSideDeltaY = box.minY - m_componentSideBox.maxY;
    }
    else if( box.maxY < m_componentSideBox.minY )
    {
        componentSideDeltaY = m_componentSideBox.minY - box.maxY;
    }
    else
    {
        componentSideDeltaY = 0;
    }

    double solderSideDeltaX;
    double solderSideDeltaY;

    if( box.minX > m_solderSideBox.maxX )
    {
        solderSideDeltaX = box.minX - m_solderSideBox.maxX;
    }
    else if( box.maxX < m_solderSideBox.minX )
    {
        solderSideDeltaX = m_solderSideBox.minX - box.maxX;
    }
    else
    {
        solderSideDeltaX = 0;
    }

    if( box.minY > m_solderSideBox.maxY )
    {
        solderSideDeltaY = box.minY - m_solderSideBox.maxY;
    }
    else if( box.maxY < m_solderSideBox.minY )
    {
        solderSideDeltaY = m_solderSideBox.minY - box.maxY;
    }
    else
    {
        solderSideDeltaY = 0;
    }

    double innerSideDeltaX;
    double innerSideDeltaY;

    if( box.minX > m_innerSideBox.maxX )
    {
        innerSideDeltaX = box.minX - m_innerSideBox.maxX;
    }
    else if( box.maxX < m_innerSideBox.minX )
    {
        innerSideDeltaX = m_innerSideBox.minX - box.maxX;
    }
    else
    {
        innerSideDeltaX = 0;
    }

    if( box.minY > m_innerSideBox.maxY )
    {
        innerSideDeltaY = box.minY - m_innerSideBox.maxY;
    }
    else if( box.maxY < m_innerSideBox.minY )
    {
        innerSideDeltaY = m_innerSideBox.minY - box.maxY;
    }
    else
    {
        innerSideDeltaY = 0;
    }

    double componentSideMaxDelta;
    double componentSideMinDelta;

    if( componentSideDeltaX > componentSideDeltaY )
    {
        componentSideMaxDelta = componentSideDeltaX;
        componentSideMinDelta = componentSideDeltaY;
    }
    else
    {
        componentSideMaxDelta = componentSideDeltaY;
        componentSideMinDelta = componentSideDeltaX;
    }

    double solderSideMaxDelta;
    double solderSideMinDelta;

    if( solderSideDeltaX > solderSideDeltaY )
    {
        solderSideMaxDelta = solderSideDeltaX;
        solderSideMinDelta = solderSideDeltaY;
    }
    else
    {
        solderSideMaxDelta = solderSideDeltaY;
        solderSideMinDelta = solderSideDeltaX;
    }

    double innerSideMaxDelta;
    double innerSideMinDelta;

    if( innerSideDeltaX > innerSideDeltaY )
    {
        innerSideMaxDelta = innerSideDeltaX;
        innerSideMinDelta = innerSideDeltaY;
    }
    else
    {
        innerSideMaxDelta = innerSideDeltaY;
        innerSideMinDelta = innerSideDeltaX;
    }

    double result = REFERENCE_MAX_COST;

    if( layer == 0 )
    { // calculate shortest distance to component side box
        // calculate one layer distance

        if( !m_componentSideBoxIsEmpty )
        {
            result =
                    INT_BOX::WeightedDistance( box,
                            m_componentSideBox, m_traceCosts[0].horizontal, m_traceCosts[0].vertical );
        }

        if( m_activeLayerCount <= 1 )
        {
            return result;
        }

        // calculate two layer distance on component and solder side

        double tmpDistance;
        if( m_minSolderSideTraceCost < m_minComponentSideTraceCost )
        {
            tmpDistance =
                    m_minSolderSideTraceCost * solderSideMaxDelta
                            + m_minComponentSideTraceCost * solderSideMinDelta
                            + minNormalViaCost;
        }
        else
        {
            tmpDistance =
                    m_minComponentSideTraceCost * solderSideMaxDelta
                            + m_minSolderSideTraceCost * solderSideMinDelta
                            + minNormalViaCost;
        }

        result = std::min( result, tmpDistance );

        // calculate two layer distance on component and solde side
        // with two vias

        tmpDistance =
                componentSideMaxDelta
                        + componentSideMinDelta * m_minComponentInnerTraceCost
                        + 2 * minNormalViaCost;

        result = std::min( result, tmpDistance );

        if( m_activeLayerCount == 2 )
        {
            return result;
        }

        // calculate two layer distance on component side and an inner side

        tmpDistance =
                innerSideMaxDelta + innerSideMinDelta * m_minComponentInnerTraceCost + minNormalViaCost;

        result = std::min( result, tmpDistance );

        // calculate three layer distance

        tmpDistance =
                solderSideMaxDelta
                        + +m_minComponentSolderInnerTraceCost * solderSideMinDelta
                        + 2 * minNormalViaCost;
        result = std::min( result, tmpDistance );

        tmpDistance = componentSideMaxDelta + componentSideMinDelta + 2 * minNormalViaCost;
        result = std::min( result, tmpDistance );

        if( m_activeLayerCount == 3 )
        {
            return result;
        }

        tmpDistance = innerSideMaxDelta + innerSideMinDelta + 2 * minNormalViaCost;

        result = std::min( result, tmpDistance );

        // calculate four layer distance

        tmpDistance = solderSideMaxDelta + solderSideMinDelta + 3 * minNormalViaCost;

        return std::min( result, tmpDistance );
    }
    if( layer == m_layerCount - 1 )
    { // calculate the shortest distance to solder side box
        // calculate one layer distance

        if( !m_solderSideBoxIsEmpty )
        {
            result =
                    INT_BOX::WeightedDistance( box,
                            m_solderSideBox, m_traceCosts[layer].horizontal, m_traceCosts[layer].vertical );
        }

        // calculate two layer distance
        double tmpDistance;
        if( m_minComponentSideTraceCost < m_minSolderSideTraceCost )
        {
            tmpDistance =
                    m_minComponentSideTraceCost * componentSideMaxDelta
                            + m_minSolderSideTraceCost * componentSideMinDelta
                            + minNormalViaCost;
        }
        else
        {
            tmpDistance =
                    m_minSolderSideTraceCost * componentSideMaxDelta
                            + m_minComponentSideTraceCost * componentSideMinDelta
                            + minNormalViaCost;
        }
        result = std::min( result, tmpDistance );
        tmpDistance =
                solderSideMaxDelta + solderSideMinDelta * m_minSolderInnerTraceCost + 2 * minNormalViaCost;
        result = std::min( result, tmpDistance );
        if( m_activeLayerCount <= 2 )
        {
            return result;
        }
        tmpDistance =
                innerSideMinDelta * m_minSolderInnerTraceCost + innerSideMaxDelta + minNormalViaCost;
        result = std::min( result, tmpDistance );

        // calculate three layer distance

        tmpDistance =
                componentSideMaxDelta
                        + m_minComponentSolderInnerTraceCost * componentSideMinDelta
                        + 2 * minNormalViaCost;
        result = std::min( result, tmpDistance );
        tmpDistance = solderSideMaxDelta + solderSideMinDelta + 2 * minNormalViaCost;
        result = std::min( result, tmpDistance );
        if( m_activeLayerCount == 3 )
        {
            return result;
        }
        tmpDistance = innerSideMaxDelta + innerSideMinDelta + 2 * minNormalViaCost;
        result = std::min( result, tmpDistance );

        // calculate four layer distance

        tmpDistance = componentSideMaxDelta + componentSideMinDelta + 3 * minNormalViaCost;
        return std::min( result, tmpDistance );
    }

    // calculate distance to inner layer box

    // calculate one layer distance

    if( !m_innerSideBoxIsEmpty )
    {
        result =
                INT_BOX::WeightedDistance( box,
                        m_innerSideBox, m_traceCosts[layer].horizontal, m_traceCosts[layer].vertical );
    }

    // calculate two layer distance

    double tmpDistance = innerSideMaxDelta + innerSideMinDelta + minNormalViaCost;

    result = std::min( result, tmpDistance );
    tmpDistance =
            componentSideMaxDelta
                    + componentSideMinDelta * m_minComponentInnerTraceCost
                    + minNormalViaCost;
    result = std::min( result, tmpDistance );
    tmpDistance =
            solderSideMaxDelta + solderSideMinDelta * m_minSolderInnerTraceCost + minNormalViaCost;
    result = std::min( result, tmpDistance );

    // calculate three layer distance

    tmpDistance = componentSideMaxDelta + componentSideMinDelta + 2 * minNormalViaCost;
    result = std::min( result, tmpDistance );
    tmpDistance = solderSideMaxDelta + solderSideMinDelta + 2 * minNormalViaCost;
    return std::min( result, tmpDistance );
}
} // namespace KICAD_AUTOROUTER
