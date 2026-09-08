/* KiCad, GPL-3.0-or-later. Explicit native-IU / reference-cost boundary. */
#pragma once

#include "DestinationDistance.h"
#include <stdexcept>

namespace KICAD_AUTOROUTER
{
/** Keep the reference's finite EMPTY-box sentinel out of real KiCad geometry.
 * Like Structure.create_board, reduce coordinates by powers of ten until
 * 5 * maxAbsCoordinate < Limits.CRIT_INT. The room graph stays in KiCad IU;
 * only heuristic queries and via costs cross this boundary, and estimated
 * costs are converted back before entering the common g/f queue.
 * This is an explicit adapter, not a full Specctra geometry conversion.
 */
class ROOM_COST_SPACE
{
public:
    explicit ROOM_COST_SPACE( ROUTER_BOX aBounds )
    {
        if( INT_BOX::Dimension( aBounds ) < 0 )
            throw std::invalid_argument( "Room cost space requires nonempty bounds" );
        const double maxCoordinate = std::max( {
                std::abs( static_cast<double>( aBounds.minX ) ),
                std::abs( static_cast<double>( aBounds.minY ) ),
                std::abs( static_cast<double>( aBounds.maxX ) ),
                std::abs( static_cast<double>( aBounds.maxY ) ) } );
        while( 5 * maxCoordinate / m_scale >= DESTINATION_DISTANCE::REFERENCE_COORDINATE_LIMIT )
            m_scale *= 10;
    }

    FLOAT_POINT ToReference( FLOAT_POINT aPoint ) const
    {
        return { aPoint.x / m_scale, aPoint.y / m_scale };
    }

    ROUTER_BOX ToReference( ROUTER_BOX aBox ) const
    {
        if( INT_BOX::Dimension( aBox ) < 0 )
            throw std::invalid_argument( "Cannot convert an empty destination region" );
        // Enclose rather than shrink destination regions at the unit boundary.
        const auto ll = ToReference( FLOAT_POINT{ static_cast<double>( aBox.minX ),
                                                  static_cast<double>( aBox.minY ) } ).BoundingBox();
        const auto ur = ToReference( FLOAT_POINT{ static_cast<double>( aBox.maxX ),
                                                  static_cast<double>( aBox.maxY ) } ).BoundingBox();
        return { ll.minX, ll.minY, ur.maxX, ur.maxY };
    }

    double ToReferenceCost( double aCost ) const { return aCost / m_scale; }
    double ToNativeCost( double aCost ) const { return aCost * m_scale; }
    double Scale() const { return m_scale; }

private:
    double m_scale = 1;
};
} // namespace KICAD_AUTOROUTER
