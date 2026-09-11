/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * This program source code file is part of KiCad, a free EDA application.
 */

#include "MazeExpansionEngine.h"

#include <algorithm>


namespace KICAD_AUTOROUTER
{

FLOAT_POINT MAZE_EXPANSION_ENGINE::Nearest(
        const PLANAR::INT_OCTAGON& aShape, FLOAT_POINT aFrom )
{
    const long double difference = static_cast<long double>( aFrom.x ) - aFrom.y;
    const long double sum = static_cast<long double>( aFrom.x ) + aFrom.y;
    if( aFrom.x >= aShape.leftX && aFrom.x <= aShape.rightX
        && aFrom.y >= aShape.bottomY && aFrom.y <= aShape.topY
        && difference >= aShape.upperLeftDiagonalX
        && difference <= aShape.lowerRightDiagonalX
        && sum >= aShape.lowerLeftDiagonalX
        && sum <= aShape.upperRightDiagonalX )
    {
        return aFrom;
    }

    FLOAT_POINT result;
    long double best = std::numeric_limits<long double>::infinity();
    for( int index = 0; index < 8; ++index )
    {
        const ROUTER_POINT first = aShape.Corner( index );
        const ROUTER_POINT second = aShape.Corner( ( index + 1 ) % 8 );
        const long double dx = static_cast<long double>( second.x ) - first.x;
        const long double dy = static_cast<long double>( second.y ) - first.y;
        const long double lengthSquared = dx * dx + dy * dy;
        if( lengthSquared <= 0 )
            continue;
        const long double ratio = std::clamp(
                ( ( static_cast<long double>( aFrom.x ) - first.x ) * dx
                  + ( static_cast<long double>( aFrom.y ) - first.y ) * dy )
                        / lengthSquared,
                0.0L, 1.0L );
        const long double x = first.x + ratio * dx;
        const long double y = first.y + ratio * dy;
        const long double distanceX = static_cast<long double>( aFrom.x ) - x;
        const long double distanceY = static_cast<long double>( aFrom.y ) - y;
        const long double distanceSquared = distanceX * distanceX + distanceY * distanceY;
        if( distanceSquared < best )
        {
            best = distanceSquared;
            result = { static_cast<double>( x ), static_cast<double>( y ) };
        }
    }
    return result;
}


FLOAT_POINT MAZE_EXPANSION_ENGINE::Nearest(
        const PLANAR::SIMPLEX& aShape, FLOAT_POINT aFrom )
{
    if( aShape.SideOfBorder( aFrom, 1e-9 ) <= 0 )
        return aFrom;
    const auto nearest = aShape.NearestPointApprox( aFrom.x, aFrom.y );
    return { nearest.first, nearest.second };
}

} // namespace KICAD_AUTOROUTER
