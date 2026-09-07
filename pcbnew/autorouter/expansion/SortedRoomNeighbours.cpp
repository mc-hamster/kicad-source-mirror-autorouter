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

#include "SortedRoomNeighbours.h"

#include <algorithm>
#include <limits>

namespace KICAD_AUTOROUTER
{

void SORTED_ROOM_NEIGHBOURS::Sort( std::vector<EXPANSION_ROOM*>& aRooms,
                                   const ROUTER_POINT& aOrigin )
{
    auto distance = [&aOrigin]( const EXPANSION_ROOM* aRoom )
    {
        if( !aRoom )
            return std::numeric_limits<long double>::max();

        const ROUTER_BOX& box = aRoom->GetShape();
        const long double dx = static_cast<long double>( box.minX + box.maxX ) / 2 - aOrigin.x;
        const long double dy = static_cast<long double>( box.minY + box.maxY ) / 2 - aOrigin.y;
        return dx * dx + dy * dy;
    };

    std::stable_sort( aRooms.begin(), aRooms.end(),
                      [&]( const EXPANSION_ROOM* aLeft, const EXPANSION_ROOM* aRight )
                      {
                          const long double left = distance( aLeft );
                          const long double right = distance( aRight );
                          if( left != right )
                              return left < right;
                          return ( aLeft ? aLeft->GetId() : 0 ) < ( aRight ? aRight->GetId() : 0 );
                      } );
}

} // namespace KICAD_AUTOROUTER
