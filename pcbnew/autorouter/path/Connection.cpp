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

#include "Connection.h"

#include <cmath>

namespace KICAD_AUTOROUTER
{

namespace
{

double distance( const ROUTER_POINT& aLeft, const ROUTER_POINT& aRight )
{
    const long double dx = static_cast<long double>( aLeft.x ) - aRight.x;
    const long double dy = static_cast<long double>( aLeft.y ) - aRight.y;
    return std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
}

} // namespace


CONNECTION CONNECTION::FromRoute( const ROUTING_CONNECTION& aRoute )
{
    CONNECTION result;
    result.m_netCode = aRoute.netCode;
    result.m_complete = aRoute.complete;
    // Each native edge materializes as one trace or via item.  Counting
    // nodes added a fictitious item to every connection and inflated the
    // source detour formula by 0.1 even for one straight trace.
    result.m_itemCount = aRoute.nodes.empty() ? 0 : aRoute.nodes.size() - 1;

    if( !aRoute.nodes.empty() )
    {
        result.m_startPoint = aRoute.nodes.front().point;
        result.m_startLayer = aRoute.nodes.front().layer;
        result.m_endPoint = aRoute.nodes.back().point;
        result.m_endLayer = aRoute.nodes.back().layer;
    }

    for( std::size_t index = 1; index < aRoute.nodes.size(); ++index )
    {
        if( aRoute.nodes[index - 1].layer == aRoute.nodes[index].layer )
            result.m_traceLength += distance( aRoute.nodes[index - 1].point,
                                              aRoute.nodes[index].point );
    }

    return result;
}


double CONNECTION::Detour() const
{
    if( m_itemCount == 0 )
        return 0.0;

    constexpr double detourAdd = 100.0;
    constexpr double detourItemCost = 0.1;
    const double minimum = distance( m_startPoint, m_endPoint );
    return ( m_traceLength + detourAdd ) / ( minimum + detourAdd )
           + detourItemCost * static_cast<double>( m_itemCount - 1 );
}

} // namespace KICAD_AUTOROUTER
