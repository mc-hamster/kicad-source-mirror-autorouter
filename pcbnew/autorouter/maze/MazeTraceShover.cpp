/*
 * This program source code file is part of KiCad, a free EDA application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#include "MazeTraceShover.h"

#include <cmath>

#include "MazeSearchEngine.h"


namespace KICAD_AUTOROUTER
{

namespace
{

double lengthBetween( const ROUTING_CONNECTION& aConnection, std::size_t aFirst,
                      std::size_t aLast )
{
    double length = 0.0;

    for( std::size_t index = aFirst + 1; index <= aLast; ++index )
    {
        const long double dx = static_cast<long double>(
                                        aConnection.nodes[index].point.x )
                                - aConnection.nodes[index - 1].point.x;
        const long double dy = static_cast<long double>(
                                        aConnection.nodes[index].point.y )
                                - aConnection.nodes[index - 1].point.y;
        length += std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
    }

    return length;
}

} // namespace


bool MAZE_TRACE_SHOVER::Shorten( ROUTING_CONNECTION& aConnection,
                                 const MAZE_SEARCH_ENGINE& aSearch )
{
    bool changedAny = false;
    bool changed = true;

    while( changed && aConnection.nodes.size() > 2 )
    {
        changed = false;

        for( std::size_t first = 0;
             first + 2 < aConnection.nodes.size() && !changed; ++first )
        {
            for( std::size_t last = first + 2; last < aConnection.nodes.size(); ++last )
            {
                const ROUTER_NODE& start = aConnection.nodes[first];
                const ROUTER_NODE& end = aConnection.nodes[last];
                const ROUTING_EDGE_STYLE* style = aConnection.edgeStyles.empty()
                        ? nullptr : &aConnection.edgeStyles[first];

                if( start.layer != end.layer
                    || !CanCollapseRouteEdges( aConnection, first, last - 1 )
                    || !aSearch.CanInsertSegment( aConnection.netCode, start, end, style ) )
                {
                    continue;
                }

                const double oldLength = lengthBetween( aConnection, first, last );
                const long double dx = static_cast<long double>( end.point.x ) - start.point.x;
                const long double dy = static_cast<long double>( end.point.y ) - start.point.y;
                const double directLength = std::sqrt( static_cast<double>( dx * dx + dy * dy ) );

                if( directLength + 1.0 < oldLength )
                {
                    if( !CollapseRouteNodes( aConnection, first, last ) )
                        continue;
                    changed = true;
                    changedAny = true;
                    break;
                }
            }
        }
    }

    for( std::size_t index = 1; index + 1 < aConnection.nodes.size(); )
    {
        const ROUTER_NODE& previous = aConnection.nodes[index - 1];
        const ROUTER_NODE& current = aConnection.nodes[index];
        const ROUTER_NODE& next = aConnection.nodes[index + 1];
        const ROUTING_EDGE_STYLE* style = aConnection.edgeStyles.empty()
                ? nullptr : &aConnection.edgeStyles[index - 1];

        if( previous.point == current.point && current.point == next.point
            && previous.layer != next.layer
            && CanCollapseRouteEdges( aConnection, index - 1, index )
            && aSearch.CanInsertSegment( aConnection.netCode, previous, next, style ) )
        {
            if( !CollapseRouteNodes( aConnection, index - 1, index + 1 ) )
            {
                ++index;
                continue;
            }
            changedAny = true;
            continue;
        }

        ++index;
    }

    return changedAny;
}

} // namespace KICAD_AUTOROUTER
