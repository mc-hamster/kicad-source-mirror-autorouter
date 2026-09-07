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

#include "AutorouteAirlineCalculator.h"

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


double AUTOROUTE_AIRLINE_CALCULATOR::ConnectionLength(
        const BOARD_SNAPSHOT& aBoard, const ROUTING_CONNECTION& aConnection )
{
    if( aConnection.fromPadIndex >= aBoard.pads.size()
        || aConnection.toPadIndex >= aBoard.pads.size() )
    {
        return 0.0;
    }

    return distance( aBoard.pads[aConnection.fromPadIndex].position,
                     aBoard.pads[aConnection.toPadIndex].position );
}


double AUTOROUTE_AIRLINE_CALCULATOR::TotalLength(
        const BOARD_SNAPSHOT& aBoard, const std::vector<ROUTING_CONNECTION>& aConnections )
{
    double result = 0.0;

    for( const ROUTING_CONNECTION& connection : aConnections )
    {
        if( connection.complete )
            result += ConnectionLength( aBoard, connection );
    }

    return result;
}

} // namespace KICAD_AUTOROUTER
