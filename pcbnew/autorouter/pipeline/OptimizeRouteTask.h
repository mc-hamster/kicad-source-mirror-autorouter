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

#pragma once

#include <functional>
#include <optional>
#include <utility>

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

/** Data-only optimization candidate corresponding to Freerouting's task. */
class OPTIMIZE_ROUTE_TASK
{
public:
    using ROUTE_FUNCTION = std::function<std::optional<ROUTING_CONNECTION>()>;

    explicit OPTIMIZE_ROUTE_TASK( ROUTE_FUNCTION aRouteFunction ) :
            m_routeFunction( std::move( aRouteFunction ) )
    {
    }

    void Run()
    {
        if( m_routeFunction )
            m_result = m_routeFunction();
    }

    const std::optional<ROUTING_CONNECTION>& Result() const { return m_result; }

private:
    ROUTE_FUNCTION                    m_routeFunction;
    std::optional<ROUTING_CONNECTION> m_result;
};

} // namespace KICAD_AUTOROUTER
