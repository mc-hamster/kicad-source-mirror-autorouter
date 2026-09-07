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

#include "../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

/** Algorithm-side worker entry point corresponding to Freerouting's class. */
class BATCH_AUTOROUTER_THREAD
{
public:
    static ROUTING_RESULT Run( const BOARD_SNAPSHOT& aBoard,
                               const AUTOROUTER_SETTINGS& aSettings,
                               const ROUTER_CANCEL_CALLBACK& aCancel,
                               const ROUTER_PROGRESS_CALLBACK& aProgress );
};

} // namespace KICAD_AUTOROUTER
