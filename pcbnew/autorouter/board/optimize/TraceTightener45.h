/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct native counterpart of Freerouting
 * board/optimize/TraceTightener45.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#pragma once

#include "../../AutorouterTypes.h"


namespace KICAD_AUTOROUTER
{

class MAZE_SEARCH_ENGINE;


/** Pull a 45-degree connection tight by transforming its exact support lines.
 *
 * Freerouting does not choose one of the two equal-length Manhattan/diagonal
 * corners globally.  It repeatedly reduces four-corner groups, smooths sharp
 * corners, and repositions one support line.  Keeping that lifecycle separate
 * from maze shoving is important: its deterministic result becomes obstacle
 * geometry for the next routing item.
 */
class TRACE_TIGHTENER_45
{
public:
    static bool PullTight( ROUTING_CONNECTION& aConnection,
                           const MAZE_SEARCH_ENGINE& aSearch );
};

} // namespace KICAD_AUTOROUTER
