/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Host-side validation boundary; not part of the Freerouting algorithm port.
 */
#pragma once

#include <memory>
#include "../AutorouterTypes.h"

class BOARD;

namespace KICAD_AUTOROUTER
{
/** Owns an isolated KiCad board for refill, DRC and bounded connection repair.
 * Construct on the host thread; Run accesses only the private copy, never the editor.
 * A session is single-use, including after failure or cancellation.
 * No files are written, no Java/DSN/SES is involved and no editor undo state changes.
 */
class KICAD_ROUTING_SESSION
{
public:
    explicit KICAD_ROUTING_SESSION( BOARD& aSource );
    ~KICAD_ROUTING_SESSION();
    ROUTING_RESULT Run( const AUTOROUTER_SETTINGS& aSettings,
                        const ROUTER_CANCEL_CALLBACK& aCancel = {},
                        const ROUTER_PROGRESS_CALLBACK& aProgress = {} );
private:
    struct IMPL;
    std::unique_ptr<IMPL> m_impl;
};
}
