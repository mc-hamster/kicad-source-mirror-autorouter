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
 * PCB Editor tool entry point for the native full-board autorouter.
 */

#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <tools/pcb_tool_base.h>

#include "AutorouterTypes.h"
#include "AutorouterPreviewItem.h"
#include "AutorouterDialogs.h"
#include "AutorouterJob.h"
#include "board/KicadBoardAdapter.h"


class BOARD_ITEM;

namespace KIGFX
{
class VIEW_GROUP;
}


namespace KICAD_AUTOROUTER
{

class AUTOROUTER_TOOL : public PCB_TOOL_BASE
{
public:
    AUTOROUTER_TOOL();
    ~AUTOROUTER_TOOL() override;

    bool Init() override;
    void Reset( RESET_REASON aReason ) override;

    int AutorouteBoard( const TOOL_EVENT& aEvent );

protected:
    void setTransitions() override;

private:
    void showProposal( const ROUTING_RESULT& aResult );
    void acceptProposal();
    void rejectProposal();
    void clearVisualProposal();
    void restoreHiddenItems();
    void reportFailure( const ROUTING_RESULT& aResult ) const;

private:
    std::unique_ptr<AUTOROUTER_JOB> m_job;
    std::optional<ROUTING_RESULT>   m_result;
    std::unique_ptr<DIALOG_AUTOROUTER_REVIEW> m_reviewDialog;

    std::unique_ptr<KIGFX::VIEW_GROUP> m_previewGroup;
    std::vector<std::unique_ptr<AUTOROUTER_PREVIEW_ITEM>> m_previewItems;
    std::vector<std::unique_ptr<BOARD_ITEM>> m_previewBoardItems;
    int m_proposalBoardTimestamp = -1;

    struct HIDDEN_ITEM
    {
        BOARD_ITEM* item = nullptr;
        bool        wasVisible = true;
    };

    std::vector<HIDDEN_ITEM> m_hiddenItems;
    bool m_proposalActive = false;
};

} // namespace KICAD_AUTOROUTER
