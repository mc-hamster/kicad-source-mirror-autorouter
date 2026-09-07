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
 * Progress and proposal-review dialogs for the native autorouter.
 */

#pragma once

#include <functional>

#include <dialog_shim.h>

#include "AutorouterJob.h"

class wxButton;
class wxCloseEvent;
class wxCommandEvent;
class wxGauge;
class wxStaticText;
class wxTimer;
class wxTimerEvent;


namespace KICAD_AUTOROUTER
{

class DIALOG_AUTOROUTER_PROGRESS : public DIALOG_SHIM
{
public:
    DIALOG_AUTOROUTER_PROGRESS( wxWindow* aParent, AUTOROUTER_JOB& aJob );
    ~DIALOG_AUTOROUTER_PROGRESS() override;

private:
    void updateProgress( wxTimerEvent& aEvent );
    void cancel( wxCommandEvent& aEvent );
    void continueToReview( wxCommandEvent& aEvent );
    void closeWindow( wxCloseEvent& aEvent );

private:
    AUTOROUTER_JOB& m_job;
    wxGauge*        m_gauge = nullptr;
    wxStaticText*   m_stage = nullptr;
    wxStaticText*   m_counts = nullptr;
    wxButton*       m_cancelButton = nullptr;
    wxButton*       m_reviewButton = nullptr;
    wxTimer*        m_timer = nullptr;
    bool            m_cancelling = false;
};


class DIALOG_AUTOROUTER_REVIEW : public DIALOG_SHIM
{
public:
    using DECISION_CALLBACK = std::function<void( bool aAccepted )>;

    DIALOG_AUTOROUTER_REVIEW( wxWindow* aParent, const ROUTING_RESULT& aResult,
                              DECISION_CALLBACK aDecisionCallback );

private:
    void accept( wxCommandEvent& aEvent );
    void reject( wxCommandEvent& aEvent );
    void closeWindow( wxCloseEvent& aEvent );
    void decide( bool aAccepted );

private:
    DECISION_CALLBACK m_decisionCallback;
    wxButton*         m_acceptButton = nullptr;
    bool              m_decided = false;
};

} // namespace KICAD_AUTOROUTER
