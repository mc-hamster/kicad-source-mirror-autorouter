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
 * This program source code file is part of KiCad, a free EDA software.
 */

#include "AutorouterTool.h"

#include "DialogAutorouter.h"
#include "board/KicadRoutingSession.h"

#include <algorithm>

#include <board_commit.h>
#include <confirm.h>
#include <pcb_track.h>
#include <tool/tool_event.h>
#include <tools/pcb_actions.h>
#include <view/view_group.h>
#include <wx/msgdlg.h>


namespace KICAD_AUTOROUTER
{

AUTOROUTER_TOOL::AUTOROUTER_TOOL() :
        PCB_TOOL_BASE( "pcbnew.Autorouter" )
{
}


AUTOROUTER_TOOL::~AUTOROUTER_TOOL()
{
    Reset( RESET_REASON::MODEL_RELOAD );
}


bool AUTOROUTER_TOOL::Init()
{
    return true;
}


void AUTOROUTER_TOOL::Reset( RESET_REASON aReason )
{
    (void) aReason;

    if( m_job )
    {
        m_job->Cancel();
        m_job->Join();
        m_job.reset();
    }

    if( m_proposalActive )
        rejectProposal();
    else
        clearVisualProposal();

    m_previewBoardItems.clear();
    m_result.reset();
    m_proposalBoardTimestamp = -1;

    if( m_reviewDialog )
    {
        m_reviewDialog->Hide();
        m_reviewDialog.reset();
    }
}


int AUTOROUTER_TOOL::AutorouteBoard( const TOOL_EVENT& )
{
    if( m_job && !m_job->IsFinished() )
        return 0;

    if( m_proposalActive )
        rejectProposal();

    KICAD_BOARD_ADAPTER adapter( board() );
    AUTOROUTER_SETTINGS settings = adapter.CreateDefaultSettings();
    DIALOG_AUTOROUTER_SETTINGS settingsDialog( frame(), board(), settings );

    if( settingsDialog.ShowModal() != wxID_OK )
        return 0;

    settings = settingsDialog.GetSettings();
    const std::shared_ptr<const BOARD_SNAPSHOT> snapshot = adapter.CreateSnapshot( settings );

    // The live ratsnest can be stale until refill. Let the private host
    // session decide whether work remains even when it initially has no edges.
    const bool hasRoutableNet = snapshot
                                       && std::any_of(
                                               snapshot->nets.begin(), snapshot->nets.end(),
                                               []( const ROUTING_NET& aNet )
                                               {
                                                   return aNet.routable;
                                               } );

    if( !snapshot || !hasRoutableNet )
    {
        wxMessageBox( _( "No routable nets were found on the board." ), _( "Autoroute Board" ),
                      wxOK | wxICON_INFORMATION, frame() );
        return 0;
    }

    m_proposalBoardTimestamp = snapshot->sourceBoardTimestamp;

    std::unique_ptr<KICAD_ROUTING_SESSION> hostSession;
    try { hostSession = std::make_unique<KICAD_ROUTING_SESSION>( *board() ); }
    catch( const std::exception& error )
    {
        wxMessageBox( wxString::FromUTF8( error.what() ), _( "Autoroute Board" ),
                      wxOK | wxICON_ERROR, frame() );
        m_proposalBoardTimestamp = -1;
        return 0;
    }
    m_job = std::make_unique<AUTOROUTER_JOB>( snapshot, std::move( settings ), std::move( hostSession ) );
    m_job->Start();

    DIALOG_AUTOROUTER_PROGRESS progressDialog( frame(), *m_job );
    const int progressResult = progressDialog.ShowModal();
    m_job->Join();

    // A completed job is not implicitly a proposal.  The user must choose
    // Review proposal; closing/cancelling the progress dialog discards the
    // worker result and leaves the board untouched.
    if( progressResult != wxID_OK )
    {
        m_job.reset();
        m_proposalBoardTimestamp = -1;
        return 0;
    }

    const std::optional<ROUTING_RESULT> result = m_job->GetResult();
    m_job.reset();

    if( !result || result->cancelled )
    {
        m_proposalBoardTimestamp = -1;
        if( result && !result->message.empty() )
            wxMessageBox( wxString::FromUTF8( result->message.c_str() ), _( "Autoroute Board" ),
                          wxOK | wxICON_INFORMATION, frame() );

        return 0;
    }

    if( !result->complete && result->segments.empty() && result->vias.empty() )
    {
        m_proposalBoardTimestamp = -1;
        reportFailure( *result );
        return 0;
    }

    m_result = *result;
    showProposal( *m_result );
    return 0;
}


void AUTOROUTER_TOOL::showProposal( const ROUTING_RESULT& aResult )
{
    clearVisualProposal();
    m_previewBoardItems.clear();
    m_hiddenItems.clear();

    KICAD_BOARD_ADAPTER adapter( board() );
    m_previewBoardItems = adapter.CreatePreviewItems( aResult );

    m_previewGroup = std::make_unique<KIGFX::VIEW_GROUP>( view() );
    m_previewGroup->SetLayer( LAYER_SELECT_OVERLAY );
    view()->Add( m_previewGroup.get() );

    for( const ROUTING_SEGMENT& segment : aResult.segments )
    {
        auto item = std::make_unique<AUTOROUTER_PREVIEW_ITEM>( segment );
        m_previewGroup->Add( item.get() );
        m_previewItems.push_back( std::move( item ) );
    }

    for( const ROUTING_VIA& via : aResult.vias )
    {
        auto item = std::make_unique<AUTOROUTER_PREVIEW_ITEM>( via );
        m_previewGroup->Add( item.get() );
        m_previewItems.push_back( std::move( item ) );
    }

    for( const std::string& uuid : aResult.removedBoardItemIds )
    {
        if( BOARD_ITEM* item = adapter.FindBoardItem( uuid ) )
        {
            const bool wasVisible = view()->IsVisible( item );
            view()->Hide( item, true );
            m_hiddenItems.push_back( { item, wasVisible } );
        }
    }

    view()->Update( m_previewGroup.get() );
    m_proposalActive = true;

    m_reviewDialog = std::make_unique<DIALOG_AUTOROUTER_REVIEW>(
            frame(), aResult,
            [this]( bool aAccepted )
            {
                if( aAccepted )
                    acceptProposal();
                else
                    rejectProposal();
            } );
    // Keep the editor model stable while the proposal is being inspected.
    // A modeless review would allow edits against the snapshot and make the
    // exact Reject guarantee impossible to uphold.
    m_reviewDialog->ShowModal();
    m_reviewDialog.reset();
}


void AUTOROUTER_TOOL::clearVisualProposal()
{
    if( m_previewGroup )
    {
        m_previewGroup->Clear();

        if( view() )
            view()->Remove( m_previewGroup.get() );

        m_previewGroup.reset();
    }

    m_previewItems.clear();
}


void AUTOROUTER_TOOL::restoreHiddenItems()
{
    for( const HIDDEN_ITEM& hidden : m_hiddenItems )
    {
        if( hidden.item && view() )
            view()->Hide( hidden.item, !hidden.wasVisible );
    }

    m_hiddenItems.clear();
}


void AUTOROUTER_TOOL::acceptProposal()
{
    if( !m_proposalActive || !m_result )
        return;

    // Enforce the same gate as the button even if acceptance was dispatched
    // programmatically, or the dialog changes in a future refactor.
    if( !m_result->CanAcceptProposal() )
        return;

    if( !board() || board()->GetTimeStamp() != m_proposalBoardTimestamp )
    {
        rejectProposal();
        wxMessageBox( _( "The board changed while the autoroute proposal was open."
                         " The proposal was discarded; run Autoroute Board again." ),
                      _( "Autoroute Board" ), wxOK | wxICON_WARNING, frame() );
        return;
    }

    BOARD_COMMIT commit( this );
    KICAD_BOARD_ADAPTER adapter( board() );

    for( const std::string& uuid : m_result->removedBoardItemIds )
    {
        if( BOARD_ITEM* item = adapter.FindBoardItem( uuid ) )
            commit.Remove( item );
    }

    for( std::unique_ptr<BOARD_ITEM>& item : m_previewBoardItems )
    {
        if( item )
        {
            item->ClearFlags( ROUTER_TRANSIENT );
            commit.Add( item.get() );
        }
    }

    commit.Push( _( "Autoroute Board" ) );

    for( std::unique_ptr<BOARD_ITEM>& item : m_previewBoardItems )
        item.release();

    m_previewBoardItems.clear();
    m_hiddenItems.clear();
    clearVisualProposal();
    m_proposalActive = false;
    m_proposalBoardTimestamp = -1;

    if( m_reviewDialog )
        m_reviewDialog->Hide();
}


void AUTOROUTER_TOOL::rejectProposal()
{
    if( !m_proposalActive )
        return;

    clearVisualProposal();
    m_previewBoardItems.clear();
    restoreHiddenItems();
    m_proposalActive = false;
    m_proposalBoardTimestamp = -1;

    if( m_reviewDialog )
        m_reviewDialog->Hide();
}


void AUTOROUTER_TOOL::reportFailure( const ROUTING_RESULT& aResult ) const
{
    const wxString message = aResult.message.empty()
                                     ? _( "The autorouter could not produce a route." )
                                     : wxString::FromUTF8( aResult.message.c_str() );
    wxMessageBox( message, _( "Autoroute Board" ), wxOK | wxICON_WARNING, frame() );
}


void AUTOROUTER_TOOL::setTransitions()
{
    Go( &AUTOROUTER_TOOL::AutorouteBoard, PCB_ACTIONS::autorouteBoard.MakeEvent() );
}

} // namespace KICAD_AUTOROUTER
