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

#include "AutorouterDialogs.h"

#include <algorithm>
#include <utility>

#include <base_units.h>

#include <wx/button.h>
#include <wx/gauge.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/timer.h>


namespace KICAD_AUTOROUTER
{

DIALOG_AUTOROUTER_PROGRESS::DIALOG_AUTOROUTER_PROGRESS( wxWindow* aParent,
                                                        AUTOROUTER_JOB& aJob ) :
        DIALOG_SHIM( aParent, wxID_ANY, _( "Autoroute Board" ), wxDefaultPosition, wxDefaultSize,
                     wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER ),
        m_job( aJob )
{
    auto* outer = new wxBoxSizer( wxVERTICAL );
    m_stage = new wxStaticText( this, wxID_ANY, _( "Starting autorouter..." ) );
    m_counts = new wxStaticText( this, wxID_ANY, wxEmptyString );
    m_gauge = new wxGauge( this, wxID_ANY, 1 );

    outer->Add( m_stage, 0, wxEXPAND | wxALL, FromDIP( 10 ) );
    outer->Add( m_counts, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP( 10 ) );
    outer->Add( m_gauge, 0, wxEXPAND | wxALL, FromDIP( 10 ) );

    auto* buttons = new wxStdDialogButtonSizer();
    m_cancelButton = new wxButton( this, wxID_CANCEL, _( "Cancel" ) );
    m_reviewButton = new wxButton( this, wxID_OK, _( "Review proposal" ) );
    m_reviewButton->Disable();
    buttons->AddButton( m_cancelButton );
    buttons->AddButton( m_reviewButton );
    buttons->Realize();
    outer->Add( buttons, 0, wxEXPAND | wxALL, FromDIP( 10 ) );

    SetSizerAndFit( outer );
    SetupStandardButtons();
    finishDialogSettings();

    Bind( wxEVT_BUTTON, &DIALOG_AUTOROUTER_PROGRESS::cancel, this, wxID_CANCEL );
    Bind( wxEVT_BUTTON, &DIALOG_AUTOROUTER_PROGRESS::continueToReview, this, wxID_OK );
    Bind( wxEVT_CLOSE_WINDOW, &DIALOG_AUTOROUTER_PROGRESS::closeWindow, this );
    m_timer = new wxTimer( this );
    Bind( wxEVT_TIMER, &DIALOG_AUTOROUTER_PROGRESS::updateProgress, this, m_timer->GetId() );
    m_timer->Start( 100 );
}


DIALOG_AUTOROUTER_PROGRESS::~DIALOG_AUTOROUTER_PROGRESS()
{
    if( m_timer )
        m_timer->Stop();
}


void DIALOG_AUTOROUTER_PROGRESS::updateProgress( wxTimerEvent& )
{
    const ROUTER_PROGRESS progress = m_job.GetProgress();
    const int total = std::max( 1, progress.totalConnections );
    m_gauge->SetRange( total );
    m_gauge->SetValue( std::clamp( progress.routedConnections, 0, total ) );
    m_stage->SetLabel( progress.stage.empty() ? _( "Routing..." )
                                               : wxString::FromUTF8( progress.stage.c_str() ) );
    m_counts->SetLabel( wxString::Format( _( "%d of %d connections, %d expanded nodes, %d rip-ups, %lld ms" ),
                                          progress.routedConnections, progress.totalConnections,
                                          progress.expandedNodes, progress.ripups,
                                          static_cast<long long>( progress.elapsedMilliseconds ) ) );

    if( m_job.IsFinished() )
    {
        m_timer->Stop();
        m_cancelButton->Disable();
        m_reviewButton->Enable();
        m_reviewButton->SetLabel( _( "Review proposal" ) );
        m_stage->SetLabel( _( "Routing finished" ) );
    }
}


void DIALOG_AUTOROUTER_PROGRESS::cancel( wxCommandEvent& )
{
    if( m_job.IsFinished() )
    {
        EndDialogShim( wxID_CANCEL );
        return;
    }

    if( !m_cancelling )
    {
        m_cancelling = true;
        m_job.Cancel();
        m_cancelButton->Disable();
        m_reviewButton->Disable();
        m_stage->SetLabel( _( "Cancelling autorouter..." ) );
    }
}


void DIALOG_AUTOROUTER_PROGRESS::continueToReview( wxCommandEvent& )
{
    if( m_job.IsFinished() )
        EndDialogShim( wxID_OK );
}


void DIALOG_AUTOROUTER_PROGRESS::closeWindow( wxCloseEvent& aEvent )
{
    if( m_job.IsFinished() )
    {
        EndDialogShim( wxID_CANCEL );
        return;
    }

    m_job.Cancel();
    m_cancelling = true;
    aEvent.Veto();
}


DIALOG_AUTOROUTER_REVIEW::DIALOG_AUTOROUTER_REVIEW( wxWindow* aParent,
                                                    const ROUTING_RESULT& aResult,
                                                    DECISION_CALLBACK aDecisionCallback ) :
        DIALOG_SHIM( aParent, wxID_ANY, _( "Autoroute Proposal" ), wxDefaultPosition,
                     wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER ),
        m_decisionCallback( std::move( aDecisionCallback ) )
{
    const ROUTER_METRICS& metrics = aResult.metrics;
    auto* outer = new wxBoxSizer( wxVERTICAL );
    auto* title = new wxStaticText( this, wxID_ANY,
                                    metrics.drcViolations > 0
                                            ? _( "The proposal contains design-rule violations." )
                                            : aResult.complete
                                                      ? _( "The board was routed completely." )
                                                      : _( "The proposal contains unrouted connections." ) );
    title->Wrap( FromDIP( 420 ) );
    outer->Add( title, 0, wxEXPAND | wxALL, FromDIP( 10 ) );

    const double lengthMm = pcbIUScale.IUTomm( static_cast<int>( metrics.routedLengthIU ) );
    const double airlineLengthMm =
            pcbIUScale.IUTomm( static_cast<int>( metrics.airlineLengthIU ) );
    wxString unroutedNets;
    for( int netCode : aResult.unroutedNetCodes )
    {
        if( !unroutedNets.empty() )
            unroutedNets += ", ";

        unroutedNets += wxString::Format( "%d", netCode );
    }

    if( unroutedNets.empty() )
        unroutedNets = _( "none" );

    auto* details = new wxStaticText(
            this, wxID_ANY,
            wxString::Format( _( "Routed connections: %d / %d\n"
                                 "Completion: %.1f%%\n"
                                 "Unrouted connections: %d\n"
                                 "Segments: %d\n"
                                 "Vias: %d\n"
                                 "SMD fanout connections: %d\n"
                                 "Worker DRC violations: %d\n"
                                 "Track length: %.3f mm\n"
                                 "Airline length: %.3f mm\n"
                                 "Passes: %d, rip-ups: %d, optimization passes: %d\n"
                                 "Unrouted net codes: %s\n"
                                 "Elapsed: %lld ms\n\n%s" ),
                             metrics.routedConnections, metrics.totalConnections,
                             metrics.completionPercent,
                             metrics.unroutedConnections, metrics.segmentCount, metrics.viaCount,
                             metrics.fanoutConnections, metrics.drcViolations, lengthMm,
                             airlineLengthMm, metrics.passes,
                             metrics.ripups,
                             metrics.optimizationPasses,
                             unroutedNets,
                             static_cast<long long>( metrics.elapsedMilliseconds ),
                             wxString::FromUTF8( aResult.message.c_str() ) ) );
    outer->Add( details, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP( 10 ) );

    auto* buttons = new wxStdDialogButtonSizer();
    auto* reject = new wxButton( this, wxID_CANCEL, _( "Reject" ) );
    m_acceptButton = new wxButton( this, wxID_OK, _( "Accept" ) );
    if( metrics.drcViolations > 0 )
        m_acceptButton->Disable();

    if( metrics.drcViolations > 0 )
    {
        auto* warning = new wxStaticText(
                this, wxID_ANY,
                _( "Accept is disabled because the worker found design-rule violations."
                   " Reject the proposal and adjust the board or routing settings." ) );
        warning->Wrap( FromDIP( 420 ) );
        outer->Add( warning, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP( 10 ) );
    }

    buttons->AddButton( reject );
    buttons->AddButton( m_acceptButton );
    buttons->Realize();
    outer->Add( buttons, 0, wxEXPAND | wxALL, FromDIP( 10 ) );

    SetSizerAndFit( outer );
    SetupStandardButtons();
    finishDialogSettings();

    Bind( wxEVT_BUTTON, &DIALOG_AUTOROUTER_REVIEW::accept, this, wxID_OK );
    Bind( wxEVT_BUTTON, &DIALOG_AUTOROUTER_REVIEW::reject, this, wxID_CANCEL );
    Bind( wxEVT_CLOSE_WINDOW, &DIALOG_AUTOROUTER_REVIEW::closeWindow, this );
}


void DIALOG_AUTOROUTER_REVIEW::accept( wxCommandEvent& )
{
    decide( true );
}


void DIALOG_AUTOROUTER_REVIEW::reject( wxCommandEvent& )
{
    decide( false );
}


void DIALOG_AUTOROUTER_REVIEW::closeWindow( wxCloseEvent& )
{
    decide( false );
}


void DIALOG_AUTOROUTER_REVIEW::decide( bool aAccepted )
{
    if( m_decided )
        return;

    m_decided = true;

    if( m_decisionCallback )
        m_decisionCallback( aAccepted );

    EndDialogShim( aAccepted ? wxID_OK : wxID_CANCEL );
}

} // namespace KICAD_AUTOROUTER
