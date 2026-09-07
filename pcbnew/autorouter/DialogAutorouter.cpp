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

#include "DialogAutorouter.h"

#include <algorithm>

#include <base_units.h>
#include <board.h>
#include <layer_ids.h>

#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/tokenzr.h>


namespace KICAD_AUTOROUTER
{

namespace
{

wxSpinCtrl* addIntegerControl( wxWindow* aParent, wxFlexGridSizer* aSizer, const wxString& aLabel,
                               int aValue, int aMin, int aMax )
{
    aSizer->Add( new wxStaticText( aParent, wxID_ANY, aLabel ), 0, wxALIGN_CENTER_VERTICAL );
    auto* control = new wxSpinCtrl( aParent, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                    wxDefaultSize, wxSP_ARROW_KEYS, aMin, aMax, aValue );
    aSizer->Add( control, 0, wxEXPAND );
    return control;
}


void setTextValue( wxTextCtrl* aControl, const std::vector<std::string>& aValues )
{
    wxString result;

    for( const std::string& value : aValues )
    {
        if( !result.empty() )
            result += ", ";

        result += wxString::FromUTF8( value.c_str() );
    }

    aControl->SetValue( result );
}

} // namespace


DIALOG_AUTOROUTER_SETTINGS::DIALOG_AUTOROUTER_SETTINGS(
        wxWindow* aParent, const BOARD* aBoard, const AUTOROUTER_SETTINGS& aSettings ) :
        DIALOG_SHIM( aParent, wxID_ANY, _( "Autoroute Board" ), wxDefaultPosition, wxDefaultSize,
                     wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER ),
        m_settings( aSettings ),
        m_board( aBoard )
{
    auto* outer = new wxBoxSizer( wxVERTICAL );

    auto* layerBox = new wxStaticBoxSizer( wxVERTICAL, this, _( "Routing layers" ) );
    auto* layerScroll = new wxScrolledWindow( this, wxID_ANY, wxDefaultPosition,
                                              FromDIP( wxSize( 560, 190 ) ), wxVSCROLL );
    layerScroll->SetScrollRate( 0, FromDIP( 10 ) );

    auto* layerGrid = new wxFlexGridSizer( 0, 4, 5, 8 );
    layerGrid->AddGrowableCol( 0, 1 );
    layerGrid->Add( new wxStaticText( layerScroll, wxID_ANY, _( "Layer" ) ), 0,
                    wxALIGN_CENTER_VERTICAL );
    layerGrid->Add( new wxStaticText( layerScroll, wxID_ANY, _( "Enabled" ) ), 0,
                    wxALIGN_CENTER_VERTICAL );
    layerGrid->Add( new wxStaticText( layerScroll, wxID_ANY, _( "Preferred direction" ) ), 0,
                    wxALIGN_CENTER_VERTICAL );
    layerGrid->Add( new wxStaticText( layerScroll, wxID_ANY, _( "Direction cost" ) ), 0,
                    wxALIGN_CENTER_VERTICAL );

    for( const ROUTER_LAYER_SETTINGS& layer : m_settings.layers )
    {
        const PCB_LAYER_ID layerId = static_cast<PCB_LAYER_ID>( layer.layerId );
        const wxString name = m_board ? m_board->GetLayerName( layerId )
                                      : wxString::Format( _( "Layer %d" ), layer.layerId );
        auto* layerName = new wxStaticText( layerScroll, wxID_ANY, name );
        auto* enabled = new wxCheckBox( layerScroll, wxID_ANY, wxEmptyString );
        enabled->SetValue( layer.enabled );

        auto* direction = new wxChoice( layerScroll, wxID_ANY );
        direction->Append( _( "Any" ) );
        direction->Append( _( "Horizontal" ) );
        direction->Append( _( "Vertical" ) );
        direction->SetSelection( std::clamp( layer.preferredDirection, 0, 2 ) );

        auto* cost = new wxSpinCtrl( layerScroll, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                     wxDefaultSize, wxSP_ARROW_KEYS, 0, 100000, layer.directionCost );

        layerGrid->Add( layerName, 0, wxALIGN_CENTER_VERTICAL );
        layerGrid->Add( enabled, 0, wxALIGN_CENTER_VERTICAL );
        layerGrid->Add( direction, 0, wxEXPAND );
        layerGrid->Add( cost, 0, wxEXPAND );

        m_layerControls.push_back( { layer.layerId, enabled, direction, cost } );
    }

    layerScroll->SetSizer( layerGrid );
    layerScroll->FitInside();
    layerBox->Add( layerScroll, 1, wxEXPAND | wxALL, FromDIP( 6 ) );
    outer->Add( layerBox, 1, wxEXPAND | wxALL, FromDIP( 8 ) );

    auto* costBox = new wxStaticBoxSizer( wxVERTICAL, this, _( "Routing costs and limits" ) );
    auto* costGrid = new wxFlexGridSizer( 0, 2, 6, 6 );
    costGrid->AddGrowableCol( 1, 1 );

    costGrid->Add( new wxStaticText( this, wxID_ANY, _( "Search grid (mm)" ) ), 0,
                   wxALIGN_CENTER_VERTICAL );
    m_gridStep = new wxSpinCtrlDouble( this, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                       wxDefaultSize, wxSP_ARROW_KEYS, 0.001, 10.0,
                                       pcbIUScale.IUTomm( m_settings.gridStepIU ), 0.001 );
    m_gridStep->SetDigits( 3 );
    costGrid->Add( m_gridStep, 0, wxEXPAND );

    m_viaCost = addIntegerControl( this, costGrid, _( "Via cost" ), m_settings.viaCost, 0, 1000000 );
    m_planeViaCost = addIntegerControl( this, costGrid, _( "Plane via cost" ),
                                        m_settings.planeViaCost, 0, 1000000 );
    m_traceLengthCost = addIntegerControl( this, costGrid, _( "Trace length cost" ),
                                           m_settings.traceLengthCost, 0, 1000000 );
    m_congestionCost = addIntegerControl( this, costGrid, _( "Congestion cost" ),
                                          m_settings.congestionCost, 0, 1000000 );
    m_bendCost = addIntegerControl( this, costGrid, _( "Bend cost" ), m_settings.bendCost, 0, 1000000 );
    m_startRipupCost = addIntegerControl( this, costGrid, _( "Start rip-up cost" ),
                                          m_settings.startRipupCost, 0, 1000000 );
    m_maxIterations = addIntegerControl( this, costGrid, _( "Maximum iterations" ),
                                         m_settings.maxIterations, 1, 1000000 );
    m_maxPasses = addIntegerControl( this, costGrid, _( "Routing passes" ), m_settings.maxPasses, 1,
                                     1000000 );
    m_optimizationPasses = addIntegerControl( this, costGrid, _( "Optimization passes" ),
                                              m_settings.optimizationPasses, 0, 1000000 );
    m_maxOptimizationItems = addIntegerControl( this, costGrid, _( "Maximum optimization items" ),
                                                m_settings.maxOptimizationItems, 0, 100000000 );
    m_maxRipups = addIntegerControl( this, costGrid, _( "Maximum rip-ups" ), m_settings.maxRipups, 0,
                                     1000000 );
    m_maxExpandedNodes = addIntegerControl( this, costGrid, _( "Maximum expanded nodes" ),
                                            m_settings.maxExpandedNodes, 1, 100000000 );
    m_maxFanoutPasses = addIntegerControl( this, costGrid, _( "Maximum fanout passes" ),
                                           m_settings.maxFanoutPasses, 0, 1000000 );
    m_routingPriority = addIntegerControl( this, costGrid, _( "Routing priority" ),
                                            m_settings.routingPriority, -1000000, 1000000 );

    costBox->Add( costGrid, 0, wxEXPAND | wxALL, FromDIP( 8 ) );
    outer->Add( costBox, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP( 8 ) );

    auto* behaviorBox = new wxStaticBoxSizer( wxVERTICAL, this, _( "Routing behavior" ) );
    m_allowRipupExisting = new wxCheckBox( this, wxID_ANY, _( "Allow rip-up of existing tracks and vias" ) );
    m_allowViaInSmdPad = new wxCheckBox( this, wxID_ANY,
                                        _( "Allow vias directly in SMD pads" ) );
    m_allowRipupRouted = new wxCheckBox( this, wxID_ANY, _( "Allow rip-up of autorouted connections" ) );
    m_allowVias = new wxCheckBox( this, wxID_ANY, _( "Allow via insertion" ) );
    m_enableFanout = new wxCheckBox( this, wxID_ANY, _( "Run SMD fanout pre-pass" ) );
    m_stopAfterFirstComplete = new wxCheckBox( this, wxID_ANY,
                                               _( "Stop after the first complete route" ) );
    m_optimizeAfterComplete = new wxCheckBox( this, wxID_ANY, _( "Optimize after routing" ) );
    m_routeOnlyUnconnected = new wxCheckBox( this, wxID_ANY,
                                             _( "Route only currently unconnected nets" ) );

    m_allowVias->SetValue( m_settings.allowVias );
    m_allowViaInSmdPad->SetValue( m_settings.allowViaInSmdPad );
    m_allowRipupExisting->SetValue( m_settings.allowRipupExisting );
    m_allowRipupRouted->SetValue( m_settings.allowRipupRouted );
    m_enableFanout->SetValue( m_settings.enableFanout );
    m_stopAfterFirstComplete->SetValue( m_settings.stopAfterFirstComplete );
    m_optimizeAfterComplete->SetValue( m_settings.optimizeAfterComplete );
    m_routeOnlyUnconnected->SetValue( m_settings.routeOnlyUnconnected );

    behaviorBox->Add( m_allowVias, 0, wxBOTTOM, FromDIP( 3 ) );
    behaviorBox->Add( m_allowViaInSmdPad, 0, wxBOTTOM, FromDIP( 3 ) );
    behaviorBox->Add( m_allowRipupExisting, 0, wxBOTTOM, FromDIP( 3 ) );
    behaviorBox->Add( m_allowRipupRouted, 0, wxBOTTOM, FromDIP( 3 ) );
    behaviorBox->Add( m_enableFanout, 0, wxBOTTOM, FromDIP( 3 ) );
    behaviorBox->Add( m_stopAfterFirstComplete, 0, wxBOTTOM, FromDIP( 3 ) );
    behaviorBox->Add( m_optimizeAfterComplete, 0, wxBOTTOM, FromDIP( 3 ) );
    behaviorBox->Add( m_routeOnlyUnconnected, 0 );
    outer->Add( behaviorBox, 0, wxEXPAND | wxALL, FromDIP( 8 ) );

    auto* netBox = new wxStaticBoxSizer( wxVERTICAL, this, _( "Net and netclass filters" ) );
    auto* netGrid = new wxFlexGridSizer( 0, 2, 5, 5 );
    netGrid->AddGrowableCol( 1, 1 );
    netGrid->Add( new wxStaticText( this, wxID_ANY, _( "Include netclasses (comma separated)" ) ), 0,
                  wxALIGN_CENTER_VERTICAL );
    m_includeNetClasses = new wxTextCtrl( this, wxID_ANY );
    setTextValue( m_includeNetClasses, m_settings.includeNetClasses );
    netGrid->Add( m_includeNetClasses, 1, wxEXPAND );
    netGrid->Add( new wxStaticText( this, wxID_ANY, _( "Exclude netclasses (comma separated)" ) ), 0,
                  wxALIGN_CENTER_VERTICAL );
    m_excludeNetClasses = new wxTextCtrl( this, wxID_ANY );
    setTextValue( m_excludeNetClasses, m_settings.excludeNetClasses );
    netGrid->Add( m_excludeNetClasses, 1, wxEXPAND );
    netGrid->Add( new wxStaticText( this, wxID_ANY, _( "Include nets (comma separated)" ) ), 0,
                  wxALIGN_CENTER_VERTICAL );
    m_includeNets = new wxTextCtrl( this, wxID_ANY );
    setTextValue( m_includeNets, m_settings.includeNets );
    netGrid->Add( m_includeNets, 1, wxEXPAND );
    netGrid->Add( new wxStaticText( this, wxID_ANY, _( "Exclude nets (comma separated)" ) ), 0,
                  wxALIGN_CENTER_VERTICAL );
    m_excludeNets = new wxTextCtrl( this, wxID_ANY );
    setTextValue( m_excludeNets, m_settings.excludeNets );
    netGrid->Add( m_excludeNets, 1, wxEXPAND );
    netBox->Add( netGrid, 0, wxEXPAND | wxALL, FromDIP( 8 ) );
    outer->Add( netBox, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP( 8 ) );

    outer->Add( CreateStdDialogButtonSizer( wxOK | wxCANCEL ), 0,
                wxEXPAND | wxALL, FromDIP( 8 ) );
    SetSizerAndFit( outer );
    SetMinSize( FromDIP( wxSize( 600, 600 ) ) );
    SetupStandardButtons();
    finishDialogSettings();
}


std::vector<std::string> DIALOG_AUTOROUTER_SETTINGS::parseNetClassList( const wxString& aValue )
{
    std::vector<std::string> result;
    wxStringTokenizer tokenizer( aValue, ",; \t\r\n" );

    while( tokenizer.HasMoreTokens() )
    {
        const wxString token = tokenizer.GetNextToken().Trim( true ).Trim( false );

        if( !token.empty() )
            result.emplace_back( token.ToUTF8() );
    }

    return result;
}


AUTOROUTER_SETTINGS DIALOG_AUTOROUTER_SETTINGS::GetSettings() const
{
    AUTOROUTER_SETTINGS result = m_settings;
    result.layers.clear();

    for( const LAYER_CONTROLS& controls : m_layerControls )
    {
        ROUTER_LAYER_SETTINGS layer;
        layer.layerId = controls.layerId;
        layer.enabled = controls.enabled->GetValue();
        layer.preferredDirection = controls.direction->GetSelection();
        layer.directionCost = controls.directionCost->GetValue();

        if( const auto it = std::find_if(
                   m_settings.layers.begin(), m_settings.layers.end(),
                   [&]( const ROUTER_LAYER_SETTINGS& aSetting )
                   {
                       return aSetting.layerId == controls.layerId;
                   } );
            it != m_settings.layers.end() )
        {
            layer.layerOrdinal = it->layerOrdinal;
        }

        result.layers.push_back( layer );
    }

    result.gridStepIU = std::max( 1, pcbIUScale.mmToIU( m_gridStep->GetValue() ) );
    result.viaCost = m_viaCost->GetValue();
    result.planeViaCost = m_planeViaCost->GetValue();
    result.traceLengthCost = m_traceLengthCost->GetValue();
    result.congestionCost = m_congestionCost->GetValue();
    result.bendCost = m_bendCost->GetValue();
    result.startRipupCost = m_startRipupCost->GetValue();
    result.maxIterations = m_maxIterations->GetValue();
    result.maxPasses = m_maxPasses->GetValue();
    result.optimizationPasses = m_optimizationPasses->GetValue();
    result.maxOptimizationItems = m_maxOptimizationItems->GetValue();
    result.maxRipups = m_maxRipups->GetValue();
    result.maxExpandedNodes = m_maxExpandedNodes->GetValue();
    result.maxFanoutPasses = m_maxFanoutPasses->GetValue();
    result.routingPriority = m_routingPriority->GetValue();
    result.allowVias = m_allowVias->GetValue();
    result.allowViaInSmdPad = m_allowViaInSmdPad->GetValue();
    result.allowRipupExisting = m_allowRipupExisting->GetValue();
    result.allowRipupRouted = m_allowRipupRouted->GetValue();
    result.enableFanout = m_enableFanout->GetValue();
    result.stopAfterFirstComplete = m_stopAfterFirstComplete->GetValue();
    result.optimizeAfterComplete = m_optimizeAfterComplete->GetValue();
    result.routeOnlyUnconnected = m_routeOnlyUnconnected->GetValue();
    result.includeNetClasses = parseNetClassList( m_includeNetClasses->GetValue() );
    result.excludeNetClasses = parseNetClassList( m_excludeNetClasses->GetValue() );
    result.includeNets = parseNetClassList( m_includeNets->GetValue() );
    result.excludeNets = parseNetClassList( m_excludeNets->GetValue() );
    return result;
}

} // namespace KICAD_AUTOROUTER
