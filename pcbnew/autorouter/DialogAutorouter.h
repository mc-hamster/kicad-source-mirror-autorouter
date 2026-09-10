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
 * Native KiCad settings dialog for the full-board autorouter.
 */

#pragma once

#include <vector>

#include <dialog_shim.h>

#include "AutorouterTypes.h"

class BOARD;
class wxCheckBox;
class wxChoice;
class wxSpinCtrl;
class wxSpinCtrlDouble;
class wxTextCtrl;


namespace KICAD_AUTOROUTER
{

class DIALOG_AUTOROUTER_SETTINGS : public DIALOG_SHIM
{
public:
    DIALOG_AUTOROUTER_SETTINGS( wxWindow* aParent, const BOARD* aBoard,
                                const AUTOROUTER_SETTINGS& aSettings );

    AUTOROUTER_SETTINGS GetSettings() const;

private:
    struct LAYER_CONTROLS
    {
        int              layerId = -1;
        wxCheckBox*      enabled = nullptr;
        wxChoice*        direction = nullptr;
        wxSpinCtrl*      directionCost = nullptr;
    };

    static std::vector<std::string> parseNetClassList( const wxString& aValue );

private:
    AUTOROUTER_SETTINGS m_settings;
    const BOARD*        m_board;
    std::vector<LAYER_CONTROLS> m_layerControls;

    wxSpinCtrlDouble* m_gridStep = nullptr;
    wxSpinCtrlDouble* m_neckWidth = nullptr;
    wxSpinCtrl*       m_viaCost = nullptr;
    wxSpinCtrl*       m_planeViaCost = nullptr;
    wxSpinCtrl*       m_traceLengthCost = nullptr;
    wxSpinCtrl*       m_congestionCost = nullptr;
    wxSpinCtrl*       m_bendCost = nullptr;
    wxSpinCtrl*       m_startRipupCost = nullptr;
    wxSpinCtrl*       m_maxIterations = nullptr;
    wxSpinCtrl*       m_maxPasses = nullptr;
    wxSpinCtrl*       m_optimizationPasses = nullptr;
    wxSpinCtrl*       m_maxOptimizationItems = nullptr;
    wxSpinCtrlDouble* m_optimizationImprovementThreshold = nullptr;
    wxSpinCtrl*       m_maxRipups = nullptr;
    wxSpinCtrl*       m_maxExpandedNodes = nullptr;
    wxSpinCtrl*       m_maxFanoutPasses = nullptr;
    wxSpinCtrl*       m_routingPriority = nullptr;
    wxCheckBox*       m_allowVias = nullptr;
    wxCheckBox*       m_allowViaInSmdPad = nullptr;
    wxCheckBox*       m_allowRipupExisting = nullptr;
    wxCheckBox*       m_allowRipupRouted = nullptr;
    wxCheckBox*       m_enableFanout = nullptr;
    wxCheckBox*       m_stopAfterFirstComplete = nullptr;
    wxCheckBox*       m_optimizeAfterComplete = nullptr;
    wxCheckBox*       m_routeOnlyUnconnected = nullptr;
    wxTextCtrl*       m_includeNetClasses = nullptr;
    wxTextCtrl*       m_excludeNetClasses = nullptr;
    wxTextCtrl*       m_includeNets = nullptr;
    wxTextCtrl*       m_excludeNets = nullptr;
};

} // namespace KICAD_AUTOROUTER
