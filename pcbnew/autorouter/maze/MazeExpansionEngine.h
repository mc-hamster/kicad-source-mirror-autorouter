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


/* KiCad, GPL-3.0-or-later. Freerouting a11c0a42 MazeExpansionEngine.
 * Pure frontier cost operations; board/forced-insertion policy stays separate.
 */
#pragma once
#include "../geometry/planar/FloatLine.h"
#include "../geometry/planar/IntOctagon.h"

namespace KICAD_AUTOROUTER
{
class MAZE_EXPANSION_ENGINE
{
public:
    // Legacy fallback's coordinate neighbours. The room/drill frontier below
    // does not use these cells as its routing state.
    static std::vector<ROUTER_NODE> Neighbours(
            const ROUTER_NODE& aNode, std::int64_t aGridStep,
            const std::vector<ROUTER_LAYER_SETTINGS>& aLayers, bool aAllowVias );

    struct COST { double expansion; double sorting; FLOAT_POINT entry; };

    static FLOAT_POINT Nearest( ROUTER_BOX aShape, FLOAT_POINT aFrom )
    {
        return { std::clamp( aFrom.x, static_cast<double>( aShape.minX ), static_cast<double>( aShape.maxX ) ),
                 std::clamp( aFrom.y, static_cast<double>( aShape.minY ), static_cast<double>( aShape.maxY ) ) };
    }

    static FLOAT_POINT Nearest( const PLANAR::INT_OCTAGON& aShape,
                                FLOAT_POINT aFrom );

    static COST ToPage( ROUTER_BOX aPage, FLOAT_POINT aFrom, double aExpansion,
                        double aViaCost, double aHorizontal, double aVertical, double aRemaining )
    {
        const auto point = Nearest( aPage, aFrom );
        // Charge the via once, but defer actual approach distance until drill
        // expansion. Including it in g here would charge that distance twice.
        const double expansion = aExpansion + aViaCost;
        return { expansion, expansion + point.WeightedDistance( aFrom, aHorizontal, aVertical )
                                      + aRemaining, aFrom };
    }

    static COST ToDrill( ROUTER_BOX aCentreShape, FLOAT_POINT aFrom, double aExpansion,
                         double aViaCost, bool aFromPage, double aHorizontal,
                         double aVertical, double aRemaining )
    {
        const auto point = Nearest( aCentreShape, aFrom );
        const double expansion = aExpansion + point.WeightedDistance( aFrom, aHorizontal, aVertical )
                                 + ( aFromPage ? 0 : aViaCost );
        return { expansion, expansion + aRemaining, point };
    }

    static COST ToDrill( const PLANAR::INT_OCTAGON& aCentreShape,
                         FLOAT_POINT aFrom, double aExpansion,
                         double aViaCost, bool aFromPage, double aHorizontal,
                         double aVertical, double aRemaining )
    {
        const auto point = Nearest( aCentreShape, aFrom );
        const double expansion = aExpansion
                                 + point.WeightedDistance( aFrom, aHorizontal, aVertical )
                                 + ( aFromPage ? 0 : aViaCost );
        return { expansion, expansion + aRemaining, point };
    }

    static double NormalViaCost( double aMaxRadius, double aViaCost, bool aPureSmd )
    {
        return aViaCost * std::max( aMaxRadius, 1.0 ) * ( aPureSmd ? 0.1 : 1.0 );
    }
};
}
