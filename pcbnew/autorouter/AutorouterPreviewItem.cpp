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

#include "AutorouterPreviewItem.h"

#include <algorithm>

#include <gal/color4d.h>
#include <gal/graphics_abstraction_layer.h>
#include <layer_ids.h>
#include <math/vector2d.h>
#include <view/view.h>


namespace KICAD_AUTOROUTER
{

AUTOROUTER_PREVIEW_ITEM::AUTOROUTER_PREVIEW_ITEM( const ROUTING_SEGMENT& aSegment ) :
        EDA_ITEM( NOT_USED ),
        m_segment( aSegment )
{
}


AUTOROUTER_PREVIEW_ITEM::AUTOROUTER_PREVIEW_ITEM( const ROUTING_VIA& aVia ) :
        EDA_ITEM( NOT_USED ),
        m_isVia( true ),
        m_via( aVia )
{
}


const BOX2I AUTOROUTER_PREVIEW_ITEM::ViewBBox() const
{
    if( m_isVia )
    {
        const VECTOR2I center( static_cast<int>( m_via.position.x ),
                               static_cast<int>( m_via.position.y ) );
        BOX2I result( center, VECTOR2I( 0, 0 ) );
        result.Inflate( std::max( 1, static_cast<int>( m_via.diameter / 2 ) ) );
        return result;
    }

    BOX2I result = BOX2I::ByCorners(
            VECTOR2I( static_cast<int>( m_segment.start.x ), static_cast<int>( m_segment.start.y ) ),
            VECTOR2I( static_cast<int>( m_segment.end.x ), static_cast<int>( m_segment.end.y ) ) );
    result.Inflate( std::max( 1, static_cast<int>( m_segment.width / 2 ) ) );
    return result;
}


void AUTOROUTER_PREVIEW_ITEM::ViewDraw( int, KIGFX::VIEW* aView ) const
{
    if( !aView || !aView->GetGAL() )
        return;

    KIGFX::GAL* gal = aView->GetGAL();
    const KIGFX::COLOR4D routeColor( 0.15, 0.85, 0.35, 0.82 );
    const KIGFX::COLOR4D viaColor( 0.95, 0.70, 0.15, 0.88 );

    gal->SetIsStroke( true );
    gal->SetIsFill( false );

    if( m_isVia )
    {
        const VECTOR2D center( m_via.position.x, m_via.position.y );
        gal->SetStrokeColor( viaColor );
        gal->SetFillColor( viaColor.WithAlpha( 0.55 ) );
        gal->SetIsFill( true );
        gal->SetLineWidth( 1.0f );
        gal->DrawCircle( center, std::max( 1.0, m_via.diameter / 2.0 ) );

        gal->SetIsFill( false );
        gal->SetStrokeColor( KIGFX::COLOR4D( 0.15, 0.15, 0.15, 0.9 ) );
        gal->DrawCircle( center, std::max( 1.0, m_via.drill / 2.0 ) );
        return;
    }

    gal->SetStrokeColor( routeColor );
    gal->SetLineWidth( std::max( 1.0, static_cast<double>( m_segment.width ) )
                       / std::max( 0.0001, gal->GetWorldScale() ) );
    gal->DrawLine( VECTOR2D( m_segment.start.x, m_segment.start.y ),
                   VECTOR2D( m_segment.end.x, m_segment.end.y ) );
}


std::vector<int> AUTOROUTER_PREVIEW_ITEM::ViewGetLayers() const
{
    return { LAYER_SELECT_OVERLAY };
}

} // namespace KICAD_AUTOROUTER
