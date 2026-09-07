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

#include "DrillPageArray.h"

#include <algorithm>
#include <cmath>

namespace KICAD_AUTOROUTER
{

namespace
{

bool overlaps( const ROUTER_BOX& aLeft, const ROUTER_BOX& aRight )
{
    return aLeft.minX <= aRight.maxX && aRight.minX <= aLeft.maxX
           && aLeft.minY <= aRight.maxY && aRight.minY <= aLeft.maxY;
}

} // namespace


DRILL_PAGE_ARRAY::DRILL_PAGE_ARRAY( const ROUTER_BOX& aBounds, std::int64_t aMaxPageWidth ) :
        m_bounds( aBounds )
{
    const std::int64_t width = std::max<std::int64_t>( 1, aBounds.maxX - aBounds.minX );
    const std::int64_t height = std::max<std::int64_t>( 1, aBounds.maxY - aBounds.minY );
    const std::int64_t maxPage = std::max<std::int64_t>( 1, aMaxPageWidth );

    m_columns = std::max( 1, static_cast<int>( std::ceil( width / static_cast<double>( maxPage ) ) ) );
    m_rows = std::max( 1, static_cast<int>( std::ceil( height / static_cast<double>( maxPage ) ) ) );
    m_pageWidth = std::max<std::int64_t>( 1, ( width + m_columns - 1 ) / m_columns );
    m_pageHeight = std::max<std::int64_t>( 1, ( height + m_rows - 1 ) / m_rows );

    m_pages.reserve( static_cast<std::size_t>( m_columns * m_rows ) );
    for( int row = 0; row < m_rows; ++row )
    {
        for( int column = 0; column < m_columns; ++column )
        {
            const std::int64_t minX = aBounds.minX + column * m_pageWidth;
            const std::int64_t minY = aBounds.minY + row * m_pageHeight;
            const std::int64_t maxX = column + 1 == m_columns
                                              ? aBounds.maxX
                                              : std::min( aBounds.maxX, minX + m_pageWidth );
            const std::int64_t maxY = row + 1 == m_rows
                                              ? aBounds.maxY
                                              : std::min( aBounds.maxY, minY + m_pageHeight );
            m_pages.emplace_back( ROUTER_BOX{ minX, minY, maxX, maxY } );
        }
    }
}


std::vector<DRILL_PAGE*> DRILL_PAGE_ARRAY::OverlappingPages( const ROUTER_BOX& aShape )
{
    std::vector<DRILL_PAGE*> result;
    for( DRILL_PAGE& page : m_pages )
    {
        if( overlaps( page.Shape(), aShape ) )
            result.push_back( &page );
    }
    return result;
}


std::vector<ROUTER_POINT> DRILL_PAGE_ARRAY::LandmarkCenters() const
{
    std::vector<ROUTER_POINT> result;
    result.reserve( m_pages.size() );
    for( const DRILL_PAGE& page : m_pages )
        result.push_back( page.Center() );
    return result;
}


void DRILL_PAGE_ARRAY::Invalidate( const ROUTER_BOX& aShape )
{
    for( DRILL_PAGE& page : m_pages )
    {
        if( overlaps( page.Shape(), aShape ) )
            page.Invalidate();
    }
}


void DRILL_PAGE_ARRAY::Reset()
{
    for( DRILL_PAGE& page : m_pages )
        page.Reset();
}

} // namespace KICAD_AUTOROUTER
