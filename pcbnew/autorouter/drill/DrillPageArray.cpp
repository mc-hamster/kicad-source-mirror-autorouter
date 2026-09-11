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
#include <stdexcept>

namespace KICAD_AUTOROUTER
{

namespace
{

bool overlaps( const ROUTER_BOX& aLeft, const ROUTER_BOX& aRight )
{
    // Drill pages require AREA overlap. Border/point contact is not a page
    // expansion or invalidation, unlike the shape-tree query predicate.
    return INT_BOX::Dimension( INT_BOX::Intersection( aLeft, aRight ) ) == 2;
}

} // namespace


DRILL_PAGE_ARRAY::DRILL_PAGE_ARRAY( const ROUTER_BOX& aBounds, std::int64_t aMaxPageWidth ) :
        m_bounds( aBounds )
{
    if( INT_BOX::Dimension( aBounds ) != 2 || aMaxPageWidth <= 0 )
        throw std::invalid_argument( "Drill pages require positive bounds and page width" );
    if( ( aBounds.minX < 0 && aBounds.maxX > std::numeric_limits<std::int64_t>::max() + aBounds.minX )
        || ( aBounds.minY < 0 && aBounds.maxY > std::numeric_limits<std::int64_t>::max() + aBounds.minY ) )
        throw std::length_error( "Drill page bounds exceed coordinate arithmetic" );
    const std::int64_t width = std::max<std::int64_t>( 1, aBounds.maxX - aBounds.minX );
    const std::int64_t height = std::max<std::int64_t>( 1, aBounds.maxY - aBounds.minY );
    const std::int64_t maxPage = std::max<std::int64_t>( 1, aMaxPageWidth );

    const double columns = std::ceil( width / static_cast<double>( maxPage ) );
    const double rows = std::ceil( height / static_cast<double>( maxPage ) );
    if( columns > std::numeric_limits<int>::max() || rows > std::numeric_limits<int>::max()
        || columns * rows > m_pages.max_size() )
        throw std::length_error( "Drill page dimensions exceed addressable storage" );
    m_columns = static_cast<int>( columns );
    m_rows = static_cast<int>( rows );
    m_pageWidth = width / m_columns + ( width % m_columns != 0 );
    m_pageHeight = height / m_rows + ( height % m_rows != 0 );

    m_pages.reserve( static_cast<std::size_t>( m_columns ) * m_rows );
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
    const auto clipped = INT_BOX::Intersection( m_bounds, aShape );
    if( INT_BOX::Dimension( clipped ) != 2 )
        return result;
    // Reference row-major bounding-page range, followed by the exact area
    // predicate. Do not scan the entire board for every small room/door.
    const auto firstColumn = ( clipped.minX - m_bounds.minX ) / m_pageWidth;
    const auto lastColumn = ( clipped.maxX - m_bounds.minX - 1 ) / m_pageWidth;
    const auto firstRow = ( clipped.minY - m_bounds.minY ) / m_pageHeight;
    const auto lastRow = ( clipped.maxY - m_bounds.minY - 1 ) / m_pageHeight;
    for( auto row = firstRow; row <= lastRow; ++row )
        for( auto column = firstColumn; column <= lastColumn; ++column )
        {
            auto& page = m_pages[static_cast<std::size_t>( row ) * m_columns + column];
            if( overlaps( page.Shape(), aShape ) )
                result.push_back( &page );
        }
    return result;
}


void DRILL_PAGE_ARRAY::AddFanoutCandidates( ROUTER_POINT aCenter,
                                             std::int64_t aMinimumDistance,
                                             std::int64_t aMaximumDistance )
{
    const std::int64_t minimum = std::max<std::int64_t>( 0, aMinimumDistance );
    const std::int64_t maximum = std::max( minimum, aMaximumDistance );
    if( maximum <= 0 )
        return;

    // A Freerouting fanout runs against a live board and its drill-page
    // centroids. The native worker instead uses an immutable control pad; on
    // a page wider than the requested escape annulus, every centroid can lie
    // outside that annulus and the room frontier has no drill state to test.
    // Seed exact annulus points into the same page/free-shape pipeline rather
    // than falling back to the retired grid search. The final canDrill and
    // ordered ViaRule checks remain authoritative.
    std::vector<std::int64_t> distances{ maximum };
    if( minimum > 0 && minimum != maximum )
        distances.push_back( minimum );

    // Cardinal and 45-degree exits are the source router's fixed-direction
    // fanout basis. General-convex routing may subsequently bend between
    // room doors; these are drill seeds, not a path discretization.
    constexpr long double DIAGONAL = 0.7071067811865475244L;
    constexpr std::pair<long double, long double> directions[] = {
        { 1, 0 }, { DIAGONAL, DIAGONAL }, { 0, 1 }, { -DIAGONAL, DIAGONAL },
        { -1, 0 }, { -DIAGONAL, -DIAGONAL }, { 0, -1 }, { DIAGONAL, -DIAGONAL }
    };

    for( std::int64_t distance : distances )
    {
        for( const auto& [dx, dy] : directions )
        {
            const ROUTER_POINT candidate{
                aCenter.x + static_cast<std::int64_t>( std::llround( distance * dx ) ),
                aCenter.y + static_cast<std::int64_t>( std::llround( distance * dy ) ) };
            if( !m_bounds.Contains( candidate ) )
                continue;

            const std::int64_t column = std::min<std::int64_t>(
                    m_columns - 1, ( candidate.x - m_bounds.minX ) / m_pageWidth );
            const std::int64_t row = std::min<std::int64_t>(
                    m_rows - 1, ( candidate.y - m_bounds.minY ) / m_pageHeight );
            m_pages[static_cast<std::size_t>( row ) * m_columns + column]
                    .AddCandidate( candidate );
        }
    }
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
    for( auto* page : OverlappingPages( aShape ) )
        page->Invalidate();
}


void DRILL_PAGE_ARRAY::Reset()
{
    for( DRILL_PAGE& page : m_pages )
        page.Reset();
}

} // namespace KICAD_AUTOROUTER
