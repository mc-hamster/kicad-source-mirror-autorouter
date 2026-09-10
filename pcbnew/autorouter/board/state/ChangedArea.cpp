/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct C++ translation of Freerouting board/state/ChangedArea.java at
 * a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#include "ChangedArea.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "../../geometry/planar/IntBox.h"


namespace KICAD_AUTOROUTER
{
namespace
{

std::int64_t outwardFloor( double aValue )
{
    if( !std::isfinite( aValue )
        || aValue < static_cast<double>( std::numeric_limits<std::int64_t>::min() )
        || aValue > static_cast<double>( std::numeric_limits<std::int64_t>::max() ) )
    {
        throw std::overflow_error( "ChangedArea coordinate overflow" );
    }

    return static_cast<std::int64_t>( std::floor( aValue ) );
}


std::int64_t outwardCeil( double aValue )
{
    if( !std::isfinite( aValue )
        || aValue < static_cast<double>( std::numeric_limits<std::int64_t>::min() )
        || aValue > static_cast<double>( std::numeric_limits<std::int64_t>::max() ) )
    {
        throw std::overflow_error( "ChangedArea coordinate overflow" );
    }

    return static_cast<std::int64_t>( std::ceil( aValue ) );
}

} // namespace


void CHANGED_AREA::MUTABLE_OCTAGON::SetEmpty()
{
    leftX = std::numeric_limits<double>::infinity();
    bottomY = std::numeric_limits<double>::infinity();
    rightX = -std::numeric_limits<double>::infinity();
    topY = -std::numeric_limits<double>::infinity();
    upperLeftDiagonalX = std::numeric_limits<double>::infinity();
    lowerRightDiagonalX = -std::numeric_limits<double>::infinity();
    lowerLeftDiagonalX = std::numeric_limits<double>::infinity();
    upperRightDiagonalX = -std::numeric_limits<double>::infinity();
}


bool CHANGED_AREA::MUTABLE_OCTAGON::IsEmpty() const
{
    return rightX < leftX || topY < bottomY
           || lowerRightDiagonalX < upperLeftDiagonalX
           || upperRightDiagonalX < lowerLeftDiagonalX;
}


PLANAR::INT_OCTAGON CHANGED_AREA::MUTABLE_OCTAGON::ToInt() const
{
    if( IsEmpty() )
        return PLANAR::INT_OCTAGON::Empty();

    return { outwardFloor( leftX ), outwardFloor( bottomY ),
             outwardCeil( rightX ), outwardCeil( topY ),
             outwardFloor( upperLeftDiagonalX ),
             outwardCeil( lowerRightDiagonalX ),
             outwardFloor( lowerLeftDiagonalX ),
             outwardCeil( upperRightDiagonalX ) };
}


CHANGED_AREA::CHANGED_AREA( int aLayerCount ) :
        m_areas( static_cast<std::size_t>( std::max( 0, aLayerCount ) ) )
{
    for( MUTABLE_OCTAGON& area : m_areas )
        area.SetEmpty();
}


void CHANGED_AREA::Join( double aX, double aY, int aLayer )
{
    if( aLayer < 0 || aLayer >= LayerCount() || !std::isfinite( aX )
        || !std::isfinite( aY ) )
    {
        return;
    }

    MUTABLE_OCTAGON& current = m_areas[static_cast<std::size_t>( aLayer )];
    current.leftX = std::min( current.leftX, aX );
    current.bottomY = std::min( current.bottomY, aY );
    current.rightX = std::max( current.rightX, aX );
    current.topY = std::max( current.topY, aY );

    const double difference = aX - aY;
    current.upperLeftDiagonalX = std::min( current.upperLeftDiagonalX, difference );
    current.lowerRightDiagonalX = std::max( current.lowerRightDiagonalX, difference );

    const double sum = aX + aY;
    current.lowerLeftDiagonalX = std::min( current.lowerLeftDiagonalX, sum );
    current.upperRightDiagonalX = std::max( current.upperRightDiagonalX, sum );
}


void CHANGED_AREA::Join( ROUTER_POINT aPoint, int aLayer )
{
    Join( static_cast<double>( aPoint.x ), static_cast<double>( aPoint.y ), aLayer );
}


void CHANGED_AREA::Join( const PLANAR::INT_OCTAGON& aShape, int aLayer )
{
    if( aShape.IsEmpty() )
        return;

    for( int corner = 0; corner < 8; ++corner )
        Join( aShape.Corner( corner ), aLayer );
}


PLANAR::INT_OCTAGON CHANGED_AREA::GetArea( int aLayer ) const
{
    if( aLayer < 0 || aLayer >= LayerCount() )
        return PLANAR::INT_OCTAGON::Empty();

    return m_areas[static_cast<std::size_t>( aLayer )].ToInt();
}


ROUTER_BOX CHANGED_AREA::SurroundingBox() const
{
    ROUTER_BOX result = INT_BOX::Empty();
    for( const MUTABLE_OCTAGON& area : m_areas )
    {
        if( !area.IsEmpty() )
            result = INT_BOX::Union( result, area.ToInt().BoundingBox() );
    }
    return result;
}


void CHANGED_AREA::SetEmpty( int aLayer )
{
    if( aLayer >= 0 && aLayer < LayerCount() )
        m_areas[static_cast<std::size_t>( aLayer )].SetEmpty();
}

} // namespace KICAD_AUTOROUTER
