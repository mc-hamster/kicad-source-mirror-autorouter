/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct translation of Freerouting ShapeSearchTree45Degree room completion
 * and restraint at a11c0a42d1b3827e5126429c5c9820c4ab5bec7c (GPL-3.0).
 */
#include "ShapeSearchTree45Degree.h"

#include <algorithm>
#include <cmath>

namespace KICAD_AUTOROUTER
{
using PLANAR::INT_OCTAGON;

MIN_AREA_TREE::HANDLE SHAPE_SEARCH_TREE_45_DEGREE::Insert( SHAPE_TREE_ENTRY aEntry )
{
    INT_OCTAGON shape = aEntry.BoundingOctagon().Normalize();
    if( shape.Dimension() < 0 )
        return MIN_AREA_TREE::NONE;
    aEntry.shape = shape.BoundingBox();
    aEntry.octagon = shape;
    return m_tree.Insert( aEntry );
}


std::vector<SHAPE_TREE_ENTRY> SHAPE_SEARCH_TREE_45_DEGREE::Overlaps(
        const INT_OCTAGON& aShape ) const
{
    auto result = m_tree.Overlaps( aShape.BoundingBox() );
    std::erase_if( result, [&]( const SHAPE_TREE_ENTRY& aEntry )
    {
        return !aEntry.BoundingOctagon().Intersects( aShape );
    } );
    return result;
}


std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>
SHAPE_SEARCH_TREE_45_DEGREE::CompleteShape(
        const INCOMPLETE_45_DEGREE_EXPANSION_ROOM& aRoom, int aNet,
        std::optional<int> aIgnoreObject, std::optional<INT_OCTAGON> aIgnoreShape,
        const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> result;
    if( m_tree.Size() == 0 || aRoom.containedShape.Dimension() < 0 )
        return result;

    INT_OCTAGON startShape = INT_OCTAGON::FromBox( m_bounds );
    if( aRoom.shape.Dimension() >= 0 )
        startShape = aRoom.shape.Intersection( startShape );
    if( startShape.Dimension() < 0 )
        return result;

    result.push_back( { startShape, aRoom.layer, aRoom.containedShape } );
    ROUTER_BOX bounding = startShape.BoundingBox();
    const bool finished = m_tree.Visit( bounding, [&]( const SHAPE_TREE_ENTRY& aEntry )
    {
        if( aCancel && aCancel() )
            return false;
        if( !aEntry.IsTraceObstacle( aNet ) || aEntry.layer != aRoom.layer
            || ( aIgnoreObject && aEntry.objectId == *aIgnoreObject ) )
        {
            return true;
        }

        const INT_OCTAGON obstacle = aEntry.BoundingOctagon();
        std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> next;
        ROUTER_BOX nextBounding = INT_BOX::Empty();
        for( const auto& room : result )
        {
            if( room.shape.Overlaps( obstacle ) )
            {
                if( aEntry.isRoom && aIgnoreShape )
                {
                    const INT_OCTAGON intersection = room.shape.Intersection( obstacle );
                    if( intersection.IsContainedIn( *aIgnoreShape ) )
                    {
                        if( !room.shape.IsContainedIn( *aIgnoreShape ) )
                        {
                            next.push_back( room );
                            nextBounding = INT_BOX::Union( nextBounding,
                                                          room.shape.BoundingBox() );
                        }
                        continue;
                    }
                }

                auto restrained = RestrainShape( room, obstacle );
                for( const auto& candidate : restrained )
                {
                    nextBounding = INT_BOX::Union( nextBounding,
                                                  candidate.shape.BoundingBox() );
                    next.push_back( candidate );
                }
            }
            else
            {
                nextBounding = INT_BOX::Union( nextBounding, room.shape.BoundingBox() );
                next.push_back( room );
            }
        }
        result = std::move( next );
        bounding = nextBounding;
        return true;
    } );

    if( !finished )
        return {};

    result = divideLargeRoom( std::move( result ) );
    std::erase_if( result, []( const auto& aCandidate )
    {
        return aCandidate.shape.IsContainedIn( aCandidate.containedShape );
    } );
    return result;
}


bool SHAPE_SEARCH_TREE_45_DEGREE::obstacleSegmentTouchesInside(
        const INT_OCTAGON& aObstacle, int aObstacleLine, const INT_OCTAGON& aRoom )
{
    int currentLine = aObstacleLine;
    const std::int64_t currentX = aObstacle.CornerX( aObstacleLine );
    const std::int64_t currentY = aObstacle.CornerY( aObstacleLine );
    for( int i = 0; i < 5; ++i )
    {
        if( aRoom.SideOfBorderLine( currentX, currentY, currentLine ) != -1 )
            return false;
        currentLine = ( currentLine + 1 ) % 8;
    }

    const int nextObstacleLine = ( aObstacleLine + 1 ) % 8;
    const std::int64_t nextX = aObstacle.CornerX( nextObstacleLine );
    const std::int64_t nextY = aObstacle.CornerY( nextObstacleLine );
    currentLine = ( aObstacleLine + 5 ) % 8;
    for( int i = 0; i < 3; ++i )
    {
        if( aRoom.SideOfBorderLine( nextX, nextY, currentLine ) != -1 )
            return false;
        currentLine = ( currentLine + 1 ) % 8;
    }
    return true;
}


double SHAPE_SEARCH_TREE_45_DEGREE::signedLineDistance(
        const INT_OCTAGON& aObstacle, int aObstacleLine, const INT_OCTAGON& aContained )
{
    switch( aObstacleLine )
    {
    case 0: return static_cast<double>( aObstacle.bottomY - aContained.topY );
    case 2: return static_cast<double>( aContained.leftX - aObstacle.rightX );
    case 4: return static_cast<double>( aContained.bottomY - aObstacle.topY );
    case 6: return static_cast<double>( aObstacle.leftX - aContained.rightX );
    case 1: return 0.5 * static_cast<double>( aContained.upperLeftDiagonalX
                                             - aObstacle.lowerRightDiagonalX );
    case 3: return 0.5 * static_cast<double>( aContained.lowerLeftDiagonalX
                                             - aObstacle.upperRightDiagonalX );
    case 5: return 0.5 * static_cast<double>( aObstacle.upperLeftDiagonalX
                                             - aContained.lowerRightDiagonalX );
    case 7: return 0.5 * static_cast<double>( aObstacle.lowerLeftDiagonalX
                                             - aContained.upperRightDiagonalX );
    default: throw std::out_of_range( "45-degree obstacle line index" );
    }
}


std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>
SHAPE_SEARCH_TREE_45_DEGREE::RestrainShape(
        const INCOMPLETE_45_DEGREE_EXPANSION_ROOM& aRoom, const INT_OCTAGON& aObstacle )
{
    std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> result;
    const INT_OCTAGON contained = aRoom.containedShape;
    if( contained.IsEmpty() )
        return result;
    const INT_OCTAGON room = aRoom.shape;

    double cutLineDistance = -1;
    int restrainingLine = -1;
    for( int obstacleLine = 0; obstacleLine < 8; ++obstacleLine )
    {
        const double distance = signedLineDistance( aObstacle, obstacleLine, contained );
        if( distance > cutLineDistance
            && obstacleSegmentTouchesInside( aObstacle, obstacleLine, room ) )
        {
            cutLineDistance = distance;
            restrainingLine = obstacleLine;
        }
    }

    if( cutLineDistance >= 0 )
    {
        result.push_back( { CalcOutsideRestrainedShape( aObstacle, restrainingLine, room ),
                            aRoom.layer, contained } );
        return result;
    }

    if( contained.Dimension() < 1 )
        return result;

    restrainingLine = -1;
    for( int obstacleLine = 0; obstacleLine < 8; ++obstacleLine )
    {
        if( !obstacleSegmentTouchesInside( aObstacle, obstacleLine, room ) )
            continue;

        bool left = false;
        bool right = false;
        const PLANAR::LINE line = aObstacle.BorderLine( obstacleLine );
        for( int corner = 0; corner < 8; ++corner )
        {
            const int side = line.SideOf( PLANAR::POINT( contained.Corner( corner ) ) );
            if( side > 0 ) right = true;
            else if( side < 0 ) left = true;
        }
        if( left && right )
        {
            restrainingLine = obstacleLine;
            break;
        }
    }
    if( restrainingLine < 0 )
        return result;

    const INT_OCTAGON restrained = CalcOutsideRestrainedShape(
            aObstacle, restrainingLine, room );
    if( restrained.Dimension() == 2 )
    {
        const INT_OCTAGON newContained = contained.Intersection( restrained );
        if( newContained.Dimension() > 0 )
            result.push_back( { restrained, aRoom.layer, newContained } );
    }

    const INT_OCTAGON restPiece = CalcInsideRestrainedShape(
            aObstacle, restrainingLine, room );
    if( restPiece.Dimension() >= 2 )
    {
        const INT_OCTAGON restContained = contained.Intersection( restPiece );
        if( restContained.Dimension() >= 0 )
        {
            auto rest = RestrainShape( { restPiece, aRoom.layer, restContained }, aObstacle );
            result.insert( result.end(), rest.begin(), rest.end() );
        }
    }
    return result;
}


INT_OCTAGON SHAPE_SEARCH_TREE_45_DEGREE::CalcOutsideRestrainedShape(
        const INT_OCTAGON& aObstacle, int aObstacleLine, const INT_OCTAGON& aRoom )
{
    INT_OCTAGON result = aRoom;
    switch( aObstacleLine )
    {
    case 0: result.topY = aObstacle.bottomY; break;
    case 2: result.leftX = aObstacle.rightX; break;
    case 4: result.bottomY = aObstacle.topY; break;
    case 6: result.rightX = aObstacle.leftX; break;
    case 1: result.upperLeftDiagonalX = aObstacle.lowerRightDiagonalX; break;
    case 3: result.lowerLeftDiagonalX = aObstacle.upperRightDiagonalX; break;
    case 5: result.lowerRightDiagonalX = aObstacle.upperLeftDiagonalX; break;
    case 7: result.upperRightDiagonalX = aObstacle.lowerLeftDiagonalX; break;
    default: throw std::out_of_range( "45-degree obstacle line index" );
    }
    return result.Normalize();
}


INT_OCTAGON SHAPE_SEARCH_TREE_45_DEGREE::CalcInsideRestrainedShape(
        const INT_OCTAGON& aObstacle, int aObstacleLine, const INT_OCTAGON& aRoom )
{
    INT_OCTAGON result = aRoom;
    switch( aObstacleLine )
    {
    case 0: result.bottomY = aObstacle.bottomY; break;
    case 2: result.rightX = aObstacle.rightX; break;
    case 4: result.topY = aObstacle.topY; break;
    case 6: result.leftX = aObstacle.leftX; break;
    case 1: result.lowerRightDiagonalX = aObstacle.lowerRightDiagonalX; break;
    case 3: result.upperRightDiagonalX = aObstacle.upperRightDiagonalX; break;
    case 5: result.upperLeftDiagonalX = aObstacle.upperLeftDiagonalX; break;
    case 7: result.lowerLeftDiagonalX = aObstacle.lowerLeftDiagonalX; break;
    default: throw std::out_of_range( "45-degree obstacle line index" );
    }
    return result.Normalize();
}


std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM>
SHAPE_SEARCH_TREE_45_DEGREE::divideLargeRoom(
        std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> aRooms ) const
{
    if( aRooms.size() != 1 )
        return aRooms;
    const ROUTER_BOX roomBox = aRooms.front().shape.BoundingBox();
    const double boardWidth = static_cast<double>( m_bounds.maxX - m_bounds.minX );
    const double boardHeight = static_cast<double>( m_bounds.maxY - m_bounds.minY );
    const double roomWidth = static_cast<double>( roomBox.maxX - roomBox.minX );
    const double roomHeight = static_cast<double>( roomBox.maxY - roomBox.minY );
    if( 2 * roomHeight <= boardHeight || 2 * roomWidth <= boardWidth )
        return aRooms;

    const double maxSectionWidth = 0.5 * std::max( boardHeight, boardWidth );
    if( maxSectionWidth <= 0 )
        return aRooms;
    const int xCount = static_cast<int>( std::ceil( roomWidth / maxSectionWidth ) );
    const int yCount = static_cast<int>( std::ceil( roomHeight / maxSectionWidth ) );
    if( xCount <= 0 || yCount <= 0 )
        return aRooms;
    const std::int64_t sectionWidth = static_cast<std::int64_t>(
            std::ceil( roomWidth / xCount ) );
    const std::int64_t sectionHeight = static_cast<std::int64_t>(
            std::ceil( roomHeight / yCount ) );

    std::vector<INCOMPLETE_45_DEGREE_EXPANSION_ROOM> result;
    for( int y = 0; y < yCount; ++y )
    {
        const std::int64_t lowerY = roomBox.minY + y * sectionHeight;
        const std::int64_t upperY = y == yCount - 1 ? roomBox.maxY
                                                     : lowerY + sectionHeight;
        for( int x = 0; x < xCount; ++x )
        {
            const std::int64_t leftX = roomBox.minX + x * sectionWidth;
            const std::int64_t rightX = x == xCount - 1 ? roomBox.maxX
                                                         : leftX + sectionWidth;
            const INT_OCTAGON section = aRooms.front().shape.Intersection(
                    INT_OCTAGON::FromBox( { leftX, lowerY, rightX, upperY } ) );
            if( section.Dimension() != 2 )
                continue;
            result.push_back( { section, aRooms.front().layer,
                                section.Intersection( aRooms.front().containedShape ) } );
        }
    }
    return result;
}

} // namespace KICAD_AUTOROUTER
