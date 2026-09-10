/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Direct translation of Freerouting ShapeSearchTree.completeShape,
 * restrainShape, and divideLargeRoom at a11c0a42 (GPL-3.0).
 */
#include "ShapeSearchTree.h"

#include "../../geometry/planar/LineSegment.h"

#include <algorithm>
#include <cmath>

namespace KICAD_AUTOROUTER
{
using PLANAR::LINE;
using PLANAR::LINE_SEGMENT;
using PLANAR::SIMPLEX;


MIN_AREA_TREE::HANDLE SHAPE_SEARCH_TREE::Insert( SHAPE_TREE_ENTRY aEntry )
{
    SIMPLEX shape = aEntry.BoundingSimplex();
    if( shape.Dimension() < 0 )
        return MIN_AREA_TREE::NONE;

    const auto bounds = shape.BoundingBox();
    if( !bounds )
        return MIN_AREA_TREE::NONE;

    aEntry.shape = *bounds;
    aEntry.simplex = std::move( shape );
    return m_tree.Insert( aEntry );
}


std::vector<SHAPE_TREE_ENTRY> SHAPE_SEARCH_TREE::Overlaps(
        const SIMPLEX& aShape ) const
{
    const auto bounds = aShape.BoundingBox();
    if( !bounds )
        return {};

    auto result = m_tree.Overlaps( *bounds );
    std::erase_if( result, [&]( const SHAPE_TREE_ENTRY& aEntry )
    {
        return aEntry.BoundingSimplex().Intersection( aShape ).Dimension() < 0;
    } );
    return result;
}


std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> SHAPE_SEARCH_TREE::CompleteShape(
        const INCOMPLETE_GENERAL_EXPANSION_ROOM& aRoom, int aNet,
        std::optional<int> aIgnoreObject, std::optional<SIMPLEX> aIgnoreShape,
        const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> result;
    if( m_tree.Size() == 0 || aRoom.containedShape.IsEmpty() )
        return result;

    SIMPLEX startShape = SIMPLEX::Box( m_bounds ).Intersection( aRoom.shape );
    if( startShape.Dimension() != 2 )
        return result;

    result.push_back( { std::move( startShape ), aRoom.layer,
                        aRoom.containedShape } );

    // The pinned source first gathers every broad-phase leaf and then sorts by
    // the leaf's natural object-id/shape-index order.  It does not prune that
    // list after room restraint shrinks the aggregate bounds.
    auto leaves = m_tree.Overlaps( result.front().shape.BoundingBox().value() );
    for( const SHAPE_TREE_ENTRY& entry : leaves )
    {
        if( aCancel && aCancel() )
            return {};
        if( !entry.IsTraceObstacle( aNet ) || entry.layer != aRoom.layer
            || ( aIgnoreObject && entry.objectId == *aIgnoreObject ) )
        {
            continue;
        }

        const SIMPLEX obstacle = entry.BoundingSimplex();
        std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> next;
        for( const auto& room : result )
        {
            const SIMPLEX intersection = room.shape.Intersection( obstacle );
            if( intersection.Dimension() == 2 )
            {
                const bool ignoreExpansionRoom = entry.isRoom && aIgnoreShape
                                                 && aIgnoreShape->Contains( intersection );
                if( !ignoreExpansionRoom )
                {
                    auto restrained = RestrainShape( room, obstacle );
                    next.insert( next.end(), restrained.begin(), restrained.end() );
                    continue;
                }
            }
            next.push_back( room );
        }
        result = std::move( next );
    }

    return divideLargeRoom( std::move( result ) );
}


std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> SHAPE_SEARCH_TREE::RestrainShape(
        const INCOMPLETE_GENERAL_EXPANSION_ROOM& aRoom,
        const SIMPLEX& aObstacle )
{
    std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> result;
    if( aRoom.containedShape.IsEmpty() )
        return result;

    std::optional<LINE> cutLine;
    double cutLineDistance = -1;
    for( std::size_t i = 0; i < aObstacle.Borders().size(); ++i )
    {
        const auto segment = LINE_SEGMENT::FromShape( aObstacle, i );
        if( !segment
            || !aRoom.shape.IsIntersectedInteriorBy(
                    segment->StartPoint(), segment->EndPoint(), segment->GetLine() ) )
        {
            continue;
        }

        const LINE& currentLine = aObstacle.Borders()[i];
        const double currentDistance = aRoom.containedShape.DistanceToTheLeft(
                currentLine );
        if( currentDistance > cutLineDistance )
        {
            cutLineDistance = currentDistance;
            cutLine = currentLine.Opposite();
        }
    }

    if( cutLine )
    {
        const SIMPLEX resultPiece = aRoom.shape.Intersection(
                SIMPLEX::GetInstance( { *cutLine } ) );
        if( resultPiece.Dimension() >= 2 )
        {
            result.push_back( { resultPiece, aRoom.layer,
                                aRoom.containedShape } );
        }
        return result;
    }

    if( aRoom.containedShape.Dimension() < 1 )
        return result;

    for( std::size_t i = 0; i < aObstacle.Borders().size(); ++i )
    {
        const auto segment = LINE_SEGMENT::FromShape( aObstacle, i );
        if( !segment
            || !aRoom.shape.IsIntersectedInteriorBy(
                    segment->StartPoint(), segment->EndPoint(), segment->GetLine() ) )
        {
            continue;
        }

        const LINE& currentLine = aObstacle.Borders()[i];
        if( aRoom.containedShape.SideOf( currentLine ) == 0 )
        {
            cutLine = currentLine.Opposite();
            break;
        }
    }

    if( !cutLine )
        return result;

    const SIMPLEX cutHalfPlane = SIMPLEX::GetInstance( { *cutLine } );
    const SIMPLEX newContained = aRoom.containedShape.Intersection( cutHalfPlane );
    const SIMPLEX resultPiece = aRoom.shape.Intersection( cutHalfPlane );
    if( resultPiece.Dimension() >= 2 )
        result.push_back( { resultPiece, aRoom.layer, newContained } );

    const SIMPLEX oppositeHalfPlane = SIMPLEX::GetInstance(
            { cutLine->Opposite() } );
    const SIMPLEX restPiece = aRoom.shape.Intersection( oppositeHalfPlane );
    if( restPiece.Dimension() >= 2 )
    {
        const SIMPLEX restContained = aRoom.containedShape.Intersection(
                oppositeHalfPlane );
        auto rest = RestrainShape(
                { restPiece, aRoom.layer, restContained }, aObstacle );
        result.insert( result.end(), rest.begin(), rest.end() );
    }
    return result;
}


std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> SHAPE_SEARCH_TREE::divideLargeRoom(
        std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> aRooms ) const
{
    if( aRooms.size() != 1 )
        return aRooms;

    const auto roomBox = aRooms.front().shape.BoundingBox();
    if( !roomBox )
        return aRooms;

    const double boardWidth = static_cast<double>( m_bounds.maxX - m_bounds.minX );
    const double boardHeight = static_cast<double>( m_bounds.maxY - m_bounds.minY );
    const double roomWidth = static_cast<double>( roomBox->maxX - roomBox->minX );
    const double roomHeight = static_cast<double>( roomBox->maxY - roomBox->minY );
    if( 2 * roomHeight <= boardHeight || 2 * roomWidth <= boardWidth )
        return aRooms;

    const double maximumSectionWidth = 0.5 * std::max( boardHeight, boardWidth );
    std::vector<INCOMPLETE_GENERAL_EXPANSION_ROOM> result;
    for( const SIMPLEX& section :
         aRooms.front().shape.DivideIntoSections( maximumSectionWidth ) )
    {
        result.push_back( { section, aRooms.front().layer,
                            section.Intersection( aRooms.front().containedShape ) } );
    }
    return result;
}

} // namespace KICAD_AUTOROUTER
