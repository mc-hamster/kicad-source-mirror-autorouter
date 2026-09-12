/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Port of Freerouting datastructures/MinAreaTree.java at a11c0a42 (GPL-3.0).
 */
#include "MinAreaTree.h"
#include "../AutorouterDebug.h"
#include <tuple>

namespace KICAD_AUTOROUTER
{
namespace
{
double areaIncrease( const std::optional<PLANAR::INT_OCTAGON>& aBounds,
                     const ROUTER_BOX& aBox,
                     const SHAPE_TREE_ENTRY& aEntry )
{
    if( aBounds && aEntry.octagon )
        return aBounds->Union( *aEntry.octagon ).Area() - aBounds->Area();

    return INT_BOX::Area( INT_BOX::Union( aBox, aEntry.shape ) )
           - INT_BOX::Area( aBox );
}


void traceMutation( const char* aAction, std::size_t aLeafCount,
                    const SHAPE_TREE_ENTRY& aEntry )
{
    if( !aEntry.octagon )
        return;

    const PLANAR::INT_OCTAGON& shape = *aEntry.octagon;
    autorouterDecisionLog(
            std::string( "MIN_AREA_TREE_45_" ) + aAction,
            { { "leaf_count", std::to_string( aLeafCount ) },
              { "object_id", std::to_string( aEntry.objectId ) },
              { "shape_index", std::to_string( aEntry.shapeIndex ) },
              { "layer", std::to_string( aEntry.layer ) },
              { "supports",
                std::to_string( shape.leftX ) + ','
                        + std::to_string( shape.bottomY ) + ','
                        + std::to_string( shape.rightX ) + ','
                        + std::to_string( shape.topY ) + ','
                        + std::to_string( shape.upperLeftDiagonalX ) + ','
                        + std::to_string( shape.lowerRightDiagonalX ) + ','
                        + std::to_string( shape.lowerLeftDiagonalX ) + ','
                        + std::to_string( shape.upperRightDiagonalX ) } } );
}
} // namespace

MIN_AREA_TREE::HANDLE MIN_AREA_TREE::Insert( const SHAPE_TREE_ENTRY& aEntry )
{
    if( INT_BOX::Dimension( aEntry.shape ) < 0 )
        return NONE;
    traceMutation( "INSERT", m_leafCount, aEntry );
    const HANDLE leaf = m_nodes.size();
    m_nodes.push_back( { aEntry.shape, NONE, NONE, NONE, aEntry,
                         aEntry.octagon } );
    ++m_leafCount;
    if( m_root == NONE )
    {
        m_root = leaf;
        return leaf;
    }
    HANDLE current = m_root;
    while( !m_nodes[current].entry )
    {
        NODE& node = m_nodes[current];
        if( node.octagonBounds && aEntry.octagon )
        {
            node.octagonBounds = node.octagonBounds->Union( *aEntry.octagon );
            node.bounds = node.octagonBounds->BoundingBox();
        }
        else
        {
            node.bounds = INT_BOX::Union( node.bounds, aEntry.shape );
        }

        const NODE& first = m_nodes[node.first];
        const NODE& second = m_nodes[node.second];
        const double firstIncrease = areaIncrease( first.octagonBounds,
                                                   first.bounds, aEntry );
        const double secondIncrease = areaIncrease( second.octagonBounds,
                                                    second.bounds, aEntry );
        current = firstIncrease <= secondIncrease ? node.first : node.second;
    }
    const HANDLE parent = m_nodes[current].parent;
    const HANDLE inner = m_nodes.size();
    std::optional<PLANAR::INT_OCTAGON> innerOctagon;
    ROUTER_BOX innerBounds = INT_BOX::Union( m_nodes[current].bounds, aEntry.shape );
    if( m_nodes[current].octagonBounds && aEntry.octagon )
    {
        innerOctagon = m_nodes[current].octagonBounds->Union( *aEntry.octagon );
        innerBounds = innerOctagon->BoundingBox();
    }
    m_nodes.push_back( { innerBounds, parent, current, leaf, std::nullopt,
                         innerOctagon } );
    m_nodes[current].parent = inner;
    m_nodes[leaf].parent = inner;
    if( parent == NONE )
        m_root = inner;
    else if( m_nodes[parent].first == current )
        m_nodes[parent].first = inner;
    else
        m_nodes[parent].second = inner;
    return leaf;
}

bool MIN_AREA_TREE::Remove( HANDLE aHandle )
{
    if( aHandle >= m_nodes.size() || !m_nodes[aHandle].entry )
        return false;
    NODE& leaf = m_nodes[aHandle];
    traceMutation( "REMOVE", m_leafCount, *leaf.entry );
    leaf.entry.reset();
    --m_leafCount;
    const HANDLE parent = leaf.parent;
    if( parent == NONE )
    {
        m_root = NONE;
        return true;
    }
    const HANDLE sibling = m_nodes[parent].first == aHandle
                                   ? m_nodes[parent].second : m_nodes[parent].first;
    HANDLE ancestor = m_nodes[parent].parent;
    m_nodes[sibling].parent = ancestor;
    if( ancestor == NONE )
        m_root = sibling;
    else if( m_nodes[ancestor].first == parent )
        m_nodes[ancestor].first = sibling;
    else
        m_nodes[ancestor].second = sibling;
    while( ancestor != NONE )
    {
        NODE& node = m_nodes[ancestor];
        const auto& firstOctagon = m_nodes[node.first].octagonBounds;
        const auto& secondOctagon = m_nodes[node.second].octagonBounds;
        if( firstOctagon && secondOctagon )
        {
            const auto bounds = secondOctagon->Union( *firstOctagon );
            if( node.octagonBounds && node.octagonBounds->IsContainedIn( bounds ) )
                break;
            node.octagonBounds = bounds;
            node.bounds = bounds.BoundingBox();
            ancestor = node.parent;
            continue;
        }

        const auto bounds = INT_BOX::Union( m_nodes[node.second].bounds,
                                            m_nodes[node.first].bounds );
        if( INT_BOX::Contains( bounds, node.bounds ) )
            break;
        node.bounds = bounds;
        ancestor = node.parent;
    }
    return true;
}

bool MIN_AREA_TREE::UpdateEntry( HANDLE aHandle,
                                 const SHAPE_TREE_ENTRY& aEntry )
{
    if( aHandle >= m_nodes.size() || !m_nodes[aHandle].entry )
        return false;

    const ROUTER_BOX& bounds = m_nodes[aHandle].bounds;
    if( bounds.minX != aEntry.shape.minX || bounds.minY != aEntry.shape.minY
        || bounds.maxX != aEntry.shape.maxX || bounds.maxY != aEntry.shape.maxY )
    {
        return false;
    }

    const auto& oldOctagon = m_nodes[aHandle].octagonBounds;
    if( oldOctagon.has_value() != aEntry.octagon.has_value()
        || ( oldOctagon && *oldOctagon != *aEntry.octagon ) )
    {
        return false;
    }

    m_nodes[aHandle].entry = aEntry;
    return true;
}

bool MIN_AREA_TREE::Visit(
        PLANAR::INT_OCTAGON& aQuery,
        const std::function<bool( const SHAPE_TREE_ENTRY& )>& aVisit ) const
{
    std::vector<HANDLE> stack;
    if( m_root != NONE )
        stack.push_back( m_root );
    while( !stack.empty() )
    {
        const NODE& node = m_nodes[stack.back()];
        stack.pop_back();
        const bool intersects = node.octagonBounds
                                ? node.octagonBounds->Intersects( aQuery )
                                : INT_BOX::Intersects( node.bounds,
                                                      aQuery.BoundingBox() );
        if( !intersects )
            continue;
        if( node.entry )
        {
            if( !aVisit( *node.entry ) )
                return false;
        }
        else
        {
            // Match ShapeTree's LIFO traversal: secondChild is visited first.
            stack.push_back( node.first );
            stack.push_back( node.second );
        }
    }
    return true;
}

bool MIN_AREA_TREE::Visit( ROUTER_BOX& aQuery,
                          const std::function<bool( const SHAPE_TREE_ENTRY& )>& aVisit ) const
{
    std::vector<HANDLE> stack;
    if( m_root != NONE )
        stack.push_back( m_root );
    while( !stack.empty() )
    {
        const NODE& node = m_nodes[stack.back()];
        stack.pop_back();
        if( !INT_BOX::Intersects( node.bounds, aQuery ) )
            continue;
        if( node.entry )
        {
            if( !aVisit( *node.entry ) )
                return false;
        }
        else
        {
            stack.push_back( node.first );
            stack.push_back( node.second );
        }
    }
    return true;
}

std::vector<SHAPE_TREE_ENTRY> MIN_AREA_TREE::Overlaps( ROUTER_BOX aQuery ) const
{
    std::vector<SHAPE_TREE_ENTRY> result;
    Visit( aQuery, [&]( const SHAPE_TREE_ENTRY& entry )
    {
        result.push_back( entry );
        return true;
    } );
    std::sort( result.begin(), result.end(), []( const auto& left, const auto& right )
    {
        return std::tie( left.objectId, left.shapeIndex )
               < std::tie( right.objectId, right.shapeIndex );
    } );
    return result;
}

std::vector<SHAPE_TREE_ENTRY> MIN_AREA_TREE::Overlaps(
        PLANAR::INT_OCTAGON aQuery ) const
{
    std::vector<SHAPE_TREE_ENTRY> result;
    Visit( aQuery, [&]( const SHAPE_TREE_ENTRY& entry )
    {
        result.push_back( entry );
        return true;
    } );
    std::sort( result.begin(), result.end(), []( const auto& left, const auto& right )
    {
        return std::tie( left.objectId, left.shapeIndex )
               < std::tie( right.objectId, right.shapeIndex );
    } );
    return result;
}
} // namespace KICAD_AUTOROUTER
