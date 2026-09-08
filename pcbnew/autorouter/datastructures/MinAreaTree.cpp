/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Port of Freerouting datastructures/MinAreaTree.java at a11c0a42 (GPL-3.0).
 */
#include "MinAreaTree.h"
#include <tuple>

namespace KICAD_AUTOROUTER
{
MIN_AREA_TREE::HANDLE MIN_AREA_TREE::Insert( const SHAPE_TREE_ENTRY& aEntry )
{
    if( INT_BOX::Dimension( aEntry.shape ) < 0 )
        return NONE;
    const HANDLE leaf = m_nodes.size();
    m_nodes.push_back( { aEntry.shape, NONE, NONE, NONE, aEntry } );
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
        node.bounds = INT_BOX::Union( node.bounds, aEntry.shape );
        const auto& first = m_nodes[node.first].bounds;
        const auto& second = m_nodes[node.second].bounds;
        const double firstIncrease = INT_BOX::Area( INT_BOX::Union( first, aEntry.shape ) )
                                     - INT_BOX::Area( first );
        const double secondIncrease = INT_BOX::Area( INT_BOX::Union( second, aEntry.shape ) )
                                      - INT_BOX::Area( second );
        current = firstIncrease <= secondIncrease ? node.first : node.second;
    }
    const HANDLE parent = m_nodes[current].parent;
    const HANDLE inner = m_nodes.size();
    m_nodes.push_back( { INT_BOX::Union( m_nodes[current].bounds, aEntry.shape ),
                        parent, current, leaf, std::nullopt } );
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
        const auto bounds = INT_BOX::Union( m_nodes[node.second].bounds,
                                            m_nodes[node.first].bounds );
        if( INT_BOX::Contains( bounds, node.bounds ) )
            break;
        node.bounds = bounds;
        ancestor = node.parent;
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
} // namespace KICAD_AUTOROUTER
