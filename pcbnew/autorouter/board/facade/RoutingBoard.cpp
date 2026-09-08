/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Worker-local item/contact model corresponding to Freerouting RoutingBoard.
 * KiMath supplies host-coordinate geometry; no editor or wxWidgets objects are used.
 */
#include "RoutingBoard.h"

#include <algorithm>
#include <limits>
#include <stdexcept>

#include <geometry/shape_circle.h>
#include <geometry/shape_compound.h>
#include <geometry/shape_index.h>
#include <geometry/shape_poly_set.h>
#include <geometry/shape_rect.h>
#include <geometry/shape_segment.h>

#include "../../rules/ViaRule.h"

namespace KICAD_AUTOROUTER
{
namespace
{
constexpr std::size_t NO_PAD = std::numeric_limits<std::size_t>::max();

int coordinate( std::int64_t aValue )
{
    if( aValue < std::numeric_limits<int>::min() || aValue > std::numeric_limits<int>::max() )
        throw std::out_of_range( "Routing copper exceeds KiCad coordinate range" );
    return static_cast<int>( aValue );
}

VECTOR2I point( const ROUTER_POINT& aPoint )
{
    return { coordinate( aPoint.x ), coordinate( aPoint.y ) };
}

std::shared_ptr<const SHAPE> shape( const ROUTING_OBSTACLE& aCopper )
{
    if( aCopper.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
        return std::make_shared<SHAPE_SEGMENT>( point( aCopper.start ), point( aCopper.end ),
                                               coordinate( 2 * aCopper.radius ) );
    if( aCopper.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
        return std::make_shared<SHAPE_RECT>(
                VECTOR2I( coordinate( aCopper.box.minX ), coordinate( aCopper.box.minY ) ),
                VECTOR2I( coordinate( aCopper.box.maxX ), coordinate( aCopper.box.maxY ) ) );

    auto polygon = std::make_unique<SHAPE_POLY_SET>();
    polygon->NewOutline();
    for( const auto& p : aCopper.polygon )
        polygon->Append( point( p ) );
    for( const auto& hole : aCopper.polygonHoles )
    {
        const int index = polygon->NewHole();
        for( const auto& p : hole )
            polygon->Append( point( p ), 0, index );
    }
    if( aCopper.radius == 0 )
        return std::shared_ptr<const SHAPE>( polygon.release() );

    // Minkowski disk sweep, preserving the polygon interior and shrinking holes.
    auto expanded = std::make_shared<SHAPE_COMPOUND>();
    expanded->AddShape( polygon.release() );
    const auto addEdges = [&]( const std::vector<ROUTER_POINT>& points )
    {
        for( std::size_t i = 0; i < points.size(); ++i )
            expanded->AddShape( new SHAPE_SEGMENT( point( points[i] ),
                    point( points[( i + 1 ) % points.size()] ), coordinate( 2 * aCopper.radius ) ) );
    };
    addEdges( aCopper.polygon );
    for( const auto& hole : aCopper.polygonHoles )
        addEdges( hole );
    return expanded;
}
}

struct ROUTING_BOARD::IMPL
{
    struct LAYER_SHAPE
    {
        ITEM_ID owner;
        int layer;
        std::shared_ptr<const SHAPE> geometry;
        const SHAPE* Shape( int ) const { return geometry.get(); }
    };
    struct ITEM
    {
        ITEM_ID id = 0;
        int net = 0;
        std::size_t pad = NO_PAD;
        bool dynamic = false;
        bool conductionArea = false;
        std::vector<LAYER_SHAPE> shapes;
        std::set<ITEM_ID> contacts;
        std::vector<ROUTING_TERMINAL> terminals;
    };
    struct ROUTE
    {
        ROUTING_CONNECTION connection;
        std::vector<ITEM_ID> items;
    };

    BOARD_SNAPSHOT snapshot;
    AUTOROUTER_SETTINGS settings;
    std::map<ITEM_ID, ITEM> items;
    std::map<std::size_t, ITEM_ID> pads;
    std::map<int, std::unique_ptr<SHAPE_INDEX<const LAYER_SHAPE*>>> index;
    std::vector<ROUTE> routes;
    ITEM_ID nextId = 1;
    std::uint64_t revision = 0;
    mutable std::uint64_t componentRevision = std::numeric_limits<std::uint64_t>::max();
    mutable std::map<ITEM_ID, ITEM_ID> components;

    void addShape( ITEM& item, const ROUTING_OBSTACLE& copper )
    {
        const auto geometry = shape( copper );
        auto layers = copper.layers.empty() ? VIA_RULE::ThroughLayers( settings ) : copper.layers;
        for( int layer : layers )
            item.shapes.push_back( { item.id, layer, geometry } );
    }

    ITEM& newItem( int net, bool dynamic = false )
    {
        const ITEM_ID id = nextId++;
        auto& result = items[id];
        result.id = id;
        result.net = net;
        result.dynamic = dynamic;
        return result;
    }

    void indexItem( ITEM& item )
    {
        for( const auto& part : item.shapes )
        {
            auto& tree = index[part.layer];
            if( !tree )
                tree = std::make_unique<SHAPE_INDEX<const LAYER_SHAPE*>>( part.layer );
            std::set<ITEM_ID> candidates;
            auto visitor = [&]( const LAYER_SHAPE* other )
            {
                if( other->owner != item.id )
                    candidates.insert( other->owner );
                return true;
            };
            tree->Query( part.geometry.get(), 0, visitor );
            // R-tree visitation order must not become routing decision order.
            for( ITEM_ID candidate : candidates )
            {
                auto& other = items.at( candidate );
                if( other.net != item.net )
                    continue;
                for( const auto& otherPart : other.shapes )
                {
                    if( otherPart.layer == part.layer
                        && part.geometry->Collide( otherPart.geometry.get(), 0 ) )
                    {
                        item.contacts.insert( other.id );
                        other.contacts.insert( item.id );
                        break;
                    }
                }
            }
            tree->Add( &part );
        }
    }

    void reindex()
    {
        index.clear();
        for( auto& [id, item] : items )
            item.contacts.clear();
        for( auto& [id, item] : items )
            indexItem( item );
        // Retained host copper can include item types not yet copied as shapes.
        // These immutable contacts are absent when existing copper is ripped up.
        for( const auto& net : snapshot.nets )
        {
            for( const auto& group : net.connectedPadGroups )
            {
                if( group.empty() || !pads.contains( group.front() ) )
                    continue;
                const auto first = pads.at( group.front() );
                for( auto pad : group )
                {
                    if( pads.contains( pad ) && pads.at( pad ) != first )
                    {
                        items.at( first ).contacts.insert( pads.at( pad ) );
                        items.at( pads.at( pad ) ).contacts.insert( first );
                    }
                }
            }
        }
        ++revision;
    }

    void removeItem( ITEM_ID id )
    {
        auto it = items.find( id );
        if( it == items.end() )
            return;
        for( ITEM_ID contact : it->second.contacts )
            items.at( contact ).contacts.erase( id );
        for( const auto& part : it->second.shapes )
            index.at( part.layer )->Remove( &part );
        items.erase( it );
    }

    void updateComponents() const
    {
        if( componentRevision == revision )
            return;
        components.clear();
        for( const auto& [id, item] : items )
        {
            if( components.contains( id ) )
                continue;
            std::vector<ITEM_ID> pending{ id };
            components[id] = id;
            while( !pending.empty() )
            {
                const ITEM_ID current = pending.back();
                pending.pop_back();
                for( auto contact : items.at( current ).contacts )
                {
                    if( components.emplace( contact, id ).second )
                        pending.push_back( contact );
                }
            }
        }
        componentRevision = revision;
    }

    std::set<ITEM_ID> padRoots( std::size_t pad ) const
    {
        updateComponents();
        if( pads.contains( pad ) )
            return { components.at( pads.at( pad ) ) };
        if( pad >= snapshot.pads.size() )
            return {};
        // A synthetic landing has no copper of its own. Resolve it only against
        // inserted/retained geometry, and never join layers via a virtual pad.
        const auto& terminal = snapshot.pads[pad];
        const SHAPE_CIRCLE probe( point( terminal.position ), 0 );
        std::set<ITEM_ID> result;
        for( int layer : terminal.layers )
        {
            if( terminal.isFanoutTarget && terminal.fanoutTargetLayer >= 0
                && layer != terminal.fanoutTargetLayer )
                continue;
            const auto tree = index.find( layer );
            if( tree == index.end() )
                continue;
            auto visitor = [&]( const LAYER_SHAPE* part )
            {
                const auto& item = items.at( part->owner );
                if( item.net == terminal.netCode
                    && ( !terminal.isPlaneTarget || item.conductionArea )
                    && part->geometry->Collide( &probe, 0 ) )
                    result.insert( components.at( item.id ) );
                return true;
            };
            tree->second->Query( &probe, 0, visitor );
        }
        return result;
    }
};

ROUTING_BOARD::ROUTING_BOARD( const BOARD_SNAPSHOT& snapshot,
                              const AUTOROUTER_SETTINGS& settings ) : m_impl( std::make_unique<IMPL>() )
{
    auto& state = *m_impl;
    state.snapshot = snapshot;
    state.settings = settings;
    std::map<std::string, ITEM_ID> hostItems;
    for( std::size_t i = 0; i < snapshot.pads.size(); ++i )
    {
        const auto& pad = snapshot.pads[i];
        if( pad.isFanoutTarget || pad.isPlaneTarget || pad.netCode <= 0 )
            continue;
        auto& item = state.newItem( pad.netCode );
        item.pad = i;
        state.pads[i] = item.id;
        if( !pad.sourceId.empty() )
            hostItems[pad.sourceId] = item.id;
        else
        {
            ROUTING_OBSTACLE copper;
            copper.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
            copper.start = copper.end = pad.position;
            copper.radius = pad.radius;
            copper.layers = pad.layers;
            state.addShape( item, copper );
        }
        item.terminals.push_back( { pad, i, {} } );
    }
    for( const auto& copper : snapshot.obstacles )
    {
        if( copper.netCode <= 0 || copper.isHole || copper.isKeepout )
            continue;
        ITEM_ID id;
        if( !copper.boardItemId.empty() && hostItems.contains( copper.boardItemId ) )
            id = hostItems.at( copper.boardItemId );
        else
        {
            id = state.newItem( copper.netCode ).id;
            if( !copper.boardItemId.empty() )
                hostItems[copper.boardItemId] = id;
        }
        auto& item = state.items.at( id );
        state.addShape( item, copper );
        if( copper.isExistingRoute && copper.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
        {
            for( int layer : copper.layers )
            {
                ROUTING_PAD pad;
                pad.netCode = copper.netCode;
                pad.position = copper.start;
                pad.layers = { layer };
                pad.trackWidth = 2 * copper.radius;
                item.terminals.push_back( { pad, NO_PAD, copper.end } );
            }
        }
    }
    for( const auto& area : snapshot.conductionAreas )
    {
        auto& item = state.newItem( area.netCode );
        item.conductionArea = true;
        state.addShape( item, area );
    }
    state.reindex();
}

ROUTING_BOARD::~ROUTING_BOARD() = default;

void ROUTING_BOARD::AddRoute( const ROUTING_CONNECTION& route )
{
    if( !route.complete )
        return;
    auto& state = *m_impl;
    IMPL::ROUTE record{ route, {} };
    std::int64_t width = 0, diameter = 600000;
    for( const auto& net : state.snapshot.nets )
        if( net.netCode == route.netCode )
            for( auto index : net.padIndices )
                width = std::max( width, state.snapshot.pads.at( index ).trackWidth );
    if( width <= 0 )
        width = 150000;
    for( const auto& net : state.snapshot.nets )
        if( net.netCode == route.netCode && net.viaDiameter > 0 )
        { diameter = net.viaDiameter; break; }
    // Reject all invalid coordinates/dimensions before changing any copper,
    // including a malformed later edge after otherwise valid first edges.
    coordinate( width );
    coordinate( diameter );
    for( const auto& node : route.nodes )
        point( node.point );
    // Reject malformed layer transitions before changing any copper.
    for( std::size_t i = 1; i < route.nodes.size(); ++i )
        if( route.nodes[i - 1].layer != route.nodes[i].layer
            && ( route.nodes[i - 1].point != route.nodes[i].point
                 || !VIA_RULE::AllowsTransition( state.settings, route.nodes[i - 1].layer,
                                                 route.nodes[i].layer ) ) )
            throw std::invalid_argument( "Invalid routing-board via transition" );
    for( std::size_t i = 1; i < route.nodes.size(); ++i )
    {
        const auto& from = route.nodes[i - 1];
        const auto& to = route.nodes[i];
        if( from == to )
            continue;
        const bool via = from.layer != to.layer;
        auto& item = state.newItem( route.netCode, true );
        ROUTING_OBSTACLE copper;
        copper.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
        copper.start = from.point;
        copper.end = to.point;
        copper.radius = ( via ? diameter : width ) / 2;
        copper.layers = via ? VIA_RULE::ThroughLayers( state.settings )
                            : std::vector<int>{ from.layer };
        state.addShape( item, copper );
        for( int layer : copper.layers )
        {
            ROUTING_PAD terminal;
            terminal.netCode = route.netCode;
            terminal.position = from.point;
            terminal.layers = { layer };
            terminal.trackWidth = width;
            item.terminals.push_back( { terminal, NO_PAD, via ? std::nullopt
                                                              : std::optional( to.point ) } );
        }
        record.items.push_back( item.id );
        state.indexItem( item );
    }
    state.routes.push_back( std::move( record ) );
    ++state.revision;
}

void ROUTING_BOARD::RemoveRoute( const ROUTING_CONNECTION& route )
{
    auto& state = *m_impl;
    const auto it = std::find_if( state.routes.begin(), state.routes.end(), [&]( const auto& existing )
    { return existing.connection.netCode == route.netCode && existing.connection.nodes == route.nodes; } );
    if( it == state.routes.end() )
        return;
    for( ITEM_ID id : it->items )
        state.removeItem( id );
    state.routes.erase( it );
    ++state.revision;
}

void ROUTING_BOARD::ClearRoutes()
{
    while( !m_impl->routes.empty() )
    {
        const auto route = m_impl->routes.back().connection;
        RemoveRoute( route );
    }
}

bool ROUTING_BOARD::Connected( std::size_t first, std::size_t second ) const
{
    const auto left = m_impl->padRoots( first );
    const auto right = m_impl->padRoots( second );
    return std::any_of( left.begin(), left.end(), [&]( ITEM_ID root ) { return right.contains( root ); } );
}

std::vector<std::vector<std::size_t>> ROUTING_BOARD::ConnectedPadGroups( int net ) const
{
    std::map<ITEM_ID, std::vector<std::size_t>> groups;
    for( std::size_t pad = 0; pad < m_impl->snapshot.pads.size(); ++pad )
    {
        if( m_impl->snapshot.pads[pad].netCode != net )
            continue;
        const auto roots = m_impl->padRoots( pad );
        if( roots.size() == 1 )
            groups[*roots.begin()].push_back( pad );
    }
    std::vector<std::vector<std::size_t>> result;
    for( auto& [root, group] : groups )
        if( group.size() > 1 )
            result.push_back( std::move( group ) );
    return result;
}

int ROUTING_BOARD::CountMissing( const ROUTING_NET& net ) const
{
    int result = 0;
    for( const auto& [from, to] : net.connections )
        if( !Connected( from, to ) )
            ++result;
    return result;
}

std::vector<ROUTING_TERMINAL> ROUTING_BOARD::Terminals( std::size_t pad ) const
{
    const auto roots = m_impl->padRoots( pad );
    std::vector<ROUTING_TERMINAL> result;
    for( const auto& [id, item] : m_impl->items )
    {
        if( !roots.contains( m_impl->components.at( id ) ) )
            continue;
        for( auto terminal : item.terminals )
        {
            if( terminal.padIndex == NO_PAD )
                terminal.padIndex = pad;
            result.push_back( std::move( terminal ) );
        }
    }
    return result;
}

std::set<ROUTING_BOARD::ITEM_ID> ROUTING_BOARD::ConnectedSet( ITEM_ID item ) const
{
    m_impl->updateComponents();
    std::set<ITEM_ID> result;
    if( !m_impl->components.contains( item ) )
        return result;
    const auto root = m_impl->components.at( item );
    for( const auto& [id, component] : m_impl->components )
        if( component == root )
            result.insert( id );
    return result;
}

bool ROUTING_BOARD::HasCopperAt( int net, ROUTER_NODE node, std::int64_t radius ) const
{
    const auto tree = m_impl->index.find( node.layer );
    if( tree == m_impl->index.end() )
        return false;
    const SHAPE_CIRCLE probe( point( node.point ), coordinate( radius ) );
    bool contact = false;
    auto visitor = [&]( const IMPL::LAYER_SHAPE* part )
    {
        if( m_impl->items.at( part->owner ).net == net && part->geometry->Collide( &probe, 0 ) )
            contact = true;
        return !contact;
    };
    tree->second->Query( &probe, 0, visitor );
    return contact;
}

std::vector<ROUTER_POINT> ROUTING_BOARD::TraceJunctions(
        int net, ROUTER_NODE start, ROUTER_NODE end ) const
{
    const auto tree = m_impl->index.find( start.layer );
    if( start.layer != end.layer || tree == m_impl->index.end() )
        return {};
    const SEG line( point( start.point ), point( end.point ) );
    const SHAPE_SEGMENT probe( line, 0 );
    std::set<ITEM_ID> candidates;
    auto visitor = [&]( const IMPL::LAYER_SHAPE* part )
    {
        if( m_impl->items.at( part->owner ).net == net )
            candidates.insert( part->owner );
        return true;
    };
    tree->second->Query( &probe, 0, visitor );
    std::vector<VECTOR2I> points;
    auto add = [&]( VECTOR2I p )
    {
        if( line.SquaredDistance( p ) == 0 )
            points.push_back( p );
    };
    for( auto id : candidates )
        for( const auto& terminal : m_impl->items.at( id ).terminals )
        {
            if( std::find( terminal.pad.layers.begin(), terminal.pad.layers.end(), start.layer )
                    == terminal.pad.layers.end() )
                continue;
            const auto a = point( terminal.pad.position );
            add( a );
            if( terminal.segmentEnd )
            {
                const auto b = point( *terminal.segmentEnd );
                add( b );
                const SEG other( a, b );
                if( const auto p = line.Intersect( other ); p && other.SquaredDistance( *p ) == 0 )
                    add( *p );
            }
        }
    std::sort( points.begin(), points.end(), [&]( auto a, auto b )
    { return ( a - line.A ).SquaredEuclideanNorm() < ( b - line.A ).SquaredEuclideanNorm(); } );
    points.erase( std::unique( points.begin(), points.end() ), points.end() );
    std::vector<ROUTER_POINT> result;
    for( auto p : points )
        result.push_back( { p.x, p.y } );
    return result;
}

std::size_t ROUTING_BOARD::ItemCount() const { return m_impl->items.size(); }
std::uint64_t ROUTING_BOARD::Revision() const { return m_impl->revision; }

struct ROUTING_BOARD::TRANSACTION::STATE
{
    std::map<ITEM_ID, IMPL::ITEM> items;
    std::vector<IMPL::ROUTE> routes;
    ITEM_ID nextId;
};

ROUTING_BOARD::TRANSACTION::TRANSACTION( ROUTING_BOARD& board ) : m_board( board ),
    m_before( std::make_unique<STATE>( STATE{ board.m_impl->items, board.m_impl->routes,
                                             board.m_impl->nextId } ) )
{}

ROUTING_BOARD::TRANSACTION::~TRANSACTION()
{
    if( !m_before )
        return;
    // Remove pointer-bearing indexes before replacing their item storage.
    m_board.m_impl->index.clear();
    m_board.m_impl->items = std::move( m_before->items );
    m_board.m_impl->routes = std::move( m_before->routes );
    m_board.m_impl->nextId = m_before->nextId;
    m_board.m_impl->reindex();
}

void ROUTING_BOARD::TRANSACTION::Commit() { m_before.reset(); }

} // namespace KICAD_AUTOROUTER
