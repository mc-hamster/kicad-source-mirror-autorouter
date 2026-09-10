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
#include "../model/items/NormalContacts.h"
#include "../../geometry/planar/ContactGeometry.h"

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
        std::set<ITEM_ID> normalContacts;
        NORMAL_CONTACT_ITEM normal;
        std::optional<ROUTING_OBSTACLE> trace;
        std::shared_ptr<const ROUTING_OBSTACLE> area;
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
        {
            item.shapes.push_back( { item.id, layer, geometry } );
            if( std::find( item.normal.layers.begin(), item.normal.layers.end(), layer )
                == item.normal.layers.end() )
                item.normal.layers.push_back( layer );
        }
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
                    if( otherPart.layer == part.layer )
                    {
                        const auto& area = item.conductionArea ? item : other;
                        auto contains = [&]( ROUTER_POINT p )
                        { return area.area && CONTACT_GEOMETRY::ContainsArea( *area.area, p ); };
                        if( item.normal.Touches( other.normal, contains ) )
                        {
                            item.normalContacts.insert( other.id );
                            other.normalContacts.insert( item.id );
                        }
                        if( part.geometry->Collide( otherPart.geometry.get(), 0 ) )
                        {
                            item.contacts.insert( other.id );
                            other.contacts.insert( item.id );
                        }
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
        {
            item.contacts.clear();
            item.normalContacts.clear();
        }
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
        for( ITEM_ID contact : it->second.normalContacts )
            items.at( contact ).normalContacts.erase( id );
        for( const auto& part : it->second.shapes )
            index.at( part.layer )->Remove( &part );
        items.erase( it );
    }

    // Split straight copper in the WORKER graph at exact centre-line contacts.
    // Retained copper is split virtually only: the original host item is not
    // edited/deleted, and these pieces never become result-emitter output.
    void normalizeJunctions( const std::vector<ITEM_ID>& added )
    {
        using namespace CONTACT_GEOMETRY;
        std::map<ITEM_ID, std::vector<ROUTER_POINT>> cuts;
        auto cut = [&]( const ITEM& item, ROUTER_POINT p )
        {
            if( item.trace && p != item.normal.first && p != item.normal.last
                && OnSegment( item.normal.first, item.normal.last, p ) )
                cuts[item.id].push_back( p );
        };
        for( auto id : added )
        {
            const auto& item = items.at( id );
            std::set<ITEM_ID> candidates;
            for( const auto& part : item.shapes )
            {
                auto visitor = [&]( const LAYER_SHAPE* other )
                {
                    if( other->owner != id )
                        candidates.insert( other->owner );
                    return true;
                };
                index.at( part.layer )->Query( part.geometry.get(), 0, visitor );
            }
            for( auto otherId : candidates )
            {
                const auto& other = items.at( otherId );
                if( item.net != other.net || !item.normal.SharesLayer( other.normal ) )
                    continue;
                const auto a = item.normal.first, b = item.normal.last;
                const auto c = other.normal.first, d = other.normal.last;
                using KIND = NORMAL_CONTACT_ITEM::KIND;
                if( item.normal.kind == KIND::TRACE && other.normal.kind == KIND::TRACE )
                {
                    cut( item, c ); cut( item, d );
                    cut( other, a ); cut( other, b );
                    if( auto p = Intersection( a, b, c, d ) )
                    {
                        cut( item, *p );
                        cut( other, *p );
                    }
                }
                else if( item.normal.kind == KIND::DRILL && other.normal.kind == KIND::TRACE )
                    cut( other, a );
                else if( item.normal.kind == KIND::TRACE && other.normal.kind == KIND::DRILL )
                    cut( item, c );
            }
        }
        for( auto& [id, points] : cuts )
        {
            const auto original = items.at( id );
            const auto start = original.normal.first, end = original.normal.last;
            points.push_back( start );
            points.push_back( end );
            std::sort( points.begin(), points.end(), [&]( auto a, auto b )
            {
                if( start.x != end.x )
                    return start.x < end.x ? a.x < b.x : a.x > b.x;
                return start.y < end.y ? a.y < b.y : a.y > b.y;
            } );
            points.erase( std::unique( points.begin(), points.end() ), points.end() );
            removeItem( id );
            std::vector<ITEM_ID> replacements;
            for( std::size_t i = 1; i < points.size(); ++i )
            {
                // Preserve the original identity on the first half. Only the
                // additional halves allocate IDs; unrelated items never move.
                ITEM piece = original;
                piece.id = i == 1 ? id : nextId++;
                piece.contacts.clear();
                piece.normalContacts.clear();
                piece.shapes.clear();
                piece.normal.first = points[i - 1];
                piece.normal.last = points[i];
                piece.trace->start = points[i - 1];
                piece.trace->end = points[i];
                for( auto& terminal : piece.terminals )
                {
                    terminal.pad.position = points[i - 1];
                    terminal.segmentEnd = points[i];
                }
                addShape( piece, *piece.trace );
                auto [it, inserted] = items.emplace( piece.id, std::move( piece ) );
                indexItem( it->second );
                replacements.push_back( it->first );
            }
            for( auto& route : routes )
            {
                auto it = std::find( route.items.begin(), route.items.end(), id );
                if( it == route.items.end() )
                    continue;
                const auto offset = it - route.items.begin();
                route.items.erase( it );
                route.items.insert( route.items.begin() + offset, replacements.begin(), replacements.end() );
                break;
            }
        }
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
        item.normal.kind = NORMAL_CONTACT_ITEM::KIND::DRILL;
        item.normal.first = item.normal.last = pad.position;
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
        // During a whole-net reroute, unsupported original copper is copied
        // into the immutable obstacle list only to keep it physically
        // blocking.  It is intentionally not electrical worker copper: if
        // it made a component here, CountMissing() could declare a task done
        // before the proposal regenerated (or safely shoved) that BOARD_ITEM.
        if( copper.netCode <= 0 || copper.isHole || copper.isKeepout
            || copper.isCollisionOnly )
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
            // A host arc tessellated into multiple shapes is not one straight trace.
            item.normal.kind = !item.terminals.empty() ? NORMAL_CONTACT_ITEM::KIND::UNKNOWN
                    : copper.start == copper.end ? NORMAL_CONTACT_ITEM::KIND::DRILL
                                                  : NORMAL_CONTACT_ITEM::KIND::TRACE;
            item.normal.first = copper.start;
            item.normal.last = copper.end;
            if( item.normal.kind == NORMAL_CONTACT_ITEM::KIND::TRACE )
                item.trace = copper;
            else
                item.trace.reset();
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
        item.area = std::make_shared<const ROUTING_OBSTACLE>( area );
        item.normal.kind = NORMAL_CONTACT_ITEM::KIND::AREA;
        state.addShape( item, area );
    }
    state.reindex();
}

ROUTING_BOARD::~ROUTING_BOARD() = default;

void ROUTING_BOARD::AddRoute( const ROUTING_CONNECTION& route )
{
    if( !route.complete )
        return;

    if( !HasValidEdgeStyles( route ) )
        throw std::invalid_argument( "Invalid routing-board edge-style count" );

    auto& state = *m_impl;
    IMPL::ROUTE record{ route, {} };
    std::int64_t width = 0, diameter = 600000, drill = 300000;
    for( const auto& net : state.snapshot.nets )
        if( net.netCode == route.netCode )
        {
            for( auto index : net.padIndices )
                width = std::max( width, state.snapshot.pads.at( index ).trackWidth );
            if( net.viaDiameter > 0 )
                diameter = net.viaDiameter;
            if( net.viaDrill > 0 )
                drill = net.viaDrill;
        }

    if( width <= 0 )
        width = 150000;

    // Reject all invalid coordinates/dimensions before changing any copper,
    // including a malformed later edge after otherwise valid first edges.
    coordinate( width );
    coordinate( diameter );
    coordinate( drill );
    for( const auto& node : route.nodes )
        point( node.point );

    // Reject malformed layer transitions before changing any copper.
    for( std::size_t i = 1; i < route.nodes.size(); ++i )
    {
        const ROUTING_EDGE_STYLE& style = EdgeStyle( route, i - 1 );
        coordinate( style.trackWidth > 0 ? style.trackWidth : width );
        coordinate( style.viaDiameter > 0 ? style.viaDiameter : diameter );
        coordinate( style.viaDrill > 0 ? style.viaDrill : drill );

        if( route.nodes[i - 1].layer != route.nodes[i].layer
            && ( route.nodes[i - 1].point != route.nodes[i].point
                 || !VIA_RULE::AllowsTransition( state.settings, route.nodes[i - 1].layer,
                                                 route.nodes[i].layer, &style ) ) )
            throw std::invalid_argument( "Invalid routing-board via transition" );

        if( route.nodes[i - 1].layer != route.nodes[i].layer
            && VIA_RULE::LayersFor( state.settings, route.nodes[i - 1].layer,
                                    route.nodes[i].layer, &style ).empty() )
        {
            throw std::invalid_argument( "Invalid routing-board via layer mask" );
        }
    }

    TRANSACTION transaction( *this );
    for( std::size_t i = 1; i < route.nodes.size(); ++i )
    {
        const auto& from = route.nodes[i - 1];
        const auto& to = route.nodes[i];
        if( from == to )
            continue;
        const bool via = from.layer != to.layer;
        const ROUTING_EDGE_STYLE& style = EdgeStyle( route, i - 1 );
        const std::int64_t edgeWidth = style.trackWidth > 0 ? style.trackWidth : width;
        const std::int64_t edgeDiameter = style.viaDiameter > 0 ? style.viaDiameter : diameter;
        const std::vector<int> edgeLayers = via
                ? VIA_RULE::LayersFor( state.settings, from.layer, to.layer, &style )
                : std::vector<int>{ from.layer };
        auto& item = state.newItem( route.netCode, true );
        ROUTING_OBSTACLE copper;
        copper.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
        copper.start = from.point;
        copper.end = to.point;
        copper.radius = ( via ? edgeDiameter : edgeWidth ) / 2;
        copper.clearance = std::max<std::int64_t>( 0, style.clearance );
        copper.layers = via ? edgeLayers
                            : std::vector<int>{ from.layer };
        item.normal.kind = via ? NORMAL_CONTACT_ITEM::KIND::DRILL
                               : NORMAL_CONTACT_ITEM::KIND::TRACE;
        item.normal.first = from.point;
        item.normal.last = to.point;
        if( !via )
            item.trace = copper;
        state.addShape( item, copper );
        for( int layer : copper.layers )
        {
            ROUTING_PAD terminal;
            terminal.netCode = route.netCode;
            terminal.position = from.point;
            terminal.layers = { layer };
            terminal.trackWidth = edgeWidth;
            item.terminals.push_back( { terminal, NO_PAD, via ? std::nullopt
                                                              : std::optional( to.point ) } );
        }
        record.items.push_back( item.id );
        state.indexItem( item );
    }
    const auto addedItems = record.items;
    state.routes.push_back( std::move( record ) );
    state.normalizeJunctions( addedItems );
    ++state.revision;
    transaction.Commit();
}

void ROUTING_BOARD::RemoveRoute( const ROUTING_CONNECTION& route )
{
    auto& state = *m_impl;
    const auto it = std::find_if( state.routes.begin(), state.routes.end(), [&]( const auto& existing )
    { return SameRouteGeometry( existing.connection, route ); } );
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

void ROUTING_BOARD::RelocateSyntheticPad( std::size_t pad, ROUTER_POINT position )
{
    if( pad >= m_impl->snapshot.pads.size() )
        return;

    ROUTING_PAD& terminal = m_impl->snapshot.pads[pad];
    if( !terminal.isFanoutTarget && !terminal.isPlaneTarget )
        return;

    terminal.position = position;
}

bool ROUTING_BOARD::Connected( std::size_t first, std::size_t second ) const
{
    const auto left = m_impl->padRoots( first );
    const auto right = m_impl->padRoots( second );
    return std::any_of( left.begin(), left.end(), [&]( ITEM_ID root ) { return right.contains( root ); } );
}

bool ROUTING_BOARD::ConnectedSetTouchesOtherLayer( std::size_t pad, int layer ) const
{
    const auto roots = m_impl->padRoots( pad );
    if( roots.empty() )
        return false;

    m_impl->updateComponents();
    return std::any_of(
            m_impl->items.begin(), m_impl->items.end(),
            [&]( const auto& aEntry )
            {
                const auto& [id, item] = aEntry;
                if( !roots.contains( m_impl->components.at( id ) ) )
                    return false;

                return std::any_of( item.shapes.begin(), item.shapes.end(),
                                    [&]( const auto& aShape )
                                    { return aShape.layer != layer; } );
            } );
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

std::set<ROUTING_BOARD::ITEM_ID> ROUTING_BOARD::GetNormalContacts( ITEM_ID id ) const
{
    const auto it = m_impl->items.find( id );
    return it == m_impl->items.end() ? std::set<ITEM_ID>{} : it->second.normalContacts;
}

std::optional<ROUTER_POINT> ROUTING_BOARD::NormalContactPoint( ITEM_ID first, ITEM_ID second ) const
{
    const auto a = m_impl->items.find( first ), b = m_impl->items.find( second );
    if( a == m_impl->items.end() || b == m_impl->items.end() || first == second )
        return {};
    return a->second.normal.Point( b->second.normal );
}

std::set<ROUTING_BOARD::ITEM_ID> ROUTING_BOARD::NormalConnectedSet( ITEM_ID id ) const
{
    if( !m_impl->items.contains( id ) )
        return {};
    std::set<ITEM_ID> result{ id };
    std::vector<ITEM_ID> pending{ id };
    while( !pending.empty() )
    {
        const auto current = pending.back();
        pending.pop_back();
        for( auto contact : m_impl->items.at( current ).normalContacts )
            if( result.insert( contact ).second )
                pending.push_back( contact );
    }
    return result;
}

std::vector<ROUTING_BOARD::ITEM_ID> ROUTING_BOARD::RouteItems( const ROUTING_CONNECTION& route ) const
{
    for( const auto& record : m_impl->routes )
        if( SameRouteGeometry( record.connection, route ) )
            return record.items;
    return {};
}

std::optional<ROUTING_BOARD::ITEM_ID> ROUTING_BOARD::PadItem( std::size_t pad ) const
{
    const auto it = m_impl->pads.find( pad );
    return it == m_impl->pads.end() ? std::nullopt : std::optional( it->second );
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
    using namespace CONTACT_GEOMETRY;
    std::vector<ROUTER_POINT> points;
    auto add = [&]( ROUTER_POINT p )
    {
        if( OnSegment( start.point, end.point, p ) )
            points.push_back( p );
    };
    for( auto id : candidates )
        for( const auto& terminal : m_impl->items.at( id ).terminals )
        {
            if( std::find( terminal.pad.layers.begin(), terminal.pad.layers.end(), start.layer )
                    == terminal.pad.layers.end() )
                continue;
            const auto a = terminal.pad.position;
            add( a );
            if( terminal.segmentEnd )
            {
                const auto b = *terminal.segmentEnd;
                add( b );
                if( const auto p = Intersection( start.point, end.point, a, b ) )
                    add( *p );
            }
        }
    // All points lie exactly on one line; order by the varying coordinate.
    // No squared integer norm (overflow) or rounded KiMath distance predicate.
    std::sort( points.begin(), points.end(), [&]( auto a, auto b )
    {
        if( start.point.x != end.point.x )
            return start.point.x < end.point.x ? a.x < b.x : a.x > b.x;
        return start.point.y < end.point.y ? a.y < b.y : a.y > b.y;
    } );
    points.erase( std::unique( points.begin(), points.end() ), points.end() );
    return points;
}

std::size_t ROUTING_BOARD::ItemCount() const { return m_impl->items.size(); }
std::uint64_t ROUTING_BOARD::Revision() const { return m_impl->revision; }

struct ROUTING_BOARD::TRANSACTION::STATE
{
    std::map<ITEM_ID, IMPL::ITEM> items;
    std::vector<IMPL::ROUTE> routes;
    ITEM_ID nextId;
    // Built before editing, pointing at the saved map nodes. Restoring during
    // stack unwinding must never allocate/reindex (and throw a second failure).
    decltype( IMPL::index ) index;
};

ROUTING_BOARD::TRANSACTION::TRANSACTION( ROUTING_BOARD& board ) : m_board( board ),
    m_before( std::make_unique<STATE>( STATE{ board.m_impl->items, board.m_impl->routes,
                                             board.m_impl->nextId, {} } ) )
{
    for( const auto& [id, item] : m_before->items )
        for( const auto& part : item.shapes )
        {
            auto& tree = m_before->index[part.layer];
            if( !tree )
                tree = std::make_unique<SHAPE_INDEX<const IMPL::LAYER_SHAPE*>>( part.layer );
            tree->Add( &part );
        }
}

ROUTING_BOARD::TRANSACTION::~TRANSACTION()
{
    if( !m_before )
        return;
    // std::map::swap preserves its nodes' addresses. Both pointer-bearing
    // indexes therefore stay paired with exactly the item storage they index.
    m_board.m_impl->index.swap( m_before->index );
    m_board.m_impl->items.swap( m_before->items );
    m_board.m_impl->routes.swap( m_before->routes );
    std::swap( m_board.m_impl->nextId, m_before->nextId );
    ++m_board.m_impl->revision;
    m_board.m_impl->components.clear();
    m_board.m_impl->componentRevision = std::numeric_limits<std::uint64_t>::max();
}

void ROUTING_BOARD::TRANSACTION::Commit() { m_before.reset(); }

} // namespace KICAD_AUTOROUTER
