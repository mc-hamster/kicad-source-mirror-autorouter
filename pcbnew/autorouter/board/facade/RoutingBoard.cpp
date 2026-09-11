/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * Worker-local item/contact model corresponding to Freerouting RoutingBoard.
 * KiMath supplies host-coordinate geometry; no editor or wxWidgets objects are used.
 */
#include "RoutingBoard.h"

#include <algorithm>
#include <cmath>
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
#include "../../geometry/planar/Polyline.h"

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
        bool routable = false;
        ROUTER_FIXED_STATE fixedState = ROUTER_FIXED_STATE::SYSTEM_FIXED;
        bool conductionArea = false;
        std::vector<LAYER_SHAPE> shapes;
        std::set<ITEM_ID> contacts;
        ITEM_ID_SET normalContacts;
        // Exact non-endpoint contacts retained when the source RationalPoint
        // cannot be represented as a KiCad integer trace vertex.
        std::map<ITEM_ID, PLANAR::POINT> exactContactPoints;
        NORMAL_CONTACT_ITEM normal;
        std::optional<ROUTING_OBSTACLE> trace;
        std::vector<ROUTER_POINT> traceCorners;
        // Exact item-local route geometry. Freerouting stores a PolylineTrace
        // or DrillItem directly on RoutingBoard; retaining the corresponding
        // value here lets item-chain removal reconstruct the remaining board
        // without falling back to the larger route request that created it.
        std::optional<ROUTING_CONNECTION> route;
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
        result.routable = dynamic;
        result.fixedState = dynamic ? ROUTER_FIXED_STATE::UNFIXED
                                    : ROUTER_FIXED_STATE::SYSTEM_FIXED;
        return result;
    }

    static bool deletionForbidden( const ITEM& item )
    {
        return item.fixedState >= ROUTER_FIXED_STATE::USER_FIXED;
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
            item.exactContactPoints.clear();
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
        {
            items.at( contact ).normalContacts.erase( id );
            items.at( contact ).exactContactPoints.erase( id );
        }
        for( const auto& part : it->second.shapes )
            index.at( part.layer )->Remove( &part );
        items.erase( it );
    }

    ITEM_ID_SET normalContactsAt( ITEM_ID traceId, ROUTER_POINT point,
                                  bool ignoreAreas = false ) const
    {
        const auto source = items.find( traceId );
        if( source == items.end()
            || source->second.normal.kind != NORMAL_CONTACT_ITEM::KIND::TRACE
            || ( point != source->second.normal.first
                 && point != source->second.normal.last ) )
        {
            return {};
        }

        ITEM_ID_SET result;
        for( ITEM_ID id : source->second.normalContacts )
        {
            const auto& contact = items.at( id );
            if( contact.normal.kind == NORMAL_CONTACT_ITEM::KIND::AREA )
            {
                if( !ignoreAreas && contact.area
                    && CONTACT_GEOMETRY::ContainsArea( *contact.area, point ) )
                {
                    result.insert( id );
                }
            }
            else if( point == contact.normal.first
                     || ( contact.normal.kind == NORMAL_CONTACT_ITEM::KIND::TRACE
                          && point == contact.normal.last ) )
            {
                // Trace.normalContactPoint() intentionally returns null when
                // two traces share both endpoints.  Trace.getNormalContacts(
                // point), used by combine/isCycle, intentionally includes the
                // same overlap.  Keep those source APIs distinct.
                result.insert( id );
            }
        }
        return result;
    }

    std::optional<PLANAR::POINT> normalContactPoint( ITEM_ID first, ITEM_ID second ) const
    {
        const auto left = items.find( first ), right = items.find( second );
        if( left == items.end() || right == items.end() || first == second )
            return {};
        const auto exact = left->second.exactContactPoints.find( second );
        if( exact != left->second.exactContactPoints.end() )
            return exact->second;
        return left->second.normal.ExactPoint( right->second.normal );
    }

    int firstCommonLayer( ITEM_ID first, ITEM_ID second ) const
    {
        const auto left = items.find( first ), right = items.find( second );
        if( left == items.end() || right == items.end() || first == second )
            return -1;

        int result = std::numeric_limits<int>::max();
        for( int layer : left->second.normal.layers )
        {
            if( std::find( right->second.normal.layers.begin(),
                           right->second.normal.layers.end(), layer )
                != right->second.normal.layers.end() )
            {
                result = std::min( result, layer );
            }
        }
        return result == std::numeric_limits<int>::max() ? -1 : result;
    }

    bool isFanoutVia( ITEM_ID id, const ITEM_ID_SET* ignoreItems ) const
    {
        const auto via = items.find( id );
        if( via == items.end()
            || via->second.normal.kind != NORMAL_CONTACT_ITEM::KIND::DRILL )
        {
            return false;
        }

        const auto isIsolatedSmdPin = [&]( ITEM_ID contactId )
        {
            const auto contact = items.find( contactId );
            if( contact == items.end() || contact->second.pad == NO_PAD
                || contact->second.pad >= snapshot.pads.size() )
            {
                return false;
            }

            const ROUTING_PAD& pad = snapshot.pads[contact->second.pad];
            // Pin.isFanoutVia() tests the actual one-layer padstack, not its
            // package attribute.  A single-copper-layer connector pad has the
            // same source semantics even when KiCad does not classify it SMD.
            return pad.layers.size() == 1
                   && contact->second.normalContacts.size() <= 1;
        };

        for( ITEM_ID contactId : via->second.normalContacts )
        {
            if( isIsolatedSmdPin( contactId ) )
                return true;

            const auto contact = items.find( contactId );
            if( contact == items.end()
                || contact->second.normal.kind != NORMAL_CONTACT_ITEM::KIND::TRACE
                || ( ignoreItems && ignoreItems->contains( contactId ) ) )
            {
                continue;
            }

            long double length = 0;
            for( std::size_t corner = 1;
                 corner < contact->second.traceCorners.size(); ++corner )
            {
                const long double dx = static_cast<long double>(
                        contact->second.traceCorners[corner].x )
                                       - contact->second.traceCorners[corner - 1].x;
                const long double dy = static_cast<long double>(
                        contact->second.traceCorners[corner].y )
                                       - contact->second.traceCorners[corner - 1].y;
                length += std::hypotl( dx, dy );
            }
            const std::int64_t halfWidth = contact->second.trace
                    ? std::max<std::int64_t>( 0, contact->second.trace->radius ) : 0;
            if( length >= 400.0L * halfWidth )
                continue;

            for( ITEM_ID traceContactId : contact->second.normalContacts )
            {
                if( isIsolatedSmdPin( traceContactId ) )
                    return true;

                const auto traceContact = items.find( traceContactId );
                if( traceContact != items.end()
                    && traceContact->second.normal.kind
                               == NORMAL_CONTACT_ITEM::KIND::TRACE
                    && traceContact->second.fixedState
                               == ROUTER_FIXED_STATE::SHOVE_FIXED
                    && traceContact->second.traceCorners.size() == 2 )
                {
                    return true;
                }
            }
        }
        return via->second.route && via->second.route->isFanoutConnection;
    }

    ITEM_ID_SET connectionItems( ITEM_ID id,
                                 STOP_CONNECTION_OPTION stopOption
                                         = STOP_CONNECTION_OPTION::NONE ) const
    {
        const auto source = items.find( id );
        if( source == items.end() )
            return {};

        ITEM_ID_SET result;
        if( source->second.routable )
            result.insert( id );
        for( ITEM_ID currentId : source->second.normalContacts )
        {
            std::optional<PLANAR::POINT> previousPoint = normalContactPoint( id, currentId );
            if( !previousPoint )
                continue;

            int previousLayer = firstCommonLayer( id, currentId );
            if( source->second.normal.kind == NORMAL_CONTACT_ITEM::KIND::TRACE
                && ( !previousPoint->Integral()
                     || normalContactsAt( id, *previousPoint->Integral() ).size() != 1 ) )
            {
                continue;
            }

            ITEM_ID_SET visited{ id };
            for( ;; )
            {
                const auto current = items.find( currentId );
                if( current == items.end() || !current->second.routable )
                    break;

                if( current->second.normal.kind == NORMAL_CONTACT_ITEM::KIND::DRILL )
                {
                    if( stopOption == STOP_CONNECTION_OPTION::VIA )
                        break;
                    if( stopOption == STOP_CONNECTION_OPTION::FANOUT_VIA
                        && isFanoutVia( currentId, &result ) )
                    {
                        break;
                    }
                }

                result.insert( currentId );
                visited.insert( currentId );
                std::optional<ITEM_ID> next;
                std::optional<PLANAR::POINT> nextPoint;
                int nextLayer = -1;
                bool forkFound = false;

                for( ITEM_ID contact : current->second.normalContacts )
                {
                    const int contactLayer = firstCommonLayer( currentId, contact );
                    if( contactLayer < 0 )
                        continue;

                    const auto contactPoint = normalContactPoint( currentId, contact );
                    if( !contactPoint )
                    {
                        forkFound = true;
                        break;
                    }

                    if( contactLayer != previousLayer || !( *contactPoint == *previousPoint ) )
                    {
                        if( next )
                        {
                            forkFound = true;
                            break;
                        }
                        next = contact;
                        nextPoint = contactPoint;
                        nextLayer = contactLayer;
                    }
                }

                if( !next || forkFound || visited.contains( *next ) )
                    break;

                currentId = *next;
                previousPoint = nextPoint;
                previousLayer = nextLayer;
            }
        }
        return result;
    }

    std::optional<ITEM_ID> traceTailAt( ROUTER_POINT location, int layer, int net ) const
    {
        for( const auto& [id, item] : items )
        {
            if( item.net != net || item.normal.kind != NORMAL_CONTACT_ITEM::KIND::TRACE
                || std::find( item.normal.layers.begin(), item.normal.layers.end(), layer )
                           == item.normal.layers.end() )
            {
                continue;
            }
            if( ( item.normal.first == location || item.normal.last == location )
                && normalContactsAt( id, location ).empty() )
            {
                return id;
            }
        }
        return {};
    }

    void removeItemFromRoutes( ITEM_ID id )
    {
        for( auto& route : routes )
            std::erase( route.items, id );
    }

    void rebuildTraceGeometry( ITEM& item, std::vector<ROUTER_POINT> corners )
    {
        if( !item.trace || corners.size() < 2 )
            throw std::invalid_argument( "Cannot rebuild invalid routing trace" );

        corners.erase( std::unique( corners.begin(), corners.end() ), corners.end() );
        item.traceCorners = std::move( corners );
        item.normal.first = item.traceCorners.front();
        item.normal.last = item.traceCorners.back();
        item.contacts.clear();
        item.normalContacts.clear();
        item.exactContactPoints.clear();
        item.shapes.clear();
        item.terminals.clear();

        ROUTING_EDGE_STYLE style;
        int layer = item.normal.layers.empty() ? -1 : item.normal.layers.front();
        if( item.route )
        {
            layer = item.route->nodes.empty() ? layer : item.route->nodes.front().layer;
            style = item.route->edgeStyles.empty()
                    ? ROUTING_EDGE_STYLE{} : item.route->edgeStyles.front();
            item.route->nodes.clear();
            item.route->edgeStyles.clear();
            for( ROUTER_POINT corner : item.traceCorners )
                item.route->nodes.push_back( { corner, layer } );
            if( item.route->nodes.size() > 1 )
                item.route->edgeStyles.assign( item.route->nodes.size() - 1, style );
        }

        for( std::size_t segment = 1; segment < item.traceCorners.size(); ++segment )
        {
            ROUTING_OBSTACLE trace = *item.trace;
            trace.start = item.traceCorners[segment - 1];
            trace.end = item.traceCorners[segment];
            if( segment == 1 )
                item.trace = trace;
            addShape( item, trace );

            ROUTING_PAD terminal;
            terminal.netCode = item.net;
            terminal.position = trace.start;
            terminal.layers = trace.layers;
            terminal.trackWidth = 2 * trace.radius;
            item.terminals.push_back( { terminal, NO_PAD, trace.end, {} } );
        }
    }

    std::vector<ITEM_ID> splitTrace( ITEM_ID id,
                                     const std::vector<ROUTER_POINT>& points )
    {
        using namespace CONTACT_GEOMETRY;
        const auto found = items.find( id );
        if( found == items.end() || !found->second.trace
            || found->second.traceCorners.size() < 2
            || deletionForbidden( found->second ) )
        {
            return {};
        }

        const ITEM original = found->second;
        std::vector<ROUTER_POINT> expanded;
        expanded.push_back( original.traceCorners.front() );
        for( std::size_t segment = 1; segment < original.traceCorners.size(); ++segment )
        {
            const ROUTER_POINT start = original.traceCorners[segment - 1];
            const ROUTER_POINT end = original.traceCorners[segment];
            std::vector<ROUTER_POINT> segmentCuts;
            for( ROUTER_POINT splitPoint : points )
            {
                if( splitPoint != start && splitPoint != end
                    && OnSegment( start, end, splitPoint ) )
                {
                    segmentCuts.push_back( splitPoint );
                }
            }

            const bool sortByX = std::abs( end.x - start.x )
                                 >= std::abs( end.y - start.y );
            std::sort( segmentCuts.begin(), segmentCuts.end(), [&]( auto left, auto right )
            {
                if( sortByX )
                    return start.x < end.x ? left.x < right.x : left.x > right.x;
                return start.y < end.y ? left.y < right.y : left.y > right.y;
            } );
            segmentCuts.erase( std::unique( segmentCuts.begin(), segmentCuts.end() ),
                               segmentCuts.end() );
            expanded.insert( expanded.end(), segmentCuts.begin(), segmentCuts.end() );
            expanded.push_back( end );
        }

        std::vector<ROUTER_POINT> actualCuts;
        for( std::size_t i = 1; i + 1 < expanded.size(); ++i )
            if( std::find( points.begin(), points.end(), expanded[i] ) != points.end() )
                actualCuts.push_back( expanded[i] );
        std::sort( actualCuts.begin(), actualCuts.end(), []( auto left, auto right )
                   { return left.x != right.x ? left.x < right.x : left.y < right.y; } );
        actualCuts.erase( std::unique( actualCuts.begin(), actualCuts.end() ),
                          actualCuts.end() );
        if( actualCuts.empty() )
            return { id };

        removeItem( id );
        std::vector<ITEM_ID> replacements;
        std::size_t pieceStart = 0;
        for( std::size_t i = 1; i < expanded.size(); ++i )
        {
            const bool splitHere = i + 1 == expanded.size()
                                   || std::find( actualCuts.begin(), actualCuts.end(), expanded[i] )
                                              != actualCuts.end();
            if( !splitHere )
                continue;

            ITEM piece = original;
            piece.id = replacements.empty() ? id : nextId++;
            rebuildTraceGeometry(
                    piece,
                    std::vector<ROUTER_POINT>(
                            expanded.begin() + static_cast<std::ptrdiff_t>( pieceStart ),
                            expanded.begin() + static_cast<std::ptrdiff_t>( i + 1 ) ) );
            auto [it, inserted] = items.emplace( piece.id, std::move( piece ) );
            if( !inserted )
                throw std::logic_error( "Duplicate routing trace identity after split" );
            indexItem( it->second );
            replacements.push_back( it->first );
            pieceStart = i;
        }

        for( auto& route : routes )
        {
            for( auto it = route.items.begin(); it != route.items.end(); )
            {
                if( *it != id )
                {
                    ++it;
                    continue;
                }
                const auto offset = it - route.items.begin();
                it = route.items.erase( it );
                route.items.insert( route.items.begin() + offset,
                                    replacements.begin(), replacements.end() );
                it = route.items.begin() + offset
                     + static_cast<std::ptrdiff_t>( replacements.size() );
            }
        }
        return replacements;
    }

    bool traceIsCycle( ITEM_ID id ) const
    {
        const auto trace = items.find( id );
        if( trace == items.end() || trace->second.normal.kind != NORMAL_CONTACT_ITEM::KIND::TRACE )
            return false;

        const ITEM_ID_SET starts = normalContactsAt( id, trace->second.normal.first );
        const ITEM_ID_SET ends = normalContactsAt( id, trace->second.normal.last );
        if( std::any_of( starts.begin(), starts.end(),
                         [&]( ITEM_ID contact ) { return ends.contains( contact ); } ) )
        {
            return true;
        }

        ITEM_ID_SET visited = starts;
        std::vector<ITEM_ID> pending( starts.begin(), starts.end() );
        while( !pending.empty() )
        {
            const ITEM_ID current = pending.back();
            pending.pop_back();
            for( ITEM_ID contact : items.at( current ).normalContacts )
            {
                if( contact == id )
                    continue;
                if( ends.contains( contact ) )
                    return true;
                if( visited.insert( contact ).second )
                    pending.push_back( contact );
            }
        }
        return false;
    }

    bool removeIfCycle( ITEM_ID id )
    {
        const auto trace = items.find( id );
        if( trace == items.end() || !trace->second.dynamic || !trace->second.routable
            || !trace->second.route || !traceIsCycle( id ) )
        {
            return false;
        }

        const ROUTER_POINT endpoints[] = { trace->second.normal.first,
                                           trace->second.normal.last };
        const int layer = trace->second.normal.layers.empty()
                ? -1 : trace->second.normal.layers.front();
        const int net = trace->second.net;
        const bool tailBefore[] = { traceTailAt( endpoints[0], layer, net ).has_value(),
                                    traceTailAt( endpoints[1], layer, net ).has_value() };
        const ITEM_ID_SET cycleConnection = connectionItems( id );
        for( ITEM_ID itemId : cycleConnection )
        {
            removeItem( itemId );
            removeItemFromRoutes( itemId );
        }

        // BasicBoard.removeIfCycle removes a tail manufactured at either
        // endpoint only when there was no tail there before the cycle was
        // deleted.  This prevents normalization from leaving disconnected
        // fragments of the removed connection.
        for( int endpoint = 0; endpoint < 2; ++endpoint )
        {
            if( tailBefore[endpoint] )
                continue;
            const auto tail = traceTailAt( endpoints[endpoint], layer, net );
            if( !tail )
                continue;
            const ITEM_ID_SET tailConnection = connectionItems( *tail );
            const bool removable = std::all_of(
                    tailConnection.begin(), tailConnection.end(), [&]( ITEM_ID itemId )
                    {
                        const auto found = items.find( itemId );
                        return found != items.end() && found->second.dynamic
                               && found->second.routable && found->second.route;
                    } );
            if( !removable )
                continue;
            for( ITEM_ID itemId : tailConnection )
            {
                removeItem( itemId );
                removeItemFromRoutes( itemId );
            }
        }
        return true;
    }

    bool traceStylesEqual( const ITEM& left, const ITEM& right ) const
    {
        if( !left.trace || !right.trace || left.net != right.net
            || left.normal.layers != right.normal.layers
            || left.trace->radius != right.trace->radius
            || left.trace->clearance != right.trace->clearance
            || left.fixedState != right.fixedState
            || deletionForbidden( left ) || deletionForbidden( right )
            || !left.routable || !right.routable || !left.route || !right.route )
        {
            return false;
        }

        const ROUTING_EDGE_STYLE leftStyle = left.route->edgeStyles.empty()
                ? ROUTING_EDGE_STYLE{} : left.route->edgeStyles.front();
        const ROUTING_EDGE_STYLE rightStyle = right.route->edgeStyles.empty()
                ? ROUTING_EDGE_STYLE{} : right.route->edgeStyles.front();
        return leftStyle.trackWidth == rightStyle.trackWidth
               && leftStyle.clearance == rightStyle.clearance
               && leftStyle.fixedState == rightStyle.fixedState;
    }

    bool combineAt( ITEM_ID id, bool atStart )
    {
        const auto current = items.find( id );
        if( current == items.end() || current->second.normal.kind
                                     != NORMAL_CONTACT_ITEM::KIND::TRACE )
        {
            return false;
        }

        const ROUTER_POINT join = atStart ? current->second.normal.first
                                          : current->second.normal.last;
        const ITEM_ID_SET contacts = normalContactsAt( id, join, true );
        if( contacts.size() != 1 )
            return false;

        const ITEM_ID otherId = *contacts.begin();
        const auto other = items.find( otherId );
        if( other == items.end()
            || other->second.normal.kind != NORMAL_CONTACT_ITEM::KIND::TRACE
            || !traceStylesEqual( current->second, other->second ) )
        {
            return false;
        }

        std::vector<ROUTER_POINT> thisCorners = current->second.traceCorners;
        std::vector<ROUTER_POINT> otherCorners = other->second.traceCorners;
        if( atStart )
        {
            if( otherCorners.back() != join )
                std::reverse( otherCorners.begin(), otherCorners.end() );
            if( otherCorners.back() != join )
                return false;
            otherCorners.pop_back();
            otherCorners.insert( otherCorners.end(), thisCorners.begin(), thisCorners.end() );
            thisCorners = std::move( otherCorners );
        }
        else
        {
            if( otherCorners.front() != join )
                std::reverse( otherCorners.begin(), otherCorners.end() );
            if( otherCorners.front() != join )
                return false;
            thisCorners.insert( thisCorners.end(), std::next( otherCorners.begin() ),
                                otherCorners.end() );
        }

        ITEM combined = current->second;
        const ITEM absorbed = other->second;
        combined.route->cost += absorbed.route->cost;
        combined.route->isPlaneConnection = combined.route->isPlaneConnection
                                            || absorbed.route->isPlaneConnection;
        combined.route->isFanoutConnection = combined.route->isFanoutConnection
                                             || absorbed.route->isFanoutConnection;
        for( const std::string& sourceId : absorbed.route->sourceBoardItemIds )
        {
            if( std::find( combined.route->sourceBoardItemIds.begin(),
                           combined.route->sourceBoardItemIds.end(), sourceId )
                == combined.route->sourceBoardItemIds.end() )
            {
                combined.route->sourceBoardItemIds.push_back( sourceId );
            }
        }

        removeItem( id );
        removeItem( otherId );
        rebuildTraceGeometry( combined, std::move( thisCorners ) );
        auto [inserted, unique] = items.emplace( id, std::move( combined ) );
        if( !unique )
            throw std::logic_error( "Duplicate routing trace identity after combine" );
        indexItem( inserted->second );

        for( auto& route : routes )
        {
            for( ITEM_ID& itemId : route.items )
                if( itemId == otherId )
                    itemId = id;
            std::set<ITEM_ID> seen;
            std::erase_if( route.items, [&]( ITEM_ID itemId )
                           { return !seen.insert( itemId ).second; } );
        }
        return true;
    }

    bool combine( ITEM_ID id )
    {
        bool changed = false;
        while( items.contains( id ) && ( combineAt( id, true ) || combineAt( id, false ) ) )
            changed = true;
        return changed;
    }

    // Split polyline copper in the WORKER graph at exact centre-line contacts.
    // Retained copper is split virtually only: the original host item is not
    // edited/deleted, and these pieces never become result-emitter output.
    void normalizeJunctions( const std::vector<ITEM_ID>& added )
    {
        using namespace CONTACT_GEOMETRY;
        struct EXACT_JUNCTION
        {
            int net = 0;
            int layer = -1;
            PLANAR::POINT point;
        };
        std::map<ITEM_ID, std::vector<ROUTER_POINT>> cuts;
        std::vector<EXACT_JUNCTION> exactJunctions;
        auto cut = [&]( const ITEM& item, ROUTER_POINT p )
        {
            if( !item.trace || p == item.normal.first || p == item.normal.last )
                return;

            for( std::size_t segment = 1; segment < item.traceCorners.size(); ++segment )
            {
                if( OnSegment( item.traceCorners[segment - 1], item.traceCorners[segment], p ) )
                {
                    cuts[item.id].push_back( p );
                    break;
                }
            }
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
                    for( std::size_t itemSegment = 1;
                         itemSegment < item.traceCorners.size(); ++itemSegment )
                    {
                        for( std::size_t otherSegment = 1;
                             otherSegment < other.traceCorners.size(); ++otherSegment )
                        {
                            if( auto p = ExactIntersection(
                                        item.traceCorners[itemSegment - 1],
                                        item.traceCorners[itemSegment],
                                        other.traceCorners[otherSegment - 1],
                                        other.traceCorners[otherSegment] ) )
                            {
                                if( const auto integral = p->Integral() )
                                {
                                    cut( item, *integral );
                                    cut( other, *integral );
                                }
                                else
                                {
                                    for( int layer : item.normal.layers )
                                    {
                                        if( std::find( other.normal.layers.begin(),
                                                       other.normal.layers.end(), layer )
                                            != other.normal.layers.end() )
                                        {
                                            exactJunctions.push_back(
                                                    { item.net, layer, *p } );
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                else if( item.normal.kind == KIND::DRILL && other.normal.kind == KIND::TRACE )
                    cut( other, a );
                else if( item.normal.kind == KIND::TRACE && other.normal.kind == KIND::DRILL )
                    cut( item, c );
            }
        }
        std::set<ITEM_ID> addedPieces( added.begin(), added.end() );
        std::vector<ITEM_ID> normalizationOrder;
        for( auto& [id, points] : cuts )
        {
            std::sort( points.begin(), points.end(), []( auto left, auto right )
            { return left.x != right.x ? left.x < right.x : left.y < right.y; } );
            points.erase( std::unique( points.begin(), points.end() ), points.end() );
            const bool own = addedPieces.contains( id );
            const std::vector<ITEM_ID> replacements = splitTrace( id, points );
            normalizationOrder.insert( normalizationOrder.end(), replacements.begin(),
                                       replacements.end() );
            if( own )
                addedPieces.insert( replacements.begin(), replacements.end() );
        }
        for( ITEM_ID id : added )
            if( items.contains( id ) )
                normalizationOrder.push_back( id );

        // PolylineTrace.split() removes cycles in the found pieces first and
        // in the trace being normalized last.  Preserve that preference so a
        // newly inserted trace wins deterministic coincident-overlap ties.
        std::stable_sort( normalizationOrder.begin(), normalizationOrder.end(),
                          [&]( ITEM_ID left, ITEM_ID right )
                          { return addedPieces.contains( left )
                                   < addedPieces.contains( right ); } );
        std::set<ITEM_ID> scheduled;
        std::erase_if( normalizationOrder, [&]( ITEM_ID id )
                       { return !scheduled.insert( id ).second; } );
        for( ITEM_ID id : normalizationOrder )
            removeIfCycle( id );

        // Source PolylineTraceNormalization combines each surviving split
        // piece and recursively retries its start before its end.  The item
        // identity of the selected piece survives every successful combine.
        for( ITEM_ID id : normalizationOrder )
            if( items.contains( id ) )
                combine( id );

        // KiCad's board API has integral trace vertices. Preserve a source
        // RationalPoint crossing as an exact worker-graph contact instead of
        // rounding it or dropping electrical connectivity. The corresponding
        // traces remain unsplit host-emission items; connection-chain walks
        // stop at this virtual interior fork.
        for( const EXACT_JUNCTION& junction : exactJunctions )
        {
            std::vector<ITEM_ID> touching;
            for( const auto& [id, candidate] : items )
            {
                if( candidate.net != junction.net
                    || candidate.normal.kind != NORMAL_CONTACT_ITEM::KIND::TRACE
                    || std::find( candidate.normal.layers.begin(),
                                  candidate.normal.layers.end(), junction.layer )
                               == candidate.normal.layers.end()
                    || candidate.traceCorners.size() < 2 )
                {
                    continue;
                }

                const PLANAR::POLYLINE polyline =
                        PLANAR::POLYLINE::FromPoints( candidate.traceCorners );
                if( polyline.Contains( junction.point ) )
                    touching.push_back( id );
            }

            for( std::size_t left = 0; left < touching.size(); ++left )
            {
                for( std::size_t right = left + 1; right < touching.size(); ++right )
                {
                    ITEM& first = items.at( touching[left] );
                    ITEM& second = items.at( touching[right] );
                    first.normalContacts.insert( second.id );
                    second.normalContacts.insert( first.id );
                    first.exactContactPoints.insert_or_assign( second.id, junction.point );
                    second.exactContactPoints.insert_or_assign( first.id, junction.point );
                }
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
        item.terminals.push_back( { pad, i, {}, {} } );
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
            {
                item.trace = copper;
                item.traceCorners = { copper.start, copper.end };
                item.fixedState = copper.fixedState;
                item.routable = item.fixedState < ROUTER_FIXED_STATE::USER_FIXED;
                ROUTING_CONNECTION sourceRoute;
                sourceRoute.netCode = copper.netCode;
                sourceRoute.nodes = { { copper.start, item.normal.layers.front() },
                                      { copper.end, item.normal.layers.front() } };
                sourceRoute.complete = true;
                sourceRoute.isExistingBoardRoute = true;
                sourceRoute.isAutorouterOwned = copper.isAutorouterOwned;
                sourceRoute.isShoveMovable = copper.isMovable;
                if( !copper.boardItemId.empty() )
                    sourceRoute.sourceBoardItemIds = { copper.boardItemId };
                ROUTING_EDGE_STYLE sourceStyle;
                sourceStyle.trackWidth = 2 * copper.radius;
                sourceStyle.clearance = std::max<std::int64_t>( 0, copper.clearance );
                sourceStyle.fixedState = item.fixedState;
                sourceRoute.edgeStyles = { sourceStyle };
                item.route = std::move( sourceRoute );
            }
            else
            {
                item.trace.reset();
                item.route.reset();
                item.fixedState = copper.fixedState;
                item.routable = item.fixedState < ROUTER_FIXED_STATE::USER_FIXED;
            }
            for( int layer : copper.layers )
            {
                ROUTING_PAD pad;
                pad.netCode = copper.netCode;
                pad.position = copper.start;
                pad.layers = { layer };
                pad.trackWidth = 2 * copper.radius;
                item.terminals.push_back( { pad, NO_PAD, copper.end, {} } );
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

        // ConductionArea.getTraceConnectionShape() is the complete area tree
        // shape, unlike Pin/DrillItem which is intentionally a centre point.
        // Reuse one known on-copper synthetic anchor only to seed room
        // completion and to carry a host pad index; target attachment itself
        // uses connectionArea below, never this sample coordinate.
        const auto anchor = std::find_if(
                snapshot.pads.begin(), snapshot.pads.end(), [&]( const ROUTING_PAD& pad )
                {
                    return pad.netCode == area.netCode && pad.isPlaneTarget
                           && std::any_of(
                                   pad.layers.begin(), pad.layers.end(), [&]( int layer )
                                   {
                                       return std::find( area.layers.begin(),
                                                         area.layers.end(), layer )
                                              != area.layers.end();
                                   } )
                           && CONTACT_GEOMETRY::ContainsArea( area, pad.position );
                } );
        if( anchor != snapshot.pads.end() )
        {
            ROUTING_PAD terminalPad = *anchor;
            item.terminals.push_back(
                    { std::move( terminalPad ), NO_PAD, {}, item.area } );
        }
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
        std::set<int> geometryLayers;
        for( const ROUTING_VIA_LAYER_GEOMETRY& layerGeometry : style.viaLayerGeometry )
        {
            if( layerGeometry.layer < 0 || layerGeometry.diameter < 0
                || !geometryLayers.insert( layerGeometry.layer ).second )
            {
                throw std::invalid_argument( "Invalid routing-board via layer geometry" );
            }
            if( layerGeometry.diameter > 0 )
                coordinate( layerGeometry.diameter );
            if( layerGeometry.clearance < -1 )
                throw std::invalid_argument( "Invalid routing-board via layer clearance" );
        }

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

        if( route.nodes[i - 1].layer != route.nodes[i].layer )
        {
            const std::vector<int> span =
                    VIA_RULE::LayersFor( state.settings, route.nodes[i - 1].layer, route.nodes[i].layer, &style );
            for( int layer : geometryLayers )
            {
                if( std::find( span.begin(), span.end(), layer ) == span.end() )
                {
                    throw std::invalid_argument( "Via layer geometry lies outside the routing-board span" );
                }
            }
        }
    }

    TRANSACTION transaction( *this );
    std::size_t edge = 0;
    while( edge + 1 < route.nodes.size() )
    {
        const auto& from = route.nodes[edge];
        const auto& to = route.nodes[edge + 1];
        if( from == to )
        {
            ++edge;
            continue;
        }

        const bool via = from.layer != to.layer;
        const ROUTING_EDGE_STYLE& style = EdgeStyle( route, edge );
        const std::int64_t edgeWidth = style.trackWidth > 0 ? style.trackWidth : width;
        const std::int64_t        edgeDiameter =
                MaximumViaDiameter( style.viaLayerGeometry, style.viaDiameter > 0 ? style.viaDiameter : diameter );
        const std::vector<int> edgeLayers = via
                ? VIA_RULE::LayersFor( state.settings, from.layer, to.layer, &style )
                : std::vector<int>{ from.layer };
        auto& item = state.newItem( route.netCode, true );
        ROUTING_EDGE_STYLE resolvedStyle = style;
        resolvedStyle.trackWidth = edgeWidth;
        resolvedStyle.clearance = std::max<std::int64_t>( 0, style.clearance );
        item.fixedState = resolvedStyle.fixedState;
        item.routable = item.fixedState < ROUTER_FIXED_STATE::USER_FIXED;
        if( via )
        {
            resolvedStyle.viaDiameter = style.viaDiameter > 0 ? style.viaDiameter : diameter;
            resolvedStyle.viaDrill = style.viaDrill > 0 ? style.viaDrill : drill;
            resolvedStyle.viaLayers = edgeLayers;
        }
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
        if( via )
        {
            item.normal.last = to.point;
            for( int layer : copper.layers )
            {
                ROUTING_OBSTACLE layerCopper = copper;
                layerCopper.radius =
                        std::max<std::int64_t>( 1, ViaStyleDiameterOnLayer( resolvedStyle, layer, edgeDiameter ) / 2 );
                layerCopper.clearance = ViaStyleClearanceOnLayer( resolvedStyle, layer );
                layerCopper.layers = { layer };
                state.addShape( item, std::move( layerCopper ) );

                ROUTING_PAD terminal;
                terminal.netCode = route.netCode;
                terminal.position = from.point;
                terminal.layers = { layer };
                terminal.trackWidth = edgeWidth;
                item.terminals.push_back( { terminal, NO_PAD, std::nullopt, {} } );
            }
            item.route = route;
            item.route->nodes = { from, to };
            item.route->edgeStyles = { resolvedStyle };
            ++edge;
        }
        else
        {
            // Insert a same-layer, same-style run as one PolylineTrace item,
            // exactly as InsertFoundConnectionAlgo does.  Corners are path
            // geometry, not separate source Item identities.
            std::size_t lastEdge = edge;
            while( lastEdge + 2 < route.nodes.size()
                   && route.nodes[lastEdge + 1].layer == route.nodes[lastEdge + 2].layer )
            {
                const ROUTING_EDGE_STYLE& nextStyle = EdgeStyle( route, lastEdge + 1 );
                if( nextStyle.trackWidth != style.trackWidth
                    || nextStyle.clearance != style.clearance
                    || nextStyle.fixedState != style.fixedState )
                {
                    break;
                }
                ++lastEdge;
            }

            item.normal.last = route.nodes[lastEdge + 1].point;
            item.trace = copper;
            item.traceCorners.push_back( from.point );
            for( std::size_t traceEdge = edge; traceEdge <= lastEdge; ++traceEdge )
            {
                const ROUTER_NODE& traceFrom = route.nodes[traceEdge];
                const ROUTER_NODE& traceTo = route.nodes[traceEdge + 1];
                if( traceFrom.point == traceTo.point )
                    continue;

                ROUTING_OBSTACLE segment = copper;
                segment.start = traceFrom.point;
                segment.end = traceTo.point;
                state.addShape( item, segment );
                item.traceCorners.push_back( traceTo.point );

                ROUTING_PAD terminal;
                terminal.netCode = route.netCode;
                terminal.position = traceFrom.point;
                terminal.layers = { traceFrom.layer };
                terminal.trackWidth = edgeWidth;
                item.terminals.push_back( { terminal, NO_PAD, traceTo.point, {} } );
            }
            item.route = route;
            item.route->nodes.assign( route.nodes.begin() + static_cast<std::ptrdiff_t>( edge ),
                                      route.nodes.begin()
                                              + static_cast<std::ptrdiff_t>( lastEdge + 2 ) );
            item.route->edgeStyles.assign( item.route->nodes.size() - 1, resolvedStyle );
            edge = lastEdge + 1;
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
    const auto target = std::find_if( state.routes.begin(), state.routes.end(),
                                      [&]( const auto& existing )
                                      { return SameRouteGeometry( existing.connection, route ); } );
    if( target == state.routes.end() )
        return;

    TRANSACTION transaction( *this );
    const std::size_t targetIndex = static_cast<std::size_t>( target - state.routes.begin() );
    std::set<std::size_t> affectedRoutes{ targetIndex };
    std::set<ITEM_ID> affectedItems( target->items.begin(), target->items.end() );

    // A source combine deletes one PolylineTrace and leaves the selected
    // item's identity.  The native host bridge still remembers the original
    // insertion requests, so more than one request may name that one item.
    // Removing any request rebuilds only this transitive alias cluster from
    // the surviving requests.  Unrelated item identities and tree entries do
    // not move.
    bool expanded = true;
    while( expanded )
    {
        expanded = false;
        for( std::size_t index = 0; index < state.routes.size(); ++index )
        {
            if( affectedRoutes.contains( index ) )
                continue;
            const auto& candidate = state.routes[index];
            if( std::none_of( candidate.items.begin(), candidate.items.end(),
                              [&]( ITEM_ID id ) { return affectedItems.contains( id ); } ) )
            {
                continue;
            }
            affectedRoutes.insert( index );
            affectedItems.insert( candidate.items.begin(), candidate.items.end() );
            expanded = true;
        }
    }

    std::vector<ROUTING_CONNECTION> survivors;
    for( std::size_t index : affectedRoutes )
        if( index != targetIndex )
            survivors.push_back( state.routes[index].connection );

    for( ITEM_ID id : affectedItems )
        state.removeItem( id );
    for( auto it = affectedRoutes.rbegin(); it != affectedRoutes.rend(); ++it )
        state.routes.erase( state.routes.begin() + static_cast<std::ptrdiff_t>( *it ) );

    for( const ROUTING_CONNECTION& survivor : survivors )
        AddRoute( survivor );
    ++state.revision;
    transaction.Commit();
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


void ROUTING_BOARD::RetireSyntheticPad( std::size_t pad )
{
    if( pad >= m_impl->snapshot.pads.size() )
        return;

    ROUTING_PAD& terminal = m_impl->snapshot.pads[pad];
    if( !terminal.isFanoutTarget )
        return;

    // Synthetic fanout pads never own a board item.  Clearing their net and
    // layer membership is therefore sufficient to make padRoots() empty,
    // while retaining the immutable index for diagnostics and for the
    // already-created fanout search engine.  This mirrors the source model,
    // where the inserted drill/trace items survive but no landing item does.
    terminal.netCode = 0;
    terminal.layers.clear();
    ++m_impl->revision;
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

std::vector<ROUTING_BOARD::TARGET_ITEM> ROUTING_BOARD::UnconnectedTargetItems(
        std::size_t pad, int net ) const
{
    const auto sourceRoots = m_impl->padRoots( pad );
    std::vector<TARGET_ITEM> result;
    if( sourceRoots.empty() || net <= 0 )
        return result;

    m_impl->updateComponents();

    // A maze target that happens to be a trace or via still has to identify
    // the real pad component it will join.  Use the lowest pad index as the
    // stable native representative; source Item identity remains TARGET_ITEM::id.
    std::map<ITEM_ID, std::size_t> representativePad;
    for( const auto& [padIndex, itemId] : m_impl->pads )
    {
        const ITEM_ID root = m_impl->components.at( itemId );
        auto [entry, inserted] = representativePad.emplace( root, padIndex );
        if( !inserted )
            entry->second = std::min( entry->second, padIndex );
    }

    // Item.compareTo() orders the reference TreeSet by descending insertion
    // id.  Preserve that order before fanout's stable distance sort.
    for( auto itemEntry = m_impl->items.rbegin(); itemEntry != m_impl->items.rend();
         ++itemEntry )
    {
        const auto& [id, item] = *itemEntry;
        if( item.net != net )
            continue;

        const ITEM_ID root = m_impl->components.at( id );
        if( sourceRoots.contains( root ) )
            continue;

        TARGET_ITEM target;
        target.id = id;
        std::optional<ROUTER_BOX> bounds;
        for( const auto& part : item.shapes )
        {
            const BOX2I box = part.geometry->BBox();
            const ROUTER_BOX partBounds{ box.GetLeft(), box.GetTop(),
                                         box.GetRight(), box.GetBottom() };
            bounds = bounds
                    ? std::optional<ROUTER_BOX>( ROUTER_BOX{
                              std::min( bounds->minX, partBounds.minX ),
                              std::min( bounds->minY, partBounds.minY ),
                              std::max( bounds->maxX, partBounds.maxX ),
                              std::max( bounds->maxY, partBounds.maxY ) } )
                    : std::optional<ROUTER_BOX>( partBounds );
        }

        target.terminals = item.terminals;
        const auto representative = representativePad.find( root );
        for( ROUTING_TERMINAL& terminal : target.terminals )
        {
            if( terminal.padIndex == NO_PAD && representative != representativePad.end() )
                terminal.padIndex = representative->second;
        }

        // ConductionArea is one connectable source Item. A synthetic target
        // contributes only its stable host index/seed; the exact area region
        // stored on the terminal is the actual target-door geometry.
        if( item.conductionArea && item.area )
        {
            for( std::size_t padIndex = 0; padIndex < m_impl->snapshot.pads.size(); ++padIndex )
            {
                const ROUTING_PAD& planeTarget = m_impl->snapshot.pads[padIndex];
                if( planeTarget.netCode == net && planeTarget.isPlaneTarget
                    && CONTACT_GEOMETRY::ContainsArea( *item.area,
                                                       planeTarget.position ) )
                {
                    for( ROUTING_TERMINAL& terminal : target.terminals )
                    {
                        terminal.pad = planeTarget;
                        terminal.padIndex = padIndex;
                    }
                    break;
                }
            }
        }

        if( target.terminals.empty() || !bounds )
            continue;

        target.bounds = *bounds;
        result.push_back( std::move( target ) );
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

ROUTING_BOARD::ITEM_ID_SET ROUTING_BOARD::GetNormalContacts( ITEM_ID id ) const
{
    const auto it = m_impl->items.find( id );
    return it == m_impl->items.end() ? ITEM_ID_SET{} : it->second.normalContacts;
}

std::optional<ROUTER_POINT> ROUTING_BOARD::NormalContactPoint( ITEM_ID first, ITEM_ID second ) const
{
    const auto exact = ExactNormalContactPoint( first, second );
    return exact ? exact->Integral() : std::nullopt;
}


std::optional<PLANAR::POINT> ROUTING_BOARD::ExactNormalContactPoint(
        ITEM_ID first, ITEM_ID second ) const
{
    return m_impl->normalContactPoint( first, second );
}


int ROUTING_BOARD::FirstCommonLayer( ITEM_ID first, ITEM_ID second ) const
{
    return m_impl->firstCommonLayer( first, second );
}


ROUTING_BOARD::ITEM_ID_SET ROUTING_BOARD::NormalContactsAt( ITEM_ID trace,
                                                             ROUTER_POINT point ) const
{
    return m_impl->normalContactsAt( trace, point );
}


ROUTING_BOARD::ITEM_ID_SET ROUTING_BOARD::NormalConnectedSet( ITEM_ID id ) const
{
    if( !m_impl->items.contains( id ) )
        return {};
    ITEM_ID_SET result{ id };
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


ROUTING_BOARD::ITEM_ID_SET ROUTING_BOARD::GetConnectionItems(
        ITEM_ID id, STOP_CONNECTION_OPTION stopOption ) const
{
    return m_impl->connectionItems( id, stopOption );
}


ROUTING_BOARD::ITEM_ID_SET ROUTING_BOARD::ConductionAreaContactsAt(
        int net, ROUTER_NODE point ) const
{
    ITEM_ID_SET result;
    for( const auto& [id, item] : m_impl->items )
    {
        if( item.net != net || !item.conductionArea || !item.area )
            continue;
        if( std::find( item.normal.layers.begin(), item.normal.layers.end(), point.layer )
            == item.normal.layers.end() )
        {
            continue;
        }
        if( CONTACT_GEOMETRY::ContainsArea( *item.area, point.point ) )
            result.insert( id );
    }
    return result;
}


std::optional<ROUTING_BOARD::ITEM_INFO> ROUTING_BOARD::GetItemInfo( ITEM_ID id ) const
{
    const auto it = m_impl->items.find( id );
    if( it == m_impl->items.end() )
        return std::nullopt;

    const IMPL::ITEM& item = it->second;
    ITEM_INFO result;
    result.id = item.id;
    result.netCode = item.net;
    result.padIndex = item.pad;
    result.routable = item.routable;
    result.fixedState = item.fixedState;
    result.first = item.normal.first;
    result.last = item.normal.last;
    result.layers = item.normal.layers;
    switch( item.normal.kind )
    {
    case NORMAL_CONTACT_ITEM::KIND::TRACE:
        result.kind = ITEM_KIND::TRACE;
        for( std::size_t i = 1; i < item.traceCorners.size(); ++i )
        {
            const long double dx = static_cast<long double>( item.traceCorners[i].x )
                                   - item.traceCorners[i - 1].x;
            const long double dy = static_cast<long double>( item.traceCorners[i].y )
                                   - item.traceCorners[i - 1].y;
            result.traceLength += std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
        }
        break;
    case NORMAL_CONTACT_ITEM::KIND::DRILL:
        result.kind = ITEM_KIND::DRILL;
        break;
    case NORMAL_CONTACT_ITEM::KIND::AREA:
        result.kind = ITEM_KIND::AREA;
        break;
    default:
        result.kind = ITEM_KIND::UNKNOWN;
        break;
    }
    return result;
}


std::optional<ROUTING_CONNECTION> ROUTING_BOARD::ItemRoute( ITEM_ID id ) const
{
    const auto item = m_impl->items.find( id );
    if( item == m_impl->items.end() || !item->second.dynamic || !item->second.routable
        || !item->second.route )
    {
        return std::nullopt;
    }

    return item->second.route;
}


bool ROUTING_BOARD::RemoveItems( const ITEM_ID_SET& ids )
{
    if( ids.empty() )
        return false;

    auto& state = *m_impl;
    for( ITEM_ID id : ids )
    {
        const auto item = state.items.find( id );
        if( item == state.items.end() || !item->second.dynamic || !item->second.routable
            || !item->second.route )
        {
            return false;
        }
    }

    for( ITEM_ID id : ids )
        state.removeItem( id );

    // The original ROUTING_CONNECTION was only an insertion request. Once a
    // subset of its items is removed it is no longer a faithful board record.
    // Normalize every survivor to the source representation: one mutable
    // PolylineTrace or DrillItem per entry, in insertion-ID order.
    state.routes.clear();
    for( const auto& [id, item] : state.items )
        if( item.dynamic && item.routable && item.route )
            state.routes.push_back( { *item.route, { id } } );
    ++state.revision;
    return true;
}


std::vector<ROUTING_CONNECTION> ROUTING_BOARD::ItemRoutes() const
{
    std::vector<ROUTING_CONNECTION> result;
    for( const auto& [id, item] : m_impl->items )
        if( item.dynamic && item.routable && item.route )
            result.push_back( *item.route );
    return result;
}


std::vector<ROUTING_BOARD::ITEM_ID> ROUTING_BOARD::RouteItems( const ROUTING_CONNECTION& route ) const
{
    for( const auto& record : m_impl->routes )
        if( SameRouteGeometry( record.connection, route ) )
            return record.items;
    return {};
}


std::size_t ROUTING_BOARD::RouteReferenceCount( ITEM_ID item ) const
{
    return static_cast<std::size_t>( std::count_if(
            m_impl->routes.begin(), m_impl->routes.end(), [&]( const auto& route )
            {
                return std::find( route.items.begin(), route.items.end(), item )
                       != route.items.end();
            } ) );
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
    std::vector<ROUTER_POINT> padPositions;
};

ROUTING_BOARD::TRANSACTION::TRANSACTION( ROUTING_BOARD& board ) : m_board( board ),
    m_before( std::make_unique<STATE>( STATE{ board.m_impl->items, board.m_impl->routes,
                                             board.m_impl->nextId, {}, {} } ) )
{
    m_before->padPositions.reserve( board.m_impl->snapshot.pads.size() );
    for( const ROUTING_PAD& pad : board.m_impl->snapshot.pads )
        m_before->padPositions.push_back( pad.position );

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
    for( std::size_t i = 0; i < m_before->padPositions.size(); ++i )
        std::swap( m_board.m_impl->snapshot.pads[i].position,
                   m_before->padPositions[i] );
    ++m_board.m_impl->revision;
    m_board.m_impl->components.clear();
    m_board.m_impl->componentRevision = std::numeric_limits<std::uint64_t>::max();
}

void ROUTING_BOARD::TRANSACTION::Commit() { m_before.reset(); }

} // namespace KICAD_AUTOROUTER
