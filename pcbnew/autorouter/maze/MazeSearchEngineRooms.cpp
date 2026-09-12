/*
 * This file is part of KiCad, licensed under GPL version 3 or later.
 * KiCad adapter for the rectangular free-room search slice. This boundary
 * deliberately does not pretend bounding boxes are exact octagonal/polygonal
 * Freerouting tree shapes. Rejected proposals use the legacy fallback.
 */
#include "MazeSearchEngine.h"
#include "../AutorouterDebug.h"
#include "MazeExpansionEngine.h"
#include "MazeRipupResolver.h"
#include "MazeSearchEngine45Degree.h"
#include "MazeSearchEngineAnyAngle.h"
#include "MazeTraceShover.h"
#include "../board/searchtree/ShapeSearchTree45Degree.h"
#include "../board/model/structure/BoardOutline.h"
#include "../board/model/items/Pin.h"
#include "../path/Connection.h"
#include "../path/FoundConnectionInserter.h"
#include "../geometry/planar/ContactGeometry.h"
#include "../geometry/planar/Simplex.h"
#include "../rules/ViaRule.h"
#include <geometry/shape_poly_set.h>
#include <algorithm>
#include <sstream>

namespace KICAD_AUTOROUTER
{

namespace
{

using PLANAR::INT_OCTAGON;

ROUTER_BOX terminalTreeBounds( const ROUTING_TERMINAL& aTerminal, int aLayer,
                               std::int64_t aClearanceCompensation )
{
    // PolylineTrace.calculateTreeShapes() enlarges each trace segment by its
    // half-width plus the trace clearance compensation.  Pin tree shapes are
    // supplied by the host adapter because their actual copper need not be a
    // circle or even axis aligned.
    if( aTerminal.segmentEnd )
    {
        const ROUTER_POINT end = *aTerminal.segmentEnd;
        const std::int64_t expansion =
                std::max<std::int64_t>( 0, aTerminal.pad.trackWidth / 2 )
                + aClearanceCompensation;
        return { std::min( aTerminal.pad.position.x, end.x ) - expansion,
                 std::min( aTerminal.pad.position.y, end.y ) - expansion,
                 std::max( aTerminal.pad.position.x, end.x ) + expansion,
                 std::max( aTerminal.pad.position.y, end.y ) + expansion };
    }

    if( aTerminal.connectionArea )
    {
        const ROUTING_OBSTACLE& area = *aTerminal.connectionArea;
        ROUTER_BOX bounds = area.box;
        if( area.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
        {
            bounds = { std::min( area.start.x, area.end.x ) - area.radius,
                       std::min( area.start.y, area.end.y ) - area.radius,
                       std::max( area.start.x, area.end.x ) + area.radius,
                       std::max( area.start.y, area.end.y ) + area.radius };
        }
        else if( area.kind == ROUTER_OBSTACLE_KIND::POLYGON
                 && !area.polygon.empty() )
        {
            bounds = { area.polygon.front().x, area.polygon.front().y,
                       area.polygon.front().x, area.polygon.front().y };
            for( const ROUTER_POINT& point : area.polygon )
            {
                bounds.minX = std::min( bounds.minX, point.x );
                bounds.minY = std::min( bounds.minY, point.y );
                bounds.maxX = std::max( bounds.maxX, point.x );
                bounds.maxY = std::max( bounds.maxY, point.y );
            }
        }
        bounds.minX -= aClearanceCompensation;
        bounds.minY -= aClearanceCompensation;
        bounds.maxX += aClearanceCompensation;
        bounds.maxY += aClearanceCompensation;
        return bounds;
    }

    const auto geometry = std::find_if(
            aTerminal.pad.layerGeometry.begin(), aTerminal.pad.layerGeometry.end(),
            [&]( const ROUTING_PAD::LAYER_GEOMETRY& aGeometry )
            {
                return aGeometry.layer == aLayer
                       && aGeometry.treeBounds.minX <= aGeometry.treeBounds.maxX
                       && aGeometry.treeBounds.minY <= aGeometry.treeBounds.maxY;
            } );
    if( geometry != aTerminal.pad.layerGeometry.end() )
        return geometry->treeBounds;

    const std::int64_t expansion =
            std::max<std::int64_t>( 0, aTerminal.pad.radius )
            + aClearanceCompensation;
    return { aTerminal.pad.position.x - expansion,
             aTerminal.pad.position.y - expansion,
             aTerminal.pad.position.x + expansion,
             aTerminal.pad.position.y + expansion };
}


std::vector<ROUTING_PAD::LAYER_GEOMETRY::TRACE_EXIT_RESTRICTION>
terminalTraceExitRestrictions( const ROUTING_TERMINAL& aTerminal, int aLayer )
{
    // A source TargetItemExpansionDoor carries the Pin only for a real pin
    // endpoint. Existing trace terminals and conduction areas must not inherit
    // the anchor pad's package restrictions.
    if( aTerminal.segmentEnd || aTerminal.connectionArea )
        return {};

    const auto geometry = std::find_if(
            aTerminal.pad.layerGeometry.begin(), aTerminal.pad.layerGeometry.end(),
            [&]( const ROUTING_PAD::LAYER_GEOMETRY& aGeometry )
            {
                return aGeometry.layer == aLayer;
            } );
    return geometry == aTerminal.pad.layerGeometry.end()
                   ? std::vector<ROUTING_PAD::LAYER_GEOMETRY::TRACE_EXIT_RESTRICTION>{}
                   : geometry->traceExitRestrictions;
}


std::int64_t pinEdgeToTurnDistance( const BOARD_SNAPSHOT& aBoard )
{
    // Structure.readScope() uses BoardRules.getMinTraceHalfWidth() when the
    // DSN has no explicit smd_to_turn_gap. KiCad's exporter does not emit that
    // rule, so derive the same minimum from the routed net widths captured on
    // the editor thread.
    std::int64_t result = std::numeric_limits<std::int64_t>::max();
    for( const ROUTING_PAD& pad : aBoard.pads )
        if( pad.netCode > 0 && pad.trackWidth > 0 )
            result = std::min( result, pad.trackWidth / 2 );

    return result == std::numeric_limits<std::int64_t>::max()
                   ? 0 : std::max<std::int64_t>( 0, result );
}

ROUTER_POINT sourceGridPoint( ROUTER_POINT aPoint )
{
    return FLOAT_POINT{ static_cast<double>( aPoint.x ),
                        static_cast<double>( aPoint.y ) }.RoundToSourceGridYDown();
}


std::int64_t sourceGridCoordinate( std::int64_t aCoordinate )
{
    return sourceGridPoint( { aCoordinate, 0 } ).x;
}


INT_OCTAGON octagonalEnvelope( const std::vector<ROUTER_POINT>& aPoints,
                               std::int64_t aExpansion )
{
    if( aPoints.empty() )
        return INT_OCTAGON::Empty();

    const ROUTER_POINT firstPoint = sourceGridPoint( aPoints.front() );
    std::int64_t left = firstPoint.x;
    std::int64_t right = left;
    std::int64_t bottom = firstPoint.y;
    std::int64_t top = bottom;
    std::int64_t upperLeft = firstPoint.x - firstPoint.y;
    std::int64_t lowerRight = upperLeft;
    std::int64_t lowerLeft = firstPoint.x + firstPoint.y;
    std::int64_t upperRight = lowerLeft;
    for( const ROUTER_POINT& inputPoint : aPoints )
    {
        const ROUTER_POINT point = sourceGridPoint( inputPoint );
        left = std::min( left, point.x );
        right = std::max( right, point.x );
        bottom = std::min( bottom, point.y );
        top = std::max( top, point.y );
        upperLeft = std::min( upperLeft, point.x - point.y );
        lowerRight = std::max( lowerRight, point.x - point.y );
        lowerLeft = std::min( lowerLeft, point.x + point.y );
        upperRight = std::max( upperRight, point.x + point.y );
    }
    return INT_OCTAGON( left, bottom, right, top, upperLeft, lowerRight,
                        lowerLeft, upperRight ).NormalizeOnGrid(
                                static_cast<std::int64_t>(
                                        FREEROUTING_COORDINATE_UNIT_IU ) ).OffsetOnGrid(
                                aExpansion,
                                static_cast<std::int64_t>(
                                        FREEROUTING_COORDINATE_UNIT_IU ) );
}


INT_OCTAGON octagonalEnvelope( const ROUTING_OBSTACLE& aObstacle,
                               std::int64_t aExpansion )
{
    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
    {
        const ROUTER_POINT minimum = FLOAT_POINT{
                static_cast<double>( aObstacle.box.minX ),
                static_cast<double>( aObstacle.box.minY ) }.RoundToSourceGridYDown();
        const ROUTER_POINT maximum = FLOAT_POINT{
                static_cast<double>( aObstacle.box.maxX ),
                static_cast<double>( aObstacle.box.maxY ) }.RoundToSourceGridYDown();
        return INT_OCTAGON::FromBox( { minimum.x, minimum.y,
                                      maximum.x, maximum.y } ).OffsetOnGrid(
                aExpansion,
                static_cast<std::int64_t>( FREEROUTING_COORDINATE_UNIT_IU ) );
    }
    if( aObstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT )
        return octagonalEnvelope( { aObstacle.start, aObstacle.end }, aExpansion );
    return octagonalEnvelope( aObstacle.polygon, aExpansion );
}


std::optional<INT_OCTAGON> sourceDrillItemTreeOctagon(
        const ROUTING_OBSTACLE& aCopper,
        std::int64_t aClearanceCompensation )
{
    const std::int64_t unit = static_cast<std::int64_t>(
            FREEROUTING_COORDINATE_UNIT_IU );
    const std::int64_t clearance = sourceGridCoordinate(
            std::max<std::int64_t>( 0, aClearanceCompensation ) );

    // ShapeSearchTree45Degree.calculateTreeShapes(DrillItem) first builds the
    // physical shape's bounding octagon.  An IntBox remains an IntBox to
    // avoid corner cut-offs; circles and paths retain their asymmetric source
    // floor/ceil supports before the clearance share is applied.
    if( aCopper.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
    {
        const ROUTER_BOX sourceBox{
            sourceGridCoordinate( aCopper.box.minX ),
            sourceGridCoordinate( aCopper.box.minY ),
            sourceGridCoordinate( aCopper.box.maxX ),
            sourceGridCoordinate( aCopper.box.maxY )
        };
        return SHAPE_SEARCH_TREE_45_DEGREE::OffsetDrillItemBox(
                sourceBox, clearance );
    }

    if( aCopper.kind == ROUTER_OBSTACLE_KIND::SEGMENT
        && aCopper.radius > 0 )
    {
        const std::int64_t radius = sourceGridCoordinate( aCopper.radius );
        if( aCopper.start == aCopper.end )
        {
            return SHAPE_SEARCH_TREE_45_DEGREE::OffsetDrillItemCircle(
                    sourceGridPoint( aCopper.start ), radius, clearance, unit );
        }

        return octagonalEnvelope( { aCopper.start, aCopper.end }, 0 )
                .OffsetOnGrid( radius, unit )
                .OffsetOnGrid( clearance, unit );
    }

    if( aCopper.kind == ROUTER_OBSTACLE_KIND::POLYGON
        && !aCopper.polygon.empty() )
    {
        return octagonalEnvelope( aCopper, clearance );
    }

    return std::nullopt;
}

bool isAxisAlignedRectangle( const std::vector<ROUTER_POINT>& aPolygon )
{
    if( aPolygon.size() != 4 )
        return false;

    std::vector<std::int64_t> x;
    std::vector<std::int64_t> y;
    x.reserve( aPolygon.size() );
    y.reserve( aPolygon.size() );

    for( const ROUTER_POINT& point : aPolygon )
    {
        x.push_back( point.x );
        y.push_back( point.y );
    }

    std::sort( x.begin(), x.end() );
    std::sort( y.begin(), y.end() );
    x.erase( std::unique( x.begin(), x.end() ), x.end() );
    y.erase( std::unique( y.begin(), y.end() ), y.end() );

    if( x.size() != 2 || y.size() != 2 )
        return false;

    return std::all_of( aPolygon.begin(), aPolygon.end(), [&]( const ROUTER_POINT& point )
    {
        return ( point.x == x.front() || point.x == x.back() )
               && ( point.y == y.front() || point.y == y.back() );
    } );
}


bool isGeneralConvexRoomObstacle( const ROUTING_OBSTACLE& aObstacle, int aNet,
                                  bool aForVia )
{
    if( aObstacle.kind != ROUTER_OBSTACLE_KIND::POLYGON || aObstacle.isHole
        || aObstacle.isPad
        || !( aForVia ? aObstacle.blocksVias : aObstacle.blocksTracks )
        || !aObstacle.polygonHoles.empty() || aObstacle.radius != 0
        || isAxisAlignedRectangle( aObstacle.polygon )
        || ( aObstacle.netCode == aNet && !aObstacle.isKeepout ) )
    {
        return false;
    }

    return PLANAR::SIMPLEX::FromConvexPolygon( aObstacle.polygon ).has_value();
}


bool isGeneralPolygonRoomObstacle( const ROUTING_OBSTACLE& aObstacle, int aNet,
                                   bool aForVia )
{
    return aObstacle.kind == ROUTER_OBSTACLE_KIND::POLYGON && !aObstacle.isHole
           && !aObstacle.isPad
           && ( aForVia ? aObstacle.blocksVias : aObstacle.blocksTracks )
           && aObstacle.radius == 0 && aObstacle.polygon.size() >= 3
           && ( aObstacle.netCode != aNet || aObstacle.isKeepout );
}


std::optional<std::vector<PLANAR::SIMPLEX>> splitPolygonAreaToConvex(
        const ROUTING_OBSTACLE& aObstacle, std::int64_t aExpansion )
{
    const auto fitsHostCoordinate = []( const ROUTER_POINT& aPoint )
    {
        return aPoint.x >= std::numeric_limits<int>::min()
               && aPoint.x <= std::numeric_limits<int>::max()
               && aPoint.y >= std::numeric_limits<int>::min()
               && aPoint.y <= std::numeric_limits<int>::max();
    };
    if( !std::all_of( aObstacle.polygon.begin(), aObstacle.polygon.end(),
                      fitsHostCoordinate ) )
    {
        return std::nullopt;
    }
    for( const auto& hole : aObstacle.polygonHoles )
        if( hole.size() < 3
            || !std::all_of( hole.begin(), hole.end(), fitsHostCoordinate ) )
        {
            return std::nullopt;
        }

    SHAPE_POLY_SET area;
    const int outline = area.NewOutline();
    for( const ROUTER_POINT& point : aObstacle.polygon )
        area.Append( static_cast<int>( point.x ), static_cast<int>( point.y ), outline, -1 );
    for( const auto& hole : aObstacle.polygonHoles )
    {
        const int holeIndex = area.NewHole( outline );
        for( const ROUTER_POINT& point : hole )
            area.Append( static_cast<int>( point.x ), static_cast<int>( point.y ),
                         outline, holeIndex );
    }

    area.CacheTriangulation( false );
    std::vector<PLANAR::SIMPLEX> result;
    for( unsigned int polygon = 0; polygon < area.TriangulatedPolyCount(); ++polygon )
    {
        const auto* triangulated = area.TriangulatedPolygon( polygon );
        if( !triangulated )
            return std::nullopt;
        for( std::size_t triangle = 0; triangle < triangulated->GetTriangleCount(); ++triangle )
        {
            VECTOR2I a, b, c;
            triangulated->GetTriangle( static_cast<int>( triangle ), a, b, c );
            const auto base = PLANAR::SIMPLEX::FromConvexPolygon(
                    { { a.x, a.y }, { b.x, b.y }, { c.x, c.y } } );
            const auto simplex = base ? base->Offset( aExpansion ) : std::nullopt;
            if( !simplex || simplex->Dimension() != 2 )
                return std::nullopt;
            result.push_back( std::move( *simplex ) );
        }
    }
    return result.empty() ? std::nullopt
                          : std::optional<std::vector<PLANAR::SIMPLEX>>(
                                    std::move( result ) );
}


bool areaTouchesVia( const ROUTING_OBSTACLE& aArea, ROUTER_POINT aCenter,
                     std::int64_t aViaRadius )
{
    if( aArea.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
    {
        ROUTER_BOX expanded = aArea.box;
        expanded.minX -= aViaRadius;
        expanded.minY -= aViaRadius;
        expanded.maxX += aViaRadius;
        expanded.maxY += aViaRadius;
        return expanded.Contains( aCenter );
    }

    if( aArea.kind != ROUTER_OBSTACLE_KIND::POLYGON )
        return false;

    if( CONTACT_GEOMETRY::ContainsArea( aArea, aCenter ) )
        return true;

    // The straight convex subset has an exact rational support-line test.
    // Non-convex areas and holes stay conservative: a centre outside their
    // copper is not invented as a normal contact merely from an AABB hit.
    if( aArea.polygonHoles.empty() )
        if( const auto expanded = PLANAR::SIMPLEX::FromConvexPolygon(
                    aArea.polygon, std::max<std::int64_t>( 0, aViaRadius ) ) )
            return expanded->Contains( PLANAR::POINT( aCenter ) );

    return false;
}


} // namespace


bool MAZE_SEARCH_ENGINE::hasGeneralConvexRoomGeometry( int aNet, int aLayer ) const
{
    for( const std::size_t index : obstacleIndices( aLayer ) )
    {
        const ROUTING_OBSTACLE& obstacle = m_board.obstacles[index];
        if( isGeneralPolygonRoomObstacle( obstacle, aNet, false ) )
            return true;
    }

    return false;
}


std::optional<INT_OCTAGON> MAZE_SEARCH_ENGINE::terminalTreeOctagon(
        const ROUTING_TERMINAL& aTerminal, int aLayer,
        std::int64_t aClearanceCompensation ) const
{
    if( aTerminal.segmentEnd )
    {
        // PolylineTrace has one tree entry for each segment.  The native
        // terminal already identifies that exact segment, so reconstruct its
        // compensated tree octagon rather than retaining a host AABB.
        const std::int64_t expansion =
                std::max<std::int64_t>( 0, aTerminal.pad.trackWidth / 2 )
                + std::max<std::int64_t>( 0, aClearanceCompensation );
        return octagonalEnvelope(
                { aTerminal.pad.position, *aTerminal.segmentEnd }, expansion );
    }

    // ConductionArea uses its exact finite polygon through connectionArea;
    // reducing it to one convex octagon would fill holes or disjoint regions.
    if( aTerminal.connectionArea )
        return std::nullopt;

    const auto geometry = std::find_if(
            aTerminal.pad.layerGeometry.begin(),
            aTerminal.pad.layerGeometry.end(),
            [&]( const ROUTING_PAD::LAYER_GEOMETRY& aGeometry )
            {
                return aGeometry.layer == aLayer;
            } );
    if( geometry == aTerminal.pad.layerGeometry.end()
        || geometry->copperShapeIndices.size() != 1 )
    {
        return std::nullopt;
    }

    const std::size_t shapeIndex = geometry->copperShapeIndices.front();
    if( shapeIndex >= m_board.obstacles.size() )
        return std::nullopt;

    const ROUTING_OBSTACLE& copper = m_board.obstacles[shapeIndex];
    if( !copper.isPad
        || ( !copper.layers.empty()
             && std::find( copper.layers.begin(), copper.layers.end(), aLayer )
                        == copper.layers.end() ) )
    {
        return std::nullopt;
    }

    return sourceDrillItemTreeOctagon(
            copper, std::max<std::int64_t>( 0, aClearanceCompensation ) );
}


std::vector<ROUTING_CONNECTION> MAZE_SEARCH_ENGINE::roomRouteItems() const
{
    std::vector<ROUTING_CONNECTION> result;

    // AddStatic records are deliberately absent from RoutingBoard's mutable
    // item graph.  They are original host PolylineTrace/DrillItem objects and
    // therefore precede newly inserted items in source insertion-id order.
    for( const ROUTING_CONNECTION& connection : m_occupancy.Connections() )
    {
        if( connection.isExistingBoardRoute && !connection.isAutorouterOwned )
            result.push_back( connection );
    }

    if( m_occupancy.Board() )
    {
        std::vector<ROUTING_CONNECTION> items =
                m_occupancy.Board()->ItemRoutes();
        result.insert( result.end(),
                       std::make_move_iterator( items.begin() ),
                       std::make_move_iterator( items.end() ) );
    }

    return result;
}


std::vector<SHAPE_TREE_ENTRY> MAZE_SEARCH_ENGINE::roomObstacles(
        int net, int aLayer, bool aForVia, bool aSkipGeneralConvex,
        bool aSourceTraceRooms,
        const ROUTER_CANCEL_CALLBACK& aCancel,
        const std::vector<ROOM_RIPUP_OBSTACLE>* aRipupObstacles,
        std::int64_t aCandidateRadius,
        std::int64_t aCandidateDrillRadius ) const
{
    const auto radius = aCandidateRadius >= 0
                                ? aCandidateRadius
                                : aForVia ? netViaRadius( net ) : netTrackRadius( net );
    const auto drillRadius = aCandidateDrillRadius >= 0
                                     ? aCandidateDrillRadius
                                     : netViaDrillRadius( net );
    const std::int64_t candidateCompensation = traceClearanceCompensation( net );
    const auto sourceTraceTreeExpansion = [&]( const ROUTING_OBSTACLE& aObstacle )
    {
        if( aObstacle.isHole )
        {
            return aObstacle.radius + std::max<std::int64_t>(
                    0, m_board.holeClearance - candidateCompensation );
        }

        const auto contextual = ContextualObstacleClearance(
                aObstacle, net, aLayer );
        const std::int64_t pair = aObstacle.netCode != 0 && aObstacle.netCode != net
                ? edgePairClearance( net, aObstacle.netCode, aLayer, 0,
                                     aObstacle.clearance )
                : std::max( netClearance( net ), aObstacle.clearance );
        const std::int64_t clearance = contextual
                ? *contextual
                : pair;
        const std::int64_t result = aObstacle.radius + std::max<std::int64_t>(
                0, clearance - candidateCompensation );
        autorouterDecisionLog(
                "SOURCE_TREE_OBSTACLE_EXPANSION",
                { { "layer", std::to_string( aLayer ) },
                  { "obstacle_net", std::to_string( aObstacle.netCode ) },
                  { "kind", std::to_string( static_cast<int>( aObstacle.kind ) ) },
                  { "is_pad", aObstacle.isPad ? "true" : "false" },
                  { "item", aObstacle.boardItemId },
                  { "radius", std::to_string( aObstacle.radius ) },
                  { "obstacle_clearance", std::to_string( aObstacle.clearance ) },
                  { "pair_clearance", std::to_string( pair ) },
                  { "contextual_clearance", contextual ? std::to_string( *contextual ) : "" },
                  { "candidate_compensation", std::to_string( candidateCompensation ) },
                  { "expansion", std::to_string( result ) } } );
        return result;
    };
    std::vector<SHAPE_TREE_ENTRY> entries;
    int id = 1;
    // ShapeSearchTree construction walks BasicBoard's undoable item store in
    // reverse insertion order.  A KiCad pad contributes one detached copper
    // obstacle per layer (and a separate manufacturing-hole record), so use
    // boardItemId to recover the single source Item identity before assigning
    // the observable tree insertion order.
    std::vector<std::uint64_t> sourceItemOrder(
            m_board.obstacles.size(),
            std::numeric_limits<std::uint64_t>::max() );
    std::unordered_map<std::string, std::uint64_t> sourceOrderByItem;
    std::uint64_t nextSourceOrder = 0;
    for( std::size_t reverse = m_board.obstacles.size(); reverse > 0; --reverse )
    {
        const std::size_t index = reverse - 1;
        const ROUTING_OBSTACLE& obstacle = m_board.obstacles[index];
        const std::string key = obstacle.boardItemId.empty()
                ? std::string( "#" ) + std::to_string( index )
                : obstacle.boardItemId;
        const auto [item, inserted] = sourceOrderByItem.emplace(
                key, nextSourceOrder );
        if( inserted )
            ++nextSourceOrder;
        sourceItemOrder[index] = item->second;
    }
    std::uint64_t currentTreeInsertionOrder =
            std::numeric_limits<std::uint64_t>::max();
    int currentTreeShapeIndex = 0;
    int currentTreeNet = 0;
    std::shared_ptr<const std::vector<ROUTING_TRACE_INSERTION_STEP>>
            currentTraceInsertionSteps;
    std::int64_t currentTraceTreeExpansion = -1;
    auto addOctagon = [&]( INT_OCTAGON shape,
                           std::optional<PLANAR::SIMPLEX> simplex = std::nullopt )
    {
        shape = shape.Normalize();
        if( shape.Dimension() >= 0 )
        {
            entries.push_back( { shape.BoundingBox(), id++, 0, aLayer,
                                 currentTreeNet,
                                 false, true, shape, std::move( simplex ) } );
            entries.back().treeInsertionOrder = currentTreeInsertionOrder;
            entries.back().shapeIndex = currentTreeShapeIndex++;
            entries.back().traceInsertionSteps = currentTraceInsertionSteps;
            entries.back().traceTreeExpansion = currentTraceTreeExpansion;
        }
    };
    auto add = [&]( ROUTER_BOX box, std::int64_t expansion )
    {
        addOctagon( INT_OCTAGON::FromBox( box ).Offset( expansion ) );
    };
    auto addSegment = [&]( ROUTER_POINT start, ROUTER_POINT end,
                           std::int64_t expansion )
    {
        std::optional<PLANAR::SIMPLEX> simplex;
        if( start != end )
            simplex = PLANAR::SIMPLEX::FromExpandedSegment(
                    start, end, expansion );
        else
        {
            simplex = PLANAR::SIMPLEX::Box(
                    { start.x - expansion, start.y - expansion,
                      start.x + expansion, start.y + expansion } );
        }
        addOctagon( octagonalEnvelope( { start, end }, expansion ),
                    std::move( simplex ) );
    };

    // ShapeSearchTree inserts BoardOutline after the reverse walk of ordinary
    // board items.  Equal-area insertion ties are observable in completed-room
    // and door ordering, so this is topology rather than cosmetic metadata.
    auto outlineEntries = BOARD_OUTLINE::CalculateTreeShapes(
            m_board, aLayer, candidateCompensation, id );
    for( SHAPE_TREE_ENTRY& entry : outlineEntries )
        entry.treeInsertionOrder = nextSourceOrder;
    entries.insert( entries.end(),
                    std::make_move_iterator( outlineEntries.begin() ),
                    std::make_move_iterator( outlineEntries.end() ) );

    for( auto index : obstacleIndices( aLayer ) )
    {
        if( aCancel && aCancel() )
            return {};
        const auto& obstacle = m_board.obstacles[index];
        currentTreeInsertionOrder = sourceItemOrder[index];
        currentTreeShapeIndex = 0;
        currentTreeNet = obstacle.isKeepout ? 0 : obstacle.netCode;
        currentTraceInsertionSteps.reset();
        currentTraceTreeExpansion = -1;
        if( !( aForVia ? obstacle.blocksVias : obstacle.blocksTracks ) )
            continue;

        // Freerouting's ShapeSearchTree stores a Pin or Via by its copper
        // shapes.  The Specctra model has no second trace-routing obstacle for
        // that item's plated drill.  The KiCad adapter retains drilled holes
        // for via-to-hole and final host DRC, but putting a net-assigned hole
        // into the trace room tree double-counts a through pad/via and changes
        // the very first completed room.  Unassigned mechanical holes remain
        // real trace obstacles.
        if( !aForVia && obstacle.isHole && obstacle.netCode != 0 )
            continue;

        if( obstacle.netCode == net && !obstacle.isKeepout
            && !( aSourceTraceRooms && !aForVia ) )
        {
            const bool ownHole = obstacle.isHole && obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT
                                 && endpointRadius( net, obstacle.start ) >= 0;
            if( !obstacle.isHole || ( !aForVia && ( ownHole || obstacle.isExistingRoute ) ) )
                continue;
        }
        // The drill-page tree remains rectangular.  The 45-degree room tree
        // retains this exact eight-support envelope instead of treating a
        // diagonal convex contour as its axis-aligned bounding box.
        if( aSkipGeneralConvex && isGeneralPolygonRoomObstacle( obstacle, net, aForVia ) )
            continue;
        // ShapeSearchTree stores obstacle copper plus the obstacle-side share
        // of clearance.  The 45-degree locator now applies the candidate's
        // compensated half width while walking the selected room corridor.
        // Keep complete-centre expansion for the paths whose source locator
        // has not yet replaced their established native locator.
        const bool sourceTraceShape = aSourceTraceRooms && !aForVia;
        const std::int64_t sourceExpansion = sourceTraceShape
                ? sourceTraceTreeExpansion( obstacle ) : 0;
        const std::int64_t expansion = ( sourceTraceShape
                ? sourceExpansion
                : obstacleExpansionRadius(
                        obstacle, net, aLayer, aForVia, radius, drillRadius ) ) + 1;
        if( isGeneralPolygonRoomObstacle( obstacle, net, aForVia )
            && !isGeneralConvexRoomObstacle( obstacle, net, aForVia ) )
        {
            if( const auto pieces = splitPolygonAreaToConvex( obstacle, expansion ) )
            {
                for( const PLANAR::SIMPLEX& piece : *pieces )
                {
                    if( const auto octagon = piece.BoundingOctagon() )
                        addOctagon( *octagon, piece );
                }
                continue;
            }
            // Malformed or out-of-range polygon data must fail closed.
            addOctagon( octagonalEnvelope( obstacle, expansion ) );
            continue;
        }

        std::optional<PLANAR::SIMPLEX> simplex;
        if( isGeneralConvexRoomObstacle( obstacle, net, aForVia ) )
        {
            const auto base = PLANAR::SIMPLEX::FromConvexPolygon(
                    obstacle.polygon );
            simplex = base ? base->Offset( expansion ) : std::nullopt;
        }
        if( aSourceTraceRooms && !aForVia && obstacle.isPad
            && obstacle.kind == ROUTER_OBSTACLE_KIND::RECTANGLE )
        {
            // ShapeSearchTree45Degree.calculateTreeShapes(DrillItem) avoids
            // corner cut-offs by offsetting an IntBox as an IntBox.  Calling
            // INT_OCTAGON::Offset here would chamfer the rectangular pad and
            // can change an orthogonal room restraint into a diagonal one.
            const ROUTER_BOX sourceBox{
                sourceGridCoordinate( obstacle.box.minX ),
                sourceGridCoordinate( obstacle.box.minY ),
                sourceGridCoordinate( obstacle.box.maxX ),
                sourceGridCoordinate( obstacle.box.maxY )
            };
            addOctagon( SHAPE_SEARCH_TREE_45_DEGREE::OffsetDrillItemBox(
                                sourceBox, sourceGridCoordinate( expansion ) ) );
        }
        else if( sourceTraceShape
                 && obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT
                 && obstacle.radius > 0 )
        {
            // ShapeSearchTree45Degree.calculateTreeShapes(DrillItem) first
            // obtains the physical Circle/Path bounding octagon and only then
            // applies clearance compensation.  Combining both radii changes
            // a Circle's deliberately asymmetric floor/ceil diagonal support.
            const std::int64_t sourceRadius = sourceGridCoordinate(
                    obstacle.radius );
            const std::int64_t sourceClearance = sourceGridCoordinate(
                    std::max<std::int64_t>( 0,
                                            sourceExpansion - obstacle.radius ) );
            const std::int64_t sourceUnit = static_cast<std::int64_t>(
                    FREEROUTING_COORDINATE_UNIT_IU );
            if( obstacle.start == obstacle.end )
            {
                addOctagon(
                        SHAPE_SEARCH_TREE_45_DEGREE::OffsetDrillItemCircle(
                                sourceGridPoint( obstacle.start ), sourceRadius,
                                sourceClearance, sourceUnit ) );
            }
            else
            {
                // PolygonPath.transformToBoardRel() offsets the endpoint
                // octagon by half the path width; the tree applies clearance
                // in a second operation.
                addOctagon( octagonalEnvelope(
                                    { obstacle.start, obstacle.end }, 0 )
                                    .OffsetOnGrid( sourceRadius, sourceUnit )
                                    .OffsetOnGrid( sourceClearance, sourceUnit ) );
            }
        }
        else
        {
            addOctagon( octagonalEnvelope( obstacle, expansion ),
                        std::move( simplex ) );
        }
    }
    // Every attempt sees current copper, including through-via copper on
    // intermediate layers. No stale per-net tree survives add/remove/rip-up.
    const std::vector<ROUTING_CONNECTION> routes = roomRouteItems();
    for( std::size_t routeIndex = 0; routeIndex < routes.size(); ++routeIndex )
    {
        if( aCancel && aCancel() )
            return {};

        const ROUTING_CONNECTION& connection = routes[routeIndex];
        currentTreeInsertionOrder = nextSourceOrder + 1 + routeIndex;
        currentTreeNet = routes[routeIndex].netCode;
        currentTraceInsertionSteps = connection.traceInsertionSteps.empty()
                ? nullptr
                : std::make_shared<const std::vector<ROUTING_TRACE_INSERTION_STEP>>(
                          connection.traceInsertionSteps );
        if( currentTraceInsertionSteps )
        {
            autorouterDecisionLog(
                    "TRACE_REPLAY_AVAILABLE",
                    { { "net", std::to_string( connection.netCode ) },
                      { "route_index", std::to_string( routeIndex ) },
                      { "tree_order", std::to_string( currentTreeInsertionOrder ) },
                      { "steps", std::to_string(
                                             currentTraceInsertionSteps->size() ) } } );
        }

        if( connection.netCode == net )
        {
            if( aForVia )
                for( std::size_t i = 1; i < connection.nodes.size(); ++i )
                    if( connection.nodes[i - 1].layer != connection.nodes[i].layer )
                    {
                        const auto p = connection.nodes[i].point;
                        add( { p.x, p.y, p.x, p.y }, drillRadius
                                + netViaDrillRadius( net )
                                + m_board.holeToHoleClearance + 1 );
                    }
            // A compensated Freerouting ShapeSearchTree contains same-net
            // PolylineTrace leaves too; leaf semantics ignore them during
            // completion. Omitting them changes tree topology for all later
            // foreign-net searches. Retain the legacy skip outside the exact
            // source-tree path and for drill-page construction.
            if( !aSourceTraceRooms || aForVia )
                continue;
        }
        for( std::size_t i = 1; i < connection.nodes.size(); ++i )
        {
            const std::size_t edge = i - 1;
            currentTreeShapeIndex = static_cast<int>( edge );
            const bool representedByRipupRoom = aRipupObstacles
                    && std::any_of( aRipupObstacles->begin(), aRipupObstacles->end(),
                                    [&]( const ROOM_RIPUP_OBSTACLE& aObstacle )
                                    {
                                        return aObstacle.connectionIndex == routeIndex
                                               && aObstacle.shape.shapeIndex
                                                          == static_cast<int>( edge )
                                               && aObstacle.shape.layer == aLayer;
                                    } );
            if( representedByRipupRoom )
                continue;

            const auto& from = connection.nodes[i - 1];
            const auto& to = connection.nodes[i];
            const ROUTING_EDGE_STYLE& style = EdgeStyle( connection, edge );
            std::int64_t expansion;
            if( from.layer == to.layer )
            {
                if( from.layer != aLayer )
                    continue;
                const std::int64_t otherRadius = style.trackWidth > 0
                        ? style.trackWidth / 2 : netTrackRadius( connection.netCode );
                const std::int64_t pairClearance = edgePairClearance(
                        net, connection.netCode, aLayer, 0, style.clearance );
                expansion = aSourceTraceRooms
                        ? otherRadius + std::max<std::int64_t>(
                                0, pairClearance - candidateCompensation )
                        : radius + otherRadius + pairClearance;
            }
            else
            {
                if( !VIA_RULE::SpansLayer( m_settings, from.layer, to.layer, style, aLayer ) )
                    continue;
                const std::int64_t otherViaRadius = std::max<std::int64_t>(
                        1, ViaStyleDiameterOnLayer( style, aLayer, 2 * netViaRadius( connection.netCode ) ) / 2 );
                const std::int64_t otherDrillRadius = style.viaDrill > 0
                        ? style.viaDrill / 2 : netViaDrillRadius( connection.netCode );
                const std::int64_t pairClearance = edgePairClearance( net, connection.netCode, aLayer, 0,
                                                                      ViaStyleClearanceOnLayer( style, aLayer ) );
                expansion = aSourceTraceRooms && !aForVia
                        ? otherViaRadius + std::max<std::int64_t>(
                                0, pairClearance - candidateCompensation )
                        : radius + std::max(
                                otherViaRadius + pairClearance,
                                otherDrillRadius + m_board.holeClearance );
                if( aForVia )
                    expansion = std::max( expansion, drillRadius
                            + otherDrillRadius + m_board.holeToHoleClearance );
            }
            currentTraceTreeExpansion = expansion;
            // PolylineTrace.calculateTreeShapes offsets by the exact
            // compensated half-width. Equality at the requested clearance is
            // legal; an extra native IU invents a different one-dimensional
            // door and changes the next maze frontier.
            addSegment( from.point, to.point,
                        expansion + ( aSourceTraceRooms && !aForVia ? 0 : 1 ) );
        }
    }
    if( aForVia && !m_settings.allowViaInSmdPad )
        for( const auto& pad : m_board.pads )
            if( pad.netCode == net && pad.isSmd && isOnPadLayer( pad, aLayer ) )
                add( { pad.position.x, pad.position.y, pad.position.x, pad.position.y },
                     std::max<std::int64_t>( 1, pad.radius ) + radius + 1 );
    return entries;
}


std::vector<ROOM_RIPUP_OBSTACLE> MAZE_SEARCH_ENGINE::roomRipupObstacles(
        int net, int aLayer, bool aForVia, int aRetry, bool aFanout,
        bool aSourceTraceRooms,
        const ROUTER_CANCEL_CALLBACK& aCancel ) const
{
    std::vector<ROOM_RIPUP_OBSTACLE> result;
    if( !m_useRoutableObstacleRooms || aForVia )
        return result;

    const std::int64_t radius = netTrackRadius( net );
    const std::int64_t candidateCompensation = traceClearanceCompensation( net );
    MAZE_RIPUP_RESOLVER resolver;
    MAZE_RIPUP_RESOLVER::CONTEXT context;
    context.startRipupCosts = std::max( 0, m_settings.startRipupCost );
    context.ripupPassNo = std::max( 1, aRetry + 1 );
    context.ripupCosts = context.startRipupCosts * context.ripupPassNo;
    context.isFanout = aFanout;

    // Java seeds one deterministic Random with ripupCosts.  The native
    // rectangular adapter calculates item-room costs before queue expansion,
    // so derive the same nextDouble sequence in stable route/edge order.
    constexpr std::uint64_t multiplier = 0x5DEECE66DULL;
    constexpr std::uint64_t addend = 0xBULL;
    constexpr std::uint64_t mask = ( 1ULL << 48 ) - 1;
    std::uint64_t randomState =
            ( static_cast<std::uint64_t>( context.ripupCosts ) ^ multiplier ) & mask;
    auto nextBits = [&]( int aBits )
    {
        randomState = ( randomState * multiplier + addend ) & mask;
        return randomState >> ( 48 - aBits );
    };
    auto nextDouble = [&]()
    {
        const std::uint64_t high = nextBits( 26 );
        const std::uint64_t low = nextBits( 27 );
        return static_cast<double>( ( high << 27 ) + low )
               / static_cast<double>( 1ULL << 53 );
    };

    const std::vector<ROUTING_CONNECTION> routes = roomRouteItems();
    std::size_t nextItemGroup = 0;
    for( std::size_t routeIndex = 0; routeIndex < routes.size(); ++routeIndex )
    {
        if( aCancel && aCancel() )
            return {};

        const ROUTING_CONNECTION& connection = routes[routeIndex];
        std::vector<std::size_t> edgeGroups;
        std::vector<std::size_t> edgeItemOrdinals;
        edgeGroups.reserve( connection.nodes.empty() ? 0 : connection.nodes.size() - 1 );
        edgeItemOrdinals.reserve( edgeGroups.capacity() );
        std::size_t routeItemCount = 0;
        for( std::size_t edge = 0; edge + 1 < connection.nodes.size(); ++edge )
        {
            const bool continuesTrace = edge > 0
                    && connection.nodes[edge - 1].layer == connection.nodes[edge].layer
                    && connection.nodes[edge].layer == connection.nodes[edge + 1].layer
                    && EdgeStyle( connection, edge - 1 ).trackWidth
                               == EdgeStyle( connection, edge ).trackWidth
                    && EdgeStyle( connection, edge - 1 ).clearance
                               == EdgeStyle( connection, edge ).clearance;
            if( !continuesTrace )
            {
                ++nextItemGroup;
                ++routeItemCount;
            }
            edgeGroups.push_back( nextItemGroup - 1 );
            edgeItemOrdinals.push_back( routeItemCount - 1 );
        }

        // Rebuild the source PolylineTrace item boundary.  KiCad stores a
        // routed connection as one node chain, while Freerouting creates one
        // PolylineTrace for each uninterrupted same-layer/same-style run.
        // Every tree shape of that item must share the same corner sequence
        // and transactional shove predicate.
        std::vector<std::shared_ptr<const MAZE_TRACE_ROOM_INFO>> edgeTraceInfo(
                edgeGroups.size() );
        for( std::size_t firstEdge = 0; firstEdge < edgeGroups.size(); )
        {
            const ROUTER_NODE& first = connection.nodes[firstEdge];
            const ROUTER_NODE& second = connection.nodes[firstEdge + 1];
            if( first.layer != second.layer )
            {
                ++firstEdge;
                continue;
            }

            const ROUTING_EDGE_STYLE style = EdgeStyle( connection, firstEdge );
            std::size_t lastEdge = firstEdge;
            while( lastEdge + 1 < edgeGroups.size()
                   && connection.nodes[lastEdge + 1].layer
                              == connection.nodes[lastEdge + 2].layer
                   && EdgeStyle( connection, lastEdge + 1 ) == style )
            {
                ++lastEdge;
            }

            auto info = std::make_shared<MAZE_TRACE_ROOM_INFO>();
            info->firstShapeIndex = firstEdge;
            info->halfWidth = style.trackWidth > 0
                                      ? style.trackWidth / 2
                                      : netTrackRadius( connection.netCode );
            info->clearance = style.clearance > 0
                                      ? style.clearance
                                      : netClearance( connection.netCode );
            info->compensatedHalfWidth = info->halfWidth
                    + std::max<std::int64_t>(
                            0,
                            edgePairClearance( net, connection.netCode,
                                               aLayer, 0, info->clearance )
                                    - candidateCompensation );
            info->sourceStyleMatches =
                    info->halfWidth == radius
                    && info->clearance == netClearance( net );
            info->corners.reserve( lastEdge - firstEdge + 2 );
            for( std::size_t node = firstEdge; node <= lastEdge + 1; ++node )
                info->corners.push_back( connection.nodes[node].point );

            if( info->sourceStyleMatches )
            {
                ROUTING_EDGE_STYLE candidateStyle;
                candidateStyle.trackWidth = 2 * radius;
                candidateStyle.clearance = netClearance( net );
                info->maxShoveLength =
                        [this, net, aLayer, candidateStyle, aCancel](
                                const FLOAT_LINE& aLine,
                                bool aShoveToTheLeft ) -> double
                {
                    if( aCancel && aCancel() )
                        return 0;

                    const double fullLength = aLine.a.Distance( aLine.b );
                    if( fullLength < 0.1 )
                        return std::numeric_limits<double>::infinity();

                    const ROUTER_NODE start{ aLine.a.Round(), aLayer };
                    const ROUTER_NODE requestedEnd{ aLine.b.Round(), aLayer };

                    // MazeTraceShover first asks RoutingBoard.checkTraceSegment
                    // for the prefix which is clear of *non-shovable* items.
                    // That query ignores movable traces and vias; removing the
                    // currently detected conflict set and calling the ordinary
                    // all-obstacle predicate is not equivalent because another
                    // movable item can still reject the segment.  Preserve both
                    // source one-coordinate safety margins: the first is
                    // applied inside CheckTraceSegmentLength(), and the second
                    // is applied here before changeLengthApprox().
                    const double immutableLength = CheckTraceSegmentLength(
                            net, start, requestedEnd, &candidateStyle );
                    double availableLength = fullLength;
                    bool segmentShortened = false;
                    if( std::isfinite( immutableLength ) )
                    {
                        availableLength = std::min(
                                fullLength,
                                immutableLength
                                        - FREEROUTING_COORDINATE_UNIT_IU );
                        if( availableLength <= 0 )
                            return 0;
                        segmentShortened = availableLength + 0.5 < fullLength;
                    }

                    const auto canShovePrefix = [&]( double aLength )
                    {
                        const bool fullProbe =
                                aLength + 0.5 >= availableLength;
                        const auto traceProbe = [&]( const char* aResult,
                                                     std::size_t aConflictCount = 0 )
                        {
                            if( fullProbe )
                                autorouterDecisionLog(
                                        "TRACE_SHOVE_PREFIX",
                                        { { "result", aResult },
                                          { "side",
                                            aShoveToTheLeft ? "LEFT" : "RIGHT" },
                                          { "length", std::to_string( aLength ) },
                                          { "conflicts",
                                            std::to_string( aConflictCount ) } } );
                        };
                        if( aCancel && aCancel() )
                        {
                            traceProbe( "cancelled" );
                            return false;
                        }

                        const FLOAT_POINT end = aLength + 0.5 >= availableLength
                                ? ( segmentShortened
                                            ? aLine.a.ChangeLength(
                                                      aLine.b, availableLength )
                                            : aLine.b )
                                : aLine.a.ChangeLength( aLine.b, aLength );
                        ROUTING_CONNECTION candidate;
                        candidate.netCode = net;
                        candidate.nodes = { start,
                                            { end.Round(), aLayer } };
                        candidate.complete = true;
                        candidate.edgeStyles = { candidateStyle };
                        if( candidate.nodes.front() == candidate.nodes.back() )
                        {
                            traceProbe( "point" );
                            return false;
                        }

                        const std::vector<ROUTING_CONNECTION> conflicts =
                                FindConflictingConnections( candidate );
                        if( conflicts.empty() )
                        {
                            traceProbe( "no_conflicts" );
                            return true;
                        }

                        ROUTING_OCCUPANCY::TRANSACTION restore( m_occupancy );
                        const auto inserted = FOUND_CONNECTION_INSERTER::Insert(
                                candidate, conflicts, m_occupancy, *this,
                                aCancel, false, nullptr );
                        if( inserted.state
                            != FOUND_CONNECTION_INSERTER::STATE::INSERTED )
                        {
                            traceProbe( "forced_insert_failed",
                                        conflicts.size() );
                            return false;
                        }

                        // ShapeEntrySide's LEFT/RIGHT flag selects the
                        // topological entry side of the incoming trace shape;
                        // it is not a fixed half-plane for every replacement
                        // corner.  A polyline bend can make two consecutive
                        // source-approved shoves displace their trace pieces
                        // onto opposite determinant sides of parallel shove
                        // segments.  sectionCanStartShove() already enforces
                        // the source entry-side rule.  Requiring every new
                        // contour corner to share one global sign here was an
                        // extra native restriction and rejected those legal
                        // bends.
                        const bool result = !inserted.shoved.empty();
                        traceProbe( result ? "success" : "not_shoved",
                                    conflicts.size() );
                        return result;
                    };

                    if( canShovePrefix( availableLength ) )
                    {
                        return segmentShortened
                                ? availableLength
                                : std::numeric_limits<double>::infinity();
                    }

                    // Both source checks return the longest usable prefix.
                    // Preserve that partial-progress contract with a bounded
                    // integral binary search rather than collapsing every
                    // blocked endpoint into an all-or-nothing result.
                    double low = 0;
                    double high = availableLength;
                    for( int iteration = 0; iteration < 24 && high - low > 1; ++iteration )
                    {
                        const double middle = std::floor( ( low + high ) / 2 );
                        if( canShovePrefix( middle ) )
                            low = middle;
                        else
                            high = middle;
                    }
                    return low;
                };
            }

            for( std::size_t edge = firstEdge; edge <= lastEdge; ++edge )
                edgeTraceInfo[edge] = info;
            firstEdge = lastEdge + 1;
        }

        const std::vector<ROUTING_BOARD::ITEM_ID> routeItems = m_occupancy.Board()
                ? m_occupancy.Board()->RouteItems( connection )
                : std::vector<ROUTING_BOARD::ITEM_ID>{};
        const bool movableRoute = connection.isShoveMovable
                                  && ( !connection.isExistingBoardRoute
                                       || connection.isAutorouterOwned
                                       || m_settings.allowRipupExisting );
        if( !movableRoute || connection.netCode == net || !connection.complete
            || !HasValidEdgeStyles( connection ) )
        {
            continue;
        }

        for( std::size_t edge = 0; edge + 1 < connection.nodes.size(); ++edge )
        {
            const ROUTER_NODE& from = connection.nodes[edge];
            const ROUTER_NODE& to = connection.nodes[edge + 1];
            const ROUTING_EDGE_STYLE& style = EdgeStyle( connection, edge );
            std::vector<std::int64_t> additionalViaTraceHalfWidths;
            bool unrippableNormalContact = false;
            if( from.layer != to.layer && from.point == to.point )
            {
                const std::vector<int> viaLayers = VIA_RULE::LayersFor(
                        m_settings, from.layer, to.layer, &style );
                const auto onViaLayer = [&]( int aCandidateLayer )
                {
                    return std::find( viaLayers.begin(), viaLayers.end(), aCandidateLayer )
                           != viaLayers.end();
                };
                // Via.getNormalContacts() rejects rip-up as soon as a pin or
                // conduction area is present; forced insertion may still
                // move the via transactionally while preserving that contact.
                for( const ROUTING_PAD& pad : m_board.pads )
                {
                    if( pad.netCode != connection.netCode || pad.position != from.point )
                        continue;
                    if( std::any_of( viaLayers.begin(), viaLayers.end(),
                                    [&]( int aViaLayer )
                                    { return isOnPadLayer( pad, aViaLayer ); } ) )
                    {
                        unrippableNormalContact = true;
                        break;
                    }
                }
                for( const ROUTING_OBSTACLE& area : m_board.conductionAreas )
                {
                    if( unrippableNormalContact || area.netCode != connection.netCode )
                        continue;
                    for( const int viaLayer : viaLayers )
                    {
                        if( ( area.layers.empty()
                              || std::find( area.layers.begin(), area.layers.end(), viaLayer ) != area.layers.end() )
                            && areaTouchesVia(
                                    area, from.point,
                                    std::max<std::int64_t>(
                                            1, ViaStyleDiameterOnLayer( style, viaLayer,
                                                                        2 * netViaRadius( connection.netCode ) )
                                                       / 2 ) ) )
                        {
                            unrippableNormalContact = true;
                            break;
                        }
                    }
                }

                // Snapshot reconstruction keeps an imported via and each
                // contacted trace as separate native connections. Rebuild
                // the source Via.getNormalContacts() width/fixed-state view.
                for( std::size_t contactRouteIndex = 0;
                     contactRouteIndex < routes.size() && !unrippableNormalContact;
                     ++contactRouteIndex )
                {
                    if( contactRouteIndex == routeIndex )
                        continue;
                    const ROUTING_CONNECTION& contact = routes[contactRouteIndex];
                    if( contact.netCode != connection.netCode
                        || !HasValidEdgeStyles( contact ) )
                    {
                        continue;
                    }
                    for( std::size_t contactEdge = 0;
                         contactEdge + 1 < contact.nodes.size(); ++contactEdge )
                    {
                        const ROUTER_NODE& contactFrom = contact.nodes[contactEdge];
                        const ROUTER_NODE& contactTo = contact.nodes[contactEdge + 1];
                        const ROUTING_EDGE_STYLE& contactStyle = EdgeStyle(
                                contact, contactEdge );
                        if( contactFrom.layer != contactTo.layer )
                        {
                            if( contactFrom.point == from.point
                                && contactTo.point == from.point )
                                unrippableNormalContact = true;
                            continue;
                        }
                        if( !onViaLayer( contactFrom.layer )
                            || ( contactFrom.point != from.point
                                 && contactTo.point != from.point ) )
                        {
                            continue;
                        }
                        if( !contact.isShoveMovable )
                        {
                            unrippableNormalContact = true;
                            break;
                        }
                        additionalViaTraceHalfWidths.push_back(
                                contactStyle.trackWidth > 0
                                        ? contactStyle.trackWidth / 2
                                        : netTrackRadius( contact.netCode ) );
                    }
                }
            }

            if( unrippableNormalContact )
                continue;

            std::int64_t expansion = 0;
            if( from.layer == to.layer )
            {
                if( from.layer != aLayer )
                    continue;
                const std::int64_t otherRadius = style.trackWidth > 0
                        ? style.trackWidth / 2 : netTrackRadius( connection.netCode );
                const std::int64_t pairClearance = edgePairClearance(
                        net, connection.netCode, aLayer, 0, style.clearance );
                expansion = aSourceTraceRooms
                        ? otherRadius + std::max<std::int64_t>(
                                0, pairClearance - candidateCompensation )
                        : radius + otherRadius + pairClearance;
            }
            else
            {
                if( !VIA_RULE::SpansLayer( m_settings, from.layer, to.layer, style, aLayer ) )
                    continue;
                const std::int64_t otherViaRadius = std::max<std::int64_t>(
                        1, ViaStyleDiameterOnLayer( style, aLayer, 2 * netViaRadius( connection.netCode ) ) / 2 );
                const std::int64_t otherDrillRadius = style.viaDrill > 0
                        ? style.viaDrill / 2 : netViaDrillRadius( connection.netCode );
                const std::int64_t pairClearance = edgePairClearance( net, connection.netCode, aLayer, 0,
                                                                      ViaStyleClearanceOnLayer( style, aLayer ) );
                expansion = aSourceTraceRooms
                        ? otherViaRadius + std::max<std::int64_t>(
                                0, pairClearance - candidateCompensation )
                        : radius + std::max(
                                otherViaRadius + pairClearance,
                                otherDrillRadius + m_board.holeClearance );
            }

            const INT_OCTAGON octagon = octagonalEnvelope(
                    { from.point, to.point }, expansion );
            std::optional<PLANAR::SIMPLEX> simplex;
            if( from.point != to.point )
                simplex = PLANAR::SIMPLEX::FromExpandedSegment(
                        from.point, to.point, expansion );
            else
                simplex = octagon.ToSimplex();
            SHAPE_TREE_ENTRY entry{ octagon.BoundingBox(), 0,
                                    static_cast<int>( edge ), aLayer,
                                    connection.netCode, false, true, octagon,
                                    std::move( simplex ) };
            entry.treeInsertionOrder = m_board.obstacles.size() + 1 + routeIndex;
            if( !connection.traceInsertionSteps.empty() )
            {
                entry.traceInsertionSteps = std::make_shared<
                        const std::vector<ROUTING_TRACE_INSERTION_STEP>>(
                                connection.traceInsertionSteps );
                entry.traceTreeExpansion = expansion;
            }
            std::optional<CONNECTION> topologyConnection;
            if( routeItems.size() == routeItemCount )
                topologyConnection = CONNECTION::Get(
                        *m_occupancy.Board(), routeItems[edgeItemOrdinals[edge]] );

            const int ripupCost = resolver.CheckRipup(
                    connection, edge, netTrackRadius( connection.netCode ), context,
                    nextDouble(), additionalViaTraceHalfWidths,
                    topologyConnection ? &*topologyConnection : nullptr );
            if( ripupCost >= 0 )
            {
                std::uint64_t sourceObjectId = 0;
                if( routeItems.size() == routeItemCount )
                    sourceObjectId = m_occupancy.Board()->SourceObjectId(
                            routeItems[edgeItemOrdinals[edge]] );
                else if( m_occupancy.Board() )
                    sourceObjectId = static_cast<std::uint64_t>(
                            m_occupancy.Board()->ItemCount() + edgeGroups[edge] + 1 );
                else
                    sourceObjectId = ( std::uint64_t{ 1 } << 32 )
                                     + edgeGroups[edge];
                result.push_back( { std::move( entry ), edgeGroups[edge], ripupCost,
                                    routeIndex, edgeTraceInfo[edge],
                                    connection.netCode, sourceObjectId } );
            }
        }
    }
    return result;
}


std::optional<ROUTING_CONNECTION> MAZE_SEARCH_ENGINE::findRoomConnection(
        const std::vector<ROUTING_TERMINAL>& aStarts,
        const std::vector<ROUTING_TERMINAL>& aTargets, int aRetry, int& aExpanded,
        const ROUTER_CANCEL_CALLBACK& aCancel,
        const ROUTER_SEARCH_PROGRESS_CALLBACK& aProgress ) const
{
    const int net = aStarts.front().pad.netCode;

    const auto radius = netTrackRadius( net );
    const auto compensation = traceClearanceCompensation( net );
    const std::int64_t edgeToTurn = pinEdgeToTurnDistance( m_board );
    const ROUTER_BOX bounds = BOARD_OUTLINE::SearchBounds( m_board );
    std::optional<ROUTING_CONNECTION> best;
    std::int64_t bestRipupCost = 0;
    const auto started = std::chrono::steady_clock::now();
    for( const auto& layer : m_settings.layers )
    {
        if( !layer.enabled || ( aCancel && aCancel() ) )
            continue;
        auto terminals = [&]( const auto& source )
        {
            std::vector<ROOM_TERMINAL> result;
            for( const auto& terminal : source )
            {
                if( !isOnPadLayer( terminal.pad, layer.layerId ) )
                    continue;
                const auto start = terminal.pad.position;
                const auto end = terminal.segmentEnd.value_or( start );
                const auto treeOctagon = terminalTreeOctagon(
                        terminal, layer.layerId, compensation );
                result.push_back( { start, end, terminal.padIndex,
                                    treeOctagon
                                            ? treeOctagon->BoundingBox()
                                            : terminalTreeBounds(
                                                      terminal, layer.layerId,
                                                      compensation ),
                                    terminal.connectionArea,
                                    terminal.connectionArea ? radius : 0,
                                    terminalTraceExitRestrictions(
                                            terminal, layer.layerId ),
                                    static_cast<double>( edgeToTurn
                                                         + std::max<std::int64_t>(
                                                                 1, radius + compensation ) ),
                                    terminal.itemId, terminal.treeEntryIndex,
                                    treeOctagon } );
            }
            return result;
        };
        const auto starts = terminals( aStarts );
        const auto targets = terminals( aTargets );
        if( starts.empty() || targets.empty() )
            continue;
        const bool anyAngle = hasGeneralConvexRoomGeometry( net, layer.layerId );
        const bool plane = std::any_of(
                aTargets.begin(), aTargets.end(),
                []( const ROUTING_TERMINAL& aTarget )
                {
                    return aTarget.pad.isPlaneTarget;
                } );
        // Freerouting's room tree contains only the obstacle-side clearance
        // share for ordinary traces.  The locator then realizes the candidate
        // centreline with its compensated half-width.  Plane targets retain
        // complete-centre geometry until their finite area semantics are
        // represented by the native target door.
        const bool sourceTraceRooms = !plane;
        const auto ripupEntries = roomRipupObstacles(
                net, layer.layerId, false, aRetry, false, sourceTraceRooms, aCancel );
        const auto entries = roomObstacles( net, layer.layerId, false, false,
                                            sourceTraceRooms, aCancel, &ripupEntries );
        // Geometric per-axis costs for the isolated no-via frontier. The
        // existing dialog's direction penalty is mapped here, not substituted
        // into the legacy queue's incompatible grid-normalized heuristic.
        const auto [horizontal, vertical] = layer.TraceCosts( m_settings.traceLengthCost );
        ROUTER_ANGLE_RESTRICTION angleRestriction = anyAngle
                ? ROUTER_ANGLE_RESTRICTION::ANY_ANGLE
                : ROUTER_ANGLE_RESTRICTION::FORTYFIVE_DEGREE;
        auto path = anyAngle
                ? MAZE_SEARCH_ENGINE_ANY_ANGLE::FindConnection(
                        bounds, entries, layer.layerId, net, starts, targets,
                        std::max<std::int64_t>( 1, radius + compensation ),
                        horizontal, vertical,
                        m_settings.maxExpandedNodes, aExpanded, m_roomMetrics,
                        aCancel, aProgress,
                        static_cast<double>( std::max( 0, m_settings.bendCost ) )
                                * std::max( 1, m_settings.gridStepIU ), ripupEntries,
                        sourceTraceRooms )
                : MAZE_SEARCH_ENGINE_45_DEGREE::FindConnection(
                        bounds, entries, layer.layerId, net, starts, targets,
                        std::max<std::int64_t>( 1, radius + compensation ),
                        horizontal, vertical,
                        m_settings.maxExpandedNodes, aExpanded, m_roomMetrics,
                        aCancel, aProgress,
                        static_cast<double>( std::max( 0, m_settings.bendCost ) )
                                * std::max( 1, m_settings.gridStepIU ), ripupEntries );
        // Keep the established rectangular frontier as a bounded transition
        // fallback until the octagonal drill frontier is connected.  General
        // convex layers must not collapse back to bounding rectangles.
        if( !path && !anyAngle
            && aExpanded < m_settings.maxExpandedNodes )
        {
            path = MAZE_SEARCH_ENGINE_90_DEGREE::FindConnection(
                    bounds, entries, layer.layerId, net, starts, targets,
                    std::max<std::int64_t>( 1, radius + compensation ),
                    horizontal, vertical,
                    m_settings.maxExpandedNodes, aExpanded, m_roomMetrics,
                    aCancel, aProgress, false,
                    static_cast<double>( std::max( 0, m_settings.bendCost ) )
                            * std::max( 1, m_settings.gridStepIU ), ripupEntries,
                    sourceTraceRooms );
            if( path )
                angleRestriction = ROUTER_ANGLE_RESTRICTION::NINETY_DEGREE;
        }
        if( !path )
            continue;
        ROUTING_CONNECTION found;
        found.netCode = net;
        found.fromPadIndex = path->startOwner;
        found.toPadIndex = path->targetOwner;
        found.complete = true;
        found.insertionBacktracksFromTarget = true;
        found.angleRestriction = angleRestriction;
        found.cost = path->ripupCost;
        for( const auto& point : path->points )
            found.nodes.push_back( { point, layer.layerId } );
        // FoundConnectionLocator removes corners that are no longer needed
        // after a room/door chain has been reconstructed.  In particular, an
        // exact convex obstacle can split free space into several rooms even
        // though the final source-to-target segment merely crosses the
        // obstacle's bounding-box corner and misses its real contour.  Pull
        // that chain tight before validating it so the room adapter does not
        // preserve decomposition-only doglegs.
        MAZE_TRACE_SHOVER::Shorten( found, *this );
        bool legal = !found.nodes.empty();
        // The first node is an already-existing source/target contact. It may
        // legitimately lie inside its pad or inside the board-edge centre
        // margin. Validate generated edges, not a synthetic zero-length edge
        // at that existing contact.
        for( std::size_t i = 1; i < found.nodes.size() && legal; ++i )
        {
            const auto& from = found.nodes[i - 1];
            const auto& to = found.nodes[i];
            legal = CanUseSegment( net, from, to );
            if( !legal && autorouterDebugEnabled() )
            {
                std::ostringstream message;
                message << "ROOM_PATH_REJECTED net=" << net << " edge="
                        << i - 1 << " from=(" << from.point.x << ','
                        << from.point.y << ",L" << from.layer << ") to=(" << to.point.x
                        << ',' << to.point.y << ",L" << to.layer << ") ripup_cost="
                        << path->ripupCost << " ripped_groups="
                        << path->rippedObstacleGroups.size();
                autorouterDebugLog( message.str() );
            }
            found.cost += std::abs( static_cast<double>( to.point.x ) - from.point.x ) * horizontal
                          + std::abs( static_cast<double>( to.point.y ) - from.point.y ) * vertical;
        }
        if( legal && ( !best || found.cost < best->cost ) )
        {
            bestRipupCost = path->ripupCost;
            best = std::move( found );
        }
    }
    m_roomMetrics.routed = best.has_value();
    if( autorouterDebugEnabled() )
    {
        std::ostringstream log;
        log << "ROOM_SEARCH net=" << net << " rooms=" << m_roomMetrics.rooms
            << " doors=" << m_roomMetrics.doors << " sections=" << m_roomMetrics.sections
            << " destination_queries=" << m_roomMetrics.destinationQueries
            << " expanded=" << aExpanded << " accepted=" << m_roomMetrics.routed
            << " elapsed_ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - started ).count();
        autorouterDebugLog( log.str() );
    }
    if( aCancel && aCancel() )
        return std::nullopt;
    if( best )
    {
        // The batch/rip-up layer still consumes legacy route-cost units. Do
        // not leak geometric IU costs from the new frontier into that score.
        AUTOROUTE_CONTROL control( m_settings, net, aRetry, false,
                                   std::max( netTrackRadius( net ), netViaRadius( net ) ),
                                   isPureSmdNet( net ) );
        best->cost = bestRipupCost;
        for( std::size_t i = 1; i < best->nodes.size(); ++i )
        {
            const auto& from = best->nodes[i - 1];
            const auto& to = best->nodes[i];
            best->cost += control.WeightedTraceCost( to.layer, from.point, to.point )
                    + control.CongestionCost( m_occupancy.SegmentUsage( from, to, net ) )
                    + ( from.point != to.point ? m_settings.bendCost : 0 );
        }
    }
    return best;
}
std::optional<ROUTING_CONNECTION> MAZE_SEARCH_ENGINE::findMultilayerRoomConnection(
        const std::vector<ROUTING_TERMINAL>& starts, const std::vector<ROUTING_TERMINAL>& targets,
        int retry, int& expanded, const ROUTER_CANCEL_CALLBACK& cancel,
        const ROUTER_SEARCH_PROGRESS_CALLBACK& progress,
        const ROUTING_PAD* fanoutTarget ) const
{
    const int net = starts.front().pad.netCode;

    const auto radius = netTrackRadius( net );
    const auto compensation = traceClearanceCompensation( net );
    const std::int64_t edgeToTurn = pinEdgeToTurnDistance( m_board );
    const ROUTER_BOX bounds = BOARD_OUTLINE::SearchBounds( m_board );
    const auto physical = VIA_RULE::ThroughLayers( m_settings );
    if( physical.size() < 2 )
        return std::nullopt;
    std::vector<ROOM_LAYER> layers;
    ROOM_VIA_SETTINGS via;
    via.transitionsEnabled = m_settings.allowVias;
    via.attachSmd = m_settings.allowViaInSmdPad;
    const auto netRule = std::find_if( m_board.nets.begin(), m_board.nets.end(),
                                      [net]( const ROUTING_NET& aNet )
                                      { return aNet.netCode == net; } );

    // AutorouteControl.rebuildViaInfo builds one maximum copper radius per
    // physical layer from the ordered ViaRule.  Preserve that distinction:
    // a top-to-inner blind rule must not inherit an unrelated bottom-layer
    // obstacle merely because another net-wide scalar happened to be larger.
    // The exact selected ViaInfo is still checked at every transition below.
    std::vector<ROUTING_VIA_PROFILE> profiles;
    if( m_viaOverride && m_viaOverrideNetCode == net )
    {
        profiles.push_back( *m_viaOverride );
    }
    else if( netRule != m_board.nets.end() && !netRule->viaProfiles.empty() )
    {
        profiles = netRule->viaProfiles;
    }
    else
    {
        ROUTING_VIA_PROFILE legacy;
        legacy.diameter = netRule != m_board.nets.end() && netRule->viaDiameter > 0
                                  ? netRule->viaDiameter : 2 * netViaRadius( net );
        legacy.drill = netRule != m_board.nets.end() && netRule->viaDrill > 0
                               ? netRule->viaDrill : 2 * netViaDrillRadius( net );
        legacy.type = ROUTER_VIA_TYPE::THROUGH;
        profiles.push_back( std::move( legacy ) );
    }

    std::map<int, std::int64_t> viaRadiusByLayer;
    std::map<int, std::int64_t> viaDrillRadiusByLayer;
    std::set<int> viaLayers;
    std::int64_t maximumViaRadius = radius;
    for( const ROUTING_VIA_PROFILE& profile : profiles )
    {
        ROUTING_EDGE_STYLE style;
        style.viaDiameter = profile.diameter > 0
                                    ? profile.diameter
                                    : netRule != m_board.nets.end()
                                              ? netRule->viaDiameter : 0;
        style.viaDrill = profile.drill > 0
                                 ? profile.drill
                                 : netRule != m_board.nets.end()
                                           ? netRule->viaDrill : 0;
        style.viaLayers = profile.layers;
        style.viaType = profile.type;
        style.viaLayerGeometry = profile.layerGeometry;
        if( style.viaDiameter <= 0 || style.viaDrill <= 0 )
            continue;

        std::vector<int> span;
        if( style.viaLayers.empty() )
        {
            span = VIA_RULE::LayersFor( m_settings, physical.front(), physical.back(),
                                        &style );
        }
        else
        {
            int firstOrdinal = std::numeric_limits<int>::max();
            int lastOrdinal = std::numeric_limits<int>::min();
            for( int declared : style.viaLayers )
            {
                const auto position = std::find( physical.begin(), physical.end(), declared );
                if( position == physical.end() )
                {
                    firstOrdinal = std::numeric_limits<int>::max();
                    break;
                }
                const int ordinal = static_cast<int>( std::distance( physical.begin(), position ) );
                firstOrdinal = std::min( firstOrdinal, ordinal );
                lastOrdinal = std::max( lastOrdinal, ordinal );
            }
            if( firstOrdinal < lastOrdinal )
                span = VIA_RULE::LayersFor( m_settings, physical[firstOrdinal],
                                            physical[lastOrdinal], &style );
        }
        if( span.empty() )
            continue;

        const std::int64_t profileDrillRadius = std::max<std::int64_t>( 1,
                                                                        style.viaDrill / 2 );
        for( int layer : span )
        {
            const std::int64_t profileRadius =
                    std::max<std::int64_t>( radius, ViaStyleDiameterOnLayer( style, layer, style.viaDiameter ) / 2 );
            maximumViaRadius = std::max( maximumViaRadius, profileRadius );
            viaLayers.insert( layer );
            viaRadiusByLayer[layer] = std::max( viaRadiusByLayer[layer], profileRadius );
            viaDrillRadiusByLayer[layer] = std::max( viaDrillRadiusByLayer[layer],
                                                     profileDrillRadius );
        }
    }
    if( via.transitionsEnabled && viaLayers.empty() )
        return std::nullopt;

    // AutorouteEngine constructs DrillPageArray over RoutingBoard.boundingBox.
    // It does not pre-inset the page lattice by a via radius or edge
    // clearance.  BoardOutline's compensated tree shapes cut the unusable
    // perimeter out of each page, and the ordered ViaRule/host DRC preflight
    // remains the final authority for every resulting centroid.  Insetting
    // here moved every page centroid off Freerouting's integer lattice (and,
    // after conversion back from KiCad IU, often off the 0.1 um source grid),
    // which made locator backtracking disagree with the selected drill and
    // prevented otherwise empty multilayer boards from routing.
    via.bounds = bounds;
    via.pageWidth = std::max<std::int64_t>( 10000, 10 * maximumViaRadius );
    via.attachSmd = via.attachSmd
                    || std::any_of( profiles.begin(), profiles.end(),
                                    []( const ROUTING_VIA_PROFILE& aProfile )
                                    { return aProfile.attachSmdAllowed; } );
    if( via.attachSmd )
        for( const auto& pad : m_board.pads )
            if( pad.netCode == net && pad.isSmd && !pad.isFanoutTarget && !pad.isPlaneTarget )
                for( std::size_t ordinal = 0; ordinal < physical.size(); ++ordinal )
                    if( isOnPadLayer( pad, physical[ordinal] ) )
                        via.pins.push_back( { pad.position, static_cast<int>( ordinal ), true } );
    bool pureSmd = true, hasPads = false;
    for( const auto& pad : m_board.pads )
        if( pad.netCode == net && !pad.isPlaneTarget && !pad.isFanoutTarget )
        { hasPads = true; pureSmd = pureSmd && pad.isSmd; }
    const bool plane = std::any_of( targets.begin(), targets.end(),
                                    []( const auto& t ) { return t.pad.isPlaneTarget; } );
    via.normalCost = MAZE_EXPANSION_ENGINE::NormalViaCost( maximumViaRadius,
            std::max( 0, plane ? m_settings.planeViaCost : m_settings.viaCost ), hasPads && pureSmd );
    const auto selectViaStyle = [&]( ROUTER_POINT point, int fromLayer, int toLayer )
            -> std::optional<ROUTING_EDGE_STYLE>
    {
        const bool attachesToSmd = std::any_of(
                m_board.pads.begin(), m_board.pads.end(),
                [&]( const ROUTING_PAD& pad )
                {
                    return pad.netCode == net && pad.isSmd && pad.position == point
                           && ( isOnPadLayer( pad, fromLayer )
                                || isOnPadLayer( pad, toLayer ) );
                } );
        return SelectViaStyle( net, { point, fromLayer }, { point, toLayer },
                               attachesToSmd );
    };
    via.selectViaStyle = selectViaStyle;
    via.canDrill = [&, selectViaStyle]( ROUTER_POINT point )
    {
        // Drill pages are a geometric broad phase.  Accept a candidate only
        // if at least one enabled transition has an actually legal ordered
        // ViaRule entry; the transition queue repeats this test and retains
        // the selected complete padstack style.
        for( std::size_t from = 0; from < physical.size(); ++from )
        {
            if( !isLayerEnabled( physical[from] ) )
                continue;
            for( std::size_t to = from + 1; to < physical.size(); ++to )
            {
                if( isLayerEnabled( physical[to] )
                    && selectViaStyle( point, physical[from], physical[to] ) )
                {
                    return true;
                }
            }
        }
        return false;
    };
    if( fanoutTarget )
    {
        via.stopAtFirstDrill = true;
        via.fanoutSourceLayer = fanoutTarget->fanoutSourceLayer;
        via.fanoutMinDistance = std::max<std::int64_t>(
                0, fanoutTarget->fanoutMinEscapeLength );
        via.fanoutMaxDistance = std::max<std::int64_t>(
                0, fanoutTarget->fanoutMaxEscapeLength );
        via.fanoutCenter = starts.front().pad.position;
        if( fanoutTarget->fanoutSourcePadIndex < m_board.pads.size() )
            via.fanoutCenter = m_board.pads[fanoutTarget->fanoutSourcePadIndex].position;

        // RoutingBoard.fanout() changes only the drill termination rule.  A
        // real destination item remains a legal target door regardless of
        // how far it is from the source pin; the configured escape envelope
        // constrains candidate drill locations, not ordinary pad-to-pad
        // completion.  Requiring every item in a large mixed-layer target set
        // to lie inside that envelope suppressed all direct targets and made
        // the native router choose a local via even when Freerouting reached a
        // nearby same-layer pad first.
        via.allowDirectFanoutTarget = !targets.empty();
    }
    const auto started = std::chrono::steady_clock::now();
    // RoutingBoard.fanout() uses the same 45-degree room/door/drill frontier
    // as an ordinary connection and changes only its termination condition:
    // the first drill which exits the source layer is a destination.  Keeping
    // Fanout on the older rectangular frontier changed both drill ordering and
    // narrow diagonal clearance decisions. Exact fixed-direction and general
    // trace rooms are available, so use the source-shaped frontier for both
    // call paths. Drill pages retain their source rectangular partition.
    const bool exactFrontier = true;
    const bool anyAngleFrontier = std::any_of(
            physical.begin(), physical.end(),
            [&]( int aLayer )
            {
                return hasGeneralConvexRoomGeometry( net, aLayer );
            } );
    // RoutingBoard.fanout() constructs the same compensated ShapeSearchTree
    // as ordinary autorouting.  Its only semantic difference is terminating
    // at the first legal drill, so fanout must retain the obstacle-side
    // clearance share and carry the candidate's compensated half-width in the
    // locator too.  Expanding every obstacle by the complete centre radius
    // shrinks the very first fanout room by one trace half-width and changes
    // door section counts.  Plane targets remain on the complete-centre path
    // until their finite-area target-door semantics are translated.
    const bool sourceTraceRooms = !plane;
    int nextObstacleId = 1;
    for( int id : physical )
    {
        if( cancel && cancel() )
            return std::nullopt;
        const auto setting = std::find_if( m_settings.layers.begin(), m_settings.layers.end(),
                                           [id]( const auto& l ) { return l.layerId == id; } );
        ROOM_LAYER layer;
        layer.id = id;
        layer.active = setting->enabled
                       && ( exactFrontier || !hasGeneralConvexRoomGeometry( net, id ) );
        layer.bounds = bounds;
        if( layer.active )
        {
            layer.ripupObstacles = roomRipupObstacles(
                    net, id, false, retry, fanoutTarget != nullptr,
                    sourceTraceRooms, cancel );
            layer.obstacles = roomObstacles( net, id, false, false,
                                             sourceTraceRooms, cancel,
                                             &layer.ripupObstacles );
        }
        const auto [horizontalCost, verticalCost] =
                setting->TraceCosts( m_settings.traceLengthCost );
        layer.horizontalCost = horizontalCost;
        layer.verticalCost = verticalCost;
        layer.bendCost = static_cast<double>( std::max( 0, m_settings.bendCost ) )
                         * std::max( 1, m_settings.gridStepIU );
        auto append = [&]( const auto& terminals, auto& output, bool aTarget )
        {
            for( const auto& t : terminals )
            {
                if( !isOnPadLayer( t.pad, id ) )
                    continue;
                const auto a = t.pad.position, b = t.segmentEnd.value_or( a );
                const auto treeOctagon = terminalTreeOctagon(
                        t, id, compensation );
                if( exactFrontier || aTarget || a.x == b.x || a.y == b.y )
                    output.push_back( { a, b, t.padIndex,
                                        treeOctagon
                                                ? treeOctagon->BoundingBox()
                                                : terminalTreeBounds(
                                                          t, id, compensation ),
                                        t.connectionArea,
                                        t.connectionArea ? radius : 0,
                                        terminalTraceExitRestrictions( t, id ),
                                        static_cast<double>( edgeToTurn
                                                             + std::max<std::int64_t>(
                                                                     1, radius
                                                                                + compensation ) ),
                                        t.itemId, t.treeEntryIndex,
                                        treeOctagon } );
                else
                {
                    ROUTING_TERMINAL first = t;
                    first.segmentEnd.reset();
                    first.pad.position = a;
                    ROUTING_TERMINAL second = first;
                    second.pad.position = b;
                    output.push_back( { a, a, t.padIndex,
                                        terminalTreeBounds( first, id, compensation ),
                                        t.connectionArea,
                                        t.connectionArea ? radius : 0, {}, 0,
                                        t.itemId, t.treeEntryIndex, {} } );
                    output.push_back( { b, b, t.padIndex,
                                        terminalTreeBounds( second, id, compensation ),
                                        t.connectionArea,
                                        t.connectionArea ? radius : 0, {}, 0,
                                        t.itemId, t.treeEntryIndex, {} } );
                }
            }
        };
        append( starts, layer.starts, false );
        append( targets, layer.targets, true );
        if( via.transitionsEnabled && viaLayers.contains( id ) )
        {
            for( auto obstacle : roomObstacles(
                         net, id, true, true, false, cancel, nullptr,
                         viaRadiusByLayer.at( id ), viaDrillRadiusByLayer.at( id ) ) )
            {
                obstacle.objectId = nextObstacleId++;
                via.obstacles.push_back( obstacle );
            }
        }
        layers.push_back( std::move( layer ) );
    }
    const auto path = anyAngleFrontier
            ? MAZE_SEARCH_ENGINE_ANY_ANGLE::FindMultilayerConnection(
                      layers, net, std::max<std::int64_t>( 1, radius + compensation ), via,
                      m_settings.maxExpandedNodes, expanded, m_roomMetrics,
                      cancel, progress, sourceTraceRooms )
            : exactFrontier
            ? MAZE_SEARCH_ENGINE_45_DEGREE::FindMultilayerConnection(
                      layers, net, std::max<std::int64_t>( 1, radius + compensation ), via,
                      m_settings.maxExpandedNodes, expanded, m_roomMetrics,
                      cancel, progress, sourceTraceRooms, &m_persistent45Tree )
            : MAZE_SEARCH_ENGINE_90_DEGREE::FindMultilayerConnection(
                      layers, net, std::max<std::int64_t>( 1, radius + compensation ), via,
                      m_settings.maxExpandedNodes, expanded, m_roomMetrics,
                      cancel, progress, false, sourceTraceRooms );
    std::optional<ROUTING_CONNECTION> result;
    if( path && !path->nodes.empty() )
    {
        ROUTING_CONNECTION found;
        found.netCode = net; found.fromPadIndex = path->startOwner; found.toPadIndex = path->targetOwner;
        found.complete = true; found.nodes = path->nodes;
        found.edgeStyles = path->edgeStyles;
        found.cost = path->ripupCost;
        found.isFanoutConnection = fanoutTarget != nullptr;
        found.angleRestriction = anyAngleFrontier
                ? ROUTER_ANGLE_RESTRICTION::ANY_ANGLE
                : exactFrontier
                ? ROUTER_ANGLE_RESTRICTION::FORTYFIVE_DEGREE
                : ROUTER_ANGLE_RESTRICTION::NINETY_DEGREE;

        // Public native routes remain start-to-target. The source inserter is
        // order-sensitive and consumes this maze result from the destination
        // back toward the start; carry that lifecycle explicitly rather than
        // reversing the public connection and its pad ownership here.
        found.insertionBacktracksFromTarget = true;

        bool legal = assignViaStyles( found );
        // FoundConnectionLocator emits every rounded room/door corner and the
        // source forced inserter consumes that sequence incrementally.  Do not
        // run a speculative line-of-sight simplifier here: even a geometrically
        // legal shortcut changes the private board seen by the next routing
        // item and bypasses the source insertion/pull-tight lifecycle.
        // The source does not run a second whole-route legality preflight after
        // locating the backtrack path. FoundConnectionInserter consumes each
        // span on the transactional board, where forced insertion can rewind,
        // spring over, neck down, or reject it without publishing partial
        // copper. Returning the located path here preserves that lifecycle;
        // pre-rejecting a rounded door-boundary point skips a source-legal
        // forced insertion before it can perform those corrections.
        if( legal && !( cancel && cancel() ) )
        {
            AUTOROUTE_CONTROL control( m_settings, net, retry, plane,
                                       std::max( radius, netViaRadius( net ) ),
                                       hasPads && pureSmd );
            for( std::size_t i = 1; i < found.nodes.size(); ++i )
            {
                const auto& a = found.nodes[i - 1]; const auto& b = found.nodes[i];
                found.cost += a.layer != b.layer ? control.ViaCost()
                        : control.WeightedTraceCost( b.layer, a.point, b.point )
                          + control.CongestionCost( m_occupancy.SegmentUsage( a, b, net ) )
                          + ( a.point != b.point ? m_settings.bendCost : 0 );
            }
            result = std::move( found );
        }
    }
    m_roomMetrics.routed = result.has_value();
    if( autorouterDebugEnabled() )
    {
        std::ostringstream log;
        log << "ROOM_DRILL_SEARCH net=" << net << " rooms=" << m_roomMetrics.rooms
            << " doors=" << m_roomMetrics.doors << " sections=" << m_roomMetrics.sections
            << " pages=" << m_roomMetrics.drillPages << " drills=" << m_roomMetrics.drills
            << " transitions=" << m_roomMetrics.layerTransitions << " expanded=" << expanded
            << " destination_queries=" << m_roomMetrics.destinationQueries
            << " accepted=" << m_roomMetrics.routed << " elapsed_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - started ).count();
        autorouterDebugLog( log.str() );
    }
    return result;
}
} // namespace KICAD_AUTOROUTER
