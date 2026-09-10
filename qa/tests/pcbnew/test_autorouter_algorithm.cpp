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

/*
 * Regression tests for the Freerouting-shaped native autoroute pipeline.  The
 * first group is data-only so the worker boundary stays independently
 * testable; the final cases exercise the KiCad board adapter against a real
 * QA board without opening the editor.
 */

#include <autorouter/AutorouterTypes.h>
#include <autorouter/AutorouterJob.h>
#include <autorouter/board/KicadRoutingSession.h>
#include <board_design_settings.h>
#include <drc/drc_engine.h>
#include <drc/drc_item.h>
#include <zone.h>
#include <pcb_shape.h>
#include <autorouter/board/KicadBoardAdapter.h>
#include <autorouter/drc/DesignRulesChecker.h>
#include <autorouter/maze/MazeSearchEngine.h>
#include <autorouter/maze/AutorouteEngine.h>
#include <autorouter/board/model/items/NormalContacts.h>
#include <autorouter/geometry/planar/ContactGeometry.h>
#include <autorouter/geometry/planar/Simplex.h>
#include <autorouter/board/optimize/TraceShover.h>
#include <autorouter/path/FoundConnectionInserter.h>
#include <autorouter/maze/MazeSearchEngine90Degree.h>
#include <autorouter/maze/MazeExpansionEngine.h>
#include <autorouter/drill/DrillPageArray.h>
#include <autorouter/geometry/planar/PolylineArea.h>
#include <autorouter/maze/MazeListElement.h>
#include <autorouter/path/FoundConnectionLocator45Degree.h>
#include <autorouter/expansion/ExpansionGraph.h>
#include <autorouter/expansion/ExpansionDoor.h>
#include <autorouter/expansion/CompleteFreeSpaceExpansionRoom.h>
#include <autorouter/expansion/SortedOrthogonalRoomNeighbours.h>
#include <autorouter/maze/DestinationDistance.h>
#include <autorouter/maze/LegacyDestinationDistance.h>
#include <autorouter/maze/RoomCostSpace.h>
#include <autorouter/BoardHistory.h>
#include <autorouter/rules/ViaRule.h>
#include <autorouter/board/searchtree/ShapeSearchTree90Degree.h>
#include <autorouter/pipeline/BatchFanout.h>
#include <autorouter/pipeline/BatchOptimizer.h>
#include <autorouter/pipeline/AutorouteUnroutedReport.h>
#include <autorouter/pipeline/RoutingPipeline.h>

#include <board.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <pad.h>
#include <pcb_track.h>
#include <pcbnew_utils/board_file_utils.h>

#include <boost/test/unit_test.hpp>
#include <array>
#include <bit>
#include <cmath>
#include <fstream>
#include <sstream>
#include <random>
#include <queue>


using namespace KICAD_AUTOROUTER;


namespace
{

BOARD_SNAPSHOT makeBoard()
{
    BOARD_SNAPSHOT board;
    board.bounds = { 0, 0, 6000000, 3000000 };

    board.pads.push_back( { 1, { 1000000, 1500000 }, { 0, 1 }, "Default", 0, 100000, 100000,
                            100000 } );
    board.pads.push_back( { 1, { 5000000, 1500000 }, { 0, 1 }, "Default", 0, 100000, 100000,
                            100000 } );

    ROUTING_NET net;
    net.netCode = 1;
    net.name = "N1";
    net.netClass = "Default";
    net.viaDiameter = 300000;
    net.viaDrill = 150000;
    net.padIndices = { 0, 1 };
    net.connections = { { 0, 1 } };
    board.nets.push_back( net );
    return board;
}


AUTOROUTER_SETTINGS makeSettings()
{
    AUTOROUTER_SETTINGS settings;
    settings.layers = { { 0, true, 1, 20 }, { 1, true, 2, 20 } };
    settings.gridStepIU = 500000;
    settings.maxExpandedNodes = 100000;
    settings.maxPasses = 3;
    settings.maxIterations = 2;
    settings.optimizeAfterComplete = true;
    return settings;
}

} // namespace


BOOST_AUTO_TEST_SUITE( NativeAutorouter )


BOOST_AUTO_TEST_CASE( OrthogonalRoomRestraintMatchesPinnedFreerouting )
{
    std::ifstream input( KI_TEST::GetPcbnewTestDataDir()
                         + "/autorouter/room-restraint-a11c0a42.txt" );
    BOOST_REQUIRE( input.good() );
    std::string line;
    int cases = 0;
    while( std::getline( input, line ) )
    {
        BOOST_TEST_CONTEXT( "Reference case " << cases )
        {
            std::istringstream values( line );
            auto readBox = [&]()
            {
                ROUTER_BOX box;
                values >> box.minX >> box.minY >> box.maxX >> box.maxY;
                BOOST_REQUIRE( !values.fail() );
                return box;
            };
            const auto room = readBox();
            const auto contained = readBox();
            const auto obstacle = readBox();
            std::size_t expectedCount;
            values >> expectedCount;
            const auto actual = SHAPE_SEARCH_TREE_90_DEGREE::RestrainShape(
                    { room, 2, contained }, obstacle );
            BOOST_REQUIRE_EQUAL( actual.size(), expectedCount );
            auto checkBox = [&]( const ROUTER_BOX& box )
            {
                const ROUTER_BOX expected = readBox();
                BOOST_CHECK_EQUAL( box.minX, expected.minX );
                BOOST_CHECK_EQUAL( box.minY, expected.minY );
                BOOST_CHECK_EQUAL( box.maxX, expected.maxX );
                BOOST_CHECK_EQUAL( box.maxY, expected.maxY );
            };
            for( const auto& result : actual )
            {
                checkBox( result.GetShape() );
                checkBox( result.GetContainedShape() );
                BOOST_CHECK_EQUAL( result.GetLayer(), 2 );
            }
            values >> std::ws;
            BOOST_CHECK( values.eof() );
        }
        ++cases;
    }
    BOOST_CHECK_EQUAL( cases, 512 );
}


BOOST_AUTO_TEST_CASE( SearchesEveryMemberOfTheDestinationSet )
{
    BOARD_SNAPSHOT board = makeBoard();
    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.allowVias = false;
    settings.maxExpandedNodes = 20;
    // The nominal destination is behind a wall. Another pad in its
    // connected set is reachable without crossing that wall.
    board.obstacles.push_back( { ROUTER_OBSTACLE_KIND::RECTANGLE, 2, {}, {}, {},
                                { 3000000, 0, 3500000, 3000000 } } );
    ROUTING_PAD reachable = board.pads[1];
    reachable.position = { 2000000, 1500000 };
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    MAZE_SEARCH_ENGINE search( board, settings, occupancy );
    int expanded = 0;
    const auto result = search.FindConnection( board.pads[0], board.pads[1], 0, expanded, {}, {},
            { { board.pads[0], 0 } }, { { board.pads[1], 1 }, { reachable, 2 } } );
    BOOST_REQUIRE( result );
    BOOST_CHECK_EQUAL( result->fromPadIndex, 0 );
    BOOST_CHECK_EQUAL( result->toPadIndex, 2 );
    BOOST_CHECK( result->nodes.back().point == reachable.position );
}


BOOST_AUTO_TEST_CASE( SearchesEveryMemberOfTheStartSetWithoutInventingCopper )
{
    BOARD_SNAPSHOT board = makeBoard();
    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.allowVias = false;
    settings.maxExpandedNodes = 20;
    board.obstacles.push_back( { ROUTER_OBSTACLE_KIND::RECTANGLE, 2, {}, {}, {},
                                { 3000000, 0, 3500000, 3000000 } } );
    ROUTING_PAD reachable = board.pads[0];
    reachable.position = { 4000000, 1500000 };
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    MAZE_SEARCH_ENGINE search( board, settings, occupancy );
    int expanded = 0;
    const auto result = search.FindConnection( board.pads[0], board.pads[1], 0, expanded, {}, {},
            { { board.pads[0], 0 }, { reachable, 2 } }, { { board.pads[1], 1 } } );
    BOOST_REQUIRE( result );
    BOOST_CHECK_EQUAL( result->fromPadIndex, 2 );
    BOOST_CHECK_EQUAL( result->toPadIndex, 1 );
    BOOST_CHECK( result->nodes.front().point == reachable.position );
    // Backtracking must not prepend an unchecked segment to the nominal
    // source on the other side of the wall.
    for( const auto& node : result->nodes )
        BOOST_CHECK_GE( node.point.x, 3500000 );
}


BOOST_AUTO_TEST_CASE( LegacyDestinationLowerBoundConsidersCheaperOtherLayer )
{
    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.viaCost = 1;
    settings.planeViaCost = 1;
    ROUTING_PAD far;
    far.position = { 100000000, 0 };
    far.layers = { 0 };
    LEGACY_DESTINATION_DISTANCE destination;
    destination.Configure( settings, far );
    destination.Join( { 0, 0, 0, 0 }, 1 );
    BOOST_CHECK_LE( destination.Calculate( { 0, 0 }, 0 ), 1.0 );
}


BOOST_AUTO_TEST_CASE( RetainedCopperComponentsOfferEveryPadToTheBatchSearch )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.pads.push_back( board.pads[1] );
    board.pads[2].position = { 4000000, 1500000 };
    board.nets[0].padIndices = { 0, 1, 2 };
    board.nets[0].connectedPadGroups = { { 0, 1 } };
    board.nets[0].connections = { { 0, 2 } };
    // Retained copper already joins pads 0 and 1 on another layer. The
    // missing connection can be completed on layer 0 only from pad 1.
    board.obstacles.push_back( { ROUTER_OBSTACLE_KIND::RECTANGLE, 2, { 0 }, {}, {},
                                { 3000000, 0, 3500000, 3000000 } } );
    auto settings = makeSettings();
    settings.layers = { { 0, true, 1, 20 } };
    settings.allowVias = false;
    settings.enableFanout = false;
    settings.optimizeAfterComplete = false;
    settings.maxExpandedNodes = 20;
    const auto result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_REQUIRE_EQUAL( result.connections.size(), 1 );
    BOOST_CHECK_EQUAL( result.connections[0].fromPadIndex, 2 );
    BOOST_CHECK_EQUAL( result.connections[0].toPadIndex, 1 );
    for( const auto& segment : result.segments )
    {
        BOOST_CHECK_GT( segment.start.x, 3500000 );
        BOOST_CHECK_GT( segment.end.x, 3500000 );
    }
}


BOOST_AUTO_TEST_CASE( DuplicateRoutesCannotHideMissingConnectivityOrARippedBridge )
{
    ROUTING_NET net;
    net.netCode = 1;
    net.padIndices = { 0, 1, 2 };
    net.connections = { { 0, 1 }, { 0, 2 } };
    ROUTING_CONNECTION first;
    first.netCode = 1;
    first.complete = true;
    first.fromPadIndex = 0;
    first.toPadIndex = 1;
    first.nodes = { { { 0, 0 }, 0 }, { { 1000, 0 }, 0 } };
    ROUTING_CONNECTION bridge = first;
    bridge.fromPadIndex = 1;
    bridge.toPadIndex = 2;
    bridge.nodes = { { { 1000, 0 }, 0 }, { { 2000, 0 }, 0 } };
    std::vector<ROUTING_CONNECTION> routes{ first, first, first };
    BOOST_CHECK_EQUAL( AUTOROUTE_UNROUTED_REPORT::CountMissing( net, routes ), 1 );
    routes.push_back( bridge );
    BOOST_CHECK_EQUAL( AUTOROUTE_UNROUTED_REPORT::CountMissing( net, routes ), 0 );
    routes.pop_back();
    // A stale logical completion has no copper and cannot replace the bridge.
    bridge.nodes.resize( 1 );
    routes.push_back( bridge );
    BOOST_CHECK_EQUAL( AUTOROUTE_UNROUTED_REPORT::CountMissing( net, routes ), 1 );
    BOOST_REQUIRE_EQUAL( AUTOROUTE_UNROUTED_REPORT::Build( { net }, routes ).size(), 1 );
}


BOOST_AUTO_TEST_CASE( FanoutShortcutCannotOverrideAnExplicitTerminalSet )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.maxExpandedNodes = 20;
    ROUTING_PAD landing = board.pads[1];
    landing.isFanoutTarget = true;
    landing.fanoutSourceLayer = 0;
    landing.fanoutTargetLayer = 1;
    ROUTING_PAD actualTarget = board.pads[1];
    actualTarget.position = { 2000000, 1500000 };
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    MAZE_SEARCH_ENGINE search( board, settings, occupancy );
    int expanded = 0;
    const auto result = search.FindConnection( board.pads[0], landing, 0, expanded, {}, {},
            { { board.pads[0], 0 } }, { { actualTarget, 2 } } );
    BOOST_REQUIRE( result );
    BOOST_CHECK_EQUAL( result->toPadIndex, 2 );
    BOOST_CHECK( result->nodes.back().point == actualTarget.position );
}


BOOST_AUTO_TEST_CASE( ObliqueTraceTerminalFinishesOnAnExactIntegralLatticePoint )
{
    BOARD_SNAPSHOT board;
    board.bounds = { 0, 0, 10000000, 7000000 };
    board.pads.push_back( { 1, { 5000000, 5500000 }, { 0 }, "Default", 0, 50000, 0,
                            100000 } );
    board.pads.push_back( { 1, { 4000000, 1000000 }, { 0 }, "Default", 0, 50000, 0,
                            100000 } );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.allowVias = false;
    settings.gridStepIU = 500000;
    settings.maxExpandedNodes = 1000;

    ROUTING_TERMINAL source{ board.pads[0], 0 };
    ROUTING_TERMINAL target{ board.pads[1], 1, ROUTER_POINT{ 7000000, 3000000 } };
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    MAZE_SEARCH_ENGINE search( board, settings, occupancy );
    int expanded = 0;
    const auto result = search.FindConnection( board.pads[0], board.pads[1], 0, expanded,
                                                {}, {}, { source }, { target } );
    BOOST_REQUIRE( result );
    BOOST_REQUIRE( !result->nodes.empty() );
    BOOST_CHECK( CONTACT_GEOMETRY::OnSegment( target.pad.position, *target.segmentEnd,
                                               result->nodes.back().point ) );
}


BOOST_AUTO_TEST_CASE( FanoutTerminatesAtTheFirstRoomFrontierDrill )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.bounds = { 0, 0, 7000000, 3000000 };
    board.pads[0].position = { 1000000, 1500000 };
    board.pads[0].layers = { 0 };
    board.pads[0].isSmd = true;
    board.pads[1].position = { 6000000, 1500000 };
    board.pads[1].layers = { 0 };

    ROUTING_PAD fanoutTarget = board.pads[1];
    fanoutTarget.position = { 2500000, 1500000 };
    fanoutTarget.layers = { 1 };
    fanoutTarget.isFanoutTarget = true;
    fanoutTarget.fanoutSourceLayer = 0;
    fanoutTarget.fanoutTargetLayer = 1;
    fanoutTarget.fanoutSourcePadIndex = 0;
    fanoutTarget.fanoutMinEscapeLength = 1000000;
    fanoutTarget.fanoutMaxEscapeLength = 2000000;

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.gridStepIU = 250000;
    settings.maxExpandedNodes = 10000;
    settings.allowViaInSmdPad = false;

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE search( board, settings, occupancy );
    int expanded = 0;
    const auto result = search.FindConnection(
            board.pads[0], fanoutTarget, 0, expanded, {}, {},
            { ROUTING_TERMINAL{ board.pads[0], 0 } },
            { ROUTING_TERMINAL{ board.pads[1], 1 } } );

    BOOST_REQUIRE( result );
    BOOST_REQUIRE( result->isFanoutConnection );
    BOOST_REQUIRE_GE( result->nodes.size(), 2U );
    BOOST_CHECK_EQUAL( result->nodes.front().layer, 0 );
    BOOST_CHECK_NE( result->nodes.back().layer, 0 );
    BOOST_CHECK( result->nodes[result->nodes.size() - 2].point
                 == result->nodes.back().point );
    const long double dx = static_cast<long double>( result->nodes.back().point.x )
                           - board.pads[0].position.x;
    const long double dy = static_cast<long double>( result->nodes.back().point.y )
                           - board.pads[0].position.y;
    const long double drillDistance = std::hypotl( dx, dy );
    BOOST_CHECK_GE( drillDistance, 1000000.0L );
    BOOST_CHECK_LE( drillDistance, 2000000.0L );
    BOOST_CHECK( search.LastRoomSearchMetrics().routed );
    BOOST_CHECK_GT( search.LastRoomSearchMetrics().drills, 0 );
}


BOOST_AUTO_TEST_CASE( HistoryNeverTradesClearanceForCompletion )
{
    BOARD_HISTORY history( 1 );
    ROUTING_RESULT clean;
    clean.metrics.unroutedConnections = 20;
    clean.metrics.routedConnections = 1;
    ROUTING_RESULT invalid;
    invalid.metrics.routedConnections = 1000000;
    invalid.metrics.drcViolations = 1;
    history.Add( invalid );
    history.Add( clean );
    BOOST_REQUIRE( history.Best() );
    BOOST_CHECK_EQUAL( history.Best()->metrics.drcViolations, 0 );
    history.Add( invalid );
    BOOST_CHECK_EQUAL( history.Best()->metrics.drcViolations, 0 );
}


BOOST_AUTO_TEST_CASE( PadEndpointDoesNotExemptForeignCopper )
{
    BOARD_SNAPSHOT board = makeBoard();
    ROUTING_OBSTACLE obstacle;
    obstacle.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    obstacle.netCode = 2;
    obstacle.box = { 900000, 1400000, 5100000, 1600000 };
    board.obstacles.push_back( obstacle );
    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.allowVias = false;
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    MAZE_SEARCH_ENGINE search( board, settings, occupancy );
    BOOST_CHECK( !search.CanUseSegment( 1, { board.pads[0].position, 0 },
                                             { board.pads[1].position, 0 } ) );
    ROUTING_RESULT result;
    result.segments.push_back( { 1, 0, board.pads[0].position, board.pads[1].position, 100000 } );
    BOOST_CHECK_GT( DESIGN_RULES_CHECKER::CountViolations( board, settings, result ), 0 );
}


BOOST_AUTO_TEST_CASE( NewViaCannotReuseAnExistingDrill )
{
    BOARD_SNAPSHOT board = makeBoard();
    ROUTING_OBSTACLE hole;
    hole.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    hole.netCode = 1;
    hole.start = hole.end = board.pads[0].position;
    hole.radius = 100000;
    hole.isHole = true;
    hole.isExistingRoute = true;
    hole.boardItemId = "existing-via";
    board.obstacles.push_back( hole );
    AUTOROUTER_SETTINGS settings = makeSettings();
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    MAZE_SEARCH_ENGINE search( board, settings, occupancy );
    BOOST_CHECK( !search.CanUseSegment( 1, { hole.start, 0 }, { hole.start, 1 }, true ) );
    ROUTING_RESULT result;
    result.vias.push_back( { 1, hole.start, 0, 1, 300000, 150000, { 0, 1 } } );
    BOOST_CHECK_GT( DESIGN_RULES_CHECKER::CountViolations( board, settings, result ), 0 );
}


BOOST_AUTO_TEST_CASE( SameNetProposedViasStillRequireDrillSpacing )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.holeToHoleClearance = 250000;
    AUTOROUTER_SETTINGS settings = makeSettings();
    ROUTING_RESULT result;
    result.vias.push_back( { 1, { 2500000, 1500000 }, 0, 1, 300000, 150000, { 0, 1 } } );
    result.vias.push_back( { 1, { 2800000, 1500000 }, 0, 1, 300000, 150000, { 0, 1 } } );
    BOOST_CHECK_GT( DESIGN_RULES_CHECKER::CountViolations( board, settings, result ), 0 );
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    ROUTING_CONNECTION firstVia;
    firstVia.netCode = 1;
    firstVia.complete = true;
    firstVia.nodes = { { result.vias[0].position, 0 }, { result.vias[0].position, 1 } };
    occupancy.Add( firstVia );
    MAZE_SEARCH_ENGINE search( board, settings, occupancy );
    BOOST_CHECK( !search.CanUseSegment( 1, { result.vias[1].position, 0 },
                                             { result.vias[1].position, 1 }, true ) );
}


BOOST_AUTO_TEST_CASE( RoutesSimpleConnectionDeterministically )
{
    const BOARD_SNAPSHOT board = makeBoard();
    const AUTOROUTER_SETTINGS settings = makeSettings();
    ROUTING_PIPELINE pipeline;

    const ROUTING_RESULT first = pipeline.Run( board, settings, {}, {} );
    const ROUTING_RESULT second = pipeline.Run( board, settings, {}, {} );

    BOOST_REQUIRE( first.complete );
    BOOST_REQUIRE( second.complete );
    BOOST_CHECK_EQUAL( first.metrics.routedConnections, 1 );
    BOOST_CHECK_EQUAL( first.segments.size(), second.segments.size() );
    BOOST_CHECK_EQUAL( first.vias.size(), second.vias.size() );
    BOOST_CHECK_EQUAL( first.segments.front().start.x, second.segments.front().start.x );
    BOOST_CHECK_EQUAL( first.segments.front().end.x, second.segments.front().end.x );
}


BOOST_AUTO_TEST_CASE( RipupAndViaSearchRespectLayerObstacles )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.obstacles.push_back( { ROUTER_OBSTACLE_KIND::RECTANGLE,
                                 0,
                                 { 0 },
                                 {},
                                 {},
                                 { 2500000, 500000, 3500000, 2500000 },
                                 {},
                                 0,
                                 true,
                                 true,
                                 false,
                                 {} } );
    const AUTOROUTER_SETTINGS settings = makeSettings();
    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( board, settings, {}, {} );

    BOOST_REQUIRE( result.complete );
    BOOST_CHECK( !result.segments.empty() );
}


BOOST_AUTO_TEST_CASE( LayerlessObstaclesApplyToEveryRoutingLayer )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.obstacles.push_back( { ROUTER_OBSTACLE_KIND::RECTANGLE,
                                 0,
                                 {},
                                 {},
                                 {},
                                 { 2500000, 0, 3500000, 3000000 },
                                 {},
                                 0,
                                 true,
                                 true,
                                 false,
                                 {} } );

    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, makeSettings(), {}, {} );

    BOOST_CHECK( !result.complete );
    BOOST_CHECK_EQUAL( result.metrics.routedConnections, 0 );
    BOOST_CHECK_EQUAL( result.metrics.unroutedConnections, 1 );
}


BOOST_AUTO_TEST_CASE( CancellationNeverProducesACommitResult )
{
    ROUTING_PIPELINE pipeline;
    const BOARD_SNAPSHOT board = makeBoard();
    const AUTOROUTER_SETTINGS settings = makeSettings();
    const ROUTING_RESULT result = pipeline.Run( board, settings, [] { return true; }, {} );

    BOOST_CHECK( result.cancelled );
    BOOST_CHECK( !result.complete );
    BOOST_CHECK( result.segments.empty() );
    BOOST_CHECK( result.vias.empty() );
}


BOOST_AUTO_TEST_CASE( FinalRuleCheckFindsForeignNetCrossing )
{
    BOARD_SNAPSHOT board = makeBoard();
    ROUTING_NET secondNet;
    secondNet.netCode = 2;
    board.nets.push_back( secondNet );

    ROUTING_RESULT result;
    result.segments.push_back( { 1, 0, { 1000000, 1500000 }, { 5000000, 1500000 }, 100000 } );
    result.segments.push_back( { 2, 0, { 3000000, 500000 }, { 3000000, 2500000 }, 100000 } );

    BOOST_CHECK_GT( DESIGN_RULES_CHECKER::CountViolations( board, makeSettings(), result ), 0 );
}


BOOST_AUTO_TEST_CASE( PairClearanceIsAppliedOnce )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.nets.front().clearance = 200000;

    board.pads.push_back( { 2, { 1000000, 1000000 }, { 0 }, "Default", 0, 100000, 0,
                            100000 } );
    ROUTING_NET secondNet;
    secondNet.netCode = 2;
    secondNet.clearance = 200000;
    secondNet.padIndices = { 2 };
    board.nets.push_back( secondNet );

    ROUTING_RESULT result;
    // The center-to-center spacing is 350000 IU.  The two 100000 IU tracks
    // need 100000 IU of copper separation plus one 200000 IU pair clearance.
    // Applying both netclass values would incorrectly reject this geometry.
    result.segments.push_back( { 1, 0, { 1000000, 1500000 }, { 5000000, 1500000 }, 100000 } );
    result.segments.push_back( { 2, 0, { 1000000, 1150000 }, { 5000000, 1150000 }, 100000 } );

    BOOST_CHECK_EQUAL( DESIGN_RULES_CHECKER::CountViolations( board, makeSettings(), result ), 0 );
}


BOOST_AUTO_TEST_CASE( PerEdgeClearanceIsPreservedByInsertionAndProposalDrc )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.pads.push_back( { 2, { 1000000, 1000000 }, { 0 }, "Default", 0, 100000, 0,
                            100000 } );
    ROUTING_NET foreign;
    foreign.netCode = 2;
    foreign.padIndices = { 2 };
    board.nets.push_back( foreign );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.allowVias = false;
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    ROUTING_CONNECTION committed;
    committed.netCode = 2;
    committed.complete = true;
    committed.nodes = { { { 1000000, 1000000 }, 0 },
                        { { 5000000, 1000000 }, 0 } };
    occupancy.Add( committed );

    MAZE_SEARCH_ENGINE search( board, settings, occupancy );
    const ROUTER_NODE start{ { 1000000, 1350000 }, 0 };
    const ROUTER_NODE end{ { 5000000, 1350000 }, 0 };
    ROUTING_EDGE_STYLE styled;
    styled.trackWidth = 100000;
    styled.clearance = 300000;

    // Copper alone needs 100000 IU centre spacing, so the ordinary net style
    // may pass. The source clearance-class value on this one edge requires
    // 400000 IU and must be carried through the strict insertion predicate.
    BOOST_CHECK( search.CanInsertSegment( 1, start, end ) );
    BOOST_CHECK( !search.CanInsertSegment( 1, start, end, &styled ) );

    ROUTING_CONNECTION route;
    route.netCode = 1;
    route.complete = true;
    route.nodes = { start, end };
    route.edgeStyles = { styled };
    ROUTING_RESULT emitted;
    FOUND_CONNECTION_INSERTER::Append( route, 100000, 300000, 150000, { 0, 1 }, emitted );
    BOOST_REQUIRE_EQUAL( emitted.segments.size(), 1U );
    BOOST_CHECK_EQUAL( emitted.segments.front().clearance, styled.clearance );

    emitted.segments.push_back( { 2, 0, { 1000000, 1000000 }, { 5000000, 1000000 },
                                  100000 } );
    BOOST_CHECK_GT( DESIGN_RULES_CHECKER::CountViolations( board, settings, emitted ), 0 );
}


BOOST_AUTO_TEST_CASE( ForeignObstacleLocalClearanceIsApplied )
{
    BOARD_SNAPSHOT board = makeBoard();
    ROUTING_NET secondNet;
    secondNet.netCode = 2;
    board.nets.push_back( secondNet );

    ROUTING_OBSTACLE obstacle;
    obstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    obstacle.netCode = 2;
    obstacle.layers = { 0 };
    obstacle.start = { 1000000, 1000000 };
    obstacle.end = { 5000000, 1000000 };
    obstacle.blocksTracks = true;
    obstacle.blocksVias = true;
    obstacle.clearance = 500000;
    board.obstacles.push_back( obstacle );

    ROUTING_RESULT result;
    result.segments.push_back( { 1, 0, { 1000000, 1500000 }, { 5000000, 1500000 },
                                 100000 } );

    // Netclass pair clearance is zero, but the foreign obstacle has a local
    // 0.5 mm clearance.  The worker and its final rule checker must retain
    // that object-specific rule instead of reducing the pair to netclasses.
    BOOST_CHECK_GT( DESIGN_RULES_CHECKER::CountViolations( board, makeSettings(), result ), 0 );
}


BOOST_AUTO_TEST_CASE( ViaDrillClearanceIsCheckedAgainstSameNetExistingHoles )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.holeClearance = 100000;
    board.holeToHoleClearance = 250000;

    ROUTING_OBSTACLE existingHole;
    existingHole.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    existingHole.netCode = 1;
    existingHole.layers = { 0, 1 };
    existingHole.start = { 3000000, 1500000 };
    existingHole.end = existingHole.start;
    existingHole.radius = 200000;
    existingHole.blocksTracks = true;
    existingHole.blocksVias = true;
    existingHole.isExistingRoute = true;
    existingHole.isHole = true;
    board.obstacles.push_back( existingHole );

    ROUTING_RESULT result;
    result.vias.push_back( { 1,
                             { 3000000, 1500000 },
                             0,
                             1,
                             600000,
                             150000,
                             { 0, 1 } } );

    // Same-net copper may overlap electrically, but two drills must still
    // obey KiCad's manufacturing hole-to-hole rule.
    BOOST_CHECK_GT( DESIGN_RULES_CHECKER::CountViolations( board, makeSettings(), result ), 0 );

    ROUTING_RESULT trackResult;
    trackResult.segments.push_back( { 1, 0, { 1000000, 1500000 }, { 5000000, 1500000 },
                                      100000 } );
    // A same-net trace may terminate at its own plated pad, but it may not
    // pass through an unrelated same-net drill in the middle of the route.
    BOOST_CHECK_GT( DESIGN_RULES_CHECKER::CountViolations( board, makeSettings(), trackResult ),
                    0 );
}


BOOST_AUTO_TEST_CASE( RoutesSinglePadToPlaneTarget )
{
    BOARD_SNAPSHOT board;
    board.bounds = { 0, 0, 6000000, 3000000 };
    board.pads.push_back( { 1, { 1000000, 1500000 }, { 0 }, "Default", 0, 100000, 0,
                            100000 } );
    board.pads.push_back( { 1, { 5000000, 1500000 }, { 0 }, "Default", 0, 0, 0, 100000,
                            true } );

    ROUTING_NET net;
    net.netCode = 1;
    net.name = "GND";
    net.netClass = "Default";
    net.padIndices = { 0 };
    net.planeTargetIndices = { 1 };
    net.connections = { { 0, 1 } };
    board.nets.push_back( net );
    ROUTING_OBSTACLE plane;
    plane.netCode = 1;
    plane.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    plane.box = { 4500000, 1000000, 5500000, 2000000 };
    plane.layers = board.pads[1].layers;
    board.conductionAreas.push_back( plane );

    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( board, makeSettings(), {}, {} );

    BOOST_REQUIRE( result.complete );
    BOOST_CHECK_EQUAL( result.metrics.totalConnections, 1 );
    BOOST_CHECK_EQUAL( result.metrics.routedConnections, 1 );
}


BOOST_AUTO_TEST_CASE( PlaneSmdPadsFanoutBeforeChangingLayers )
{
    BOARD_SNAPSHOT board;
    board.bounds = { 0, 0, 6000000, 3000000 };
    board.pads.push_back( { 1, { 1000000, 1500000 }, { 0 }, "Default", 0, 100000, 0,
                            100000, false, true } );
    board.pads.push_back( { 1, { 5000000, 1500000 }, { 1 }, "Default", 0, 0, 0, 100000,
                            true } );

    ROUTING_NET net;
    net.netCode = 1;
    net.name = "GND";
    net.netClass = "Default";
    // Data-only snapshots do not inherit KiCad's resolved netclass via
    // rule. Model the real adapter input explicitly: a fanout needs either
    // its net rule or an explicitly captured board-via fallback.
    net.viaDiameter = 600000;
    net.viaDrill = 300000;
    net.padIndices = { 0 };
    net.planeTargetIndices = { 1 };
    net.connections = { { 0, 1 } };
    board.nets.push_back( net );
    ROUTING_OBSTACLE plane;
    plane.netCode = 1;
    plane.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    plane.box = { 4500000, 1000000, 5500000, 2000000 };
    plane.layers = board.pads[1].layers;
    board.conductionAreas.push_back( plane );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.allowViaInSmdPad = false;

    const BOARD_SNAPSHOT fanned = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_REQUIRE_EQUAL( fanned.nets.size(), 1 );
    BOOST_CHECK_EQUAL( fanned.nets.front().connections.size(), 2 );

    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( board, settings, {}, {} );

    BOOST_REQUIRE( result.complete );
    BOOST_CHECK_EQUAL( result.metrics.routedConnections, 2 );
    BOOST_CHECK_EQUAL( result.metrics.fanoutConnections, 1 );
    BOOST_REQUIRE_EQUAL( result.vias.size(), 1 );
    BOOST_CHECK( result.vias.front().position != board.pads[0].position );
}


BOOST_AUTO_TEST_CASE( PlaneTargetOnPadLayerDoesNotCreateAnUnusedVia )
{
    BOARD_SNAPSHOT board;
    board.bounds = { 0, 0, 6000000, 3000000 };
    board.pads.push_back( { 1, { 1000000, 1500000 }, { 0 }, "Default", 0, 100000, 0,
                            100000, false, true } );
    board.pads.push_back( { 1, { 5000000, 1500000 }, { 0 }, "Default", 0, 0, 0, 100000,
                            true } );

    ROUTING_NET net;
    net.netCode = 1;
    net.name = "GND";
    net.netClass = "Default";
    net.padIndices = { 0 };
    net.planeTargetIndices = { 1 };
    net.connections = { { 0, 1 } };
    board.nets.push_back( net );
    ROUTING_OBSTACLE plane;
    plane.netCode = 1;
    plane.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    plane.box = { 4500000, 1000000, 5500000, 2000000 };
    plane.layers = board.pads[1].layers;
    board.conductionAreas.push_back( plane );

    const BOARD_SNAPSHOT fanned = BATCH_FANOUT::PrepareSnapshot( board, makeSettings() );
    BOOST_REQUIRE_EQUAL( fanned.nets.size(), 1 );
    BOOST_CHECK_EQUAL( fanned.pads.size(), 2 );
    BOOST_CHECK_EQUAL( fanned.nets.front().connections.size(), 1 );

    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, makeSettings(), {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_CHECK_EQUAL( result.vias.size(), 0 );
}


BOOST_AUTO_TEST_CASE( SmdPadsUseTheFreeroutingStyleFanoutStage )
{
    BOARD_SNAPSHOT board;
    board.bounds = { 0, 0, 6000000, 3000000 };
    board.pads.push_back( { 1, { 1000000, 1500000 }, { 0 }, "Default", 0, 100000, 0,
                            100000, false, true } );
    board.pads.push_back( { 1, { 5000000, 1500000 }, { 0 }, "Default", 0, 100000, 0,
                            100000, false, true } );

    ROUTING_NET net;
    net.netCode = 1;
    net.name = "SMD_NET";
    net.netClass = "Default";
    net.viaDiameter = 300000;
    net.viaDrill = 150000;
    net.padIndices = { 0, 1 };
    net.connections = { { 0, 1 } };
    board.nets.push_back( net );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.enableFanout = true;
    settings.maxFanoutPasses = 1;

    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( board, settings, {}, {} );

    BOOST_REQUIRE( result.complete );
    // The first fanout item reaches the other SMD pin directly.  Freerouting
    // then reports the second pin as having no unconnected net items instead
    // of creating either synthetic escape edge.
    BOOST_CHECK_EQUAL( result.metrics.routedConnections, 1 );
    const auto prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_CHECK_EQUAL( prepared.pads.size() - board.pads.size(), 2 );
    BOOST_CHECK_EQUAL( result.metrics.fanoutConnections, 1 );
    BOOST_CHECK_EQUAL( result.metrics.fanoutConnections,
            std::count_if( result.connections.begin(), result.connections.end(),
                           []( const auto& c ) { return c.isFanoutConnection; } ) );
    // Both layer transitions are bypassed; no dangling vias may be exported.
    BOOST_CHECK_EQUAL( result.vias.size(), 0 );
    ROUTING_BOARD copper( board, settings );
    for( const auto& route : result.connections )
        copper.AddRoute( route );
    BOOST_CHECK( copper.Connected( 0, 1 ) );
    for( const auto& route : result.connections )
    {
        BOOST_REQUIRE_GT( route.nodes.size(), 1 );
        copper.RemoveRoute( route );
        BOOST_CHECK( copper.HasCopperAt( 1, route.nodes.front(), 50000 ) );
        BOOST_CHECK( copper.HasCopperAt( 1, route.nodes.back(), 50000 ) );
        copper.AddRoute( route );
    }
    for( const ROUTING_VIA& via : result.vias )
    {
        BOOST_CHECK( via.position != board.pads[0].position );
        BOOST_CHECK( via.position != board.pads[1].position );
    }
}


BOOST_AUTO_TEST_CASE( FanoutTriesTheClosestOfAtMostFourItemsFirst )
{
    BOARD_SNAPSHOT board;
    board.bounds = { 0, 0, 10000000, 3000000 };
    board.pads.push_back( { 1, { 1000000, 1500000 }, { 0 }, "Default", 0, 100000, 0,
                            100000, false, true } );
    board.pads.push_back( { 1, { 4000000, 1500000 }, { 0 }, "Default", 0, 100000, 0,
                            100000 } );
    board.pads.push_back( { 1, { 8500000, 1500000 }, { 0 }, "Default", 0, 100000, 0,
                            100000 } );

    ROUTING_NET net;
    net.netCode = 1;
    net.name = "SMALL_FANOUT_NET";
    net.netClass = "Default";
    net.viaDiameter = 300000;
    net.viaDrill = 150000;
    net.padIndices = { 0, 1, 2 };
    net.connections = { { 0, 1 }, { 1, 2 } };
    board.nets.push_back( net );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.maxFanoutPasses = 1;
    settings.optimizationPasses = 0;
    settings.fanoutMinEscapeLengthIU = 2500000;
    settings.fanoutMaxEscapeLengthIU = 4500000;

    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );

    const auto fanout = std::find_if(
            result.connections.begin(), result.connections.end(),
            []( const ROUTING_CONNECTION& aConnection )
            { return aConnection.isFanoutConnection && aConnection.fromPadIndex == 0; } );
    BOOST_REQUIRE( fanout != result.connections.end() );
    BOOST_CHECK_EQUAL( fanout->toPadIndex, 1U );
}


BOOST_AUTO_TEST_CASE( FanoutUsesBoardViaFallbackAndHonorsItsEscapeEnvelope )
{
    // The real KiCad adapter captures the resolved netclass via rule.  This
    // deliberately omits it so the test exercises Freerouting's independent
    // fallback-to-board-vias policy rather than the ordinary net rule.
    BOARD_SNAPSHOT board = makeBoard();
    board.bounds = { 0, 0, 8000000, 3000000 };
    board.pads[0].layers = { 0 };
    board.pads[0].isSmd = true;
    board.pads[1].position = { 7000000, 1500000 };
    board.pads[1].layers = { 1 };
    board.nets[0].viaDiameter = 0;
    board.nets[0].viaDrill = 0;
    board.boardViaDimensions = { { 400000, 200000 } };

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.maxFanoutPasses = 1;
    settings.fanoutMinEscapeLengthIU = 1000000;
    settings.fanoutMaxEscapeLengthIU = 1000000;

    const BOARD_SNAPSHOT prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_REQUIRE_EQUAL( prepared.pads.size(), 3U );
    const ROUTING_PAD& landing = prepared.pads.back();
    BOOST_REQUIRE( landing.isFanoutTarget );
    BOOST_CHECK_EQUAL( landing.fanoutViaDiameter, 400000 );
    BOOST_CHECK_EQUAL( landing.fanoutViaDrill, 200000 );
    BOOST_CHECK_EQUAL( landing.fanoutMinEscapeLength, 1000000 );
    BOOST_CHECK_EQUAL( landing.fanoutMaxEscapeLength, 1000000 );
    BOOST_CHECK_EQUAL( landing.position.y, board.pads[0].position.y );
    BOOST_CHECK_EQUAL( std::llabs( landing.position.x - board.pads[0].position.x ), 1000000 );

    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_REQUIRE_EQUAL( result.vias.size(), 1U );
    BOOST_CHECK_EQUAL( result.vias.front().diameter, 400000 );
    BOOST_CHECK_EQUAL( result.vias.front().drill, 200000 );
    const ROUTER_POINT viaDelta{ result.vias.front().position.x - board.pads[0].position.x,
                                 result.vias.front().position.y - board.pads[0].position.y };
    BOOST_CHECK_EQUAL( viaDelta.x * viaDelta.x + viaDelta.y * viaDelta.y,
                       1000000LL * 1000000LL );

    // Falling back is opt-in: a net without a via rule must retain its
    // ordinary graph when board-via fallback is disabled.
    AUTOROUTER_SETTINGS noFallback = settings;
    noFallback.fanoutFallbackToBoardVias = false;
    BOOST_CHECK_EQUAL( BATCH_FANOUT::PrepareSnapshot( board, noFallback ).pads.size(),
                       board.pads.size() );

    // Java applies max-start-room and min-drill checks separately. An
    // inverted interval has no legal escape; the native planner must not
    // silently widen the requested maximum to its minimum.
    AUTOROUTER_SETTINGS impossibleEnvelope = settings;
    impossibleEnvelope.fanoutMinEscapeLengthIU = 2000000;
    impossibleEnvelope.fanoutMaxEscapeLengthIU = 1000000;
    BOOST_CHECK_EQUAL( BATCH_FANOUT::PrepareSnapshot( board, impossibleEnvelope ).pads.size(),
                       board.pads.size() );
}


BOOST_AUTO_TEST_CASE( FanoutPreservesSelectedMicroviaSpanAndType )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.bounds = { 0, 0, 8000000, 3000000 };
    board.pads[0].layers = { 0 };
    board.pads[0].isSmd = true;
    board.pads[1].position = { 7000000, 1500000 };
    board.pads[1].layers = { 1 };
    board.nets[0].viaProfiles = {
        { 300000, 100000, { 0, 1 }, false, ROUTER_VIA_TYPE::MICROVIA }
    };
    // A later board-rule fallback remains available but must not replace the
    // first legal net ViaRule entry or widen its physical span.
    board.boardViaDimensions = { { 500000, 200000 } };

    // The deterministic first landing is (2 mm, 1.5 mm).  Copper on an
    // unrelated deeper layer blocks a through via there, but is irrelevant
    // to the selected F.Cu-In1.Cu microvia.
    ROUTING_OBSTACLE deeperLayerBlocker;
    deeperLayerBlocker.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    deeperLayerBlocker.netCode = 2;
    deeperLayerBlocker.layers = { 3 };
    deeperLayerBlocker.start = { 2000000, 1500000 };
    deeperLayerBlocker.end = deeperLayerBlocker.start;
    deeperLayerBlocker.radius = 200000;
    deeperLayerBlocker.blocksTracks = true;
    deeperLayerBlocker.blocksVias = true;
    board.obstacles.push_back( deeperLayerBlocker );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.layers = { { 0, true, 1, 20, 0 }, { 1, true, 2, 20, 1 },
                        { 2, true, 1, 20, 2 }, { 3, true, 2, 20, 3 } };
    settings.maxFanoutPasses = 1;
    settings.fanoutMinEscapeLengthIU = 1000000;
    settings.fanoutMaxEscapeLengthIU = 1000000;

    const BOARD_SNAPSHOT prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_REQUIRE_EQUAL( prepared.pads.size(), 3U );
    const ROUTING_PAD& landing = prepared.pads.back();
    BOOST_REQUIRE( landing.isFanoutTarget );
    BOOST_CHECK( landing.fanoutViaLayers == std::vector<int>( { 0, 1 } ) );
    BOOST_CHECK( landing.fanoutViaType == ROUTER_VIA_TYPE::MICROVIA );

    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_REQUIRE_EQUAL( result.vias.size(), 1U );
    BOOST_CHECK( result.vias.front().layers == std::vector<int>( { 0, 1 } ) );
    BOOST_CHECK( result.vias.front().type == ROUTER_VIA_TYPE::MICROVIA );
}


BOOST_AUTO_TEST_CASE( FanoutNormalizesNonCardinalEscapeDirections )
{
    // A real fanout maze evaluates physical distances in all legal outgoing
    // directions.  Block the four cardinal exits but leave a 45-degree
    // channel.  The synthetic planner must treat its diagonal probe as a
    // one-millimetre physical escape, not as a sqrt(2)-millimetre vector that
    // falls outside the configured envelope before it is checked.
    BOARD_SNAPSHOT board = makeBoard();
    board.bounds = { 0, 0, 10000000, 10000000 };
    board.pads[0].position = { 5000000, 5000000 };
    board.pads[0].layers = { 0 };
    board.pads[0].isSmd = true;
    board.pads[0].radius = 100000;
    board.pads[0].clearance = 0;
    board.pads[0].trackWidth = 100000;
    board.pads[1].position = { 8000000, 5000000 };
    board.pads[1].layers = { 1 };

    for( const ROUTER_POINT& offset : { ROUTER_POINT{ 1000000, 0 },
                                        ROUTER_POINT{ -1000000, 0 },
                                        ROUTER_POINT{ 0, 1000000 },
                                        ROUTER_POINT{ 0, -1000000 } } )
    {
        ROUTING_OBSTACLE blocker;
        blocker.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
        blocker.netCode = 2;
        blocker.layers = { 0, 1 };
        blocker.start = blocker.end = { board.pads[0].position.x + offset.x,
                                        board.pads[0].position.y + offset.y };
        blocker.radius = 100000;
        blocker.blocksTracks = true;
        blocker.blocksVias = true;
        board.obstacles.push_back( blocker );
    }

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.maxFanoutPasses = 1;
    settings.fanoutMinEscapeLengthIU = 1000000;
    settings.fanoutMaxEscapeLengthIU = 1200000;

    const BOARD_SNAPSHOT prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_REQUIRE_EQUAL( prepared.pads.size(), 3U );
    const ROUTING_PAD& landing = prepared.pads.back();
    BOOST_REQUIRE( landing.isFanoutTarget );
    const std::int64_t dx = landing.position.x - board.pads[0].position.x;
    const std::int64_t dy = landing.position.y - board.pads[0].position.y;
    BOOST_CHECK_NE( dx, 0 );
    BOOST_CHECK_NE( dy, 0 );
    const long double escape = std::sqrt( static_cast<long double>( dx ) * dx
                                          + static_cast<long double>( dy ) * dy );
    BOOST_CHECK_GE( escape, 1000000.0L );
    BOOST_CHECK_LE( escape, 1200000.0L );
}


BOOST_AUTO_TEST_CASE( FanoutRefinesBeyondItsLegacyDirectionSet )
{
    // RoutingBoard.fanout() searches maze doors, rather than being limited to
    // the native adapter's original cardinal, diagonal and 1:2 probes. Block
    // all sixteen legacy directions and leave the bounded angular refinement
    // clear. Before the refinement this board had no synthetic landing.
    BOARD_SNAPSHOT board = makeBoard();
    board.bounds = { 0, 0, 10000000, 10000000 };
    board.pads[0].position = { 5000000, 5000000 };
    board.pads[0].layers = { 0 };
    board.pads[0].isSmd = true;
    board.pads[0].radius = 1;
    board.pads[0].clearance = 0;
    board.pads[0].trackWidth = 2;
    board.pads[1].position = { 8000000, 5000000 };
    board.pads[1].layers = { 1 };
    board.nets[0].viaDiameter = 2;
    board.nets[0].viaDrill = 1;

    const std::array<std::array<int, 2>, 16> legacyDirections = {
            std::array<int, 2>{ 1, 0 },   std::array<int, 2>{ -1, 0 },
            std::array<int, 2>{ 0, 1 },   std::array<int, 2>{ 0, -1 },
            std::array<int, 2>{ 1, 1 },   std::array<int, 2>{ -1, 1 },
            std::array<int, 2>{ -1, -1 }, std::array<int, 2>{ 1, -1 },
            std::array<int, 2>{ 2, 1 },   std::array<int, 2>{ 2, -1 },
            std::array<int, 2>{ -2, 1 },  std::array<int, 2>{ -2, -1 },
            std::array<int, 2>{ 1, 2 },   std::array<int, 2>{ -1, 2 },
            std::array<int, 2>{ 1, -2 },  std::array<int, 2>{ -1, -2 } };

    for( const auto& direction : legacyDirections )
    {
        const long double length = std::hypotl( static_cast<long double>( direction[0] ),
                                                static_cast<long double>( direction[1] ) );
        ROUTING_OBSTACLE blocker;
        blocker.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
        blocker.netCode = 2;
        blocker.layers = { 0, 1 };
        blocker.start = blocker.end = {
                board.pads[0].position.x + static_cast<std::int64_t>( std::llround(
                                                   1000000.0L * direction[0] / length ) ),
                board.pads[0].position.y + static_cast<std::int64_t>( std::llround(
                                                   1000000.0L * direction[1] / length ) ) };
        blocker.radius = 1;
        blocker.blocksTracks = true;
        blocker.blocksVias = true;
        board.obstacles.push_back( blocker );
    }

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.maxFanoutPasses = 1;
    settings.fanoutMinEscapeLengthIU = 1000000;
    settings.fanoutMaxEscapeLengthIU = 1000000;

    const BOARD_SNAPSHOT prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_REQUIRE_EQUAL( prepared.pads.size(), 3U );
    const ROUTING_PAD& landing = prepared.pads.back();
    const std::int64_t dx = landing.position.x - board.pads[0].position.x;
    const std::int64_t dy = landing.position.y - board.pads[0].position.y;
    BOOST_REQUIRE( dx != 0 && dy != 0 );

    // Rounding an angular escape to integral IU leaves a sub-IU error. The
    // selected direction must nevertheless be meaningfully distinct from
    // every legacy support direction; otherwise one of the deliberately
    // blocked probes leaked through instead of the refinement being used.
    const bool isLegacyDirection = std::any_of(
            legacyDirections.begin(), legacyDirections.end(), [&]( const auto& direction )
            {
                const long double cross = std::abs(
                        static_cast<long double>( dx ) * direction[1]
                        - static_cast<long double>( dy ) * direction[0] );
                return cross / std::hypotl( static_cast<long double>( direction[0] ),
                                             static_cast<long double>( direction[1] ) ) < 10.0L;
            } );
    BOOST_CHECK( !isLegacyDirection );
}


BOOST_AUTO_TEST_CASE( FanoutRejectsAStubCrossingSolidBetweenConcaveHoleArms )
{
    // Both endpoints of the preferred eastward escape are inside different
    // arms of one concave hole, so they are individually legal. Its direct
    // chord crosses the solid U-shaped region between them. An outer-contour
    // only check incorrectly accepts that first (roomiest) eastward landing;
    // a legal northward escape remains available so the planner must choose
    // it instead of falling back to a bent route through the blocked chord.
    BOARD_SNAPSHOT board = makeBoard();
    board.bounds = { 500000, 500000, 5000000, 4500000 };
    board.pads[0].position = { 1000000, 1500000 };
    board.pads[0].layers = { 0 };
    board.pads[0].isSmd = true;
    board.pads[0].radius = 1;
    board.pads[0].clearance = 0;
    board.pads[0].trackWidth = 2;
    board.pads[1].position = { 3000000, 1500000 };
    board.pads[1].layers = { 1 };
    board.pads[1].isSmd = false;
    board.nets[0].viaDiameter = 2;
    board.nets[0].viaDrill = 1;

    ROUTING_OBSTACLE obstacle;
    obstacle.kind = ROUTER_OBSTACLE_KIND::POLYGON;
    obstacle.netCode = 2;
    obstacle.layers = { 0, 1 };
    obstacle.polygon = { { 600000, 900000 }, { 3400000, 900000 },
                         { 3400000, 4100000 }, { 600000, 4100000 } };
    obstacle.polygonHoles = { { { 800000, 1100000 }, { 1400000, 1100000 },
                                { 1400000, 3400000 }, { 2600000, 3400000 },
                                { 2600000, 1100000 }, { 3200000, 1100000 },
                                { 3200000, 4000000 }, { 800000, 4000000 } } };
    obstacle.blocksTracks = true;
    obstacle.blocksVias = true;
    board.obstacles.push_back( std::move( obstacle ) );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.maxFanoutPasses = 1;
    settings.fanoutMinEscapeLengthIU = 2000000;
    settings.fanoutMaxEscapeLengthIU = 2000000;

    const BOARD_SNAPSHOT prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_REQUIRE_EQUAL( prepared.pads.size(), board.pads.size() + 1 );
    const ROUTING_PAD& landing = prepared.pads.back();
    BOOST_REQUIRE( landing.isFanoutTarget );
    BOOST_CHECK_EQUAL( landing.position.x, board.pads[0].position.x );
    BOOST_CHECK_GT( landing.position.y, board.pads[0].position.y );
}


BOOST_AUTO_TEST_CASE( FanoutUsesBoundedPreflightedBentEscapeBeforeGlobalSearch )
{
    // The direct fanout probes are deliberately blocked at every sampled
    // radius. A source-layer wall still leaves a short bent escape: travel
    // above the wall, then place the via on its far side. The old planner
    // accepted an arbitrary free via point and made the global maze rediscover
    // this local path. The bounded local search must retain the actual escape
    // and the maze must consume it without expanding a board-wide frontier.
    BOARD_SNAPSHOT board = makeBoard();
    board.bounds = { 0, 0, 7000000, 7000000 };
    board.pads[0].position = { 1000000, 1000000 };
    board.pads[0].layers = { 0 };
    board.pads[0].isSmd = true;
    board.pads[0].radius = 1;
    board.pads[0].clearance = 0;
    board.pads[0].trackWidth = 2;
    board.pads[1].position = { 5000000, 1000000 };
    board.pads[1].layers = { 1 };
    board.pads[1].isSmd = false;
    board.nets[0].viaDiameter = 2;
    board.nets[0].viaDrill = 1;

    ROUTING_OBSTACLE wall;
    wall.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    wall.netCode = 2;
    wall.layers = { 0 };
    wall.box = { 1250000, 0, 2000000, 2250000 };
    wall.blocksTracks = true;
    wall.blocksVias = true;
    board.obstacles.push_back( wall );

    // The source can travel through this region on its top layer, but may not
    // stop a via there. It forces the accepted landing to be on the far side
    // of the source-layer wall instead of a trivial left-hand escape.
    ROUTING_OBSTACLE viaKeepout;
    viaKeepout.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    viaKeepout.netCode = 2;
    viaKeepout.layers = { 1 };
    viaKeepout.box = { 0, 0, 2000000, 7000000 };
    viaKeepout.blocksTracks = false;
    viaKeepout.blocksVias = true;
    board.obstacles.push_back( viaKeepout );

    // Mirror the bounded direct probe set used by BatchFanout and block each
    // sampled endpoint. A non-probe local grid point remains available only
    // through the bent source-layer path around the wall.
    std::vector<std::array<int, 2>> directions = {
            { 1, 0 },   { -1, 0 }, { 0, 1 },   { 0, -1 }, { 1, 1 },  { -1, 1 },
            { -1, -1 }, { 1, -1 }, { 2, 1 },   { 2, -1 }, { -2, 1 }, { -2, -1 },
            { 1, 2 },   { -1, 2 }, { 1, -2 },  { -1, -2 } };
    for( int x = -4; x <= 4; ++x )
    {
        for( int y = -4; y <= 4; ++y )
        {
            if( ( x != 0 || y != 0 ) && std::gcd( std::abs( x ), std::abs( y ) ) == 1
                && std::max( std::abs( x ), std::abs( y ) ) > 2 )
            {
                directions.push_back( { x, y } );
            }
        }
    }

    for( std::int64_t distance : { 2000000, 2500000, 3000000, 3500000 } )
    {
        for( const auto& direction : directions )
        {
            const long double length = std::hypotl( static_cast<long double>( direction[0] ),
                                                    static_cast<long double>( direction[1] ) );
            ROUTING_OBSTACLE blocker;
            blocker.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
            blocker.netCode = 2;
            blocker.layers = { 0, 1 };
            blocker.start = blocker.end = {
                    board.pads[0].position.x + static_cast<std::int64_t>( std::llround(
                                                       distance * direction[0] / length ) ),
                    board.pads[0].position.y + static_cast<std::int64_t>( std::llround(
                                                       distance * direction[1] / length ) ) };
            blocker.radius = 1;
            blocker.blocksTracks = true;
            blocker.blocksVias = true;
            board.obstacles.push_back( blocker );
        }
    }

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.maxFanoutPasses = 1;
    settings.fanoutMinEscapeLengthIU = 2000000;
    settings.fanoutMaxEscapeLengthIU = 3500000;
    settings.fanoutLandingSearchSteps = 20;

    const BOARD_SNAPSHOT prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_REQUIRE_EQUAL( prepared.pads.size(), board.pads.size() + 1 );
    const ROUTING_PAD& landing = prepared.pads.back();
    BOOST_REQUIRE( landing.isFanoutTarget );
    BOOST_REQUIRE_GT( landing.fanoutEscapePath.size(), 2U );
    BOOST_CHECK( landing.fanoutEscapePath.front() == board.pads[0].position );
    BOOST_CHECK( landing.fanoutEscapePath.back() == landing.position );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( prepared, settings );
    MAZE_SEARCH_ENGINE engine( prepared, settings, occupancy );
    int expanded = -1;
    const auto route = engine.FindConnection( prepared.pads[0], landing, 0, expanded, {} );
    BOOST_REQUIRE( route );
    BOOST_CHECK_EQUAL( expanded, 0 );
    BOOST_REQUIRE_EQUAL( route->nodes.size(), landing.fanoutEscapePath.size() + 1 );
    for( std::size_t index = 1; index < landing.fanoutEscapePath.size(); ++index )
        BOOST_CHECK( route->nodes[index].point == landing.fanoutEscapePath[index] );
    BOOST_CHECK_NE( route->nodes[route->nodes.size() - 2].layer,
                    route->nodes.back().layer );

    // The connected-set scheduler can also visit the synthetic landing
    // first. Its reverse fanout edge must consume the same certified bent
    // escape rather than falling through to a board-wide search that assumes
    // the two endpoints have a direct source-layer stub.
    int reverseExpanded = -1;
    const auto reverse = engine.FindConnection( landing, prepared.pads[0], 0, reverseExpanded,
                                                {} );
    BOOST_REQUIRE( reverse );
    BOOST_CHECK_EQUAL( reverseExpanded, 0 );
    BOOST_REQUIRE_EQUAL( reverse->nodes.size(), landing.fanoutEscapePath.size() + 1 );
    BOOST_CHECK( reverse->nodes.front().point == landing.position );
    BOOST_CHECK_EQUAL( reverse->nodes.front().layer, landing.fanoutTargetLayer );
    BOOST_CHECK_EQUAL( reverse->nodes[1].layer, landing.fanoutSourceLayer );
    for( std::size_t index = 2; index < reverse->nodes.size(); ++index )
    {
        const std::size_t pathIndex = landing.fanoutEscapePath.size() - index;
        BOOST_CHECK( reverse->nodes[index].point == landing.fanoutEscapePath[pathIndex] );
        BOOST_CHECK_EQUAL( reverse->nodes[index].layer, landing.fanoutSourceLayer );
    }
    BOOST_CHECK( reverse->nodes.back().point == board.pads[0].position );
}


BOOST_AUTO_TEST_CASE( FanoutTriesLaterBoardViaProfileWhenEarlierProfileCannotEscape )
{
    // RoutingBoard.fanout appends every board ViaRule alternative to the
    // netclass rule.  The first board profile is intentionally too large to
    // land in this channel; the second profile must be considered rather
    // than discarding the SMD escape after the first rejection.
    BOARD_SNAPSHOT board = makeBoard();
    board.bounds = { 0, 0, 8000000, 3000000 };
    board.pads[0].layers = { 0 };
    board.pads[0].isSmd = true;
    board.pads[1].position = { 7000000, 1500000 };
    board.pads[1].layers = { 1 };
    board.nets[0].viaDiameter = 0;
    board.nets[0].viaDrill = 0;
    board.boardViaDimensions = { { 2000000, 100000 }, { 300000, 100000 } };

    // With a fixed 1 mm escape, the 2 mm via only has one in-board landing
    // at (2 mm, 1.5 mm).  This small foreign-net obstacle blocks its annulus
    // but remains clear of the 0.3 mm fallback profile.
    ROUTING_OBSTACLE blocker;
    blocker.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    blocker.netCode = 2;
    blocker.layers = { 0, 1 };
    blocker.start = { 2800000, 1500000 };
    blocker.end = blocker.start;
    blocker.radius = 100000;
    blocker.blocksTracks = true;
    blocker.blocksVias = true;
    board.obstacles.push_back( blocker );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.enableFanout = true;
    settings.maxFanoutPasses = 1;
    settings.fanoutMinEscapeLengthIU = 1000000;
    settings.fanoutMaxEscapeLengthIU = 1000000;

    const BOARD_SNAPSHOT prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_REQUIRE_EQUAL( prepared.pads.size(), 3U );
    const ROUTING_PAD& landing = prepared.pads.back();
    BOOST_REQUIRE( landing.isFanoutTarget );
    BOOST_CHECK_EQUAL( landing.fanoutViaDiameter, 300000 );
    BOOST_CHECK_EQUAL( landing.fanoutViaDrill, 100000 );

    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_REQUIRE_EQUAL( result.vias.size(), 1U );
    BOOST_CHECK_EQUAL( result.vias.front().diameter, 300000 );
    BOOST_CHECK_EQUAL( result.vias.front().drill, 100000 );
}


BOOST_AUTO_TEST_CASE( FanoutKeepsEachSelectedViaProfileScopedToItsPin )
{
    // Freerouting's combined ViaRule is evaluated separately for every SMD
    // pin.  Here the first pin can use the large first profile, while the
    // second pin's only legal landing requires the later small profile.  A
    // net-wide profile mutation makes the second fanout search test its via
    // against the first pin's large annulus and reject a legal escape.
    BOARD_SNAPSHOT board = makeBoard();
    // Make the usable vertical margin around the large annulus smaller than
    // the shallowest non-cardinal 1 mm escape. A fanout can consequently
    // leave this horizontal channel only in either cardinal horizontal
    // direction; every angular landing puts the large annulus beyond the
    // board edge. This keeps the test about ViaRule ownership rather than
    // accidentally relying on the old finite probe list now that fanout also
    // searches non-cardinal doors.
    board.bounds = { 0, 0, 12000000, 1400000 };
    board.pads[0].position = { 2000000, 700000 };
    board.pads[0].layers = { 0 };
    board.pads[0].isSmd = true;
    board.pads[0].componentId = 1;
    board.pads[0].pinIndex = 1;
    board.pads[1].position = { 8000000, 700000 };
    board.pads[1].layers = { 0 };
    board.pads[1].isSmd = true;
    board.pads[1].componentId = 2;
    board.pads[1].pinIndex = 1;

    ROUTING_PAD destination = board.pads[1];
    destination.position = { 11000000, 700000 };
    destination.layers = { 1 };
    destination.isSmd = false;
    destination.componentId = 3;
    board.pads.push_back( destination );

    board.nets[0].viaDiameter = 0;
    board.nets[0].viaDrill = 0;
    board.nets[0].padIndices = { 0, 1, 2 };
    board.nets[0].connections = { { 0, 2 }, { 1, 2 } };
    board.boardViaDimensions = { { 1200000, 100000 }, { 300000, 100000 } };

    // The second pin is ordered after the first and escapes left to (7 mm,
    // 0.7 mm).  This point is clear for the 0.3 mm alternative but not the
    // 1.2 mm first alternative.  The only other in-board large-via landing
    // is blocked, so a large-profile search cannot quietly use a different
    // transition and mask a profile leak.
    const auto addLayerZeroBlocker = [&]( ROUTER_POINT aPoint )
    {
        ROUTING_OBSTACLE blocker;
        blocker.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
        blocker.netCode = 2;
        blocker.layers = { 0 };
        blocker.start = aPoint;
        blocker.end = aPoint;
        blocker.radius = 100000;
        blocker.blocksTracks = true;
        blocker.blocksVias = true;
        board.obstacles.push_back( blocker );
    };
    addLayerZeroBlocker( { 7000000, 1450000 } );
    addLayerZeroBlocker( { 9000000, 700000 } );
    ROUTING_OBSTACLE sourceLayerWall;
    sourceLayerWall.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    sourceLayerWall.netCode = 0;
    sourceLayerWall.layers = { 0 };
    sourceLayerWall.box = { 5000000, 0, 5100000, 1400000 };
    sourceLayerWall.blocksTracks = true;
    sourceLayerWall.blocksVias = false;
    board.obstacles.push_back( sourceLayerWall );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.enableFanout = true;
    settings.maxFanoutPasses = 1;
    settings.fanoutMinEscapeLengthIU = 1000000;
    settings.fanoutMaxEscapeLengthIU = 1000000;

    const BOARD_SNAPSHOT prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    std::map<std::size_t, ROUTING_VIA_DIMENSION> landingProfiles;
    for( const ROUTING_PAD& pad : prepared.pads )
    {
        if( pad.isFanoutTarget )
        {
            landingProfiles.emplace( pad.fanoutSourcePadIndex,
                                     ROUTING_VIA_DIMENSION{ pad.fanoutViaDiameter,
                                                            pad.fanoutViaDrill } );
        }
    }
    BOOST_REQUIRE_EQUAL( landingProfiles.size(), 2U );
    BOOST_CHECK( ( landingProfiles.at( 0 ) == ROUTING_VIA_DIMENSION{ 1200000, 100000 } ) );
    BOOST_CHECK( ( landingProfiles.at( 1 ) == ROUTING_VIA_DIMENSION{ 300000, 100000 } ) );

    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_CHECK_EQUAL( std::count_if( result.vias.begin(), result.vias.end(),
                                      []( const ROUTING_VIA& aVia )
                                      { return aVia.diameter == 1200000 && aVia.drill == 100000; } ),
                       1 );
    BOOST_CHECK_EQUAL( std::count_if( result.vias.begin(), result.vias.end(),
                                      []( const ROUTING_VIA& aVia )
                                      { return aVia.diameter == 300000 && aVia.drill == 100000; } ),
                       1 );
}


BOOST_AUTO_TEST_CASE( FanoutPinTimeoutFallsBackToTheOrdinaryBatchStage )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.bounds = { 0, 0, 8000000, 3000000 };
    board.pads[0].layers = { 0 };
    board.pads[0].isSmd = true;
    board.pads[1].position = { 7000000, 1500000 };
    board.pads[1].layers = { 1 };

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.maxFanoutPasses = 1;
    settings.maxFanoutMillisecondsPerPin = 0;

    // A per-pin fanout budget may expire without cancelling the entire job.
    // The synthetic landing must be removed and the normal batch router must
    // restart from the real SMD pad rather than leaving disconnected copper.
    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( !result.cancelled );
    BOOST_REQUIRE( result.complete );
    BOOST_CHECK( !result.fanoutTimedOut );
    BOOST_CHECK_EQUAL( result.metrics.fanoutConnections, 0 );
    BOOST_CHECK_EQUAL( result.metrics.routedConnections, 1 );
    BOOST_CHECK_EQUAL( result.vias.size(), 1U );
}


BOOST_AUTO_TEST_CASE( ViaInSmdPadSettingDisablesSyntheticFanout )
{
    BOARD_SNAPSHOT board;
    board.bounds = { 0, 0, 6000000, 3000000 };
    board.pads.push_back( { 1, { 1000000, 1500000 }, { 0 }, "Default", 0, 100000, 0,
                            100000, false, true } );
    board.pads.push_back( { 1, { 5000000, 1500000 }, { 1 }, "Default", 0, 100000, 0,
                            100000, false, true } );

    ROUTING_NET net;
    net.netCode = 1;
    net.name = "SMD_NET";
    net.netClass = "Default";
    net.padIndices = { 0, 1 };
    net.connections = { { 0, 1 } };
    board.nets.push_back( net );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.allowViaInSmdPad = true;
    const BOARD_SNAPSHOT fanned = BATCH_FANOUT::PrepareSnapshot( board, settings );

    BOOST_REQUIRE_EQUAL( fanned.pads.size(), board.pads.size() );
    BOOST_REQUIRE_EQUAL( fanned.nets.front().connections.size(), 1 );

    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_REQUIRE_EQUAL( result.vias.size(), 1 );
}


BOOST_AUTO_TEST_CASE( RoutesAroundOffGridObstaclesOnEveryEnabledLayer )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.obstacles.push_back( { ROUTER_OBSTACLE_KIND::RECTANGLE,
                                 0,
                                 { 0, 1 },
                                 {},
                                 {},
                                 { 2450000, 1250000, 3550000, 1750000 },
                                 {},
                                 0,
                                 true,
                                 true,
                                 false,
                                 {},
                                 0 } );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.gridStepIU = 1000000;
    settings.maxExpandedNodes = 50000;

    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( board, settings, {}, {} );

    BOOST_REQUIRE( result.complete );
    BOOST_CHECK_GT( result.segments.size(), 1 );
    BOOST_CHECK_EQUAL( result.metrics.drcViolations, 0 );
}


BOOST_AUTO_TEST_CASE( PreservesPhysicalLayerOrderForViaSpans )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.pads[0].layers = { 10 };
    board.pads[1].layers = { 20 };

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.layers = { { 10, true, 1, 20, 1 }, { 20, true, 2, 20, 0 } };

    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( board, settings, {}, {} );

    BOOST_REQUIRE( result.complete );
    for( const ROUTING_VIA& via : result.vias )
    {
        BOOST_CHECK_EQUAL( via.topLayer, 20 );
        BOOST_CHECK_EQUAL( via.bottomLayer, 10 );
        BOOST_CHECK_EQUAL( via.layers.size(), 2 );
        BOOST_CHECK_EQUAL( via.layers.front(), 20 );
        BOOST_CHECK_EQUAL( via.layers.back(), 10 );
    }
}


BOOST_AUTO_TEST_CASE( GrowsAConnectedSetForMultiPadNets )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.pads.push_back( { 1, { 3000000, 2500000 }, { 0, 1 }, "Default", 0, 100000,
                            100000, 100000 } );
    board.nets.front().padIndices.push_back( 2 );
    board.nets.front().connections = { { 0, 1 }, { 1, 2 } };

    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( board, makeSettings(), {}, {} );

    BOOST_REQUIRE( result.complete );
    BOOST_CHECK_EQUAL( result.metrics.totalConnections, 2 );
    BOOST_CHECK_EQUAL( result.metrics.routedConnections, 2 );
    BOOST_REQUIRE_EQUAL( result.connections.size(), 2 );
    // A ratsnest edge is a connectivity requirement, not a mandated pair of
    // routing endpoints. Either orientation/tree is legal if all pads join.
    std::vector<std::size_t> parent{ 0, 1, 2 };
    auto root = [&]( std::size_t index )
    {
        while( parent[index] != index )
            index = parent[index];
        return index;
    };
    for( const auto& connection : result.connections )
    {
        BOOST_REQUIRE_LT( connection.fromPadIndex, parent.size() );
        BOOST_REQUIRE_LT( connection.toPadIndex, parent.size() );
        BOOST_REQUIRE( !connection.nodes.empty() );
        BOOST_CHECK( connection.nodes.front().point == board.pads[connection.fromPadIndex].position );
        BOOST_CHECK( connection.nodes.back().point == board.pads[connection.toPadIndex].position );
        parent[root( connection.fromPadIndex )] = root( connection.toPadIndex );
    }
    BOOST_CHECK_EQUAL( root( 0 ), root( 1 ) );
    BOOST_CHECK_EQUAL( root( 1 ), root( 2 ) );
}


BOOST_AUTO_TEST_CASE( PolygonHolesRemainLegalFreeSpace )
{
    BOARD_SNAPSHOT board;
    board.bounds = { 0, 0, 6000000, 6000000 };
    board.pads.push_back( { 1, { 2600000, 3000000 }, { 0 }, "Default", 0, 100000, 0,
                            100000 } );
    board.pads.push_back( { 1, { 3400000, 3000000 }, { 0 }, "Default", 0, 100000, 0,
                            100000 } );

    ROUTING_NET net;
    net.netCode = 1;
    net.name = "N1";
    net.netClass = "Default";
    net.padIndices = { 0, 1 };
    net.connections = { { 0, 1 } };
    board.nets.push_back( net );

    ROUTING_OBSTACLE obstacle;
    obstacle.kind = ROUTER_OBSTACLE_KIND::POLYGON;
    obstacle.netCode = 2;
    obstacle.layers = { 0 };
    obstacle.polygon = { { 2000000, 2000000 }, { 4000000, 2000000 },
                         { 4000000, 4000000 }, { 2000000, 4000000 } };
    obstacle.polygonHoles = { { { 2500000, 2500000 }, { 3500000, 2500000 },
                                { 3500000, 3500000 }, { 2500000, 3500000 } } };
    board.obstacles.push_back( obstacle );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.layers = { { 0, true, 1, 20 } };
    settings.gridStepIU = 250000;

    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( board, settings, {}, {} );

    BOOST_REQUIRE( result.complete );
    BOOST_CHECK_EQUAL( result.metrics.drcViolations, 0 );
}


BOOST_AUTO_TEST_CASE( NetAssignedKeepoutStillBlocksTheNet )
{
    BOARD_SNAPSHOT board = makeBoard();
    ROUTING_OBSTACLE keepout;
    keepout.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    keepout.netCode = 1;
    keepout.layers = { 0 };
    keepout.box = { 2400000, 0, 3600000, 3000000 };
    keepout.blocksTracks = true;
    keepout.blocksVias = true;
    keepout.isKeepout = true;
    board.obstacles.push_back( keepout );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.layers = { { 0, true, 1, 20 } };

    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( board, settings, {}, {} );

    BOOST_CHECK( !result.complete );
    BOOST_CHECK_EQUAL( result.metrics.routedConnections, 0 );
    BOOST_CHECK_EQUAL( result.metrics.unroutedConnections, 1 );
}


BOOST_AUTO_TEST_CASE( KiCadAdapterPreservesCopperClustersAcrossSeveralTracks )
{
    BOARD board;
    auto* net = new NETINFO_ITEM( &board, "CLUSTER", 1 );
    board.Add( net );
    auto* footprint = new FOOTPRINT( &board );
    board.Add( footprint );
    for( int x : { 1000000, 5000000, 8000000 } )
    {
        auto* pad = new PAD( footprint );
        pad->SetAttribute( PAD_ATTRIB::SMD );
        pad->SetLayerSet( LSET( { F_Cu } ) );
        pad->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::CIRCLE );
        pad->SetSize( PADSTACK::ALL_LAYERS, { 300000, 300000 } );
        pad->SetPosition( { x, 1000000 } );
        pad->SetNet( net );
        footprint->Add( pad );
    }
    // The ratsnest anchor is at a dangling track end, not beside any pad.
    for( const auto& [start, end] : std::vector<std::pair<int, int>>{
                 { 1000000, 3000000 }, { 3000000, 5000000 }, { 5000000, 6000000 },
                 { 6000000, 7000000 } } )
    {
        auto* track = new PCB_TRACK( &board );
        track->SetStart( { start, 1000000 } );
        track->SetEnd( { end, 1000000 } );
        track->SetWidth( 100000 );
        track->SetLayer( F_Cu );
        track->SetNet( net );
        board.Add( track );
    }
    board.BuildConnectivity();
    board.GetConnectivity()->RecalculateRatsnest();
    BOOST_REQUIRE_EQUAL( board.GetConnectivity()->GetUnconnectedCount( false ), 1 );
    KICAD_BOARD_ADAPTER adapter( &board );
    auto settings = adapter.CreateDefaultSettings();
    const auto snapshot = adapter.CreateSnapshot( settings );
    BOOST_REQUIRE( snapshot );
    BOOST_REQUIRE_EQUAL( snapshot->nets.size(), 1 );
    const auto& captured = snapshot->nets.front();
    BOOST_REQUIRE_EQUAL( captured.connectedPadGroups.size(), 1 );
    BOOST_CHECK_EQUAL( captured.connectedPadGroups.front().size(), 2 );
    BOOST_CHECK_EQUAL( captured.connections.size(), 1 );
    for( const auto index : captured.connectedPadGroups.front() )
        BOOST_CHECK_LE( snapshot->pads[index].position.x, 5000000 );
    settings.allowRipupExisting = true;
    const auto reroute = adapter.CreateSnapshot( settings );
    BOOST_REQUIRE( reroute );
    BOOST_CHECK( reroute->nets.front().connectedPadGroups.empty() );
    BOOST_CHECK_EQUAL( board.Tracks().size(), 4 );

    // A repair pass sees copper emitted by the first autorouter pass as real
    // BOARD_ITEMs.  It must not freeze connectivity through those items or
    // classify them as protected user copper merely because they now have a
    // KiCad UUID.  Mark one item as job-owned and verify that it remains in
    // the removable route model even though ordinary source rip-up is off.
    std::string ownedTrackId;
    for( const PCB_TRACK* track : board.Tracks() )
    {
        ownedTrackId = track->m_Uuid.AsString().ToStdString();
        break;
    }
    BOOST_REQUIRE( !ownedTrackId.empty() );

    KICAD_BOARD_ADAPTER repairAdapter( &board, { ownedTrackId } );
    auto repairSettings = repairAdapter.CreateDefaultSettings();
    repairSettings.allowRipupExisting = false;
    const auto repair = repairAdapter.CreateSnapshot( repairSettings );
    BOOST_REQUIRE( repair );
    BOOST_CHECK( repair->nets.front().connectedPadGroups.empty() );
    BOOST_CHECK_GE( repair->nets.front().connections.size(), 2U );
    BOOST_CHECK( std::any_of(
            repair->removableExistingRoutes.begin(),
            repair->removableExistingRoutes.end(), [&]( const ROUTING_OBSTACLE& obstacle )
            {
                return obstacle.boardItemId == ownedTrackId
                       && obstacle.isAutorouterOwned;
            } ) );
    BOOST_CHECK( std::none_of(
            repair->obstacles.begin(), repair->obstacles.end(),
            [&]( const ROUTING_OBSTACLE& obstacle )
            {
                return obstacle.boardItemId == ownedTrackId;
            } ) );
}


BOOST_AUTO_TEST_CASE( KiCadAdapterCapturesARealBoardWithoutMutatingIt )
{
    const std::string path = KI_TEST::GetPcbnewTestDataDir()
                             + "pns_regressions/boards/simple.kicad_pcb";
    std::unique_ptr<BOARD> board = KI_TEST::ReadBoardFromFileOrStream( path );

    BOOST_REQUIRE( board );

    KICAD_BOARD_ADAPTER adapter( board.get() );
    AUTOROUTER_SETTINGS settings = adapter.CreateDefaultSettings();
    settings.routeOnlyUnconnected = false;
    settings.maxPasses = 1;
    settings.maxIterations = 1;
    settings.optimizationPasses = 0;
    settings.maxExpandedNodes = 10000;
    settings.enableFanout = false;
    settings.includeNets = { "Net-(IC1-Vdd)" };

    const std::shared_ptr<const BOARD_SNAPSHOT> snapshot = adapter.CreateSnapshot( settings );
    BOOST_REQUIRE( snapshot );
    BOOST_CHECK_GT( snapshot->pads.size(), 0 );
    BOOST_CHECK_GT( snapshot->nets.size(), 0 );
    BOOST_CHECK( !snapshot->obstacles.empty() );

    const std::size_t trackCount = board->Tracks().size();
    const std::size_t padCount = board->GetPads().size();
    const std::size_t zoneCount = board->Zones().size();
    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( *snapshot, settings, {}, {} );

    BOOST_CHECK( !result.cancelled );
    BOOST_CHECK_EQUAL( board->Tracks().size(), trackCount );
    BOOST_CHECK_EQUAL( board->GetPads().size(), padCount );
    BOOST_CHECK_EQUAL( board->Zones().size(), zoneCount );
}


BOOST_AUTO_TEST_CASE( KiCadAdapterCreatesOrdinaryPreviewBoardItems )
{
    const std::string path = KI_TEST::GetPcbnewTestDataDir()
                             + "pns_regressions/boards/simple.kicad_pcb";
    std::unique_ptr<BOARD> board = KI_TEST::ReadBoardFromFileOrStream( path );

    BOOST_REQUIRE( board );

    KICAD_BOARD_ADAPTER adapter( board.get() );
    const std::size_t trackCount = board->Tracks().size();
    ROUTING_RESULT result;
    result.segments.push_back( { 1, static_cast<int>( F_Cu ), { 1000000, 1000000 },
                                 { 1500000, 1000000 }, 100000 } );
    result.vias.push_back( { 1, { 1500000, 1000000 }, static_cast<int>( F_Cu ),
                             static_cast<int>( B_Cu ), 600000, 300000,
                             { static_cast<int>( F_Cu ), static_cast<int>( B_Cu ) } } );

    std::vector<std::unique_ptr<BOARD_ITEM>> preview = adapter.CreatePreviewItems( result );

    BOOST_REQUIRE_EQUAL( preview.size(), 2 );
    BOOST_CHECK_EQUAL( preview[0]->Type(), PCB_TRACE_T );
    BOOST_CHECK_EQUAL( preview[1]->Type(), PCB_VIA_T );
    BOOST_CHECK_EQUAL( board->Tracks().size(), trackCount );
}


BOOST_AUTO_TEST_CASE( ThroughViasOccupyInactiveLayersOutsideSearchTransition )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    // Deliberately non-numeric stack order, with an inactive outer layer.
    settings.layers = { { 20, false, 0, 0, 0 }, { 5, true, 0, 0, 1 },
                        { 18, true, 0, 0, 2 }, { 10, true, 0, 0, 3 } };
    const ROUTER_POINT position{ 3000000, 1500000 };
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    MAZE_SEARCH_ENGINE freeSearch( board, settings, occupancy );
    BOOST_CHECK( freeSearch.CanUseSegment( 1, { position, 5 }, { position, 18 }, true ) );
    BOOST_CHECK( !freeSearch.CanUseSegment( 1, { position, 5 }, { position, 20 }, true ) );
    BOOST_CHECK( !freeSearch.CanUseSegment( 1, { position, 5 },
                                             { { 3100000, 1500000 }, 18 }, true ) );

    ROUTING_OBSTACLE obstacle;
    obstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    obstacle.netCode = 2;
    obstacle.layers = { 20 };
    obstacle.start = obstacle.end = position;
    obstacle.radius = 100000;
    board.obstacles.push_back( obstacle );
    MAZE_SEARCH_ENGINE blockedSearch( board, settings, occupancy );
    BOOST_CHECK( !blockedSearch.CanUseSegment( 1, { position, 5 }, { position, 18 }, true ) );

    // A via already proposed between two inner layers also occupies the outside.
    board.obstacles.clear();
    ROUTING_CONNECTION existing;
    existing.netCode = 2;
    existing.complete = true;
    existing.nodes = { { position, 5 }, { position, 18 } };
    occupancy.Add( existing );
    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.nodes = { { { 2500000, 1500000 }, 10 }, { { 3500000, 1500000 }, 10 } };
    MAZE_SEARCH_ENGINE occupiedSearch( board, settings, occupancy );
    BOOST_CHECK_EQUAL( occupiedSearch.FindConflictingConnections( candidate ).size(), 1 );
}


BOOST_AUTO_TEST_CASE( DirectFanoutChecksTheWholeSelectedViaPadstack )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.layers = { { 20, false, 0, 0, 0 }, { 5, true, 0, 0, 1 },
                        { 18, true, 0, 0, 2 }, { 10, false, 0, 0, 3 } };
    settings.maxExpandedNodes = 100;
    board.pads[0].layers = { 5 };
    auto landing = board.pads[1];
    landing.layers = { 18 };
    landing.isFanoutTarget = true;
    landing.fanoutSourceLayer = 5;
    landing.fanoutTargetLayer = 18;
    ROUTING_OBSTACLE blocker;
    blocker.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    blocker.layers = { 20 };
    blocker.box = board.bounds;
    blocker.blocksTracks = false;
    board.obstacles.push_back( blocker );
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    MAZE_SEARCH_ENGINE blocked( board, settings, occupancy );
    int expanded = 0;
    BOOST_CHECK( !blocked.FindConnection( board.pads[0], landing, 0, expanded, {} ) );
    BOOST_CHECK( !blocked.FindConnection( landing, board.pads[0], 0, expanded, {} ) );
    board.obstacles.clear();
    MAZE_SEARCH_ENGINE clear( board, settings, occupancy );
    BOOST_CHECK( clear.FindConnection( board.pads[0], landing, 0, expanded, {} ) );
    BOOST_CHECK( clear.FindConnection( landing, board.pads[0], 0, expanded, {} ) );
}


BOOST_AUTO_TEST_CASE( FanoutRejectsLandingThatViolatesDrillToHoleClearance )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.enableFanout = true;
    board.pads[0].isSmd = true;
    board.pads[0].layers = { 0 };
    board.pads[0].radius = 100000;
    board.pads[0].clearance = 0;
    board.pads[0].trackWidth = 100000;
    board.pads[1].layers = { 0, 1 };
    board.nets[0].viaDiameter = 300000;
    board.nets[0].viaDrill = 150000;

    // The first deterministic fanout probe is 150000 IU to the right of
    // the SMD pad. Copper-to-hole spacing would permit that via, but its
    // 75000-IU drill plus this existing 50000-IU hole must observe the
    // 200000-IU hole-to-hole rule. This used to select a landing that the
    // final DRC would reject.
    board.holeClearance = 0;
    board.holeToHoleClearance = 200000;
    ROUTING_OBSTACLE hole;
    hole.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    hole.layers = { 0, 1 };
    hole.start = hole.end = { 1400000, 1500000 };
    hole.radius = 50000;
    hole.blocksTracks = true;
    hole.blocksVias = true;
    hole.isHole = true;
    board.obstacles.push_back( hole );

    const BOARD_SNAPSHOT prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_REQUIRE_EQUAL( prepared.pads.size(), 3 );
    const ROUTING_PAD& landing = prepared.pads.back();
    BOOST_REQUIRE( landing.isFanoutTarget );
    BOOST_CHECK( landing.position != ( ROUTER_POINT{ 1150000, 1500000 } ) );
}

BOOST_AUTO_TEST_CASE( InnerLayerRouteInsertsTheSelectedThroughPadstack )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.layers = { { 20, false, 0, 0, 0 }, { 5, true, 0, 0, 1 },
                        { 18, true, 0, 0, 2 }, { 10, false, 0, 0, 3 } };
    board.pads[0].layers = { 5 };
    board.pads[1].layers = { 18 };
    const auto result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_REQUIRE_EQUAL( result.vias.size(), 1 );
    BOOST_CHECK_EQUAL( result.vias[0].topLayer, 20 );
    BOOST_CHECK_EQUAL( result.vias[0].bottomLayer, 10 );
    const std::vector<int> expected{ 20, 5, 18, 10 };
    BOOST_CHECK( result.vias[0].layers == expected );
    BOOST_CHECK_EQUAL( DESIGN_RULES_CHECKER::CountViolations( board, settings, result ), 0 );
}


BOOST_AUTO_TEST_CASE( KiCadAdapterPreservesRotatedCopperAndInactiveLayerObstacles )
{
    BOARD board;
    board.SetCopperLayerCount( 4 );
    auto* net = new NETINFO_ITEM( &board, "OBSTACLE", 2 );
    board.Add( net );
    auto* footprint = new FOOTPRINT( &board );
    board.Add( footprint );
    auto* pad = new PAD( footprint );
    pad->SetAttribute( PAD_ATTRIB::SMD );
    pad->SetLayerSet( LSET( { F_Cu } ) );
    pad->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::RECTANGLE );
    pad->SetSize( PADSTACK::ALL_LAYERS, { 3000000, 400000 } );
    pad->SetPosition( { 3000000, 3000000 } );
    pad->SetOrientation( EDA_ANGLE( 45, DEGREES_T ) );
    pad->SetNet( net );
    footprint->Add( pad );
    board.BuildConnectivity();
    KICAD_BOARD_ADAPTER adapter( &board );
    auto settings = adapter.CreateDefaultSettings();
    auto snapshot = adapter.CreateSnapshot( settings );
    BOOST_REQUIRE( snapshot );
    BOOST_REQUIRE_EQUAL( snapshot->obstacles.size(), 1 );
    BOOST_CHECK( snapshot->obstacles[0].kind == ROUTER_OBSTACLE_KIND::POLYGON );
    BOOST_CHECK_EQUAL( snapshot->obstacles[0].polygon.size(), 4 );
    BOARD_SNAPSHOT routingBoard = *snapshot;
    routingBoard.bounds = { 0, 0, 6000000, 6000000 };
    routingBoard.boardOutline.clear();
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    MAZE_SEARCH_ENGINE search( routingBoard, settings, occupancy );
    // This point lies inside the old axis-aligned bounding box but well
    // outside the rotated copper. The centre must remain blocked.
    const ROUTER_NODE freePoint{ { 2200000, 2200000 }, F_Cu };
    const ROUTER_NODE blockedPoint{ { 3000000, 3000000 }, F_Cu };
    BOOST_CHECK( search.CanUseSegment( 999, freePoint, freePoint ) );
    BOOST_CHECK( !search.CanUseSegment( 999, blockedPoint, blockedPoint ) );

    for( auto& layer : settings.layers )
        if( layer.layerId == F_Cu )
            layer.enabled = false;
    snapshot = adapter.CreateSnapshot( settings );
    BOOST_REQUIRE( snapshot );
    BOOST_REQUIRE_EQUAL( snapshot->obstacles.size(), 1 );
    BOOST_CHECK_EQUAL( snapshot->obstacles[0].layers.front(), F_Cu );
}


BOOST_AUTO_TEST_CASE( KiCadAdapterUsesExactOvalCapsules )
{
    BOARD board;
    auto* footprint = new FOOTPRINT( &board );
    board.Add( footprint );
    auto* pad = new PAD( footprint );
    pad->SetAttribute( PAD_ATTRIB::SMD );
    pad->SetLayerSet( LSET( { F_Cu } ) );
    pad->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::OVAL );
    pad->SetSize( PADSTACK::ALL_LAYERS, { 3000000, 400000 } );
    pad->SetPosition( { 3000000, 3000000 } );
    footprint->Add( pad );
    KICAD_BOARD_ADAPTER adapter( &board );
    const auto settings = adapter.CreateDefaultSettings();
    for( int angle : { 0, 90 } )
    {
        pad->SetOrientation( EDA_ANGLE( angle, DEGREES_T ) );
        const auto snapshot = adapter.CreateSnapshot( settings );
        BOOST_REQUIRE( snapshot );
        BOOST_REQUIRE_EQUAL( snapshot->obstacles.size(), 1 );
        const auto& oval = snapshot->obstacles.front();
        BOOST_CHECK( oval.kind == ROUTER_OBSTACLE_KIND::SEGMENT );
        BOOST_CHECK_EQUAL( oval.radius, 200000 );
        BOOST_CHECK( oval.polygon.empty() );
        if( angle == 0 )
        {
            BOOST_CHECK_EQUAL( oval.start.y, 3000000 );
            BOOST_CHECK_EQUAL( oval.end.y, 3000000 );
            BOOST_CHECK_EQUAL( std::min( oval.start.x, oval.end.x ), 1700000 );
            BOOST_CHECK_EQUAL( std::max( oval.start.x, oval.end.x ), 4300000 );
        }
        else
        {
            BOOST_CHECK_EQUAL( oval.start.x, 3000000 );
            BOOST_CHECK_EQUAL( oval.end.x, 3000000 );
            BOOST_CHECK_EQUAL( std::min( oval.start.y, oval.end.y ), 1700000 );
            BOOST_CHECK_EQUAL( std::max( oval.start.y, oval.end.y ), 4300000 );
        }
    }
}


BOOST_AUTO_TEST_CASE( KiCadAdapterUsesRoundedRectangleCoreWithoutArcTessellation )
{
    BOARD board;
    auto* footprint = new FOOTPRINT( &board );
    board.Add( footprint );
    auto* pad = new PAD( footprint );
    pad->SetAttribute( PAD_ATTRIB::SMD );
    pad->SetLayerSet( LSET( { F_Cu } ) );
    pad->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::ROUNDRECT );
    pad->SetSize( PADSTACK::ALL_LAYERS, { 3000000, 400000 } );
    pad->SetRoundRectRadiusRatio( PADSTACK::ALL_LAYERS, 0.25 );
    pad->SetPosition( { 3000000, 3000000 } );
    footprint->Add( pad );
    KICAD_BOARD_ADAPTER adapter( &board );
    const auto snapshot = adapter.CreateSnapshot( adapter.CreateDefaultSettings() );
    BOOST_REQUIRE( snapshot );
    BOOST_REQUIRE_EQUAL( snapshot->obstacles.size(), 1 );
    const auto& rounded = snapshot->obstacles.front();
    BOOST_CHECK( rounded.kind == ROUTER_OBSTACLE_KIND::POLYGON );
    BOOST_REQUIRE_EQUAL( rounded.polygon.size(), 4 );
    BOOST_CHECK_EQUAL( rounded.radius, 100000 );
    BOOST_CHECK_EQUAL( rounded.polygon[0].x, 1600000 );
    BOOST_CHECK_EQUAL( rounded.polygon[0].y, 2900000 );
    BOOST_CHECK_EQUAL( rounded.polygon[2].x, 4400000 );
    BOOST_CHECK_EQUAL( rounded.polygon[2].y, 3100000 );
}


BOOST_AUTO_TEST_CASE( FanoutPinOrderMatchesPinnedJavaComponents )
{
    std::ifstream input( KI_TEST::GetPcbnewTestDataDir() + "/autorouter/fanout-search-a11c0a42.txt" );
    BOOST_REQUIRE( input.good() );
    std::string tag;
    int count = 0;
    while( input >> tag )
    {
        BOOST_REQUIRE_EQUAL( tag, "FANOUT" );
        int order, size;
        input >> order >> size;
        BOARD_SNAPSHOT board;
        for( int i = 0; i < size; ++i )
        {
            ROUTING_PAD pin;
            int smd;
            input >> pin.componentId >> pin.pinIndex >> pin.netCode >> smd >> pin.position.x >> pin.position.y;
            pin.isSmd = smd != 0;
            pin.layers = smd ? std::vector<int>{ 0 } : std::vector<int>{ 0, 1 };
            board.pads.push_back( pin );
        }
        int outputSize;
        input >> outputSize;
        std::vector<std::size_t> expected( outputSize );
        for( auto& index : expected )
            input >> index;
        const auto actual = BATCH_FANOUT::OrderedPins( board, static_cast<FANOUT_PIN_ORDER>( order ) );
        BOOST_TEST_CONTEXT( "Java fanout record " << count )
        {
            BOOST_REQUIRE( input.good() );
            BOOST_CHECK_EQUAL_COLLECTIONS( actual.begin(), actual.end(), expected.begin(), expected.end() );
        }
        ++count;
    }
    BOOST_CHECK_EQUAL( count, 640 );
    auto board = makeBoard();
    board.pads[0].isSmd = true; board.pads[0].layers = { 0 };
    BOOST_CHECK( BATCH_FANOUT::OrderedPins( board, FANOUT_PIN_ORDER::OUTER_FIRST, [] { return true; } ).empty() );
}

BOOST_AUTO_TEST_CASE( FanoutLandingPlanningUsesGlobalComponentOrderNotNetOrder )
{
    BOARD_SNAPSHOT board;
    board.bounds = { 0, 0, 20000000, 20000000 };
    for( int i = 0; i < 4; ++i )
    {
        ROUTING_PAD pin;
        pin.netCode = i == 0 || i == 3 ? 1 : 2;
        pin.position = { 4000000 + ( i % 2 ) * 10000000, 4000000 + ( i / 2 ) * 10000000 };
        pin.layers = { 0 }; pin.isSmd = true;
        pin.trackWidth = 100000; pin.radius = 100000;
        pin.componentId = i < 3 ? 1 : 2; pin.pinIndex = i;
        board.pads.push_back( pin );
    }
    ROUTING_NET a; a.netCode = 1; a.viaDiameter = 300000; a.viaDrill = 150000;
    a.padIndices = { 0, 3 }; a.connections = { { 0, 3 } };
    ROUTING_NET b; b.netCode = 2; b.viaDiameter = 300000; b.viaDrill = 150000;
    b.padIndices = { 1, 2 }; b.connections = { { 1, 2 } };
    board.nets = { a, b };
    auto settings = makeSettings(); settings.enableFanout = true;
    const auto expected = BATCH_FANOUT::OrderedPins( board, settings.fanoutPinOrder );
    const auto prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_REQUIRE_EQUAL( prepared.pads.size(), 8 );
    for( std::size_t i = 0; i < expected.size(); ++i )
        BOOST_CHECK_EQUAL( prepared.pads[4 + i].fanoutSourcePadIndex, expected[i] );
    settings.maxFanoutPasses = 1;
    const auto onePass = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_REQUIRE_EQUAL( onePass.pads.size(), prepared.pads.size() );
    for( std::size_t i = 4; i < onePass.pads.size(); ++i )
        BOOST_CHECK( onePass.pads[i].position == prepared.pads[i].position );
    // Every source bridge survived the single graph rewrite, including nets
    // interleaved by component/pin priority.
    for( const auto& net : prepared.nets )
    {
        BOOST_CHECK_EQUAL( net.connections.size(), 3 );
        for( auto pin : net.padIndices )
            BOOST_CHECK( std::any_of( net.connections.begin(), net.connections.end(),
                    [&]( const auto& edge ) { return edge.first == pin
                        && prepared.pads[edge.second].fanoutSourcePadIndex == pin; } ) );
    }
}

BOOST_AUTO_TEST_CASE( FanoutUsesPhysicalPadLayersAndDistinctPackagePinIndices )
{
    BOARD board;
    board.SetCopperLayerCount( 2 );
    auto* net = new NETINFO_ITEM( &board, "N", 1 ); board.Add( net );
    auto* footprint = new FOOTPRINT( &board ); board.Add( footprint );
    for( int i = 0; i < 3; ++i )
    {
        auto* pad = new PAD( footprint );
        pad->SetNet( net ); pad->SetNumber( "1" ); // deliberately repeated host label
        pad->SetPosition( { 1000000 + i * 2000000, 1000000 } );
        pad->SetSize( PADSTACK::ALL_LAYERS, { 1000000, 1000000 } );
        pad->SetAttribute( i == 0 ? PAD_ATTRIB::PTH : PAD_ATTRIB::SMD );
        pad->SetLayerSet( i == 0 ? LSET::AllCuMask() : LSET( { F_Cu } ) );
        footprint->Add( pad );
    }
    board.BuildConnectivity();
    KICAD_BOARD_ADAPTER adapter( &board );
    auto settings = adapter.CreateDefaultSettings();
    for( auto& layer : settings.layers )
        layer.enabled = layer.layerId == F_Cu;
    const auto snapshot = adapter.CreateSnapshot( settings );
    BOOST_REQUIRE( snapshot );
    BOOST_REQUIRE_EQUAL( snapshot->pads.size(), 3 );
    std::set<int> pins;
    int smdCount = 0;
    for( const auto& pad : snapshot->pads )
    {
        BOOST_CHECK_EQUAL( pad.componentId, 1 );
        pins.insert( pad.pinIndex );
        if( pad.isSmd ) ++smdCount;
    }
    BOOST_CHECK_EQUAL( pins.size(), 3 );
    BOOST_CHECK_EQUAL( smdCount, 2 );
}

BOOST_AUTO_TEST_CASE( NormalContactsMatchPinnedJavaItems )
{
    std::ifstream input( KI_TEST::GetPcbnewTestDataDir() + "/autorouter/contacts-search-a11c0a42.txt" );
    BOOST_REQUIRE( input.good() );
    std::string tag;
    int count = 0;
    while( input >> tag )
    {
        BOOST_REQUIRE_EQUAL( tag, "CONTACT" );
        NORMAL_CONTACT_ITEM items[2];
        ROUTING_OBSTACLE areas[2];
        int nets[2];
        for( int i = 0; i < 2; ++i )
        {
            int kind, layer;
            auto& item = items[i];
            input >> kind >> nets[i] >> layer >> item.first.x >> item.first.y >> item.last.x >> item.last.y;
            item.kind = kind == 0 ? NORMAL_CONTACT_ITEM::KIND::TRACE
                      : kind == 1 ? NORMAL_CONTACT_ITEM::KIND::DRILL : NORMAL_CONTACT_ITEM::KIND::AREA;
            item.layers = { layer };
            auto& area = areas[i];
            area.box = { item.first.x, item.first.y, item.last.x, item.last.y };
            area.polygonHoles = { { { item.first.x + 25, item.first.y + 25 },
                                   { item.last.x - 25, item.first.y + 25 },
                                   { item.last.x - 25, item.last.y - 25 },
                                   { item.first.x + 25, item.last.y - 25 } } };
        }
        bool left, right;
        input >> std::boolalpha >> left >> right;
        auto contains = [&]( ROUTER_POINT p )
        { return CONTACT_GEOMETRY::ContainsArea( areas[items[0].kind == NORMAL_CONTACT_ITEM::KIND::AREA ? 0 : 1], p ); };
        BOOST_TEST_CONTEXT( "Java contact record " << count )
        {
            BOOST_CHECK_EQUAL( nets[0] == nets[1] && items[0].Touches( items[1], contains ), left );
            BOOST_CHECK_EQUAL( nets[0] == nets[1] && items[1].Touches( items[0], contains ), right );
            for( int side = 0; side < 2; ++side )
            {
                int present;
                input >> present;
                const auto actual = items[side].Point( items[1 - side] );
                BOOST_CHECK_EQUAL( actual.has_value(), present != 0 );
                if( present )
                {
                    ROUTER_POINT expected;
                    input >> expected.x >> expected.y;
                    BOOST_REQUIRE( actual );
                    BOOST_CHECK( *actual == expected );
                }
            }
            BOOST_REQUIRE( input.good() );
        }
        ++count;
    }
    BOOST_CHECK_EQUAL( count, 2048 );
}

BOOST_AUTO_TEST_CASE( OptimizerJunctionsRejectSubIuOffLineContacts )
{
    auto board = makeBoard();
    auto pad = board.pads[0];
    pad.position = { 1500001, 500000 }; // 0.316 IU from the queried centre-line
    board.pads.push_back( pad );
    pad.position = { 1500000, 500000 }; // actual contact, must be kept
    board.pads.push_back( pad );
    ROUTING_BOARD copper( board, makeSettings() );
    const auto points = copper.TraceJunctions( 1, { { 0, 0 }, 0 }, { { 3000000, 1000000 }, 0 } );
    BOOST_REQUIRE_EQUAL( points.size(), 1 );
    BOOST_CHECK( points[0] == pad.position );
}

BOOST_AUTO_TEST_CASE( ExactJunctionGeometryNeverRoundsOrOverflows )
{
    using namespace CONTACT_GEOMETRY;
    BOOST_CHECK( !Intersection( { 0, 0 }, { 3, 3 }, { 0, 3 }, { 3, 0 } ) );
    const auto p = Intersection( { INT_MIN, INT_MIN }, { INT_MAX, INT_MAX },
                                 { INT_MIN, INT_MAX }, { INT_MAX, INT_MIN } );
    BOOST_CHECK( !p ); // exact crossing is (-0.5,-0.5), not (0,0)
    const auto q = Intersection( { INT_MIN, 0 }, { INT_MAX, 0 },
                                 { 0, INT_MIN }, { 0, INT_MAX } );
    BOOST_REQUIRE( q );
    BOOST_CHECK( ( *q == ROUTER_POINT{ 0, 0 } ) );
    BOOST_CHECK( OnSegment( { INT_MIN, INT_MIN }, { INT_MAX, INT_MAX }, { 0, 0 } ) );
    BOOST_CHECK( !OnSegment( { INT_MIN, INT_MIN }, { INT_MAX, INT_MAX }, { 0, 1 } ) );
}

BOOST_AUTO_TEST_CASE( NormalContactsSplitGeneratedBranchesAndRollbackIdentity )
{
    auto board = makeBoard(); auto settings = makeSettings();
    ROUTING_BOARD copper( board, settings );
    ROUTING_CONNECTION trunk;
    trunk.netCode = 1; trunk.complete = true;
    trunk.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    copper.AddRoute( trunk );
    const auto before = copper.RouteItems( trunk );
    BOOST_REQUIRE_EQUAL( before.size(), 1 );
    ROUTING_CONNECTION branch = trunk;
    branch.nodes = { { { 3000000, 2500000 }, 0 }, { { 3000000, 1500000 }, 0 } };
    std::vector<ROUTING_BOARD::ITEM_ID> split, branchIds;
    {
        ROUTING_BOARD::TRANSACTION transaction( copper );
        copper.AddRoute( branch );
        split = copper.RouteItems( trunk ); branchIds = copper.RouteItems( branch );
        BOOST_REQUIRE_EQUAL( split.size(), 2 );
        BOOST_CHECK_EQUAL( split.front(), before.front() );
        BOOST_REQUIRE_EQUAL( branchIds.size(), 1 );
        BOOST_CHECK( copper.GetNormalContacts( branchIds.front() ).contains( split[0] ) );
        BOOST_CHECK( copper.GetNormalContacts( branchIds.front() ).contains( split[1] ) );
        const auto p = copper.NormalContactPoint( split[0], branchIds[0] );
        BOOST_REQUIRE( p );
        BOOST_CHECK( *p == branch.nodes.back().point );
        BOOST_CHECK( copper.NormalConnectedSet( *copper.PadItem( 0 ) ).contains( *copper.PadItem( 1 ) ) );
    }
    BOOST_CHECK( copper.RouteItems( trunk ) == before );
    BOOST_CHECK( copper.RouteItems( branch ).empty() );
    copper.AddRoute( branch );
    BOOST_CHECK( copper.RouteItems( trunk ) == split );
    BOOST_CHECK( copper.RouteItems( branch ) == branchIds );
    copper.RemoveRoute( trunk );
    BOOST_CHECK( copper.GetNormalContacts( branchIds[0] ).empty() );
    // A pad-edge overlap is electrically connected, but NOT a normal contact.
    copper.ClearRoutes();
    trunk.nodes.back().point.x = board.pads[1].position.x - 50000;
    copper.AddRoute( trunk );
    BOOST_CHECK( copper.Connected( 0, 1 ) );
    BOOST_CHECK( !copper.NormalConnectedSet( *copper.PadItem( 0 ) ).contains( *copper.PadItem( 1 ) ) );
}

BOOST_AUTO_TEST_CASE( RetainedTraceContactsSplitVirtuallyAndRestoreOnRejectedInsertion )
{
    auto board = makeBoard(); auto settings = makeSettings();
    ROUTING_OBSTACLE retained;
    retained.kind = ROUTER_OBSTACLE_KIND::SEGMENT; retained.netCode = 1;
    retained.start = board.pads[0].position; retained.end = board.pads[1].position;
    retained.layers = { 0 }; retained.radius = 50000;
    retained.boardItemId = "retained-host-uuid"; retained.isExistingRoute = true;
    board.obstacles.push_back( retained );
    ROUTING_BOARD copper( board, settings );
    ROUTING_CONNECTION branch;
    branch.netCode = 1; branch.complete = true;
    branch.nodes = { { { 3000000, 1500000 }, 0 }, { { 3000000, 2500000 }, 0 } };
    const auto before = copper.NormalConnectedSet( *copper.PadItem( 0 ) );
    const auto count = copper.ItemCount();
    {
        ROUTING_BOARD::TRANSACTION transaction( copper );
        copper.AddRoute( branch );
        BOOST_CHECK_EQUAL( copper.ItemCount(), count + 2 ); // branch plus split half
        const auto ids = copper.RouteItems( branch );
        BOOST_REQUIRE_EQUAL( ids.size(), 1 );
        BOOST_CHECK_EQUAL( copper.GetNormalContacts( ids[0] ).size(), 2 );
        BOOST_CHECK( copper.NormalConnectedSet( *copper.PadItem( 0 ) ).contains( ids[0] ) );
    }
    BOOST_CHECK_EQUAL( copper.ItemCount(), count );
    BOOST_CHECK( copper.NormalConnectedSet( *copper.PadItem( 0 ) ) == before );
    BOOST_CHECK_EQUAL( board.obstacles[0].boardItemId, "retained-host-uuid" );
    BOOST_CHECK( board.obstacles[0].start == retained.start );
    BOOST_CHECK( board.obstacles[0].end == retained.end );
}

BOOST_AUTO_TEST_CASE( InsertionNeverInheritsNegotiatedCrossingPermission )
{
    auto board = makeBoard(); auto settings = makeSettings();
    board.nets.push_back( board.nets[0] ); board.nets.back().netCode = 2;
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU ); occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION victim;
    victim.netCode = 2; victim.complete = true;
    victim.nodes = { { { 3000000, 0 }, 0 }, { { 3000000, 3000000 }, 0 } };
    occupancy.Add( victim );
    int expanded = 0;
    // Even cancellation during a retry leaves the maze's negotiated mode set.
    engine.FindConnection( board.pads[0], board.pads[1], 1, expanded, [] { return true; } );
    ROUTING_CONNECTION candidate;
    candidate.netCode = 1; candidate.complete = true;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    const auto result = FOUND_CONNECTION_INSERTER::Insert( candidate, {}, occupancy, engine );
    BOOST_CHECK( result.state == FOUND_CONNECTION_INSERTER::STATE::BLOCKED );
    BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 1 );
    BOOST_CHECK_EQUAL( occupancy.Connections()[0].netCode, 2 );
}

BOOST_AUTO_TEST_CASE( CheckedInsertionRollsBackRipupUsageContactsAndCancellation )
{
    auto board = makeBoard(); auto settings = makeSettings();
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU ); occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION original;
    original.netCode = 1; original.complete = true;
    original.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    occupancy.Add( original );
    const auto ids = occupancy.Board()->RouteItems( original );
    const auto cells = occupancy.CellsForSegment( original.nodes[0], original.nodes[1] );
    ROUTING_CONNECTION invalid = original;
    invalid.nodes.push_back( { { 8000000, 1500000 }, 0 } ); // later edge leaves outline
    using INSERTER = FOUND_CONNECTION_INSERTER;
    auto verify = [&]
    {
        BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 1 );
        BOOST_CHECK( occupancy.Connections()[0].nodes == original.nodes );
        BOOST_CHECK( occupancy.Board()->RouteItems( original ) == ids );
        BOOST_CHECK( occupancy.Board()->Connected( 0, 1 ) );
        for( const auto& cell : cells )
            BOOST_CHECK_EQUAL( occupancy.Usage( cell, 2 ), 1 );
    };
    auto result = INSERTER::Insert( invalid, { original }, occupancy, engine );
    BOOST_CHECK( result.state == INSERTER::STATE::BLOCKED );
    BOOST_CHECK_EQUAL( result.edge, 2 ); verify();
    for( int cancelAt = 1; cancelAt <= 4; ++cancelAt )
    {
        int calls = 0;
        result = INSERTER::Insert( original, { original }, occupancy, engine,
                                   [&] { return ++calls == cancelAt; } );
        BOOST_CHECK( result.state == INSERTER::STATE::CANCELLED ); verify();
    }
    invalid.nodes.back() = { { 5000000, 1500000 }, 1 };
    invalid.nodes[1].point.x = 4000000; // diagonal via forbidden
    result = INSERTER::Insert( invalid, { original }, occupancy, engine );
    BOOST_CHECK( result.state == INSERTER::STATE::BLOCKED ); verify();
    result = INSERTER::Insert( original, { original }, occupancy, engine );
    BOOST_CHECK( result.state == INSERTER::STATE::INSERTED );
    BOOST_CHECK( occupancy.Board()->Connected( 0, 1 ) );
}

BOOST_AUTO_TEST_CASE( ForcedSpringOverPublishesCheckedReplacementAtomically )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.gridStepIU = 100000;
    ROUTING_OBSTACLE box;
    box.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    box.netCode = 2;
    box.layers = { 0 };
    box.box = { 2700000, 1200000, 3300000, 1800000 };
    board.obstacles.push_back( box );
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION route;
    route.netCode = 1;
    route.complete = true;
    route.fromPadIndex = 0;
    route.toPadIndex = 1;
    route.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    BOOST_CHECK( !engine.CanInsertSegment( 1, route.nodes.front(), route.nodes.back() ) );
    auto inserted = FOUND_CONNECTION_INSERTER::Insert( route, {}, occupancy, engine );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE( inserted.connection );
    BOOST_CHECK( inserted.connection->nodes != route.nodes );
    BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 1 );
    BOOST_CHECK( occupancy.Connections()[0].nodes == inserted.connection->nodes );
    BOOST_CHECK( inserted.connection->nodes.front() == route.nodes.front() );
    BOOST_CHECK( inserted.connection->nodes.back() == route.nodes.back() );
    BOOST_CHECK_EQUAL( inserted.connection->fromPadIndex, 0 );
    BOOST_CHECK_EQUAL( inserted.connection->toPadIndex, 1 );
    BOOST_CHECK( occupancy.Board()->Connected( 0, 1 ) );
    ROUTING_RESULT emitted;
    FOUND_CONNECTION_INSERTER::Append( *inserted.connection, 100000, 300000, 150000, { 0, 1 }, emitted );
    BOOST_CHECK_EQUAL( emitted.segments.size(), inserted.connection->nodes.size() - 1 );
    occupancy.Remove( *inserted.connection );
    BOOST_CHECK( occupancy.Connections().empty() );
    BOOST_CHECK( !occupancy.Board()->Connected( 0, 1 ) );

    // Every cancellation boundary, including after a speculative ripup and
    // after adding replacement geometry, must leave the original IDs intact.
    occupancy.Add( *inserted.connection );
    int polls = 0;
    auto countCancel = [&] { ++polls; return false; };
    const auto success = FOUND_CONNECTION_INSERTER::Insert( route, { *inserted.connection }, occupancy, engine, countCancel );
    BOOST_REQUIRE( success.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE( success.connection );
    const int totalPolls = polls;
    // Reestablish identical allocator state for every cancelled attempt.
    for( int boundary = 1; boundary <= totalPolls; ++boundary )
    {
        const auto beforeIds = occupancy.Board()->RouteItems( *success.connection );
        int calls = 0;
        auto cancelled = FOUND_CONNECTION_INSERTER::Insert( route, { *success.connection }, occupancy, engine,
                                                           [&] { return ++calls == boundary; } );
        BOOST_CHECK( cancelled.state == FOUND_CONNECTION_INSERTER::STATE::CANCELLED );
        BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 1 );
        BOOST_CHECK( occupancy.Connections()[0].nodes == success.connection->nodes );
        BOOST_CHECK( occupancy.Board()->RouteItems( *success.connection ) == beforeIds );
        BOOST_CHECK( occupancy.Board()->Connected( 0, 1 ) );
    }
}

BOOST_AUTO_TEST_CASE( ForcedSpringOverHandlesFixedCircularAndOvalObstacles )
{
    // KiCad snapshots circles and ovals as SEGMENT + radius.  These are the
    // normal representation of round pads, vias and straight copper, so a
    // forced insert must be able to spring around their clearance contour just
    // as it can around a rectangular or polygonal TileShape.
    for( const auto& [start, end] : std::array<std::pair<ROUTER_POINT, ROUTER_POINT>, 2>{
                 std::pair{ ROUTER_POINT{ 5000000, 2000000 },
                            ROUTER_POINT{ 5000000, 2000000 } },
                 std::pair{ ROUTER_POINT{ 4250000, 2000000 },
                            ROUTER_POINT{ 5750000, 2000000 } } } )
    {
        BOOST_TEST_CONTEXT( "fixed " << ( start == end ? "circle" : "oval" ) )
        {
            auto board = makeBoard();
            board.bounds = { 0, 0, 10000000, 4000000 };
            board.pads[0].position = { 1000000, 2000000 };
            board.pads[1].position = { 9000000, 2000000 };
            auto settings = makeSettings();
            settings.layers = { { 0, true, 1, 20 } };
            settings.allowVias = false;

            ROUTING_OBSTACLE obstacle;
            obstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
            obstacle.netCode = 2;
            obstacle.layers = { 0 };
            obstacle.start = start;
            obstacle.end = end;
            obstacle.radius = 400000;
            obstacle.blocksTracks = true;
            obstacle.blocksVias = true;
            board.obstacles.push_back( obstacle );

            ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
            occupancy.InitializeBoard( board, settings );
            MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
            ROUTING_CONNECTION route;
            route.netCode = 1;
            route.complete = true;
            route.fromPadIndex = 0;
            route.toPadIndex = 1;
            route.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };

            BOOST_REQUIRE( !engine.CanInsertSegment( route.netCode, route.nodes.front(),
                                                      route.nodes.back() ) );
            const auto wrapped = engine.SpringOverConnection( route, {} );
            BOOST_REQUIRE( wrapped );
            BOOST_CHECK_GT( wrapped->nodes.size(), route.nodes.size() );
            BOOST_CHECK( wrapped->nodes.front() == route.nodes.front() );
            BOOST_CHECK( wrapped->nodes.back() == route.nodes.back() );
            for( std::size_t index = 1; index < wrapped->nodes.size(); ++index )
            {
                BOOST_CHECK( engine.CanInsertSegment( wrapped->netCode,
                                                      wrapped->nodes[index - 1],
                                                      wrapped->nodes[index] ) );
            }

            const auto inserted = FOUND_CONNECTION_INSERTER::Insert( route, {}, occupancy, engine );
            BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
            BOOST_REQUIRE( inserted.connection );
            BOOST_CHECK( inserted.connection->nodes == wrapped->nodes );
            BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 1U );
        }
    }
}


BOOST_AUTO_TEST_CASE( ConfiguredWholeConnectionNeckRetryUsesNarrowOutputCopper )
{
    // This is the RouterSettings.neckWidthUm path, not a pad-local
    // getTraceNeckdownHalfwidth escape.  The pair of board-edge walls leaves
    // a 80-IU channel: the ordinary 100-IU trace cannot enter it, while a
    // 50-IU retry can.  The proposed copper must retain the narrow style all
    // the way through materialization rather than silently reverting to the
    // net's ordinary width.
    auto board = makeBoard();
    board.minimumTrackWidth = 0;
    board.nets[0].clearance = 0;

    ROUTING_OBSTACLE topWall;
    topWall.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    topWall.netCode = 2;
    topWall.layers = { 0 };
    topWall.box = { 1500000, 0, 4500000, 1460000 };
    ROUTING_OBSTACLE bottomWall = topWall;
    bottomWall.box = { 1500000, 1540000, 4500000, 3000000 };
    board.obstacles = { topWall, bottomWall };

    auto settings = makeSettings();
    settings.layers = { { 0, true, 1, 20 } };
    settings.allowVias = false;
    settings.enableFanout = false;
    settings.optimizeAfterComplete = false;
    settings.maxPasses = 1;
    // The worker may otherwise choose a longer, congestion-avoiding path
    // around the static via before forced insertion runs. This fixture is an
    // insertion/provenance test, so select the direct source-valid route.
    settings.congestionCost = 0;
    settings.maxIterations = 1;
    settings.maxExpandedNodes = 50000;

    const ROUTING_RESULT normal = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_CHECK( !normal.complete );
    BOOST_CHECK_EQUAL( normal.metrics.routedConnections, 0 );

    settings.neckWidthIU = 50000;
    const ROUTING_RESULT necked = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( necked.complete );
    BOOST_REQUIRE_EQUAL( necked.metrics.routedConnections, 1 );
    BOOST_REQUIRE( !necked.segments.empty() );
    BOOST_CHECK( std::all_of( necked.segments.begin(), necked.segments.end(),
                              []( const ROUTING_SEGMENT& aSegment )
                              { return aSegment.width == 50000; } ) );
    BOOST_CHECK_EQUAL( DESIGN_RULES_CHECKER::CountViolations( board, settings, necked ), 0 );
}


BOOST_AUTO_TEST_CASE( ForcedTerminalNeckdownUsesSourcePinWidthAndEmitsPerEdgeCopper )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.minimumTrackWidth = 0;
    // Pin.getTraceNeckdownHalfwidth(40000) is 19999 source units, hence a
    // 39998-wide final segment. The ordinary net width is 100000.
    board.pads[1].layerGeometry.push_back( { 0, 40000, 100000, 0 } );
    ROUTING_OBSTACLE nearPin;
    nearPin.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    nearPin.netCode = 2;
    nearPin.layers = { 0 };
    // A 100000-wide trace cannot pass beneath this box, while the source
    // pin-entry width can. Its position makes the normal prefix stop before
    // the obstacle and exercises the partial-forcing reconstruction.
    nearPin.box = { 4500000, 1540000, 4750000, 1800000 };
    board.obstacles.push_back( nearPin );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION route;
    route.netCode = 1;
    route.complete = true;
    route.fromPadIndex = 0;
    route.toPadIndex = 1;
    route.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    BOOST_CHECK( !engine.CanInsertSegment( 1, route.nodes.front(), route.nodes.back() ) );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert( route, {}, occupancy, engine );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE( inserted.connection );
    BOOST_REQUIRE_EQUAL( inserted.connection->nodes.size(), 3 );
    BOOST_REQUIRE_EQUAL( inserted.connection->edgeStyles.size(), 2 );
    BOOST_CHECK_EQUAL( inserted.connection->edgeStyles[0].trackWidth, 0 );
    BOOST_CHECK_EQUAL( inserted.connection->edgeStyles[1].trackWidth, 39998 );
    BOOST_CHECK_LT( inserted.connection->nodes[1].point.x, nearPin.box.minX );
    BOOST_CHECK_EQUAL( occupancy.Connections().size(), 1 );

    ROUTING_RESULT emitted;
    FOUND_CONNECTION_INSERTER::Append( *inserted.connection, 100000, 300000, 150000,
                                       { 0, 1 }, emitted );
    BOOST_REQUIRE_EQUAL( emitted.segments.size(), 2 );
    BOOST_CHECK_EQUAL( emitted.segments[0].width, 100000 );
    BOOST_CHECK_EQUAL( emitted.segments[1].width, 39998 );
    const auto items = occupancy.Board()->RouteItems( *inserted.connection );
    BOOST_REQUIRE_EQUAL( items.size(), 2 );
    occupancy.Remove( *inserted.connection );
    BOOST_CHECK( occupancy.Connections().empty() );
}

BOOST_AUTO_TEST_CASE( ForcedTerminalNeckdownUsesSourceStartPinForLongTerminalEdge )
{
    // The source invokes tryNeckDown with a reversed segment at the start
    // pin.  Its pin-distance check therefore validates the selected pin
    // endpoint, not the four-millimetre terminal edge.  Keeping the old
    // native whole-edge guard made this exact mirror of the end-pin case
    // fall back to an unnecessarily large spring-over.
    auto board = makeBoard();
    auto settings = makeSettings();
    board.minimumTrackWidth = 0;
    board.pads[0].layerGeometry.push_back( { 0, 40000, 100000, 0 } );
    ROUTING_OBSTACLE nearPin;
    nearPin.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    nearPin.netCode = 2;
    nearPin.layers = { 0 };
    nearPin.box = { 1250000, 1540000, 1500000, 1800000 };
    board.obstacles.push_back( nearPin );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION route;
    route.netCode = 1;
    route.complete = true;
    route.fromPadIndex = 0;
    route.toPadIndex = 1;
    route.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    BOOST_CHECK( !engine.CanInsertSegment( 1, route.nodes.front(), route.nodes.back() ) );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert( route, {}, occupancy, engine );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE( inserted.connection );
    BOOST_REQUIRE_EQUAL( inserted.connection->nodes.size(), 3U );
    BOOST_REQUIRE_EQUAL( inserted.connection->edgeStyles.size(), 2U );
    BOOST_CHECK_EQUAL( inserted.connection->edgeStyles[0].trackWidth, 39998 );
    BOOST_CHECK_EQUAL( inserted.connection->edgeStyles[1].trackWidth, 0 );
    BOOST_CHECK_GT( inserted.connection->nodes[1].point.x, nearPin.box.maxX );

    ROUTING_RESULT emitted;
    FOUND_CONNECTION_INSERTER::Append( *inserted.connection, 100000, 300000, 150000,
                                       { 0, 1 }, emitted );
    BOOST_REQUIRE_EQUAL( emitted.segments.size(), 2U );
    BOOST_CHECK_EQUAL( emitted.segments[0].width, 39998 );
    BOOST_CHECK_EQUAL( emitted.segments[1].width, 100000 );
}


BOOST_AUTO_TEST_CASE( ForcedTerminalMicroNeckdownUsesFanoutFallbackWidths )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.minimumTrackWidth = 0;
    // This pad is not narrower than the ordinary 100000-IU trace, so the
    // source Pin.getTraceNeckdownHalfwidth path has no usable reduction.
    // The Freerouting fanout fallback must still try its deterministic
    // micro-neckdown widths; 3/4 of the normal trace clears this obstacle.
    board.pads[1].layerGeometry.push_back( { 0, 102000, 102000, 0 } );
    ROUTING_OBSTACLE nearPin;
    nearPin.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    nearPin.netCode = 2;
    nearPin.layers = { 0 };
    nearPin.box = { 4500000, 1540000, 4750000, 1800000 };
    board.obstacles.push_back( nearPin );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION route;
    route.netCode = 1;
    route.complete = true;
    route.fromPadIndex = 0;
    route.toPadIndex = 1;
    route.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    BOOST_CHECK( !engine.CanInsertSegment( route.netCode, route.nodes.front(), route.nodes.back() ) );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert( route, {}, occupancy, engine );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE( inserted.connection );
    BOOST_REQUIRE_EQUAL( inserted.connection->nodes.size(), 3 );
    BOOST_REQUIRE_EQUAL( inserted.connection->edgeStyles.size(), 2 );
    BOOST_CHECK_EQUAL( inserted.connection->edgeStyles[0].trackWidth, 0 );
    BOOST_CHECK_EQUAL( inserted.connection->edgeStyles[1].trackWidth, 75000 );
    BOOST_CHECK_LT( inserted.connection->nodes[1].point.x, nearPin.box.minX );
}

BOOST_AUTO_TEST_CASE( SpringOverPreservesTerminalNeckdownStyleBoundaries )
{
    // The centre edge needs a forced spring-over, while the two terminal
    // edges retain a narrower pin-entry style.  Source TraceShover acts on
    // one homogeneous trace at a time; refusing the whole connection just
    // because the terminal neckdown has a different style loses this valid
    // forced insertion.
    auto board = makeBoard();
    board.bounds = { 0, 0, 10000000, 4000000 };
    board.pads[0].position = { 1000000, 2000000 };
    board.pads[1].position = { 9000000, 2000000 };
    auto settings = makeSettings();
    settings.layers = { { 0, true, 1, 20 } };
    settings.allowVias = false;

    ROUTING_OBSTACLE obstacle;
    obstacle.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    obstacle.netCode = 2;
    obstacle.layers = { 0 };
    obstacle.box = { 4000000, 1500000, 6000000, 2500000 };
    obstacle.blocksTracks = true;
    obstacle.blocksVias = true;
    board.obstacles.push_back( obstacle );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_EDGE_STYLE narrow;
    narrow.trackWidth = 50000;
    ROUTING_EDGE_STYLE normal;
    normal.trackWidth = 100000;
    ROUTING_CONNECTION route;
    route.netCode = 1;
    route.complete = true;
    route.fromPadIndex = 0;
    route.toPadIndex = 1;
    route.nodes = { { board.pads[0].position, 0 }, { { 2000000, 2000000 }, 0 },
                    { { 8000000, 2000000 }, 0 }, { board.pads[1].position, 0 } };
    route.edgeStyles = { narrow, normal, narrow };
    BOOST_CHECK( !engine.CanInsertSegment( route.netCode, route.nodes[1], route.nodes[2],
                                           &route.edgeStyles[1] ) );

    const auto wrapped = engine.SpringOverConnection( route, {} );
    BOOST_REQUIRE( wrapped );
    BOOST_REQUIRE( HasValidEdgeStyles( *wrapped ) );
    BOOST_CHECK_GT( wrapped->nodes.size(), route.nodes.size() );
    BOOST_CHECK( wrapped->nodes.front() == route.nodes.front() );
    BOOST_CHECK( wrapped->nodes.back() == route.nodes.back() );
    BOOST_CHECK_EQUAL( wrapped->edgeStyles.front().trackWidth, narrow.trackWidth );
    BOOST_CHECK_EQUAL( wrapped->edgeStyles.back().trackWidth, narrow.trackWidth );
    BOOST_CHECK_GE( std::count_if( wrapped->edgeStyles.begin(), wrapped->edgeStyles.end(),
                                   [&]( const ROUTING_EDGE_STYLE& aStyle )
                                   { return aStyle.trackWidth == normal.trackWidth; } ),
                    2 );
    for( std::size_t index = 1; index < wrapped->nodes.size(); ++index )
        BOOST_CHECK( engine.CanInsertSegment( wrapped->netCode, wrapped->nodes[index - 1],
                                              wrapped->nodes[index],
                                              &wrapped->edgeStyles[index - 1] ) );
}


BOOST_AUTO_TEST_CASE( ForcedInsertionShovesStaticHostTraceWithoutBorrowingItsConnectivity )
{
    // Whole-net reroute keeps eligible source copper in occupancy as a static
    // collision participant.  It must not count as worker electrical copper
    // (otherwise the proposal could delete an old trace it never regenerated),
    // but a checked forced spring-over may promote it into new proposal
    // copper rather than deleting it.
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_PAD first = board.pads.front();
    first.netCode = 2;
    first.position = { 3000000, 500000 };
    ROUTING_PAD second = first;
    second.position = { 3000000, 2500000 };
    const std::size_t firstIndex = board.pads.size();
    board.pads.push_back( first );
    const std::size_t secondIndex = board.pads.size();
    board.pads.push_back( second );
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices = { firstIndex, secondIndex };
    foreign.connections = { { firstIndex, secondIndex } };
    board.nets.push_back( foreign );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION hostTrace;
    hostTrace.netCode = 2;
    hostTrace.complete = true;
    hostTrace.isExistingBoardRoute = true;
    hostTrace.isShoveMovable = true;
    hostTrace.sourceBoardItemIds = { "host-straight-trace" };
    hostTrace.nodes = { { first.position, 0 }, { second.position, 0 } };
    occupancy.AddStatic( hostTrace );

    // Static copper blocks the candidate, but does not make its two pads
    // electrically connected in the worker routing board.
    BOOST_CHECK_EQUAL( occupancy.Board()->CountMissing( board.nets.back() ), 1 );

    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.complete = true;
    candidate.fromPadIndex = 0;
    candidate.toPadIndex = 1;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    const auto conflicts = engine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( conflicts.size(), 1U );
    BOOST_CHECK( SameRouteGeometry( conflicts.front(), hostTrace ) );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert(
            candidate, conflicts, occupancy, engine, {}, false );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE_EQUAL( inserted.shoved.size(), 1U );
    const ROUTING_CONNECTION& moved = inserted.shoved.front().replacement;
    BOOST_CHECK( !moved.isExistingBoardRoute );
    BOOST_CHECK( moved.isShoveMovable );
    BOOST_CHECK_EQUAL_COLLECTIONS( moved.sourceBoardItemIds.begin(), moved.sourceBoardItemIds.end(),
                                   hostTrace.sourceBoardItemIds.begin(),
                                   hostTrace.sourceBoardItemIds.end() );
    BOOST_CHECK( moved.nodes != hostTrace.nodes );
    BOOST_CHECK( !occupancy.Board()->RouteItems( moved ).empty() );
    BOOST_CHECK( std::none_of( occupancy.Connections().begin(), occupancy.Connections().end(),
                               []( const ROUTING_CONNECTION& aRoute )
                               { return aRoute.isExistingBoardRoute; } ) );

    // A host route that failed the snapshot/contact eligibility guard may
    // never fall through to an ordinary rip-up, even if the caller allows
    // destructive fallback for generated worker routes.
    ROUTING_OCCUPANCY fixedOccupancy( settings.gridStepIU );
    fixedOccupancy.InitializeBoard( board, settings );
    hostTrace.isShoveMovable = false;
    fixedOccupancy.AddStatic( hostTrace );
    MAZE_SEARCH_ENGINE fixedEngine( board, settings, fixedOccupancy );
    const auto fixedConflicts = fixedEngine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( fixedConflicts.size(), 1U );
    const auto rejected = FOUND_CONNECTION_INSERTER::Insert(
            candidate, fixedConflicts, fixedOccupancy, fixedEngine, {}, true );
    BOOST_CHECK( rejected.state == FOUND_CONNECTION_INSERTER::STATE::BLOCKED );
    BOOST_REQUIRE_EQUAL( fixedOccupancy.Connections().size(), 1U );
    BOOST_CHECK( fixedOccupancy.Connections().front().isExistingBoardRoute );
}


BOOST_AUTO_TEST_CASE( RepairPassCanRipUpAutorouterOwnedHostCopper )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_PAD first = board.pads.front();
    first.netCode = 2;
    first.position = { 3000000, 500000 };
    ROUTING_PAD second = first;
    second.position = { 3000000, 2500000 };
    const std::size_t firstIndex = board.pads.size();
    board.pads.push_back( first );
    const std::size_t secondIndex = board.pads.size();
    board.pads.push_back( second );
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices = { firstIndex, secondIndex };
    foreign.connections = { { firstIndex, secondIndex } };
    board.nets.push_back( foreign );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION ownedTrace;
    ownedTrace.netCode = 2;
    ownedTrace.complete = true;
    ownedTrace.fromPadIndex = firstIndex;
    ownedTrace.toPadIndex = secondIndex;
    ownedTrace.isExistingBoardRoute = true;
    ownedTrace.isShoveMovable = false;
    ownedTrace.sourceBoardItemIds = { "job-owned-track" };
    ownedTrace.nodes = { { first.position, 0 }, { second.position, 0 } };
    ownedTrace.isAutorouterOwned = true;
    occupancy.Add( ownedTrace );

    // Unlike protected source copper, a job-owned host item remains genuine
    // worker-board copper: it satisfies connectivity until the transaction
    // removes it, and ordinary negotiated-congestion rip-up may replace it.
    BOOST_CHECK_EQUAL( occupancy.Board()->CountMissing( board.nets.back() ), 0 );

    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.complete = true;
    candidate.fromPadIndex = 0;
    candidate.toPadIndex = 1;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    const auto conflicts = engine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( conflicts.size(), 1U );
    BOOST_CHECK( conflicts.front().isAutorouterOwned );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert(
            candidate, conflicts, occupancy, engine, {}, true );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_CHECK_EQUAL( occupancy.Board()->CountMissing( board.nets.back() ), 1 );
    BOOST_CHECK_EQUAL( occupancy.Board()->CountMissing( board.nets.front() ), 0 );
    BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 1U );
    BOOST_CHECK( SameRouteGeometry( occupancy.Connections().front(), candidate ) );
}


BOOST_AUTO_TEST_CASE( ForcedInsertionShovesStaticHostViaWithAttachedTraceLegs )
{
    // The adapter reconstructs a host via only when it has exactly one
    // direct trace contact on each layer.  Model that composite here: its
    // source UUIDs must survive the move so proposal acceptance removes all
    // three original BOARD_ITEMs before adding the replacement legs/via.
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION hostVia;
    hostVia.netCode = 2;
    hostVia.complete = true;
    hostVia.isExistingBoardRoute = true;
    hostVia.isShoveMovable = true;
    hostVia.sourceBoardItemIds = { "host-trace-top", "host-via", "host-trace-bottom" };
    hostVia.nodes = { { { 2500000, 750000 }, 0 }, { { 3000000, 1500000 }, 0 },
                      { { 3000000, 1500000 }, 1 }, { { 3500000, 2250000 }, 1 } };
    occupancy.AddStatic( hostVia );

    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.complete = true;
    candidate.fromPadIndex = 0;
    candidate.toPadIndex = 1;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    const auto conflicts = engine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( conflicts.size(), 1U );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert(
            candidate, conflicts, occupancy, engine, {}, false );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE_EQUAL( inserted.shoved.size(), 1U );
    const ROUTING_CONNECTION& moved = inserted.shoved.front().replacement;
    BOOST_CHECK( !moved.isExistingBoardRoute );
    BOOST_CHECK_EQUAL_COLLECTIONS( moved.sourceBoardItemIds.begin(), moved.sourceBoardItemIds.end(),
                                   hostVia.sourceBoardItemIds.begin(),
                                   hostVia.sourceBoardItemIds.end() );
    BOOST_REQUIRE( HasValidEdgeStyles( moved ) );
    const auto transition = std::find_if(
            moved.nodes.begin() + 1, moved.nodes.end(),
            []( const ROUTER_NODE& aNode ) { return aNode.layer == 1; } );
    BOOST_REQUIRE( transition != moved.nodes.end() );
    BOOST_CHECK( ( transition - 1 )->point != hostVia.nodes[1].point );

    ROUTING_RESULT emitted;
    FOUND_CONNECTION_INSERTER::Append( moved, 100000, 300000, 150000, { 0, 1 }, emitted );
    BOOST_CHECK_GE( emitted.segments.size(), 2U );
    BOOST_REQUIRE_EQUAL( emitted.vias.size(), 1U );
    BOOST_CHECK( emitted.vias.front().position != hostVia.nodes[1].point );
}


BOOST_AUTO_TEST_CASE( ForcedInsertionMaterializesStaticViaTraceContactsAndBridges )
{
    // DrillItem.moveBy() does not drag a contacted trace. It leaves that
    // source trace in place and creates an old-via-centre -> new-via-centre
    // bridge on the trace layer. Model one lower-layer source trace here: the
    // incoming upper-layer route conflicts only with the via, which lets the
    // test distinguish the new contact-graph plan from the older
    // trace-via-trace path reconstruction.
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION hostTrace;
    hostTrace.netCode = 2;
    hostTrace.complete = true;
    hostTrace.isExistingBoardRoute = true;
    // The trace itself need not be spring-over movable: source DrillItem.moveBy
    // retains its exact centreline and only materialises it for proposal output.
    hostTrace.isShoveMovable = false;
    hostTrace.sourceBoardItemIds = { "host-bottom-trace" };
    hostTrace.nodes = { { { 2500000, 500000 }, 1 }, { { 3000000, 1500000 }, 1 } };
    hostTrace.edgeStyles = { { 100000, 0, 0, 0, {} } };
    occupancy.AddStatic( hostTrace );

    // A branch is not a two-sided trace-via-trace chain. The source stores
    // TraceInfo in a set, so both same-style bottom contacts must be
    // materialised but share exactly one bridge trace.
    ROUTING_CONNECTION secondHostTrace = hostTrace;
    secondHostTrace.sourceBoardItemIds = { "host-bottom-trace-branch" };
    secondHostTrace.nodes = { { { 3500000, 500000 }, 1 },
                              { { 3000000, 1500000 }, 1 } };
    occupancy.AddStatic( secondHostTrace );

    ROUTING_CONNECTION hostVia;
    hostVia.netCode = 2;
    hostVia.complete = true;
    hostVia.isExistingBoardRoute = true;
    hostVia.isShoveMovable = true;
    hostVia.sourceBoardItemIds = { "host-via" };
    hostVia.nodes = { { { 3000000, 1500000 }, 0 }, { { 3000000, 1500000 }, 1 } };
    hostVia.edgeStyles = { { 0, 0, 300000, 150000, { 0, 1 } } };
    occupancy.AddStatic( hostVia );

    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.complete = true;
    candidate.fromPadIndex = 0;
    candidate.toPadIndex = 1;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    const auto conflicts = engine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( conflicts.size(), 1U );
    BOOST_CHECK( SameRouteGeometry( conflicts.front(), hostVia ) );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert(
            candidate, conflicts, occupancy, engine, {}, false );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE_EQUAL( inserted.shoved.size(), 1U );
    const auto& shove = inserted.shoved.front();
    BOOST_REQUIRE_EQUAL( shove.materializedContacts.size(), 2U );
    BOOST_REQUIRE_EQUAL( shove.bridges.size(), 1U );
    BOOST_CHECK( !shove.replacement.isExistingBoardRoute );
    BOOST_CHECK( shove.replacement.nodes.front().point != hostVia.nodes.front().point );
    BOOST_CHECK( std::any_of( shove.materializedContacts.begin(), shove.materializedContacts.end(),
                              [&]( const ROUTING_CONNECTION_REPLACEMENT& aContact )
                              { return SameRouteGeometry( aContact.original, hostTrace ); } ) );
    BOOST_CHECK( std::any_of( shove.materializedContacts.begin(), shove.materializedContacts.end(),
                              [&]( const ROUTING_CONNECTION_REPLACEMENT& aContact )
                              { return SameRouteGeometry( aContact.original, secondHostTrace ); } ) );
    BOOST_CHECK( std::all_of( shove.materializedContacts.begin(), shove.materializedContacts.end(),
                              []( const ROUTING_CONNECTION_REPLACEMENT& aContact )
                              { return !aContact.replacement.isExistingBoardRoute; } ) );

    const ROUTING_CONNECTION& bridge = shove.bridges.front();
    BOOST_REQUIRE_EQUAL( bridge.nodes.size(), 2U );
    BOOST_REQUIRE_EQUAL( bridge.edgeStyles.size(), 1U );
    BOOST_CHECK_EQUAL( bridge.nodes.front().layer, 1 );
    BOOST_CHECK_EQUAL( bridge.nodes.back().layer, 1 );
    BOOST_CHECK( bridge.nodes.front().point == hostVia.nodes.front().point );
    BOOST_CHECK( bridge.nodes.back().point == shove.replacement.nodes.front().point );
    BOOST_CHECK_EQUAL( bridge.edgeStyles.front().trackWidth, 100000 );
    BOOST_CHECK( engine.FindConflictingConnections( candidate ).empty() );
    BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 5U );
    BOOST_CHECK( std::none_of( occupancy.Connections().begin(), occupancy.Connections().end(),
                               []( const ROUTING_CONNECTION& aRoute )
                               { return aRoute.isExistingBoardRoute; } ) );

    ROUTING_RESULT emitted;
    FOUND_CONNECTION_INSERTER::Append( candidate, 100000, 300000, 150000, { 0, 1 }, emitted );
    FOUND_CONNECTION_INSERTER::Append( shove.replacement, 100000, 300000, 150000, { 0, 1 },
                                       emitted );
    for( const ROUTING_CONNECTION_REPLACEMENT& contact : shove.materializedContacts )
        FOUND_CONNECTION_INSERTER::Append( contact.replacement, 100000, 300000, 150000,
                                           { 0, 1 }, emitted );
    FOUND_CONNECTION_INSERTER::Append( bridge, 100000, 300000, 150000, { 0, 1 }, emitted );
    BOOST_REQUIRE_EQUAL( emitted.vias.size(), 1U );
    BOOST_REQUIRE_EQUAL( emitted.segments.size(), 4U );
    BOOST_CHECK_EQUAL( DESIGN_RULES_CHECKER::CountViolations( board, settings, emitted ), 0 );
}


BOOST_AUTO_TEST_CASE( StaticViaShoveUsesOnlyExactNormalTraceEndpoints )
{
    // DrillItem.getNormalContacts() intentionally sees a Trace only at one
    // of its endpoints. A same-net trace whose *interior* crosses the via
    // centre is not a normal Java contact, so DrillItem.moveBy() moves the
    // via without materialising that trace or adding a bridge to it. The
    // worker must not turn that geometric overlap into an invented contact
    // edge during source-via reconstruction.
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( std::move( foreign ) );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION throughTrace;
    throughTrace.netCode = 2;
    throughTrace.complete = true;
    throughTrace.isExistingBoardRoute = true;
    throughTrace.sourceBoardItemIds = { "through-trace" };
    throughTrace.nodes = { { { 2500000, 1500000 }, 1 },
                           { { 3500000, 1500000 }, 1 } };
    throughTrace.edgeStyles = { { 100000, 0, 0, 0, {} } };

    ROUTING_CONNECTION sourceVia;
    sourceVia.netCode = 2;
    sourceVia.complete = true;
    sourceVia.isExistingBoardRoute = true;
    sourceVia.isShoveMovable = true;
    sourceVia.sourceBoardItemIds = { "source-via" };
    sourceVia.nodes = { { { 3000000, 1500000 }, 0 },
                        { { 3000000, 1500000 }, 1 } };
    sourceVia.edgeStyles = { { 0, 0, 300000, 150000, { 0, 1 } } };

    ROUTING_CONNECTION incoming;
    incoming.netCode = 1;
    incoming.complete = true;
    incoming.nodes = { { { 1000000, 1500000 }, 0 },
                       { { 5000000, 1500000 }, 0 } };
    occupancy.Add( incoming );

    const auto plan = engine.ShoveViaConnectionPlan(
            sourceVia, { incoming }, { throughTrace, sourceVia }, {} );
    BOOST_REQUIRE( plan );
    BOOST_CHECK( plan->replacement.nodes.front().point != sourceVia.nodes.front().point );
    BOOST_CHECK( plan->materializedContacts.empty() );
    BOOST_CHECK( plan->bridges.empty() );
}


BOOST_AUTO_TEST_CASE( StaticViaShoveUsesOneBridgePerNormalContactLayer )
{
    // DrillItem.TraceInfo's TreeSet comparator keys only by layer. Two
    // normal trace contacts on the same layer are both retained as source
    // copper, but moveBy() creates exactly one old-centre-to-new-centre
    // bridge, using the first stable contact's style.
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( std::move( foreign ) );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    const ROUTER_POINT centre{ 3000000, 1500000 };
    ROUTING_CONNECTION firstTrace;
    firstTrace.netCode = 2;
    firstTrace.complete = true;
    firstTrace.isExistingBoardRoute = true;
    firstTrace.sourceBoardItemIds = { "a-first-layer-one-contact" };
    firstTrace.nodes = { { { 2500000, 500000 }, 1 }, { centre, 1 } };
    firstTrace.edgeStyles = { { 100000, 0, 0, 0, {} } };

    ROUTING_CONNECTION secondTrace = firstTrace;
    secondTrace.sourceBoardItemIds = { "b-second-layer-one-contact" };
    secondTrace.nodes = { { { 3500000, 500000 }, 1 }, { centre, 1 } };
    secondTrace.edgeStyles = { { 160000, 0, 0, 0, {} } };

    ROUTING_CONNECTION sourceVia;
    sourceVia.netCode = 2;
    sourceVia.complete = true;
    sourceVia.isExistingBoardRoute = true;
    sourceVia.isShoveMovable = true;
    sourceVia.sourceBoardItemIds = { "source-via" };
    sourceVia.nodes = { { centre, 0 }, { centre, 1 } };
    sourceVia.edgeStyles = { { 0, 0, 300000, 150000, { 0, 1 } } };

    ROUTING_CONNECTION incoming;
    incoming.netCode = 1;
    incoming.complete = true;
    incoming.nodes = { { { 1000000, 1500000 }, 0 },
                       { { 5000000, 1500000 }, 0 } };
    occupancy.Add( incoming );

    const auto plan = engine.ShoveViaConnectionPlan(
            sourceVia, { incoming }, { firstTrace, secondTrace, sourceVia }, {} );
    BOOST_REQUIRE( plan );
    BOOST_REQUIRE_EQUAL( plan->materializedContacts.size(), 2U );
    BOOST_REQUIRE_EQUAL( plan->bridges.size(), 1U );
    BOOST_REQUIRE_EQUAL( plan->bridges.front().nodes.front().layer, 1 );
    BOOST_REQUIRE_EQUAL( plan->bridges.front().edgeStyles.size(), 1U );
    BOOST_CHECK_EQUAL( plan->bridges.front().edgeStyles.front().trackWidth, 100000 );
}


BOOST_AUTO_TEST_CASE( StaticViaShoveRetriesWhenItsBridgeHitsTransientCopper )
{
    // DrillItemMover.check sees the complete temporary item set before it
    // accepts a via translation. The native representation emits a separate
    // bridge from the old drill centre to its new centre, so that bridge must
    // participate in the same candidate check. Otherwise the nearest via
    // location is returned, then rejected by the outer forced-insertion
    // transaction without trying the next legal projection.
    auto board = makeBoard();
    auto settings = makeSettings();
    for( int netCode : { 2, 3 } )
    {
        ROUTING_NET foreign = board.nets.front();
        foreign.netCode = netCode;
        foreign.name = "N" + std::to_string( netCode );
        foreign.padIndices.clear();
        foreign.connections.clear();
        board.nets.push_back( std::move( foreign ) );
    }

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    const ROUTER_POINT oldCentre{ 3000000, 1500000 };
    ROUTING_CONNECTION contact;
    contact.netCode = 2;
    contact.complete = true;
    contact.isExistingBoardRoute = true;
    contact.sourceBoardItemIds = { "source-bottom-contact" };
    contact.nodes = { { { 2500000, 500000 }, 1 }, { oldCentre, 1 } };
    contact.edgeStyles = { { 100000, 0, 0, 0, {} } };

    ROUTING_CONNECTION sourceVia;
    sourceVia.netCode = 2;
    sourceVia.complete = true;
    sourceVia.isExistingBoardRoute = true;
    sourceVia.isShoveMovable = true;
    sourceVia.sourceBoardItemIds = { "source-via" };
    sourceVia.nodes = { { oldCentre, 0 }, { oldCentre, 1 } };
    sourceVia.edgeStyles = { { 0, 0, 300000, 150000, { 0, 1 } } };

    ROUTING_CONNECTION incoming;
    incoming.netCode = 1;
    incoming.complete = true;
    incoming.nodes = { { { 1000000, 1500000 }, 0 }, { { 5000000, 1500000 }, 0 } };
    // Make the primary collision envelope long enough that a narrow foreign
    // trace can cross the bridge midpoint while remaining clear of both the
    // old and translated via annuli.
    incoming.edgeStyles = { { 1000000, 0, 0, 0, {} } };

    const auto firstPlan = engine.ShoveViaConnectionPlan(
            sourceVia, { incoming }, { contact, sourceVia }, {} );
    BOOST_REQUIRE( firstPlan );
    BOOST_REQUIRE_EQUAL( firstPlan->materializedContacts.size(), 1U );
    BOOST_REQUIRE_EQUAL( firstPlan->bridges.size(), 1U );
    const ROUTING_CONNECTION& firstBridge = firstPlan->bridges.front();
    BOOST_REQUIRE_EQUAL( firstBridge.nodes.size(), 2U );
    BOOST_REQUIRE( firstBridge.nodes.front().point != firstBridge.nodes.back().point );

    // Put a two-IU foreign trace across the middle of the bridge. It remains
    // clear of both via centres (so it does not alter the placement candidate
    // set) but necessarily collides with the 100,000-IU bridge trace.
    const ROUTER_POINT midpoint{
            firstBridge.nodes.front().point.x
                    + ( firstBridge.nodes.back().point.x - firstBridge.nodes.front().point.x )
                              / 2,
            firstBridge.nodes.front().point.y
                    + ( firstBridge.nodes.back().point.y - firstBridge.nodes.front().point.y )
                              / 2 };
    const std::int64_t deltaX = firstBridge.nodes.back().point.x
                                - firstBridge.nodes.front().point.x;
    const std::int64_t deltaY = firstBridge.nodes.back().point.y
                                - firstBridge.nodes.front().point.y;
    ROUTING_CONNECTION lateBlocker;
    lateBlocker.netCode = 3;
    lateBlocker.complete = true;
    if( std::llabs( deltaX ) >= std::llabs( deltaY ) )
    {
        lateBlocker.nodes = { { { midpoint.x, midpoint.y - 1000 }, 1 },
                              { { midpoint.x, midpoint.y + 1000 }, 1 } };
    }
    else
    {
        lateBlocker.nodes = { { { midpoint.x - 1000, midpoint.y }, 1 },
                              { { midpoint.x + 1000, midpoint.y }, 1 } };
    }
    lateBlocker.edgeStyles = { { 2, 0, 0, 0, {} } };

    occupancy.Add( incoming );
    occupancy.Add( lateBlocker );
    const auto originalConflicts = engine.FindConflictingConnections( sourceVia );
    BOOST_REQUIRE_EQUAL( originalConflicts.size(), 1U );
    BOOST_CHECK( SameRouteGeometry( originalConflicts.front(), incoming ) );
    BOOST_REQUIRE( !engine.FindConflictingConnections( firstBridge ).empty() );

    const auto secondPlan = engine.ShoveViaConnectionPlan(
            sourceVia, { incoming, lateBlocker }, { contact, sourceVia }, {} );
    BOOST_REQUIRE( secondPlan );
    BOOST_REQUIRE_EQUAL( secondPlan->bridges.size(), 1U );
    BOOST_CHECK( secondPlan->replacement.nodes.front().point
                 != firstPlan->replacement.nodes.front().point );
    BOOST_CHECK( engine.FindConflictingConnections( secondPlan->bridges.front() ).empty() );

    // Exercise the checked forced-insertion transaction as well. The blocker
    // is immutable host copper: older code returned the first bad bridge, saw
    // this fixed conflict only after planning, and abandoned the entire shove
    // instead of trying the alternate via centre above.
    ROUTING_CONNECTION fixedBlocker = lateBlocker;
    fixedBlocker.isExistingBoardRoute = true;
    fixedBlocker.isShoveMovable = false;
    fixedBlocker.sourceBoardItemIds = { "fixed-bridge-blocker" };
    ROUTING_OCCUPANCY insertionOccupancy( settings.gridStepIU );
    insertionOccupancy.InitializeBoard( board, settings );
    insertionOccupancy.AddStatic( contact );
    insertionOccupancy.AddStatic( sourceVia );
    insertionOccupancy.AddStatic( fixedBlocker );
    MAZE_SEARCH_ENGINE insertionEngine( board, settings, insertionOccupancy );
    const auto insertionConflicts = insertionEngine.FindConflictingConnections( incoming );
    BOOST_REQUIRE_EQUAL( insertionConflicts.size(), 1U );
    BOOST_CHECK( SameRouteGeometry( insertionConflicts.front(), sourceVia ) );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert(
            incoming, insertionConflicts, insertionOccupancy, insertionEngine, {}, false );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE_EQUAL( inserted.shoved.size(), 1U );
    BOOST_REQUIRE_EQUAL( inserted.shoved.front().bridges.size(), 1U );
    BOOST_CHECK( inserted.shoved.front().replacement.nodes.front().point
                 != firstPlan->replacement.nodes.front().point );
    BOOST_CHECK( insertionEngine.FindConflictingConnections( incoming ).empty() );
    BOOST_CHECK( insertionEngine.FindConflictingConnections(
                         inserted.shoved.front().bridges.front() )
                         .empty() );
}


BOOST_AUTO_TEST_CASE( StaticViaShovePreservesConductionAreaContacts )
{
    // DrillItemMover explicitly permits ConductionArea normal contacts. The
    // native snapshot must therefore not freeze every via on a plane layer,
    // but its immutable proposal also cannot move the annulus off the filled
    // island and leave a false electrical completion for host refill to
    // discover later. A broad plane permits the move; the same source via in
    // a tiny island has no legal translated annulus and fails closed.
    const auto makeSourceVia = []
    {
        ROUTING_CONNECTION via;
        via.netCode = 2;
        via.complete = true;
        via.isExistingBoardRoute = true;
        via.isShoveMovable = true;
        via.sourceBoardItemIds = { "plane-contact-via" };
        via.nodes = { { { 3000000, 1500000 }, 0 }, { { 3000000, 1500000 }, 1 } };
        via.edgeStyles = { { 0, 0, 300000, 150000, { 0, 1 } } };
        return via;
    };
    const auto makeIncoming = []
    {
        ROUTING_CONNECTION incoming;
        incoming.netCode = 1;
        incoming.complete = true;
        incoming.nodes = { { { 1000000, 1500000 }, 0 }, { { 5000000, 1500000 }, 0 } };
        return incoming;
    };
    const auto addNet = []( BOARD_SNAPSHOT& aBoard )
    {
        ROUTING_NET foreign = aBoard.nets.front();
        foreign.netCode = 2;
        foreign.name = "N2";
        foreign.padIndices.clear();
        foreign.connections.clear();
        aBoard.nets.push_back( std::move( foreign ) );
    };
    const auto area = []( ROUTER_BOX aBox )
    {
        ROUTING_OBSTACLE result;
        result.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
        result.netCode = 2;
        result.layers = { 1 };
        result.box = aBox;
        return result;
    };

    {
        auto board = makeBoard();
        addNet( board );
        board.conductionAreas.push_back( area( { 2500000, 500000, 3500000, 2500000 } ) );
        auto settings = makeSettings();
        ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
        occupancy.InitializeBoard( board, settings );
        MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
        const ROUTING_CONNECTION sourceVia = makeSourceVia();
        const ROUTING_CONNECTION incoming = makeIncoming();
        occupancy.Add( incoming );

        const auto plan = engine.ShoveViaConnectionPlan( sourceVia, { incoming }, { sourceVia }, {} );
        BOOST_REQUIRE( plan );
        BOOST_CHECK( plan->replacement.nodes.front().point != sourceVia.nodes.front().point );
        BOOST_CHECK( CONTACT_GEOMETRY::ContainsArea( board.conductionAreas.front(),
                                                     plan->replacement.nodes.front().point ) );
    }

    {
        auto board = makeBoard();
        addNet( board );
        // Candidate locations are clearance + 2 IU outside the incoming
        // trace. This small island contains the old annulus but none of the
        // bounded translated candidates.
        board.conductionAreas.push_back( area( { 2950000, 1450000, 3050000, 1550000 } ) );
        auto settings = makeSettings();
        ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
        occupancy.InitializeBoard( board, settings );
        MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
        const ROUTING_CONNECTION sourceVia = makeSourceVia();
        const ROUTING_CONNECTION incoming = makeIncoming();
        occupancy.Add( incoming );

        BOOST_CHECK( !engine.ShoveViaConnectionPlan( sourceVia, { incoming }, { sourceVia }, {} ) );
    }
}


BOOST_AUTO_TEST_CASE( StaticViaShoveRetainsRemovedSameNetVictimDrillClearance )
{
    // FOUND_CONNECTION_INSERTER removes its complete initial conflict set
    // before it asks the via mover for a legal location.  A second source via
    // of the same net can therefore be absent from live occupancy even though
    // its drill still exists on the board.  DrillItemMover's forced-pad check
    // retains that hole-to-hole constraint.  Enumerate every bounded native
    // candidate as an explicitly removed source victim and ensure the plan
    // fails closed instead of placing a replacement drill on one of them.
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION incoming;
    incoming.netCode = 1;
    incoming.complete = true;
    incoming.nodes = { { { 1000000, 1500000 }, 0 }, { { 5000000, 1500000 }, 0 } };
    occupancy.Add( incoming );

    ROUTING_CONNECTION sourceVia;
    sourceVia.netCode = 2;
    sourceVia.complete = true;
    sourceVia.isExistingBoardRoute = true;
    sourceVia.isShoveMovable = true;
    sourceVia.sourceBoardItemIds = { "source-via" };
    sourceVia.nodes = { { { 3000000, 1500000 }, 0 }, { { 3000000, 1500000 }, 1 } };
    sourceVia.edgeStyles = { { 0, 0, 300000, 150000, { 0, 1 } } };

    std::vector<ROUTING_CONNECTION> removedVictims{ sourceVia };
    bool exhaustedCandidates = false;
    std::size_t attemptedPlacements = 0;
    for( ; attemptedPlacements <= 20; ++attemptedPlacements )
    {
        const auto plan = engine.ShoveViaConnectionPlan( sourceVia, { incoming },
                                                          removedVictims, {} );
        if( !plan )
        {
            exhaustedCandidates = true;
            break;
        }

        ROUTING_CONNECTION blocker = sourceVia;
        blocker.sourceBoardItemIds = { "removed-same-net-via-"
                                       + std::to_string( attemptedPlacements ) };
        blocker.nodes = { { plan->replacement.nodes.front().point, 0 },
                          { plan->replacement.nodes.front().point, 1 } };
        removedVictims.push_back( std::move( blocker ) );
    }

    BOOST_CHECK_GT( attemptedPlacements, 0U );
    BOOST_CHECK( exhaustedCandidates );
}


BOOST_AUTO_TEST_CASE( BatchReconstructsSupportedStaticViaTraceContactsForForcedShove )
{
    // Exercise the snapshot -> static occupancy reconstruction path. Net 2
    // deliberately has no ratsnest task, so normal batch routing leaves its
    // source copper static. The via nevertheless has a supported lower-layer
    // direct trace contact and must be marked eligible for a later atomic
    // DrillItem.moveBy-style forced shove rather than being treated as an
    // isolated-via-only special case.
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.allowVias = false;
    settings.allowRipupExisting = true;
    settings.enableFanout = false;
    settings.optimizeAfterComplete = false;
    settings.maxPasses = 1;

    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    // The reference permits a DrillItem with a ConductionArea normal contact.
    // Keep a real filled-region model beside the source via so snapshot
    // reconstruction proves it remains eligible for the constrained
    // area-preserving shove plan rather than freezing every plane-layer via.
    ROUTING_OBSTACLE plane;
    plane.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    plane.netCode = 2;
    plane.layers = { 1 };
    plane.box = { 2500000, 500000, 3500000, 2500000 };
    board.conductionAreas.push_back( std::move( plane ) );

    const auto appendSource = [&]( const std::string& aId, const ROUTER_POINT& aStart,
                                   const ROUTER_POINT& aEnd, std::int64_t aRadius,
                                   std::vector<int> aLayers, bool aHole = false )
    {
        ROUTING_OBSTACLE obstacle;
        obstacle.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
        obstacle.netCode = 2;
        obstacle.layers = std::move( aLayers );
        obstacle.start = aStart;
        obstacle.end = aEnd;
        obstacle.radius = aRadius;
        obstacle.blocksTracks = true;
        obstacle.blocksVias = true;
        obstacle.isExistingRoute = true;
        obstacle.isMovable = true;
        obstacle.isHole = aHole;
        obstacle.boardItemId = aId;
        board.removableExistingRoutes.push_back( std::move( obstacle ) );
    };

    const ROUTER_POINT viaCenter{ 3000000, 1500000 };
    appendSource( "host-bottom-trace", { 2500000, 500000 }, viaCenter, 50000, { 1 } );
    appendSource( "host-via", viaCenter, viaCenter, 150000, { 0 } );
    appendSource( "host-via", viaCenter, viaCenter, 150000, { 1 } );
    appendSource( "host-via", viaCenter, viaCenter, 75000, { 0, 1 }, true );

    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_CHECK_EQUAL( result.metrics.drcViolations, 0 );
    BOOST_CHECK( result.removedBoardItemIds.empty() );

    const auto staticVia = std::find_if(
            result.connections.begin(), result.connections.end(),
            [&]( const ROUTING_CONNECTION& aConnection )
            {
                return aConnection.netCode == 2 && aConnection.isExistingBoardRoute
                       && aConnection.sourceBoardItemIds == std::vector<std::string>{ "host-via" }
                       && aConnection.nodes.size() == 2
                       && aConnection.nodes.front().layer != aConnection.nodes.back().layer;
            } );
    BOOST_REQUIRE( staticVia != result.connections.end() );
    BOOST_CHECK( staticVia->isShoveMovable );

    const auto staticTrace = std::find_if(
            result.connections.begin(), result.connections.end(),
            [&]( const ROUTING_CONNECTION& aConnection )
            {
                return aConnection.netCode == 2 && aConnection.isExistingBoardRoute
                       && aConnection.sourceBoardItemIds
                                  == std::vector<std::string>{ "host-bottom-trace" }
                       && aConnection.nodes.size() == 2
                       && aConnection.nodes.front().point == ROUTER_POINT{ 2500000, 500000 }
                        && aConnection.nodes.back().point == viaCenter;
            } );
    BOOST_REQUIRE( staticTrace != result.connections.end() );
    BOOST_CHECK_EQUAL( DESIGN_RULES_CHECKER::CountViolations( board, settings, result ), 0 );
}


BOOST_AUTO_TEST_CASE( BatchKeepsStaticTraceWithOffCentreInteriorPadContactFixed )
{
    // KiCad tracks need not join a pad at its centre.  A pad whose copper
    // overlaps a trace's interior is a real fixed contact even if its centre
    // is a little off the trace centreline.  Do not promote that source trace
    // to a movable forced-shove route: preserving only its two endpoints
    // would disconnect the pad.
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.allowRipupExisting = true;
    settings.enableFanout = false;
    settings.optimizeAfterComplete = false;
    settings.maxPasses = 1;

    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    ROUTING_PAD interiorPad = board.pads.front();
    interiorPad.netCode = 2;
    interiorPad.position = { 3000000, 1550000 };
    interiorPad.layers = { 0 };
    interiorPad.sourceId = "interior-pad";
    board.pads.push_back( interiorPad );

    ROUTING_OBSTACLE trace;
    trace.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    trace.netCode = 2;
    trace.layers = { 0 };
    trace.start = { 2000000, 1500000 };
    trace.end = { 4000000, 1500000 };
    trace.radius = 50000;
    trace.isExistingRoute = true;
    trace.isMovable = true;
    trace.boardItemId = "host-trace-with-interior-pad";
    board.removableExistingRoutes.push_back( trace );

    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_CHECK( result.removedBoardItemIds.empty() );

    const auto staticTrace = std::find_if(
            result.connections.begin(), result.connections.end(),
            []( const ROUTING_CONNECTION& aConnection )
            {
                return aConnection.isExistingBoardRoute
                       && aConnection.sourceBoardItemIds
                                  == std::vector<std::string>{ "host-trace-with-interior-pad" };
            } );
    BOOST_REQUIRE( staticTrace != result.connections.end() );
    BOOST_CHECK( !staticTrace->isShoveMovable );
}


BOOST_AUTO_TEST_CASE( BatchKeepsStaticTraceWithInteriorLockedViaContactFixed )
{
    // KiCad permits an existing via to land in the middle of an unsplit
    // track.  A locked via is intentionally absent from the movable source
    // atom set, so the old atom-only contact scan promoted the track to a
    // spring-over candidate.  Moving that track preserves its endpoints but
    // disconnects the fixed via.  Source geometry has to participate in the
    // fail-closed contact classification as well.
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.allowRipupExisting = true;
    settings.enableFanout = false;
    settings.optimizeAfterComplete = false;
    settings.maxPasses = 1;

    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    ROUTING_OBSTACLE trace;
    trace.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    trace.netCode = 2;
    trace.layers = { 0 };
    trace.start = { 2000000, 1500000 };
    trace.end = { 4000000, 1500000 };
    trace.radius = 50000;
    trace.isExistingRoute = true;
    trace.isMovable = true;
    trace.boardItemId = "host-trace-with-locked-interior-via";
    board.removableExistingRoutes.push_back( trace );

    // This is the copper on layer 0 of a locked through via.  The layer-1
    // annulus and drill do not need to be modeled for this classification;
    // the physical contact on the trace layer is sufficient to make moving
    // only the trace unsafe.
    ROUTING_OBSTACLE lockedVia;
    lockedVia.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    lockedVia.netCode = 2;
    lockedVia.layers = { 0 };
    lockedVia.start = { 3000000, 1500000 };
    lockedVia.end = lockedVia.start;
    lockedVia.radius = 150000;
    lockedVia.isExistingRoute = true;
    lockedVia.isMovable = false;
    lockedVia.boardItemId = "locked-interior-via";
    board.obstacles.push_back( lockedVia );

    const ROUTING_RESULT result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_CHECK( result.removedBoardItemIds.empty() );

    const auto staticTrace = std::find_if(
            result.connections.begin(), result.connections.end(),
            []( const ROUTING_CONNECTION& aConnection )
            {
                return aConnection.isExistingBoardRoute
                       && aConnection.sourceBoardItemIds
                                  == std::vector<std::string>{
                                          "host-trace-with-locked-interior-via" };
            } );
    BOOST_REQUIRE( staticTrace != result.connections.end() );
    BOOST_CHECK( !staticTrace->isShoveMovable );
}


BOOST_AUTO_TEST_CASE( ForcedInsertionShovesAnIsolatedStaticHostViaWithoutRipup )
{
    // DrillItem.moveBy() is also valid for a via with no normal contacts: it
    // translates the drill and contributes no bridge trace.  The native
    // static-source reconstruction now admits exactly that subset.  Its UUID
    // must survive the transactional promotion to proposal copper so the
    // accepted KiCad proposal replaces (rather than duplicates) the host via.
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION hostVia;
    hostVia.netCode = 2;
    hostVia.complete = true;
    hostVia.isExistingBoardRoute = true;
    hostVia.isShoveMovable = true;
    hostVia.sourceBoardItemIds = { "host-isolated-via" };
    hostVia.nodes = { { { 3000000, 1500000 }, 0 }, { { 3000000, 1500000 }, 1 } };
    hostVia.edgeStyles = { { 0, 0, 300000, 150000, { 0, 1 } } };
    occupancy.AddStatic( hostVia );

    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.complete = true;
    candidate.fromPadIndex = 0;
    candidate.toPadIndex = 1;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    const auto conflicts = engine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( conflicts.size(), 1U );
    BOOST_CHECK( SameRouteGeometry( conflicts.front(), hostVia ) );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert(
            candidate, conflicts, occupancy, engine, {}, false );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE_EQUAL( inserted.shoved.size(), 1U );
    const ROUTING_CONNECTION& moved = inserted.shoved.front().replacement;
    BOOST_CHECK( !moved.isExistingBoardRoute );
    BOOST_CHECK( moved.isShoveMovable );
    BOOST_CHECK_EQUAL_COLLECTIONS( moved.sourceBoardItemIds.begin(), moved.sourceBoardItemIds.end(),
                                   hostVia.sourceBoardItemIds.begin(),
                                   hostVia.sourceBoardItemIds.end() );
    BOOST_REQUIRE_EQUAL( moved.nodes.size(), 2U );
    BOOST_REQUIRE_EQUAL( moved.edgeStyles.size(), 1U );
    BOOST_CHECK( moved.nodes.front().point == moved.nodes.back().point );
    BOOST_CHECK( moved.nodes.front().point != hostVia.nodes.front().point );
    BOOST_CHECK_EQUAL( moved.edgeStyles.front().viaDiameter, 300000 );
    BOOST_CHECK_EQUAL( moved.edgeStyles.front().viaDrill, 150000 );
    BOOST_CHECK( engine.FindConflictingConnections( candidate ).empty() );
    BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 2U );

    ROUTING_RESULT emitted;
    FOUND_CONNECTION_INSERTER::Append( moved, 100000, 300000, 150000, { 0, 1 }, emitted );
    BOOST_REQUIRE_EQUAL( emitted.vias.size(), 1U );
    BOOST_CHECK( emitted.vias.front().position == moved.nodes.front().point );
}


BOOST_AUTO_TEST_CASE( WholeNetRerouteKeepsUnrepresentableExistingCopperAsCollisionOnly )
{
    // A full-net reroute omits removable host copper from the ordinary
    // immutable snapshot.  That is only safe if every omitted BOARD_ITEM is
    // represented by a complete static occupancy route.  Model a
    // tessellated/compound source item with two capsules under one UUID:
    // the bounded host-shove reconstruction must fail closed, retain both
    // collision pieces, and never produce an unsafe proposal through them.
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.layers = { { 0, true, 1, 20 } };
    settings.allowVias = false;
    settings.enableFanout = false;
    settings.optimizeAfterComplete = false;
    settings.maxPasses = 1;
    settings.maxIterations = 1;
    settings.maxExpandedNodes = 10000;
    settings.allowRipupExisting = true;

    ROUTING_NET foreign;
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.netClass = "Default";
    board.nets.push_back( foreign );

    for( const auto [firstY, lastY] : std::array<std::pair<std::int64_t, std::int64_t>, 2>{
                 std::pair{ 0, 1500000 }, std::pair{ 1500000, 3000000 } } )
    {
        ROUTING_OBSTACLE source;
        source.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
        source.netCode = 2;
        source.layers = { 0 };
        source.start = { 3000000, firstY };
        source.end = { 3000000, lastY };
        source.radius = 100000;
        source.isExistingRoute = true;
        // It is deliberately marked as an adapter-eligible straight piece,
        // but two pieces with one UUID cannot be reconstructed as one direct
        // mutable trace. This is the fail-closed item-topology guard.
        source.isMovable = true;
        source.boardItemId = "compound-host-copper";
        board.removableExistingRoutes.push_back( std::move( source ) );
    }

    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( board, settings, {}, {} );

    BOOST_CHECK( !result.complete );
    BOOST_CHECK_EQUAL( result.metrics.routedConnections, 0 );
    BOOST_CHECK_EQUAL( result.metrics.unroutedConnections, 1 );
    BOOST_CHECK_EQUAL( result.metrics.drcViolations, 0 );
    BOOST_CHECK( result.segments.empty() );
    BOOST_CHECK( result.vias.empty() );
    BOOST_CHECK( result.removedBoardItemIds.empty() );
}


BOOST_AUTO_TEST_CASE( ForcedInsertionRelocatesAMutableGeneratedTraceBeforeRipup )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION victim;
    victim.netCode = 2;
    victim.complete = true;
    victim.nodes = { { { 3000000, 500000 }, 0 }, { { 3000000, 2500000 }, 0 } };
    occupancy.Add( victim );

    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.complete = true;
    candidate.fromPadIndex = 0;
    candidate.toPadIndex = 1;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    const auto conflicts = engine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( conflicts.size(), 1 );
    BOOST_CHECK( SameRouteGeometry( conflicts.front(), victim ) );
    const auto directShove = engine.SpringOverConnection( victim, { candidate }, {} );
    BOOST_REQUIRE( directShove );
    BOOST_CHECK( directShove->nodes != victim.nodes );
    {
        ROUTING_OCCUPANCY::TRANSACTION transaction( occupancy );
        occupancy.Remove( victim );
        occupancy.Add( candidate );
        for( std::size_t index = 1; index < directShove->nodes.size(); ++index )
            BOOST_CHECK( engine.CanInsertSegment( directShove->netCode,
                                                  directShove->nodes[index - 1],
                                                  directShove->nodes[index] ) );
    }

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert( candidate, conflicts, occupancy, engine );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE_EQUAL( inserted.shoved.size(), 1 );
    BOOST_CHECK( SameRouteGeometry( inserted.shoved.front().original, victim ) );
    BOOST_CHECK( inserted.shoved.front().replacement.nodes != victim.nodes );
    BOOST_CHECK( inserted.shoved.front().replacement.nodes.front() == victim.nodes.front() );
    BOOST_CHECK( inserted.shoved.front().replacement.nodes.back() == victim.nodes.back() );
    BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 2 );
    BOOST_CHECK( engine.FindConflictingConnections( candidate ).empty() );
    BOOST_CHECK( occupancy.Board()->RouteItems( victim ).empty() );
    BOOST_CHECK( !occupancy.Board()->RouteItems( inserted.shoved.front().replacement ).empty() );

    // BatchAutorouter must still attempt this source-style move after its
    // ordinary rip-up budget is exhausted. The guarded insertion may move
    // every generated victim, but it must never fall through to deleting one.
    ROUTING_OCCUPANCY guardedOccupancy( settings.gridStepIU );
    guardedOccupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE guardedEngine( board, settings, guardedOccupancy );
    guardedOccupancy.Add( victim );
    const auto guardedConflicts = guardedEngine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( guardedConflicts.size(), 1 );
    const auto guarded = FOUND_CONNECTION_INSERTER::Insert(
            candidate, guardedConflicts, guardedOccupancy, guardedEngine, {}, false );
    BOOST_REQUIRE( guarded.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE_EQUAL( guarded.shoved.size(), 1 );
    BOOST_REQUIRE_EQUAL( guardedOccupancy.Connections().size(), 2 );
}

BOOST_AUTO_TEST_CASE( ForcedInsertionConsidersGeneratedTraceChainBeforeRipup )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    for( int netCode : { 2, 3 } )
    {
        ROUTING_NET foreign = board.nets.front();
        foreign.netCode = netCode;
        foreign.name = "N" + std::to_string( netCode );
        foreign.padIndices.clear();
        foreign.connections.clear();
        board.nets.push_back( std::move( foreign ) );
    }

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION primary;
    primary.netCode = 2;
    primary.complete = true;
    primary.nodes = { { { 3000000, 500000 }, 0 }, { { 3000000, 2500000 }, 0 } };
    occupancy.Add( primary );

    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.complete = true;
    candidate.fromPadIndex = 0;
    candidate.toPadIndex = 1;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };

    const auto primaryReplacement = engine.SpringOverConnection( primary, { candidate }, {} );
    BOOST_REQUIRE( primaryReplacement );

    // Construct a second pre-existing generated route that does not collide
    // with the incoming candidate or the original trace, but does collide
    // with the replacement contour. This expresses the source TraceShover
    // recursion rather than relying on a hand-picked corner ordering.
    ROUTING_CONNECTION secondary;
    bool               foundSecondary = false;
    for( std::size_t edge = 1; edge < primaryReplacement->nodes.size() && !foundSecondary;
         ++edge )
    {
        const ROUTER_NODE& first = primaryReplacement->nodes[edge - 1];
        const ROUTER_NODE& last = primaryReplacement->nodes[edge];
        if( first.layer != last.layer || first.point.y != last.point.y )
            continue;

        // A vertical route through the spring-over contour's outermost bend
        // collides with that replacement without entering the incoming
        // candidate's clearance envelope. Unlike a tiny endpoint overlap,
        // both of its tails retain enough room for the child spring-over.
        const std::int64_t outerX = std::min( first.point.x, last.point.x );
        const ROUTER_POINT start{ outerX, board.bounds.minY + 250000 };
        const ROUTER_POINT end{ outerX, board.bounds.maxY - 250000 };
        if( outerX <= board.bounds.minX || outerX >= board.bounds.maxX )
            continue;

        ROUTING_CONNECTION probe;
        probe.netCode = 3;
        probe.complete = true;
        probe.nodes = { { start, first.layer }, { end, first.layer } };

        if( !engine.FindConflictingConnections( probe ).empty() )
            continue;
        {
            ROUTING_OCCUPANCY::TRANSACTION transaction( occupancy );
            occupancy.Add( candidate );
            if( !engine.FindConflictingConnections( probe ).empty() )
                continue;
        }
        {
            ROUTING_OCCUPANCY::TRANSACTION transaction( occupancy );
            occupancy.Remove( primary );
            occupancy.Add( *primaryReplacement );
            if( engine.FindConflictingConnections( probe ).empty() )
                continue;
        }
        secondary = std::move( probe );
        foundSecondary = true;
    }
    BOOST_REQUIRE( foundSecondary );
    occupancy.Add( secondary );

    const auto initialConflicts = engine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( initialConflicts.size(), 1 );
    BOOST_CHECK( SameRouteGeometry( initialConflicts.front(), primary ) );

    // No ordinary rip-up is permitted. The mover must include the second
    // generated trace in its source-style obstacle set rather than deleting
    // it while forcing the primary route around the incoming candidate.
    const auto inserted = FOUND_CONNECTION_INSERTER::Insert(
            candidate, initialConflicts, occupancy, engine, {}, false );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE_GE( inserted.shoved.size(), 1U );
    BOOST_CHECK( std::any_of( inserted.shoved.begin(), inserted.shoved.end(),
                              [&]( const auto& shove )
                              { return SameRouteGeometry( shove.original, primary ); } ) );
    BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 3 );
    BOOST_CHECK( engine.FindConflictingConnections( candidate ).empty() );
}

BOOST_AUTO_TEST_CASE( ForcedInsertionRelocatesAMutableGeneratedViaBeforeRipup )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION victim;
    victim.netCode = 2;
    victim.complete = true;
    // The source DrillItemMover translates an unfixed via and reconnects its
    // attached traces. Use diagonal original legs so fixed-obstacle
    // spring-over cannot solve this case; the via-shove fallback must build
    // its own orthogonal attachment doglegs.
    victim.nodes = { { { 2500000, 750000 }, 0 }, { { 3000000, 1500000 }, 0 },
                     { { 3000000, 1500000 }, 1 }, { { 3500000, 2250000 }, 1 } };
    occupancy.Add( victim );

    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.complete = true;
    candidate.fromPadIndex = 0;
    candidate.toPadIndex = 1;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    const auto conflicts = engine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( conflicts.size(), 1 );
    BOOST_CHECK( SameRouteGeometry( conflicts.front(), victim ) );
    BOOST_CHECK( !engine.SpringOverConnection( victim, { candidate }, {} ) );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert( candidate, conflicts, occupancy,
                                                               engine );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE_EQUAL( inserted.shoved.size(), 1 );
    const ROUTING_CONNECTION& moved = inserted.shoved.front().replacement;
    BOOST_CHECK( moved.nodes.front() == victim.nodes.front() );
    BOOST_CHECK( moved.nodes.back() == victim.nodes.back() );
    std::size_t index = 1;
    while( index < moved.nodes.size() && moved.nodes[index - 1].layer == moved.nodes[index].layer )
        ++index;
    BOOST_REQUIRE_LT( index, moved.nodes.size() );
    BOOST_CHECK( moved.nodes[index - 1].point != victim.nodes[1].point );
    BOOST_CHECK( moved.nodes[index - 1].point == moved.nodes[index].point );
    BOOST_CHECK_NE( moved.nodes[index - 1].layer, moved.nodes[index].layer );
    BOOST_REQUIRE( HasValidEdgeStyles( moved ) );
    BOOST_CHECK( engine.FindConflictingConnections( candidate ).empty() );
    BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 2 );
}


BOOST_AUTO_TEST_CASE( ForcedInsertionShovesGeneratedFanoutCopperBeforeRipup )
{
    // Fanout vias/traces are inserted as unfixed copper by Freerouting. They
    // need the same bounded recursive shove opportunity as an ordinary
    // generated route, but must never be silently discarded if that move
    // fails. The prior native guard rejected this victim before the shove
    // engine could inspect it.
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION fanout;
    fanout.netCode = 2;
    fanout.complete = true;
    fanout.isFanoutConnection = true;
    fanout.nodes = { { { 2500000, 750000 }, 0 }, { { 3000000, 1500000 }, 0 },
                     { { 3000000, 1500000 }, 1 }, { { 3500000, 2250000 }, 1 } };
    occupancy.Add( fanout );

    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.complete = true;
    candidate.fromPadIndex = 0;
    candidate.toPadIndex = 1;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    const auto conflicts = engine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( conflicts.size(), 1U );
    BOOST_CHECK( SameRouteGeometry( conflicts.front(), fanout ) );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert( candidate, conflicts, occupancy,
                                                               engine, {}, false );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE_EQUAL( inserted.shoved.size(), 1U );
    BOOST_CHECK( inserted.shoved.front().replacement.isFanoutConnection );
    BOOST_CHECK( inserted.shoved.front().replacement.nodes != fanout.nodes );
    BOOST_CHECK( engine.FindConflictingConnections( candidate ).empty() );
    BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 2U );
}


BOOST_AUTO_TEST_CASE( GeneratedViaShoveSkipsTransientCollisionAtNearestProjection )
{
    // DrillItemMover checks a translated via against every temporary item in
    // the forced-insertion search tree.  The nearest projection away from the
    // incoming trace can itself be occupied by a second route that did not
    // overlap the *original* via.  It therefore contributes no projection of
    // its own.  The mover must reject that first candidate and continue to a
    // later legal projection instead of returning a plan which the outer
    // transaction immediately rejects.
    auto board = makeBoard();
    auto settings = makeSettings();
    for( int netCode : { 2, 3 } )
    {
        ROUTING_NET foreign = board.nets.front();
        foreign.netCode = netCode;
        foreign.name = "N" + std::to_string( netCode );
        foreign.padIndices.clear();
        foreign.connections.clear();
        board.nets.push_back( std::move( foreign ) );
    }

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION sourceVia;
    sourceVia.netCode = 2;
    sourceVia.complete = true;
    sourceVia.nodes = { { { 3000000, 1500000 }, 0 }, { { 3000000, 1500000 }, 1 } };
    sourceVia.edgeStyles = { { 0, 0, 300000, 150000, { 0, 1 } } };

    ROUTING_CONNECTION incoming;
    incoming.netCode = 1;
    incoming.complete = true;
    incoming.nodes = { { { 1000000, 1500000 }, 0 }, { { 5000000, 1500000 }, 0 } };

    const auto nearest = engine.ShoveViaConnection( sourceVia, { incoming }, {} );
    BOOST_REQUIRE( nearest );
    BOOST_CHECK( nearest->nodes.front().point != sourceVia.nodes.front().point );

    // Place a narrow second trace exactly on the nearest destination.  Its
    // 50,000-IU radius is clear of the old via by the mover's +2-IU tolerance,
    // so it cannot generate an alternative by itself; it only validates that
    // transient-copper collision rechecking is part of candidate selection.
    ROUTING_CONNECTION lateBlocker;
    lateBlocker.netCode = 3;
    lateBlocker.complete = true;
    lateBlocker.nodes = { { { nearest->nodes.front().point.x - 1000,
                              nearest->nodes.front().point.y }, 0 },
                           { { nearest->nodes.front().point.x + 1000,
                              nearest->nodes.front().point.y }, 0 } };
    lateBlocker.edgeStyles = { { 100000, 0, 0, 0, {} } };

    occupancy.Add( incoming );
    occupancy.Add( lateBlocker );
    const auto oldConflicts = engine.FindConflictingConnections( sourceVia );
    BOOST_REQUIRE_EQUAL( oldConflicts.size(), 1U );
    BOOST_CHECK( SameRouteGeometry( oldConflicts.front(), incoming ) );

    const auto moved = engine.ShoveViaConnection( sourceVia, { incoming, lateBlocker }, {} );
    BOOST_REQUIRE( moved );
    BOOST_CHECK( moved->nodes.front().point != nearest->nodes.front().point );
    BOOST_CHECK( engine.FindConflictingConnections( *moved ).empty() );
}


BOOST_AUTO_TEST_CASE( GeneratedViaShoveUsesDiagonalTraceNormalCandidates )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    // These tiny fixed keepouts make the old bounding-box-side projections
    // illegal.  The diagonally incoming trace still leaves a legal normal
    // displacement, as DrillItemMover.tryShoveViaPoints would find from the
    // compensated segment geometry.
    for( const ROUTER_POINT point : { ROUTER_POINT{ 799000, 1500000 },
                                     ROUTER_POINT{ 5201000, 1500000 },
                                     ROUTER_POINT{ 3000000, 299000 },
                                     ROUTER_POINT{ 3000000, 2701000 } } )
    {
        ROUTING_OBSTACLE blocker;
        blocker.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
        blocker.netCode = 3;
        blocker.layers = { 0, 1 };
        blocker.start = blocker.end = point;
        blocker.radius = 100000;
        blocker.blocksTracks = true;
        blocker.blocksVias = true;
        board.obstacles.push_back( std::move( blocker ) );
    }

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION victim;
    victim.netCode = 2;
    victim.complete = true;
    victim.nodes = { { { 2500000, 2000000 }, 0 }, { { 3000000, 1500000 }, 0 },
                     { { 3000000, 1500000 }, 1 }, { { 3500000, 1000000 }, 1 } };

    ROUTING_CONNECTION incoming;
    incoming.netCode = 1;
    incoming.complete = true;
    incoming.nodes = { { { 1000000, 500000 }, 0 }, { { 5000000, 2500000 }, 0 } };
    occupancy.Add( incoming );

    const auto moved = engine.ShoveViaConnection( victim, { incoming }, {} );
    BOOST_REQUIRE( moved );
    BOOST_REQUIRE( HasValidEdgeStyles( *moved ) );

    const auto transition = std::find_if(
            moved->nodes.begin() + 1, moved->nodes.end(),
            []( const ROUTER_NODE& aNode )
            {
                // The test's first via is the only layer change; the actual
                // comparison is performed below using its preceding node.
                return aNode.layer == 1;
            } );
    BOOST_REQUIRE( transition != moved->nodes.end() );
    const std::size_t viaIndex = static_cast<std::size_t>( transition - moved->nodes.begin() );
    BOOST_REQUIRE_GT( viaIndex, 0U );
    const ROUTER_POINT movedVia = moved->nodes[viaIndex - 1].point;
    BOOST_CHECK_NE( movedVia.x, 3000000 );
    BOOST_CHECK_NE( movedVia.y, 1500000 );

    for( std::size_t index = 1; index < moved->nodes.size(); ++index )
        BOOST_CHECK( engine.CanInsertSegment( moved->netCode, moved->nodes[index - 1],
                                              moved->nodes[index], &moved->edgeStyles[index - 1] ) );
}


BOOST_AUTO_TEST_CASE( GeneratedViaShovePreservesLegalDiagonalTraceLegs )
{
    // The movable via's two attached traces are diagonal.  Every Manhattan
    // bend around the nearest legal via centre is blocked, while the direct
    // diagonal legs are clear.  DrillItemMover preserves a legal attached
    // trace geometry; making the worker invent only 90-degree doglegs loses
    // this source-valid shove.
    auto board = makeBoard();
    board.bounds = { 500000, 0, 5500000, 6000000 };
    auto settings = makeSettings();
    ROUTING_NET foreign = board.nets.front();
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.padIndices.clear();
    foreign.connections.clear();
    board.nets.push_back( foreign );

    const auto addBlocker = [&]( ROUTER_POINT aPoint, int aLayer )
    {
        ROUTING_OBSTACLE blocker;
        blocker.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
        blocker.netCode = 3;
        blocker.layers = { aLayer };
        blocker.start = blocker.end = aPoint;
        blocker.radius = 100000;
        blocker.blocksTracks = true;
        blocker.blocksVias = true;
        board.obstacles.push_back( std::move( blocker ) );
    };

    // The incoming horizontal trace makes the nearest legal via centres
    // (3 mm, 2.8 mm) and (3 mm, 3.2 mm).  These blockers cover the two
    // synthetic Manhattan corner choices on each attached layer without
    // touching either direct diagonal leg.
    addBlocker( { 3000000, 1000000 }, 0 );
    addBlocker( { 1000000, 2800000 }, 0 );
    addBlocker( { 1000000, 3200000 }, 0 );
    addBlocker( { 5000000, 2800000 }, 1 );
    addBlocker( { 5000000, 3200000 }, 1 );
    addBlocker( { 3000000, 5000000 }, 1 );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION incoming;
    incoming.netCode = 1;
    incoming.complete = true;
    incoming.nodes = { { { 500000, 3000000 }, 0 }, { { 5500000, 3000000 }, 0 } };
    occupancy.Add( incoming );

    ROUTING_CONNECTION victim;
    victim.netCode = 2;
    victim.complete = true;
    victim.nodes = { { { 1000000, 1000000 }, 0 }, { { 3000000, 3000000 }, 0 },
                     { { 3000000, 3000000 }, 1 }, { { 5000000, 5000000 }, 1 } };

    const auto moved = engine.ShoveViaConnection( victim, { incoming }, {} );
    BOOST_REQUIRE( moved );
    BOOST_REQUIRE( HasValidEdgeStyles( *moved ) );
    BOOST_REQUIRE_EQUAL( moved->nodes.size(), 4U );
    BOOST_CHECK_EQUAL( moved->nodes[1].layer, 0 );
    BOOST_CHECK_EQUAL( moved->nodes[2].layer, 1 );
    BOOST_CHECK( moved->nodes[1].point == moved->nodes[2].point );
    BOOST_CHECK( moved->nodes[1].point != victim.nodes[1].point );
    for( std::size_t index = 1; index < moved->nodes.size(); ++index )
        BOOST_CHECK( engine.CanInsertSegment( moved->netCode, moved->nodes[index - 1],
                                              moved->nodes[index], &moved->edgeStyles[index - 1] ) );
}


BOOST_AUTO_TEST_CASE( ForcedInsertionRelocatesATerminalGeneratedViaBeforeRipup )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.pads[0].position.y = 1800000;
    board.pads[1].position.y = 1800000;
    board.nets[0].clearance = 100000;

    // The source endpoint is a real generated-route terminal rather than a
    // preceding trace edge. Its tiny pad is deliberately outside the incoming
    // trace's clearance envelope; only the adjacent via conflicts, so moving
    // the generated via is electrically meaningful and does not pretend that
    // a fixed host pad itself can be shoved.
    ROUTING_PAD terminal;
    terminal.netCode = 2;
    terminal.position = { 3000000, 1500000 };
    terminal.layers = { 0 };
    terminal.trackWidth = 100000;
    ROUTING_PAD farTerminal = terminal;
    farTerminal.position = { 3500000, 2250000 };
    farTerminal.layers = { 1 };
    board.pads.push_back( terminal );
    board.pads.push_back( farTerminal );

    ROUTING_NET foreign;
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.netClass = "Default";
    foreign.clearance = 100000;
    foreign.viaDiameter = 300000;
    foreign.viaDrill = 150000;
    foreign.padIndices = { 2, 3 };
    foreign.connections = { { 2, 3 } };
    board.nets.push_back( foreign );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );

    ROUTING_CONNECTION victim;
    victim.netCode = 2;
    victim.complete = true;
    victim.fromPadIndex = 2;
    victim.toPadIndex = 3;
    victim.nodes = { { terminal.position, 0 }, { terminal.position, 1 },
                     { farTerminal.position, 1 } };
    occupancy.Add( victim );

    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.complete = true;
    candidate.fromPadIndex = 0;
    candidate.toPadIndex = 1;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    const auto conflicts = engine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( conflicts.size(), 1U );
    BOOST_CHECK( SameRouteGeometry( conflicts.front(), victim ) );

    const auto inserted = FOUND_CONNECTION_INSERTER::Insert( candidate, conflicts, occupancy,
                                                               engine, {}, false );
    BOOST_REQUIRE( inserted.state == FOUND_CONNECTION_INSERTER::STATE::INSERTED );
    BOOST_REQUIRE_EQUAL( inserted.shoved.size(), 1U );
    const ROUTING_CONNECTION& moved = inserted.shoved.front().replacement;
    BOOST_CHECK( moved.nodes.front() == victim.nodes.front() );
    BOOST_CHECK( moved.nodes.back() == victim.nodes.back() );
    BOOST_REQUIRE( HasValidEdgeStyles( moved ) );

    std::size_t transition = 1;
    while( transition < moved.nodes.size()
           && moved.nodes[transition - 1].layer == moved.nodes[transition].layer )
    {
        ++transition;
    }
    BOOST_REQUIRE_LT( transition, moved.nodes.size() );
    BOOST_CHECK( moved.nodes[transition - 1].point != terminal.position );
    BOOST_CHECK( moved.nodes[transition - 1].point == moved.nodes[transition].point );
    // The former direct terminal-via contact is now a real source-layer
    // trace into the moved via; the endpoint pad identity remains intact.
    BOOST_REQUIRE_GT( transition, 1U );
    BOOST_CHECK( moved.nodes.front().point == terminal.position );
    BOOST_CHECK( engine.FindConflictingConnections( candidate ).empty() );
    BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 2U );
}


BOOST_AUTO_TEST_CASE( ForcedViaShoveFailsClosedForAHostPadAnchor )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_PAD terminal;
    terminal.netCode = 2;
    terminal.position = { 3000000, 1500000 };
    terminal.layers = { 0 };
    terminal.trackWidth = 100000;
    terminal.sourceId = "host-pad-uuid";
    ROUTING_PAD farTerminal = terminal;
    farTerminal.position = { 3500000, 2250000 };
    farTerminal.layers = { 1 };
    farTerminal.sourceId = "host-pad-uuid-2";
    board.pads.push_back( terminal );
    board.pads.push_back( farTerminal );

    ROUTING_NET foreign;
    foreign.netCode = 2;
    foreign.name = "N2";
    foreign.netClass = "Default";
    foreign.viaDiameter = 300000;
    foreign.viaDrill = 150000;
    foreign.padIndices = { 2, 3 };
    foreign.connections = { { 2, 3 } };
    board.nets.push_back( foreign );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION victim;
    victim.netCode = 2;
    victim.complete = true;
    victim.fromPadIndex = 2;
    victim.toPadIndex = 3;
    victim.nodes = { { terminal.position, 0 }, { terminal.position, 1 },
                     { farTerminal.position, 1 } };
    occupancy.Add( victim );

    ROUTING_CONNECTION candidate;
    candidate.netCode = 1;
    candidate.complete = true;
    candidate.fromPadIndex = 0;
    candidate.toPadIndex = 1;
    candidate.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    const auto conflicts = engine.FindConflictingConnections( candidate );
    BOOST_REQUIRE_EQUAL( conflicts.size(), 1U );
    BOOST_CHECK( SameRouteGeometry( conflicts.front(), victim ) );

    // DrillItemMover rejects an actual PAD in the moved via's normal-contact
    // set.  The native worker must not manufacture a dogleg that appears to
    // move copper off a fixed KiCad pad merely because both centres match.
    BOOST_CHECK( !engine.ShoveViaConnection( victim, { candidate }, {} ) );
    const auto inserted = FOUND_CONNECTION_INSERTER::Insert( candidate, conflicts, occupancy,
                                                               engine, {}, false );
    BOOST_CHECK( inserted.state == FOUND_CONNECTION_INSERTER::STATE::BLOCKED );
    BOOST_REQUIRE_EQUAL( occupancy.Connections().size(), 1U );
    BOOST_CHECK( SameRouteGeometry( occupancy.Connections().front(), victim ) );
}


BOOST_AUTO_TEST_CASE( StyledBlindViaChecksOnlyItsPhysicalSpanAndPreservesItInOutput )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.layers.push_back( { 2, true, 1, 20 } );
    ROUTING_OBSTACLE blockedOuterLayer;
    blockedOuterLayer.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    blockedOuterLayer.netCode = 2;
    blockedOuterLayer.layers = { 2 };
    blockedOuterLayer.start = blockedOuterLayer.end = { 3000000, 1500000 };
    blockedOuterLayer.radius = 100000;
    board.obstacles.push_back( blockedOuterLayer );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    const ROUTER_NODE top{ { 3000000, 1500000 }, 0 };
    const ROUTER_NODE inner{ top.point, 1 };
    ROUTING_EDGE_STYLE blind;
    blind.viaDiameter = 300000;
    blind.viaDrill = 150000;
    blind.viaLayers = { 0, 1 };
    BOOST_CHECK( engine.CanInsertSegment( 1, top, inner, &blind ) );
    BOOST_CHECK( !engine.CanInsertSegment( 1, top, inner ) );

    ROUTING_CONNECTION route;
    route.netCode = 1;
    route.complete = true;
    route.nodes = { top, inner };
    route.edgeStyles = { blind };
    occupancy.Add( route );
    ROUTING_RESULT emitted;
    FOUND_CONNECTION_INSERTER::Append( route, 100000, 300000, 150000, { 0, 1, 2 }, emitted );
    BOOST_REQUIRE_EQUAL( emitted.vias.size(), 1 );
    BOOST_CHECK( emitted.vias.front().layers == std::vector<int>( { 0, 1 } ) );
    BOOST_CHECK_EQUAL( emitted.vias.front().topLayer, 0 );
    BOOST_CHECK_EQUAL( emitted.vias.front().bottomLayer, 1 );
}


BOOST_AUTO_TEST_CASE( ViaRuleUsesCompletePadstackSpanForContainedTransition )
{
    auto settings = makeSettings();
    settings.layers = { { 20, true, 0, 0, 0 }, { 5, true, 0, 0, 1 },
                        { 18, true, 0, 0, 2 }, { 10, true, 0, 0, 3 } };

    ROUTING_EDGE_STYLE blind;
    blind.viaLayers = { 20, 18 };
    blind.viaType = ROUTER_VIA_TYPE::BLIND_BURIED;

    // The selected padstack spans 20 -> 5 -> 18 and can therefore perform
    // the smaller 20 -> 5 transition.  Its emitted copper must not be shrunk
    // to that transition.
    BOOST_CHECK( VIA_RULE::AllowsTransition( settings, 20, 5, &blind ) );
    BOOST_CHECK( VIA_RULE::LayersFor( settings, 20, 5, &blind )
                 == std::vector<int>( { 20, 5, 18 } ) );
    BOOST_CHECK( !VIA_RULE::AllowsTransition( settings, 18, 10, &blind ) );
}


BOOST_AUTO_TEST_CASE( OrderedViaRuleFallsThroughToFirstGeometricallyLegalProfile )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.layers = { { 0, true, 0, 0, 0 }, { 1, true, 0, 0, 1 },
                        { 2, true, 0, 0, 2 }, { 3, true, 0, 0, 3 } };
    ROUTING_NET& net = board.nets.front();
    net.viaProfiles.clear();
    net.viaProfiles.push_back( { 1000000, 150000, { 0, 2 }, false,
                                 ROUTER_VIA_TYPE::BLIND_BURIED } );
    net.viaProfiles.push_back( { 200000, 100000, { 0, 2 }, false,
                                 ROUTER_VIA_TYPE::BLIND_BURIED } );

    ROUTING_OBSTACLE blocker;
    blocker.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    blocker.netCode = 2;
    blocker.layers = { 0, 1, 2 };
    blocker.start = blocker.end = { 3500000, 1500000 };
    blocker.radius = 100000;
    board.obstacles.push_back( blocker );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    const ROUTER_NODE from{ { 3000000, 1500000 }, 0 };
    const ROUTER_NODE to{ from.point, 1 };
    const auto selected = engine.SelectViaStyle( 1, from, to );
    BOOST_REQUIRE( selected );
    BOOST_CHECK_EQUAL( selected->viaDiameter, 200000 );
    BOOST_CHECK_EQUAL( selected->viaDrill, 100000 );
    BOOST_CHECK( selected->viaLayers == std::vector<int>( { 0, 1, 2 } ) );

    // An explicit rule is authoritative: it cannot service a transition
    // outside either profile and must not invent a through-via fallback.
    BOOST_CHECK( !engine.SelectViaStyle( 1, { from.point, 2 }, { from.point, 3 } ) );
}


BOOST_AUTO_TEST_CASE( ViaRuleAttachSmdAndMicroviaTypeSurviveMaterialization )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.layers = { { static_cast<int>( F_Cu ), true, 0, 0, 0 },
                        { static_cast<int>( In1_Cu ), true, 0, 0, 1 },
                        { static_cast<int>( B_Cu ), true, 0, 0, 2 } };
    board.pads.front().layers = { static_cast<int>( F_Cu ) };
    board.pads.front().isSmd = true;
    ROUTING_NET& net = board.nets.front();
    net.viaProfiles.clear();
    net.viaProfiles.push_back( { 350000, 120000,
                                 { static_cast<int>( F_Cu ), static_cast<int>( In1_Cu ) },
                                 false, ROUTER_VIA_TYPE::MICROVIA } );
    net.viaProfiles.push_back( { 300000, 100000,
                                 { static_cast<int>( F_Cu ), static_cast<int>( In1_Cu ) },
                                 true, ROUTER_VIA_TYPE::MICROVIA } );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    const ROUTER_NODE from{ board.pads.front().position, static_cast<int>( F_Cu ) };
    const ROUTER_NODE to{ from.point, static_cast<int>( In1_Cu ) };
    const auto selected = engine.SelectViaStyle( 1, from, to, true );
    BOOST_REQUIRE( selected );
    BOOST_CHECK_EQUAL( selected->viaDiameter, 300000 );
    BOOST_CHECK( selected->viaType == ROUTER_VIA_TYPE::MICROVIA );

    ROUTING_CONNECTION route;
    route.netCode = 1;
    route.complete = true;
    route.nodes = { from, to };
    route.edgeStyles = { *selected };
    ROUTING_RESULT emitted;
    FOUND_CONNECTION_INSERTER::Append( route, 100000, 600000, 300000,
                                       VIA_RULE::ThroughLayers( settings ), emitted );
    BOOST_REQUIRE_EQUAL( emitted.vias.size(), 1U );
    BOOST_CHECK( emitted.vias.front().type == ROUTER_VIA_TYPE::MICROVIA );

    BOARD host;
    KICAD_BOARD_ADAPTER adapter( &host );
    const auto preview = adapter.CreatePreviewItems( emitted );
    BOOST_REQUIRE_EQUAL( preview.size(), 1U );
    BOOST_REQUIRE_EQUAL( preview.front()->Type(), PCB_VIA_T );
    BOOST_CHECK( static_cast<PCB_VIA*>( preview.front().get() )->GetViaType()
                 == VIATYPE::MICROVIA );
}


BOOST_AUTO_TEST_CASE( ConvexPolygonSpringOverKeepsGeneralAngleRoutesExact )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.bounds = { 0, 0, 10000000, 6000000 };
    ROUTING_OBSTACLE diamond;
    diamond.kind = ROUTER_OBSTACLE_KIND::POLYGON;
    diamond.netCode = 2;
    diamond.layers = { 0 };
    diamond.polygon = { { 5000000, 2000000 }, { 6000000, 3000000 },
                        { 5000000, 4000000 }, { 4000000, 3000000 } };
    board.obstacles.push_back( diamond );

    const auto convex = PLANAR::SIMPLEX::FromConvexPolygon( diamond.polygon, 50000 );
    BOOST_REQUIRE( convex );
    const auto directPath = PLANAR::POLYLINE::FromPoints(
            { { 2000000, 2699994 }, { 8000000, 3300006 } } );
    BOOST_REQUIRE( !directPath.Empty() );
    BOOST_CHECK( convex->IntersectsSegment( directPath, 1 ) );
    const auto offset = PLANAR::SIMPLEX::FromConvexPolygon( diamond.polygon, 50001 );
    BOOST_REQUIRE( offset );
    const auto spring = TRACE_SHOVER::SpringOverObstacles(
            directPath, { { 1, { 4000000, 2000000, 6000000, 4000000 }, *convex, *offset } } );
    BOOST_REQUIRE( spring.polyline );
    BOOST_REQUIRE( spring.polyline->IntegralCorners() );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION route;
    route.netCode = 1;
    route.complete = true;
    // This diagonal crosses the convex diamond at integral entrance points;
    // its replacement must use exact support-line intersections, not an
    // axis-aligned obstacle box or a rounded vertex.
    route.nodes = { { { 2000000, 2699994 }, 0 }, { { 8000000, 3300006 }, 0 } };
    BOOST_CHECK( !engine.CanInsertSegment( 1, route.nodes.front(), route.nodes.back() ) );
    const auto wrapped = engine.SpringOverConnection( route, {} );
    BOOST_REQUIRE( wrapped );
    BOOST_CHECK( wrapped->nodes.front() == route.nodes.front() );
    BOOST_CHECK( wrapped->nodes.back() == route.nodes.back() );
    BOOST_CHECK_GT( wrapped->nodes.size(), route.nodes.size() );
    for( std::size_t index = 1; index < wrapped->nodes.size(); ++index )
        BOOST_CHECK( engine.CanInsertSegment( wrapped->netCode, wrapped->nodes[index - 1],
                                              wrapped->nodes[index] ) );
}


BOOST_AUTO_TEST_CASE( ConvexPolygonNormalizesCollinearSupportCorners )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.bounds = { 0, 0, 10000000, 4000000 };
    settings.layers = { { 0, true, 1, 20 } };
    settings.allowVias = false;

    ROUTING_OBSTACLE obstacle;
    obstacle.kind = ROUTER_OBSTACLE_KIND::POLYGON;
    obstacle.netCode = 2;
    obstacle.layers = { 0 };
    // The middle point along the lower edge is a redundant, forward
    // collinear support point. Freerouting's Polygon normalization removes
    // it before TileShape construction; treating the whole contour as
    // unsupported would drop this common KiCad shape into the sampled path.
    obstacle.polygon = { { 4000000, 1000000 }, { 5000000, 1000000 },
                         { 6000000, 1000000 }, { 6000000, 3000000 },
                         { 4000000, 3000000 } };
    board.obstacles.push_back( obstacle );

    const auto simplex = PLANAR::SIMPLEX::FromConvexPolygon( obstacle.polygon, 50000 );
    BOOST_REQUIRE( simplex );
    const auto path = PLANAR::POLYLINE::FromPoints(
            { { 1000000, 2000000 }, { 9000000, 2000000 } } );
    BOOST_REQUIRE( !path.Empty() );
    BOOST_CHECK( simplex->IntersectsSegment( path, 1 ) );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION route;
    route.netCode = 1;
    route.complete = true;
    route.nodes = { { { 1000000, 2000000 }, 0 }, { { 9000000, 2000000 }, 0 } };
    BOOST_CHECK( !engine.CanInsertSegment( 1, route.nodes.front(), route.nodes.back() ) );
    const auto wrapped = engine.SpringOverConnection( route, {} );
    BOOST_REQUIRE( wrapped );
    BOOST_CHECK_GT( wrapped->nodes.size(), route.nodes.size() );
    for( std::size_t index = 1; index < wrapped->nodes.size(); ++index )
        BOOST_CHECK( engine.CanInsertSegment( wrapped->netCode, wrapped->nodes[index - 1],
                                              wrapped->nodes[index] ) );
}


BOOST_AUTO_TEST_CASE( GeneralConvexGeometryDoesNotCloseABoundingBoxCornerWedge )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.bounds = { 0, 0, 10000000, 6000000 };
    board.pads[0].position = { 1000000, 3400000 };
    board.pads[1].position = { 9000000, 200000 };
    settings.layers = { { 0, true, 1, 20 } };
    settings.allowVias = false;

    ROUTING_OBSTACLE diamond;
    diamond.kind = ROUTER_OBSTACLE_KIND::POLYGON;
    diamond.netCode = 2;
    diamond.layers = { 0 };
    diamond.polygon = { { 5000000, 2000000 }, { 6000000, 3000000 },
                        { 5000000, 4000000 }, { 4000000, 3000000 } };
    board.obstacles.push_back( diamond );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    int expanded = 0;
    const auto route = engine.FindConnection( board.pads[0], board.pads[1], 0, expanded, {}, {} );
    BOOST_REQUIRE( route );
    // The line crosses only the diamond's axis-aligned bounding box corner;
    // it stays outside the real convex contour. A rectangular room model
    // would manufacture an unnecessary dogleg here.
    BOOST_REQUIRE_EQUAL( route->nodes.size(), 2 );
    BOOST_CHECK( engine.CanInsertSegment( route->netCode, route->nodes.front(),
                                          route->nodes.back() ) );
    BOOST_CHECK( !engine.LastRoomSearchMetrics().routed );
}


BOOST_AUTO_TEST_CASE( GeneralConvexGeometryUsesExactOffsetSupportCornersForDetours )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.bounds = { 0, 0, 10000000, 6000000 };
    board.pads[0].position = { 1000000, 3000000 };
    board.pads[1].position = { 9000000, 3000000 };
    settings.layers = { { 0, true, 1, 20 } };
    settings.allowVias = false;
    settings.maxExpandedNodes = 64;

    ROUTING_OBSTACLE diamond;
    diamond.kind = ROUTER_OBSTACLE_KIND::POLYGON;
    diamond.netCode = 2;
    diamond.layers = { 0 };
    diamond.polygon = { { 5000000, 2000000 }, { 6000000, 3000000 },
                        { 5000000, 4000000 }, { 4000000, 3000000 } };
    board.obstacles.push_back( diamond );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    int expanded = 0;
    const auto route = engine.FindConnection( board.pads[0], board.pads[1], 0, expanded, {}, {} );
    BOOST_REQUIRE( route );
    BOOST_CHECK_GT( route->nodes.size(), 2 );
    BOOST_CHECK_LE( expanded, settings.maxExpandedNodes );
    BOOST_CHECK( std::any_of( route->nodes.begin() + 1, route->nodes.end() - 1,
                              [&]( const ROUTER_NODE& node )
                              {
                                  return node.point.x % settings.gridStepIU != 0
                                         || node.point.y % settings.gridStepIU != 0;
                              } ) );
    for( std::size_t index = 1; index < route->nodes.size(); ++index )
        BOOST_CHECK( engine.CanInsertSegment( route->netCode, route->nodes[index - 1],
                                              route->nodes[index] ) );
    BOOST_CHECK( !engine.LastRoomSearchMetrics().routed );
}


BOOST_AUTO_TEST_CASE( GeneralConvexOnOtherLayerKeepsSafeLayerRoomSearchActive )
{
    // A through-via still needs exact full-stack clearance, but a diagonal
    // contour on an otherwise unused layer must not disable the bounded room
    // search on a clear signal layer.  The old all-or-nothing guard skipped
    // the whole room/drill frontier here and made this simple direct route
    // fall back to the broad visibility graph.
    auto board = makeBoard();
    auto settings = makeSettings();
    board.bounds = { 0, 0, 10000000, 6000000 };
    board.pads[0].position = { 1000000, 3000000 };
    board.pads[1].position = { 9000000, 3000000 };
    settings.maxExpandedNodes = 256;

    ROUTING_OBSTACLE diamond;
    diamond.kind = ROUTER_OBSTACLE_KIND::POLYGON;
    diamond.netCode = 2;
    diamond.layers = { 1 };
    diamond.polygon = { { 5000000, 2000000 }, { 6000000, 3000000 },
                        { 5000000, 4000000 }, { 4000000, 3000000 } };
    board.obstacles.push_back( diamond );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    int expanded = 0;
    const auto route = engine.FindConnection( board.pads[0], board.pads[1], 0, expanded, {}, {} );

    BOOST_REQUIRE( route );
    BOOST_CHECK( engine.LastRoomSearchMetrics().routed );
    BOOST_REQUIRE_EQUAL( route->nodes.size(), 2U );
    BOOST_CHECK_EQUAL( route->nodes.front().layer, 0 );
    BOOST_CHECK_EQUAL( route->nodes.back().layer, 0 );
    BOOST_CHECK_LE( expanded, settings.maxExpandedNodes );
    BOOST_CHECK( engine.CanInsertSegment( route->netCode, route->nodes.front(),
                                          route->nodes.back() ) );
}


BOOST_AUTO_TEST_CASE( DenseConvexSearchReservesConnectionLocalSupportDoors )
{
    // A dense board can contribute hundreds of nearby base landmarks before
    // the exact support corners of the blocking convex contour. The regular
    // 8 mm grid has no in-board escape here, so dropping those local corners
    // behind the dense-board visibility cap incorrectly exhausts the maze.
    // Connection-local exact corners must retain a small reserved fan even
    // while the broad board graph stays bounded for editor responsiveness.
    auto board = makeBoard();
    auto settings = makeSettings();
    board.bounds = { 0, 0, 10000000, 6000000 };
    board.pads[0].position = { 1000000, 3000000 };
    board.pads[1].position = { 9000000, 3000000 };
    settings.layers = { { 0, true, 1, 20 } };
    settings.allowVias = false;
    settings.gridStepIU = 8000000;
    settings.maxExpandedNodes = 64;

    // These are deliberately data-only, netless landmark sources. They do
    // not obstruct the active route; their role is to model the many nearby
    // ordinary board landmarks seen on a dense MCU layout.
    for( int index = 0; index < 400; ++index )
    {
        ROUTING_PAD dummy;
        dummy.position = { 1100000 + ( index % 20 ) * 10000,
                           3100000 + ( index / 20 ) * 10000 };
        dummy.layers = { 0 };
        board.pads.push_back( std::move( dummy ) );
    }

    ROUTING_OBSTACLE diamond;
    diamond.kind = ROUTER_OBSTACLE_KIND::POLYGON;
    diamond.netCode = 2;
    diamond.layers = { 0 };
    diamond.polygon = { { 5000000, 2000000 }, { 6000000, 3000000 },
                        { 5000000, 4000000 }, { 4000000, 3000000 } };
    board.obstacles.push_back( diamond );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    int expanded = 0;
    const auto route = engine.FindConnection( board.pads[0], board.pads[1], 0, expanded, {}, {} );
    BOOST_REQUIRE( route );
    BOOST_CHECK_GT( route->nodes.size(), 2U );
    BOOST_CHECK_LE( expanded, settings.maxExpandedNodes );
    BOOST_CHECK( std::any_of( route->nodes.begin() + 1, route->nodes.end() - 1,
                              [&]( const ROUTER_NODE& node )
                              {
                                  return node.point.x % settings.gridStepIU != 0
                                         || node.point.y % settings.gridStepIU != 0;
                              } ) );
    for( std::size_t index = 1; index < route->nodes.size(); ++index )
        BOOST_CHECK( engine.CanInsertSegment( route->netCode, route->nodes[index - 1],
                                              route->nodes[index] ) );
    BOOST_CHECK( !engine.LastRoomSearchMetrics().routed );
}


BOOST_AUTO_TEST_CASE( GeneralConvexVisibilityGraphRetainsRationalSupportCandidates )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.layers = { { 0, true, 1, 20 } };
    settings.allowVias = false;

    ROUTING_OBSTACLE triangle;
    triangle.kind = ROUTER_OBSTACLE_KIND::POLYGON;
    triangle.netCode = 2;
    triangle.layers = { 0 };
    // Adjacent support lines have determinant 19.  The compensated corner
    // is therefore rational for this ordinary track radius, rather than a
    // grid-aligned rectangle corner that the old landmark builder happened
    // to retain.
    triangle.polygon = { { 2000000, 1000000 }, { 2004000, 1001000 },
                         { 2001000, 1005000 } };
    board.obstacles.push_back( triangle );

    constexpr std::int64_t trackRadius = 50000;
    const auto simplex = PLANAR::SIMPLEX::FromConvexPolygon( triangle.polygon,
                                                              trackRadius + 2 );
    BOOST_REQUIRE( simplex );
    const auto rational = std::find_if(
            simplex->Borders().begin(), simplex->Borders().end(),
            [&]( const PLANAR::LINE& line )
            {
                const std::size_t index = static_cast<std::size_t>( &line
                        - simplex->Borders().data() );
                return !simplex->Corner( index ).Integral();
            } );
    BOOST_REQUIRE( rational != simplex->Borders().end() );
    const std::size_t rationalIndex = static_cast<std::size_t>( rational
            - simplex->Borders().begin() );
    const auto bounds = simplex->Corner( rationalIndex ).SurroundingBox();
    BOOST_REQUIRE( bounds );

    const auto landmarks = EXPANSION_GRAPH::BuildLandmarks(
            board, settings, ROUTING_PAD{}, ROUTING_PAD{}, trackRadius, 150000, 1 );
    const bool foundSibling = std::any_of(
            landmarks.begin(), landmarks.end(), [&]( const ROUTER_NODE& node )
            {
                return node.layer == 0 && node.point.x >= bounds->minX
                       && node.point.x <= bounds->maxX && node.point.y >= bounds->minY
                       && node.point.y <= bounds->maxY;
            } );
    BOOST_CHECK( foundSibling );
}


BOOST_AUTO_TEST_CASE( ConcaveHoleCannotHideAnInteriorSolidCrossing )
{
    // A route may legally use the free space of a polygon hole, but it must
    // not jump from one arm of a *concave* hole to another through the solid
    // copper/keepout between them.  The old generic-polygon path sampled at
    // half the maze grid.  With this deliberately coarse grid it examined
    // only the endpoints (both in the hole) and missed the interior crossing.
    auto board = makeBoard();
    auto settings = makeSettings();
    board.bounds = { 0, 0, 10000000, 10000000 };
    board.pads[0].position = { 3500000, 6000000 };
    board.pads[1].position = { 6500000, 6000000 };
    settings.layers = { { 0, true, 1, 20 } };
    settings.allowVias = false;
    settings.enableFanout = false;
    settings.gridStepIU = 10000000;

    ROUTING_OBSTACLE keepout;
    keepout.kind = ROUTER_OBSTACLE_KIND::POLYGON;
    keepout.layers = { 0 };
    keepout.blocksTracks = true;
    keepout.blocksVias = true;
    keepout.polygon = { { 1000000, 1000000 }, { 9000000, 1000000 },
                        { 9000000, 9000000 }, { 1000000, 9000000 } };
    // U-shaped free space: the two pad centres lie in its arms, while the
    // direct chord passes through the solid centre of the outer polygon.
    keepout.polygonHoles = { { { 3000000, 3000000 }, { 7000000, 3000000 },
                               { 7000000, 7000000 }, { 6000000, 7000000 },
                               { 6000000, 4000000 }, { 4000000, 4000000 },
                               { 4000000, 7000000 }, { 3000000, 7000000 } } };
    board.obstacles.push_back( keepout );

    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    BOOST_CHECK( !engine.CanInsertSegment( 1, { board.pads[0].position, 0 },
                                           { board.pads[1].position, 0 } ) );
}


BOOST_AUTO_TEST_CASE( SpringOverRejectsUnsupportedShapesAndInvalidVias )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    ROUTING_CONNECTION via;
    via.netCode = 1; via.complete = true;
    via.nodes = { { { 1000000, 1000000 }, 0 }, { { 2000000, 1000000 }, 1 } };
    BOOST_CHECK( FOUND_CONNECTION_INSERTER::Insert( via, {}, occupancy, engine ).state
                  == FOUND_CONNECTION_INSERTER::STATE::BLOCKED );
    BOOST_CHECK( occupancy.Connections().empty() );
    // The production adapter does not round rational/diagonal corners or
    // pretend that its rectangle snapshot mapping supports arbitrary offsets.
    via.nodes = { { { 1000000, 1000000 }, 0 }, { { 2000000, 2000000 }, 0 } };
    BOOST_CHECK( !engine.SpringOverConnection( via, {} ) );


}

BOOST_AUTO_TEST_CASE( CopperContactsDoNotTrustLogicalRouteEndpoints )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_BOARD copper( board, settings );
    ROUTING_CONNECTION route;
    route.netCode = 1;
    route.complete = true;
    route.fromPadIndex = 0;
    route.toPadIndex = 1;
    route.nodes = { { board.pads[0].position, 0 }, { { 3000000, 1500000 }, 0 } };
    copper.AddRoute( route );
    BOOST_CHECK( !copper.Connected( 0, 1 ) );
    BOOST_CHECK_EQUAL( copper.CountMissing( board.nets[0] ), 1 );
    copper.RemoveRoute( route );
    route.nodes.back().point = board.pads[1].position;
    copper.AddRoute( route );
    BOOST_CHECK( copper.Connected( 0, 1 ) );
    BOOST_CHECK_EQUAL( copper.CountMissing( board.nets[0] ), 0 );
}

BOOST_AUTO_TEST_CASE( CopperTJunctionsSurviveRollbackAndDisconnectOnRipup )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.pads.push_back( board.pads[0] );
    board.pads.back().position = { 3000000, 2500000 };
    board.nets[0].padIndices.push_back( 2 );
    board.nets[0].connections.push_back( { 0, 2 } );
    ROUTING_BOARD copper( board, settings );
    ROUTING_CONNECTION trunk;
    trunk.netCode = 1;
    trunk.complete = true;
    trunk.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    ROUTING_CONNECTION branch = trunk;
    branch.nodes = { { board.pads[2].position, 0 }, { { 3000000, 1500000 }, 0 } };
    copper.AddRoute( trunk );
    copper.AddRoute( branch );
    BOOST_REQUIRE( copper.Connected( 0, 2 ) );
    const auto count = copper.ItemCount();
    const auto revision = copper.Revision();
    {
        ROUTING_BOARD::TRANSACTION attempt( copper );
        copper.RemoveRoute( trunk );
        BOOST_CHECK( !copper.Connected( 0, 2 ) );
        copper.AddRoute( trunk );
        copper.RemoveRoute( branch );
    }
    BOOST_CHECK_GT( copper.Revision(), revision );
    BOOST_CHECK_EQUAL( copper.ItemCount(), count );
    BOOST_REQUIRE( copper.Connected( 0, 2 ) );
    const auto terminals = copper.Terminals( 2 );
    BOOST_CHECK( std::any_of( terminals.begin(), terminals.end(),
                             []( const auto& t ) { return t.segmentEnd.has_value(); } ) );
    {
        ROUTING_BOARD::TRANSACTION attempt( copper );
        copper.RemoveRoute( branch );
        attempt.Commit();
    }
    BOOST_CHECK( copper.Connected( 0, 1 ) );
    BOOST_CHECK( !copper.Connected( 0, 2 ) );
    copper.ClearRoutes();
    BOOST_CHECK_EQUAL( copper.ItemCount(), 3 );
}

BOOST_AUTO_TEST_CASE( SyntheticLandingsRequireRealViasAndValidTransitions )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.pads[0].layers = { 0 };
    board.pads[1].position = board.pads[0].position;
    board.pads[1].layers = { 0, 1 };
    board.pads[1].isFanoutTarget = true;
    board.pads[1].fanoutTargetLayer = 1;
    ROUTING_BOARD copper( board, settings );
    BOOST_CHECK( !copper.Connected( 0, 1 ) );
    BOOST_CHECK( copper.Terminals( 1 ).empty() );
    ROUTING_CONNECTION via;
    via.netCode = 1;
    via.complete = true;
    via.nodes = { { board.pads[0].position, 0 }, { board.pads[0].position, 1 } };
    auto malformed = via;
    malformed.nodes.back().point.x += 100000;
    const auto revision = copper.Revision();
    BOOST_CHECK_THROW( copper.AddRoute( malformed ), std::invalid_argument );
    BOOST_CHECK_EQUAL( copper.Revision(), revision );
    auto outOfRange = via;
    outOfRange.nodes.push_back( { { std::numeric_limits<std::int64_t>::max(), 0 }, 1 } );
    BOOST_CHECK_THROW( copper.AddRoute( outOfRange ), std::out_of_range );
    BOOST_CHECK_EQUAL( copper.Revision(), revision );
    copper.AddRoute( via );
    BOOST_CHECK( copper.Connected( 0, 1 ) );
    copper.RemoveRoute( via );
    BOOST_CHECK( !copper.Connected( 0, 1 ) );
}

BOOST_AUTO_TEST_CASE( ConductionIslandsAndHolesAreNotVirtualConnections )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.pads.push_back( board.pads[0] );
    board.pads.back().position = { 3000000, 1500000 };
    for( auto& pad : board.pads )
        pad.layers = { 0 };
    ROUTING_OBSTACLE area;
    area.netCode = 1;
    area.kind = ROUTER_OBSTACLE_KIND::POLYGON;
    area.layers = { 0 };
    area.polygon = { { 500000, 500000 }, { 4000000, 500000 },
                     { 4000000, 2500000 }, { 500000, 2500000 } };
    area.polygonHoles = { { { 2500000, 1000000 }, { 3500000, 1000000 },
                           { 3500000, 2000000 }, { 2500000, 2000000 } } };
    board.conductionAreas.push_back( area );
    area.polygon = { { 4500000, 500000 }, { 5500000, 500000 },
                     { 5500000, 2500000 }, { 4500000, 2500000 } };
    area.polygonHoles.clear();
    board.conductionAreas.push_back( area );
    ROUTING_BOARD copper( board, settings );
    BOOST_CHECK( !copper.Connected( 0, 1 ) );
    BOOST_CHECK( !copper.Connected( 0, 2 ) );
    ROUTING_CONNECTION stub;
    stub.netCode = 1;
    stub.complete = true;
    stub.nodes = { { board.pads[2].position, 0 }, { { 2000000, 1500000 }, 0 } };
    copper.AddRoute( stub );
    BOOST_CHECK( copper.Connected( 0, 2 ) );
    BOOST_CHECK( !copper.Connected( 0, 1 ) );
}

BOOST_AUTO_TEST_CASE( PipelineRoutesToRetainedTraceInterior )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.enableFanout = false;
    settings.optimizeAfterComplete = false;
    board.pads.push_back( board.pads[0] );
    board.pads.back().position = { 3000000, 2500000 };
    board.nets[0].padIndices.push_back( 2 );
    board.nets[0].connections = { { 0, 2 } };
    ROUTING_OBSTACLE trunk;
    trunk.netCode = 1;
    trunk.kind = ROUTER_OBSTACLE_KIND::SEGMENT;
    trunk.start = board.pads[0].position;
    trunk.end = board.pads[1].position;
    trunk.radius = 50000;
    trunk.layers = { 0 };
    trunk.isExistingRoute = true;
    board.obstacles.push_back( trunk );
    const auto result = ROUTING_PIPELINE().Run( board, settings, {}, {} );
    BOOST_REQUIRE( result.complete );
    BOOST_REQUIRE_EQUAL( result.connections.size(), 1 );
    const auto& nodes = result.connections.front().nodes;
    BOOST_REQUIRE( !nodes.empty() );
    const auto onInterior = []( const ROUTER_POINT& p )
    { return p.y == 1500000 && p.x > 1000000 && p.x < 5000000; };
    BOOST_CHECK( onInterior( nodes.front().point ) || onInterior( nodes.back().point ) );
    ROUTING_BOARD copper( board, settings );
    copper.AddRoute( result.connections.front() );
    BOOST_CHECK( copper.Connected( 0, 2 ) );
}

BOOST_AUTO_TEST_CASE( OptimizerCannotShortenAwayBranchContacts )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.pads.push_back( board.pads[0] );
    board.pads.back().position = { 3000000, 2700000 };
    board.nets[0].padIndices.push_back( 2 );
    ROUTING_CONNECTION trunk;
    trunk.netCode = 1;
    trunk.complete = true;
    trunk.nodes = { { board.pads[0].position, 0 }, { { 3000000, 2200000 }, 0 },
                    { board.pads[1].position, 0 } };
    ROUTING_CONNECTION branch = trunk;
    branch.nodes = { { board.pads[2].position, 0 }, { { 3000000, 2200000 }, 0 } };
    std::vector<ROUTING_CONNECTION> routes{ trunk, branch };
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    occupancy.Add( trunk );
    occupancy.Add( branch );
    BOOST_REQUIRE( occupancy.Board()->Connected( 0, 2 ) );
    BATCH_OPTIMIZER( board, settings, occupancy ).Optimize( routes, {} );
    BOOST_CHECK( occupancy.Board()->Connected( 0, 2 ) );
    BOOST_CHECK( occupancy.Board()->Connected( 0, 1 ) );
    BOOST_CHECK( routes.front().nodes == trunk.nodes );
    // Removing an absent route must not decrement another route's congestion.
    occupancy.Remove( trunk );
    occupancy.Remove( trunk );
    BOOST_CHECK_EQUAL( occupancy.Connections().size(), 1 );
    BOOST_CHECK( !occupancy.Board()->Connected( 0, 2 ) );
}


BOOST_AUTO_TEST_CASE( OptimizerReroutesAWholeConnectionAndKeepsOnlyAnImprovement )
{
    auto board = makeBoard();
    board.bounds.maxY = 4000000;

    ROUTING_OBSTACLE wall;
    wall.kind = ROUTER_OBSTACLE_KIND::RECTANGLE;
    wall.netCode = 2;
    wall.layers = { 0 };
    wall.box = { 2600000, 1100000, 3400000, 1900000 };
    board.obstacles.push_back( wall );

    auto settings = makeSettings();
    settings.layers[1].enabled = false;
    settings.allowVias = false;
    settings.optimizationPasses = 1;
    settings.maxOptimizationAutoroutePasses = 2;

    ROUTING_CONNECTION detour;
    detour.netCode = 1;
    detour.complete = true;
    detour.fromPadIndex = 0;
    detour.toPadIndex = 1;
    detour.nodes = { { board.pads[0].position, 0 }, { { 1000000, 3000000 }, 0 },
                     { { 5000000, 3000000 }, 0 }, { board.pads[1].position, 0 } };

    const auto length = []( const ROUTING_CONNECTION& aConnection )
    {
        double result = 0.0;
        for( std::size_t index = 1; index < aConnection.nodes.size(); ++index )
        {
            const long double dx = static_cast<long double>( aConnection.nodes[index].point.x )
                                   - aConnection.nodes[index - 1].point.x;
            const long double dy = static_cast<long double>( aConnection.nodes[index].point.y )
                                   - aConnection.nodes[index - 1].point.y;
            result += std::sqrt( static_cast<double>( dx * dx + dy * dy ) );
        }
        return result;
    };

    std::vector<ROUTING_CONNECTION> routes{ detour };
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    occupancy.Add( detour );
    BOOST_REQUIRE( occupancy.Board()->Connected( 0, 1 ) );

    const int passes = BATCH_OPTIMIZER( board, settings, occupancy ).Optimize( routes, {} );

    BOOST_CHECK_EQUAL( passes, 1 );
    BOOST_REQUIRE_EQUAL( routes.size(), 1U );
    BOOST_CHECK( occupancy.Board()->Connected( 0, 1 ) );
    BOOST_CHECK( !SameRouteGeometry( routes.front(), detour ) );
    BOOST_CHECK_LT( length( routes.front() ), length( detour ) );
    BOOST_CHECK( std::none_of( routes.front().nodes.begin(), routes.front().nodes.end(),
                              []( const ROUTER_NODE& aNode )
                              { return aNode.point.y == 3000000; } ) );
    int transitions = 0;
    for( std::size_t index = 1; index < routes.front().nodes.size(); ++index )
        if( routes.front().nodes[index - 1].layer != routes.front().nodes[index].layer )
            ++transitions;
    BOOST_CHECK_EQUAL( transitions, 0 );
}

BOOST_AUTO_TEST_CASE( ARouteCannotInventCopperAtAVirtualPlaneTarget )
{
    auto board = makeBoard();
    board.pads[1].isPlaneTarget = true;
    ROUTING_BOARD copper( board, makeSettings() );
    ROUTING_CONNECTION stub;
    stub.netCode = 1;
    stub.complete = true;
    stub.nodes = { { board.pads[0].position, 0 }, { board.pads[1].position, 0 } };
    copper.AddRoute( stub );
    BOOST_CHECK( !copper.Connected( 0, 1 ) );
    BOOST_CHECK_EQUAL( copper.CountMissing( board.nets[0] ), 1 );
}

namespace
{
std::unique_ptr<BOARD> makeHostRoutingBoard( bool withPlane )
{
    auto board = std::make_unique<BOARD>();
    board->SetCopperLayerCount( 2 );
    auto* net = new NETINFO_ITEM( board.get(), "SIGNAL", 17 );
    board->Add( net );
    auto* ground = new NETINFO_ITEM( board.get(), "GND", 23 );
    board->Add( ground );
    auto* footprint = new FOOTPRINT( board.get() );
    footprint->SetReference( "TEST1" );
    footprint->Reference().SetVisible( false );
    footprint->Value().SetVisible( false );
    board->Add( footprint );
    auto addPad = [&]( VECTOR2I position, NETINFO_ITEM* n, int diameter )
    {
        auto* pad = new PAD( footprint );
        pad->SetAttribute( PAD_ATTRIB::SMD );
        pad->SetLayerSet( LSET( { F_Cu, F_Mask } ) );
        pad->SetShape( PADSTACK::ALL_LAYERS, PAD_SHAPE::CIRCLE );
        pad->SetSize( PADSTACK::ALL_LAYERS, { diameter, diameter } );
        pad->SetPosition( position );
        pad->SetNumber( wxString::Format( "%zu", footprint->Pads().size() + 1 ) );
        pad->SetNet( n );
        footprint->Add( pad );
    };
    if( withPlane )
    {
        addPad( { 15000000, 1400000 }, net, 1600000 );
        addPad( { 15000000, 28600000 }, net, 1600000 );
        addPad( { 5000000, 15000000 }, ground, 1000000 );
        addPad( { 25000000, 15000000 }, ground, 1000000 );
        auto* zone = new ZONE( board.get() );
        zone->SetLayer( F_Cu );
        zone->SetNet( ground );
        zone->SetLocalClearance( 200000 );
        zone->SetMinThickness( 200000 );
        zone->SetPadConnection( ZONE_CONNECTION::FULL );
        zone->SetIslandRemovalMode( ISLAND_REMOVAL_MODE::NEVER );
        zone->Outline()->NewOutline();
        for( auto p : { VECTOR2I( 0, 0 ), VECTOR2I( 30000000, 0 ),
                       VECTOR2I( 30000000, 30000000 ), VECTOR2I( 0, 30000000 ) } )
            zone->Outline()->Append( p );
        board->Add( zone );
    }
    else
    {
        addPad( { 5000000, 15000000 }, net, 1000000 );
        addPad( { 25000000, 15000000 }, net, 1000000 );
    }
    const std::vector<VECTOR2I> corners{ { 0, 0 }, { 30000000, 0 },
                                        { 30000000, 30000000 }, { 0, 30000000 } };
    for( std::size_t i = 0; i < corners.size(); ++i )
    {
        auto* edge = new PCB_SHAPE( board.get() );
        edge->SetShape( SHAPE_T::SEGMENT );
        edge->SetStart( corners[i] );
        edge->SetEnd( corners[( i + 1 ) % corners.size()] );
        edge->SetLayer( Edge_Cuts );
        edge->SetWidth( 50000 );
        board->Add( edge );
    }
    board->BuildConnectivity();
    return board;
}
}

BOOST_AUTO_TEST_CASE( HostSessionValidatesWithoutChangingSourceOrNetCodes )
{
    auto board = makeHostRoutingBoard( false );
    const auto time = board->GetTimeStamp();
    const auto netSettings = board->GetDesignSettings().m_NetSettings;
    const int code = board->FindNet( "SIGNAL" )->GetNetCode();
    auto settings = KICAD_BOARD_ADAPTER( board.get() ).CreateDefaultSettings();
    settings.enableFanout = false;
    settings.maxIterations = 1;
    settings.maxPasses = 1;
    settings.optimizationPasses = 0;
    auto result = KICAD_ROUTING_SESSION( *board ).Run( settings );
    BOOST_REQUIRE_MESSAGE( result.hostValidated, result.message );
    BOOST_REQUIRE_MESSAGE( result.complete, result.message );
    BOOST_CHECK_EQUAL( result.hostUnconnected, 0 );
    BOOST_CHECK_EQUAL( result.hostNewDrcViolations, 0 );
    BOOST_REQUIRE( !result.segments.empty() );
    for( const auto& segment : result.segments )
        BOOST_CHECK_EQUAL( segment.netCode, code );
    BOOST_CHECK( board->Tracks().empty() );
    BOOST_CHECK_EQUAL( board->GetTimeStamp(), time );
    BOOST_CHECK( board->GetDesignSettings().m_NetSettings == netSettings );
    BOOST_CHECK( !board->GetDesignSettings().m_DRCEngine );
    // Re-running DRC on the source must not use the destroyed private engine.
    auto drc = std::make_shared<DRC_ENGINE>( board.get(), &board->GetDesignSettings() );
    drc->InitEngine( wxFileName() );
    board->GetDesignSettings().m_DRCEngine = drc;
    { KICAD_ROUTING_SESSION other( *board ); other.Run( settings ); }
    int missing = 0;
    drc->SetViolationHandler( [&]( const std::shared_ptr<DRC_ITEM>& item, const VECTOR2I&, int,
                                  const std::function<void( PCB_MARKER* )>& )
    { if( item->GetErrorCode() == DRCE_UNCONNECTED_ITEMS ) ++missing; } );
    drc->RunTests( EDA_UNITS::MM, true, false );
    drc->ClearViolationHandler();
    BOOST_CHECK( drc->TestsCompleted() );
    BOOST_CHECK_EQUAL( missing, 1 );
}


BOOST_AUTO_TEST_CASE( KiCadAdapterReservesThermalReliefSpokeExits )
{
    auto board = makeHostRoutingBoard( true );
    ZONE* zone = board->Zones().front();
    zone->SetPadConnection( ZONE_CONNECTION::THERMAL );
    zone->SetThermalReliefGap( 250000 );
    zone->SetThermalReliefSpokeWidth( 300000 );

    PAD* groundPad = nullptr;
    for( PAD* pad : board->GetPads() )
    {
        if( pad->GetNetCode() == zone->GetNetCode() )
        {
            groundPad = pad;
            break;
        }
    }

    BOOST_REQUIRE( groundPad );
    BOOST_CHECK( groundPad->FlashLayer( F_Cu ) );
    BOOST_CHECK( zone->GetLayerSet().Contains( F_Cu ) );
    BOOST_CHECK( zone->GetBoundingBox().Intersects( groundPad->GetBoundingBox( F_Cu ) ) );
    BOOST_CHECK( zone->GetBoardOutline().Contains( groundPad->GetPosition() ) );
    BOOST_CHECK( zone->GetPadConnection() == ZONE_CONNECTION::THERMAL );
    BOOST_CHECK_EQUAL( zone->GetThermalReliefGap(), 250000 );
    const auto settings = KICAD_BOARD_ADAPTER( board.get() ).CreateDefaultSettings();
    const auto snapshot = KICAD_BOARD_ADAPTER( board.get() ).CreateSnapshot( settings );
    BOOST_REQUIRE( snapshot );

    const ROUTER_POINT center{ groundPad->ShapePos( F_Cu ).x,
                               groundPad->ShapePos( F_Cu ).y };
    BOOST_CHECK_GE( std::count_if(
                            snapshot->obstacles.begin(), snapshot->obstacles.end(),
                            [&]( const ROUTING_OBSTACLE& obstacle )
                            {
                                return obstacle.netCode == groundPad->GetNetCode()
                                       && obstacle.kind == ROUTER_OBSTACLE_KIND::SEGMENT
                                       && obstacle.layers
                                                  == std::vector<int>{ static_cast<int>( F_Cu ) }
                                       && obstacle.start == center
                                       && obstacle.end != obstacle.start
                                       && obstacle.radius == 150000;
                            } ),
                    2 );
}

BOOST_AUTO_TEST_CASE( DrcErrorLimitOverrideSupportsPrivateProposalValidation )
{
    auto board = makeHostRoutingBoard( false );
    auto drc = std::make_shared<DRC_ENGINE>( board.get(), &board->GetDesignSettings() );
    drc->InitEngine( wxFileName() );
    drc->SetErrorLimitOverride( DRCE_CLEARANCE, 1000 );
    drc->RunTests( EDA_UNITS::MM, true, false );
    BOOST_REQUIRE( drc->TestsCompleted() );
    BOOST_CHECK_EQUAL( drc->GetErrorLimit( DRCE_CLEARANCE ), 1000 );

    drc->SetErrorLimitOverride( DRCE_CLEARANCE, -1 );
    drc->RunTests( EDA_UNITS::MM, true, false );
    BOOST_REQUIRE( drc->TestsCompleted() );
    BOOST_CHECK_EQUAL( drc->GetErrorLimit( DRCE_CLEARANCE ), 499 );
}

BOOST_AUTO_TEST_CASE( HostSessionRepairsAPlaneSplitByTheNewRouting )
{
    auto board = makeHostRoutingBoard( true );
    auto settings = KICAD_BOARD_ADAPTER( board.get() ).CreateDefaultSettings();
    settings.enableFanout = false;
    // Preserve this fixture's deliberate initial same-layer plane split. The
    // production defaults now match Freerouting's 50/5 via costs and can
    // choose the clean via alternative without entering host repair.
    settings.viaCost = 500;
    settings.planeViaCost = 50;
    settings.maxPasses = 2;
    settings.maxIterations = 2;
    settings.optimizationPasses = 0;
    settings.maxExpandedNodes = 10000;
    const auto stamp = board->GetTimeStamp();
    auto result = KICAD_ROUTING_SESSION( *board ).Run( settings );
    BOOST_REQUIRE_MESSAGE( result.hostValidated, result.message );
    BOOST_REQUIRE_MESSAGE( result.complete, result.message );
    BOOST_CHECK_EQUAL( result.hostUnconnected, 0 );
    BOOST_CHECK_EQUAL( result.hostNewDrcViolations, 0 );
    BOOST_CHECK_GT( result.hostRepairPasses, 0 );
    BOOST_CHECK( board->Tracks().empty() );
    BOOST_CHECK_EQUAL( board->GetTimeStamp(), stamp );
    BOOST_CHECK( !board->Zones().front()->IsFilled() );
}

BOOST_AUTO_TEST_CASE( HostSessionCancellationDiscardsThePrivateProposal )
{
    auto board = makeHostRoutingBoard( false );
    auto settings = KICAD_BOARD_ADAPTER( board.get() ).CreateDefaultSettings();
    auto snapshot = KICAD_BOARD_ADAPTER( board.get() ).CreateSnapshot( settings );
    AUTOROUTER_JOB job( snapshot, settings, std::make_unique<KICAD_ROUTING_SESSION>( *board ) );
    job.Cancel();
    job.Start();
    job.Join();
    BOOST_REQUIRE( job.GetResult().has_value() );
    BOOST_CHECK( job.GetResult()->cancelled );
    BOOST_CHECK( !job.GetResult()->complete );
    BOOST_CHECK( !job.GetResult()->hostValidated );
    BOOST_CHECK( job.GetResult()->segments.empty() );
    BOOST_CHECK( board->Tracks().empty() );
}

BOOST_AUTO_TEST_CASE( HostSessionRunsRefillAndRepairOnTheJobWorker )
{
    auto board = makeHostRoutingBoard( true );
    auto settings = KICAD_BOARD_ADAPTER( board.get() ).CreateDefaultSettings();
    settings.enableFanout = false;
    settings.viaCost = 500;
    settings.planeViaCost = 50;
    settings.maxPasses = 2;
    settings.maxIterations = 2;
    settings.optimizationPasses = 0;
    settings.maxExpandedNodes = 10000;
    auto snapshot = KICAD_BOARD_ADAPTER( board.get() ).CreateSnapshot( settings );
    const auto stamp = board->GetTimeStamp();
    AUTOROUTER_JOB job( snapshot, settings, std::make_unique<KICAD_ROUTING_SESSION>( *board ) );
    job.Start();
    job.Join();
    const auto result = job.GetResult();
    BOOST_REQUIRE( result.has_value() );
    BOOST_REQUIRE_MESSAGE( result->complete, result->message );
    BOOST_CHECK( result->CanAcceptProposal() );
    BOOST_CHECK_GT( result->hostRepairPasses, 0 );
    BOOST_CHECK_EQUAL( result->hostUnconnected, 0 );
    BOOST_CHECK( board->Tracks().empty() );
    BOOST_CHECK_EQUAL( board->GetTimeStamp(), stamp );
    BOOST_CHECK( !board->Zones().front()->IsFilled() );
}

BOOST_AUTO_TEST_CASE( HostSessionDoesNotHideAnUnrepairablePlaneSplit )
{
    auto board = makeHostRoutingBoard( true );
    auto settings = KICAD_BOARD_ADAPTER( board.get() ).CreateDefaultSettings();
    settings.enableFanout = false;
    settings.allowVias = false;
    settings.maxPasses = 1;
    settings.maxIterations = 1;
    settings.optimizationPasses = 0;
    settings.maxExpandedNodes = 2000;
    const auto result = KICAD_ROUTING_SESSION( *board ).Run( settings );
    BOOST_REQUIRE_MESSAGE( result.hostValidated, result.message );
    BOOST_CHECK( !result.complete );
    BOOST_CHECK_GT( result.hostUnconnected, 0 );
    BOOST_CHECK_EQUAL( result.hostNewDrcViolations, 0 );
    BOOST_CHECK_EQUAL( result.hostRepairPasses, 0 );
    BOOST_CHECK( result.vias.empty() );
    BOOST_CHECK( board->Tracks().empty() );
    BOOST_CHECK( !board->Zones().front()->IsFilled() );
}

BOOST_AUTO_TEST_CASE( HostSessionCanCancelAfterRoutingBeforePlaneRepair )
{
    auto board = makeHostRoutingBoard( true );
    auto settings = KICAD_BOARD_ADAPTER( board.get() ).CreateDefaultSettings();
    settings.enableFanout = false;
    settings.viaCost = 500;
    settings.planeViaCost = 50;
    settings.maxPasses = 1;
    settings.maxIterations = 1;
    settings.optimizationPasses = 0;
    settings.maxExpandedNodes = 10000;
    const auto stamp = board->GetTimeStamp();
    KICAD_ROUTING_SESSION session( *board );
    bool cancelled = false;
    BOOST_CHECK_THROW( session.Run( settings, [&] { return cancelled; },
            [&]( const ROUTER_PROGRESS& state )
            {
                if( state.stage == "Repairing connections found after zone refill" )
                    cancelled = true;
            } ), std::runtime_error );
    BOOST_CHECK( cancelled );
    BOOST_CHECK( board->Tracks().empty() );
    BOOST_CHECK_EQUAL( board->GetTimeStamp(), stamp );
    BOOST_CHECK( !board->Zones().front()->IsFilled() );
    BOOST_CHECK_THROW( session.Run( settings ), std::logic_error );
}

BOOST_AUTO_TEST_CASE( ProposalAcceptanceRequiresHostValidationNotWorkerTaskSuccess )
{
    ROUTING_RESULT result;
    result.complete = true;
    BOOST_CHECK( !result.CanAcceptProposal() );
    result.hostValidated = true;
    result.hostUnconnected = 0;
    result.hostNewDrcViolations = 0;
    BOOST_CHECK( result.CanAcceptProposal() );
    result.complete = false;
    result.hostUnconnected = 1;
    BOOST_CHECK( result.CanAcceptProposal() ); // Deliberate, safe partial acceptance.
    result.hostNewDrcViolations = 1;
    BOOST_CHECK( !result.CanAcceptProposal() );
    result.hostNewDrcViolations = 0;
    result.metrics.drcViolations = 1;
    BOOST_CHECK( !result.CanAcceptProposal() );
    result.metrics.drcViolations = 0;
    result.cancelled = true;
    BOOST_CHECK( !result.CanAcceptProposal() );
}

BOOST_AUTO_TEST_CASE( RoomTreeCompletionNeighboursAndSectionsMatchPinnedFreerouting )
{
    std::ifstream input( KI_TEST::GetPcbnewTestDataDir() + "/autorouter/room-search-a11c0a42.txt" );
    BOOST_REQUIRE( input.good() );
    auto readBox = [&]()
    {
        ROUTER_BOX box;
        input >> box.minX >> box.minY >> box.maxX >> box.maxY;
        BOOST_REQUIRE( !input.fail() );
        return box;
    };
    auto marker = [&]( const char* expected )
    {
        std::string word;
        input >> word;
        BOOST_REQUIRE_EQUAL( word, expected );
    };
    auto checkBox = [&]( const ROUTER_BOX& actual )
    {
        const auto expected = readBox();
        BOOST_CHECK_EQUAL( actual.minX, expected.minX );
        BOOST_CHECK_EQUAL( actual.minY, expected.minY );
        BOOST_CHECK_EQUAL( actual.maxX, expected.maxX );
        BOOST_CHECK_EQUAL( actual.maxY, expected.maxY );
    };
    for( int test = 0; test < 256; ++test )
    {
        BOOST_TEST_CONTEXT( "Tree oracle " << test )
        {
            marker( "TREE" );
            int count;
            input >> count;
            SHAPE_SEARCH_TREE_90_DEGREE tree( { -100, -100, 100, 100 } );
            MIN_AREA_TREE order;
            std::map<int, std::pair<MIN_AREA_TREE::HANDLE, MIN_AREA_TREE::HANDLE>> handles;
            for( int i = 0; i < count; ++i )
            {
                SHAPE_TREE_ENTRY entry;
                int kind;
                input >> entry.objectId;
                entry.shape = readBox();
                input >> entry.layer >> entry.net >> kind;
                entry.isRoom = kind == 1;
                entry.obstacle = kind != 2;
                handles[entry.objectId] = { tree.Insert( entry ), order.Insert( entry ) };
            }
            marker( "REMOVE" );
            int removed;
            input >> removed;
            BOOST_CHECK( tree.Remove( handles.at( removed ).first ) );
            BOOST_CHECK( order.Remove( handles.at( removed ).second ) );
            BOOST_CHECK( !tree.Remove( handles.at( removed ).first ) );
            BOOST_CHECK( !order.Remove( handles.at( removed ).second ) );
            marker( "INSERT" );
            int inserted;
            input >> inserted;
            SHAPE_TREE_ENTRY extra{ readBox(), inserted };
            tree.Insert( extra );
            order.Insert( extra );
            marker( "VISIT" );
            int visits;
            input >> visits;
            BOOST_CHECK_EQUAL( order.Size(), visits );
            ROUTER_BOX all{ -1000, -1000, 1000, 1000 };
            order.Visit( all, [&]( const auto& entry )
            {
                int expected;
                input >> expected;
                BOOST_CHECK_EQUAL( entry.objectId, expected );
                return true;
            } );
            marker( "COMPLETE" );
            const auto room = readBox();
            const auto contained = readBox();
            int layer, net, ignored, hasIgnoreShape;
            input >> layer >> net >> ignored >> hasIgnoreShape;
            std::optional<ROUTER_BOX> ignoreShape;
            if( hasIgnoreShape ) ignoreShape = readBox();
            const auto completed = tree.CompleteShape( { room, layer, contained }, net,
                    ignored == 0 ? std::nullopt : std::optional<int>( ignored ), ignoreShape );
            marker( "RESULT" );
            int expectedRooms;
            input >> expectedRooms;
            BOOST_REQUIRE_EQUAL( completed.size(), expectedRooms );
            for( const auto& result : completed )
            {
                checkBox( result.GetShape() );
                checkBox( result.GetContainedShape() );
            }
            marker( "NEIGHBOURS" );
            const auto shape = readBox();
            auto entries = tree.Overlaps( shape );
            std::erase_if( entries, [&]( const auto& e )
            { return e.layer != layer || !e.IsTraceObstacle( net ); } );
            SORTED_ORTHOGONAL_ROOM_NEIGHBOURS sorted( shape, entries );
            int neighbours;
            input >> neighbours;
            BOOST_REQUIRE_EQUAL( sorted.Neighbours().size(), neighbours );
            for( const auto& neighbour : sorted.Neighbours() )
            {
                int id, first, last;
                input >> id;
                BOOST_CHECK_EQUAL( neighbour.entry.objectId, id );
                checkBox( neighbour.intersection );
                input >> first >> last;
                BOOST_CHECK_EQUAL( neighbour.firstSide, first );
                BOOST_CHECK_EQUAL( neighbour.lastSide, last );
            }
            int missingSide;
            input >> missingSide;
            BOOST_CHECK_EQUAL( sorted.FirstUnrestrainedSide(), missingSide );
            marker( "GAPS" );
            int gapCount;
            input >> gapCount;
            const auto gaps = sorted.IncompleteRooms( { -100, -100, 100, 100 }, layer );
            BOOST_REQUIRE_EQUAL( gaps.size(), gapCount );
            for( const auto& gap : gaps )
            {
                checkBox( gap.GetShape() );
                checkBox( gap.GetContainedShape() );
            }
        }
    }
    for( int test = 0; test < 256; ++test )
    {
        BOOST_TEST_CONTEXT( "Door oracle " << test )
        {
            marker( "DOOR" );
            COMPLETE_FREE_SPACE_EXPANSION_ROOM first( 1, 0, readBox() );
            COMPLETE_FREE_SPACE_EXPANSION_ROOM second( 2, 0, readBox() );
            EXPANSION_DOOR door( &first, &second );
            double offset;
            std::size_t count;
            input >> offset >> count;
            const auto sections = door.GetSectionSegments( offset );
            BOOST_REQUIRE_EQUAL( sections.size(), count );
            for( const auto& section : sections )
            {
                for( double coordinate : { section.a.x, section.a.y, section.b.x, section.b.y } )
                {
                    double expected;
                    input >> expected;
                    BOOST_CHECK_SMALL( coordinate - expected, 1e-9 );
                }
            }
        }
    }
    for( int test = 0; test < 256; ++test )
    {
        BOOST_TEST_CONTEXT( "Locator and distance oracle " << test )
        {
            marker( "CORNER" );
            FLOAT_POINT from, to, expected;
            bool horizontal, orthogonal;
            input >> from.x >> from.y >> to.x >> to.y >> horizontal >> orthogonal >> expected.x >> expected.y;
            const auto corner = FOUND_CONNECTION_LOCATOR_45_DEGREE::CalculateAdditionalCorner(
                    from, to, horizontal, orthogonal );
            BOOST_CHECK_SMALL( corner.x - expected.x, 1e-9 );
            BOOST_CHECK_SMALL( corner.y - expected.y, 1e-9 );
            marker( "DISTANCE" );
            double h, v, distance;
            input >> from.x >> from.y >> to.x >> to.y >> h >> v >> distance;
            BOOST_CHECK_SMALL( from.WeightedDistance( to, h, v ) - distance, 1e-9 );
        }
    }
    for( int test = 0; test < 256; ++test )
    {
        BOOST_TEST_CONTEXT( "Frontier ordering oracle " << test )
        {
            marker( "QUEUE" );
            auto read = [&]()
            {
                MAZE_LIST_ELEMENT element;
                int first, second;
                input >> element.sortingValue >> element.expansionValue >> first >> second >> element.sectionNoOfDoor;
                COMPLETE_FREE_SPACE_EXPANSION_ROOM a( first, 0, { 0, 0, 100, 100 } );
                COMPLETE_FREE_SPACE_EXPANSION_ROOM b( second, 0, { 100, 0, 200, 100 } );
                EXPANSION_DOOR door( &a, &b );
                element.doorId = door.GetId();
                return element;
            };
            const auto left = read(), right = read();
            int expected;
            input >> expected;
            const int comparison = left.SortKey() < right.SortKey() ? -1
                                   : left.SortKey() > right.SortKey() ? 1 : 0;
            BOOST_CHECK_EQUAL( comparison, expected );
        }
    }
    input >> std::ws;
    BOOST_CHECK( input.eof() );
}

BOOST_AUTO_TEST_CASE( ProductionNoViaSearchUsesRoomsAndRefreshesMutableObstacles )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    settings.allowVias = false;
    // This test verifies room-tree invalidation with a hard mutable wall, not
    // the production batch policy that may negotiate/rip that wall on pass 1.
    settings.allowRipupOnFirstIteration = false;
    settings.layers.resize( 1 );
    board.obstacles.push_back( { ROUTER_OBSTACLE_KIND::RECTANGLE, 0, { 0 }, {}, {},
                                { 2500000, 0, 3500000, 2200000 } } );
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy );
    int expanded = 0;
    const auto route = engine.FindConnection( board.pads[0], board.pads[1], 0, expanded, {} );
    BOOST_REQUIRE( route );
    BOOST_CHECK( engine.LastRoomSearchMetrics().routed );
    BOOST_CHECK_GT( engine.LastRoomSearchMetrics().rooms, 1 );
    BOOST_CHECK_LT( expanded, 1000 );
    for( std::size_t i = 1; i < route->nodes.size(); ++i )
        BOOST_CHECK( engine.CanUseSegment( 1, route->nodes[i - 1], route->nodes[i] ) );
    ROUTING_CONNECTION wall;
    wall.netCode = 2;
    wall.complete = true;
    wall.nodes = { { { 3000000, 2000000 }, 0 }, { { 3000000, 3000000 }, 0 } };
    occupancy.Add( wall );
    BOOST_CHECK( !engine.FindConnection( board.pads[0], board.pads[1], 0, expanded, {} ) );
    BOOST_CHECK( !engine.LastRoomSearchMetrics().routed );

    // Production pass one negotiates movable copper.  The room engine must
    // find that proposal itself rather than falling into the unbounded legacy
    // raster retry; insertion still sees the concrete wall as a victim.
    settings.allowRipupOnFirstIteration = true;
    MAZE_SEARCH_ENGINE negotiatedEngine( board, settings, occupancy );
    const auto negotiated = negotiatedEngine.FindConnection(
            board.pads[0], board.pads[1], 0, expanded, {} );
    BOOST_REQUIRE( negotiated );
    BOOST_CHECK( negotiatedEngine.LastRoomSearchMetrics().routed );
    const auto conflicts = negotiatedEngine.FindConflictingConnections( *negotiated );
    BOOST_REQUIRE_EQUAL( conflicts.size(), 1 );
    BOOST_CHECK( SameRouteGeometry( conflicts.front(), wall ) );

    occupancy.Remove( wall );
    BOOST_REQUIRE( engine.FindConnection( board.pads[0], board.pads[1], 0, expanded, {} ) );
    BOOST_CHECK( engine.LastRoomSearchMetrics().routed );
}

BOOST_AUTO_TEST_CASE( RoomSearchCrossesSubGridCorridorsWithoutGridExpansion )
{
    const ROUTER_BOX bounds{ 0, 0, 10000, 10000 };
    const std::vector<SHAPE_TREE_ENTRY> obstacles{
        { { 3000, 0, 4000, 8300 }, 1 },
        { { 6000, 8700, 7000, 10000 }, 2 },
        { { 6000, 0, 7000, 8100 }, 3 }
    };
    int expanded = 0;
    ROOM_SEARCH_METRICS metrics;
    const auto path = MAZE_SEARCH_ENGINE_90_DEGREE::FindConnection(
            bounds, obstacles, 0, 10, { { { 1000, 2000 }, { 1000, 2000 }, 7 } },
            { { { 9000, 2000 }, { 9000, 2000 }, 9 } }, 100, 1, 1,
            10000, expanded, metrics );
    BOOST_REQUIRE( path );
    BOOST_CHECK( metrics.routed );
    BOOST_CHECK_GT( metrics.rooms, 2 );
    BOOST_CHECK_GT( metrics.doors, 2 );
    BOOST_CHECK_GT( metrics.sections, 2 );
    BOOST_CHECK_EQUAL( path->startOwner, 7 );
    BOOST_CHECK_EQUAL( path->targetOwner, 9 );
    BOOST_CHECK( path->points.front() == ROUTER_POINT( { 1000, 2000 } ) );
    BOOST_CHECK( path->points.back() == ROUTER_POINT( { 9000, 2000 } ) );
    for( std::size_t i = 1; i < path->points.size(); ++i )
    {
        const auto a = path->points[i - 1];
        const auto b = path->points[i];
        BOOST_CHECK( a.x == b.x || a.y == b.y );
        BOOST_CHECK( bounds.Contains( a ) && bounds.Contains( b ) );
        for( const auto& obstacle : obstacles )
        {
            // Test the OPEN rectangle: input rectangles already include the
            // required physical clearance, so their boundary is centre-space.
            const auto overlap = INT_BOX::Intersection( obstacle.shape,
                    { std::min( a.x, b.x ), std::min( a.y, b.y ),
                      std::max( a.x, b.x ), std::max( a.y, b.y ) } );
            const bool insideX = overlap.minX < obstacle.shape.maxX
                                 && overlap.maxX > obstacle.shape.minX;
            const bool insideY = overlap.minY < obstacle.shape.maxY
                                 && overlap.maxY > obstacle.shape.minY;
            BOOST_CHECK( INT_BOX::Dimension( overlap ) < 0 || !insideX || !insideY );
        }
    }
    BOOST_CHECK_LT( expanded, 1000 );
}

BOOST_AUTO_TEST_CASE( RoomSearchStopsAtBudgetsCancellationAndSolidWalls )
{
    const ROUTER_BOX bounds{ 0, 0, 10000, 10000 };
    const std::vector<ROOM_TERMINAL> starts{ { { 1000, 5000 }, { 1000, 5000 }, 1 } };
    const std::vector<ROOM_TERMINAL> targets{ { { 9000, 5000 }, { 9000, 5000 }, 2 } };
    for( int budget : { 0, 1, 10, 1000 } )
    {
        int expanded = 0;
        ROOM_SEARCH_METRICS metrics;
        const auto result = MAZE_SEARCH_ENGINE_90_DEGREE::FindConnection(
                bounds, { { { 4000, 0, 5000, 10000 }, 1 } }, 0, 10, starts, targets,
                100, 1, 1, budget, expanded, metrics );
        BOOST_CHECK( !result );
        BOOST_CHECK_LE( expanded, budget );
    }
    int expanded = 0;
    ROOM_SEARCH_METRICS metrics;
    const auto result = MAZE_SEARCH_ENGINE_90_DEGREE::FindConnection(
            bounds, {}, 0, 10, starts, targets, 100, 1, 1, 1000, expanded, metrics,
            [] { return true; } );
    BOOST_CHECK( !result );
    BOOST_CHECK_EQUAL( expanded, 0 );
}

BOOST_AUTO_TEST_CASE( DoorSectionsRejectDisjointShapesAndPreserveNarrowDoors )
{
    EXPANSION_ROOM first( 1, 0, { 0, 0, 100, 100 } );
    EXPANSION_ROOM separate( 2, 0, { 200, 0, 300, 100 } );
    EXPANSION_DOOR invalid( &first, &separate );
    BOOST_CHECK_EQUAL( invalid.GetDimension(), -1 );
    BOOST_CHECK( invalid.GetSectionSegments( 10 ).empty() );
    EXPANSION_ROOM second( 3, 0, { 100, 40, 200, 50 } );
    EXPANSION_DOOR narrow( &first, &second );
    const auto sections = narrow.GetSectionSegments( 10 );
    BOOST_REQUIRE_EQUAL( sections.size(), 1 );
    BOOST_CHECK_EQUAL( sections[0].a.x, 100 );
    BOOST_CHECK_EQUAL( sections[0].a.y, 45 );
    BOOST_CHECK_EQUAL( sections[0].b.y, 45 );
    BOOST_CHECK( narrow.GetSectionSegments( 1e-12, 0, 0, 100 ).empty() );
    BOOST_CHECK( narrow.GetSectionSegments( std::numeric_limits<double>::infinity() ).empty() );
    BOOST_CHECK_EQUAL( FLOAT_POINT( { -1.5, -0.5 } ).Round().x, -1 );
    BOOST_CHECK_EQUAL( FLOAT_POINT( { -1.5, -0.5 } ).Round().y, 0 );
}

BOOST_AUTO_TEST_CASE( RoomSearchPreservesMultipleSourceAndTraceInteriorTargets )
{
    const ROUTER_BOX bounds{ 0, 0, 1000, 1000 };
    const std::vector<ROOM_TERMINAL> starts{ { { 50, 50 }, { 50, 50 }, 11 },
                                           { { 150, 600 }, { 150, 600 }, 12 } };
    const std::vector<ROOM_TERMINAL> targets{ { { 850, 200 }, { 850, 800 }, 20 } };
    std::optional<ROOM_PATH> previous;
    for( int repeat = 0; repeat < 3; ++repeat )
    {
        int expanded = 0;
        ROOM_SEARCH_METRICS metrics;
        const auto path = MAZE_SEARCH_ENGINE_90_DEGREE::FindConnection(
                bounds, {}, 0, 1, starts, targets, 10, 1, 1, 1000, expanded, metrics );
        BOOST_REQUIRE( path );
        BOOST_CHECK_EQUAL( path->startOwner, 12 );
        BOOST_CHECK_EQUAL( path->targetOwner, 20 );
        BOOST_CHECK( path->points.front() == ROUTER_POINT( { 150, 600 } ) );
        BOOST_CHECK( path->points.back() == ROUTER_POINT( { 850, 600 } ) );
        if( previous )
            BOOST_CHECK( path->points == previous->points );
        previous = path;
    }
}

BOOST_AUTO_TEST_CASE( RoomLocatorRespectsAnglesAndBendThreshold )
{
    const std::vector<RECTANGULAR_CORRIDOR_STEP> corridor{
        { { 0, 0, 100, 100 }, ROUTER_BOX{ 100, 50, 100, 100 }, { { 100, 50 }, { 100, 100 } } },
        { { 100, 50, 200, 150 }, {}, { { 180, 130 }, { 180, 130 } } }
    };
    for( bool orthogonal : { true, false } )
    {
        const auto result = FOUND_CONNECTION_LOCATOR_45_DEGREE::LocateRectangular( { 10, 10 }, corridor, orthogonal );
        BOOST_REQUIRE( result );
        BOOST_CHECK( result->front() == ROUTER_POINT( { 10, 10 } ) );
        BOOST_CHECK( result->back() == ROUTER_POINT( { 180, 130 } ) );
        double length = 0;
        for( std::size_t i = 1; i < result->size(); ++i )
        {
            const auto dx = std::abs( ( *result )[i].x - ( *result )[i - 1].x );
            const auto dy = std::abs( ( *result )[i].y - ( *result )[i - 1].y );
            BOOST_CHECK( dx == 0 || dy == 0 || ( !orthogonal && dx == dy ) );
            length += std::hypot( dx, dy );
        }
        if( !orthogonal )
            BOOST_CHECK_LT( length, 290 );
    }
    BOOST_CHECK_EQUAL( MAZE_LIST_ELEMENT::BendPenalty( { 0, 0 }, { 100, 0 }, { 200, 0 }, 17 ), 0 );
    BOOST_CHECK_EQUAL( MAZE_LIST_ELEMENT::BendPenalty( { 0, 0 }, { 100, 0 }, { 200, 5 }, 17 ), 0 );
    BOOST_CHECK_EQUAL( MAZE_LIST_ELEMENT::BendPenalty( { 0, 0 }, { 100, 0 }, { 100, 100 }, 17 ), 17 );
    BOOST_CHECK_EQUAL( MAZE_LIST_ELEMENT::BendPenalty( { 100, 0 }, { 100, 0 }, { 100, 100 }, 17 ), 0 );
}

BOOST_AUTO_TEST_CASE( RoomSearchFindsIndependentlyCertifiedRectangularPaths )
{
    std::mt19937 random( 230090 );
    const ROUTER_BOX bounds{ 0, 0, 100, 100 };
    int reachable = 0;
    for( int test = 0; test < 300; ++test )
    {
        BOOST_TEST_CONTEXT( "Rectangular connectivity case " << test )
        {
            std::vector<SHAPE_TREE_ENTRY> obstacles;
            for( int i = 0; i < 12; ++i )
            {
                const std::int64_t x = 10 + random() % 75;
                const std::int64_t y = 10 + random() % 75;
                obstacles.push_back( { { x, y, x + 1 + static_cast<std::int64_t>( random() % 14 ),
                                         y + 1 + static_cast<std::int64_t>( random() % 14 ) }, i + 1 } );
            }
            auto allowed = [&]( ROUTER_POINT a, ROUTER_POINT b )
            {
                if( !bounds.Contains( a ) || !bounds.Contains( b ) )
                    return false;
                const ROUTER_BOX segment{ std::min( a.x, b.x ), std::min( a.y, b.y ),
                                           std::max( a.x, b.x ), std::max( a.y, b.y ) };
                for( const auto& obstacle : obstacles )
                {
                    const auto& box = obstacle.shape;
                    if( segment.minX < box.maxX && segment.maxX > box.minX
                        && segment.minY < box.maxY && segment.maxY > box.minY )
                        return false;
                }
                return true;
            };
            // This fine graph is only a feasibility certificate in the TEST,
            // never a production fallback or a source of the expected polyline.
            bool seen[21][21]{};
            std::queue<std::pair<int, int>> queue;
            queue.emplace( 1, 1 );
            seen[1][1] = true;
            while( !queue.empty() )
            {
                const auto [x, y] = queue.front();
                queue.pop();
                for( auto [dx, dy] : { std::pair{ 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } } )
                {
                    const int nx = x + dx, ny = y + dy;
                    if( nx < 0 || ny < 0 || nx > 20 || ny > 20 || seen[nx][ny]
                        || !allowed( { x * 5, y * 5 }, { nx * 5, ny * 5 } ) )
                        continue;
                    seen[nx][ny] = true;
                    queue.emplace( nx, ny );
                }
            }
            if( !seen[19][19] )
                continue;
            ++reachable;
            int expanded = 0;
            ROOM_SEARCH_METRICS metrics;
            const auto path = MAZE_SEARCH_ENGINE_90_DEGREE::FindConnection(
                    bounds, obstacles, 0, 1, { { { 5, 5 }, { 5, 5 }, 0 } },
                    { { { 95, 95 }, { 95, 95 }, 1 } }, 1, 1, 1, 10000, expanded, metrics );
            BOOST_REQUIRE( path );
            BOOST_CHECK( path->points.front() == ROUTER_POINT( { 5, 5 } ) );
            BOOST_CHECK( path->points.back() == ROUTER_POINT( { 95, 95 } ) );
            for( std::size_t i = 1; i < path->points.size(); ++i )
            {
                const auto a = path->points[i - 1], b = path->points[i];
                BOOST_CHECK( a.x == b.x || a.y == b.y );
                BOOST_CHECK( allowed( a, b ) );
            }
        }
    }
    BOOST_CHECK_GE( reachable, 250 );
}


BOOST_AUTO_TEST_CASE( DrillGeometryAndExpansionCostsMatchPinnedFreerouting )
{
    std::ifstream input( KI_TEST::GetPcbnewTestDataDir() + "/autorouter/drill-search-a11c0a42.txt" );
    BOOST_REQUIRE( input.good() );
    auto marker = [&]( const char* expected )
    { std::string value; input >> value; BOOST_REQUIRE_EQUAL( value, expected ); };
    auto readBox = [&]()
    { ROUTER_BOX b; input >> b.minX >> b.minY >> b.maxX >> b.maxY; BOOST_REQUIRE( !input.fail() ); return b; };
    auto checkBox = [&]( ROUTER_BOX actual )
    {
        const auto expected = readBox();
        BOOST_CHECK_EQUAL( actual.minX, expected.minX ); BOOST_CHECK_EQUAL( actual.minY, expected.minY );
        BOOST_CHECK_EQUAL( actual.maxX, expected.maxX ); BOOST_CHECK_EQUAL( actual.maxY, expected.maxY );
    };
    for( int test = 0; test < 256; ++test )
    {
        BOOST_TEST_CONTEXT( "Cutout oracle " << test )
        {
            marker( "CUT" ); const auto d = readBox(), hole = readBox(); std::size_t count;
            input >> count;
            const auto pieces = INT_BOX::Cutout( d, hole );
            BOOST_REQUIRE_EQUAL( pieces.size(), count );
            for( auto piece : pieces ) checkBox( piece );
        }
    }
    for( int test = 0; test < 256; ++test )
    {
        BOOST_TEST_CONTEXT( "Drill free-region oracle " << test )
        {
            marker( "AREA" ); const auto border = readBox(); std::size_t count; input >> count;
            std::vector<ROUTER_BOX> holes;
            for( std::size_t i = 0; i < count; ++i ) holes.push_back( readBox() );
            const auto pieces = POLYLINE_AREA::SplitToConvex( border, holes );
            BOOST_REQUIRE( pieces ); input >> count; BOOST_REQUIRE_EQUAL( pieces->size(), count );
            for( auto piece : *pieces )
            {
                checkBox( piece ); int expectedId; input >> expectedId;
                const auto center = DRILL_PAGE( piece ).Center();
                BOOST_CHECK_EQUAL( INT_BOX::PointId( center ), expectedId );
            }
        }
    }
    for( int test = 0; test < 256; ++test )
    {
        BOOST_TEST_CONTEXT( "Drill page-array oracle " << test )
        {
            marker( "PAGES" ); const auto bounds = readBox(); std::int64_t width; input >> width;
            const auto query = readBox(); std::size_t count; input >> count;
            DRILL_PAGE_ARRAY array( bounds, width );
            BOOST_REQUIRE_EQUAL( array.Pages().size(), count );
            for( const auto& page : array.Pages() )
            { checkBox( page.Shape() ); int id; input >> id; BOOST_CHECK_EQUAL( page.GetId(), id ); }
            const auto overlapping = array.OverlappingPages( query );
            input >> count; BOOST_REQUIRE_EQUAL( overlapping.size(), count );
            for( auto* page : overlapping ) checkBox( page->Shape() );
        }
    }
    for( int test = 0; test < 256; ++test )
    {
        BOOST_TEST_CONTEXT( "Drill/page frontier-cost oracle " << test )
        {
            marker( "PAGE_COST" ); auto shape = readBox(); FLOAT_POINT from;
            double g, via, h, v, remaining, expectedG, expectedF;
            input >> from.x >> from.y >> g >> via >> h >> v >> remaining >> expectedG >> expectedF;
            const auto page = MAZE_EXPANSION_ENGINE::ToPage( shape, from, g, via, h, v, remaining );
            BOOST_CHECK_SMALL( page.expansion - expectedG, 1e-8 );
            BOOST_CHECK_SMALL( page.sorting - expectedF, 1e-8 );
            marker( "DRILL_COST" ); shape = readBox(); int fromPage, expectedId; FLOAT_POINT expected;
            input >> from.x >> from.y >> g >> via >> fromPage >> h >> v >> remaining
                  >> expectedG >> expectedF >> expected.x >> expected.y >> expectedId;
            const auto drill = MAZE_EXPANSION_ENGINE::ToDrill( shape, from, g, via, fromPage, h, v, remaining );
            BOOST_CHECK_SMALL( drill.expansion - expectedG, 1e-8 );
            BOOST_CHECK_SMALL( drill.sorting - expectedF, 1e-8 );
            BOOST_CHECK_SMALL( drill.entry.x - expected.x, 1e-8 );
            BOOST_CHECK_SMALL( drill.entry.y - expected.y, 1e-8 );
            EXPANSION_DRILL object; object.location = { 100, 100 }; object.firstLayer = 0; object.lastLayer = 1;
            BOOST_CHECK_EQUAL( object.GetId(), expectedId );
        }
    }
    input >> std::ws; BOOST_CHECK( input.eof() );
}

BOOST_AUTO_TEST_CASE( DrillPageCacheResetsStateButInvalidatesGeometryOnMutation )
{
    DRILL_PAGE page( { -100, -100, 100, 100 } );
    std::vector<SHAPE_TREE_ENTRY> obstacles{ { { -5, -100, 5, 100 }, 1, 0, 2, 2 } };
    auto* drills = page.GetDrills( obstacles, 1, 3 );
    BOOST_REQUIRE( drills ); BOOST_REQUIRE_EQUAL( drills->size(), 2 );
    BOOST_CHECK( page.IsValid() );
    const auto first = drills->front().location;
    drills->front().occupied[1] = true;
    page.Reset();
    BOOST_REQUIRE_EQUAL( page.GetDrills( obstacles, 1, 3 )->size(), 2 );
    BOOST_CHECK( !drills->front().occupied[1] );
    BOOST_CHECK( drills->front().location == first );
    // A net change makes foreign copper drillable; no stale cached cutouts.
    BOOST_REQUIRE_EQUAL( page.GetDrills( obstacles, 2, 3 )->size(), 1 );
    page.Invalidate(); BOOST_CHECK( !page.IsValid() );
    BOOST_REQUIRE_EQUAL( page.GetDrills( {}, 1, 3 )->size(), 1 );
    page.Invalidate();
    BOOST_CHECK( !page.GetDrills( obstacles, 1, 3, false, {}, [] { return true; } ) );
    BOOST_CHECK( !page.IsValid() );
    BOOST_CHECK( !page.GetDrills( obstacles, 1, 3, false, {}, {}, 1 ) );
    BOOST_REQUIRE_EQUAL( page.GetDrills( obstacles, 1, 3 )->size(), 2 );
    // Attachment selection: last eligible top pin, otherwise bottom, strict
    // interior only. Changing attach policy invalidates the cache as well.
    page.Invalidate();
    const std::vector<DRILL_PIN> pins{ { { 1, 2 }, 0, true }, { { 3, 4 }, 0, true },
                                     { { 7, 8 }, 2, true }, { { 100, 0 }, 0, true } };
    BOOST_CHECK( page.GetDrills( {}, 1, 3, true, pins )->front().location == ( ROUTER_POINT{ 3, 4 } ) );
    BOOST_CHECK( page.GetDrills( {}, 1, 3, false, pins )->front().location == ( ROUTER_POINT{ 0, 0 } ) );
}

BOOST_AUTO_TEST_CASE( MultilayerRoomSearchUsesDrillSectionsAndFullPhysicalStack )
{
    ROOM_LAYER a, b, inactive;
    a.id = 0; inactive.id = 17; inactive.active = false; b.id = 31;
    a.bounds = b.bounds = inactive.bounds = { 0, 0, 10000, 10000 };
    a.starts = { { { 1000, 5000 }, { 1000, 5000 }, 7 } };
    b.targets = { { { 9000, 5000 }, { 9000, 5000 }, 9 } };
    ROOM_VIA_SETTINGS via;
    via.bounds = a.bounds; via.pageWidth = 2000; via.normalCost = 1000;
    via.canDrill = []( auto ) { return true; };
    auto route = [&]( const auto& layers )
    {
        int expanded = 0; ROOM_SEARCH_METRICS metrics;
        auto found = MAZE_SEARCH_ENGINE_90_DEGREE::FindMultilayerConnection(
                layers, 1, 100, via, 10000, expanded, metrics, {}, {}, true );
        BOOST_REQUIRE( found ); BOOST_CHECK_GT( metrics.drillPages, 0 );
        BOOST_CHECK_GT( metrics.drills, 0 ); BOOST_CHECK_GT( metrics.layerTransitions, 0 );
        BOOST_CHECK_LT( expanded, 1000 );
        BOOST_CHECK_EQUAL( found->startOwner, 7 ); BOOST_CHECK_EQUAL( found->targetOwner, 9 );
        int transitions = 0;
        for( std::size_t i = 1; i < found->nodes.size(); ++i )
        {
            const auto& from = found->nodes[i - 1]; const auto& to = found->nodes[i];
            BOOST_CHECK_NE( to.layer, 17 );
            if( from.layer != to.layer ) { ++transitions; BOOST_CHECK( from.point == to.point ); }
        }
        return transitions;
    };
    BOOST_CHECK_EQUAL( route( std::vector{ a, inactive, b } ), 1 );
    // A foreign plane/obstacle on an inactive inner layer blocks the physical
    // through-drill. It is not enough that its trace entry/exit layers are clear.
    inactive.obstacles = { { inactive.bounds, 1, 0, inactive.id } };
    int expanded = 0; ROOM_SEARCH_METRICS metrics;
    BOOST_CHECK( !MAZE_SEARCH_ENGINE_90_DEGREE::FindMultilayerConnection(
            { a, inactive, b }, 1, 100, via, 10000, expanded, metrics ) );
    inactive.obstacles.clear();
    a.targets = b.targets; b.targets.clear();
    a.obstacles = { { { 4500, 0, 5500, 10000 }, 1, 0, 0 } };
    via.obstacles = a.obstacles;
    BOOST_CHECK_EQUAL( route( std::vector{ a, b } ), 2 );
    expanded = 0; metrics = {};
    BOOST_CHECK( !MAZE_SEARCH_ENGINE_90_DEGREE::FindMultilayerConnection(
            { a, b }, 1, 100, via, 1, expanded, metrics ) );
    expanded = 0;
    BOOST_CHECK( !MAZE_SEARCH_ENGINE_90_DEGREE::FindMultilayerConnection(
            { a, b }, 1, 100, via, 10000, expanded, metrics, [] { return true; } ) );
    via.canDrill = []( auto ) { return false; }; expanded = 0;
    BOOST_CHECK( !MAZE_SEARCH_ENGINE_90_DEGREE::FindMultilayerConnection(
            { a, b }, 1, 100, via, 10000, expanded, metrics ) );
}

BOOST_AUTO_TEST_CASE( ProductionMultilayerRoutingUsesTheRoomDrillFrontier )
{
    auto board = makeBoard(); auto settings = makeSettings();
    settings.enableFanout = false;
    board.pads[0].layers = { 0 }; board.pads[1].layers = { 1 };
    board.pads[0].isSmd = board.pads[1].isSmd = true;
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU ); occupancy.InitializeBoard( board, settings );
    MAZE_SEARCH_ENGINE engine( board, settings, occupancy ); int expanded = 0;
    const auto path = engine.FindConnection( board.pads[0], board.pads[1], 0, expanded, {} );
    BOOST_REQUIRE( path ); BOOST_CHECK( engine.LastRoomSearchMetrics().routed );
    BOOST_CHECK_GT( engine.LastRoomSearchMetrics().drillPages, 0 );
    BOOST_CHECK_GT( engine.LastRoomSearchMetrics().layerTransitions, 0 );
    BOOST_CHECK_GT( engine.LastRoomSearchMetrics().destinationQueries, 0 );
    for( std::size_t i = 1; i < path->nodes.size(); ++i )
    {
        const auto& a = path->nodes[i - 1]; const auto& b = path->nodes[i];
        BOOST_CHECK( engine.CanUseSegment( 1, a, b ) );
        if( a.layer != b.layer )
        {
            BOOST_CHECK( a.point != board.pads[0].position );
            BOOST_CHECK( a.point != board.pads[1].position );
        }
    }
    settings.allowViaInSmdPad = true;
    auto attachBoard = board;
    // Pin-centre substitution requires strict interior containment. The
    // original y=1.5mm lies exactly on a drill-page boundary in this fixture.
    for( auto& pad : attachBoard.pads )
        pad.position.y -= 200000;
    // One page makes its substituted pin centre the only drill candidate;
    // allowing via-in-pad does not otherwise require the queue to choose it.
    attachBoard.nets[0].viaDiameter = 1200000;
    ROUTING_OCCUPANCY attachOccupancy( settings.gridStepIU );
    attachOccupancy.InitializeBoard( attachBoard, settings );
    MAZE_SEARCH_ENGINE attachEngine( attachBoard, settings, attachOccupancy );
    expanded = 0;
    const auto attached = attachEngine.FindConnection( attachBoard.pads[0], attachBoard.pads[1], 0, expanded, {} );
    BOOST_REQUIRE( attached );
    BOOST_CHECK( attachEngine.LastRoomSearchMetrics().routed );
    bool attachedToPin = false;
    for( std::size_t i = 1; i < attached->nodes.size(); ++i )
        if( attached->nodes[i - 1].layer != attached->nodes[i].layer )
        {
            attachedToPin |= attached->nodes[i].point == attachBoard.pads[0].position
                             || attached->nodes[i].point == attachBoard.pads[1].position;
        }
    BOOST_CHECK( attachedToPin );
}

BOOST_AUTO_TEST_CASE( TraceTailCleanupSplitsAtInteriorViaAndKeepsTheUsefulTrunk )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.pads[0].layers = { 0 };
    board.pads[1].layers = { 1 };
    ROUTING_CONNECTION trunk;
    trunk.complete = true; trunk.netCode = 1;
    trunk.nodes = { { board.pads[0].position, 0 }, { { 4000000, 1500000 }, 0 } };
    auto via = trunk, branch = trunk;
    via.nodes = { { { 3000000, 1500000 }, 0 }, { { 3000000, 1500000 }, 1 } };
    branch.nodes = { via.nodes.back(), { board.pads[1].position, 1 } };
    std::vector<ROUTING_CONNECTION> routes{ trunk, via, branch };
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    for( const auto& route : routes )
        occupancy.Add( route );
    BOOST_REQUIRE( occupancy.Board()->Connected( 0, 1 ) );
    BATCH_OPTIMIZER optimizer( board, settings, occupancy );
    optimizer.RemoveRedundantViaTails( routes, [] { return true; } );
    BOOST_REQUIRE_EQUAL( routes.size(), 3 );
    BOOST_CHECK( routes.front().nodes == trunk.nodes );
    BOOST_CHECK( occupancy.Board()->Connected( 0, 1 ) );
    optimizer.RemoveRedundantViaTails( routes, {} );
    BOOST_REQUIRE_EQUAL( routes.size(), 3 );
    BOOST_REQUIRE_EQUAL( routes.front().nodes.size(), 2 );
    BOOST_CHECK( routes.front().nodes.back() == via.nodes.front() );
    BOOST_CHECK( routes[1].nodes == via.nodes );
    BOOST_CHECK( occupancy.Board()->Connected( 0, 1 ) );
    for( const auto& route : routes )
    {
        occupancy.Remove( route );
        BOOST_CHECK( occupancy.Board()->HasCopperAt( 1, route.nodes.front() ) );
        BOOST_CHECK( occupancy.Board()->HasCopperAt( 1, route.nodes.back() ) );
        occupancy.Add( route );
    }
    optimizer.RemoveRedundantViaTails( routes, {} );
    BOOST_CHECK( routes.front().nodes.back() == via.nodes.front() );
    BOOST_CHECK_EQUAL( occupancy.Connections().size(), routes.size() );
}


BOOST_AUTO_TEST_CASE( RequiredFanoutViaIsNeverReducedToADanglingSourceStub )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.pads[0].layers = { 0 };
    board.pads[1].layers = { 1 };

    ROUTING_CONNECTION fanout;
    fanout.complete = true;
    fanout.isFanoutConnection = true;
    fanout.netCode = 1;
    fanout.fromPadIndex = 0;
    fanout.nodes = { { board.pads[0].position, 0 }, { { 3000000, 1500000 }, 0 },
                     { { 3000000, 1500000 }, 1 } };
    ROUTING_CONNECTION branch;
    branch.complete = true;
    branch.netCode = 1;
    branch.toPadIndex = 1;
    branch.nodes = { fanout.nodes.back(), { board.pads[1].position, 1 } };

    std::vector<ROUTING_CONNECTION> routes{ fanout, branch };
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    occupancy.Add( fanout );
    occupancy.Add( branch );
    BOOST_REQUIRE( occupancy.Board()->Connected( 0, 1 ) );

    BATCH_OPTIMIZER( board, settings, occupancy ).RemoveRedundantViaTails( routes, {} );
    BOOST_REQUIRE_EQUAL( routes.size(), 2U );
    BOOST_CHECK( routes.front().nodes == fanout.nodes );
    BOOST_CHECK( occupancy.Board()->Connected( 0, 1 ) );
}


BOOST_AUTO_TEST_CASE( OptimizerPromotesChangedAutorouterOwnedCopperForHostReplacement )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    board.pads[0].layers = { 0 };
    board.pads[1].layers = { 1 };

    ROUTING_CONNECTION trunk;
    trunk.complete = true;
    trunk.netCode = 1;
    trunk.isExistingBoardRoute = true;
    trunk.isAutorouterOwned = true;
    trunk.sourceBoardItemIds = { "job-owned-tail" };
    trunk.nodes = { { board.pads[0].position, 0 }, { { 4000000, 1500000 }, 0 } };
    ROUTING_CONNECTION via;
    via.complete = true;
    via.netCode = 1;
    via.nodes = { { { 3000000, 1500000 }, 0 }, { { 3000000, 1500000 }, 1 } };
    ROUTING_CONNECTION branch;
    branch.complete = true;
    branch.netCode = 1;
    branch.nodes = { via.nodes.back(), { board.pads[1].position, 1 } };

    std::vector<ROUTING_CONNECTION> routes{ trunk, via, branch };
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    for( const auto& route : routes )
        occupancy.Add( route );
    BOOST_REQUIRE( occupancy.Board()->Connected( 0, 1 ) );

    BATCH_OPTIMIZER( board, settings, occupancy ).RemoveRedundantViaTails( routes, {} );
    BOOST_REQUIRE_EQUAL( routes.size(), 3U );
    BOOST_CHECK( routes.front().nodes.back() == via.nodes.front() );
    BOOST_CHECK( !routes.front().isExistingBoardRoute );
    BOOST_CHECK( !routes.front().isAutorouterOwned );
    BOOST_CHECK_EQUAL_COLLECTIONS(
            routes.front().sourceBoardItemIds.begin(), routes.front().sourceBoardItemIds.end(),
            trunk.sourceBoardItemIds.begin(), trunk.sourceBoardItemIds.end() );
    BOOST_CHECK( occupancy.Board()->Connected( 0, 1 ) );
}

BOOST_AUTO_TEST_CASE( TraceJunctionsUseRealCopperLayerAndExactIntersection )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_BOARD copper( board, settings );
    ROUTING_CONNECTION diagonal;
    diagonal.complete = true; diagonal.netCode = 1;
    diagonal.nodes = { { { 2000000, 500000 }, 0 }, { { 4000000, 2500000 }, 0 } };
    copper.AddRoute( diagonal );
    ROUTER_NODE a{ { 1500000, 1500000 }, 0 }, b{ { 4500000, 1500000 }, 0 };
    const ROUTER_POINT junction{ 3000000, 1500000 };
    auto points = copper.TraceJunctions( 1, a, b );
    BOOST_REQUIRE_EQUAL( points.size(), 1 );
    BOOST_CHECK( points.front() == junction );
    BOOST_CHECK( copper.TraceJunctions( 2, a, b ).empty() );
    a.layer = b.layer = 1;
    BOOST_CHECK( copper.TraceJunctions( 1, a, b ).empty() );
    BOOST_CHECK( !copper.HasCopperAt( 1, { junction, 1 } ) );
    BOOST_CHECK( copper.HasCopperAt( 1, { junction, 0 } ) );
    auto through = diagonal;
    through.nodes = { { junction, 0 }, { junction, 1 } };
    copper.AddRoute( through );
    BOOST_CHECK( copper.HasCopperAt( 1, { junction, 1 } ) );
    points = copper.TraceJunctions( 1, a, b );
    BOOST_REQUIRE_EQUAL( points.size(), 1 );
    BOOST_CHECK( points.front() == junction );
}

BOOST_AUTO_TEST_CASE( OverlappingTraceEndIsTrimmedToTheJunctionNotKeptAsADoubledStub )
{
    auto board = makeBoard();
    auto settings = makeSettings();
    ROUTING_CONNECTION left;
    left.complete = true; left.netCode = 1;
    left.nodes = { { board.pads[0].position, 0 }, { { 3500000, 1500000 }, 0 } };
    auto right = left;
    right.nodes = { { { 3000000, 1500000 }, 0 }, { board.pads[1].position, 0 } };
    std::vector<ROUTING_CONNECTION> routes{ left, right };
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    occupancy.Add( left ); occupancy.Add( right );
    BOOST_REQUIRE( occupancy.Board()->Connected( 0, 1 ) );
    BATCH_OPTIMIZER( board, settings, occupancy ).RemoveRedundantViaTails( routes, {} );
    BOOST_REQUIRE_EQUAL( routes.size(), 2 );
    BOOST_CHECK( routes.front().nodes.back() == routes.back().nodes.front() );
    BOOST_CHECK( routes.front().nodes.front() == left.nodes.front() );
    BOOST_CHECK( routes.back().nodes.back() == right.nodes.back() );
    BOOST_CHECK( occupancy.Board()->Connected( 0, 1 ) );
}

BOOST_AUTO_TEST_CASE( DrillAllocationIsValidatedAndUnusedGridsAreNotConstructed )
{
    const ROUTER_BOX empty{ 0, 0, 0, 1 }, bounds{ 0, 0, 100, 100 };
    const ROUTER_BOX huge{ 0, 0, std::numeric_limits<std::int64_t>::max(), 100 };
    const ROUTER_BOX overflow{ std::numeric_limits<std::int64_t>::min(), 0,
                               std::numeric_limits<std::int64_t>::max(), 100 };
    BOOST_CHECK_THROW( DRILL_PAGE_ARRAY( empty, 10 ), std::invalid_argument );
    BOOST_CHECK_THROW( DRILL_PAGE_ARRAY( bounds, 0 ), std::invalid_argument );
    BOOST_CHECK_THROW( DRILL_PAGE_ARRAY( huge, 1 ), std::length_error );
    BOOST_CHECK_THROW( DRILL_PAGE_ARRAY( overflow, 1 ), std::length_error );
    auto board = makeBoard();
    auto settings = makeSettings();
    board.bounds = { 0, 0, 2000000000, 2000000000 };
    board.nets[0].viaDiameter = 0;
    settings.maxExpandedNodes = 1;
    settings.allowVias = false;
    ROUTING_OCCUPANCY occupancy( settings.gridStepIU );
    occupancy.InitializeBoard( board, settings );
    AUTOROUTE_ENGINE engine( board, settings, occupancy );
    int expanded = 0;
    BOOST_CHECK( !engine.AutorouteConnection( board.pads[0], board.pads[1], 0, expanded, {} ) );
    BOOST_CHECK_LE( expanded, 1 );
}

BOOST_AUTO_TEST_CASE( DestinationDistanceMatchesPinnedJavaForAllLayerStates )
{
    std::ifstream input( KI_TEST::GetPcbnewTestDataDir() + "/autorouter/destination-search-a11c0a42.txt" );
    BOOST_REQUIRE( input.good() );
    auto readBox = [&]()
    {
        ROUTER_BOX box;
        input >> box.minX >> box.minY >> box.maxX >> box.maxY;
        BOOST_REQUIRE( !input.fail() );
        return box;
    };
    for( int record = 0; record < 7712; ++record )
    {
        BOOST_TEST_CONTEXT( "DestinationDistance Java record " << record )
        {
            std::string marker;
            int count, activeMask;
            double normal, cheap;
            input >> marker >> count >> activeMask >> normal >> cheap;
            BOOST_REQUIRE_EQUAL( marker, "DEST" );
            BOOST_REQUIRE( count > 0 && count <= 6 );
            std::vector<DESTINATION_DISTANCE::EXPANSION_COST_FACTOR> costs( count );
            std::vector<bool> active( count );
            for( int layer = 0; layer < count; ++layer )
            {
                input >> costs[layer].horizontal >> costs[layer].vertical;
                active[layer] = ( activeMask & ( 1 << layer ) ) != 0;
            }
            DESTINATION_DISTANCE distance( costs, active, normal, cheap );
            int targets;
            input >> targets;
            BOOST_REQUIRE( targets >= 0 && targets <= 18 );
            for( int i = 0; i < targets; ++i )
            {
                int layer;
                input >> layer;
                distance.Join( readBox(), layer );
            }
            int layer;
            input >> layer;
            const auto box = readBox();
            FLOAT_POINT point;
            double expectedBox, expectedPoint, expectedCheap, expectedRestored;
            input >> point.x >> point.y >> expectedBox >> expectedPoint >> expectedCheap >> expectedRestored;
            BOOST_REQUIRE( !input.fail() );
            auto check = []( double actual, double expected )
            { BOOST_CHECK_EQUAL( std::bit_cast<std::uint64_t>( actual ), std::bit_cast<std::uint64_t>( expected ) ); };
            check( distance.Calculate( box, layer ), expectedBox );
            check( distance.Calculate( point, layer ), expectedPoint );
            check( distance.CalculateCheapDistance( box, layer ), expectedCheap );
            check( distance.Calculate( box, layer ), expectedRestored );
        }
    }
    input >> std::ws;
    BOOST_CHECK( input.eof() );
}

BOOST_AUTO_TEST_CASE( DestinationDistanceUsesSourceUnionAndPointBoundingBox )
{
    DESTINATION_DISTANCE distance( { { 1, 4 }, { 3, 1 } }, { true, true }, 10, 8 );
    BOOST_CHECK_EQUAL( distance.Calculate( FLOAT_POINT{ 50, 50 }, 0 ),
                       DESTINATION_DISTANCE::REFERENCE_MAX_COST );
    distance.Join( { 0, 100, 0, 100 }, 0 );
    distance.Join( { 100, 0, 100, 0 }, 0 );
    // Reference measures to the UNION box. The previous min-over-terminals
    // substitute returned a positive value for this gap between targets.
    BOOST_CHECK_EQUAL( distance.Calculate( FLOAT_POINT{ 50, 50 }, 0 ), 0 );
    const auto box = FLOAT_POINT{ -0.5, 0.25 }.BoundingBox();
    BOOST_CHECK_EQUAL( box.minX, -1 );
    BOOST_CHECK_EQUAL( box.maxX, 0 );
    BOOST_CHECK_EQUAL( box.minY, 0 );
    BOOST_CHECK_EQUAL( box.maxY, 1 );
    BOOST_CHECK_EQUAL( distance.Calculate( FLOAT_POINT{ -0.5, 0.25 }, 0 ), 0 );
    BOOST_CHECK_THROW( distance.Join( { 0, 0, 1, 1 }, 2 ), std::out_of_range );
    BOOST_CHECK_THROW( distance.Calculate( FLOAT_POINT{ 0, 0 }, -1 ), std::out_of_range );
    BOOST_CHECK_THROW( DESTINATION_DISTANCE( {}, {}, 0, 0 ), std::invalid_argument );
    BOOST_CHECK_THROW( DESTINATION_DISTANCE( { { 1, 1 } }, { true, true }, 0, 0 ), std::invalid_argument );
}

BOOST_AUTO_TEST_CASE( ReferenceCostAdapterPreservesUnitsAndEmptySentinels )
{
    const ROOM_COST_SPACE units( { -200000000, -200000000, 200000000, 200000000 } );
    BOOST_CHECK_EQUAL( units.Scale(), 100 );
    const auto box = units.ToReference( ROUTER_BOX{ -150, 25, -50, 75 } );
    BOOST_CHECK_EQUAL( box.minX, -2 );
    BOOST_CHECK_EQUAL( box.minY, 0 );
    BOOST_CHECK_EQUAL( box.maxX, 0 );
    BOOST_CHECK_EQUAL( box.maxY, 1 );
    DESTINATION_DISTANCE distance( { { 1, 1 }, { 1, 1 } }, { true, true },
            units.ToReferenceCost( 5000000 ), units.ToReferenceCost( 4000000 ) );
    distance.Join( units.ToReference( ROUTER_BOX{ 100000000, 0, 100000000, 0 } ), 1 );
    const double cost = units.ToNativeCost( distance.Calculate( units.ToReference( FLOAT_POINT{ 0, 0 } ), 0 ) );
    BOOST_CHECK_EQUAL( cost, 105000000 );
    BOOST_CHECK_THROW( units.ToReference( INT_BOX::Empty() ), std::invalid_argument );
    // A raw IU query exceeds the Java EMPTY-box range. This is precisely why
    // the unit conversion must not be folded into heuristic-specific branches.
    const ROOM_COST_SPACE large( { std::numeric_limits<std::int64_t>::min(), 0,
                                  std::numeric_limits<std::int64_t>::max(), 100 } );
    BOOST_CHECK( std::isfinite( large.Scale() ) );
    BOOST_CHECK_LT( std::abs( large.ToReference( FLOAT_POINT{ -9e18, 0 } ).x ) * 5,
                    DESTINATION_DISTANCE::REFERENCE_COORDINATE_LIMIT );
    BOOST_CHECK_THROW( ( ROOM_COST_SPACE{ INT_BOX::Empty() } ), std::invalid_argument );
}


BOOST_AUTO_TEST_SUITE_END()
