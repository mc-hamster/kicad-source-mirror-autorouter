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
#include <autorouter/path/FoundConnectionInserter.h>
#include <autorouter/maze/MazeSearchEngine90Degree.h>
#include <autorouter/maze/MazeExpansionEngine.h>
#include <autorouter/drill/DrillPageArray.h>
#include <autorouter/geometry/planar/PolylineArea.h>
#include <autorouter/maze/MazeListElement.h>
#include <autorouter/path/FoundConnectionLocator45Degree.h>
#include <autorouter/expansion/ExpansionDoor.h>
#include <autorouter/expansion/CompleteFreeSpaceExpansionRoom.h>
#include <autorouter/expansion/SortedOrthogonalRoomNeighbours.h>
#include <autorouter/maze/DestinationDistance.h>
#include <autorouter/maze/LegacyDestinationDistance.h>
#include <autorouter/maze/RoomCostSpace.h>
#include <autorouter/BoardHistory.h>
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
#include <bit>
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
    net.padIndices = { 0, 1 };
    net.connections = { { 0, 1 } };
    board.nets.push_back( net );

    AUTOROUTER_SETTINGS settings = makeSettings();
    settings.enableFanout = true;
    settings.maxFanoutPasses = 1;

    ROUTING_PIPELINE pipeline;
    const ROUTING_RESULT result = pipeline.Run( board, settings, {}, {} );

    BOOST_REQUIRE( result.complete );
    BOOST_CHECK_EQUAL( result.metrics.routedConnections, 3 );
    const auto prepared = BATCH_FANOUT::PrepareSnapshot( board, settings );
    BOOST_CHECK_EQUAL( prepared.pads.size() - board.pads.size(), 2 );
    // The connected-set frontier may retain either escape stub as useful
    // same-layer copper. Count that retained work, not a legacy path ordering.
    BOOST_CHECK_LE( result.metrics.fanoutConnections, 2 );
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
    ROUTING_NET a; a.netCode = 1; a.padIndices = { 0, 3 }; a.connections = { { 0, 3 } };
    ROUTING_NET b; b.netCode = 2; b.padIndices = { 1, 2 }; b.connections = { { 1, 2 } };
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

BOOST_AUTO_TEST_CASE( HostSessionRepairsAPlaneSplitByTheNewRouting )
{
    auto board = makeHostRoutingBoard( true );
    auto settings = KICAD_BOARD_ADAPTER( board.get() ).CreateDefaultSettings();
    settings.enableFanout = false;
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
