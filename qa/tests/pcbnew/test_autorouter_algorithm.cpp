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
#include <autorouter/board/KicadBoardAdapter.h>
#include <autorouter/drc/DesignRulesChecker.h>
#include <autorouter/pipeline/BatchFanout.h>
#include <autorouter/pipeline/RoutingPipeline.h>

#include <board.h>
#include <pcbnew_utils/board_file_utils.h>

#include <boost/test/unit_test.hpp>


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
    BOOST_CHECK_EQUAL( result.metrics.fanoutConnections, 2 );
    BOOST_CHECK_EQUAL( result.vias.size(), 2 );
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


BOOST_AUTO_TEST_CASE( UsesVisibilityLandmarksWhenGridDoesNotAlign )
{
    BOARD_SNAPSHOT board = makeBoard();
    board.obstacles.push_back( { ROUTER_OBSTACLE_KIND::RECTANGLE,
                                 0,
                                 { 0 },
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
    BOOST_CHECK_EQUAL( result.connections[1].fromPadIndex, 1 );
    BOOST_CHECK_EQUAL( result.connections[1].toPadIndex, 2 );
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


BOOST_AUTO_TEST_SUITE_END()
