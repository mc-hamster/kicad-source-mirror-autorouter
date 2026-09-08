/* KiCad, GPL-3.0-or-later. Exact geometry / recursive spring-over parity. */
#include "autorouter_geometry_oracle.h"
#include <pcbnew_utils/board_file_utils.h>
#include <boost/test/unit_test.hpp>
#include <fstream>

using namespace KICAD_AUTOROUTER;
using namespace KICAD_AUTOROUTER::PLANAR;

BOOST_AUTO_TEST_SUITE( NativeAutorouter )
BOOST_AUTO_TEST_CASE( ConvexGeometryMatchesPinnedJava )
{
    for( const auto& name : { "convex", "spring" } )
    {
        const auto path = KI_TEST::GetPcbnewTestDataDir() + "/autorouter/" + name + "-search-a11c0a42.txt";
        std::ifstream in( path ); BOOST_REQUIRE( in.good() );
        std::string line; int count = 0;
        while( std::getline( in, line ) )
        {
            ++count;
            BOOST_TEST_CONTEXT( name << " record " << count )
            { BOOST_CHECK_MESSAGE( AUTOROUTER_GEOMETRY_QA::CheckRecord( line ).empty(), AUTOROUTER_GEOMETRY_QA::CheckRecord( line ) ); }
        }
        BOOST_CHECK_EQUAL( count, std::string( name ) == "convex" ? 2048 : 384 );
    }
}
BOOST_AUTO_TEST_CASE( ExactRationalGeometryRejectsRoundingAndInvalidShapes )
{
    // x is integral but y is not: do not accidentally divide both numerators
    // merely because ONE coordinate is divisible by the denominator.
    POINT p( 6, 1, 2 );
    BOOST_CHECK( !p.Integral() );
    BOOST_CHECK( p == POINT( -12, -2, -4 ) );
    const INTEGER huge = INTEGER( 1 ) << 400;
    BOOST_CHECK( POINT( huge * 6, huge, huge * 2 ) == p );
    BOOST_CHECK_EQUAL( p.CompareX( POINT( 3, 9 ) ), 0 );
    BOOST_CHECK_EQUAL( p.CompareY( POINT( 3, 1 ) ), -1 );
    BOOST_CHECK_THROW( POINT( 1, 1, 0 ), std::domain_error );
    LINE a( { INT64_MIN, INT64_MIN }, { INT64_MAX, INT64_MAX } );
    LINE b( { INT64_MIN, INT64_MAX }, { INT64_MAX, INT64_MIN } );
    const auto cross = a.Intersection( b );
    BOOST_REQUIRE( cross );
    BOOST_CHECK( *cross == POINT( -1, -1, 2 ) );
    BOOST_CHECK_EQUAL( a.SideOf( *cross ), 0 );
    BOOST_CHECK( !a.Intersection( a.Opposite() ) );
    BOOST_CHECK_THROW( SIMPLEX::Box( { 0, 0, 0, 1 } ), std::invalid_argument );
    BOOST_CHECK_THROW( SIMPLEX( { { { 0, 0 }, { 1, 0 } }, { { 0, 1 }, { 1, 1 } },
                                   { { 1, 0 }, { 0, 1 } } } ), std::invalid_argument );
}
BOOST_AUTO_TEST_CASE( SpringOverHonorsRecursionCancellationAndFixedObstacles )
{
    const auto path = POLYLINE::FromPoints( { { -100, 0 }, { 100, 0 } } );
    TRACE_SHOVER::OBSTACLE obstacle{ 1, { -10, -10, 10, 10 }, SIMPLEX::Box( { -12, -12, 12, 12 } ),
                                                             SIMPLEX::Box( { -13, -13, 13, 13 } ) };
    auto result = TRACE_SHOVER::SpringOverObstacles( path, { obstacle }, {}, 0 );
    BOOST_CHECK( !result.polyline );
    result = TRACE_SHOVER::SpringOverObstacles( path, { obstacle }, {}, 1 );
    BOOST_REQUIRE( result.polyline );
    BOOST_CHECK( result.changed );
    BOOST_CHECK_EQUAL( result.recursiveSteps, 2 ); // one wrap in each direction
    BOOST_CHECK( result.polyline->FirstCorner() == path.FirstCorner() );
    BOOST_CHECK( result.polyline->LastCorner() == path.LastCorner() );
    result = TRACE_SHOVER::SpringOverObstacles( path, { obstacle }, [] { return true; } );
    BOOST_CHECK( result.cancelled ); BOOST_CHECK( !result.polyline );
    obstacle.canSpringOver = false;
    BOOST_CHECK( !TRACE_SHOVER::SpringOverObstacles( path, { obstacle } ).polyline );
    obstacle.canSpringOver = true;
    auto inside = POLYLINE::FromPoints( { { -100, 0 }, { 0, 0 } } );
    BOOST_CHECK( !TRACE_SHOVER::SpringOverObstacles( inside, { obstacle } ).polyline );
    // Touch enters the obstacle query but not the convex interior. Both
    // semantics are needed, and neither can be substituted for the other.
    auto touch = POLYLINE::FromPoints( { { -20, -12 }, { 20, -12 } } );
    BOOST_CHECK( obstacle.checkShape.IntersectsSegment( touch, 1 ) );
    BOOST_CHECK( obstacle.checkShape.EntrancePoints( touch ).empty() );
}
BOOST_AUTO_TEST_CASE( PinnedSpringOverLoopCannotBeAcceptedAsACompleteConnection )
{
    const auto path = POLYLINE::FromPoints( { { -100, -10 }, { 90, -10 }, { 90, 40 }, { -100, 40 } } );
    const std::vector<ROUTER_BOX> boxes{ { -60, -11, -41, 3 }, { -20, -11, 5, 7 },
                                         { 20, 10, 29, 31 }, { 60, -15, 75, 6 } };
    std::vector<TRACE_SHOVER::OBSTACLE> obstacles;
    for( std::size_t i = boxes.size(); i > 0; --i )
    {
        auto b = boxes[i - 1];
        const auto inflate = [&]( int r )
        { return SIMPLEX::Box( { b.minX - r, b.minY - r, b.maxX + r, b.maxY + r } ); };
        obstacles.push_back( { i + 1, b, inflate( 1 ), inflate( 18 ) } );
    }
    auto result = TRACE_SHOVER::SpringOverObstacles( path, obstacles );
    BOOST_REQUIRE( result.polyline );
    BOOST_CHECK( result.polyline->FirstCorner() == POINT( 2, 25 ) );
    BOOST_CHECK( !result.polyline->HasSameEndpoints( path ) ); // actual adapter guard
}
BOOST_AUTO_TEST_SUITE_END()
