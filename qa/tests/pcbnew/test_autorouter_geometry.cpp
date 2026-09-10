/* KiCad, GPL-3.0-or-later. Exact geometry / recursive spring-over parity. */
#include "autorouter_geometry_oracle.h"
#include <autorouter/geometry/planar/PolylineArea.h>
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

BOOST_AUTO_TEST_CASE( SimplexNormalizationAndIntersectionMatchPinnedJava )
{
    const auto path = KI_TEST::GetPcbnewTestDataDir()
                      + "/autorouter/simplex-search-a11c0a42.txt";
    std::ifstream input( path );
    BOOST_REQUIRE( input.good() );
    std::string line;
    int count = 0;
    while( std::getline( input, line ) )
    {
        ++count;
        BOOST_TEST_CONTEXT( "simplex record " << count )
        {
            const std::string mismatch = AUTOROUTER_GEOMETRY_QA::CheckRecord( line );
            BOOST_CHECK_MESSAGE( mismatch.empty(), mismatch );
        }
    }
    BOOST_CHECK_EQUAL( count, 1216 );
}

BOOST_AUTO_TEST_CASE( PolylineOperationsMatchPinnedJava )
{
    const auto path = KI_TEST::GetPcbnewTestDataDir()
                      + "/autorouter/polyline-search-a11c0a42.txt";
    std::ifstream input( path );
    BOOST_REQUIRE( input.good() );
    std::string line;
    int count = 0;
    while( std::getline( input, line ) )
    {
        ++count;
        BOOST_TEST_CONTEXT( "polyline record " << count )
        {
            try
            {
                const std::string mismatch = AUTOROUTER_GEOMETRY_QA::CheckRecord( line );
                BOOST_CHECK_MESSAGE( mismatch.empty(), mismatch );
            }
            catch( const std::exception& error )
            {
                BOOST_ERROR( "oracle evaluation failed: " << error.what() );
            }
        }
    }
    BOOST_CHECK_EQUAL( count, 384 );
}

BOOST_AUTO_TEST_CASE( LineSegmentOperationsMatchPinnedJava )
{
    const auto path = KI_TEST::GetPcbnewTestDataDir()
                      + "/autorouter/segment-search-a11c0a42.txt";
    std::ifstream input( path );
    BOOST_REQUIRE( input.good() );
    std::string line;
    int count = 0;
    while( std::getline( input, line ) )
    {
        ++count;
        BOOST_TEST_CONTEXT( "line segment record " << count )
        {
            try
            {
                const std::string mismatch = AUTOROUTER_GEOMETRY_QA::CheckRecord( line );
                BOOST_CHECK_MESSAGE( mismatch.empty(), mismatch );
            }
            catch( const std::exception& error )
            {
                BOOST_ERROR( "oracle evaluation failed: " << error.what() );
            }
        }
    }
    BOOST_CHECK_EQUAL( count, 384 );
}

BOOST_AUTO_TEST_CASE( PolylineTransformsAndProjectionMatchPinnedJava )
{
    const auto path = KI_TEST::GetPcbnewTestDataDir()
                      + "/autorouter/polytransform-search-a11c0a42.txt";
    std::ifstream input( path );
    BOOST_REQUIRE( input.good() );
    std::string line;
    int count = 0;
    while( std::getline( input, line ) )
    {
        ++count;
        BOOST_TEST_CONTEXT( "polyline transform record " << count )
        {
            try
            {
                const std::string mismatch = AUTOROUTER_GEOMETRY_QA::CheckRecord( line );
                BOOST_CHECK_MESSAGE( mismatch.empty(), mismatch );
            }
            catch( const std::exception& error )
            {
                BOOST_ERROR( "oracle evaluation failed: " << error.what() );
            }
        }
    }
    BOOST_CHECK_EQUAL( count, 384 );
}

BOOST_AUTO_TEST_CASE( FloatPointAndLineOperationsMatchPinnedJava )
{
    const auto path = KI_TEST::GetPcbnewTestDataDir()
                      + "/autorouter/float-search-a11c0a42.txt";
    std::ifstream input( path );
    BOOST_REQUIRE( input.good() );
    std::string line;
    int count = 0;
    while( std::getline( input, line ) )
    {
        ++count;
        BOOST_TEST_CONTEXT( "floating geometry record " << count )
        {
            try
            {
                const std::string mismatch = AUTOROUTER_GEOMETRY_QA::CheckRecord( line );
                BOOST_CHECK_MESSAGE( mismatch.empty(), mismatch );
            }
            catch( const std::exception& error )
            {
                BOOST_ERROR( "oracle evaluation failed: " << error.what() );
            }
        }
    }
    BOOST_CHECK_EQUAL( count, 512 );
}

BOOST_AUTO_TEST_CASE( GeneralConvexPolylineAreaCutoutIsBoundedAndFailClosed )
{
    const SIMPLEX border = SIMPLEX::Box( { -100, -80, 100, 80 } );
    const auto hole = SIMPLEX::FromConvexPolygon(
            { { -25, -20 }, { 35, -11 }, { 8, 37 } } );
    BOOST_REQUIRE( hole );
    const auto pieces = POLYLINE_AREA::SplitSimplexToConvex(
            border, { *hole } );
    BOOST_REQUIRE( pieces );
    BOOST_CHECK( pieces->size() >= 3 );
    for( const SIMPLEX& piece : *pieces )
    {
        BOOST_CHECK_EQUAL( piece.Dimension(), 2 );
        BOOST_CHECK( piece.IsBounded() );
        BOOST_CHECK( !piece.ContainsInside( POINT( 0, 0 ) ) );
    }

    BOOST_CHECK( !POLYLINE_AREA::SplitSimplexToConvex(
            border, { *hole }, {}, 1 ) );
    BOOST_CHECK( !POLYLINE_AREA::SplitSimplexToConvex(
            border, { *hole }, [] { return true; } ) );
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
    // Rational support intersections are not rounded into a single arbitrary
    // grid point.  The router must enumerate the exact floor/ceil envelope
    // and prove each candidate legal against the real convex obstacle.
    const auto pBounds = p.SurroundingBox();
    BOOST_REQUIRE( pBounds );
    BOOST_CHECK_EQUAL( pBounds->minX, 3 );
    BOOST_CHECK_EQUAL( pBounds->maxX, 3 );
    BOOST_CHECK_EQUAL( pBounds->minY, 0 );
    BOOST_CHECK_EQUAL( pBounds->maxY, 1 );
    const auto negativeBounds = POINT( -1, -3, 2 ).SurroundingBox();
    BOOST_REQUIRE( negativeBounds );
    BOOST_CHECK_EQUAL( negativeBounds->minX, -1 );
    BOOST_CHECK_EQUAL( negativeBounds->maxX, 0 );
    BOOST_CHECK_EQUAL( negativeBounds->minY, -2 );
    BOOST_CHECK_EQUAL( negativeBounds->maxY, -1 );
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

BOOST_AUTO_TEST_CASE( ExpandedTraceSegmentKeepsDiagonalFreeSpaceWedges )
{
    // A swept diagonal trace is a convex hexagon under the worker's
    // Chebyshev clearance model.  Its axis-aligned bounding box would close
    // the upper-left and lower-right wedges, changing source TraceShover's
    // obstacle ordering and rejecting valid generated-trace moves.
    const auto sweep = SIMPLEX::FromExpandedSegment(
            { 2000000, 2000000 }, { 4000000, 4000000 }, 100000 );
    BOOST_REQUIRE( sweep );
    BOOST_CHECK( sweep->Contains( POINT( 3000000, 3000000 ) ) );
    BOOST_CHECK( !sweep->Contains( POINT( 1900000, 4100000 ) ) );
    BOOST_CHECK( !SIMPLEX::FromExpandedSegment(
                           { 2000000, 2000000 }, { 2000000, 2000000 }, 100000 ) );

    const auto cornerPass = POLYLINE::FromPoints(
            { { 1800000, 4000000 }, { 2000000, 4200000 } } );
    BOOST_REQUIRE( !cornerPass.Empty() );
    BOOST_CHECK( !sweep->IntersectsSegment( cornerPass, 1 ) );
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
