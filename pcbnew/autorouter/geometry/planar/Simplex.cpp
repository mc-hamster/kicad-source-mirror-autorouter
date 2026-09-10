/* KiCad, GPL-3.0-or-later. Translated LineSegment.borderIntersections and
 * TileShape.entrancePoints/cutout(Polyline), Freerouting a11c0a42.
 */
#include "Simplex.h"
#include "IntOctagon.h"

#include <algorithm>
#include <array>
#include <limits>
#include <numeric>

namespace KICAD_AUTOROUTER::PLANAR
{
namespace
{

std::optional<std::int64_t> checkedOffset( std::int64_t aValue, std::int64_t aOffset )
{
    const INTEGER result = INTEGER( aValue ) + aOffset;
    if( result < std::numeric_limits<std::int64_t>::min()
        || result > std::numeric_limits<std::int64_t>::max() )
    {
        return {};
    }

    return result.convert_to<std::int64_t>();
}


int sign( const INTEGER& aValue )
{
    return aValue == 0 ? 0 : aValue < 0 ? -1 : 1;
}


bool isForwardCollinear( const ROUTER_POINT& aPrevious, const ROUTER_POINT& aCurrent,
                         const ROUTER_POINT& aNext )
{
    const INTEGER firstX = INTEGER( aCurrent.x ) - aPrevious.x;
    const INTEGER firstY = INTEGER( aCurrent.y ) - aPrevious.y;
    const INTEGER secondX = INTEGER( aNext.x ) - aCurrent.x;
    const INTEGER secondY = INTEGER( aNext.y ) - aCurrent.y;

    // Freerouting's Polygon constructor removes redundant collinear corners
    // before it constructs TileShape support lines.  Do the same only for a
    // forward continuation: a 180-degree reversal is malformed polygon
    // input, not a harmless support-line subdivision that may be erased.
    return firstX * secondY - firstY * secondX == 0
           && firstX * secondX + firstY * secondY > 0;
}


std::optional<LINE> chebyshevOffset( const LINE& aLine, std::int64_t aRadius )
{
    if( aRadius < 0 )
        return {};

    // For an oriented line whose inside is SideOf <= 0, a square radius r
    // moves the support outward by r * (|dx| + |dy|).  Translating either
    // endpoint by (r*sign(dy), -r*sign(dx)) produces exactly that support
    // constant without a floating normal, division, or rounded vertex.
    const INTEGER dx = aLine.Dx();
    const INTEGER dy = aLine.Dy();
    const INTEGER shiftX = INTEGER( aRadius ) * sign( dy );
    const INTEGER shiftY = -INTEGER( aRadius ) * sign( dx );
    if( shiftX < std::numeric_limits<std::int64_t>::min()
        || shiftX > std::numeric_limits<std::int64_t>::max()
        || shiftY < std::numeric_limits<std::int64_t>::min()
        || shiftY > std::numeric_limits<std::int64_t>::max() )
    {
        return {};
    }

    const auto ax = checkedOffset( aLine.a.x, shiftX.convert_to<std::int64_t>() );
    const auto ay = checkedOffset( aLine.a.y, shiftY.convert_to<std::int64_t>() );
    const auto bx = checkedOffset( aLine.b.x, shiftX.convert_to<std::int64_t>() );
    const auto by = checkedOffset( aLine.b.y, shiftY.convert_to<std::int64_t>() );
    if( !ax || !ay || !bx || !by )
        return {};

    return LINE( { *ax, *ay }, { *bx, *by } );
}


std::vector<LINE> normalizeBorders( std::vector<LINE> aBorders )
{
    if( aBorders.empty() )
        return {};

    // java.util.Arrays.sort(Object[]) is stable.  Equal directions therefore
    // retain input order, which matters when equal support lines are removed.
    std::stable_sort( aBorders.begin(), aBorders.end(), []( const LINE& aLeft,
                                                            const LINE& aRight )
    {
        return aLeft.CompareDirection( aRight ) < 0;
    } );

    std::vector<LINE> lines;
    lines.reserve( aBorders.size() );
    for( const LINE& border : aBorders )
    {
        if( lines.empty() || !border.SameDirectedSupport( lines.back() ) )
            lines.push_back( border );
    }

    // Direct control-flow equivalent of Simplex.removeRedundantLines.  The
    // source caches side-of-intersection values for speed; recomputing those
    // exact values after each erase is simpler and avoids stale-index state
    // while producing the same ordered fixed point.
    bool removed = lines.size() > 2;
    while( removed && lines.size() > 2 )
    {
        removed = false;

        for( std::size_t index = 0; index < lines.size(); ++index )
        {
            const std::size_t previousIndex =
                    ( index + lines.size() - 1 ) % lines.size();
            const std::size_t nextIndex = ( index + 1 ) % lines.size();
            const LINE previous = lines[previousIndex];
            const LINE current = lines[index];
            const LINE next = lines[nextIndex];
            const INTEGER determinant = previous.DirectionDeterminant( next );

            if( determinant != 0 )
            {
                const auto intersection = previous.Intersection( next );
                if( !intersection )
                    return {};
                const int intersectionSide = current.SideOf( *intersection );

                if( determinant > 0 )
                {
                    // Native +1 is source ON_THE_LEFT.
                    if( intersectionSide != 1 )
                    {
                        lines.erase( lines.begin() + index );
                        removed = true;
                        break;
                    }
                }
                else if( intersectionSide == 1
                         && previous.DirectionDeterminant( current ) > 0 )
                {
                    // The current half-plane cannot intersect the wedge made
                    // by its neighbours.
                    return {};
                }
            }
            else if( previous.SideOf( POINT( next.a ) ) == 1 )
            {
                // Opposing parallel supports face away from one another.
                return {};
            }
        }
    }

    if( lines.size() == 2 && lines[0].Parallel( lines[1] ) )
    {
        if( lines[0].SameDirection( lines[1] ) )
        {
            // Retain the more restrictive of two same-direction half-planes.
            if( lines[1].SideOf( POINT( lines[0].a ) ) == 1 )
                lines[0] = lines[1];
            lines.erase( lines.begin() + 1, lines.end() );
        }
        else if( lines[1].SideOf( POINT( lines[0].a ) ) == 1 )
        {
            return {};
        }
    }

    return lines;
}


std::optional<ROUTER_POINT> translatedPoint( ROUTER_POINT aPoint,
                                              ROUTER_POINT aVector )
{
    const auto x = checkedOffset( aPoint.x, aVector.x );
    const auto y = checkedOffset( aPoint.y, aVector.y );
    if( !x || !y )
        return {};
    return ROUTER_POINT{ *x, *y };
}


struct DIVISION_LINE
{
    ROUTER_POINT a;
    ROUTER_POINT b;

    bool Degenerate() const { return a == b; }
    INTEGER Dx() const { return INTEGER( b.x ) - a.x; }
    INTEGER Dy() const { return INTEGER( b.y ) - a.y; }
    INTEGER Determinant( const DIVISION_LINE& aOther ) const
    { return Dx() * aOther.Dy() - Dy() * aOther.Dx(); }
    INTEGER ScalarProduct( const DIVISION_LINE& aOther ) const
    { return Dx() * aOther.Dx() + Dy() * aOther.Dy(); }
    DIVISION_LINE Opposite() const { return { b, a }; }
    std::optional<LINE> Support() const
    {
        if( Degenerate() )
            return {};
        return LINE( a, b );
    }
};


DIVISION_LINE divisionLine( const LINE& aLine )
{
    return { aLine.a, aLine.b };
}


std::optional<ROUTER_POINT> directionEndpoint( ROUTER_POINT aStart,
                                               const INTEGER& aDx,
                                               const INTEGER& aDy )
{
    if( aDx == 0 && aDy == 0 )
        return aStart;

    const INTEGER divisor = Gcd( aDx, aDy );
    const INTEGER dx = aDx / divisor;
    const INTEGER dy = aDy / divisor;
    if( dx < std::numeric_limits<std::int64_t>::min()
        || dx > std::numeric_limits<std::int64_t>::max()
        || dy < std::numeric_limits<std::int64_t>::min()
        || dy > std::numeric_limits<std::int64_t>::max() )
    {
        return {};
    }

    return translatedPoint( aStart, { dx.convert_to<std::int64_t>(),
                                      dy.convert_to<std::int64_t>() } );
}


std::optional<DIVISION_LINE> perpendicularDirectionLine( ROUTER_POINT aPoint,
                                                          const LINE& aLine )
{
    const int side = aLine.SideOf( POINT( aPoint ) );
    if( side == 0 )
        return DIVISION_LINE{ aPoint, aPoint };

    // Point.perpendicularDirection(Line): source Point-side RIGHT chooses
    // +90 degrees and LEFT chooses -90.  Native Line::SideOf has the inverse
    // sign convention, so +1 selects (-dy, dx).
    const INTEGER dx = side > 0 ? -aLine.Dy() : aLine.Dy();
    const INTEGER dy = side > 0 ? aLine.Dx() : -aLine.Dx();
    const auto endpoint = directionEndpoint( aPoint, dx, dy );
    if( !endpoint )
        return {};
    return DIVISION_LINE{ aPoint, *endpoint };
}


double distanceToSupport( ROUTER_POINT aPoint, const LINE& aLine )
{
    const double dx = aLine.Dx().convert_to<double>();
    const double dy = aLine.Dy().convert_to<double>();
    const double px = static_cast<double>( aPoint.x ) - aLine.a.x;
    const double py = static_cast<double>( aPoint.y ) - aLine.a.y;
    return std::abs( dy * px - dx * py ) / std::sqrt( dx * dx + dy * dy );
}


std::optional<std::vector<DIVISION_LINE>> calculateDivisionLines(
        const SIMPLEX& aInner, std::size_t aCornerIndex,
        const SIMPLEX& aOuter )
{
    const auto& innerBorders = aInner.Borders();
    const LINE& currentInner = innerBorders[aCornerIndex];
    const LINE& previousInner =
            innerBorders[( aCornerIndex + innerBorders.size() - 1 )
                         % innerBorders.size()];
    const auto exactCorner = currentInner.Intersection( previousInner );
    if( !exactCorner )
        return {};

    const auto innerCorner = exactCorner->Integral();
    if( !innerCorner )
    {
        // A non-integral corner was introduced by clipping against the outer
        // simplex and already lies on its border; the source does not divide
        // from it.
        return std::vector<DIVISION_LINE>{ divisionLine( previousInner ) };
    }

    const DIVISION_LINE previousDirection = divisionLine( previousInner ).Opposite();
    const DIVISION_LINE nextDirection = divisionLine( currentInner );

    std::optional<DIVISION_LINE> firstProjection;
    std::optional<DIVISION_LINE> secondProjection;
    double minimumDistance = std::numeric_limits<double>::max();
    const auto& outerBorders = aOuter.Borders();

    for( std::size_t outerIndex = 0; outerIndex < outerBorders.size(); ++outerIndex )
    {
        const auto projection = perpendicularDirectionLine( *innerCorner,
                                                             outerBorders[outerIndex] );
        if( !projection )
            return {};
        if( projection->Degenerate() )
            return std::vector<DIVISION_LINE>{ *projection };

        const bool visible = previousDirection.Determinant( *projection ) >= 0;
        if( !visible )
            continue;

        double currentDistance = distanceToSupport( *innerCorner,
                                                     outerBorders[outerIndex] );
        const bool secondNecessary = projection->Determinant( nextDirection ) < 0;
        DIVISION_LINE currentSecond = *projection;

        if( secondNecessary )
        {
            bool secondVisible = false;
            std::size_t secondIndex = outerIndex;
            std::size_t checked = 0;
            while( !secondVisible && checked++ <= outerBorders.size() )
            {
                secondIndex = ( secondIndex + 1 ) % outerBorders.size();
                const auto candidate = perpendicularDirectionLine(
                        *innerCorner, outerBorders[secondIndex] );
                if( !candidate )
                    return {};
                if( candidate->Degenerate() )
                    return std::vector<DIVISION_LINE>{ *candidate };

                currentSecond = *candidate;
                if( projection->Determinant( currentSecond ) < 0 )
                {
                    currentDistance = std::numeric_limits<double>::max();
                    break;
                }
                secondVisible = currentSecond.Determinant( nextDirection ) >= 0;
                if( secondVisible )
                {
                    currentDistance += distanceToSupport( *innerCorner,
                                                          outerBorders[secondIndex] );
                }
            }
            if( !secondVisible )
                currentDistance = std::numeric_limits<double>::max();
        }

        if( currentDistance < minimumDistance )
        {
            minimumDistance = currentDistance;
            firstProjection = *projection;
            secondProjection = currentSecond;
        }
    }

    if( !firstProjection || !secondProjection
        || minimumDistance == std::numeric_limits<double>::max() )
    {
        return {};
    }

    if( firstProjection->Determinant( *secondProjection ) == 0
        && firstProjection->ScalarProduct( *secondProjection ) > 0 )
    {
        return std::vector<DIVISION_LINE>{ *firstProjection };
    }
    return std::vector<DIVISION_LINE>{ *firstProjection, *secondProjection };
}


void appendSupport( std::vector<LINE>& aLines, const DIVISION_LINE& aDivision,
                    bool aOpposite = false )
{
    const DIVISION_LINE selected = aOpposite ? aDivision.Opposite() : aDivision;
    if( const auto support = selected.Support() )
        aLines.push_back( *support );
}


std::optional<LINE> translateSupport( const LINE& aLine, double aDistance )
{
    const INTEGER divisor = Gcd( aLine.Dx(), aLine.Dy() );
    if( divisor == 0 )
        return {};
    const INTEGER exactDx = aLine.Dx() / divisor;
    const INTEGER exactDy = aLine.Dy() / divisor;
    if( exactDx < std::numeric_limits<std::int64_t>::min()
        || exactDx > std::numeric_limits<std::int64_t>::max()
        || exactDy < std::numeric_limits<std::int64_t>::min()
        || exactDy > std::numeric_limits<std::int64_t>::max() )
    {
        return {};
    }

    const std::int64_t dx = exactDx.convert_to<std::int64_t>();
    const std::int64_t dy = exactDy.convert_to<std::int64_t>();
    const double dxSquared = static_cast<double>( dx ) * dx;
    const double dySquared = static_cast<double>( dy ) * dy;
    const double length = std::sqrt( dxSquared + dySquared );
    ROUTER_POINT shift{};

    // Java Math.round is floor(value + 0.5), including for negative ties.
    const auto javaRound = []( double aValue )
    {
        const double rounded = std::floor( aValue + 0.5 );
        if( rounded < static_cast<double>( std::numeric_limits<std::int64_t>::min() )
            || rounded > static_cast<double>( std::numeric_limits<std::int64_t>::max() ) )
        {
            return std::optional<std::int64_t>{};
        }
        return std::optional<std::int64_t>{ static_cast<std::int64_t>( rounded ) };
    };

    if( dxSquared <= dySquared )
    {
        const auto relativeX = javaRound( aDistance * length / dy );
        if( !relativeX || *relativeX == std::numeric_limits<std::int64_t>::min() )
            return {};
        shift.x = -*relativeX;
    }
    else
    {
        const auto relativeY = javaRound( aDistance * length / dx );
        if( !relativeY )
            return {};
        shift.y = *relativeY;
    }

    const auto newA = translatedPoint( aLine.a, shift );
    if( !newA )
        return {};
    const auto newB = translatedPoint( *newA, { dx, dy } );
    if( !newB )
        return {};
    return LINE( *newA, *newB );
}


std::optional<std::int64_t> floorRational( const INTEGER& aNumerator,
                                           const INTEGER& aDenominator )
{
    INTEGER result = aNumerator / aDenominator;
    if( aNumerator < 0 && aNumerator % aDenominator != 0 )
        --result;
    if( result < std::numeric_limits<std::int64_t>::min()
        || result > std::numeric_limits<std::int64_t>::max() )
    {
        return {};
    }
    return result.convert_to<std::int64_t>();
}


std::optional<std::int64_t> ceilRational( const INTEGER& aNumerator,
                                          const INTEGER& aDenominator )
{
    INTEGER result = aNumerator / aDenominator;
    if( aNumerator > 0 && aNumerator % aDenominator != 0 )
        ++result;
    if( result < std::numeric_limits<std::int64_t>::min()
        || result > std::numeric_limits<std::int64_t>::max() )
    {
        return {};
    }
    return result.convert_to<std::int64_t>();
}


std::optional<INT_OCTAGON> boundingOctagon( const SIMPLEX& aSimplex )
{
    if( aSimplex.IsEmpty() || !aSimplex.IsBounded() )
        return {};

    std::int64_t left = std::numeric_limits<std::int64_t>::max();
    std::int64_t bottom = std::numeric_limits<std::int64_t>::max();
    std::int64_t right = std::numeric_limits<std::int64_t>::min();
    std::int64_t top = std::numeric_limits<std::int64_t>::min();
    std::int64_t upperLeft = std::numeric_limits<std::int64_t>::max();
    std::int64_t lowerRight = std::numeric_limits<std::int64_t>::min();
    std::int64_t lowerLeft = std::numeric_limits<std::int64_t>::max();
    std::int64_t upperRight = std::numeric_limits<std::int64_t>::min();

    for( std::size_t index = 0; index < aSimplex.Borders().size(); ++index )
    {
        const POINT& corner = aSimplex.Corner( index );
        const auto lx = floorRational( corner.x, corner.z );
        const auto ly = floorRational( corner.y, corner.z );
        const auto rx = ceilRational( corner.x, corner.z );
        const auto uy = ceilRational( corner.y, corner.z );
        const auto ulx = floorRational( corner.x - corner.y, corner.z );
        const auto lrx = ceilRational( corner.x - corner.y, corner.z );
        const auto llx = floorRational( corner.x + corner.y, corner.z );
        const auto urx = ceilRational( corner.x + corner.y, corner.z );
        if( !lx || !ly || !rx || !uy || !ulx || !lrx || !llx || !urx )
            return {};
        left = std::min( left, *lx );
        bottom = std::min( bottom, *ly );
        right = std::max( right, *rx );
        top = std::max( top, *uy );
        upperLeft = std::min( upperLeft, *ulx );
        lowerRight = std::max( lowerRight, *lrx );
        lowerLeft = std::min( lowerLeft, *llx );
        upperRight = std::max( upperRight, *urx );
    }

    return INT_OCTAGON( left, bottom, right, top, upperLeft, lowerRight,
                        lowerLeft, upperRight );
}


std::pair<double, double> projectToLine( double aX, double aY,
                                         const LINE& aLine )
{
    const long double dx = aLine.Dx().convert_to<long double>();
    const long double dy = aLine.Dy().convert_to<long double>();
    const long double lengthSquared = dx * dx + dy * dy;
    const long double ratio =
            ( ( static_cast<long double>( aX ) - aLine.a.x ) * dx
              + ( static_cast<long double>( aY ) - aLine.a.y ) * dy )
            / lengthSquared;
    return { static_cast<double>( aLine.a.x + ratio * dx ),
             static_cast<double>( aLine.a.y + ratio * dy ) };
}


int sideOfApprox( const LINE& aLine, const std::pair<double, double>& aPoint )
{
    const long double value = aLine.Dy().convert_to<long double>()
                                      * ( aPoint.first - aLine.a.x )
                              - aLine.Dx().convert_to<long double>()
                                      * ( aPoint.second - aLine.a.y );
    return value > 0 ? 1 : value < 0 ? -1 : 0;
}

} // namespace


SIMPLEX::SIMPLEX( std::vector<LINE> borders ) : m_borders( std::move( borders ) )
{
    if( m_borders.size() < 3 ) throw std::invalid_argument( "bounded convex shape needs 3 borders" );
    for( std::size_t i = 0; i < m_borders.size(); ++i )
    {
        if( !CornerIsBounded( i ) )
            throw std::invalid_argument( "unbounded or unordered convex borders" );
        auto p = m_borders[( i + m_borders.size() - 1 ) % m_borders.size()]
                         .Intersection( m_borders[i] );
        if( !p ) throw std::invalid_argument( "parallel adjacent convex borders" );
        for( const auto& line : m_borders )
            if( line.SideOf( *p ) > 0 ) throw std::invalid_argument( "non-convex or unordered borders" );
        m_corners.push_back( *p );
    }
    for( std::size_t i = 0; i < m_corners.size(); ++i )
        if( *m_corners[i] == *m_corners[( i + 1 ) % m_corners.size()] )
            throw std::invalid_argument( "redundant convex border" );
}


SIMPLEX::SIMPLEX( std::vector<LINE> aBorders, UNCHECKED_TAG ) :
        m_borders( std::move( aBorders ) )
{
    calculateCorners();
}


SIMPLEX SIMPLEX::GetInstance( std::vector<LINE> aBorders )
{
    return SIMPLEX( normalizeBorders( std::move( aBorders ) ), UNCHECKED_TAG{} );
}


SIMPLEX SIMPLEX::Empty()
{
    return SIMPLEX( {}, UNCHECKED_TAG{} );
}


void SIMPLEX::calculateCorners()
{
    m_corners.assign( m_borders.size(), std::nullopt );
    for( std::size_t index = 0; index < m_borders.size(); ++index )
    {
        if( !CornerIsBounded( index ) )
            continue;

        m_corners[index] =
                m_borders[( index + m_borders.size() - 1 ) % m_borders.size()]
                        .Intersection( m_borders[index] );
    }
}


const POINT& SIMPLEX::Corner( std::size_t aIndex ) const
{
    if( aIndex >= m_corners.size() || !m_corners[aIndex] )
        throw std::domain_error( "simplex corner is unbounded" );
    return *m_corners[aIndex];
}


bool SIMPLEX::CornerIsBounded( std::size_t aIndex ) const
{
    if( m_borders.size() < 2 || aIndex >= m_borders.size() )
        return false;

    const std::size_t previous =
            ( aIndex + m_borders.size() - 1 ) % m_borders.size();
    return m_borders[previous].DirectionDeterminant( m_borders[aIndex] ) > 0;
}


bool SIMPLEX::IsBounded() const
{
    if( IsEmpty() )
        return true;
    if( m_borders.size() < 3 )
        return false;

    for( std::size_t index = 0; index < m_borders.size(); ++index )
    {
        if( !CornerIsBounded( index ) )
            return false;
    }
    return true;
}


int SIMPLEX::Dimension() const
{
    if( IsEmpty() )
        return -1;
    if( m_borders.size() > 4 || m_borders.size() == 1 )
        return 2;
    if( m_borders.size() == 2 )
        return m_borders[0].EqualOrOpposite( m_borders[1] ) ? 1 : 2;
    if( m_borders.size() == 3 )
    {
        if( m_borders[0].EqualOrOpposite( m_borders[1] )
            || m_borders[0].EqualOrOpposite( m_borders[2] )
            || m_borders[1].EqualOrOpposite( m_borders[2] ) )
        {
            return 1;
        }

        const auto intersection = m_borders[1].Intersection( m_borders[2] );
        if( !intersection )
            return -1;
        const int side = m_borders[0].SideOf( *intersection );
        if( side < 0 )
            return 2;
        if( side > 0 )
            return -1;
        return 0;
    }

    const bool collinear02 = m_borders[0].EqualOrOpposite( m_borders[2] );
    const bool collinear13 = m_borders[1].EqualOrOpposite( m_borders[3] );
    if( collinear02 && collinear13 )
        return 0;
    if( collinear02 || collinear13 )
        return 1;
    return 2;
}


bool SIMPLEX::IsIntBox() const
{
    for( std::size_t index = 0; index < m_borders.size(); ++index )
    {
        if( !m_borders[index].IsOrthogonal() || !CornerIsBounded( index ) )
            return false;
    }
    return true;
}


bool SIMPLEX::IsIntOctagon() const
{
    for( std::size_t index = 0; index < m_borders.size(); ++index )
    {
        if( !m_borders[index].IsMultipleOf45Degree() || !CornerIsBounded( index ) )
            return false;
    }
    return true;
}


std::optional<ROUTER_BOX> SIMPLEX::BoundingBox() const
{
    if( IsEmpty() )
        return ROUTER_BOX{ 1, 1, 0, 0 };
    if( !IsBounded() )
        return {};

    ROUTER_BOX result{};
    bool first = true;
    for( std::size_t index = 0; index < m_corners.size(); ++index )
    {
        const auto bounds = Corner( index ).SurroundingBox();
        if( !bounds )
            return {};
        if( first )
        {
            result = *bounds;
            first = false;
        }
        else
        {
            result.minX = std::min( result.minX, bounds->minX );
            result.minY = std::min( result.minY, bounds->minY );
            result.maxX = std::max( result.maxX, bounds->maxX );
            result.maxY = std::max( result.maxY, bounds->maxY );
        }
    }
    return result;
}


int SIMPLEX::BorderLineIndex( const LINE& aLine ) const
{
    for( std::size_t index = 0; index < m_borders.size(); ++index )
    {
        if( m_borders[index].SameDirectedSupport( aLine ) )
            return static_cast<int>( index );
    }
    return -1;
}


SIMPLEX SIMPLEX::RemoveBorderLine( std::size_t aIndex ) const
{
    if( aIndex >= m_borders.size() )
        return *this;
    std::vector<LINE> result = m_borders;
    result.erase( result.begin() + aIndex );
    return SIMPLEX( std::move( result ), UNCHECKED_TAG{} );
}


SIMPLEX SIMPLEX::Intersection( const SIMPLEX& aOther ) const
{
    if( IsEmpty() || aOther.IsEmpty() )
        return Empty();
    std::vector<LINE> result = m_borders;
    result.insert( result.end(), aOther.m_borders.begin(), aOther.m_borders.end() );
    return GetInstance( std::move( result ) );
}


bool SIMPLEX::Intersects( const SIMPLEX& aOther ) const
{
    return !Intersection( aOther ).IsEmpty();
}


std::optional<std::vector<SIMPLEX>> SIMPLEX::CutoutFrom(
        const SIMPLEX& aOuter ) const
{
    if( Dimension() < 2 )
        return {};

    const SIMPLEX inner = Intersection( aOuter );
    if( inner.Dimension() < 2 )
        return std::vector<SIMPLEX>{ aOuter };

    std::vector<std::vector<DIVISION_LINE>> divisionLines;
    divisionLines.reserve( inner.Borders().size() );
    for( std::size_t corner = 0; corner < inner.Borders().size(); ++corner )
    {
        const auto lines = calculateDivisionLines( inner, corner, aOuter );
        if( !lines )
        {
            // The pinned source fails closed to the uncut outer simplex when
            // it cannot construct a division line.
            return std::vector<SIMPLEX>{ aOuter };
        }
        divisionLines.push_back( *lines );
    }

    bool checkCrossFirstLine = false;
    const DIVISION_LINE firstDivision = divisionLines.front().front();
    std::vector<SIMPLEX> result;

    for( std::size_t corner = 0; corner < divisionLines.size(); ++corner )
    {
        const std::size_t nextCorner = ( corner + 1 ) % divisionLines.size();
        const DIVISION_LINE nextDivision = divisionLines[nextCorner].front();
        const auto& currentDivisions = divisionLines[corner];

        if( currentDivisions.size() == 2 )
        {
            const DIVISION_LINE& currentDirection = currentDivisions.front();
            bool mergeFirstDivision = false;
            if( !checkCrossFirstLine )
            {
                checkCrossFirstLine = corner > 0
                                      && currentDirection.Determinant(
                                                 firstDivision ) > 0;
            }
            if( checkCrossFirstLine
                && currentDivisions.back().Determinant( firstDivision ) < 0 )
            {
                mergeFirstDivision = true;
            }

            std::vector<LINE> pieceLines;
            appendSupport( pieceLines, currentDivisions.back(), true );
            appendSupport( pieceLines, currentDivisions.front() );
            if( mergeFirstDivision )
                appendSupport( pieceLines, firstDivision, true );
            result.push_back( SIMPLEX::GetInstance( std::move( pieceLines ) )
                                      .Intersection( aOuter ) );
        }

        const DIVISION_LINE& lastCurrent = currentDivisions.back();
        bool mergeFirstDivision = false;
        if( !checkCrossFirstLine )
        {
            checkCrossFirstLine = corner > 0
                                  && lastCurrent.Determinant( firstDivision ) > 0
                                  && lastCurrent.ScalarProduct( firstDivision ) < 0;
        }
        if( checkCrossFirstLine
            && nextDivision.Determinant( firstDivision ) < 0 )
        {
            mergeFirstDivision = true;
        }

        std::vector<LINE> pieceLines;
        pieceLines.push_back( inner.Borders()[corner].Opposite() );
        appendSupport( pieceLines, nextDivision, true );
        appendSupport( pieceLines, lastCurrent );
        if( mergeFirstDivision )
            appendSupport( pieceLines, firstDivision, true );
        result.push_back( SIMPLEX::GetInstance( std::move( pieceLines ) )
                                  .Intersection( aOuter ) );

        // Preserve pinned a11c0a42 control flow exactly.  The Java source
        // assigns nextDivisionLine = prevDivisionLine here rather than the
        // apparent inverse, so prevDivisionLine remains null and contributes
        // no extra support to later pieces.
    }

    return result;
}


std::optional<SIMPLEX> SIMPLEX::TranslateBy( ROUTER_POINT aVector ) const
{
    if( aVector.x == 0 && aVector.y == 0 )
        return *this;
    std::vector<LINE> result;
    result.reserve( m_borders.size() );
    for( const LINE& border : m_borders )
    {
        const auto a = translatedPoint( border.a, aVector );
        const auto b = translatedPoint( border.b, aVector );
        if( !a || !b )
            return {};
        result.emplace_back( *a, *b );
    }
    return SIMPLEX( std::move( result ), UNCHECKED_TAG{} );
}


std::optional<SIMPLEX> SIMPLEX::Offset( double aWidth ) const
{
    if( aWidth == 0 )
        return *this;

    std::vector<LINE> result;
    result.reserve( m_borders.size() );
    for( const LINE& border : m_borders )
    {
        const auto translated = translateSupport( border, -aWidth );
        if( !translated )
            return {};
        result.push_back( *translated );
    }

    if( aWidth < 0 )
        return GetInstance( std::move( result ) );
    return SIMPLEX( std::move( result ), UNCHECKED_TAG{} );
}


std::optional<SIMPLEX> SIMPLEX::Enlarge( double aOffset ) const
{
    if( aOffset == 0 )
        return *this;
    const auto offsetSimplex = Offset( aOffset );
    const auto bounds = boundingOctagon( *this );
    if( !offsetSimplex || !bounds )
        return Empty();
    const auto offsetBounds = bounds->Offset( aOffset ).ToSimplex();
    if( !offsetBounds )
        return Empty();
    return offsetSimplex->Intersection( *offsetBounds );
}


std::pair<double, double> SIMPLEX::CentreOfGravity() const
{
    if( IsEmpty() || !IsBounded() )
        return { 0, 0 };
    double x = 0;
    double y = 0;
    for( std::size_t index = 0; index < m_corners.size(); ++index )
    {
        x += Corner( index ).X();
        y += Corner( index ).Y();
    }
    return { x / m_corners.size(), y / m_corners.size() };
}


std::pair<double, double> SIMPLEX::NearestBorderPointApprox(
        double aX, double aY ) const
{
    if( IsEmpty() )
        return { aX, aY };
    if( m_borders.size() == 1 )
        return projectToLine( aX, aY, m_borders.front() );
    if( Dimension() == 0 && CornerIsBounded( 0 ) )
        return { Corner( 0 ).X(), Corner( 0 ).Y() };

    std::pair<double, double> nearest{ aX, aY };
    long double minimum = std::numeric_limits<long double>::infinity();
    for( std::size_t index = 0; index < m_corners.size(); ++index )
    {
        if( !CornerIsBounded( index ) )
            continue;
        const double x = Corner( index ).X();
        const double y = Corner( index ).Y();
        const long double dx = static_cast<long double>( x ) - aX;
        const long double dy = static_cast<long double>( y ) - aY;
        const long double distance = dx * dx + dy * dy;
        if( distance < minimum )
        {
            minimum = distance;
            nearest = { x, y };
        }
    }

    std::size_t previous = m_borders.size() - 2;
    std::size_t current = m_borders.size() - 1;
    for( std::size_t next = 0; next < m_borders.size(); ++next )
    {
        const auto projection = projectToLine( aX, aY, m_borders[current] );
        if( ( !CornerIsBounded( current )
              || sideOfApprox( m_borders[previous], projection ) < 0 )
            && ( !CornerIsBounded( next )
                 || sideOfApprox( m_borders[next], projection ) < 0 ) )
        {
            const long double dx = static_cast<long double>( projection.first ) - aX;
            const long double dy = static_cast<long double>( projection.second ) - aY;
            const long double distance = dx * dx + dy * dy;
            if( distance < minimum )
            {
                minimum = distance;
                nearest = projection;
            }
        }
        previous = current;
        current = next;
    }
    return nearest;
}


std::pair<double, double> SIMPLEX::NearestPointApprox(
        double aX, double aY ) const
{
    if( !IsEmpty() )
    {
        bool contains = true;
        for( const LINE& border : m_borders )
        {
            if( sideOfApprox( border, { aX, aY } ) > 0 )
            {
                contains = false;
                break;
            }
        }
        if( contains )
            return { aX, aY };
    }
    return NearestBorderPointApprox( aX, aY );
}


int SIMPLEX::IndexOfRightMostCorner( const POINT& aFromPoint ) const
{
    if( !IsBounded() || IsEmpty() )
        return -1;

    int result = 0;
    const POINT* rightMost = &Corner( 0 );
    for( std::size_t index = 1; index < m_corners.size(); ++index )
    {
        const POINT& current = Corner( index );
        const INTEGER firstX = rightMost->x * aFromPoint.z - aFromPoint.x * rightMost->z;
        const INTEGER firstY = rightMost->y * aFromPoint.z - aFromPoint.y * rightMost->z;
        const INTEGER secondX = current.x * aFromPoint.z - aFromPoint.x * current.z;
        const INTEGER secondY = current.y * aFromPoint.z - aFromPoint.y * current.z;
        const INTEGER determinant = firstX * secondY - firstY * secondX;
        if( determinant < 0 )
        {
            rightMost = &current;
            result = static_cast<int>( index );
        }
    }
    return result;
}


SIMPLEX SIMPLEX::Box( ROUTER_BOX b )
{
    if( b.minX >= b.maxX || b.minY >= b.maxY ) throw std::invalid_argument( "empty convex box" );
    if( b.minX == std::numeric_limits<std::int64_t>::max()
        || b.minY == std::numeric_limits<std::int64_t>::min()
        || b.maxX == std::numeric_limits<std::int64_t>::min()
        || b.maxY == std::numeric_limits<std::int64_t>::max() )
    {
        throw std::overflow_error( "box support line endpoint overflow" );
    }
    // Preserve IntBox.toSimplex's exact anchors, not merely equivalent
    // supports.  Stable line identity participates in source tie ordering.
    return SIMPLEX( { { { b.minX, b.minY }, { b.minX + 1, b.minY } },
                      { { b.maxX, b.maxY }, { b.maxX, b.maxY + 1 } },
                      { { b.maxX, b.maxY }, { b.maxX - 1, b.maxY } },
                      { { b.minX, b.minY }, { b.minX, b.minY - 1 } } } );
}


std::optional<SIMPLEX> SIMPLEX::FromExpandedSegment(
        const ROUTER_POINT& aStart, const ROUTER_POINT& aEnd,
        std::int64_t aChebyshevRadius )
{
    if( aChebyshevRadius <= 0 || aStart == aEnd )
        return {};

    // The square Minkowski sweep of a segment is the convex hull of the
    // endpoint squares.  Construct that hull with exact integer cross
    // products rather than turning a diagonal trace into its enclosing
    // rectangle (which closes free-space wedges and changes shove order).
    std::vector<ROUTER_POINT> points;
    points.reserve( 8 );
    for( const ROUTER_POINT& endpoint : { aStart, aEnd } )
    {
        for( const std::int64_t offsetX : { -aChebyshevRadius, aChebyshevRadius } )
        {
            for( const std::int64_t offsetY : { -aChebyshevRadius, aChebyshevRadius } )
            {
                const auto x = checkedOffset( endpoint.x, offsetX );
                const auto y = checkedOffset( endpoint.y, offsetY );
                if( !x || !y )
                    return {};
                points.push_back( { *x, *y } );
            }
        }
    }

    std::sort( points.begin(), points.end(), []( const ROUTER_POINT& aLeft,
                                                 const ROUTER_POINT& aRight )
    {
        return aLeft.x != aRight.x ? aLeft.x < aRight.x : aLeft.y < aRight.y;
    } );
    points.erase( std::unique( points.begin(), points.end() ), points.end() );

    const auto cross = []( const ROUTER_POINT& aFirst, const ROUTER_POINT& aSecond,
                           const ROUTER_POINT& aThird )
    {
        return ( INTEGER( aSecond.x ) - aFirst.x ) * ( INTEGER( aThird.y ) - aFirst.y )
               - ( INTEGER( aSecond.y ) - aFirst.y ) * ( INTEGER( aThird.x ) - aFirst.x );
    };
    const auto appendHull = [&]( std::vector<ROUTER_POINT>& aHull,
                                 const ROUTER_POINT& aPoint )
    {
        while( aHull.size() >= 2
               && cross( aHull[aHull.size() - 2], aHull.back(), aPoint ) <= 0 )
        {
            aHull.pop_back();
        }
        aHull.push_back( aPoint );
    };

    std::vector<ROUTER_POINT> lower;
    std::vector<ROUTER_POINT> upper;
    lower.reserve( points.size() );
    upper.reserve( points.size() );
    for( const ROUTER_POINT& point : points )
        appendHull( lower, point );
    for( auto it = points.rbegin(); it != points.rend(); ++it )
        appendHull( upper, *it );

    if( lower.size() < 2 || upper.size() < 2 )
        return {};

    lower.pop_back();
    upper.pop_back();
    lower.insert( lower.end(), upper.begin(), upper.end() );
    return FromConvexPolygon( lower );
}


std::optional<SIMPLEX> SIMPLEX::FromConvexPolygon(
        const std::vector<ROUTER_POINT>& aPolygon, std::int64_t aChebyshevOffset )
{
    if( aChebyshevOffset < 0 || aPolygon.size() < 3 )
        return {};

    std::vector<ROUTER_POINT> points;
    points.reserve( aPolygon.size() );
    for( const ROUTER_POINT& point : aPolygon )
    {
        if( points.empty() || points.back() != point )
            points.push_back( point );
    }
    if( points.size() > 1 && points.front() == points.back() )
        points.pop_back();
    if( points.size() < 3 )
        return {};

    // KiCad polygonal shapes commonly retain collinear vertices after
    // tessellation or line-segment merging.  A convex support-line model has
    // no distinct border at such a point, and rejecting the whole shape
    // silently sent otherwise ordinary convex copper through the sampled
    // fallback.  Normalize the same harmless subdivisions that the source
    // Polygon constructor removes, then validate the remaining turns below.
    bool removedCorner = true;
    while( removedCorner && points.size() >= 3 )
    {
        removedCorner = false;
        std::vector<ROUTER_POINT> normalized;
        normalized.reserve( points.size() );
        for( std::size_t index = 0; index < points.size(); ++index )
        {
            const ROUTER_POINT& previous = points[( index + points.size() - 1 ) % points.size()];
            const ROUTER_POINT& current = points[index];
            const ROUTER_POINT& next = points[( index + 1 ) % points.size()];
            if( isForwardCollinear( previous, current, next ) )
            {
                removedCorner = true;
                continue;
            }
            normalized.push_back( current );
        }
        points = std::move( normalized );
    }
    if( points.size() < 3 )
        return {};

    int winding = 0;
    INTEGER centroidX = 0;
    INTEGER centroidY = 0;
    for( std::size_t index = 0; index < points.size(); ++index )
    {
        const ROUTER_POINT& previous = points[( index + points.size() - 1 ) % points.size()];
        const ROUTER_POINT& current = points[index];
        const ROUTER_POINT& next = points[( index + 1 ) % points.size()];
        const INTEGER firstX = INTEGER( current.x ) - previous.x;
        const INTEGER firstY = INTEGER( current.y ) - previous.y;
        const INTEGER secondX = INTEGER( next.x ) - current.x;
        const INTEGER secondY = INTEGER( next.y ) - current.y;
        const int cornerWinding = sign( firstX * secondY - firstY * secondX );
        if( cornerWinding == 0 || ( winding != 0 && winding != cornerWinding ) )
            return {};
        winding = cornerWinding;
        centroidX += current.x;
        centroidY += current.y;
    }

    const POINT centroid( centroidX, centroidY, points.size() );
    std::vector<LINE> borders;
    borders.reserve( points.size() );
    for( std::size_t index = 0; index < points.size(); ++index )
    {
        LINE border( points[index], points[( index + 1 ) % points.size()] );
        if( border.SideOf( centroid ) > 0 )
            border = border.Opposite();
        const auto expanded = chebyshevOffset( border, aChebyshevOffset );
        if( !expanded )
            return {};
        borders.push_back( *expanded );
    }

    try
    {
        return SIMPLEX( std::move( borders ) );
    }
    catch( const std::invalid_argument& )
    {
        return {};
    }
}


bool SIMPLEX::Contains( const POINT& p ) const
{
    return !IsEmpty()
           && std::all_of( m_borders.begin(), m_borders.end(),
                           [&]( const auto& l ) { return l.SideOf( p ) <= 0; } );
}
bool SIMPLEX::ContainsInside( const POINT& p ) const
{
    return !IsEmpty()
           && std::all_of( m_borders.begin(), m_borders.end(),
                           [&]( const auto& l ) { return l.SideOf( p ) < 0; } );
}

bool SIMPLEX::IntersectsSegment( const POLYLINE& polyline, std::size_t index ) const
{
    const auto a = polyline.Corner( index - 1 ), b = polyline.Corner( index );
    if( Contains( a ) || Contains( b ) ) return true;
    for( const auto& border : m_borders )
    {
        const auto p = polyline.lines.at( index ).Intersection( border );
        if( p && Contains( *p ) && p->CompareX( a ) * p->CompareX( b ) <= 0
            && p->CompareY( a ) * p->CompareY( b ) <= 0 ) return true;
    }
    return false;
}

std::vector<std::size_t> SIMPLEX::BorderIntersections( const POLYLINE& polyline, std::size_t index ) const
{
    const auto start = polyline.Corner( index - 1 ), end = polyline.Corner( index );
    const auto& middle = polyline.lines.at( index );
    std::vector<std::size_t> result;
    std::vector<POINT> intersections;
    const std::size_t n = m_borders.size();
    for( std::size_t i = 0; i < n; ++i )
    {
        const auto& current = m_borders[i];
        const int startSide = current.SideOf( start ), endSide = current.SideOf( end );
        if( startSide > 0 && endSide > 0 ) return {};
        if( startSide == 0 && endSide != -1 ) return {};
        if( endSide == 0 && startSide != -1 ) return {};
        if( startSide != -1 || endSide != -1 )
        {
            const auto p = middle.Intersection( current );
            if( !p ) return {};
            const int prevSide = m_borders[( i + n - 1 ) % n].SideOf( *p );
            const int nextSide = m_borders[( i + 1 ) % n].SideOf( *p );
            if( prevSide <= 0 && nextSide <= 0 )
            {
                if( prevSide == 0 )
                {
                    int a = middle.SideOf( Corner( ( i + n - 1 ) % n ) );
                    int b = middle.SideOf( Corner( ( i + 1 ) % n ) );
                    if( a == 0 || b == 0 || a == b ) return {};
                }
                if( nextSide == 0 )
                {
                    int a = middle.SideOf( Corner( i ) ), b = middle.SideOf( Corner( ( i + 2 ) % n ) );
                    if( a == 0 || b == 0 || a == b ) return {};
                }
                if( std::find( intersections.begin(), intersections.end(), *p ) == intersections.end() )
                {
                    if( result.size() == 2 ) throw std::logic_error( "more than two convex intersections" );
                    result.push_back( i ); intersections.push_back( *p );
                }
            }
        }
    }
    if( result.size() == 2 && start.DistanceSquared( intersections[1] ) < start.DistanceSquared( intersections[0] ) )
        std::swap( result[0], result[1] );
    return result;
}

std::vector<std::pair<std::size_t, std::size_t>> SIMPLEX::EntrancePoints( const POLYLINE& line ) const
{
    std::vector<std::pair<std::size_t, std::size_t>> result;
    for( std::size_t i = 1; i + 1 < line.lines.size(); ++i )
        for( auto side : BorderIntersections( line, i ) ) result.emplace_back( i, side );
    return result;
}

std::vector<POLYLINE> SIMPLEX::Cutout( const POLYLINE& polyline ) const
{
    if( polyline.Empty() ) return {};
    const auto intersections = EntrancePoints( polyline );
    const bool inside = ContainsInside( polyline.FirstCorner() );
    if( intersections.empty() ) return inside ? std::vector<POLYLINE>{} : std::vector<POLYLINE>{ polyline };
    std::vector<POLYLINE> result;
    const auto append = [&]( std::vector<LINE> lines )
    { POLYLINE piece( std::move( lines ) ); if( !piece.Empty() ) result.push_back( std::move( piece ) ); };
    std::size_t cursor = 0;
    auto [lineIndex, side] = intersections[0];
    const auto first = polyline.lines[lineIndex].Intersection( m_borders[side] ).value();
    if( !inside )
    {
        if( !( polyline.FirstCorner() == first ) )
        {
            std::vector<LINE> lines( polyline.lines.begin(), polyline.lines.begin() + lineIndex + 1 );
            lines.push_back( m_borders[side] ); append( std::move( lines ) );
        }
        ++cursor;
    }
    while( cursor + 1 < intersections.size() )
    {
        const auto [from, firstSide] = intersections[cursor];
        const auto [to, lastSide] = intersections[cursor + 1];
        bool insert = false;
        for( auto i = from + 1; i < to; ++i ) if( !Contains( polyline.Corner( i ) ) ) { insert = true; break; }
        if( insert )
        {
            std::vector<LINE> lines{ m_borders[firstSide] };
            lines.insert( lines.end(), polyline.lines.begin() + from, polyline.lines.begin() + to + 1 );
            lines.push_back( m_borders[lastSide] ); append( std::move( lines ) );
        }
        cursor += 2;
    }
    if( cursor < intersections.size() )
    {
        const auto [from, lastSide] = intersections[cursor];
        std::vector<LINE> lines{ m_borders[lastSide] };
        lines.insert( lines.end(), polyline.lines.begin() + from, polyline.lines.end() );
        append( std::move( lines ) );
    }
    return result;
}
} // namespace KICAD_AUTOROUTER::PLANAR
