/* KiCad, GPL-3.0-or-later. Recursive contour replacement and two-direction
 * selection translated from TraceShover.springOver[Obstacles], a11c0a42.
 * Board-specific item eligibility/clearance compensation is the caller's job.
 */
#include "TraceShover.h"
#include "../../geometry/planar/IntBox.h"

namespace KICAD_AUTOROUTER
{
namespace
{
using namespace PLANAR;
std::optional<POLYLINE> springOver( const POLYLINE& polyline,
                                    const std::vector<TRACE_SHOVER::OBSTACLE>& obstacles,
                                    int depth, const ROUTER_CANCEL_CALLBACK& cancel,
                                    TRACE_SHOVER::RESULT& status )
{
    const TRACE_SHOVER::OBSTACLE* found = nullptr;
    for( std::size_t i = 1; i + 1 < polyline.lines.size(); ++i )
    {
        for( const auto& obstacle : obstacles )
        {
            if( cancel && cancel() ) { status.cancelled = true; return {}; }
            const auto& shape = obstacle.checkShape;
            if( !shape.IntersectsSegment( polyline, i ) ) continue;
            if( !found ) found = &obstacle;
            else if( found->id != obstacle.id && INT_BOX::Intersects( found->bounds, obstacle.bounds ) )
            {
                if( INT_BOX::Contains( obstacle.bounds, found->bounds ) ) found = &obstacle;
                else if( !INT_BOX::Contains( found->bounds, obstacle.bounds ) ) return {};
            }
        }
        if( found ) break;
    }
    if( !found ) return polyline;
    if( depth <= 0 || !found->canSpringOver ) return {};
    const auto& shape = found->offsetShape;
    if( shape.ContainsInside( polyline.FirstCorner() ) || shape.ContainsInside( polyline.LastCorner() ) ) return {};
    const auto entries = shape.EntrancePoints( polyline );
    if( entries.empty() ) return polyline;
    if( entries.size() < 2 ) return {};
    const auto [firstLine, firstSide] = entries.front();
    const auto [lastLine, lastSide] = entries.back();
    const auto& borders = shape.Borders();
    const std::size_t n = borders.size();
    std::size_t sideDiff = ( lastSide + n - firstSide ) % n;
    if( sideDiff == 0 )
    {
        const auto first = polyline.lines[firstLine].Intersection( borders[firstSide] ).value();
        const auto second = polyline.lines[lastLine].Intersection( borders[lastSide] ).value();
        const auto& corner = shape.Corner( firstSide );
        if( corner.DistanceSquared( second ) < corner.DistanceSquared( first ) ) sideDiff += n;
    }
    std::vector<LINE> substitute{ polyline.lines[firstLine] };
    for( std::size_t i = 0; i <= sideDiff; ++i ) substitute.push_back( borders[( firstSide + i ) % n] );
    substitute.push_back( polyline.lines[lastLine] );
    POLYLINE result( std::move( substitute ) );
    const auto pieces = shape.Cutout( polyline );
    if( !pieces.empty() ) result = pieces[0].Combine( result );
    if( pieces.size() > 1 ) result = result.Combine( pieces[1] );
    if( result.Empty() ) return {};
    status.changed = true;
    ++status.recursiveSteps;
    return springOver( result, obstacles, depth - 1, cancel, status );
}
}
TRACE_SHOVER::RESULT TRACE_SHOVER::SpringOverObstacles( const PLANAR::POLYLINE& polyline,
        const std::vector<OBSTACLE>& obstacles, const ROUTER_CANCEL_CALLBACK& cancel, int depth )
{
    RESULT result;
    if( polyline.Empty() || depth < 0 || depth > 20 ) return result;
    if( cancel && cancel() ) { result.cancelled = true; return result; }
    auto ccw = springOver( polyline, obstacles, depth, cancel, result );
    if( result.cancelled ) return result;
    if( ccw && !result.changed ) { result.polyline = std::move( ccw ); return result; }
    auto cw = springOver( polyline.Reverse(), obstacles, depth, cancel, result );
    if( result.cancelled ) return result;
    if( cw && ( !ccw || cw->LengthApprox() <= ccw->LengthApprox() ) ) result.polyline = cw->Reverse();
    else result.polyline = std::move( ccw );
    return result;
}
} // namespace KICAD_AUTOROUTER
