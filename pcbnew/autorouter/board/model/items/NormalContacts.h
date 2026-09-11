/* KiCad, GPL-3.0-or-later.
 * Endpoint/centre rules translated from Freerouting a11c0a42 Trace,
 * DrillItem and ConductionArea. Physical copper overlap is a DIFFERENT graph.
 */
#pragma once
#include "../../../AutorouterTypes.h"
#include "../../../geometry/planar/Point.h"

namespace KICAD_AUTOROUTER
{
struct NORMAL_CONTACT_ITEM
{
    enum class KIND { UNKNOWN, TRACE, DRILL, AREA };
    KIND kind = KIND::UNKNOWN;
    std::vector<int> layers;
    ROUTER_POINT first, last;
    // Polyline endpoints can be RationalPoint values after source-equivalent
    // support-line splits. Integral host items leave these disengaged.
    std::optional<PLANAR::POINT> exactFirst, exactLast;

    PLANAR::POINT First() const
    {
        return exactFirst ? *exactFirst : PLANAR::POINT( first );
    }

    PLANAR::POINT Last() const
    {
        return exactLast ? *exactLast : PLANAR::POINT( last );
    }

    bool SharesLayer( const NORMAL_CONTACT_ITEM& other ) const
    {
        return std::any_of( layers.begin(), layers.end(), [&]( int layer )
        { return std::find( other.layers.begin(), other.layers.end(), layer ) != other.layers.end(); } );
    }

    /** Java normalContactPoint deliberately does not test nets, and returns
     * null for areas, no contact, or two distinct common trace endpoints.
     */
    std::optional<PLANAR::POINT> ExactPoint( const NORMAL_CONTACT_ITEM& other ) const
    {
        if( !SharesLayer( other ) || kind == KIND::UNKNOWN || other.kind == KIND::UNKNOWN
            || kind == KIND::AREA || other.kind == KIND::AREA )
            return {};
        if( kind == KIND::DRILL )
        {
            if( First() == other.First()
                || ( other.kind == KIND::TRACE && First() == other.Last() ) )
                return First();
            return {};
        }
        if( other.kind == KIND::DRILL )
            return other.ExactPoint( *this );
        const bool atFirst = First() == other.First() || First() == other.Last();
        const bool atLast = Last() == other.First() || Last() == other.Last();
        if( atFirst == atLast )
            return {};
        return atFirst ? First() : Last();
    }

    /** Integral KiCad adapter. A rational contact is deliberately not rounded. */
    std::optional<ROUTER_POINT> Point( const NORMAL_CONTACT_ITEM& other ) const
    {
        const auto exact = ExactPoint( other );
        return exact ? exact->Integral() : std::nullopt;
    }

    /** Caller enforces distinct IDs and a common net. Contains takes a layer
     * and point of an area, not a width-inflated copper collision test.
     */
    template<typename CONTAINS>
    bool Touches( const NORMAL_CONTACT_ITEM& other, const CONTAINS& contains ) const
    {
        if( !SharesLayer( other ) || kind == KIND::UNKNOWN || other.kind == KIND::UNKNOWN )
            return false;
        if( kind == KIND::AREA )
        {
            if( other.kind == KIND::AREA )
                return false;
            const auto otherFirst = other.First().Integral();
            const auto otherLast = other.Last().Integral();
            return ( otherFirst && contains( *otherFirst ) )
                   || ( other.kind == KIND::TRACE && otherLast && contains( *otherLast ) );
        }
        if( other.kind == KIND::AREA )
            return other.Touches( *this, contains );
        if( kind == KIND::TRACE && other.kind == KIND::TRACE )
            return First() == other.First() || First() == other.Last()
                   || Last() == other.First() || Last() == other.Last();
        return ExactPoint( other ).has_value();
    }
};
} // namespace KICAD_AUTOROUTER
