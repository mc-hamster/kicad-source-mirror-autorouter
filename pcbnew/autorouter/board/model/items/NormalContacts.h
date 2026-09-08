/* KiCad, GPL-3.0-or-later.
 * Endpoint/centre rules translated from Freerouting a11c0a42 Trace,
 * DrillItem and ConductionArea. Physical copper overlap is a DIFFERENT graph.
 */
#pragma once
#include "../../../AutorouterTypes.h"

namespace KICAD_AUTOROUTER
{
struct NORMAL_CONTACT_ITEM
{
    enum class KIND { UNKNOWN, TRACE, DRILL, AREA };
    KIND kind = KIND::UNKNOWN;
    std::vector<int> layers;
    ROUTER_POINT first, last;

    bool SharesLayer( const NORMAL_CONTACT_ITEM& other ) const
    {
        return std::any_of( layers.begin(), layers.end(), [&]( int layer )
        { return std::find( other.layers.begin(), other.layers.end(), layer ) != other.layers.end(); } );
    }

    /** Java normalContactPoint deliberately does not test nets, and returns
     * null for areas, no contact, or two distinct common trace endpoints.
     */
    std::optional<ROUTER_POINT> Point( const NORMAL_CONTACT_ITEM& other ) const
    {
        if( !SharesLayer( other ) || kind == KIND::UNKNOWN || other.kind == KIND::UNKNOWN
            || kind == KIND::AREA || other.kind == KIND::AREA )
            return {};
        if( kind == KIND::DRILL )
        {
            if( first == other.first || ( other.kind == KIND::TRACE && first == other.last ) )
                return first;
            return {};
        }
        if( other.kind == KIND::DRILL )
            return other.Point( *this );
        const bool atFirst = first == other.first || first == other.last;
        const bool atLast = last == other.first || last == other.last;
        if( atFirst == atLast )
            return {};
        return atFirst ? first : last;
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
            return contains( other.first )
                   || ( other.kind == KIND::TRACE && contains( other.last ) );
        }
        if( other.kind == KIND::AREA )
            return other.Touches( *this, contains );
        if( kind == KIND::TRACE && other.kind == KIND::TRACE )
            return first == other.first || first == other.last
                   || last == other.first || last == other.last;
        return Point( other ).has_value();
    }
};
} // namespace KICAD_AUTOROUTER
