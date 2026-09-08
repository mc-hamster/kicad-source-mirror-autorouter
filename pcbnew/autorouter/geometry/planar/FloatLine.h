/* This file is part of KiCad, licensed under GPL version 3 or later. */
#pragma once
#include <cmath>
#include "IntBox.h"

namespace KICAD_AUTOROUTER
{
struct FLOAT_POINT
{
    double x = 0;
    double y = 0;
    double WeightedDistance( FLOAT_POINT aOther, double aHorizontal, double aVertical ) const
    {
        const double dx = ( x - aOther.x ) * aHorizontal;
        const double dy = ( y - aOther.y ) * aVertical;
        return std::sqrt( dx * dx + dy * dy );
    }
    ROUTER_POINT Round() const
    {
        // Java Math.round, including negative half-integers.
        return { static_cast<std::int64_t>( std::floor( x + 0.5 ) ),
                 static_cast<std::int64_t>( std::floor( y + 0.5 ) ) };
    }

    ROUTER_BOX BoundingBox() const
    {
        // FloatPoint.boundingBox uses floor/ceil, NOT Math.round. The clamp
        // matches Java's defined double -> int conversion (including NaN),
        // avoiding undefined C++ casts for out-of-domain diagnostic inputs.
        auto javaInt = []( double aValue ) -> std::int64_t
        {
            return std::isnan( aValue ) ? 0 : static_cast<std::int64_t>( std::clamp(
                    aValue, -2147483648.0, 2147483647.0 ) );
        };
        return { javaInt( std::floor( x ) ), javaInt( std::floor( y ) ),
                 javaInt( std::ceil( x ) ), javaInt( std::ceil( y ) ) };
    }
};

struct FLOAT_LINE
{
    FLOAT_POINT a;
    FLOAT_POINT b;
    FLOAT_POINT Middle() const { return { ( a.x + b.x ) / 2, ( a.y + b.y ) / 2 }; }
};
} // namespace KICAD_AUTOROUTER
