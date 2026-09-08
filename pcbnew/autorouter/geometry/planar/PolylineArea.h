/*
 * KiCad, GPL-3.0-or-later. Freerouting a11c0a42 PolylineArea.splitToConvex:
 * rectangular border/holes subset. Ordering is part of the routing algorithm.
 */
#pragma once
#include "IntBox.h"

namespace KICAD_AUTOROUTER::POLYLINE_AREA
{
inline std::optional<std::vector<ROUTER_BOX>> SplitToConvex(
        ROUTER_BOX aBorder, const std::vector<ROUTER_BOX>& aHoles,
        const ROUTER_CANCEL_CALLBACK& aCancel = {},
        std::size_t aMaxPieces = std::numeric_limits<std::size_t>::max() )
{
    if( INT_BOX::Dimension( aBorder ) != 2 || aMaxPieces == 0 )
        return std::nullopt;
    std::vector<ROUTER_BOX> pieces{ aBorder };
    for( const auto& hole : aHoles )
    {
        if( aCancel && aCancel() )
            return std::nullopt;
        if( INT_BOX::Dimension( hole ) < 2 )
            continue;
        std::vector<ROUTER_BOX> next;
        for( const auto& piece : pieces )
        {
            if( aCancel && aCancel() )
                return std::nullopt;
            for( const auto& result : INT_BOX::Cutout( piece, hole ) )
                if( INT_BOX::Dimension( result ) == 2 )
                {
                    if( next.size() >= aMaxPieces )
                        return std::nullopt; // Failure, never a truncated free-space model.
                    next.push_back( result );
                }
        }
        pieces = std::move( next );
    }
    return aCancel && aCancel() ? std::nullopt
                              : std::optional<std::vector<ROUTER_BOX>>( std::move( pieces ) );
}
}
