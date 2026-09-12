/*
 * This program source code file is part of KiCad, a free EDA application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include "../AutorouterTypes.h"
#include "../geometry/planar/FloatLine.h"


namespace KICAD_AUTOROUTER
{

class MAZE_SEARCH_ENGINE;
class EXPANSION_DOOR;
class OBSTACLE_EXPANSION_ROOM;


/** Direction retained while a shove crosses a two-dimensional trace-item
 * link door.  This is the native equivalent of MazeSearchElement.Adjustment;
 * preserving it prevents the next obstacle room from immediately attempting
 * the opposite shove direction.
 */
enum class MAZE_ADJUSTMENT
{
    NONE,
    LEFT,
    RIGHT
};


/** Immutable source PolylineTrace data needed by expansion-time shoving.
 *
 * One instance is shared by every compensated tree shape belonging to a
 * same-layer, same-style trace item.  The callback checks the exact live
 * worker board transactionally and returns the usable prefix length of the
 * proposed shove segment; infinity denotes the complete segment.
 */
struct MAZE_TRACE_ROOM_INFO
{
    std::vector<ROUTER_POINT> corners;
    std::size_t firstShapeIndex = 0;
    std::int64_t halfWidth = 0;
    std::int64_t clearance = 0;
    /** Exact PolylineTrace half-width after search-tree clearance
     * compensation.  MazeSearchEngine.roomShapeIsThick() compares this
     * value directly with the candidate's compensated half-width; deriving
     * it from an octagon's minimum width is not equivalent for diagonal
     * trace shapes. */
    std::int64_t compensatedHalfWidth = 0;
    bool sourceStyleMatches = false;
    std::function<double( const FLOAT_LINE&, bool )> maxShoveLength;
};


struct MAZE_SHOVE_DOOR_SECTION
{
    EXPANSION_DOOR* door = nullptr;
    std::size_t section = 0;
    FLOAT_LINE line;
};

/** Contact-preserving line-of-sight pull-tight used by the batch optimizer.
 * Movable-neighbour displacement is handled by the checked forced inserter;
 * this local operation only accepts strict, immediately insertable copper.
 */
class MAZE_TRACE_SHOVER
{
public:
    /** Translation of MazeTraceShover.checkShoveTraceLine().
     *
     * The boolean has the source's deliberately unusual meaning: false asks
     * the caller to delay paid rip-up so another outer door section may try;
     * true permits normal expansion to continue, whether or not zero-cost
     * shove doors were produced.
     */
    static bool CheckShoveTraceLine(
            EXPANSION_DOOR& aFromDoor, std::size_t aFromSection,
            const FLOAT_LINE& aShapeEntry, OBSTACLE_EXPANSION_ROOM& aObstacleRoom,
            double aCompensatedTraceHalfWidth, bool aShoveToTheLeft,
            std::vector<MAZE_SHOVE_DOOR_SECTION>& aToDoors,
            double aTraceWidthTolerance = 2 );

    static bool Shorten( ROUTING_CONNECTION& aConnection, const MAZE_SEARCH_ENGINE& aSearch );
};

} // namespace KICAD_AUTOROUTER
