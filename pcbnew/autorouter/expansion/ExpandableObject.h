/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#pragma once

namespace KICAD_AUTOROUTER
{

/**
 * Freerouting equivalent: autoroute/expansion/ExpandableObject.
 *
 * Expansion objects keep their search bookkeeping separate from the immutable
 * board snapshot.  The native implementation currently uses a lightweight
 * visibility graph, but retaining this seam makes the room/door search state
 * portable as upstream Freerouting's expansion algorithm evolves.
 */
class EXPANDABLE_OBJECT
{
public:
    virtual ~EXPANDABLE_OBJECT() = default;

    virtual void Reset() = 0;
    virtual int  GetId() const = 0;
};

} // namespace KICAD_AUTOROUTER
