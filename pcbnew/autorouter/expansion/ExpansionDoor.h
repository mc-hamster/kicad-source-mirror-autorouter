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

#include "ExpandableObject.h"
#include "ExpansionRoom.h"
#include "../geometry/planar/FloatLine.h"


namespace KICAD_AUTOROUTER
{

/** Freerouting equivalent: autoroute/expansion/ExpansionDoor. */
class EXPANSION_DOOR : public EXPANDABLE_OBJECT
{
public:
    EXPANSION_DOOR( EXPANSION_ROOM* aFirstRoom, EXPANSION_ROOM* aSecondRoom,
                    int aDimension = -1, bool aRegisterWithRooms = true );

    EXPANSION_ROOM*       FirstRoom() const { return m_firstRoom; }
    EXPANSION_ROOM*       SecondRoom() const { return m_secondRoom; }
    EXPANSION_ROOM*       OtherRoom( EXPANSION_ROOM* aRoom ) const;
    bool                  Connects( const EXPANSION_ROOM* aRoom ) const;
    int                   GetDimension() const { return m_dimension; }
    virtual ROUTER_BOX          GetShape() const;
    virtual PLANAR::INT_OCTAGON GetOctagonShape() const;
    virtual PLANAR::SIMPLEX     GetSimplexShape() const;
    /** Angle-specific translations of MazeSearchEngine.doorIsSmall(). */
    bool                  IsSmallFor90DegreeTrace( double aTraceWidth ) const;
    bool                  IsSmallFor45DegreeTrace( double aTraceWidth ) const;
    bool                  IsSmallForAnyAngleTrace( double aTraceWidth ) const;
    // Tolerance is explicit because KiCad IU and reference coordinates differ.
    std::vector<FLOAT_LINE> GetSectionSegments( double aOffset, double aTolerance = 2,
                                               double aMaxSectionWidth = 0,
                                               std::size_t aMaxSections = std::numeric_limits<std::size_t>::max(),
                                               bool aReflectYToSource = false ) const;
    int                   GetId() const override;
    void                  Reset() override {}

private:
    EXPANSION_ROOM* m_firstRoom;
    EXPANSION_ROOM* m_secondRoom;
    int             m_dimension;
};

} // namespace KICAD_AUTOROUTER
