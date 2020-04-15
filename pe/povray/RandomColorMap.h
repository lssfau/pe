//=================================================================================================
/*!
 *  \file pe/povray/RandomColorMap.h
 *  \brief Implementation of a random color map for the POV-Ray visualization
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================

#ifndef _PE_POVRAY_RANDOMCOLORMAP_H_
#define _PE_POVRAY_RANDOMCOLORMAP_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/povray/ColorMap.h>
#include <pe/util/constraints/SameSize.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A random POV-Ray color map.
 * \ingroup povray_colormap
 *
 * The RandomColorMap class represents a random color map built from random colors without
 * transparency. See the ColorMap class description for more details about color maps in
 * general.
 */
class PE_PUBLIC RandomColorMap : public ColorMap
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructor */
   //@{
   explicit RandomColorMap();
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~RandomColorMap();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the RandomColorMap class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline RandomColorMap::~RandomColorMap()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( ColorMap, RandomColorMap );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
