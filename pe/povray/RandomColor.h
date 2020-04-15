//=================================================================================================
/*!
 *  \file pe/povray/RandomColor.h
 *  \brief Implementation of a random rgb color for the POV-Ray visualization
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

#ifndef _PE_POVRAY_RANDOMCOLOR_H_
#define _PE_POVRAY_RANDOMCOLOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/povray/Color.h>
#include <pe/system/Precision.h>
#include <pe/util/constraints/SameSize.h>
#include <pe/util/Random.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A random rgb color value.
 * \ingroup povray_color
 *
 * The RandomColor class represents a random rgb color value without transparency. See the Color
 * class description for more details about colors in general.
 */
class PE_PUBLIC RandomColor : public Color
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline RandomColor();
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~RandomColor();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for the RandomColor class.
 */
inline RandomColor::RandomColor()
   : Color( rand<real>(), rand<real>(), rand<real>() )  // Initialization of the base class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the RandomColor class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline RandomColor::~RandomColor()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Color, RandomColor );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
