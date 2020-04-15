//=================================================================================================
/*!
 *  \file pe/povray/Reflection.h
 *  \brief Finish component for reflections
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

#ifndef _PE_POVRAY_REFLECTION_H_
#define _PE_POVRAY_REFLECTION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/povray/FinishItem.h>
#include <pe/system/Precision.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Finish component for reflections.
 * \ingroup povray_finish
 *
 * The Reflection class represents the POV-Ray finish component for reflections. The reflection
 * finish gives a rigid body a mirrored or partially mirrored surface. This body will then
 * reflect other bodies in the scene. The relection value has to be in the range \f$ [0..1] \f$.
 * A value of 0 turns the reflection off, a value of 1 gives the body an almost perfectly
 * mirrored surface. The images illustrate the reflection effect: the first image has no
 * reflection, whereas the second one uses a reflection of 0.3.
 *
 * \image html reflection.png
 * \image latex reflection.eps "Examples for reflection" width=400pt
 *
 * If no reflection is specified for a rigid body, the default POV-Ray value of 0 is used.
 */
class PE_PUBLIC Reflection : public FinishItem
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Reflection( real reflection );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os,                  bool newline ) const;
   void print( std::ostream& os, const char* tab, bool newline ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   real reflection_;  //!< Value of the reflection \f$ [0..1] \f$.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Reflection operators */
//@{
std::ostream& operator<<( std::ostream& os, const Reflection& reflection );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
