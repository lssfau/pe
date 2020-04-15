//=================================================================================================
/*!
 *  \file pe/povray/Adaptive.h
 *  \brief Adaptive light source modifier
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

#ifndef _PE_POVRAY_ADAPTIVE_H_
#define _PE_POVRAY_ADAPTIVE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/povray/LightItem.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adaptive light source modifier.
 * \ingroup povray_lightsource
 *
 * The Adaptive class represents the POV-Ray adaptive light modifier. It enables adaptive sampling
 * of a light source. By default POV-Ray calculates the amount of light that reaches a surface
 * from an area light by shooting a test ray at every point light within the array. This approach
 * may be very slow depending on the details of a POV-Ray scene. Adaptive sampling on the other
 * hand attempts to approximate the same calculation by using a minimum number of test rays. The
 * higher the number of test rays the more accurate the shadows will be but the longer they will
 * take to render.\n
 * For a full explanation of adaptive sampling, see the POV-Ray documentation:\n
 *
 * http://www.povray.org/documentation/view/3.6.1/313/
 */
class PE_PUBLIC Adaptive : public LightItem
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Adaptive( unsigned int adaptive );
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
   unsigned int adaptive_;  //!< The adaptive value \f$ [0..\infty) \f$.
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
/*!\name Adaptive operators */
//@{
std::ostream& operator<<( std::ostream& os, const Adaptive& adaptive );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
