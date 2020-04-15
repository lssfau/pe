//=================================================================================================
/*!
 *  \file pe/povray/Radius.h
 *  \brief Radius light source modifier
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

#ifndef _PE_POVRAY_RADIUS_H_
#define _PE_POVRAY_RADIUS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/povray/LightItem.h>
#include <pe/system/Precision.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Radius light source modifier.
 * \ingroup povray_lightsource
 *
 * The Radius class represents the POV-Ray radius light modifier. It specifies the size of the
 * "hot-spot" at the center of a focused light cone (as for instance in case of a spotlight).
 * In case of a spotlight, the "hot-spot" is a brighter cone of light inside the spotlight cone
 * and has the same center line defined by the center of the light source and the focus point.
 * The radius value specifies the angle (in radian measure) between the edge of the bright,
 * inner cone and the center line. The light inside the inner cone is of uniform intensity,
 * whereas the light between the inner and outer cone tapers off to zero.\n
 * For a full explanation of the radius modifier, see the POV-Ray documentation:\n
 *
 * http://www.povray.org/documentation/view/3.6.1/310/
 */
class PE_PUBLIC Radius : public LightItem
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Radius( real radius );
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
   real radius_;  //!< The radius value \f$ [0..\frac{\pi}{2}] \f$.
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
/*!\name Radius operators */
//@{
std::ostream& operator<<( std::ostream& os, const Radius& radius );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
