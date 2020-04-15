//=================================================================================================
/*!
 *  \file pe/povray/Falloff.h
 *  \brief Falloff light source modifier
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

#ifndef _PE_POVRAY_FALLOFF_H_
#define _PE_POVRAY_FALLOFF_H_


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
/*!\brief Falloff light source modifier.
 * \ingroup povray_lightsource
 *
 * The Falloff class represents the POV-Ray falloff light modifier. It specifies the overall
 * size of the cone of light of a focused light source as for example a spotlight. Inside
 * the light cone, the light intensity tapers off to zero, outside this light cone the light
 * has zero intensity. In combination with the Radius light modifier, the decrease of the
 * light intensity starts outside the bright, inner light cone specified by the Radius modifier
 * and reaches zero at the edge of the outside light cone specified by the Falloff modifier.
 * The falloff value is the angle (in radian measure) between the edge of the cone and center
 * line.\n
 * For a full explanation of the falloff modifier, see the POV-Ray documentation:\n
 *
 * http://www.povray.org/documentation/view/3.6.1/310/
 */
class PE_PUBLIC Falloff : public LightItem
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Falloff( real falloff );
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
   real falloff_;  //!< The falloff value \f$ [0..\frac{\pi}{2}] \f$.
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
/*!\name Falloff operators */
//@{
std::ostream& operator<<( std::ostream& os, const Falloff& falloff );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
