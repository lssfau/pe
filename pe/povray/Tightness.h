//=================================================================================================
/*!
 *  \file pe/povray/Tightness.h
 *  \brief Tightness light source modifier
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

#ifndef _PE_POVRAY_TIGHTNESS_H_
#define _PE_POVRAY_TIGHTNESS_H_


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
/*!\brief Tightness light source modifier.
 * \ingroup povray_lightsource
 *
 * The Tightness class represents the POV-Ray tightness light modifier. This modifier can be
 * used to specify an additional exponential softening on the edges of the light cone. A value
 * other than 0 will affect light within the Radius cone as well as light in the Falloff cone.
 * The default value for tightness is 0. Low tightness values will make the spotlight brighter,
 * making spot wider and the edges sharper. High values will dim the spotlight, making the spot
 * tighter and the edges softer.\n
 * For a full explanation of the tightness modifier, see the POV-Ray documentation:\n
 *
 * http://www.povray.org/documentation/view/3.6.1/310/
 */
class PE_PUBLIC Tightness : public LightItem
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Tightness( real tightness );
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
   real tightness_;  //!< The tightness value \f$ [0..100] \f$.
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
/*!\name Tightness operators */
//@{
std::ostream& operator<<( std::ostream& os, const Tightness& tightness );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
