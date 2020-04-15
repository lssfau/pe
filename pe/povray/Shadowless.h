//=================================================================================================
/*!
 *  \file pe/povray/Shadowless.h
 *  \brief Shadowless light source modifier
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

#ifndef _PE_POVRAY_SHADOWLESS_H_
#define _PE_POVRAY_SHADOWLESS_H_


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
/*!\brief Shadowless light source modifier.
 * \ingroup povray_lightsource
 *
 * The Shadowless class represents the \a shadowless keyword for POV-Ray light sources. By
 * default, all light sources are creating shadows. If the Shadowless light modifier is
 * used the light source is not casting any shadows. These lights are sometimes called
 * "fill lights". They are another way to simulate ambient light however shadowless lights
 * have a definite source. Note that shadowless lights will not cause highlights on the
 * illuminated rigid bodies.
 */
class PE_PUBLIC Shadowless : public LightItem
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Shadowless();
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
};
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Shadowless operators */
//@{
std::ostream& operator<<( std::ostream& os, const Shadowless& shadowless );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
