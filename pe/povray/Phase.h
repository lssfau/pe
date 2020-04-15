//=================================================================================================
/*!
 *  \file pe/povray/Phase.h
 *  \brief POV-Ray phase modifier
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

#ifndef _PE_POVRAY_PHASE_H_
#define _PE_POVRAY_PHASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <string>
#include <pe/povray/Modifier.h>
#include <pe/system/Precision.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A POV-Ray phase modifier.
 * \ingroup povray_modifier
 *
 * The Phase class represents the POV-Ray phase modifier. The phase modifier can be used to
 * offset a color map or certain normals in the range from 0 to 1. Phase values of 0 (the
 * default) and 1 result in no changes of the texture. All value in-between change the phase.
 * For instance, for a RadialPigment a phase modifier results in a rotation of the pigment
 * around the y-axis:
 *
 * \image html phase.png
 * \image latex phase.eps "POV-Ray phase modifier" width=400pt
 *
 * The example code below demonstrates the setup of the illustrated texture:

   \code
   using namespace pe::pov;

   // Definition of a color map
   ColorMap colormap( "[0.0 color Red][0.33 color Yellow][0.66 color Blue][1.0 color Red]" );

   // Texture setup
   Texture texture(
      RadialPigment(
         colormap,
         Frequency( 5 ),
         Phase( 0.5 )
      )
      Finish(
         Ambient( 0.1 ),
         Diffuse( 0.6 ),
         Phong( 1, 50 ),
         Reflection( 0.05 )
      )
   );
   \endcode

 * \b Note: The phase modifier is always applied before a frequency modifier regardless of
 * their order.
 */
class PE_PUBLIC Phase : public Modifier
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Phase( real phase );
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
   real phase_;  //!< The phase value \f$ (0..1] \f$.
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
/*!\name Phase operators */
//@{
std::ostream& operator<<( std::ostream& os, const Phase& phase );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
