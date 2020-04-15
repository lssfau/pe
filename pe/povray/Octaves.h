//=================================================================================================
/*!
 *  \file pe/povray/Octaves.h
 *  \brief POV-Ray octaves modifier
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

#ifndef _PE_POVRAY_OCTAVES_H_
#define _PE_POVRAY_OCTAVES_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/povray/Modifier.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A POV-Ray octaves modifier.
 * \ingroup povray_modifier
 *
 * The Octaves class represents the POV-Ray octaves modifier for Turbulence. It controls the
 * number of semi-random steps taken by the turbulence function when it is generating turbulence.
 * The octaves value can be any integral number between 1 and 10 (the default is 6). Due to the
 * exponential decrease of the step size due to the Omega value, a larger number of steps has
 * only marginal effect, whereas a smaller number of steps creates a smoother, wavy kind of
 * turbulence.\n
 * The images demonstrate the effect of the octaves modifier on turbulence. Image one shows
 * turbulence with only 2 steps. Image two uses 4 random steps, whereas image three demonstrates
 * the POV-Ray default of 6 steps. The more steps are used during the turbulence calculations,
 * the more caotic the turbulence effect becomes.
 *
 * \image html octaves.png
 * \image latex octaves.eps "POV-Ray octaves modifier" width=600pt
 *
 * The example code below demonstrates the setup of the illustrated texture:

   \code
   using namespace pe::pov;

   // Definition of a color map
   ColorMap colormap( "[0.0 color Red][0.33 color Yellow][0.66 color Blue][1.0 color Red]" );

   // Texture setup
   Texture texture(
      MarblePigment(
         colormap,
         Turbulence( 0.6 ),
         Octaves( 4 )
      )
      Finish(
         Ambient( 0.1 ),
         Diffuse( 0.6 ),
         Phong( 1, 50 ),
         Reflection( 0.05 )
      )
   );
   \endcode

 * \b Note: The Octaves modifier has only an effect if it is used in combination with the
 * Turbulence modifier.
 */
class PE_PUBLIC Octaves : public Modifier
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Octaves( unsigned int octaves );
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
   unsigned int octaves_;  //!< The octaves value \f$ [1..10] \f$.
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
/*!\name Octaves operators */
//@{
std::ostream& operator<<( std::ostream& os, const Octaves& octaves );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
