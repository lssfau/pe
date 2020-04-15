//=================================================================================================
/*!
 *  \file pe/povray/Omega.h
 *  \brief POV-Ray omega modifier
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

#ifndef _PE_POVRAY_OMEGA_H_
#define _PE_POVRAY_OMEGA_H_


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
/*!\brief A POV-Ray omega modifier.
 * \ingroup povray_modifier
 *
 * The Omega class represents the POV-Ray omega modifier for Turbulence. The omega modifier
 * controls the size of the semi-random steps during the creation of the turbulence effect:
 * every subsequent step is omega times as long as the previous step. The default omega value
 * is 0.5. Higher values tend to make the turbulence more random and chaotic, smaller values
 * tend to smooth the pattern.\n
 * The following images give an impression of the effects the omega modifier has on turbulence.
 * The first image shows a turbulence pattern with an omega value of 0.2. The second image uses
 * the POV-Ray default of 0.5, whereas the third image was rendered with omega equal to 0.8.
 *
 * \image html omega.png
 * \image latex omega.eps "POV-Ray omega modifier" width=600pt
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
         Omega( 0.5 )
      )
      Finish(
         Ambient( 0.1 ),
         Diffuse( 0.6 ),
         Phong( 1, 50 ),
         Reflection( 0.05 )
      )
   );
   \endcode

 * \b Note: The Omega modifier has only an effect if it is used in combination with the
 * Turbulence modifier.
 */
class PE_PUBLIC Omega : public Modifier
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Omega( real omega );
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
   real omega_;  //!< The omega value \f$ (0..1) \f$.
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
/*!\name Omega operators */
//@{
std::ostream& operator<<( std::ostream& os, const Omega& omega );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
