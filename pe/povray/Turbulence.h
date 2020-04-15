//=================================================================================================
/*!
 *  \file pe/povray/Turbulence.h
 *  \brief POV-Ray turbulence modifier
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

#ifndef _PE_POVRAY_TURBULENCE_H_
#define _PE_POVRAY_TURBULENCE_H_


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
/*!\brief A POV-Ray turbulence modifier.
 * \ingroup povray_modifier
 *
 * The Turbulence class represents the POV-Ray turbulence modifier. The turbulence modifier can
 * be used to distort a specific pattern (for instance pigments or normals) to some extent. The
 * turbulance value has to be in the range 0 to 1 inclusive. A high value corresponds to lots
 * of turbulence, a low value means only a litte turbulence.\n
 * The effect of turbulence is created by a displacement of the original color pixels on the
 * surface of an object. Turbulence works by taking a number of semi-random steps and using the
 * original color at the destination point. The random number generator used to produce these
 * steps is deterministic, so rendering a scene file with turbulence will always result in the
 * same image.\n
 * The number of these steps and their size can be modified by several turbulence parameters.
 * The overall size of the semi-random steps is directly determined by the turbulence value
 * itself (high value means high distortion, low value means small distortion). However, every
 * subsequent step is smaller than the previous one. This behavior is controlled by the Omega
 * modifier: every step is omega times as long as the previous step. The default omega value
 * is 0.5. Higher values tend to make the turbulence more random, smaller values tend to smooth
 * the pattern. The choice for the directions of the steps is influenced by the Lambda modifier:
 * it controls the randomness of each step. A lambda value close to 1 causes the steps to be in
 * approximately the same direction. Higher lambda values increase the probability for different
 * directions. The default for the lambda parameter is 2. The total number of semi-random steps
 * is specified by the Octaves modifier. The default value for octaves is 6. Due to the
 * exponential decrease of the step size due to the omega value, a larger number of steps has
 * only marginal effect, whereas a smaller number of step may create interesting effects.\n
 * The following images give an impression of the turbulence modifier. The first image shows
 * a marble pigment without any turbulence. The second image uses a turbulence modifier of 0.6
 * with no further modifications, whereas the third image shows the effect of turbulance with
 * an octave value of 2 and an omega value of 0.4.
 *
 * \image html turbulence.png
 * \image latex turbulence.eps "POV-Ray turbulence modifier" width=600pt
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
         Octaves( 2 ),
         Omega( 0.4 )
      )
      Finish(
         Ambient( 0.1 ),
         Diffuse( 0.6 ),
         Phong( 1, 50 ),
         Reflection( 0.05 )
      )
   );
   \endcode
 */
class PE_PUBLIC Turbulence : public Modifier
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Turbulence( real turbulence );
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
   real turbulence_;      //!< The turbulence value \f$ [0..1] \f$.
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
/*!\name Turbulence operators */
//@{
std::ostream& operator<<( std::ostream& os, const Turbulence& turbulence );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
