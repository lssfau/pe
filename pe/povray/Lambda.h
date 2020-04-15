//=================================================================================================
/*!
 *  \file pe/povray/Lambda.h
 *  \brief POV-Ray lambda modifier
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

#ifndef _PE_POVRAY_LAMBDA_H_
#define _PE_POVRAY_LAMBDA_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
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
/*!\brief A POV-Ray lambda modifier.
 * \ingroup povray_modifier
 *
 * The Lambda class represents the POV-Ray lambda modifier for Turbulence. The lambda value
 * controls the choice of the random displacement directions that create the turbulence effect.
 * Lambda can be assigned any value larger than 1. A lambda value close to 1 causes the step
 * to be in approximately the same direction. Higher lambda values increase the probability for
 * different directions. The default for the lambda parameter is 2.\n
 * The following pictures give an impression of the effect of the lambda modifier. The first
 * image was rendered using a lambda value of 1.01, creating a very smooth turbulence. The
 * second image uses the default value of 2.0 and the third image a lambda value of 3.0.
 *
 * \image html lambda.png
 * \image latex lambda.eps "POV-Ray lambda modifier" width=600pt
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
         Lambda( 2.0 )
      )
      Finish(
         Ambient( 0.1 ),
         Diffuse( 0.6 ),
         Phong( 1, 50 ),
         Reflection( 0.05 )
      )
   );
   \endcode

 * \b Note: The Lambda modifier has only an effect if it is used in combination with the
 * Turbulence modifier.
 */
class PE_PUBLIC Lambda : public Modifier
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Lambda( real lambda );
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
   real lambda_;  //!< The lambda value \f$ (1..\infty) \f$.
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
/*!\name Lambda operators */
//@{
std::ostream& operator<<( std::ostream& os, const Lambda& lambda );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
