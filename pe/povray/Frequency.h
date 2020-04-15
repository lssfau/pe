//=================================================================================================
/*!
 *  \file pe/povray/Frequency.h
 *  \brief POV-Ray frequency modifier
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

#ifndef _PE_POVRAY_FREQUENCY_H_
#define _PE_POVRAY_FREQUENCY_H_


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
/*!\brief A POV-Ray frequency modifier.
 * \ingroup povray_modifier
 *
 * The Frequency class represents the POV-Ray frequency modifier. The frequency modifier controls
 * how many times a specific pattern (for instance a color map or ripples) is used over the range
 * from 0 to 1. A value greater than 1 will compress the pattern, a value less than 1 will stretch
 * it. Negative values will reverse the pattern. However, a frequency value of 0 is invalid. The
 * following sample images demonstrate the effect of the frequency modifier. The first image uses
 * the default frequency value of 1, the second uses a frequency value of 5. The third image was
 * rendered using a value of -2, therefore reversing the color scheme.
 *
 * \image html frequency.png
 * \image latex frequency.eps "POV-Ray frequency modifier" width=600pt
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
         Frequency( 5 )
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
class PE_PUBLIC Frequency : public Modifier
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Frequency( real frequency );
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
   real frequency_;  //!< The frequency value \f$ (0..1] \f$.
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
/*!\name Frequency operators */
//@{
std::ostream& operator<<( std::ostream& os, const Frequency& frequency );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
