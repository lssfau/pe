//=================================================================================================
/*!
 *  \file pe/povray/Scale.h
 *  \brief POV-Ray transformation for scaling operations
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

#ifndef _PE_POVRAY_SCALE_H_
#define _PE_POVRAY_SCALE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/povray/Transformation.h>
#include <pe/system/Precision.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A POV-Ray scale transformation.
 * \ingroup povray_transformation
 *
 * The Scale class is a POV-Ray transformation for scaling operations. The default value for
 * the size of a texture, pigment or normal is 1. In order to either stretch or squeeze them,
 * a Scale transformation can be used as demonstrated in the following example:

   \code
   // Stretching an image pigment by a factor of 20.
   pe::povray::Texture texture( pe::povray::ImagePigment(...), pe::povray::Scale( 20.0 ) );

   // Squeezing a bumps normal using a scaling factor of 0.1
   pe::povray::Bumps bumps( 2, pe::povray::Scale( 0.1 ) );
   \endcode

 * A scale value larger than 1 will stretch the texture, pigment or normal, a value smaller
 * than 1 will squeeze it. A negative value will reverse the texture, pigment or normal.
 * However, a scale value of 0 triggers a \a std::invalid_argument exception.
 */
class PE_PUBLIC Scale : public Transformation
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Scale( real scale );
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
   real scale_;  //!< The scale value \f$ (-\infty..0) \cup (0..\infty) \f$.
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
/*!\name Scale operators */
//@{
std::ostream& operator<<( std::ostream& os, const Scale& scale );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
