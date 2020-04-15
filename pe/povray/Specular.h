//=================================================================================================
/*!
 *  \file pe/povray/Specular.h
 *  \brief Finish component for specular highlighting
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

#ifndef _PE_POVRAY_SPECULAR_H_
#define _PE_POVRAY_SPECULAR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/povray/FinishItem.h>
#include <pe/system/Precision.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Finish component for specular highlights.
 * \ingroup povray_finish
 *
 * The Specular class represents the POV-Ray finish component for specular highlights. Adding
 * a specular highlight to a finish creates a highlight on the rigid body that is the color
 * of the light source. The specular value specifies the saturation of the highlight and has
 * to be in the range \f$ [0..1] \f$. Additionally, the size of the specular highlight can be
 * specified individually. See the Roughness class description for more details.\n
 * Specular highlights are similar to phong highlights. However, specular highlights are more
 * accurate as far as physical laws are concerned. Note that specular highlights and phong
 * highlights are usually not both used on the same body!\n
 * The following images illustrate the effect of the specular value. The first image shows a
 * sphere without a specular highlight, the second image demonstrates a specular highlight of
 * strength 0.6.
 *
 * \image html specular.png
 * \image latex specular.eps "Examples for the specular highlight" width=400pt
 *
 * If no specular highlight is specified, the default POV-Ray value of 0 is used, which turns off
 * specular highlighting completely. In case a specular highlight is used and the roughness is
 * not specified, the default POV-Ray value of 0.05 is used.
 */
class PE_PUBLIC Specular : public FinishItem
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Specular( real specular );
   explicit Specular( real specular, real roughness );
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
   real specular_;  //!< The value of the specular highlight \f$ [0..1] \f$.
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
/*!\name Specular operators */
//@{
std::ostream& operator<<( std::ostream& os, const Specular& specular );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
