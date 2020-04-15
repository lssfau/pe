//=================================================================================================
/*!
 *  \file pe/povray/Phong.h
 *  \brief Finish component for phong highlighting
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

#ifndef _PE_POVRAY_PHONG_H_
#define _PE_POVRAY_PHONG_H_


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
/*!\brief Finish component for phong highlighting.
 * \ingroup povray_finish
 *
 * The Phong class represents the POV-Ray finish component for phong highlights. Adding a phong
 * highlight to a finish creates a highlight on the rigid body that is the color of the light
 * source. The phong value specifies the saturation of the highlight and has to be in the range
 * \f$ [0..1] \f$. Additionally, the size of the phong highlight can be specified individually.
 * See the PhongSize class description for more details.\n
 * Phong highlights are similar to specular highlights. However, specular highlights are more
 * accurate as far as physical laws are concerned. Note that phong highlights and specular
 * highlights are usually not both used on the same body!\n
 * The following images illustrate the effect of the phong value. The first image shows a sphere
 * without a phong highlight, the second image demonstrates a phong highlight of strength 0.9.
 *
 * \image html phong.png
 * \image latex phong.eps "Examples for the phong highlight" width=400pt
 *
 * If no phong highlight is specified, the default POV-Ray value of 0 is used, which turns off
 * phong highlighting completely. In case a phong highlight is used and the phong size is not
 * specified, the default POV-Ray value of 40 is used.
 */
class PE_PUBLIC Phong : public FinishItem
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Phong( real phong );
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
   real phong_;  //!< The phong highlight value \f$ [0..1] \f$.
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
/*!\name Phong operators */
//@{
std::ostream& operator<<( std::ostream& os, const Phong& phong );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
