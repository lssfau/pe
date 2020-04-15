//=================================================================================================
/*!
 *  \file pe/povray/Roughness.h
 *  \brief Modifier for the specular highlighting finish component
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

#ifndef _PE_POVRAY_ROUGHNESS_H_
#define _PE_POVRAY_ROUGHNESS_H_


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
/*!\brief Modifier for the specular highlighting finish component
 * \ingroup povray_finish
 *
 * The Roughness class represents the POV-Ray \a roughness modifier for specular highlights. It
 * can be used to specify the size of a specular highlight. The roughness has to be a value in
 * the range \f$ (0..1] \f$. The next images illustrate the effect of the roughness parameter:
 * the first image has a roughness of 0.01, the second uses the POV-Ray default of 0.05 and the
 * third has a roughness of 0.1.
 *
 * \image html roughness.png
 * \image latex roughness.eps "Examples for the roughness" width=600pt
 *
 * In case a specular highlight is used and the roughness is not specified, the default POV-Ray
 * value of 0.05 is used.
 *
 * \b Note: The Roughness finish component has only an effect if it is used in combination with
 * the Specular component.
 */
class PE_PUBLIC Roughness : public FinishItem
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Roughness( real roughness );
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
   real roughness_;  //!< The roughness of the specular highlight \f$ (0..1] \f$.
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
/*!\name Roughness operators */
//@{
std::ostream& operator<<( std::ostream& os, const Roughness& roughness );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
