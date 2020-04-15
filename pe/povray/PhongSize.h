//=================================================================================================
/*!
 *  \file pe/povray/PhongSize.h
 *  \brief Modifier for the phong highlighting finish component
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

#ifndef _PE_POVRAY_PHONGSIZE_H_
#define _PE_POVRAY_PHONGSIZE_H_


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
/*!\brief Modifier for the phong highlighting finish component
 * \ingroup povray_finish
 *
 * The PhongSize class represents the POV-Ray \a phong_size modifier for phong highlights. It can
 * be used to specify the size of a phong highlight. The phong size has to be a value in the range
 * \f$ [0..\infty) \f$. The following images illustarte the effect of the phong size value: the
 * first image has a phong size of 180, the second uses the POV-Ray default of 40 and the third
 * has a phong size of 4.
 *
 * \image html phongsize.png
 * \image latex phongsize.eps "Examples for the phong size" width=600pt
 *
 * In case a phong highlight is used and the phong size is not specified, the default POV-Ray
 * value of 40 is used.
 *
 * \b Note: The PhongSize finish component has only an effect if it is used in combination with
 * the Phong component.
 */
class PE_PUBLIC PhongSize : public FinishItem
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit PhongSize( real phongsize );
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
   real phongsize_;   //!< The size of the phong highlight \f$ [0..\infty) \f$.
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
/*!\name PhongSize operators */
//@{
std::ostream& operator<<( std::ostream& os, const PhongSize& phongsize );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
