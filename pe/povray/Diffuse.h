//=================================================================================================
/*!
 *  \file pe/povray/Diffuse.h
 *  \brief Finish component for diffuse lighting
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

#ifndef _PE_POVRAY_DIFFUSE_H_
#define _PE_POVRAY_DIFFUSE_H_


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
/*!\brief Finish component for diffuse lighting.
 * \ingroup povray_finish
 *
 * The Diffuse class represents the POV-Ray finish component for diffuse lighting. Diffuse
 * lighting specifies how much light is diffused at the surface of the rigid body. The lighting
 * value has to be in the range \f$ [0..1] \f$. The images give an impression of three diffuse
 * lighting values: the first image uses a diffuse lighting of 0.3, the second was rendered
 * with the POV-Ray default of 0.6 and the third has a diffuse value of 0.9.
 *
 * \image html diffuse.png
 * \image latex diffuse.eps "Examples for diffuse lighting" width=600pt
 *
 * If the diffuse lighting value of a body is not specified, the default POV-Ray value of 0.6
 * is used.
 */
class PE_PUBLIC Diffuse : public FinishItem
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Diffuse( real diffuse );
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
   real diffuse_;  //!< The diffuse value \f$ [0..1] \f$.
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
/*!\name Diffuse operators */
//@{
std::ostream& operator<<( std::ostream& os, const Diffuse& diffuse );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
