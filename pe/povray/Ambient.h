//=================================================================================================
/*!
 *  \file pe/povray/Ambient.h
 *  \brief Finish component for ambient lighting
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

#ifndef _PE_POVRAY_AMBIENT_H_
#define _PE_POVRAY_AMBIENT_H_


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
/*!\brief Finish component for ambient lighting.
 * \ingroup povray_finish
 *
 * The Ambient class represents the POV-Ray Finish component for ambient lighting. Ambient
 * lighting specifies the luminance of the body itself. The lighting value has to be in
 * the range \f$ [0..1] \f$. If the ambient lighting is set to 0, the rigid body will
 * appear black if it is not directly lighted by a light source. The following shows how
 * ambient lighting works: the first image uses the POV-Ray default of 0.1, the second was
 * rendered with an ambient value of 0.6.
 *
 * \image html ambient.png
 * \image latex ambient.eps "Examples for ambient lighting" width=400pt
 *
 * If the ambient lighting value of a body is not specified, the default POV-Ray value of
 * 0.1 is used.
 */
class PE_PUBLIC Ambient : public FinishItem
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Ambient( real ambient );
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
   real ambient_;  //!< The ambient value \f$ [0..1] \f$.
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
/*!\name Ambient operators */
//@{
std::ostream& operator<<( std::ostream& os, const Ambient& ambient );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
