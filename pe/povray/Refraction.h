//=================================================================================================
/*!
 *  \file pe/povray/Refraction.h
 *  \brief Finish component for refractions
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

#ifndef _PE_POVRAY_REFRACTION_H_
#define _PE_POVRAY_REFRACTION_H_


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
/*!\brief Finish component for refractions.
 * \ingroup povray_finish
 *
 * The Refraction class represents the POV-Ray finish component for refractions. Refraction only
 * has meaning on rigid bodies that have at least a litte bit of transparency. Refraction is the
 * bending of light rays as they pass into a more dense or less dense medium. As light does not
 * go through opaque things, they don't refract. Without refraction, transparent bodies look like
 * colored air. The refraction value has to be in the range \f$ [1..\infty) \f$. A value of 1.0
 * is the POV-Ray default and will not change the refraction of a body. Examples for some physical
 * refraction values are 1.000292 for air, 1.33 for water or 1.5 for glass. The images illustrate
 * the refraction effect: the first image has no refraction, whereas the second image uses a
 * refraction of 2.0.
 *
 * \image html refraction.png
 * \image latex refraction.eps "Examples for refraction" width=400pt
 *
 * If no refraction is specified for a rigid body, refraction is turned off by default.
 */
class PE_PUBLIC Refraction : public FinishItem
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Refraction( real refraction );
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
   real refraction_;  //!< The refraction value \f$ [1..\infty) \f$.
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
/*!\name Refraction operators */
//@{
std::ostream& operator<<( std::ostream& os, const Refraction& refraction );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
