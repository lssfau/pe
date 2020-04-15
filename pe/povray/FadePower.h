//=================================================================================================
/*!
 *  \file pe/povray/FadePower.h
 *  \brief Fade power light source modifier
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

#ifndef _PE_POVRAY_FADEPOWER_H_
#define _PE_POVRAY_FADEPOWER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/povray/LightItem.h>
#include <pe/system/Precision.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Fade power light source modifier.
 * \ingroup povray_lightsource
 *
 * By default POV-Ray does not diminish light from any light source as it travels through space.
 * In order to get a more realistic effect \a distance and \a power values can be specified to
 * model the distance based falloff in light intensity. Light fading in POV-Ray is modelled by
 * the following formula:

           \f[ attenuation = \frac{2}{ 1 + (\frac{d}{distance})^{power} }, \f]

 * where \a distance is the fade distance setting and \a power is the fade power setting of the
 * light source.\n
 * The FadePower class represents the fade power setting of a POV-Ray light source. It determines
 * the falloff rate of a POV-Ray light source, which describes the actual attenuation of the light
 * source. For example linear or quadratic falloff can be used by setting fade_power to 1 or 2
 * respectively. For a complete discussion about light fading, see the POV-Ray documentation:\n
 *
 * http://www.povray.org/documentation/view/3.6.1/317/
 */
class PE_PUBLIC FadePower : public LightItem
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit FadePower( real power );
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
   real power_;  //!< The fade power value \f$ (0..\infty) \f$.
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
/*!\name FadePower operators */
//@{
std::ostream& operator<<( std::ostream& os, const FadePower& fadepower );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
