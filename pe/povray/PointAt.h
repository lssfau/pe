//=================================================================================================
/*!
 *  \file pe/povray/PointAt.h
 *  \brief Focus point light source modifier
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

#ifndef _PE_POVRAY_POINTAT_H_
#define _PE_POVRAY_POINTAT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/math/Vector3.h>
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
/*!\brief Focus point light source modifier.
 * \ingroup povray_lightsource
 *
 * The PointAt class represents the 'point_at' keyword for POV-Ray light sources that is used
 * for parallel light sources and spotlights to focus the emitted light. It declares the point
 * the light source is supposed to point at and therefore orients the light source. In case
 * the PointAt light modifier is not used for parallel light sources or spotlights, the POV-Ray
 * default focus point (0,0,0) is used.
 */
class PE_PUBLIC PointAt : public LightItem
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit PointAt( real x, real y, real z );
   explicit PointAt( const Vec3& point );
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
   Vec3 point_;  //!< The light source focus point.
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
/*!\name PointAt operators */
//@{
std::ostream& operator<<( std::ostream& os, const PointAt& pointAt );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
