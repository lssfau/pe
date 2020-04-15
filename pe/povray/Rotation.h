//=================================================================================================
/*!
 *  \file pe/povray/Rotation.h
 *  \brief POV-Ray transformation for rotations
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

#ifndef _PE_POVRAY_ROTATION_H_
#define _PE_POVRAY_ROTATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/math/Vector3.h>
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
/*!\brief A POV-Ray rotation transformation.
 * \ingroup povray_transformation
 *
 * The Rotation class is a POV-Ray transformation for rotation operations. There are two ways
 * to construct a rotation:
 *
 * -# Rotation::Rotation( real xangle, real yangle, real zangle );
 * -# Rotation::Rotation( const Vec3& euler );
 *
 * The rotations are applied in the order x, y and z. In order to change the order, several
 * rotations have to be specified. Note that all three angles have to be specified in radian
 * measure.\n
 * The following pictures give an impression of texture rotations. The first image shows an
 * image map of planet earth without any rotations. The second image shows the same image
 * map rotated by \f$ \pi/2 \f$ (radian measure) around the z-axis. In the third image the
 * rotation was increased to \f$ \pi \f$ (radian measure) around the z-axis, whereas the
 * fourth image shows a rotation of \f$ \pi/2 \f$ around the x-axis.
 *
 * \image html rotation.png
 * \image latex rotation.eps "Examples for POV-Ray textures" width=600pt
 */
class PE_PUBLIC Rotation : public Transformation
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Rotation( real xangle, real yangle, real zangle );
   explicit Rotation( const Vec3& euler );
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
   Vec3 rotation_;  //!< The euler rotation angles.
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
/*!\name Rotation operators */
//@{
std::ostream& operator<<( std::ostream& os, const Rotation& rotation );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
