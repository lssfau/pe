//=================================================================================================
/*!
 *  \file src/povray/Rotation.cpp
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iostream>
#include <pe/math/Constants.h>
#include <pe/povray/Rotation.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a Rotation transformation.
 *
 * \param xangle Rotation angle for the rotation around the x-axis (radian measure).
 * \param yangle Rotation angle for the rotation around the y-axis (radian measure).
 * \param zangle Rotation angle for the rotation around the z-axis (radian measure).
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
Rotation::Rotation( real xangle, real yangle, real zangle )
   : Transformation()                     // Initialization of the base class
   , rotation_( xangle, zangle, yangle )  // The euler rotation angles
{
   rotation_ *= ( -real(180)/M_PI );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a Rotation transformation.
 *
 * \param euler 3-dimensional vector containing the three rotation angles.
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
Rotation::Rotation( const Vec3& euler )
   : Transformation()                           // Initialization of the base class
   , rotation_( euler[0], euler[2], euler[1] )  // The euler rotation angles
{
   rotation_ *= ( -real(180)/M_PI );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUITPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray rotation transformation.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Rotation::print( std::ostream& os, bool newline ) const
{
   os << "rotate " << rotation_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray rotation transformation.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the rotation output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Rotation::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "rotate " << rotation_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Rotation class.
 * \ingroup povray_transformation
 *
 * \param os Reference to the output stream.
 * \param rotation Reference to a rotation transformation object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Rotation& rotation )
{
   rotation.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
