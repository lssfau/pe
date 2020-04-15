//=================================================================================================
/*!
 *  \file pe/povray/ImagePigment.h
 *  \brief Implementation of a POV-Ray image pigment
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

#ifndef _PE_POVRAY_IMAGEPIGMENT_H_
#define _PE_POVRAY_IMAGEPIGMENT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <sstream>
#include <string>
#include <pe/povray/ImageMap.h>
#include <pe/povray/Pigment.h>
#include <pe/povray/Transformation.h>
#include <pe/util/constraints/SameSize.h>
#include <pe/util/constraints/TypeRestriction.h>
#include <pe/util/TypeList.h>


namespace pe {

namespace povray {

//*************************************************************************************************
/*!\brief A POV-Ray image pigment.
 * \ingroup povray_pigment
 *
 * The ImagePigment class represents a POV-Ray pigment built from an image map. Image pigments
 * create a POV-Ray pigment by projecting the given image map onto a rigid body. The images
 * show three possible applications for image pigments. The first and second picture show
 * grass and water textures projected onto a plane, the third demonstrates spherical mapping
 * of a globe image onto a sphere.
 *
 * \image html imagemap.png
 * \image latex imagemap.eps "Examples for image maps" width=550pt
 *
 * Image pigments are created via one of the ImagePigment constructors:
 * -# ImagePigment::ImagePigment( const pe::povray::ImageMap& imagemap );
 * -# ImagePigment::ImagePigment( ImageType itype, const std::string& image, MappingType mtype, bool repeat );
 *
 * Examples:

   \code
   using namespac pe::pov;
   ImageMap imagemap( gif, "grass.gif", planar );
   Pigment grass = ImagePigment( imagemap );  (1)
   \endcode
 */
class PE_PUBLIC ImagePigment : public Pigment
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_1( Transformation )  ValidTypes;  //!< Valid modifiers/transformations.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit ImagePigment( const ImageMap& imagemap );
   explicit ImagePigment( ImageType itype, const std::string& image, MappingType mtype, bool repeat=true );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~ImagePigment();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template< typename A >
   void add( const A& a );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the ImagePigment class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline ImagePigment::~ImagePigment()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Pigment, ImagePigment );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a transformation to the POV-Ray image pigment.
 *
 * \param a The new transformation.
 * \return void
 *
 * This function adds a new transformation to the image pigment. Valid transformations are
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A >
void ImagePigment::add( const A& a )
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << pigment_;
   a.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
