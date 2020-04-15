//=================================================================================================
/*!
 *  \file src/povray/ImagePigment.cpp
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/povray/ImagePigment.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating an image map pigment.
 *
 * \param imagemap The image map for the pigment.
 *
 * This constructor creates a POV-Ray pigment by projecting the given image map onto the rigid
 * body. The default mapping method is projecting the image onto the xz-plane in such a way
 * that it fits into the 2D-square (0,0,0), (1,0,1). The pixels of the original image are
 * turned into infinitely long boxes that are parallel to the y-axis. In order to change the
 * projection, the texture using the pigment can be rotated and scaled. Note that larger images
 * are only increasing the resolution of the projected image, not the image itself. In order to
 * scale the image, the texture using the pigment has to be scaled.\n
 * The images below give some impressions of image pigments. The first and second picture show
 * grass and water textures projected onto a plane, the third demonstrates spherical mapping of
 * a globe image onto a sphere.
 *
 * \image html imagemap.png
 * \image latex imagemap.eps "Examples for image maps" width=550pt
 */
ImagePigment::ImagePigment( const ImageMap& imagemap )
   : Pigment()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << pigment_ << "   " << imagemap << "\n";

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating an image map pigment.
 *
 * \param itype The format type of given image.
 * \param image The path and name of the image file.
 * \param mtype The mapping type for the image.
 * \param repeat \a true if the image is repeated infinitely, \a false if is used only once.
 *
 * This constructor creates a POV-Ray pigment by projecting the given image \a image onto the
 * rigid body. The default mapping method is projecting the image onto the xz-plane in such
 * a way that it fits into the 2D-square (0,0,0), (1,0,1). The pixels of the original image
 * are turned into infinitely long boxes that are parallel to the y-axis. In order to change
 * the projection, the texture using the pigment can be rotated and scaled. Note that larger
 * images are only increasing the resolution of the projected image, not the image itself.
 * In order to scale the image, the texture using the pigment has to be scaled.\n
 * The \a itype parameter specifies the format of the image file: use \a gif in case of a
 * gif-file, \a tga in case of a tga-file and \a iff in case of a iff-file. The mapping type
 * \a mtype specifies how the image is treated when mapped on a rigid body. A \a planar mapping
 * will project the image without adjustments on the surface of the rigid body. The \a spherical
 * mapping assumes that the body is a sphere of any size sitting at the origin and wrappes
 * the image around this sphere. The \a repeat parameter specifies if the image is repeated
 * infinitely or if it used only once. In case of a spherical mapping, the \a repeat parameter
 * has no effect and is ignored.\n
 * The images below give some impressions of image pigments. The first and second picture
 * show grass and water textures projected onto a plane, the third demonstrates spherical
 * mapping of a globe image onto a sphere.
 *
 * \image html imagemap.png
 * \image latex imagemap.eps "Examples for image maps" width=550pt
 *
 * Examples:

   \code
   using namespace pe::pov;
   ImageMap imagemap( gif, "grass.gif", planar );
   Pigment grass = ImagePigment( imagemap );
   \endcode
 */
ImagePigment::ImagePigment( ImageType itype, const std::string& image, MappingType mtype, bool repeat )
   : Pigment()  // Initialization of the base class
{
   ImageMap imagemap( itype, image, mtype, repeat );

   std::ostringstream oss;
   oss << pigment_ << "   " << imagemap << "\n";

   oss.str().swap( pigment_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
