//=================================================================================================
/*!
 *  \file src/povray/ImageMap.cpp
 *  \brief Implementation of an image map for the POV-Ray visualization
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
#include <sstream>
#include <pe/povray/ImageMap.h>
#include <pe/util/Assert.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The constructor for the ImageMap class.
 *
 * \param itype The format type of given image.
 * \param image The path and name of the image file.
 * \param mtype The mapping type for the image.
 * \param repeat \a true if the image is repeated infinitely, \a false if is used only once.
 *
 * This constructor creates a new image map from the given image file \a image. \a itype
 * specifies the format of the image file: use \a gif in case of a gif-file, \a tga in case
 * of a tga-file and \a iff in case of a iff-file. The mapping type \a mtype specifies how
 * the image is treated when mapped on a rigid body. A \a planar mapping will project the image
 * without adjustments on the surface of the rigid body. The \a spherical mapping assumes that
 * the rigid body is a sphere of any size sitting at the origin and wrappes the image around
 * this sphere. The \a repeat parameter specifies if the image is repeated infinitely or if
 * it used only once. In case of a spherical mapping, the \a repeat parameter has no effect
 * and is ignored.\n
 * The following pictures show three possible image maps. The first two demonstrate grass and
 * water images mapped onto a plane, the third demonstrates spherical mapping of a globe image
 * onto a sphere.
 *
 * \image html imagemap.png
 * \image latex imagemap.eps "Examples for image maps" width=550pt
 */
ImageMap::ImageMap( ImageType itype, const std::string& image, MappingType mtype, bool repeat )
   : imagemap_()  // POV-Ray string representation
{
   std::ostringstream oss;

   switch( itype ) {
      case gif : oss << "gif" ; break;
      case tga : oss << "tga" ; break;
      case iff : oss << "iff" ; break;
      case ppm : oss << "ppm" ; break;
      case pgm : oss << "pgm" ; break;
      case png : oss << "png" ; break;
      case jpeg: oss << "jpeg"; break;
      case tiff: oss << "tiff"; break;
      case sys : oss << "sys" ; break;
      default  : pe_INTERNAL_ASSERT( false, "Unknown ImageType" ); break;
   }

   oss << " \"" << image << "\" map_type " << mtype;
   if( mtype != spherical && !repeat ) oss << " once";
   oss << " interpolate 2";

   oss.str().swap( imagemap_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray image map.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void ImageMap::print( std::ostream& os, bool newline ) const
{
   os << "image_map { " << imagemap_ << " }";
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray image map.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the image map output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void ImageMap::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "image_map { " << imagemap_ << " }";
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the ImageMap class.
 * \ingroup povray_imagemap
 *
 * \param os Reference to the output stream.
 * \param imagemap Reference to a image map object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const ImageMap& imagemap )
{
   imagemap.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
