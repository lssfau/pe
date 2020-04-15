//=================================================================================================
/*!
 *  \file pe/povray/ImageMap.h
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

#ifndef _PE_POVRAY_IMAGEMAP_H_
#define _PE_POVRAY_IMAGEMAP_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <string>


namespace pe {

namespace povray {

//=================================================================================================
//
//  ENUMERATIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The type of an image for a POV-Ray image map.
 * \ingroup povray
 */
enum ImageType {
   gif  = 0,  //!< Type of a gif image file.
   tga  = 1,  //!< Type of a tga image file.
   iff  = 2,  //!< Type of a iff image file.
   ppm  = 3,  //!< Type of a ppm image file.
   pgm  = 4,  //!< Type of a pgm image file.
   png  = 5,  //!< Type of a png image file.
   jpeg = 6,  //!< Type of a jpeg image file.
   tiff = 7,  //!< Type of a tiff image file.
   sys  = 8   //!< Type of a sys image file.
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The mapping type for an image in a POV-Ray image map.
 * \ingroup povray
 */
enum MappingType {
   planar    = 0,  //!< Planar mapping of the image.
                   /*!< The default mapping of an image on a rigid body is planar mapping. Planar
                        mapping is projecting the image onto the xz-plane as it is. */
   spherical = 1   //!< Spherical mapping of the image.
                   /*!< Spherical mapping assumes that the rigid body is a sphere of any size sitting
                        at the origin. The z-axis is the north/south pole of the spherical mapping.
                        The top and bottom edges of the image just touch the pole regardless of any
                        scaling. The left edge of the image begins at the positive x-axis and wraps
                        the image around the sphere from west to east in a z-rotation. The image
                        covers the sphere exactly once, so no repetition is possible. */
};
//*************************************************************************************************




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup povray_imagemap Image map
 * \ingroup povray
 */
/*!\brief A POV-Ray image map.
 * \ingroup povray_imagemap
 *
 * An image map offers the possibility to map images on rigid bodies in a POV-Ray visualization.
 * In order to create an image map, gif-, tga- or iff-files can be used. The default mapping
 * method (planar) is to project the image onto the xz-plane in such a way that it fits into
 * the 2D-square (0,0,0), (1,0,1). The pixels of the original image are turned into infinitely
 * long boxes that are parallel to the y-axis. Next to the default planar mapping, spherical
 * mapping can be used to project the image onto spheres. For a detailed description of the
 * mapping types see pe::MappingType. By default, the image is repeated infinitely in the
 * x- and z-directions.\n
 * Image maps are created via the only constructor of the ImageMap class:

   \code
   pe::ImageMap( ImageType itype, const std::string& image, MappingType mtype, bool repeat )
   \endcode

 * The following pictures give an impression of what image maps are able to achieve. The
 * first two pictures show image maps of grass and water textures projected onto a plane,
 * the third demonstrates spherical mapping of a globe image onto a sphere.
 *
 * \image html imagemap.png
 * \image latex imagemap.eps "Examples for image maps" width=550pt
 *
 * \b Note: \n
 * -# To change the default projection, the image needs to be rotated to fit the surface
 *    it is mapped on. In order to rotate an image, rotate the texture using the image map.
 * -# Larger images are only increasing the resolution of the projected image, not the
 *    image itself. In order to scale the image, scale the texture using the image map.
 */
class PE_PUBLIC ImageMap
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   ImageMap( ImageType itype, const std::string& image, MappingType mtype, bool repeat=true );
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
   std::string imagemap_;  //!< POV-Ray string representation of the image map.
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
/*!\name ImageMap operators */
//@{
std::ostream& operator<<( std::ostream& os, const ImageMap& imagemap );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
