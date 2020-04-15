//=================================================================================================
/*!
 *  \file pe/povray/Pigment.h
 *  \brief Implementation of a POV-Ray pigment
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

#ifndef _PE_POVRAY_PIGMENT_H_
#define _PE_POVRAY_PIGMENT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <string>
#include <pe/povray/TextureItem.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup povray_pigment Pigment
 * \ingroup povray
 *
 * A pigment is what gives life to the rigid bodies in a POV-Ray visualization and is the
 * primary part of a POV-Ray texture. Possible pigments include single-colored pigments or
 * pigments with a specific color pattern like e.g. marble pigments or spotted pigments.\n\n
 *
 *
 * \section content Contents
 *
 * 1.) \ref color_pigment\n
 * 2.) \ref agate_pigment\n
 * 3.) \ref bozo_pigment\n
 * 4.) \ref granite_pigment\n
 * 5.) \ref marble_pigment\n
 * 6.) \ref radial_pigment\n
 * 7.) \ref spotted_pigment\n
 * 8.) \ref image_pigment\n\n
 *
 *
 * \section color_pigment Single-colored pigments
 *
 * Single-colored pigments (implemented in the ColorPigment class) are the simplest possible
 * pigments. The following images give an idea of what single-colored pigments can look like:
 *
 * \image html singlecolor.png
 * \image latex singlecolor.eps "Examples for single-colored pigments" width=600pt
 *
 * Single-colored pigments are created by the ColorPigment constructors:
 * -# ColorPigment::ColorPigment( real red, real green, real blue, real alpha );
 * -# ColorPigment::ColorPigment( const pe::povray::Color& color );
 * -# ColorPigment::ColorPigment( const std::string& color );
 *
 * Examples:

   \code
   using namespace pe::pov;
   Pigment yellow = ColorPigment( 1.0, 1.0, 0.0, 0.0 );            (1)
   Pigment red    = ColorPigment( Color( "color rgb <1,0,0>" ) );  (2)
   Pigment blue   = ColorPigment( "color rgb <0,0,1>" );           (3)
   \endcode

 * \b Note: In case of spheres, a single-colored pigment is not able to show the rotation of
 * the spheres.\n\n
 *
 *
 * \section agate_pigment Agate pigments
 *
 * Agate pigments (implemented in the AgatePigment class) use a POV-Ray ColorMap to create a
 * very turbulent color pattern. The images show some examples for agate pigments:
 *
 * \image html agatePigment.png
 * \image latex agatePigment.eps "Examples for agate pigments" width=600pt
 *
 * Agate pigments are created by specifying at least a ColorMap. Additionally, up to five
 * modifiers (as for instance Turbulence, Frequency, Phase, ...) or transformations (Scale,
 * Translation, Rotation) can be directly specified.
 *
 * Examples:

   \code
   using namespace pe::pov;
   ColorMap colormap( "[0.0 color Red][0.33 color Yellow][0.66 color Blue][1.0 color Red]" );
   Pigment plain     = AgatePigment( colormap );                     (1)
   Pigment distorted = AgatePigment( colormap, Turbulence( 0.6 ) );  (2)
   \endcode

 * \b Note: For an explanation of the color map parameter string format, take a look at the
 * details of the ColorMap class!\n\n
 *
 *
 * \section bozo_pigment Bozo pigments
 *
 * Bozo pigments (implemented in the BozoPigment class) use a POV-Ray ColorMap to create
 * splotches of color on a rigid body. The images demonstrate some possible bozo pigments:
 *
 * \image html bozoPigment.png
 * \image latex bozoPigment.eps "Examples for bozo pigments" width=600pt
 *
 * Bozo pigments are created by specifying at least a ColorMap. Additionally, up to five
 * modifiers (as for instance Turbulence, Frequency, Phase, ...) or transformations (Scale,
 * Translation, Rotation) can be directly specified.
 *
 * Examples:

   \code
   using namespace pe::pov;
   ColorMap colormap( "[0.0 color Red][0.33 color Yellow][0.66 color Blue][1.0 color Red]" );
   Pigment plain     = BozoPigment( colormap );                    (1)
   Pigment distorted = BozoPigment( colormap, Frequency( 6.0 ) );  (2)
   \endcode

 * \b Note: For an explanation of the color map parameter string format, take a look at the
 * details of the ColorMap class!\n\n
 *
 *
 * \section granite_pigment Granite pigments
 *
 * Granite pigments (implemented in the GranitePigment class) form a color pattern that looks
 * very convincingly like real granite. The following four pictures give an impression of the
 * capabilities of granite pigments:
 *
 * \image html granitePigment.png
 * \image latex granitePigment.eps "Examples for granite pigments" width=800pt
 *
 * Granite pigments are created by specifying at least a ColorMap. Additionally, up to five
 * modifiers (as for instance Turbulence, Frequency, Phase, ...) or transformations (Scale,
 * Translation, Rotation) can be directly specified.
 *
 * Examples:

   \code
   using namespace pe::pov;
   ColorMap colormap( "[0.0 color Red][0.33 color Yellow][0.66 color Blue][1.0 color Red]" );
   Pigment granite   = GranitePigment( colormap );                     (1)
   Pigment distorted = GranitePigment( colormap, Turbulence( 0.8 ) );  (2)
   \endcode

 * \b Note: For an explanation of the color map parameter string format, take a look at the
 * details of the ColorMap class!\n\n
 *
 *
 * \section marble_pigment Marble pigments
 *
 * Marble pigments (implemented in the MarblePigment class) map a POV-Ray ColorMap on the rigid
 * body that can be distorted by a specified level of turbulence. The images demonstrate some
 * possible marble pigments:
 *
 * \image html marblePigment.png
 * \image latex marblePigment.eps "Examples for marble pigments" width=600pt
 *
 * Marble pigments are created by specifying at least a ColorMap. Additionally, up to five
 * modifiers (as for instance Turbulence, Frequency, Phase, ...) or transformations (Scale,
 * Translation, Rotation) can be directly specified.
 *
 * Examples:

   \code
   using namespace pe::pov;
   ColorMap colormap( "[0.5 color Red][0.5 color White]" );
   Pigment striped   = MarblePigment( colormap );                     (1)
   Pigment distorted = MarblePigment( colormap, Turbulence( 0.8 ) );  (2)
   \endcode

 * \b Note: For an explanation of the color map parameter string format, take a look at the
 * details of the ColorMap class!\n\n
 *
 *
 * \section radial_pigment Radial pigments
 *
 * Radial pigments (implemented in the RadialPigment class) wrap a given POV-Ray ColorMap clockwise
 * around the z-axis, starting in the positive x direction. The following images give an impression
 *of possible radial pigments:
 *
 * \image html radialPigment.png
 * \image latex radialPigment.eps "Examples for radial pigments" width=400pt
 *
 * The setup of a RadialPigment requires at least a ColorMap. In order to modify the appearance
 * of the radial pigment, up to 5 additional modifiers (as for instance Turbulence, Frequency,
 * Phase, ...) and transformations (Scale, Translation, Rotation) can be directly specified.
 *
 * Examples:

   \code
   using namespace pe::pov;
   ColorMap colormap( "[0.0 color Red][0.33 color Yellow][0.66 color Blue][1.0 color Red]" );
   Pigment colorful   = RadialPigment( colormap );                    (1)
   Pigment redstripes = RadialPigment( colormap, Frequency( 8.0 ) );  (2)
   \endcode

 * \b Note: For an explanation of the color map parameter string format, take a look at the
 * details of the ColorMap class!\n\n
 *
 *
 * \section spotted_pigment Spotted pigments
 *
 * Spotted pigments (implemented in the SpottedPigment class) use a POV-Ray ColorMap to create
 * colored spots on the rigid bodies. The images demonstrate several spotted pigments:
 *
 * \image html spottedPigment.png
 * \image latex spottedPigment.eps "Examples for spotted pigments" width=800pt
 *
 * In order to create a SpottedPigment it is necessary to specify a ColorMap. Additionally,
 * up to five modifiers (Frequency, Phase, ...) or transformations (Scale, Translation,
 * Rotation) can be directly specified.
 *
 * Examples:

   \code
   using namespace pe::pov;
   ColorMap colormap( "[0.4 color Red][0.4 color White]" )
   Pigment reddots   = SpottedPigment( colormap );                    (1)
   Pigment whitedots = SpottedPigment( colormap, Frequency( 3.0 ) );  (2)
   \endcode

 * \b Note: For an explanation of the color map parameter string format, take a look at the
 * details of the ColorMap class!\n\n
 *
 *
 * \section image_pigment Image pigments
 *
 * Image pigments (implemented in the ImagePigment class) create a POV-Ray pigment by projecting
 * the given image map onto a rigid body. The images show three possible applications for image
 * pigments. The first and second picture show grass and water textures projected onto a plane,
 * the third demonstrates spherical mapping of a globe image onto a sphere.
 *
 * \image html imagemap.png
 * \image latex imagemap.eps "Examples for image maps" width=550pt
 *
 * Image pigments are created via the ImagePigment constructors:
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
/*!\brief A POV-Ray pigment.
 * \ingroup povray_pigment
 *
 * The Pigment class represents a POV-Ray pigment. It is the base class for all possible POV-Ray
 * pigments and offers the necessary common functionality as a TextureItem.
 */
class PE_PUBLIC Pigment : public TextureItem
{
public:
   //**Type definitions****************************************************************************
   typedef Pigment  Type;  //!< Type of the TextureItem.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline Pigment();
   explicit        Pigment( const char* const pigment );
   explicit        Pigment( const std::string& pigment );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void reset();
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os,                  bool newline ) const;
   void print( std::ostream& os, const char* tab, bool newline ) const;
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   std::string pigment_;  //!< POV-Ray string representation of the pigment.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for the Pigment class.
 *
 * The default pigment is a solid black color.
 */
inline Pigment::Pigment()
   : TextureItem()  // Initialization of the base class
   , pigment_()     // POV-Ray string representation of the pigment
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Clearing the POV-Ray pigment.
 *
 * \return void
 *
 * This function removes all components from the POV-Ray pigment and resets all values to the
 * POV-Ray defaults.
 */
inline void Pigment::reset()
{
   pigment_.clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Pigment operators */
//@{
std::ostream& operator<<( std::ostream& os, const Pigment& pigment );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
