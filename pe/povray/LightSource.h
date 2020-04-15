//=================================================================================================
/*!
 *  \file pe/povray/LightSource.h
 *  \brief Implementation of light sources for the POV-Ray visualization
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

#ifndef _PE_POVRAY_LIGHTSOURCE_H_
#define _PE_POVRAY_LIGHTSOURCE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <string>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup povray_lightsource Light source
 * \ingroup povray
 *
 * Light sources are used to illuminate a POV-Ray scene. Without light sources, rigid bodies
 * are only lighted by their own ambient lighting, which gives them a rather two dimensional
 * appearance. Due to shadows, diffuse lighting and reflections, light sources are able to
 * create a fully three dimensional impression.\n
 * There are several different light sources available for the illumination of a POV-Ray scene.
 * The most basic light source is a point light that emits light of a specified color uniformly
 * in all directions (see the section about \ref pointlight). In contrast to point lights,
 * spotlights are directed light sources (see \ref spotlight). For far away light sources, like
 * for example sunlight, a parallel light source can be used (see the \ref parallel section).
 * The \ref arealight section provides information about area lights, that consist of several
 * point lights and enable soft shadows.\n\n
 *
 *
 * \section pointlight Point light sources
 *
 * Point lights are the default POV-Ray light sources. The point light emits light of the specified
 * color uniformly in all directions. The following image gives an impression of a point light:
 *
 * \image html pointlight.png
 * \image latex pointlight.eps "Examples for a point light source" width=200pt
 *
 * Point lights are created via the PointLight class. In order to create a point light source,
 * at least a global position and a color value have to be specified. The following code example
 * demonstrates the setup of a single point light source:

   \code
   using namespace pe::povray;
   LightSource ls = PointLight( Vec3( -2.0, -2.0, 10.0 ),   // The global position of the point light
                                Color( 0.95, 0.95, 0.95 ),  // The emitted light color
                                FadeDistance( 20.0 )        // Optional: the fading distance
                                FadePower( 2 ) );           // Optional: the fading power
   \endcode

 * For more details about point lights see the class description of the PointLight class.\n\n
 *
 *
 * \section spotlight Spotlights
 *
 * Spotlights offer the possibility to specify directed light sources. Spotlights can be used
 * to create a cone of light that is bright in the center and falls of to darkness in a soft
 * fringe effect at the edge. The following illustration shows how a spotlight works:
 *
 * \image html spotlight.png
 * \image latex spotlight.eps "Example for a spotlight" width=200pt
 *
 * Spotlights are created via the SpotLight class. In order to create a spotlight, at least a
 * global position and a color value have to be specified. However, in order to direct the
 * spotlight towards a specific focus point, a PointAt light modifier should be used. The
 * following example demonstrates the setup of a single spotlight:

   \code
   using namespace pe::povray;
   LightSource ls = SpotLight( Vec3( -2.0, -2.0, 10.0 ),    // The global position of the spotlight
                               Color( 0.95, 0.95, 0.95 ),   // Its color specification
                               PointAt( 2.0, 2.0, 0.0 ) );  // Optional: the focus point of the spotlight
   \endcode

 * For more details about spotlights see the SpotLight class description.\n\n
 *
 *
 * \section parallel Parallel light sources
 *
 * \image html parallellight.png
 * \image latex parallellight.eps "Examples for a parallel light source" width=200pt
 *
 * Parallel lights are useful for simulating very distant light sources, such as sunlight. A
 * parallel light source emits parallel light from a plane determined by the global position
 * of the light source and an additional focus point. The following code example illustrates
 * the setup of a single parallel light source at the global position (-2,-2,20), focused at
 * the point (-2,-2,0):

   \code
   using namespace pe::pov;
   LightSource ls = ParallelLight( Vec3( -2.0, -2.0, 20.0 ),     // The global position of the parallel light
                                   Color( 0.95, 0.95, 0.95 ),    // The light color
                                   PointAt( -2.0, -2.0,  0.0 ),  // Optional: its focus point
                                   FadeDistance( 20.0 ),         // Optional: the fading distance
                                   FadePower( 2 ) );             // Optional: the fading power
   \endcode

 * For more details about parallel light sources see the class description of the ParallelLight
 * class.\n\n
 *
 *
 * \section arealight Area light sources
 *
 * Area light sources are represented by a one- or two-dimensional array of individual point
 * lights. Each single point light source of the area light emits light of the specified color
 * uniformly in all directions. In constrast to a single point light source, area lights are able
 * to cast soft shadows because an object can partially block their light (whereas a single point
 * light source is either totally blocked or not blocked). The following images give an impression
 * of an area light source (especially note the soft shadow):
 *
 * \image html arealight.png
 * \image latex arealight.eps "Example for an area light source" width=200pt
 *
 * Area light sources are created via the AreaLight class. In order to create an area light, at
 * least the global position of the light source, its color and two axes that set the extension
 * the area light have to be specified. Additionally, the number of point lights along the two
 * axes have to be set. The following example demonstrates the setup of an area light source:

   \code
   using namespace pe::povray;
   LightSource ls = AreaLight( Vec3( 2.0, 2.0, 20.0 ),  // The global position of the area light
                               Color( 0.9, 0.9, 0.9 ),  // Its color
                               Vec3( 10.0, 0.0, 0.0 ),  // The x-extension of the area light
                               Vec3( 0.0, 10.0, 0.0 ),  // The y-extension of the area light
                               6, 6,                    // The number of point lights in x- and y-direction
                               FadeDistance( 20.0 ),    // Optional: the fading distance
                               FadePower( 2 ) );        // Optional: the fading power
   \endcode

 * For more informations about area light sources see the AreaLight class description.
 */
/*!\brief A POV-Ray light source.
 * \ingroup povray_lightsource
 *
 * The LightSource class represents a light source for a POV-Ray visualization. It is the base
 * class for all possible POV-Ray LightSources and offers the necessary common functionality.
 */
class PE_PUBLIC LightSource
{
private:
   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend std::ostream& operator<<( std::ostream& os, const LightSource& lightsource );
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline LightSource();
   explicit        LightSource( const char* const lightsource );
   explicit        LightSource( const std::string& lightsource );
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
   std::string lightsource_;  //!< POV-Ray string representation of the light source.
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
/*!\brief Default constructor for the LightSource class.
 *
 * The default light source is a solid white colored point light at (0,0,0).
 */
inline LightSource::LightSource()
   : lightsource_()  // POV-Ray string representation
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Clearing the POV-Ray light source.
 *
 * \return void
 *
 * This function removes all components from the POV-Ray light source and resets all values
 * to the POV-Ray defaults.
 */
inline void LightSource::reset()
{
   lightsource_.clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name LightSource operators */
//@{
std::ostream& operator<<( std::ostream& os, const LightSource& lightsource );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
