//=================================================================================================
/*!
 *  \file pe/povray/Normal.h
 *  \brief Implementation of a POV-Ray normal
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

#ifndef _PE_POVRAY_NORMAL_H_
#define _PE_POVRAY_NORMAL_H_


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
/*!\defgroup povray_normal Normal
 * \ingroup povray
 *
 * By adding a surface normal to a texture it's possible to create effects on the surface of
 * an object. Since much of the way we perceive objects is based on how they reflect light, it
 * is possible to trick the eye into thinking a surface is bumpy or corrugated by just modify
 * the surface normal vectors. The following example creates a texture for a water surface. In
 * order to make it more realistic, we add a surface normal to add ripples to the texture.
 * Although this effect is not perfectly realistic, the visual effect is close enough to real
 * water ripples. Note though, that adding a normal to a texture does not actually change the
 * location of the surface, it just makes it look different.
 *
 * \image html ripples.png
 * \image latex ripples.eps "POV-Ray ripples normal" width=400pt
 *
 * There are ten available sorts of normals available: \ref agate, \ref bozo, \ref bumps,
 * \ref dents, \ref granite, \ref marble, \ref ripples, \ref spotted, \ref waves and
 * \ref wrinkles.\n\n
 *
 *
 * \section agate Agate normals
 *
 * The AgateNormal creates an impressive, 3-dimensional pattern of turbulent valleys. The
 * illustration demonstrates the effect of an agate normal:
 *
 * \image html agateNormal.png
 * \image latex agateNormal.eps "POV-Ray agate normal" width=400pt
 *
 * The following code demonstrates the setup of this particular texture:

   \code
   // Creating a yellow color pigment
   pe::povray::ColorPigment pigment( 1.0, 1.0, 0.0 );

   // Creating a reflecting finish
   pe::povray::Finish finish( pe::povray::Phong( 0.8 ),
                              pe::povray::Reflection( 0.15 ) );

   // Creating a scaled agate normal with a depth of 0.8
   pe::povray::AgateNormal agate( 0.8, pe::Scale( 0.25 ) );

   // Creating the POV-Ray texture for the example image above
   pe::povray::PlainTexture texture( pigment,
                                     finish ,
                                     agate   );
   \endcode

 * \n \section bozo Bozo normals
 *
 * The BozoNormals create a random, bumpy pattern similar to the Bumps normal (see the \ref bumps
 * section). However, the effect of the bozo normal is weaker and softer. The images give an
 * impression of the effect of a bozo normal:
 *
 * \image html bozoNormal.png
 * \image latex bozoNormal.eps "POV-Ray bozo normal" width=400pt
 *
 * The following code demonstrates the setup of this particular texture:

   \code
   // Creating a yellow color pigment
   pe::povray::ColorPigment pigment( 1.0, 1.0, 0.0 );

   // Creating a reflecting finish
   pe::povray::Finish finish( pe::povray::Phong( 0.8 ),
                              pe::povray::Reflection( 0.15 ) );

   // Creating a scaled bozo normal with a depth parameter of 0.5 and additional turbulence
   pe::povray::BozoNormal bozo( 0.6, pe::povray::Turbulence( 0.5 ),
                                     pe::povray::Scale( 0.167 ) );

   // Creating the POV-Ray texture for the example image above
   pe::povray::PlainTexture texture( pigment,
                                     finish ,
                                     bozo    );
   \endcode

 * \n \section bumps Bumps
 *
 * The Bumps normal adds a random bumpy pattern to the texture. The images give an impression
 * of the effect of a bumps normal:
 *
 * \image html bumps.png
 * \image latex bumps.eps "POV-Ray bumps normal" width=400pt
 *
 * The following code demonstrates the setup of this particular bumpy texture:

   \code
   // Creating a yellow color pigment
   pe::povray::ColorPigment pigment( 1.0, 1.0, 0.0 );

   // Creating a reflecting finish
   pe::povray::Finish finish( pe::povray::Phong( 0.8 ),
                              pe::povray::Reflection( 0.15 ) );

   // Creating a scaled bumps normal with a bump depth of 0.5 and additional turbulence
   pe::povray::Bumps bumps( 0.6, pe::povray::Turbulence( 0.5 ),
                                 pe::povray::Scale( 0.167 ) );

   // Creating the POV-Ray texture for the example image above
   pe::povray::PlainTexture texture( pigment,
                                     finish ,
                                     bumps   );
   \endcode

 * \n \section dents Dents
 *
 * The Dents normal basically makes an object look like someone attacked it with a sledgehammer.
 * The surface appears to have dents beaten into it. The depth parameter adjustes the size of
 * the dents. A value of 0 creates the impression of a brand new object (and turn off the normal
 * effect), a value of 1 gives the impression that the object survived a hail storm.
 *
 * \image html dents.png
 * \image latex dents.eps "POV-Ray dents normal" width=400pt
 *
 * This code shows the setup of the above dents texture:

   \code
   // Creating a yellow color pigment
   pe::povray::ColorPigment pigment( 1.0, 1.0, 0.0 );

   // Creating a reflecting finish
   pe::povray::Finish finish( pe::povray::Phong( 0.8 ),
                              pe::povray::Reflection( 0.15 ) );

   // Creating a scaled dents normal with a dents depth of 0.5 and additional turbulence
   pe::povray::Dents dents( 1.0, pe::povray::Turbulence( 0.6 ),
                                 pe::povray::Scale( 0.167 ) );

   // Creating the POV-Ray texture for the example image above
   pe::povray::PlainTexture texture( pigment,
                                     finish ,
                                     dents   );
   \endcode

 * \n \section granite Granite normals
 *
 * The GraniteNormal creates a strong, rocky normal effect that closly resembles a rough stone
 * formation (see the illustration below).
 *
 * \image html graniteNormal.png
 * \image latex graniteNormal.eps "POV-Ray granite normal" width=400pt
 *
 * This code shows the setup of the above texture:

   \code
   // Creating a yellow color pigment
   pe::povray::ColorPigment pigment( 1.0, 1.0, 0.0 );

   // Creating a reflecting finish
   pe::povray::Finish finish( pe::povray::Phong( 0.8 ),
                              pe::povray::Reflection( 0.15 ) );

   // Creating a scaled granite normal with a depth of 1.0 and additional turbulence
   pe::povray::GraniteNormal granite( 1.0, pe::povray::Scale( 0.5 ) );

   // Creating the POV-Ray texture for the example image above
   pe::povray::PlainTexture texture( pigment,
                                     finish ,
                                     granite );
   \endcode

 * \n \section marble Marble normals
 *
 * The MarbleNormal offer a strong impression of a rough surface as for example stones or wood.
 * Note however that this effect is only achieved by at least a small amount of turbulence.
 * Otherwise, the effect may look boring due to the regular, striped structure. The image below
 * demonstrates the effect of a marble normal:
 *
 * \image html marbleNormal.png
 * \image latex marbleNormal.eps "POV-Ray marble normal" width=400pt
 *
 * This code shows the setup of the above texture:

   \code
   // Creating a yellow color pigment
   pe::povray::ColorPigment pigment( 1.0, 1.0, 0.0 );

   // Creating a reflecting finish
   pe::povray::Finish finish( pe::povray::Phong( 0.8 ),
                              pe::povray::Reflection( 0.15 ) );

   // Creating a scaled marble normal with a depth of 0.9 and additional turbulence
   pe::povray::MarbleNormal marble( 0.9, pe::povray::Turbulence( 0.8 ),
                                         pe::povray::Scale( 0.25 ) );

   // Creating the POV-Ray texture for the example image above
   pe::povray::PlainTexture texture( pigment,
                                     finish ,
                                     marble );
   \endcode

 * \n \section ripples Ripples
 *
 * The Ripples normal consists of evenly spaced, smooth ripples which originate from 10 random
 * locations inside the box with corners (0,0,0) and (1,1,1). In order to shift the origin of
 * the ripples to a different location, Translation/Rotation transformations have to be used.
 * All resulting waves have the same frequency, so the ripple effect is smooth at a significant
 * distance from the center.
 *
 * \image html ripples.png
 * \image latex ripples.eps "POV-Ray ripples normal" width=400pt
 *
 * The following code example demonstrates the setup of the rippled water surface texture:

   \code
   // Creating a transparent pigment for the water
   pe::povray::ColorPigment pigment( 0.9, 0.9, 0.9, 0.85 );

   // Creating a fitting finish for a water surface
   pe::povray::Finish finish( pe::povray::Reflection( 0.3 )
                              pe::povray::Refraction( 2.0 ) );

   // Creating a surface normal to create the ripples effect
   pe::povray::Ripples ripples( 1.0, pe::povray::Turbulence( 0.4 ),
                                     pe::povray::Frequency( 2.0 ),
                                     pe::povray::Phase( 0.35 ) );

   // Creating the water texture for the example image above
   pe::povray::PlainTexture texture( pigment,
                                     finish ,
                                     ripples );
   \endcode

 * \n \section spotted Spotted normals
 *
 * Using a SpottedNormal creates a slight pertubation in the perfectly smooth surface of a rigid
 * body. This bumpy effect comes in very handy in case a nearly smooth surface is required, as for
 * instance old metal. The following image demonstrates the effect of a spotted normal:
 *
 * \image html spottedNormal.png
 * \image latex spottedNormal.eps "POV-Ray spotted normal" width=400pt
 *
 * The following code example demonstrates the setup of the example texture:

   \code
   // Creating a yellow color pigment
   pe::povray::ColorPigment pigment( 1.0, 1.0, 0.0 );

   // Creating a reflecting finish
   pe::povray::Finish finish( pe::povray::Phong( 0.8 ),
                              pe::povray::Reflection( 0.15 ) );

   // Creating a scaled spotted normal with a depth of 1.0 and additional turbulence
   pe::povray::SpottedNormal spotted( 1.0, pe::povray::Turbulence( 0.6 ),
                                           pe::povray::Scale( 0.167 ) );

   // Creating the POV-Ray texture for the example image above
   pe::povray::PlainTexture texture( pigment,
                                     finish ,
                                     spotted );
   \endcode

 * \n \section waves Waves
 *
 * The Waves normal is similar to the Ripples normal, except instead of creating smooth, even
 * ripples, it creates more rough and tumble waves that look more like ocean waves. The depth
 * parameter adjusts the size of the waves. A value of 0 creates a calm surface, the maximum
 * value of 1 creates tsunami-like waves.
 *
 * \image html waves.png
 * \image latex waves.eps "POV-Ray waves normal" width=400pt
 *
 * The next code example shows the setup of the illustrated waves texture:

   \code
   // Creating a transparent pigment for the water
   pe::povray::ColorPigment pigment( 0.9, 0.9, 0.9, 0.85 );

   // Creating a fitting finish for a water surface
   pe::povray::Finish finish( pe::povray::Reflection( 0.3 )
                              pe::povray::Refraction( 2.0 ) );

   // Creating a surface normal to create the wave effect
   pe::povray::Waves waves( 0.7, pe::povray::Turbulence( 0.4 ),
                                 pe::povray::Frequency( 2.0 ),
                                 pe::povray::Phase( 0.35 ) );

   // Creating the water texture for the example image above
   pe::povray::PlainTexture texture( pigment,
                                     finish ,
                                     waves   );
   \endcode

 * \n \section wrinkles Wrinkles
 *
 * A Wrinkles normal creates one of the neatest surface effects, but it is also one of the
 * slowest to compute. This normal basically makes an object look like it had been wadded up
 * and then stretched back out again. It basically works by repeatedly denting the surface
 * with smaller and smaller dents. The following images give an impression of a wrinkles
 * normal:
 *
 * \image html wrinkles.png
 * \image latex wrinkles.eps "POV-Ray dents normal" width=400pt
 *
 * The code example demonstrates the setup of the wrinkled texture:

   \code
   // Creating a yellow color pigment
   pe::povray::ColorPigment pigment( 1.0, 1.0, 0.0 );

   // Creating a reflecting finish
   pe::povray::Finish finish( pe::povray::Phong( 0.8 ),
                              pe::povray::Reflection( 0.15 ) );

   // Creating a scaled dents normal with a dents depth of 0.5 and additional turbulence
   pe::povray::Wrinkles wrinkles( 0.6, pe::povray::Turbulence( 0.3 ),
                                       pe::povray::Scale( 0.167 ) );

   // Creating the POV-Ray texture for the example image above
   pe::povray::PlainTexture texture( pigment,
                                     finish ,
                                     wrinkles );
   \endcode
 */
/*!\brief A POV-Ray normal.
 * \ingroup povray_normal
 *
 * The Normal class represents the POV-Ray normal component of a texture. It is the base class for
 * all possible POV-Ray normals and offers the necessary common functionality as a TextureItem.
 */
class PE_PUBLIC Normal : public TextureItem
{
public:
   //**Type definitions****************************************************************************
   typedef Normal  Type;  //!< Type of the TextureItem.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline Normal();
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
   std::string normal_;  //!< POV-Ray string representation of the normal.
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
/*!\brief Default constructor for the Normal class.
 */
inline Normal::Normal()
   : TextureItem()  // Initialization of the base class
   , normal_()      // POV-Ray string representation of the normal
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Clearing the POV-Ray normal.
 *
 * \return void
 *
 * This function removes all components from the POV-Ray normal and resets all values to the
 * POV-Ray defaults.
 */
inline void Normal::reset()
{
   normal_.clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Normal operators */
//@{
std::ostream& operator<<( std::ostream& os, const Normal& normal );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
