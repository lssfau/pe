//=================================================================================================
/*!
 *  \file pe/povray/SpotLight.h
 *  \brief Implementation of spotlights for the POV-Ray visualization
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

#ifndef _PE_POVRAY_SPOTLIGHT_H_
#define _PE_POVRAY_SPOTLIGHT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <sstream>
#include <pe/math/Vector3.h>
#include <pe/povray/Color.h>
#include <pe/povray/FadeDistance.h>
#include <pe/povray/FadePower.h>
#include <pe/povray/Falloff.h>
#include <pe/povray/LightSource.h>
#include <pe/povray/PointAt.h>
#include <pe/povray/Radius.h>
#include <pe/povray/Shadowless.h>
#include <pe/povray/Tightness.h>
#include <pe/system/Precision.h>
#include <pe/util/constraints/SameSize.h>
#include <pe/util/constraints/SameType.h>
#include <pe/util/constraints/TypeRestriction.h>
#include <pe/util/TypeList.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A POV-Ray spotlight.
 * \ingroup povray_lightsource
 *
 * Normally light radiates outward equally in all directions from a specified light source.
 * Spotlights however offer the possibility to specify directed light sources. Spotlights can
 * be used to create a cone of light that is bright in the center and falls of to darkness in
 * a soft fringe effect at the edge. The following illustration shows how a spotlight works:
 *
 * \image html spotlight2.png
 * \image latex spotlight2.eps "Structure of a spotlight" width=330pt
 *
 * The direction of the light is specified by both the global position of the light source
 * and its focus point. Per default, the focus point is set to (0,0,0), but it can be set
 * individually by the PointAt light modifier. The Radius and the Falloff modifiers configure
 * the light intensity: the radius value specifies the angle (in radian measure) between the
 * edge of the bright, inner cone and the center line, the falloff value on the other hand
 * specifies the overall size of the outer light cone. The light inside the inner cone is of
 * uniform intensity, whereas the light between the inner and outer cone tapers off to zero.
 * The following image gives an impression of a POV-Ray spotlight:
 *
 * \image html spotlight.png
 * \image latex spotlight.eps "Example for a spotlight" width=200pt
 *
 * For the individual configuration of a spotlight, the following light modifers can be used:
 *  - PointAt
 *  - Falloff
 *  - Radius
 *  - Tightness
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * Any other modifier result in a compile time error!\n
 * The following code example demonstrates the construction of a spotlight. The first and
 * obligatory parameters specify the center and color of the spotlight. Additionally, up to
 * five light modifiers can be used to tune the spotlight effect:

   \code
   // Creating a simple spotlight source at location (2,2,20) with a nearly white color.
   pe::povray::SpotLight white( Vec3( 2.0, 2.0, 20.0 ),
                                pe::povray::Color( 0.9, 0.9, 0.9 ) );

   // Creating a red spotlight at location (-2,-2,20) pointing towards (-2,-2,0). Additionally,
   // the spotlight properties falloff, radius and tightness are specified individually.
   pe::povray::SpotLight red( Vec3( -2.0, -2.0, 20.0 ),
                              pe::povray::Color( 1.0, 0.0, 0.0 ),
                              pe::povray::PointAt( -1.0, -1.0, 0.0 ),
                              pe::povray::Falloff( PI/4 ),
                              pe::povray::Radius( PI/6 ),
                              pe::povray::Tightness( 2 ) );
   \endcode
 */
class PE_PUBLIC SpotLight : public LightSource
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_7( PointAt, Falloff, Radius, Tightness,
                          FadeDistance, FadePower, Shadowless )  ValidTypes;  //!< Valid light modifiers.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   SpotLight( const Vec3& gpos, const Color& color );

   template< typename A >
   SpotLight( const Vec3& gpos, const Color& color, const A& a );

   template< typename A, typename B >
   SpotLight( const Vec3& gpos, const Color& color, const A& a, const B& b );

   template< typename A, typename B, typename C >
   SpotLight( const Vec3& gpos, const Color& color, const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   SpotLight( const Vec3& gpos, const Color& color,
              const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   SpotLight( const Vec3& gpos, const Color& color,
              const A& a, const B& b, const C& c, const D& d, const E& e );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~SpotLight();
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
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating a spotlight source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param a The additional light modifier.
 *
 * This constructor creates a POV-Ray spotlight using one additional light modifier. The
 * spotlight is placed at the global position \a gpos and emits light of the color \a color.
 * Valid modifiers for the additional light modifier \a a are
 *  - PointAt
 *  - Falloff
 *  - Radius
 *  - Tightness
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A >
SpotLight::SpotLight( const Vec3& gpos, const Color& color, const A& a )
   : LightSource()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n"
       << "   spotlight\n";

   a.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a spotlight source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 *
 * This constructor creates a POV-Ray spotlight using two additional light modifiers. The
 * spotlight is placed at the global position \a gpos and emits light of the color \a color.
 * Valid modifiers for the additional light modifiers \a a and \a b are
 *  - PointAt
 *  - Falloff
 *  - Radius
 *  - Tightness
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * In case either \a A or \a B is not a valid light modifier or in case \a A and \a B happen to
 * be the same modifier, a compile time error is created!
 */
template< typename A, typename B >
SpotLight::SpotLight( const Vec3& gpos, const Color& color, const A& a, const B& b )
   : LightSource()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, B );

   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n"
       << "   spotlight\n";

   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a spotlight source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 * \param c The third additional light modifier.
 *
 * This constructor creates a POV-Ray spotlight using three additional light modifiers. The
 * spotlight is placed at the global position \a gpos and emits light of the color \a color.
 * Valid modifiers for the additional light modifiers \a a, \a b and \a c are
 *  - PointAt
 *  - Falloff
 *  - Radius
 *  - Tightness
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * In case at least one of the given parameters is not a valid light modifier or in case two
 * of these modifiers have the same type, a compile time error is created!
 */
template< typename A, typename B, typename C >
SpotLight::SpotLight( const Vec3& gpos, const Color& color, const A& a, const B& b, const C& c )
   : LightSource()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, B );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, C );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, C );

   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n"
       << "   spotlight\n";

   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a spotlight source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 * \param c The third additional light modifier.
 * \param d The fourth additional light modifier.
 *
 * This constructor creates a POV-Ray spotlight using four additional light modifiers. The
 * spotlight is placed at the global position \a gpos and emits light of the color \a color.
 * Valid modifiers for the additional light modifiers \a a, \a b, \a c and \a d are
 *  - PointAt
 *  - Falloff
 *  - Radius
 *  - Tightness
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * In case at least one of the given parameters is not a valid light modifier or in case two
 * of these modifiers have the same type, a compile time error is created!
 */
template< typename A, typename B, typename C, typename D >
SpotLight::SpotLight( const Vec3& gpos, const Color& color,
                      const A& a, const B& b, const C& c, const D& d )
   : LightSource()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, B );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, C );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, D );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, C );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, D );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( C, D );

   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n"
       << "   spotlight\n";

   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a spotlight source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 * \param c The third additional light modifier.
 * \param d The fourth additional light modifier.
 * \param e The fifth additional light modifier.
 *
 * This constructor creates a POV-Ray spotlight using five additional light modifiers. The
 * spotlight is placed at the global position \a gpos and emits light of the color \a color.
 * Valid modifiers for the additional light modifiers \a a, \a b, \a c, \a d and \a e are
 *  - PointAt
 *  - Falloff
 *  - Radius
 *  - Tightness
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * In case at least one of the given parameters is not a valid light modifier or in case two
 * of these modifiers have the same type, a compile time error is created!
 */
template< typename A, typename B, typename C, typename D, typename E >
SpotLight::SpotLight( const Vec3& gpos, const Color& color,
                      const A& a, const B& b, const C& c, const D& d, const E& e )
   : LightSource()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( E, ValidTypes );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, B );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, C );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, D );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, E );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, C );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, D );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, E );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( C, D );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( C, E );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( D, E );

   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n"
       << "   spotlight\n";

   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );
   e.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the SpotLight class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline SpotLight::~SpotLight()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( LightSource, SpotLight );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a light modifier to the POV-Ray spotlight.
 *
 * \param a The new light modifier.
 * \return void
 *
 * This function adds a new light modifier to the spotlight source. Valid modifiers are
 *  - PointAt
 *  - Falloff
 *  - Radius
 *  - Tightness
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A >  // Type of the modifier/transformation
void SpotLight::add( const A& a )
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << lightsource_;
   a.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
