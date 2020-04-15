//=================================================================================================
/*!
 *  \file pe/povray/AreaLight.h
 *  \brief Implementation of area light sources for the POV-Ray visualization
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

#ifndef _PE_POVRAY_AREALIGHT_H_
#define _PE_POVRAY_AREALIGHT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <sstream>
#include <pe/math/Vector3.h>
#include <pe/povray/Adaptive.h>
#include <pe/povray/Color.h>
#include <pe/povray/FadeDistance.h>
#include <pe/povray/FadePower.h>
#include <pe/povray/LightSource.h>
#include <pe/povray/Shadowless.h>
#include <pe/system/Precision.h>
#include <pe/util/constraints/SameSize.h>
#include <pe/util/constraints/SameType.h>
#include <pe/util/constraints/TypeRestriction.h>
#include <pe/util/TypeList.h>
#include <pe/util/Types.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A POV-Ray area light source.
 * \ingroup povray_lightsource
 *
 * The AreaLight class represents area light sources for the POV-Ray visualization. Instead of
 * a single flat panel light of rectangular shape, an area light is represented by a one- or
 * two-dimenstional array of individual point lights in order to avoid complex calculations.
 * Each single point light source of the area light emits light of a specified color uniformly
 * in all directions.
 *
 * \image html arealight2.png
 * \image latex arealight2.eps "Layout of an area light source" width=400pt
 *
 * In constrast to a single point light source, area lights are able to cast
 * soft shadows because an object can partially block their light (whereas a single point light
 * source iseither totally blocked or not blocked). The following images give an impression of
 * an area light source:
 *
 * \image html arealight.png
 * \image latex arealight.eps "Example for an area light source" width=200pt
 *
 * For the individual configuration of an area light, the following light modifiers can be used:
 *  - Adaptive
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * Any other modifier result in a compile time error!\n
 * The following code example demonstrates the construction of an area light source. The first
 * and obligatory parameters specify the basic properties of an area light. Additionally, up to
 * five light modifiers can be used to tune the area light effect:

   \code
   // Creating a simple area light source at location (2,2,20) with a nearly white color.
   // The area light is spanned by the two vectors (10,0,0) and (0,10,0) at consists of
   // 6 x 6 point light sources.
   pe::povray::AreaLight white( Vec3( 2.0, 2.0, 20.0 ),
                                pe::povray::Color( 0.9, 0.9, 0.9 ),
                                Vec3( 10.0, 0.0, 0.0 ), Vec3( 0.0, 10.0, 0.0 ),
                                6, 6 );

   // Creating a red area light source at location (-2,-2,20) consisting of 5 x 5 point light
   // sources in the area spanned by (3,0,0) and (0,3,0). Additionally, adaptive sampling is
   // set to level 1 and the fade distance and power are specified individually.
   pe::povray::AreaLight red( Vec3( -2.0, -2.0, 20.0 ),
                              pe::povray::Color( 1.0, 0.0, 0.0 ),
                              Vec3( 3.0, 0.0, 0.0 ), Vec3( 0.0, 3.0, 0.0 ),
                              5, 5,
                              pe::povray::Adaptive( 1 ),
                              pe::povray::FadeDistance( 20.0 ),
                              pe::povray::FadePower( 2 ) );
   \endcode
 */
class PE_PUBLIC AreaLight : public LightSource
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_4( Adaptive, FadeDistance,
                          FadePower, Shadowless )  ValidTypes;  //!< Valid modifiers/transformations.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
              size_t m, size_t n );

   template< typename A >
   AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
              size_t m, size_t n, const A& a );

   template< typename A, typename B >
   AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
              size_t m, size_t n, const A& a, const B& b );

   template< typename A, typename B, typename C >
   AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
              size_t m, size_t n, const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
              size_t m, size_t n, const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
              size_t m, size_t n, const A& a, const B& b, const C& c, const D& d, const E& e );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~AreaLight();
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
/*!\brief Creating an area light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param u First axis of the area light source.
 * \param v Second axis of the area light source.
 * \param m Number of point light sources along the first axis.
 * \param n Number of point light sources along the second axis.
 * \param a The additional light modifier.
 *
 * This constructor creates a POV-Ray area light source using one additional light modifier.
 * The center of the area light source is placed at \a gpos. The \a m times \a n point light
 * sources are placed on the plane spanned by the two specified axes \a u and \a v. Valid
 * modifiers for the additional light modifier \a a are
 *  - Adaptive
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A >
AreaLight::AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
                      size_t m, size_t n, const A& a )
   : LightSource()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n"
       << "   area_light\n"
       << "   " << u << ", " << v << ", " << m << ", " << n << "\n";

   a.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating an area light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param u First axis of the area light source.
 * \param v Second axis of the area light source.
 * \param m Number of point light sources along the first axis.
 * \param n Number of point light sources along the second axis.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 *
 * This constructor creates a POV-Ray area light source using two additional light modifiers.
 * The center of the area light source is placed at \a gpos. The \a m times \a n point light
 * sources are placed on the plane spanned by the two specified axes \a u and \a v. Valid
 * modifiers for the additional light modifiers \a a and \a b are
 *  - Adaptive
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * In case either \a A or \a B is not a valid light modifier or in case \a A and \a B happen to
 * be the same modifier, a compile time error is created!
 */
template< typename A, typename B >
AreaLight::AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
                      size_t m, size_t n, const A& a, const B& b )
   : LightSource()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, B );

   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n"
       << "   area_light\n"
       << "   " << u << ", " << v << ", " << m << ", " << n << "\n";

   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating an area light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param u First axis of the area light source.
 * \param v Second axis of the area light source.
 * \param m Number of point light sources along the first axis.
 * \param n Number of point light sources along the second axis.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 * \param c The third additional light modifier.
 *
 * This constructor creates a POV-Ray area light source using three additional light modifiers.
 * The center of the area light source is placed at \a gpos. The \a m times \a n point light
 * sources are placed on the plane spanned by the two specified axes \a u and \a v. Valid
 * modifiers for the additional light modifiers \a a, \a b and \a c are
 *  - Adaptive
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * In case at least one of the given parameters is not a valid light modifier or in case two
 * of these modifiers have the same type, a compile time error is created!
 */
template< typename A, typename B, typename C >
AreaLight::AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
                      size_t m, size_t n, const A& a, const B& b, const C& c )
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
       << "   area_light\n"
       << "   " << u << ", " << v << ", " << m << ", " << n << "\n";

   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating an area light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param u First axis of the area light source.
 * \param v Second axis of the area light source.
 * \param m Number of point light sources along the first axis.
 * \param n Number of point light sources along the second axis.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 * \param c The third additional light modifier.
 * \param d The fourth additional light modifier.
 *
 * This constructor creates a POV-Ray area light source using three additional light modifiers.
 * The center of the area light source is placed at \a gpos. The \a m times \a n point light
 * sources are placed on the plane spanned by the two specified axes \a u and \a v. Valid
 * modifiers for the additional light modifiers \a a, \a b, \a c and \a d are
 *  - Adaptive
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * In case at least one of the given parameters is not a valid light modifier or in case two
 * of these modifiers have the same type, a compile time error is created!
 */
template< typename A, typename B, typename C, typename D >
AreaLight::AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
                      size_t m, size_t n, const A& a, const B& b, const C& c,
                      const D& d )
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
       << "   area_light\n"
       << "   " << u << ", " << v << ", " << m << ", " << n << "\n";

   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating an area light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param u First axis of the area light source.
 * \param v Second axis of the area light source.
 * \param m Number of point light sources along the first axis.
 * \param n Number of point light sources along the second axis.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 * \param c The third additional light modifier.
 * \param d The fourth additional light modifier.
 * \param e The fifth additional light modifier.
 *
 * This constructor creates a POV-Ray area light source using three additional light modifiers.
 * The center of the area light source is placed at \a gpos. The \a m times \a n point light
 * sources are placed on the plane spanned by the two specified axes \a u and \a v. Valid
 * modifiers for the additional light modifiers \a a, \a b, \a c, \a d and \a e are
 *  - Adaptive
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * In case at least one of the given parameters is not a valid light modifier or in case two
 * of these modifiers have the same type, a compile time error is created!
 */
template< typename A, typename B, typename C, typename D, typename E >
AreaLight::AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
                      size_t m, size_t n, const A& a, const B& b, const C& c, const D& d, const E& e )
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
       << "   area_light\n"
       << "   " << u << ", " << v << ", " << m << ", " << n << "\n";

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
/*!\brief Destructor for the AreaLight class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline AreaLight::~AreaLight()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( LightSource, AreaLight );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a light modifier to the POV-Ray area light.
 *
 * \param a The new light modifier.
 * \return void
 *
 * This function adds a new light modifier to the area light source. Valid modifiers are
 *  - Adaptive
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A >  // Type of the modifier/transformation
void AreaLight::add( const A& a )
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
