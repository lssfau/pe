//=================================================================================================
/*!
 *  \file pe/povray/PointLight.h
 *  \brief Implementation of point light sources for the POV-Ray visualization
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

#ifndef _PE_POVRAY_POINTLIGHT_H_
#define _PE_POVRAY_POINTLIGHT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <sstream>
#include <pe/math/Vector3.h>
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


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A POV-Ray point light source.
 * \ingroup povray_lightsource
 *
 * The PointLight class acts as a veneer class for the LightSource class to create point
 * light sources for the POV-Ray visualization. A point light emits light of a specified
 * color uniformly in all directions. The following image gives an impression of a point
 * light:
 *
 * \image html pointlight.png
 * \image latex pointlight.eps "Example for a point light source" width=200pt
 *
 * For the individual configuration of a point light, the following light modifiers can be
 * used:
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * Any other modifier result in a compile time error!\n
 * The following code example demonstrates the construction of some point light sources. The
 * first and obligatory parameters specify the basic properties of a point light. Additionally,
 * up to five light modifiers can be used to tune the point light effect:

   \code
   // Creating a simple point light source at location (2,2,20) with a nearly white color.
   pe::povray::PointLight white( Vec3( 2.0, 2.0, 20.0 ),
                                 pe::povray::Color( 0.9, 0.9, 0.9 ) );

   // Creating a red point light source at location (-2,-2,20). Additionally, the fade distance
   // and power are specified individually.
   pe::povray::PointLight red( Vec3( -2.0, -2.0, 20.0 ),
                               pe::povray::Color( 1.0, 0.0, 0.0 ),
                               pe::povray::FadeDistance( 20.0 ),
                               pe::povray::FadePower( 2 ) );
   \endcode
 */
class PE_PUBLIC PointLight : public LightSource
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_3( FadeDistance, FadePower, Shadowless )  ValidTypes;  //!< Valid modifiers/transformations.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   PointLight( const Vec3& gpos, const Color& color );

   template< typename A >
   PointLight( const Vec3& gpos, const Color& color, const A& a );

   template< typename A, typename B >
   PointLight( const Vec3& gpos, const Color& color, const A& a, const B& b );

   template< typename A, typename B, typename C >
   PointLight( const Vec3& gpos, const Color& color, const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   PointLight( const Vec3& gpos, const Color& color, const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   PointLight( const Vec3& gpos, const Color& color, const A& a, const B& b, const C& c, const D& d, const E& e );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~PointLight();
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
/*!\brief Creating a point light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param a The additional light modifier.
 *
 * This constructor creates a point light using one additional light modifier. The center of
 * the point light is placed at the global position \a gpos and emits light of the specified
 * color uniformly in all directions. Valid modifiers for the additional light modifier \a a
 * are
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A >
PointLight::PointLight( const Vec3& gpos, const Color& color, const A& a )
   : LightSource()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n";

   a.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a point light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 *
 * This constructor creates a point light using two additional light modifiers. The center of
 * the point light is placed at the global position \a gpos and emits light of the specified
 * color uniformly in all directions. Valid modifiers for the additional light modifiers \a a
 * and \b b are
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A, typename B >
PointLight::PointLight( const Vec3& gpos, const Color& color, const A& a, const B& b )
   : LightSource()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, B );

   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n";

   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a point light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 * \param c The third additional light modifier.
 *
 * This constructor creates a point light using three additional light modifiers. The center of
 * the point light is placed at the global position \a gpos and emits light of the specified
 * color uniformly in all directions. Valid modifiers for the additional light modifiers \a a,
 * \a b and \a c are
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A, typename B, typename C >
PointLight::PointLight( const Vec3& gpos, const Color& color, const A& a, const B& b, const C& c )
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
       << "   " << color << "\n";

   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a point light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 * \param c The third additional light modifier.
 * \param d The fourth additional light modifier.
 *
 * This constructor creates a point light using four additional light modifiers. The center of
 * the point light is placed at the global position \a gpos and emits light of the specified
 * color uniformly in all directions. Valid modifiers for the additional light modifiers \a a,
 * \a b, \a c and \a d are
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A, typename B, typename C, typename D >
PointLight::PointLight( const Vec3& gpos, const Color& color, const A& a, const B& b,
                        const C& c, const D& d )
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
       << "   " << color << "\n";

   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a point light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param a The first additional light modifier.
 * \param b The second additional light modifier.
 * \param c The third additional light modifier.
 * \param d The fourth additional light modifier.
 * \param e The fifth additional light modifier.
 *
 * This constructor creates a point light using four additional light modifiers. The center of
 * the point light is placed at the global position \a gpos and emits light of the specified
 * color uniformly in all directions. Valid modifiers for the additional light modifiers \a a,
 * \a b, \a c, \a d and \a e are
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A, typename B, typename C, typename D, typename E >
PointLight::PointLight( const Vec3& gpos, const Color& color, const A& a, const B& b,
                        const C& c, const D& d, const E& e )
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
       << "   " << color << "\n";

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
/*!\brief Destructor for the PointLight class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline PointLight::~PointLight()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( LightSource, PointLight );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a light modifier to the POV-Ray point light.
 *
 * \param a The new light modifier.
 * \return void
 *
 * This function adds a new light modifier to the point light source. Valid modifiers are
 *  - FadeDistance
 *  - FadePower
 *  - Shadowless
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A >  // Type of the modifier/transformation
void PointLight::add( const A& a )
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
