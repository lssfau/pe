//=================================================================================================
/*!
 *  \file pe/povray/CustomLight.h
 *  \brief Implementation of a custom POV-Ray light source
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

#ifndef _PE_POVRAY_CUSTOMLIGHT_H_
#define _PE_POVRAY_CUSTOMLIGHT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <string>
#include <pe/povray/LightSource.h>
#include <pe/povray/Transformation.h>
#include <pe/util/constraints/SameSize.h>
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
/*!\brief A POV-Ray custom light source.
 * \ingroup povray_lightsource
 *
 * The CustomLight class offers the possibility to manually create POV-Ray light sources. The
 * following example illustrates the setup of a custom light source:

   \code
   pe::povray::CustomLight( "<2,20,2> color rgb <0.9,0.9,0.9>" );  // User-specific light source
   \endcode

 * This setup of a light sources uses a custom POV-Ray light source string in order to create
 * a white point light source at the location (2,2,20). Note that since the given light source
 * is directly used in the POV-Ray files, points and vectors have to be directly specified in
 * the left-handed POV-Ray coordinate system! Also note that no checks are performed for the
 * user-specific light source setup. Possible errors will only be detected by POV-Ray, not the
 * \b pe physics engine!
 */
class PE_PUBLIC CustomLight : public LightSource
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_1( Transformation )  ValidTypes;  //!< Valid texture types.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit CustomLight( const char* const lightsource );
   explicit CustomLight( const std::string& lightsource );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~CustomLight();
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
/*!\brief Destructor for the CustomLight class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline CustomLight::~CustomLight()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( LightSource, CustomLight );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a transformation to the custom POV-Ray light source.
 *
 * \param a The new transformation (scale, translation, rotation, ...).
 * \return void
 */
template< typename A >
void CustomLight::add( const A& a )
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
