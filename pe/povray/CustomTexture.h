//=================================================================================================
/*!
 *  \file pe/povray/CustomTexture.h
 *  \brief Implementation of a custom POV-Ray texture
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

#ifndef _PE_POVRAY_CUSTOMTEXTURE_H_
#define _PE_POVRAY_CUSTOMTEXTURE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <sstream>
#include <string>
#include <pe/povray/Texture.h>
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
/*!\brief A POV-Ray custom texture.
 * \ingroup povray_texture
 *
 * The CustomTexture class offers the possibility to manually create POV-Ray textures or to
 * use predefined POV-Ray texture identifiers. The following images illustrate some declared
 * textures:
 *
 * \image html declared.png
 * \image latex declared.eps "Examples for POV-Ray textures" width=820pt
 *
 * The code to create these textures is shown in the following:

   \code
   CustomTexture texture1( "T_Stone8 scale 10.0" );   (1)   // Declared in the "stones.inc" header
   CustomTexture texture2( "T_Stone9 scale 10.0" );   (2)   // Declared in the "stones.inc" header
   CustomTexture texture3( "T_Grnt10 scale  5.0" );   (3)   // Declared in the "stones.inc" header
   CustomTexture texture4( "T_Wood1  scale  3.0" );   (4)   // Declared in the "woods.inc" header
   CustomTexture texture5( "T_Silver_1A"         );   (5)   // Declared in the "metals.inc" header
   \endcode

 * \b Note: No checks are performed for both the POV-Ray texture identifiers and the user-specific
 * textures. Possible errors will only be detected by POV-Ray, not by the \b pe physics engine!
 */
class PE_PUBLIC CustomTexture : public Texture
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_1( Transformation )  ValidTypes;  //!< Valid texture types.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit CustomTexture( const char* const texture );
   explicit CustomTexture( const std::string& texture );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~CustomTexture();
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
/*!\brief Destructor for the CustomTexture class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline CustomTexture::~CustomTexture()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Texture, CustomTexture );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a transformation to the custom POV-Ray texture.
 *
 * \param a The new transformation (scale, translation, rotation, ...).
 * \return void
 */
template< typename A >
void CustomTexture::add( const A& a )
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << texture_;
   a.print( oss, "   ", true );

   oss.str().swap( texture_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
