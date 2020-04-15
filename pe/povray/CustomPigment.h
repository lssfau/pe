//=================================================================================================
/*!
 *  \file pe/povray/CustomPigment.h
 *  \brief Implementation of a custom POV-Ray pigment
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

#ifndef _PE_POVRAY_CUSTOMPIGMENT_H_
#define _PE_POVRAY_CUSTOMPIGMENT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <string>
#include <pe/povray/Pigment.h>
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
/*!\brief A POV-Ray custom pigment.
 * \ingroup povray_pigment
 *
 * The CustomPigment class offers the possibility to manually create a POV-Ray pigment or to
 * use predefined POV-Ray pigment identifiers. The following examples illustrates the setup
 * of two custom pigments:

   \code
   pe::povray::CustomPigment( "P_WoodGrain1A" );               (1)   // Declared in the "woods.inc" header
   pe::povray::CustomPigment( "checker color Black White" );   (2)   // User-specific pigment
   \endcode
 *
 * The first pigment uses a predefined POV-Ray pigment from the \a woods.inc header file. The
 * second pigment is a user-specific pigment specification. Note however that no checks are
 * performed for both the POV-Ray pigment identifiers and the user-specific pigments. Possible
 * errors will only be detected by POV-Ray, not by the \b pe physics engine!
 */
class PE_PUBLIC CustomPigment : public Pigment
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_1( Transformation )  ValidTypes;  //!< Valid texture types.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit CustomPigment( const char* const pigment );
   explicit CustomPigment( const std::string& pigment );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~CustomPigment();
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
/*!\brief Destructor for the CustomPigment class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline CustomPigment::~CustomPigment()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Pigment, CustomPigment );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a transformation to the custom POV-Ray pigment.
 *
 * \param a The new transformation (scale, translation, rotation, ...).
 * \return void
 */
template< typename A >
void CustomPigment::add( const A& a )
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << pigment_;
   a.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
