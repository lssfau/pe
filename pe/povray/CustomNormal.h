//=================================================================================================
/*!
 *  \file pe/povray/CustomNormal.h
 *  \brief Implementation of a custom POV-Ray normal
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

#ifndef _PE_POVRAY_CUSTOMNORMAL_H_
#define _PE_POVRAY_CUSTOMNORMAL_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <string>
#include <pe/povray/Normal.h>
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
/*!\brief A POV-Ray custom normal.
 * \ingroup povray_normal
 *
 * The CustomNormal class offers the possibility to manually create POV-Ray normals or to
 * use predefined POV-Ray normal identifiers. The following example illustrates the setup
 * of a custom normal:

   \code
   pe::povray::CustomNormal( "ripples 2.2 translate <0,1,0>" );  // User-specific normal
   \endcode

 * This normal uses a user-specific normal specification. Note however that no checks are
 * performed for both the POV-Ray normal identifiers and the user-specific normals. Possible
 * errors will only be detected by POV-Ray, not the \b pe physics engine!
 */
class PE_PUBLIC CustomNormal : public Normal
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_1( Transformation )  ValidTypes;  //!< Valid texture types.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit CustomNormal( const char* const normal );
   explicit CustomNormal( const std::string& normal );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~CustomNormal();
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
/*!\brief Destructor for the CustomNormal class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline CustomNormal::~CustomNormal()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Normal, CustomNormal );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a transformation to the custom POV-Ray normal.
 *
 * \param a The new transformation (scale, translation, rotation, ...).
 * \return void
 */
template< typename A >
void CustomNormal::add( const A& a )
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << normal_;
   a.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
