//=================================================================================================
/*!
 *  \file pe/povray/CustomFinish.h
 *  \brief Implementation of a custom POV-Ray finish
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

#ifndef _PE_POVRAY_CUSTOMFINISH_H_
#define _PE_POVRAY_CUSTOMFINISH_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <string>
#include <pe/povray/Finish.h>
#include <pe/util/constraints/SameSize.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A POV-Ray custom finish.
 * \ingroup povray_finish
 *
 * The CustomFinish class offers the possibility to manually create a POV-Ray finish or to
 * use predefined POV-Ray finish identifiers. The following examples illustrates the setup
 * of two custom finishes:

   \code
   pe::povray::CustomFinish( "F_Glass1" );                  (1)   // Declared in the "glass.inc" header
   pe::povray::CustomFinish( "ambient 0.1 diffuse 0.6" );   (2)   // User-specific finish
   \endcode

 * The first finish uses a predefined POV-Ray finish from the \a glass.inc header file. The
 * second finish is a user-specific finish specification. Note however that no checks are
 * performed for both the POV-Ray finish identifiers and the user-specific finishes. Possible
 * errors will only be detected by POV-Ray, not by the \b pe physics engine!
 */
class PE_PUBLIC CustomFinish : public Finish
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit CustomFinish( const char* const finish );
   explicit CustomFinish( const std::string& finish );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~CustomFinish();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the CustomFinish class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline CustomFinish::~CustomFinish()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Finish, CustomFinish );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
