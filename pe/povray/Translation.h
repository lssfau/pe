//=================================================================================================
/*!
 *  \file pe/povray/Translation.h
 *  \brief POV-Ray transformation for translations
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

#ifndef _PE_POVRAY_TRANSLATION_H_
#define _PE_POVRAY_TRANSLATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/math/Vector3.h>
#include <pe/povray/Transformation.h>
#include <pe/system/Precision.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A POV-Ray translation transformation.
 * \ingroup povray_transformation
 *
 * The Translation class is a POV-Ray transformation for translation operations. There are two
 * ways to construct a translation:
 *
 * -# Translation::Translation( real x, real y, real z );
 * -# Translation::Translation( const Vec3& translation );
 *
 * A translation can be used to shift a texture, pigment or normal to a different location. The
 * order of the arguments doesn't matter (in contrast to the arguments order of a Rotation).
 */
class PE_PUBLIC Translation : public Transformation
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Translation( real x, real y, real z );
   explicit Translation( const Vec3& translation );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os,                  bool newline ) const;
   void print( std::ostream& os, const char* tab, bool newline ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 translation_;  //!< The translation vector.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Translation operators */
//@{
std::ostream& operator<<( std::ostream& os, const Translation& translation );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
