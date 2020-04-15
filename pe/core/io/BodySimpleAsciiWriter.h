//=================================================================================================
/*!
 *  \file pe/core/io/BodySimpleAsciiWriter.h
 *  \brief Writer for simple rigid body Ascii files
 *
 *  Copyright (C) 2011 Tobias Preclik
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

#ifndef _PE_CORE_IO_BODYSIMPLEASCIIWRITER_H_
#define _PE_CORE_IO_BODYSIMPLEASCIIWRITER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/RigidBody.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Writes out descriptions of all rigid bodies in a simple Ascii format.
 * \ingroup core
 *
 * The BodySimpleAsciiWriter marshals all bodies in the simulation world. In parallel simulations
 * all non-local bodies are communicated to the root process. The root process will then output
 * all bodies in a simple textual description. The writer is intended to be used for debugging
 * purposes. In particular it can be used to compare time integrations of different collision
 * systems.
 */
class PE_PUBLIC BodySimpleAsciiWriter
{
private:
   //**Type definitions****************************************************************************
   struct RigidBodyCompare {
      bool operator()(const RigidBody::Parameters &a, const RigidBody::Parameters &b) const {
         return a.uid_ < b.uid_;
      }
   };
   //**********************************************************************************************

public:
   //**I/O functions*******************************************************************************
   /*!\name I/O functions */
   //@{
   static void writeFile( const char* filename );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
