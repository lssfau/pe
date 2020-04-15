//=================================================================================================
/*!
 *  \file pe/core/BodyBinaryReader.h
 *  \brief Reader for rigid body binary parameter files
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

#ifndef _PE_CORE_BODYBINARYREADER_H_
#define _PE_CORE_BODYBINARYREADER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/TypeConvertingRecvBuffer.h>
#include <pe/core/rigidbody/Union.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rigid body binary parameter file reader.
 * \ingroup core
 *
 * Reads in snapshots of the simulation world written by the BodyBinaryWriter. The BodyBinaryReader
 * uses the MPI I/O functionality to read in the binary parameter file in parallel. The current
 * simulation world is replaced by the simulation recorded in the parameter file.
 */
class PE_PUBLIC BodyBinaryReader
{
public:
   //**Type definitions****************************************************************************
   typedef TypeConvertingRecvBuffer<NtohEndiannessConversion> Buffer;
   //**********************************************************************************************

   //**I/O functions*******************************************************************************
   /*!\name I/O functions */
   //@{
   void readFile( const char* filename );
   //@}
   //**********************************************************************************************

private:
   //**I/O functions*******************************************************************************
   /*!\name I/O functions */
   //@{
   UnionID instantiateUnion( const Union::Parameters& objparam, bool global, bool reassignSystemID );
   void unmarshalAll( Buffer& buffer, bool global = false, bool reassignSystemID = false );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Buffer buffer_;
   Buffer header_;
   Buffer globals_;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
