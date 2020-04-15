//=================================================================================================
/*!
 *  \file pe/core/BodyBinaryWriter.h
 *  \brief Writer for rigid body binary parameter files
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

#ifndef _PE_CORE_BODYBINARYWRITER_H_
#define _PE_CORE_BODYBINARYWRITER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/MPISettings.h>
#include <vector>
#include <list>
#include <pe/core/TypeConvertingSendBuffer.h>
#include <pe/core/World.h>
#include <pe/core/Marshalling.h>
#include <boost/numeric/conversion/cast.hpp>
#include <pe/util/logging/DebugSection.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rigid body binary parameter file writer.
 * \ingroup core
 *
 * The BodyBinaryWriter marshalls all global and local bodies and uses MPI I/O functionality
 * to write out a snapshot of the world to a file in parallel. Dynamic type conversions are
 * supported such that real data is e.g. automatically converted to lower precision sufficient for
 * visualization purposes. Snapshots can be parsed by the BodyBinaryReader. Writing can be
 * performed asynchronously. Deactivating type conversions the BodyBinaryWriter can be used for
 * checkpointing.
 *
 * The BodyBinaryWriter is currently limited in that it does not cast integral data types to
 * a specified length. Even though the BodyBinaryReader asserts that the type sizes match, it
 * bails out if the integral type sizes mismatch.
 */
class PE_PUBLIC BodyBinaryWriter
{
public:
   //**Type definitions****************************************************************************
   typedef TypeConvertingSendBuffer<HtonEndiannessConversion> Buffer;
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   BodyBinaryWriter();
   BodyBinaryWriter( const BodyBinaryWriter& o );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~BodyBinaryWriter();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   /*!\name Copy assignment operator */
   //@{
   BodyBinaryWriter& operator=( const BodyBinaryWriter& c );
   //@}
   //**********************************************************************************************

   //**I/O functions*******************************************************************************
   /*!\name I/O functions */
   //@{
   void writeFile( const char* filename );
   void writeFileAsync( const char* filename );
   void wait();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void setFloatingPointSize( int fpSize );
   //@}
   //**********************************************************************************************


private:
   //**I/O functions*******************************************************************************
   /*!\name I/O functions */
   //@{
   template<typename T>
   size_t marshalAllPrimitives( Buffer& buffer, ConstWorldID world, bool global = false );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Buffer buffer_;
   Buffer header_;
   Buffer globals_;
#if HAVE_MPI
   std::list<MPI_Request> requests_;
   MPI_File fh_;
#else
   std::ofstream fh_;
#endif
   bool fhOpen_;
   int fpSize_;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructs the binary writer.
 */
inline BodyBinaryWriter::BodyBinaryWriter() : fhOpen_( false ), fpSize_( 0 ) {
}
//*************************************************************************************************


//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructs the binary writer after waiting for all operations to finish and closing all file handles.
 */
inline BodyBinaryWriter::~BodyBinaryWriter() {
   wait();
}
//*************************************************************************************************




//=================================================================================================
//
//  I/O FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Writes out a binary rigid body parameter file and waits for the operation to finish.
 * \param filename The filename of the parameter file to write to.
 * \return void
 */
inline void BodyBinaryWriter::writeFile( const char* filename ) {
   writeFileAsync( filename );
   wait();
}
//*************************************************************************************************




//*************************************************************************************************
/*!\brief Wait for output operations to finish and close file handle.
 * \return void
 */
inline void BodyBinaryWriter::wait() {
   if( fhOpen_ ) {
#if HAVE_MPI
      MPI_Status status;
      while( !requests_.empty() ) {
         MPI_Wait( &requests_.front(), &status );
         requests_.pop_front();
      }
      MPI_File_close( &fh_ );
#else
      fh_.close();
#endif
      fhOpen_ = false;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshals all geometric primitives of the same type.
 * \param buffer The buffer where to marshal to.
 * \param world The world containing the bodies to marshal.
 * \param global If set global bodies are written, if unset local bodies are written.
 * \return The number of primitives marshaled.
 */
template<typename T>
inline size_t BodyBinaryWriter::marshalAllPrimitives( Buffer& buffer, ConstWorldID world, bool global ) {
   marshal( buffer, static_cast<byte>( geomType<T>() ) );
   const World::Bodies::ConstCastIterator<T> end( world->end<T>() );

   size_t n = 0;
   for( World::Bodies::ConstCastIterator<T> obj=world->begin<T>(); obj!=end; ++obj )
      if( (*obj)->isGlobal() == global && !(*obj)->isRemote() )
         ++n;
   marshal( buffer, boost::numeric_cast<uint32_t>( n ) );

   for( World::Bodies::ConstCastIterator<T> obj=world->begin<T>(); obj!=end; ++obj )
      if( (*obj)->isGlobal() == global && !(*obj)->isRemote() )
         marshal( buffer, *(*obj) );

   pe_LOG_DEBUG_SECTION( log ) {
      if( n != 0 )
      {
         const char* const specifier = (global ? " global" : " local");
         log << "On rank " << MPISettings::rank() << " marshaled " << n << specifier << " objects.\n";
      }
   }

   return n;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================


//*************************************************************************************************
/*!\brief Determines the amount of data written when buffering floating point values.
 *
 * \param fpSize The exponent of the power of two of the output floating point type or 0 for no type conversion.
 * \return void
 *
 * Floating point values are dynamically converted when buffering them. The size of the output
 * floating point type is specified with this function. The bytes written when buffering a floating
 * point value are \f$2^{fpSize}\f$ unless fpSize is 0 indicating that no conversion should take
 * place. Currently values 0 through 3 are supported.
 */
inline void BodyBinaryWriter::setFloatingPointSize( int fpSize )
{
   fpSize_ = fpSize;
}
//*************************************************************************************************

} // namespace pe

#endif
