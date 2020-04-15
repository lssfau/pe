//=================================================================================================
/*!
 *  \file pe/core/SendBuffer.h
 *  \brief Implementation of a send buffer.
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2011 Tobias Preclik
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

#ifndef _PE_CORE_SENDBUFFER_H_
#define _PE_CORE_SENDBUFFER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Builtin.h>
#include <pe/util/Byte.h>
#include <pe/util/Endianness.h>
#include <pe/util/Policies.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a send buffer.
 * \ingroup core
 *
 * The SendBuffer class can be used to assemble a buffer from different data types. The send buffer
 * supports endianness conversion if desired. For example it can be used for mixed-type MPI
 * communications. The following example gives an impression of the usage of the SendBuffer class:

   \code
   // Adding a single double value to a send buffer
   SendBuffer<> buffer;
   double d( 2.1 );
   buffer << d ;

   // Sending an MPI message to process 0 with the contents of the send buffer
   MPI_Send( buffer, buffer.size(), MPI_BYTE, 0, 0, MPI_COMM_WORLD );
   \endcode

 * Note that the order of data values in the send buffer is depending on the order the
 * elements are added to the buffer. The same order has to be used during the extraction of the
 * data elements. See also the RecvBuffer class description for the receiver side.
 */
template< typename E = NoEndiannessConversion  // Endianness conversion policy
        , typename G = OptimalGrowth >         // Growth policy
class SendBuffer
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline SendBuffer( size_t initCapacity = 0 );
            inline SendBuffer( const SendBuffer& sb );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~SendBuffer();
   //@}
   //**********************************************************************************************

   //**Assignment operator*************************************************************************
   /*!\name Assignment operator */
   //@{
   SendBuffer& operator=( const SendBuffer& sb );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline size_t maxSize () const;
   inline size_t size    () const;
   inline size_t capacity() const;
   inline bool   isEmpty () const;
   //@}
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   template< typename V > SendBuffer& operator<<( V value );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline byte* ptr    () const;
   inline void  reserve( size_t newCapacity );
   inline void  clear  ();
   //@}
   //**********************************************************************************************

protected:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   void extendMemory( size_t newCapacity );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   byte* begin_;  //!< Pointer to the first byte of the send buffer.
   byte* cur_;    //!< Pointer to the current/last byte of the send buffer.
   byte* end_;    //!< Pointer to the end of the storage of the send buffer.
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
/*!\brief Standard constructor for SendBuffer.
 *
 * \param initCapacity The initial capacity of the send buffer.
 *
 * The default initial capacity of the send buffer is specified by the selected growth policy.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline SendBuffer<E,G>::SendBuffer( size_t initCapacity )
   : begin_( new byte[initCapacity] )  // Pointer to the first byte
   , cur_  ( begin_ )                  // Pointer to the current/last byte
   , end_  ( begin_ )                  // Pointer to the end of the storage
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy constructor for SendBuffer.
 *
 * \param sb The send buffer to be copied.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline SendBuffer<E,G>::SendBuffer( const SendBuffer& sb )
   : begin_( new byte[sb.size()] )  // Pointer to the first byte
   , cur_  ( begin_+sb.size() )     // Pointer to the current/last byte
   , end_  ( cur_ )                 // Pointer to the end of the storage
{
   for( size_t i=0; i<sb.size(); ++i )
      begin_[i] = sb.begin_[i];
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for SendBuffer.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline SendBuffer<E,G>::~SendBuffer()
{
   delete [] begin_;
}
//*************************************************************************************************




//=================================================================================================
//
//  ASSIGNMENT OPERATOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Copy assignment operator for SendBuffer.
 *
 * \param sb The send buffer to be copied.
 * \return Reference to the assigned send buffer.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
SendBuffer<E,G>& SendBuffer<E,G>::operator=( const SendBuffer& sb )
{
   if( &sb == this ) return *this;

   if( sb.size() > capacity() ) {
      byte* newBegin( new byte[sb.size()] );
      end_ = std::copy( sb.begin_, sb.cur_, newBegin );
      std::swap( begin_, newBegin );
      delete [] newBegin;

      cur_ = end_;
   }
   else {
      cur_ = std::copy( sb.begin_, sb.end_, begin_ );
   }

   return *this;
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the maximum possible size of the send buffer.
 *
 * \return The maximum possible size.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline size_t SendBuffer<E,G>::maxSize() const
{
   return size_t(-1);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current size of the send buffer.
 *
 * \return The current size.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline size_t SendBuffer<E,G>::size() const
{
   return cur_ - begin_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the capacity of the send buffer.
 *
 * \return The capacity.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline size_t SendBuffer<E,G>::capacity() const
{
   return end_ - begin_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns \a true if the send buffer is empty.
 *
 * \return \a true if the send buffer is empty, \a false if it is not.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline bool SendBuffer<E,G>::isEmpty() const
{
   return begin_ == cur_;
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adds a built-in data value to the send buffer.
 *
 * \return Reference to the send buffer.
 *
 * This function adds one data value of built-in data type to the send buffer.
 *
 * \b Note: This operator may only be used for built-in data types. The attempt to use
 * a user-defined data type results in a compile time error!
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
template< typename V >  // Type of the built-in data value
SendBuffer<E,G>& SendBuffer<E,G>::operator<<( V value )
{
   pe_CONSTRAINT_MUST_BE_BUILTIN_TYPE( V );

   const size_t count( sizeof(V) );
   const size_t rest ( end_ - cur_ );

   // Checking the size of the remaining memory
   if( rest < count ) {
      extendMemory( size() + count );
   }

   // Adding the data value
   V* const tmp( reinterpret_cast<V*>( cur_ ) );
   *tmp  = E()( value );
   cur_ += count;

   // Invariants check
   pe_INTERNAL_ASSERT( cur_ <= end_, "Invalid pointer calculations" );

   return *this;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a pointer to the first byte of the send buffer.
 *
 * \return Pointer to the first byte of the send buffer.
 *
 * This utility function enables the SendBuffer to be used directly as send buffer in MPI
 * send functions:

   \code
   // Filling a send buffer with a single double value and a 3-dimensional vector
   SendBuffer<> buffer;
   double d( 2.1 );
   Vec3 v( 3.5, -2.1, 0.7 );
   buffer << d << v;

   // Sending an MPI message to process 0 with the contents of the send buffer
   MPI_Send( buffer.ptr(), buffer.size(), MPI_BYTE, 0, 0, MPI_COMM_WORLD );
   \endcode
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline byte* SendBuffer<E,G>::ptr() const
{
   return begin_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the minimum capacity of the send buffer.
 *
 * \param newCapacity The new minimum capacity of the send buffer.
 * \return void
 *
 * This function reserves at least \a newCapacity bytes for the send buffer.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline void SendBuffer<E,G>::reserve( size_t newCapacity )
{
   if( newCapacity > capacity() )
      extendMemory( newCapacity );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extending the internal memory of the send buffer.
 *
 * \param newCapacity The new minimum capacity of the send buffer.
 * \return void
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
void SendBuffer<E,G>::extendMemory( size_t newCapacity )
{
   // Calculating the new capacity
   pe_INTERNAL_ASSERT( newCapacity > capacity(), "Invalid new send buffer capacity" );
   newCapacity = G()( capacity(), newCapacity );
   pe_INTERNAL_ASSERT( newCapacity > capacity(), "Invalid new send buffer capacity" );

   // Allocating a new array
   byte* tmp = new byte[newCapacity];

   // Replacing the old array
   cur_ = std::copy( begin_, cur_, tmp );
   std::swap( tmp, begin_ );
   end_ = begin_ + newCapacity;
   delete [] tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the send buffer.
 *
 * \return void
 *
 * This function performs a complete reset of the send buffer.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline void SendBuffer<E,G>::clear()
{
   cur_ = begin_;
}
//*************************************************************************************************

} // namespace pe

#endif
