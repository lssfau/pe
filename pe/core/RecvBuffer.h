//=================================================================================================
/*!
 *  \file pe/core/RecvBuffer.h
 *  \brief Implementation of a receive buffer.
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

#ifndef _PE_CORE_RECVBUFFER_H_
#define _PE_CORE_RECVBUFFER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Builtin.h>
#include <pe/util/Byte.h>
#include <pe/util/Endianness.h>
#include <pe/util/Null.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a receive buffer.
 * \ingroup mpi
 *
 * The RecvBuffer class can be used to disassemble a buffer into different data types. The receive
 * buffer supports endianness conversion if desired. For example it can be used for mixed-type MPI
 * communication. The following example gives an impression of the usage of the RecvBuffer class:

   \code
   // Preparing a receive buffer for an incoming message of 32 bytes
   RecvBuffer<> buffer;
   buffer.resize( 32 );

   // Receiving an MPI message from process 0 in a blocking MPI_Recv() function
   MPI_Status status;
   MPI_Recv( buffer, 32, MPI_BYTE, 0, MPI_ANY_TAG, MPI_COMM_WORLD, status );

   // Extracting a single double value from the receive buffer
   double d;
   buffer >> d;
   \endcode

 * Note that the order of data values in the receive buffer is depending on the order in which
 * the data values were added. See also the SendBuffer class description for the sender side.
 */
template< typename E = NoEndiannessConversion >  // Endianness conversion policy
class RecvBuffer
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline RecvBuffer();
            inline RecvBuffer( const RecvBuffer& rb );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~RecvBuffer();
   //@}
   //**********************************************************************************************

   //**Assignment operator*************************************************************************
   /*!\name Assignment operator */
   //@{
   RecvBuffer& operator=( const RecvBuffer& sb );
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
   template< typename V > RecvBuffer& operator>>( V& value );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
                          inline byte* ptr    () const;
                          inline void  reserve( size_t newCapacity );
                          inline void  resize ( size_t newSize     );
   template< typename V >        void  peek   ( V& value ) const;
                          inline void  skip   ( size_t count       );
                          inline void  clear  ();
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t capacity_;  //!< The current size of the receive buffer.
   byte* begin_;      //!< Pointer to the first byte of the receive buffer.
   byte* cur_;        //!< Pointer to the current byte of the receive buffer.
   byte* end_;        //!< Pointer to the last byte of the receive buffer.
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
/*!\brief Standard constructor for RecvBuffer.
 */
template< typename E >  // Endianness conversion policy
inline RecvBuffer<E>::RecvBuffer()
   : capacity_( 0    )  // Capacity of the receive buffer
   , begin_   ( NULL )  // Pointer to the first byte
   , cur_     ( NULL )  // Pointer to the current byte
   , end_     ( NULL )  // Pointer to the last byte
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy constructor for RecvBuffer.
 *
 * \param rb The receive buffer to be copied.
 */
template< typename E >  // Endianness conversion policy
inline RecvBuffer<E>::RecvBuffer( const RecvBuffer& rb )
   : capacity_( rb.size()        )     // Capacity of the receive buffer
   , begin_   ( new byte[capacity_] )  // Pointer to the first byte
   , cur_     ( begin_           )     // Pointer to the current byte
   , end_     ( begin_+capacity_ )     // Pointer to the last byte
{
   for( size_t i=0; i<capacity_; ++i )
      cur_[i] = rb.cur_[i];
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for RecvBuffer.
 */
template< typename E >  // Endianness conversion policy
inline RecvBuffer<E>::~RecvBuffer()
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
/*!\brief Copy assignment operator for RecvBuffer.
 *
 * \param rb The receive buffer to be copied.
 * \return Reference to the assigned receive buffer.
 */
template< typename E >  // Endianness conversion policy
RecvBuffer<E>& RecvBuffer<E>::operator=( const RecvBuffer& rb )
{
   if( &rb == this ) return *this;

   if( rb.size() > capacity_ ) {
      byte* newBegin( new byte[rb.size()] );
      end_ = std::copy( rb.cur_, rb.end_, newBegin );
      std::swap( begin_, newBegin );
      delete [] newBegin;

      capacity_ = rb.size();
      cur_ = begin_;
   }
   else {
      end_ = std::copy( rb.cur_, rb.end_, begin_ );
      cur_ = begin_;
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
/*!\brief Returns the maximum possible size of the receive buffer.
 *
 * \return The maximum possible size.
 */
template< typename E >  // Endianness conversion policy
inline size_t RecvBuffer<E>::maxSize() const
{
   return size_t(-1);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current size of the receive buffer.
 *
 * \return The current size.
 */
template< typename E >  // Endianness conversion policy
inline size_t RecvBuffer<E>::size() const
{
   return end_ - cur_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the capacity of the receive buffer.
 *
 * \return The capacity.
 */
template< typename E >  // Endianness conversion policy
inline size_t RecvBuffer<E>::capacity() const
{
   return capacity_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns \a true if the receive buffer is empty.
 *
 * \return \a true if the receive buffer is empty, \a false if it is not.
 */
template< typename E >  // Endianness conversion policy
inline bool RecvBuffer<E>::isEmpty() const
{
   return cur_ >= end_;
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Reads a built-in data value from the receive buffer.
 *
 * \return Reference to the receive buffer.
 *
 * This function extracts one data value of built-in data type from the receive buffer.
 * In case pe_USER_ASSERT() is active, this operator performs a validity check of the
 * read operation.
 *
 * \b Note: This operator may only be used for built-in data types. The attempt to use
 * a user-defined data type results in a compile time error!
 */
template< typename E >  // Endianness conversion policy
template< typename V >  // Type of the built-in data value
RecvBuffer<E>& RecvBuffer<E>::operator>>( V& value )
{
   pe_CONSTRAINT_MUST_BE_BUILTIN_TYPE( V );

   // Checking the validity of the read operation
   pe_USER_ASSERT( cur_ + sizeof(V) <= end_, "Invalid receive buffer access" );

   // Extracting the data value
   V* tmp( reinterpret_cast<V*>( cur_ ) );
   value = E()( *tmp );
   cur_  = reinterpret_cast<byte*>( ++tmp );

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
/*!\brief Returns a pointer to the first byte of the receive buffer.
 *
 * \return Pointer to the first byte of the receive buffer.
 *
 * This utility function enables the RecvBuffer to be used directly as receive buffer for example
 * in MPI receive functions. Note however, that this operation is only allowed for a reinitialized
 * receive buffer (for instance via the resize() function).

   \code
   // Preparing a receive buffer for a message of 50 bytes
   RecvBuffer<> buffer;
   buffer.resize( 50 );

   // Receiving an MPI message from process 0 in a blocking MPI_Recv() function
   MPI_Status status;
   MPI_Recv( buffer.ptr(), buffer.size(), MPI_BYTE, 0, MPI_ANY_TAG, MPI_COMM_WORLD, status );
   \endcode
 */
template< typename E >  // Endianness conversion policy
inline byte* RecvBuffer<E>::ptr() const
{
   pe_INTERNAL_ASSERT( begin_ == cur_, "Receive buffer was not reinitialized" );
   return begin_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the minimum capacity of the receive buffer.
 *
 * \param newCapacity The new minimum capacity of the receive buffer.
 * \return void
 *
 * This function reserves at least \a newCapacity bytes for the receive
 * buffer. Note that this operation involves a complete reset of the receive buffer!
 */
template< typename E >  // Endianness conversion policy
void RecvBuffer<E>::reserve( size_t newCapacity )
{
   if( newCapacity > capacity_ )
   {
      // Allocating a new array
      byte* tmp = new byte[newCapacity];

      // Replacing the old array
      std::swap( tmp, begin_ );
      delete [] tmp;

      // Adjusting the members
      capacity_ = newCapacity;
   }

   // Clearing the receive buffer
   clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Changing the size of the receive buffer.
 *
 * \param newSize The new size of the receive buffer.
 * \return void
 *
 * This function resizes the receive buffer to the given size \a newSize. Note that this
 * operation does not preserve the current contents of the receive buffer!
 */
template< typename E >  // Endianness conversion policy
void RecvBuffer<E>::resize( size_t newSize )
{
   if( newSize > capacity_ )
   {
      // Allocating a new array
      byte* tmp = new byte[newSize];

      // Replacing the old array
      std::swap( tmp, begin_ );
      delete [] tmp;

      // Adjusting the capacity
      capacity_ = newSize;
   }

   cur_ = begin_;
   end_ = begin_+newSize;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reads a built-in data value from the receive buffer without extracting it.
 *
 * \return void
 *
 * This function reads the next built-in data value of type \a V from the receive buffer
 * without extracting/removing it from the buffer.
 */
template< typename E >  // Endianness conversion policy
template< typename V >  // Type of the built-in data value
inline void RecvBuffer<E>::peek( V& value ) const
{
   pe_CONSTRAINT_MUST_BE_BUILTIN_TYPE( V );

   // Checking the validity of the read operation
   pe_USER_ASSERT( cur_ + sizeof(V) <= end_, "Invalid receive buffer access" );

   // Extracting the data value
   V* tmp( reinterpret_cast<V*>( cur_ ) );
   value = E()( *tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Skipping the given number of bytes.
 *
 * \param count The number of bytes to be skipped.
 * \return void
 *
 * This function skips \a count bytes in the receive buffer.
 */
template< typename E >  // Endianness conversion policy
void RecvBuffer<E>::skip( size_t count )
{
   cur_ += count;
   if( cur_ > end_ ) cur_ = end_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the receive buffer.
 *
 * \return void
 *
 * This function performs a complete reset of the receive buffer.
 */
template< typename E >  // Endianness conversion policy
inline void RecvBuffer<E>::clear()
{
   cur_ = begin_;
   end_ = begin_;
}
//*************************************************************************************************

} // namespace pe

#endif
