//=================================================================================================
/*!
 *  \file pe/core/TypeConvertingRecvBuffer.h
 *  \brief Implementation of a type converting receive buffer.
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

#ifndef _PE_CORE_TYPECONVERTINGRECVBUFFER_H_
#define _PE_CORE_TYPECONVERTINGRECVBUFFER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <pe/core/RecvBuffer.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Builtin.h>
#include <pe/util/Byte.h>
#include <pe/util/IEEE754Binary.h>
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
/*!\brief Implementation of a type converting receive buffer.
 * \ingroup mpi
 *
 * The RecvBuffer class can be used to disassemble a buffer into different data types. The receive
 * buffer supports endianness conversion if desired. For example it can be used for mixed-type MPI
 * communication between hosts with different data type sizes. The following example gives an
 * impression of the usage of the TypeConvertingRecvBuffer class:

   \code
   // Preparing a receive buffer for an incoming message of 32 bytes
   TypeConvertingRecvBuffer<> buffer;
   buffer.resize( 32 );

   // Receiving an MPI message from process 0 in a blocking MPI_Recv() function
   MPI_Status status;
   MPI_Recv( buffer, 32, MPI_BYTE, 0, MPI_ANY_TAG, MPI_COMM_WORLD, status );

   // Extracting a single double value and a 3-dimensional vector from the receive buffer
   double d;
   buffer >> d;
   \endcode

 * Note that the order of data values in the receive buffer is depending on the order in which
 * the data values were added. See also the TypeConvertingSendBuffer class description for the sender side.
 */
template< typename E >  // Endianness conversion policy
class TypeConvertingRecvBuffer : public RecvBuffer<E>
{
public:
   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   TypeConvertingRecvBuffer& operator>>( bool& value );
   TypeConvertingRecvBuffer& operator>>( float& value );
   TypeConvertingRecvBuffer& operator>>( double& value );
   TypeConvertingRecvBuffer& operator>>( long double& value );
   using RecvBuffer<E>::operator>>;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void setFloatingPointSize( int fpSize );
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   unsigned int fpSize_;
   using RecvBuffer<E>::cur_;
   using RecvBuffer<E>::end_;
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template<typename T>
   inline TypeConvertingRecvBuffer<E>& recvFloat( T& t );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================


//*************************************************************************************************
/*!\brief Reads a boolean value.
 *
 * \param value The output floating-point number.
 * \return Reference to the receive buffer.
 *
 * This function extracts one byte and casts it to a boolean value.
 * In case pe_USER_ASSERT() is active, this operator performs a validity check of the
 * read operation.
 */
template< typename E >  // Endianness conversion policy
inline TypeConvertingRecvBuffer<E>& TypeConvertingRecvBuffer<E>::operator>>( bool& value )
{
   byte tmp;
   operator>>( tmp );
   value = ( tmp != 0 );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reads a floating-point number of previously dynamically specified size.
 *
 * \param value The output floating-point number.
 * \return Reference to the receive buffer.
 *
 * This function extracts one floating-point number of a size specified by a call to
 * setFloatingPointSize() and casts it a single-precision floating-point value.
 * In case pe_USER_ASSERT() is active, this operator performs a validity check of the
 * read operation.
 */
template< typename E >  // Endianness conversion policy
inline TypeConvertingRecvBuffer<E>& TypeConvertingRecvBuffer<E>::operator>>( float& value )
{
   return recvFloat( value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reads a floating-point number of previously dynamically specified size.
 *
 * \param value The output floating-point number.
 * \return Reference to the receive buffer.
 *
 * This function extracts one floating-point number of a size specified by a call to
 * setFloatingPointSize() and casts it a double-precision floating-point value.
 * In case pe_USER_ASSERT() is active, this operator performs a validity check of the
 * read operation.
 */
template< typename E >  // Endianness conversion policy
inline TypeConvertingRecvBuffer<E>& TypeConvertingRecvBuffer<E>::operator>>( double& value )
{
   return recvFloat( value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reads a floating-point number of previously dynamically specified size.
 *
 * \param value The output floating-point number.
 * \return Reference to the receive buffer.
 *
 * This function extracts one floating-point number of a size specified by a call to
 * setFloatingPointSize() and casts it a double-precision floating-point value.
 * In case pe_USER_ASSERT() is active, this operator performs a validity check of the
 * read operation.
 */
template< typename E >  // Endianness conversion policy
inline TypeConvertingRecvBuffer<E>& TypeConvertingRecvBuffer<E>::operator>>( long double& value )
{
   return recvFloat( value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param value TODO
 * \return TODO
 */
template< typename E >  // Endianness conversion policy
template< typename T >
inline TypeConvertingRecvBuffer<E>& TypeConvertingRecvBuffer<E>::recvFloat( T& value ) {
   switch( fpSize_ ) {
      case 0:
         // IEEE 754 output data type
         RecvBuffer<E>::operator>>( value );
         break;
      case 1: {
         // IEEE 754 half-precision data type
         pe_USER_ASSERT( cur_ + sizeof( IEEE754Binary<2>::var_t ) <= end_, "Invalid receive buffer access" );
         uint16_t* tmp( reinterpret_cast<uint16_t*>( cur_ ) );
         value = fp_cast< IEEE754Binary<2>, IEEE754Binary<sizeof(T)> >( IEEE754Binary<2>::var_t( E()( *tmp ) ).fp );
         cur_  = reinterpret_cast<byte*>( ++tmp );

         // Invariants check
         pe_INTERNAL_ASSERT( cur_ <= end_, "Invalid pointer calculations" );
         break;
      }
      case 2: {
         // IEEE 754 single-precision data type
         float tmp;
         pe_STATIC_ASSERT( sizeof( tmp ) == 4 );
         RecvBuffer<E>::operator>>( tmp );
         value = static_cast<T>( tmp );
         break;
      }
      case 3: {
         // IEEE 754 double-precision data type
         double tmp;
         pe_STATIC_ASSERT( sizeof( tmp ) == 8 );
         RecvBuffer<E>::operator>>( tmp );
         value = static_cast<T>( tmp );
         break;
      }
      case 4: {
         // IEEE 754 quadruple-precision data type
         // Fall through here since there is no integral data type for 128bit integers which could be used for the byte swapper.
      }
      default:
         pe_INTERNAL_ASSERT( false, "Invalid floating point conversion size." );
         break;
   }

   return *this;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================


//*************************************************************************************************
/*!\brief Determines the amount of data read when parsing floating point values.
 *
 * \param fpSize The exponent of the power of two of the input floating point type or 0 for no type conversion.
 * \return void
 *
 * Floating point values are dynamically converted in this receive buffer. The size of the input
 * floating point type is specified with this function. The bytes read when parsing a floating
 * point value are \f$2^{fpSize}\f$ unless fpSize is 0 indicating that no conversion should take
 * place. Currently values 0 through 3 are supported.
 */
template< typename E >  // Endianness conversion policy
inline void TypeConvertingRecvBuffer<E>::setFloatingPointSize( int fpSize )
{
   fpSize_ = fpSize;
}
//*************************************************************************************************

} // namespace pe

#endif
