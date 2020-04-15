//=================================================================================================
/*!
 *  \file pe/core/TypeConvertingSendBuffer.h
 *  \brief Implementation of a type converting send buffer.
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

#ifndef _PE_CORE_TYPECONVERTINGSENDBUFFER_H_
#define _PE_CORE_TYPECONVERTINGSENDBUFFER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <pe/core/SendBuffer.h>
//#include <pe/math/Matrix3x3.h>
//#include <pe/math/Twist.h>
//#include <pe/math/Quaternion.h>
//#include <pe/math/Vector3.h>
//#include <pe/math/Vector6.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Builtin.h>
#include <pe/util/Byte.h>
#include <pe/util/Endianness.h>
#include <pe/util/IEEE754Binary.h>
#include <pe/util/Policies.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a type converting send buffer.
 * \ingroup core
 *
 * The TypeConvertingSendBuffer class can be used to assemble a buffer from different primitive
 * data types. The send buffer supports endianness conversion if desired. For example it can b
 * used for mixed-type MPI communications. The following example gives an impression of the usage
 * of the TypeConvertingSendBuffer class:

   \code
   // Adding a single double value to a send buffer
   SendBuffer<> buffer;
   double d( 2.1 );
   buffer << d;

   // Sending an MPI message to process 0 with the contents of the send buffer
   MPI_Send( buffer, buffer.size(), MPI_BYTE, 0, 0, MPI_COMM_WORLD );
   \endcode

 * Note that the order of data values in the send buffer is depending on the order the
 * elements are added to the buffer. The same order has to be used during the extraction of the
 * data elements. See also the RecvBuffer class description for the receiver side.
 */
template< typename E = NoEndiannessConversion  // Endianness conversion policy
        , typename G = OptimalGrowth >         // Growth policy
class TypeConvertingSendBuffer : public SendBuffer<E, G>
{
private:
   //**Type definitions****************************************************************************
   typedef SendBuffer<E, G> Base;  //!< Type of the base class.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline TypeConvertingSendBuffer( size_t initCapacity = 0 );
   //@}
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   TypeConvertingSendBuffer& operator<<( const bool& value );
   TypeConvertingSendBuffer& operator<<( const float& value );
   TypeConvertingSendBuffer& operator<<( const double& value );
   TypeConvertingSendBuffer& operator<<( const long double& value );
#if defined(_MSC_VER)
   // Bug in Visual C++ at least until 10.0 (2010, _MSC_VER = 1600) see http://stackoverflow.com/questions/9396470/iso-c-standard-conformant-result-of-the-following-code
   template< typename V > SendBuffer<E, G>& operator<<( V value ) {
      return Base::operator<<( value );
   }
#else
   using Base::operator<<;
#endif
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
   using Base::cur_;
   using Base::end_;
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template<typename T>
   inline TypeConvertingSendBuffer<E, G>& sendFloat( const T& t );
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
/*!\brief Standard constructor for TypeConvertingSendBuffer.
 *
 * \param initCapacity The initial capacity of the send buffer.
 *
 * The default initial capacity of the send buffer is specified by the selected growth policy.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline TypeConvertingSendBuffer<E,G>::TypeConvertingSendBuffer( size_t initCapacity ) : SendBuffer<E,G>( initCapacity )
{}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================


//*************************************************************************************************
/*!\brief Adds a boolean value to the send buffer.
 *
 * \return Reference to the send buffer.
 *
 * This function adds one boolean value to the send buffer and converts it to a single byte
 * beforehand.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
TypeConvertingSendBuffer<E,G>& TypeConvertingSendBuffer<E,G>::operator<<( const bool& value )
{
   operator<<( static_cast<byte>( value ) );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a single-precision floating point data value to the send buffer.
 *
 * \return Reference to the send buffer.
 *
 * This function adds one data value of single-precision floating point data type to the send
 * buffer.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
TypeConvertingSendBuffer<E,G>& TypeConvertingSendBuffer<E,G>::operator<<( const float& value )
{
   return sendFloat( value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a double-precision floating point data value to the send buffer.
 *
 * \return Reference to the send buffer.
 *
 * This function adds one data value of double-precision floating point data type to the send
 * buffer.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
TypeConvertingSendBuffer<E,G>& TypeConvertingSendBuffer<E,G>::operator<<( const double& value )
{
   return sendFloat( value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a quadruple-precision floating point data value to the send buffer.
 *
 * \return Reference to the send buffer.
 *
 * This function adds one data value of double-precision floating point data type to the send
 * buffer.
 */
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
TypeConvertingSendBuffer<E,G>& TypeConvertingSendBuffer<E,G>::operator<<( const long double& value )
{
   return sendFloat( value );
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
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
inline void TypeConvertingSendBuffer<E,G>::setFloatingPointSize( int fpSize )
{
   fpSize_ = fpSize;
}
//*************************************************************************************************


//*************************************************************************************************
template< typename E    // Endianness conversion policy
        , typename G >  // Growth policy
template< typename T >
inline TypeConvertingSendBuffer<E,G>& TypeConvertingSendBuffer<E,G>::sendFloat( const T& value ) {
   switch( fpSize_ ) {
      case 0:
         // IEEE 754 input data type
         Base::operator<<( value );
         break;
      case 1: {
         // IEEE 754 half-precision data type
         // Checking the size of the remaining memory
         if( end_ - cur_ < 2 )
            Base::extendMemory( Base::size() + 2 );

         // Adding the data value
         typedef IEEE754Binary<2>::var_t var16_t;
         IEEE754Binary<2>::uint_t* const tmp( reinterpret_cast<IEEE754Binary<2>::uint_t*>( cur_ ) );
         *tmp  = E()( var16_t( fp_cast< IEEE754Binary<sizeof(T)>, IEEE754Binary<2> >( value ) ).uint );
         cur_ += 2;

         // Invariants check
         pe_INTERNAL_ASSERT( cur_ <= end_, "Invalid pointer calculations" );
         break;
      }
      case 2:
         // IEEE 754 single-precision data type
         pe_STATIC_ASSERT( sizeof( float ) == 4 );
         Base::operator<<( static_cast<float>( value ) );
         break;
      case 3:
         // IEEE 754 double-precision data type
         pe_STATIC_ASSERT( sizeof( double ) == 8 );
         Base::operator<<( static_cast<double>( value ) );
         break;
      case 4:
         // IEEE 754 quadruple-precision data type
         // Fall through here since there is no integral data type for 128bit integers which could be used for the byte swapper.
      default:
         pe_INTERNAL_ASSERT( false, "Invalid floating point conversion size." );
         break;
   }

   return *this;
}
//*************************************************************************************************

} // namespace pe

#endif
