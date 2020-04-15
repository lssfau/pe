//=================================================================================================
/*!
 *  \file pe/util/Endianness.h
 *  \brief Header file for endianness related functions
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

#ifndef _PE_UTIL_ENDIANNESS_H_
#define _PE_UTIL_ENDIANNESS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/detail/endian.hpp>
#include <pe/util/EmptyType.h>
#include <stdint.h>


namespace pe {

//=================================================================================================
//
//  ENUMERATIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Enumeration distinguishing little, big, host and network endianness.
 * \ingroup util
 *
 * Depending on the endianness of the system the host endianness \a hostEndian is either
 * \a littleEndian or \a bigEndian. Network endianness \a networkEndian is always a synonym for
 * \a bigEndian.
 */
enum Endianness {
   littleEndian,
   bigEndian,
   networkEndian = bigEndian,
#if defined(BOOST_LITTLE_ENDIAN)
   hostEndian = littleEndian
#elif defined(BOOST_BIG_ENDIAN)
   hostEndian = bigEndian
#else
#  error "Unkown system endianness."
#endif
};
//*************************************************************************************************




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================


//*************************************************************************************************
/*!\brief Swaps endianness of 1, 2, 4 and 8 Byte integral datatypes.
 * \ingroup util
 *
 * Provides a function call operator for swapping of the endianness. The function call operator
 * expects a parameter of type \a Type.
 */
template<size_t n>
struct ByteSwapper {
#if 0
   typedef EmptyType Type; //!< Type expected by the function call operator.

   /*!\brief Function call operator for swapping the endianness of the argument.
   // \param val Argument who's endianness is to be swapped.
   // \return The argument with swapped endianness.
   */
   inline Type operator()( Type val );
#endif
};
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of byte swapper for 1 byte datatypes.
 * \ingroup util
 */
template<>
struct ByteSwapper<1> {
   typedef int8_t Type; //!< Type expected by the function call operator.

   /*!\brief Function call operator for swapping the endianness of the argument.
   // \param val Argument who's endianness is to be swapped.
   // \return The argument with swapped endianness.
   */
   inline Type operator()( Type val ) {
      return val;
   }
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of byte swapper for 2 byte datatypes.
 * \ingroup util
 */
template<>
struct ByteSwapper<2> {
   typedef int16_t Type; //!< Type expected by the function call operator.

   /*!\brief Function call operator for swapping the endianness of the argument.
   // \param val Argument who's endianness is to be swapped.
   // \return The argument with swapped endianness.
   */
   inline Type operator()( Type val ) {
      return ((((val) >> 8) & 0xff) | (((val) & 0xff) << 8));
   }
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of byte swapper for 4 byte datatypes.
 * \ingroup util
 */
template<>
struct ByteSwapper<4> {
   typedef int32_t Type; //!< Type expected by the function call operator.

   /*!\brief Function call operator for swapping the endianness of the argument.
   // \param val Argument who's endianness is to be swapped.
   // \return The argument with swapped endianness.
   */
   inline Type operator()( Type val ) {
#if defined(__GNUC__) && ((__GNUC__ == 4 && __GNUC_MINOR__ >= 3) || __GNUC__ > 4)
      return __builtin_bswap32( val );
#else
      return ((((val) & 0xff000000ul) >> 24) |
              (((val) & 0x00ff0000ul) >>  8) |
              (((val) & 0x0000ff00ul) <<  8) |
              (((val) & 0x000000fful) << 24));
#endif
   }
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of byte swapper for 8 byte datatypes.
 * \ingroup util
 */
template<>
struct ByteSwapper<8> {
   typedef int64_t Type; //!< Type expected by the function call operator.

   /*!\brief Function call operator for swapping the endianness of the argument.
   // \param val Argument who's endianness is to be swapped.
   // \return The argument with swapped endianness.
   */
   inline Type operator()( Type val ) {
#if defined(__GNUC__) && ((__GNUC__ == 4 && __GNUC_MINOR__ >= 3) || __GNUC__ > 4)
      return __builtin_bswap64( val );
#else
      return ((((val) & 0xff00000000000000ull) >> 56) |
              (((val) & 0x00ff000000000000ull) >> 40) |
              (((val) & 0x0000ff0000000000ull) >> 24) |
              (((val) & 0x000000ff00000000ull) >> 8 ) |
              (((val) & 0x00000000ff000000ull) << 8 ) |
              (((val) & 0x0000000000ff0000ull) << 24) |
              (((val) & 0x000000000000ff00ull) << 40) |
              (((val) & 0x00000000000000ffull) << 56));
#endif
   }
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swaps endianness of arbitrary 1, 2, 4 and 8 Byte datatypes.
 * \ingroup util
 * \param val Argument who's endianness is to be swapped.
 * \return The argument with swapped endianness.
 */
template<typename T>
inline T byteswap( const T& val ) {
   // reinterpret input value without using references or pointers, works also if input type equals output type
   union {
      typename ByteSwapper<sizeof(T)>::Type integral;
      T original;
   } u;

   u.original = val;
   u.integral = ByteSwapper<sizeof(T)>()( u.integral );
   return u.original;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Endianness converter for arbitrary 1, 2, 4 or 8 byte datatypes.
 * \ingroup util
 *
 * Provides a function call operator for converting the endianness.
 */
template<Endianness from, Endianness to>
struct EndiannessConverter {
   /*!\brief Function call operator for converting the endianness of the argument.
   // \param val Argument who's endianness is to be converted.
   // \return The argument with converted endianness.
   */
   template<typename T>
   inline T operator()( T val ) {
      return byteswap( val );
   }
};
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the endianness converter for arbitrary 1, 2, 4 or 8 byte datatypes.
 * \ingroup util
 */
template<Endianness endianness>
struct EndiannessConverter<endianness, endianness> {
   /*!\brief Function call operator for converting the endianness of the argument.
   // \param val Argument who's endianness is to be converted.
   // \return The argument with converted endianness.
   */
   template<typename T>
   inline T operator()( T val ) {
      return val;
   }
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Converts the endianness from host to network byte order for arbitrary 1, 2, 4 or 8 byte datatypes.
 * \ingroup util
 * \param val Argument who's endianness is to be converted.
 * \return The argument with converted endianness.
 */
template<typename T>
T hton( const T& val ) {
   return EndiannessConverter<hostEndian, networkEndian>()( val );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Converts the endianness from network to host byte order for arbitrary 1, 2, 4 or 8 byte datatypes.
 * \ingroup util
 * \param val Argument who's endianness is to be converted.
 * \return The argument with converted endianness.
 */
template<typename T>
T ntoh( const T& val ) {
   return EndiannessConverter<networkEndian, hostEndian>()( val );
}
//*************************************************************************************************




//=================================================================================================
//
//  POLICY TYPES
//
//=================================================================================================


//*************************************************************************************************
/*!\brief Policy type that can be used for preventing endianness conversions.
 * \ingroup util
 */
typedef EndiannessConverter<hostEndian, hostEndian> NoEndiannessConversion;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Policy type that can be used for host to network endianness conversions.
 * \ingroup util
 */
typedef EndiannessConverter<hostEndian, networkEndian> HtonEndiannessConversion;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Policy type that can be used for network to host endianness conversions.
 * \ingroup util
 */
typedef EndiannessConverter<networkEndian, hostEndian> NtohEndiannessConversion;
//*************************************************************************************************

} // namespace pe

#endif
