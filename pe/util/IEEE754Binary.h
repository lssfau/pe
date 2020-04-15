//=================================================================================================
/*!
 *  \file pe/util/IEEE754Binary.h
 *  \brief Casting of IEEE754 floating point types.
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

#ifndef _PE_UTIL_IEEE754BINARY_H_
#define _PE_UTIL_IEEE754BINARY_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************


#include <pe/util/StaticAssert.h>
#include <pe/util/logging/DebugSection.h>
#include <stdint.h>


const bool debugIEEE754Binary = false;
#define pe_LOG_DEBUG_IEEE754BINARY_SECTION( NAME ) \
   if( debugIEEE754Binary ) pe_LOG_DEBUG_SECTION( NAME )




namespace pe {


template<int N>
struct IEEE754Binary {
};


template<>
struct IEEE754Binary<2> {
   typedef uint16_t uint_t;
   typedef uint16_t fp_t;
   union var_t {
      uint_t uint;
      fp_t fp;

      var_t( const uint_t o ) : uint(o) {}
   };
   pe_STATIC_ASSERT( sizeof( fp_t ) == 2 );  // "Half is not an IEEE754 half-precision floating point number."

   static const int    significand_size   = 10;
   static const int    exponent_size      = 5;
   static const int    bias               = 15;

   static const int    significand_offset = 0;
   static const int    exponent_offset    = significand_size;
   static const int    sign_offset        = exponent_offset + exponent_size;

   static const uint_t bit0               = static_cast<uint_t>( 1 );
   static const uint_t signaling_bit      = bit0 << ( significand_size - 1 );
   static const uint_t sign_bit           = bit0 << sign_offset;
   static const uint_t exponent_mask      = ( ( bit0 << exponent_size ) - 1 ) << exponent_offset;        // = 0x7c00u;
   static const uint_t significand_mask   = ( ( bit0 << significand_size ) - 1 ) << significand_offset;  // = 0x03ffu;

   static const int    exponent_min       = 1 - bias;
   static const int    exponent_max       = ( exponent_mask >> exponent_offset ) - ( bias + 1 );

   static const uint_t eps                = ( static_cast<uint_t>( bias - ( significand_size + 1 ) ) ) << exponent_offset;
   static const uint_t max                = ( exponent_mask | significand_mask ) & ( ~( bit0 << exponent_offset ) );
};


template<>
struct IEEE754Binary<4> {
   typedef uint32_t uint_t;
   typedef float    fp_t;
   union var_t {
      uint_t uint;
      fp_t fp;

      var_t( const uint_t o ) : uint(o) {}
      var_t( const fp_t o ) : fp(o) {}
   };
   pe_STATIC_ASSERT( sizeof( fp_t ) == 4 );  // "Float is not an IEEE754 single-precision floating point number."

   static const int    significand_size   = 23;
   static const int    exponent_size      = 8;
   static const int    bias               = 127;

   static const int    significand_offset = 0;
   static const int    exponent_offset    = significand_size;
   static const int    sign_offset        = exponent_offset + exponent_size;

   static const uint_t bit0               = static_cast<uint_t>( 1 );
   static const uint_t signaling_bit      = bit0 << ( significand_size - 1 );
   static const uint_t sign_bit           = bit0 << sign_offset;
   static const uint_t exponent_mask      = ( ( bit0 << exponent_size ) - 1 ) << exponent_offset;        // = 0x7f800000u;
   static const uint_t significand_mask   = ( ( bit0 << significand_size ) - 1 ) << significand_offset;  // = 0x007fffffu;

   static const int    exponent_min       = 1 - bias;
   static const int    exponent_max       = ( exponent_mask >> exponent_offset ) - ( bias + 1 );

   static const uint_t eps                = ( static_cast<uint_t>( bias - ( significand_size + 1 ) ) ) << exponent_offset;
   static const uint_t max                = ( exponent_mask | significand_mask ) & ( ~( bit0 << exponent_offset ) );
};


template<>
struct IEEE754Binary<8> {
   typedef uint64_t uint_t;
   typedef double   fp_t;
   union var_t {
      uint_t uint;
      fp_t fp;

      var_t( const uint_t o ) : uint(o) {}
      var_t( const fp_t o ) : fp(o) {}
   };
   pe_STATIC_ASSERT( sizeof( fp_t ) == 8 );  // "Double is not an IEEE754 double-precision floating point number."

   static const int    significand_size   = 52;
   static const int    exponent_size      = 11;
   static const int    bias               = 1023;

   static const int    significand_offset = 0;
   static const int    exponent_offset    = significand_size;
   static const int    sign_offset        = exponent_offset + exponent_size;

   static const uint_t bit0               = static_cast<uint_t>( 1 );
   static const uint_t signaling_bit      = bit0 << ( significand_size - 1 );
   static const uint_t sign_bit           = bit0 << sign_offset;
   static const uint_t exponent_mask      = ( ( bit0 << exponent_size ) - 1 ) << exponent_offset;
   static const uint_t significand_mask   = ( ( bit0 << significand_size ) - 1 ) << significand_offset;

   static const int    exponent_min       = 1 - bias;
   static const int    exponent_max       = ( exponent_mask >> exponent_offset ) - ( bias + 1 );

   static const uint_t eps                = ( static_cast<uint_t>( bias - ( significand_size + 1 ) ) ) << exponent_offset;
   static const uint_t max                = ( exponent_mask | significand_mask ) & ( ~( bit0 << exponent_offset ) );
};


enum RoundingMode {
   RoundToNearestTiesToEven = 0,
   RoundToNearestTiesAwayFromZero = 1,
   DirectedRoundTowardsZero = 2,
   Truncation = DirectedRoundTowardsZero,
   DirectedRoundTowardsPositiveInfintiy = 3,
   DirectedRoundTowardsNegativeInfinity = 4
};


const RoundingMode fprounding = Truncation;

pe_STATIC_ASSERT( fprounding == Truncation );  // "The only IEEE rounding mode supported is directed rounding towards zero (truncation)."


template<typename FromType, typename ToType>
typename ToType::fp_t fp_widen( const typename FromType::fp_t& x ) {
   typedef typename FromType::fp_t from_fp_t;
   typedef typename FromType::uint_t from_uint_t;
   typedef typename ToType::fp_t to_fp_t;
   typedef typename ToType::uint_t to_uint_t;

   to_uint_t inval = static_cast<to_uint_t>( typename FromType::var_t( x ).uint );
   to_uint_t signbit = ( inval & FromType::sign_bit ) << (ToType::sign_offset - FromType::sign_offset);  // mask sign bit, shift and shift back to ToType position

   to_uint_t exponent, significand;
   if( ( inval & FromType::exponent_mask ) == 0 ) {
      pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
         log << "exp zero\n";
      }
      if( ( inval & FromType::significand_mask ) != 0 ) {
         // significand is non-zero => subnormal number => no longer subnormal
         pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
            log << "significand non-zero => subnormal => regular\n";
         }
         significand = inval & FromType::significand_mask;
         exponent = (1 - FromType::bias) - 1 + ToType::bias;  // exponent of ToType subnormals (-14), -1 for final significand shift (where the leading 1 is discarded), +127 for ToType bias
         while( ( significand & FromType::signaling_bit ) == 0 ) {
            significand <<= 1;
            --exponent;
         }
         exponent <<= ToType::exponent_offset;
         significand = ( significand << 1 ) & FromType::significand_mask;   // discard leading 1 of fraction
         significand = significand << ( ToType::exponent_offset - FromType::exponent_offset );   // shift to correct ToType position by filling in zeros in the least significant places
      }
      else {
         // significand is zero  =>  +-zero
         pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
            log << "significand zero => zero\n";
         }
         exponent = 0;
         significand = 0;
      }
   }
   else if( ( inval & FromType::exponent_mask ) == FromType::exponent_mask ) {
      // NaNs and infs are kept when casting.
      pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
         log << "exp all ones\n";
      }
      exponent = ToType::exponent_mask;  // exponent is all ones
      if( ( inval & FromType::significand_mask ) != 0 ) {
         // significand is non-zero  =>  NaN
         if( inval & FromType::signaling_bit )
            // signaling NaN
            significand = ( inval & ( FromType::significand_mask & (~FromType::signaling_bit) ) ) | ToType::signaling_bit;  // mask significand without signaling bit, higher significant bits of payload are padded with zeros, set signaling bit
         else
            // quiet NaN
            significand = inval & FromType::significand_mask;  // mask significand, higher significant bits of payload are padded with zeros, signaling bit thus remains zero
         pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
            log << "significand non-zero => NaN\n";
         }
      }
      else {
         // significand is zero  =>  +-inf
         pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
            log << "significand zero => inf\n";
         }
         significand = 0;
      }
   }
   else {
      pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
         log << "exp/significand regular\n";
      }
      exponent    = ( ( ( ( inval & FromType::exponent_mask ) >> FromType::exponent_offset ) + ( ToType::bias - FromType::bias ) ) << ToType::exponent_offset ); // mask exponent, shift, unbias with ToType bias (15), bias with ToType bias (127), shift to ToType exponent offset
      significand = ( inval & FromType::significand_mask ) << ( ToType::exponent_offset - FromType::exponent_offset );  // mask significand, and shift to correct position (appends zeros at the end of the fraction)
   }

   to_uint_t outval = signbit | exponent | significand;
   return typename ToType::var_t( outval ).fp;
}


template<typename FromType, typename ToType>
typename ToType::fp_t fp_shorten( const typename FromType::fp_t& x ) {
   typedef typename FromType::fp_t from_fp_t;
   typedef typename FromType::uint_t from_uint_t;
   typedef typename ToType::fp_t to_fp_t;
   typedef typename ToType::uint_t to_uint_t;

   from_uint_t inval = typename FromType::var_t( x ).uint;
   to_uint_t signbit = ( inval & FromType::sign_bit ) >> ( FromType::sign_offset - ToType::sign_offset );  // mask sign bit, shift and shift back to ToType position

   to_uint_t exponent, significand;
   if( ( inval & FromType::exponent_mask ) == 0 ) {
      pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
         log << "exp zero\n";
      }
      if( ( inval & FromType::significand_mask ) != 0 ) {
         // significand is non-zero => subnormal number => underflow to zero
         // The largest single-precision subnormal 2^{-126}*0.111...111
         // (with 23 fraction bits) is far away from the smallest
         // half-precision subnormal 2^{-14}*0.0...01=2^{-24}*1.0...0 (with
         // 10 fraction bits). Thus all subnormals underflow to zero.
         pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
            log << "significand non-zero => subnormal => underflow to zero\n";
         }
         exponent = 0;
         significand = 0;
      }
      else {
         // significand is zero  =>  +-zero
         pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
            log << "significand zero => zero\n";
         }
         exponent = 0;
         significand = 0;
      }
   }
   else if( ( inval & FromType::exponent_mask ) == FromType::exponent_mask ) {
      // NaNs and infs are kept when casting.
      exponent = ToType::exponent_mask;  // exponent is all ones
      pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
         log << "exp all ones\n";
      }
      if( ( inval & FromType::significand_mask ) != 0 ) {
         // significand is non-zero  =>  NaN
         if( inval & FromType::signaling_bit ) {
            // signaling NaN
            if( inval & ( ( FromType::significand_mask & (~FromType::signaling_bit) & (~ToType::significand_mask) ) | ToType::signaling_bit ) )
               // payload overflow (=> payload all ones)
               significand = ToType::significand_mask;
            else
               // payload fits into ToType payload
               significand = ( inval & ToType::significand_mask ) | ToType::signaling_bit;
         }
         else {
            // quiet NaN
            significand = inval & ( ToType::significand_mask & (~ToType::signaling_bit) );
            if( significand == 0 )
               // NaN payload is non-zero in the higher significant digits => indicate "payload overflow" by all ones (we must ensure that the payload is not zero for quiet NaNs since this would indicate infs)
               significand = ToType::significand_mask & (~ToType::signaling_bit);
         }
         // use lower significant bits of payload.
         significand |= ( inval & FromType::signaling_bit ) >> ( FromType::exponent_offset - ToType::exponent_offset );  // mask most significant bit (signaling bit) of significand and shift it to the correct position of ToType significand
         pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
            log << "significand non-zero => NaN\n";
         }
      }
      else {
         // significand is zero  =>  +-inf
         pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
            log << "significand zero => inf\n";
         }
         significand = 0;
      }
   }
   else {
      from_uint_t tmp = ( ( inval & FromType::exponent_mask ) >> FromType::exponent_offset );  // mask exponent and shift
      pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
         log << "exp regular: " << (int)tmp - FromType::bias << "\n";
      }
      if( tmp > static_cast<from_uint_t>( ToType::exponent_max + FromType::bias ) ) {
         // exponent overflows  =>  +-inf
         exponent = ToType::exponent_mask;
         significand = 0;
         pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
            log << "significand overflow => inf\n";
         }
      }
      else if( tmp < static_cast<from_uint_t>( ToType::exponent_min + FromType::bias ) ) {
         // exponent underflows  =>  subnormal or underflow to zero
         exponent = 0;

         from_uint_t full_significand = ( ( inval & FromType::significand_mask ) | (  static_cast<from_uint_t>( 1 ) << FromType::exponent_offset ) );  // mask significand and make implicit leading 1 explicit
         const int shift = FromType::exponent_offset - ToType::exponent_offset + ToType::exponent_min + FromType::bias - static_cast<int>( tmp );
         if( shift >= FromType::exponent_offset + 1 ) {
            // significand is completely shifted away => underflow to zero
            significand = 0;
         }
         else {
            significand = static_cast<to_uint_t>( full_significand >> shift );

            // TODO Analyze lower significant bits for rounding.
            /*
            from_uint_t lsbits = ( inval & ( ( static_cast<from_uint_t>( 1 ) << shift ) - 1 ) );
            if( lsbits & ( static_cast<from_uint_t>( 1 ) << ( shift - 1 ) ) ) {
               // bit to be rounded is 1
               if( ( lsbits & ( ~( static_cast<from_uint_t>( 1 ) << ( shift - 1 ) ) ) ) == 0 ) {
                  // all other lsbits are 0 => tie
               }
               else {
                  // rounding to nearest = +1

                  // WARNING: +1 can overflow and cause exponent to increment, then exponent can overflow and turn to inf
               }
            }
            else {
               // bit to be rounded is 0 (truncation = rounding to nearest)
            }
            */
         }
         pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
            log << "significand underflow => subnormal/zero\n";
         }
      }
      else {
         pe_LOG_DEBUG_IEEE754BINARY_SECTION( log ) {
            log << "significand regular\n";
         }
         exponent    = ( ( static_cast<to_uint_t>( tmp - ( FromType::bias - ToType::bias ) ) ) << ToType::exponent_offset ); // unbias with FromType bias (127), bias with ToType bias (15), shift to FromType exponent offset
         significand = static_cast<to_uint_t>( ( inval & FromType::significand_mask ) >> ( FromType::exponent_offset - ToType::exponent_offset ) );  // mask significand, and shift to correct position (truncates the least significant bits)

         // TODO Analyze lower significant bits for rounding.
         /*
         from_uint_t lsbits = ( inval & ( ( static_cast<from_uint_t>( 1 ) << ( FromType::exponent_offset - ToType::exponent_offset ) ) - 1 ) );
         if( lsbits & ( static_cast<from_uint_t>( 1 ) << ( FromType::exponent_offset - ToType::exponent_offset - 1 ) ) ) {
            // bit to be rounded is 1
            if( ( lsbits & ( ~( static_cast<from_uint_t>( 1 ) << ( FromType::exponent_offset - ToType::exponent_offset - 1 ) ) ) ) == 0 ) {
               // all other lsbits are 0 => tie
            }
            else {
               // rounding to nearest = +1

               // WARNING: +1 can overflow and cause exponent to increment, then exponent can overflow and turn to inf
            }
         }
         else {
            // bit to be rounded is 0 (truncation = rounding to nearest)
         }
         */
      }
   }
   
   to_uint_t outval = signbit | exponent | significand;
   return typename ToType::var_t( outval ).fp;
}


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary class for the fp_cast function.
 * \ingroup util
 */
template<typename FromType, typename ToType, bool>
struct FpCastHelper {
   typename ToType::fp_t operator()( const typename FromType::fp_t& x ) {
      return fp_shorten<FromType, ToType>( x );
   }
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the auxiliary FpCastHelper class template.
 * \ingroup util
 */
template<typename FromType, typename ToType>
struct FpCastHelper<FromType, ToType, true> {
   typename ToType::fp_t operator()( const typename FromType::fp_t& x ) {
      return fp_widen<FromType, ToType>( x );
   }
};
/*! \endcond */
//*************************************************************************************************

template<typename FromType, typename ToType>
typename ToType::fp_t fp_cast( const typename FromType::fp_t& x ) {
   return FpCastHelper<FromType, ToType, sizeof( typename FromType::fp_t ) < sizeof( typename ToType::fp_t )>()( x );
}

#undef FPDEBUG

} // namespace pe

#endif
