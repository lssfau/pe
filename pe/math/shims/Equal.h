//=================================================================================================
/*!
 *  \file pe/math/shims/Equal.h
 *  \brief Header file for the equal shim
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

#ifndef _PE_MATH_SHIMS_EQUAL_H_
#define _PE_MATH_SHIMS_EQUAL_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/util/SelectType.h>
#include <pe/util/typetraits/IsBuiltin.h>


namespace pe {

//=================================================================================================
//
//  EQUAL SHIM
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Default equality check for integral and user-defined data types.
 * \ingroup math_shims
 *
 * \param a First value.
 * \param b Second value.
 * \return \a true if the two values are equal, \a false if not.
 *
 * Default comparison for integral data values and user-defined class types.
 */
template< typename T1    // Type of the first value/object
        , typename T2 >  // Type of the second value/object
inline bool equal_backend( T1 a, T2 b )
{
   return a == b;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Equality check for two single precision floating point values.
 * \ingroup math_shims
 *
 * \param a The left-hand side single precision floating point value.
 * \param b The right-hand side single precision floating point value.
 * \return \a true if the two values are equal, \a false if not.
 *
 * Equal function for the comparison of two single precision floating point numbers. Due to the
 * limited machine accuracy, a direct comparison of two floating point numbers should be avoided.
 * This function offers the possibility to compare two floating-point values with a certain
 * accuracy margin.
 */
template<>
inline bool equal_backend<float,float>( float a, float b )
{
   return std::fabs( a - b ) < 1E-6F;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Equality check for a single precision and a double precision floating point value.
 * \ingroup math_shims
 *
 * \param a The left-hand side single precision floating point value.
 * \param b The right-hand side double precision floating point value.
 * \return \a true if the two values are equal, \a false if not.
 *
 * Equal function for the comparison of a single precision and a double precision floating point
 * number. Due to the limited machine accuracy, a direct comparison of two floating point numbers
 * should be avoided. This function offers the possibility to compare two floating-point values
 * with a certain accuracy margin.
 */
template<>
inline bool equal_backend<float,double>( float a, double b )
{
   return std::fabs( a - b ) < 1E-6F;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Equality check for a single precision and an extended precision floating point value.
 * \ingroup math_shims
 *
 * \param a The left-hand side single precision floating point value.
 * \param b The right-hand side extended precision floating point value.
 * \return \a true if the two values are equal, \a false if not.
 *
 * Equal function for the comparison of a single precision and an extended precision floating point
 * number. Due to the limited machine accuracy, a direct comparison of two floating point numbers
 * should be avoided. This function offers the possibility to compare two floating-point values
 * with a certain accuracy margin.
 */
template<>
inline bool equal_backend<float,long double>( float a, long double b )
{
   return std::fabs( a - b ) < 1E-6F;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Equality check for a double precision and a single precision floating point value.
 * \ingroup math_shims
 *
 * \param a The left-hand side double precision floating point value.
 * \param b The right-hand side single precision floating point value.
 * \return \a true if the two values are equal, \a false if not.
 *
 * Equal function for the comparison of a double precision and a single precision floating point
 * number. Due to the limited machine accuracy, a direct comparison of two floating point numbers
 * should be avoided. This function offers the possibility to compare two floating-point values
 * with a certain accuracy margin.
 */
template<>
inline bool equal_backend<double,float>( double a, float b )
{
   return std::fabs( a - b ) < 1E-6F;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Equality check for two double precision floating point values.
 * \ingroup math_shims
 *
 * \param a The left-hand side double precision floating point value.
 * \param b The right-hand side double precision floating point value.
 * \return \a true if the two values are equal, \a false if not.
 *
 * Equal function for the comparison of two double precision floating point numbers. Due to the
 * limited machine accuracy, a direct comparison of two floating point numbers should be avoided.
 * This function offers the possibility to compare two floating-point values with a certain
 * accuracy margin.
 */
template<>
inline bool equal_backend<double,double>( double a, double b )
{
   return std::fabs( a - b ) < 1E-8;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Equality check for a double precision and an extended precision floating point value.
 * \ingroup math_shims
 *
 * \param a The left-hand side double precision floating point value.
 * \param b The right-hand side extended precision floating point value.
 * \return \a true if the two values are equal, \a false if not.
 *
 * Equal function for the comparison of a double precision and an extended precision floating point
 * number. Due to the limited machine accuracy, a direct comparison of two floating point numbers
 * should be avoided. This function offers the possibility to compare two floating-point values
 * with a certain accuracy margin.
 */
template<>
inline bool equal_backend<double,long double>( double a, long double b )
{
   return std::fabs( a - b ) < 1E-8F;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Equality check for an extended precision and a single precision floating point value.
 * \ingroup math_shims
 *
 * \param a The left-hand side extended precision floating point value.
 * \param b The right-hand side single precision floating point value.
 * \return \a true if the two values are equal, \a false if not.
 *
 * Equal function for the comparison of an extended precision and a single precision floating point
 * number. Due to the limited machine accuracy, a direct comparison of two floating point numbers
 * should be avoided. This function offers the possibility to compare two floating-point values
 * with a certain accuracy margin.
 */
template<>
inline bool equal_backend<long double,float>( long double a, float b )
{
   return std::fabs( a - b ) < 1E-6F;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Equality check for an extended precision and a double precision floating point value.
 * \ingroup math_shims
 *
 * \param a The left-hand side extended precision floating point value.
 * \param b The right-hand side double precision floating point value.
 * \return \a true if the two values are equal, \a false if not.
 *
 * Equal function for the comparison of an extended precision and a double precision floating point
 * number. Due to the limited machine accuracy, a direct comparison of two floating point numbers
 * should be avoided. This function offers the possibility to compare two floating-point values
 * with a certain accuracy margin.
 */
template<>
inline bool equal_backend<long double,double>( long double a, double b )
{
   return std::fabs( a - b ) < 1E-8F;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Equality check for two long double precision floating point values.
 * \ingroup math_shims
 *
 * \param a The left-hand side extended precision floating point value.
 * \param b The right-hand side extended precision floating point value.
 * \return \a true if the two values are equal, \a false if not.
 *
 * Equal function for the comparison of two long double precision floating point numbers. Due
 * to the limited machine accuracy, a direct comparison of two floating point numbers should be
 * avoided. This function offers the possibility to compare two floating-point values with a
 * certain accuracy margin.
 */
template<>
inline bool equal_backend<long double,long double>( long double a, long double b )
{
   return std::fabs( a - b ) < 1E-10L;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Generic equality check.
 * \ingroup math_shims
 *
 * \param a First value/object.
 * \param b Second value/object.
 * \return \a true if the two values/objects are equal, \a false if not.
 *
 * The equal shim represents an abstract interface for testing two values/objects for equality.
 * In case the two values/objects are equal, the function returns \a true, otherwise it returns
 * \a false. Per default, the comparison of the two values/objects uses the equality operator
 * operator==(). For built-in floating point data types a special comparison is selected that
 * takes the limited machine accuracy into account.
 */
template< typename T1    // Type of the first value/object
        , typename T2 >  // Type of the second value/object
inline bool equal( const T1& a, const T2& b )
{
   typedef typename SelectType<IsBuiltin<T1>::value,T1,const T1&>::Type  A;
   typedef typename SelectType<IsBuiltin<T2>::value,T2,const T2&>::Type  B;

   return equal_backend<A,B>( a, b );
}
//*************************************************************************************************

} // namespace pe

#endif
