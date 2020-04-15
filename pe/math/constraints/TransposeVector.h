//=================================================================================================
/*!
 *  \file pe/math/constraints/TransposeVector.h
 *  \brief Constraint on the data type
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

#ifndef _PE_MATH_CONSTRAINTS_TRANSPOSEVECTOR_H_
#define _PE_MATH_CONSTRAINTS_TRANSPOSEVECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/typetraits/IsTransposeVector.h>
#include <pe/math/typetraits/IsVector.h>
#include <pe/util/constraints/ConstraintTest.h>
#include <pe/util/Suffix.h>


namespace pe {

//=================================================================================================
//
//  MUST_BE_TRANSPOSE_VECTOR_TYPE CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup math_constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the compile time
 * constant expression used for the template instantiation, either the undefined basic template
 * or the specialization is selected. If the undefined basic template is selected, a compilation
 * error is created.
 */
template< bool > struct CONSTRAINT_MUST_BE_TRANSPOSE_VECTOR_TYPE_FAILED;
template<> struct CONSTRAINT_MUST_BE_TRANSPOSE_VECTOR_TYPE_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup math_constraints
 *
 * In case the given data type \a T is not a transpose dense or sparse vector type a compilation
 * error is created.
 */
#define pe_CONSTRAINT_MUST_BE_TRANSPOSE_VECTOR_TYPE(T) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_BE_TRANSPOSE_VECTOR_TYPE_FAILED< IsTransposeVector<T>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_BE_TRANSPOSE_VECTOR_TYPE_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_NOT_BE_TRANSPOSE_VECTOR_TYPE CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup math_constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the compile time
 * constant expression used for the template instantiation, either the undefined basic template
 * or the specialization is selected. If the undefined basic template is selected, a compilation
 * error is created.
 */
template< bool > struct CONSTRAINT_MUST_NOT_BE_TRANSPOSE_VECTOR_TYPE_FAILED;
template<> struct CONSTRAINT_MUST_NOT_BE_TRANSPOSE_VECTOR_TYPE_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup math_constraints
 *
 * In case the given data type \a T is a transpose dense or sparse vector type a compilation
 * error is created.
 */
#define pe_CONSTRAINT_MUST_NOT_BE_TRANSPOSE_VECTOR_TYPE(T) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_NOT_BE_TRANSPOSE_VECTOR_TYPE_FAILED< !IsTransposeVector<T>::value > \
      pe_JOIN( CONSTRAINT_MUST_NOT_BE_TRANSPOSE_VECTOR_TYPE_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_BE_NONTRANSPOSE_VECTOR_TYPE CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup math_constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the compile time
 * constant expression used for the template instantiation, either the undefined basic template
 * or the specialization is selected. If the undefined basic template is selected, a compilation
 * error is created.
 */
template< bool > struct CONSTRAINT_MUST_BE_NONTRANSPOSE_VECTOR_TYPE_FAILED;
template<> struct CONSTRAINT_MUST_BE_NONTRANSPOSE_VECTOR_TYPE_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup math_constraints
 *
 * In case the given data type \a T is a non-transpose dense or sparse vector type (i.e., a
 * vector type that is not transposed) a compilation error is created.
 */
#define pe_CONSTRAINT_MUST_BE_NONTRANSPOSE_VECTOR_TYPE(T) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_BE_NONTRANSPOSE_VECTOR_TYPE_FAILED< \
            IsVector<T>::value && \
            !IsTransposeVector<T>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_BE_NONTRANSPOSE_VECTOR_TYPE_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_NOT_BE_NONTRANSPOSE_VECTOR_TYPE CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup math_constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the compile time
 * constant expression used for the template instantiation, either the undefined basic template
 * or the specialization is selected. If the undefined basic template is selected, a compilation
 * error is created.
 */
template< bool > struct CONSTRAINT_MUST_NOT_BE_NONTRANSPOSE_VECTOR_TYPE_FAILED;
template<> struct CONSTRAINT_MUST_NOT_BE_NONTRANSPOSE_VECTOR_TYPE_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup math_constraints
 *
 * In case the given data type \a T is not a non-transpose dense or sparse vector type (i.e.,
 * any data type except a normal, non-transposed vector type) a compilation error is created.
 */
#define pe_CONSTRAINT_MUST_NOT_BE_NONTRANSPOSE_VECTOR_TYPE(T) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_NOT_BE_NONTRANSPOSE_VECTOR_TYPE_FAILED< \
            !IsVector<T>::value || \
            IsTransposeVector<T>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_NOT_BE_NONTRANSPOSE_VECTOR_TYPE_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup math_constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the compile time
 * constant expression used for the template instantiation, either the undefined basic template
 * or the specialization is selected. If the undefined basic template is selected, a compilation
 * error is created.
 */
template< bool > struct CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG_FAILED;
template<> struct CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup math_constraints
 *
 * In case the given data type \a T is not a dense or sparse vector type and in case the
 * transposition flag of the given dense or sparse vector type \a T is not set to \a TF,
 * a compilation error is created.
 */
#define pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG(T,TF) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG_FAILED< \
            IsVector<T>::value && \
            IsTransposeVector<T>::value == TF >::value > \
      pe_JOIN( CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  VECTORS_MUST_HAVE_SAME_TRANSPOSE_FLAG CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup math_constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the compile time
 * constant expression used for the template instantiation, either the undefined basic template
 * or the specialization is selected. If the undefined basic template is selected, a compilation
 * error is created.
 */
template< bool > struct CONSTRAINT_VECTORS_MUST_HAVE_SAME_TRANSPOSE_FLAG_FAILED;
template<> struct CONSTRAINT_VECTORS_MUST_HAVE_SAME_TRANSPOSE_FLAG_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup math_constraints
 *
 * In case either of the two given data types \a T1 or \a T2 is not a vector type and in case
 * the transposition flags of both vector types doesn't match, a compilation error is created.
 */
#define pe_CONSTRAINT_VECTORS_MUST_HAVE_SAME_TRANSPOSE_FLAG(T1,T2) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_VECTORS_MUST_HAVE_SAME_TRANSPOSE_FLAG_FAILED< \
            IsVector<T1>::value && \
            IsVector<T2>::value && \
            static_cast<int>( IsTransposeVector<T1>::value ) == static_cast<int>( IsTransposeVector<T2>::value ) >::value > \
      pe_JOIN( CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG_TYPEDEF, __LINE__ )
//*************************************************************************************************

} // namespace pe

#endif
