//=================================================================================================
/*!
 *  \file pe/math/expressions/TDVecSVecMultExpr.h
 *  \brief Header file for the dense vector/sparse vector inner product expression
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

#ifndef _PE_MATH_EXPRESSIONS_TDVECSVECMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_TDVECSVECMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/constraints/DenseVector.h>
#include <pe/math/constraints/SparseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/expressions/SparseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Multiplication operator for the scalar product (inner product) of a dense and a
 *        sparse vector (\f$ s=\vec{a}*\vec{b} \f$).
 * \ingroup sparse_vector
 *
 * \param lhs The left-hand side dense vector for the inner product.
 * \param rhs The right-hand side sparse vector for the inner product.
 * \return The scalar product.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * This operator represents the scalar product (inner product) of a dense vector and a sparse
 * vector:

   \code
   pe::VecN a;
   pe::SVecN b;
   pe::real res;
   // ... Resizing and initialization
   res = a * b;
   \endcode

 * The operator returns a scalar value of the higher-order element type of the two involved
 * vector element types \a T1::ElementType and \a T2::ElementType. Both vector types \a T1
 * and \a T2 as well as the two element types \a T1::ElementType and \a T2::ElementType have
 * to be supported by the MathTrait class template.\n
 * In case the current sizes of the two given vectors don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename T1    // Type of the left-hand side dense vector
        , typename T2 >  // Type of the right-hand side sparse vector
inline const typename MathTrait<typename T1::ElementType,typename T2::ElementType>::MultType
   operator*( const DenseVector<T1,true>& lhs, const SparseVector<T2,false>& rhs )
{
   using boost::remove_reference;

   typedef typename T1::CompositeType             Lhs;            // Composite type of the left-hand side dense vector expression
   typedef typename T2::CompositeType             Rhs;            // Composite type of the right-hand side sparse vector expression
   typedef typename remove_reference<Lhs>::type   X1;             // Auxiliary type for the left-hand side composite type
   typedef typename remove_reference<Rhs>::type   X2;             // Auxiliary type for the right-hand side composite type
   typedef typename X1::ElementType               ET1;            // Element type of the left-hand side dense vector expression
   typedef typename X2::ElementType               ET2;            // Element type of the right-hand side sparse vector expression
   typedef typename MathTrait<ET1,ET2>::MultType  MultType;       // Multiplication result type
   typedef typename X2::ConstIterator             ConstIterator;  // Iterator type of the right-hand sparse vector expression

   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE ( T1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( T2 );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( T1, true  );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( T2, false );

   if( (~lhs).size() != (~rhs).size() )
      throw std::invalid_argument( "Vector sizes do not match" );

   Lhs left ( ~lhs );
   Rhs right( ~rhs );

   MultType sp = MultType();

   for( ConstIterator element=right.begin(); element!=right.end(); ++element )
      sp += left[ element->index() ] * element->value();

   return sp;
}
//*************************************************************************************************

} // namespace pe

#endif
