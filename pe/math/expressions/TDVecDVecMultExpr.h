//=================================================================================================
/*!
 *  \file pe/math/expressions/TDVecDVecMultExpr.h
 *  \brief Header file for the dense vector/dense vector inner product expression
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

#ifndef _PE_MATH_EXPRESSIONS_TDVECDVECMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_TDVECDVECMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Multiplication operator for the scalar product (inner product) of two dense vectors
 *        (\f$ s=\vec{a}*\vec{b} \f$).
 * \ingroup dense_vector
 *
 * \param lhs The left-hand side dense vector for the inner product.
 * \param rhs The right-hand side dense vector for the inner product.
 * \return The scalar product.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * This operator represents the scalar product (inner product) of two dense vectors:

   \code
   pe::VecN a, b;
   pe::real res;
   // ... Resizing and initialization
   res = trans(a) * b;
   \endcode

 * The operator returns a scalar value of the higher-order element type of the two involved
 * vector element types \a T1::ElementType and \a T2::ElementType. Both vector types \a T1
 * and \a T2 as well as the two element types \a T1::ElementType and \a T2::ElementType have
 * to be supported by the MathTrait class template.\n
 * In case the current sizes of the two given vectors don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename T1    // Type of the left-hand side dense vector
        , typename T2 >  // Type of the right-hand side dense vector
inline const typename MathTrait<typename T1::ElementType,typename T2::ElementType>::MultType
   operator*( const DenseVector<T1,true>& lhs, const DenseVector<T2,false>& rhs )
{
   if( (~lhs).size() != (~rhs).size() )
      throw std::invalid_argument( "Vector sizes do not match" );

   typedef typename T1::CompositeType             Lhs;
   typedef typename T2::CompositeType             Rhs;
   typedef typename T1::ElementType               ET1;
   typedef typename T2::ElementType               ET2;
   typedef typename MathTrait<ET1,ET2>::MultType  MultType;

   Lhs left ( ~lhs );
   Rhs right( ~rhs );

   MultType sp = MultType();

   for( size_t i=0; i<left.size(); ++i )
      sp += left[i] * right[i];

   return sp;
}
//*************************************************************************************************

} // namespace pe

#endif
