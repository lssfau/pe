//=================================================================================================
/*!
 *  \file pe/math/Matrix3x3.h
 *  \brief Implementation of a 3x3 matrix
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

#ifndef _PE_MATH_MATRIX3X3_H_
#define _PE_MATH_MATRIX3X3_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <cmath>
#include <ostream>
#include <pe/math/DenseMatrix.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/Equal.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/IsNaN.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Const.h>
#include <pe/util/constraints/FloatingPoint.h>
#include <pe/util/constraints/Numeric.h>
#include <pe/util/constraints/Volatile.h>
#include <pe/util/EnableIf.h>
#include <pe/util/Types.h>
#include <pe/util/typetraits/IsBuiltin.h>
#include <pe/util/typetraits/IsFloatingPoint.h>
#include <pe/util/typetraits/IsNumeric.h>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

template< typename >       struct SparseMatrix;
template< typename, bool > class  SparseVectorN;
template< typename, bool > class  Vector3;
template< typename, bool > class  VectorN;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup dense_matrix_3x3 Matrix3x3
 * \ingroup dense_matrix
 */
/*!\brief Efficient, generic implementation of a 3x3 matrix.
 * \ingroup dense_matrix_3x3
 *
 * The Matrix3x3 class is the representation of a 3x3 matrix with a total of 9 statically
 * allocated elements of arbitrary type. The naming convention of the elements is as following:

                          \f[\left(\begin{array}{*{3}{c}}
                          xx & xy & xz \\
                          yx & yy & yz \\
                          zx & zy & zz \\
                          \end{array}\right)\f]\n

 * These elements can be accessed directly with the 1D subscript operator or with the 2D function
 * operator. The numbering of the matrix elements is

                          \f[\left(\begin{array}{*{3}{c}}
                          0 & 1 & 2 \\
                          3 & 4 & 5 \\
                          6 & 7 & 8 \\
                          \end{array}\right)\f]

 * Matrix3x3 can be used with any non-cv-qualified element type. The arithmetic operators for
 * matrix/matrix, matrix/vector and matrix/element operations with the same element type work
 * for any element type as long as the element type supports the arithmetic operation. Arithmetic
 * operations between matrices, vectors and elements of different element types are only supported
 * for all data types supported by the MathTrait class template (for details see the MathTrait
 * class description).

   \code
   Matrix3x3< double > a, b, c;
   Matrix3x3< float  > d;
   Matrix3x3< std::complex<double> > e, f, g;
   Matrix3x3< std::complex<float>  > h;

   c = a + b;  // OK: Same element type, supported
   c = a + d;  // OK: Different element types, supported by the MathTrait class template

   g = e + f;  // OK: Same element type, supported
   g = e + h;  // Error: Different element types, not supported by the MathTrait class template
   \endcode
 */
template< typename Type >  // Data type of the matrix
class Matrix3x3 : public DenseMatrix< Matrix3x3<Type> >
{
public:
   //**Type definitions****************************************************************************
   typedef Matrix3x3<Type>   This;           //!< Type of this Matrix3x3 instance.
   typedef This              ResultType;     //!< Result type for expression template evaluations.
   typedef Type              ElementType;    //!< Type of the matrix elements.
   typedef const Matrix3x3&  CompositeType;  //!< Data type for composite expression templates.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline Matrix3x3();
   explicit inline Matrix3x3( Type init );
   explicit inline Matrix3x3( Type xx, Type yy, Type zz );
   explicit inline Matrix3x3( Type xx, Type xy, Type xz, Type yx, Type yy, Type yz, Type zx, Type zy, Type zz );

   template< typename Other >
   explicit inline Matrix3x3( const Vector3<Other,false>& c1,
                              const Vector3<Other,false>& c2,
                              const Vector3<Other,false>& c3 );

                              inline Matrix3x3( const Matrix3x3& m );
   template< typename Other > inline Matrix3x3( const Matrix3x3<Other>& m  );
   template< typename MT >    inline Matrix3x3( const DenseMatrix<MT>&  dm );
   template< typename MT >    inline Matrix3x3( const SparseMatrix<MT>& sm );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
                              inline Matrix3x3&  operator= ( Type set );
                              inline Matrix3x3&  operator= ( const Matrix3x3& rhs );
   template< typename Other > inline Matrix3x3&  operator= ( const Matrix3x3<Other>& rhs );
   template< typename MT >    inline Matrix3x3&  operator= ( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline Matrix3x3&  operator= ( const SparseMatrix<MT>& rhs );
                              inline Type&       operator[]( size_t index );
                              inline const Type& operator[]( size_t index )       const;
                              inline Type&       operator()( size_t i, size_t j );
                              inline const Type& operator()( size_t i, size_t j ) const;
   template< typename MT >    inline Matrix3x3&  operator+=( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline Matrix3x3&  operator+=( const SparseMatrix<MT>& rhs );
   template< typename MT >    inline Matrix3x3&  operator-=( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline Matrix3x3&  operator-=( const SparseMatrix<MT>& rhs );
   template< typename MT >    inline Matrix3x3&  operator*=( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline Matrix3x3&  operator*=( const SparseMatrix<MT>& rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, Matrix3x3 >::Type&
      operator*=( Other rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, Matrix3x3 >::Type&
      operator/=( Other rhs );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
                              inline size_t          rows()           const;
                              inline size_t          columns()        const;
                              inline void            reset();
                              inline Type            getDeterminant() const;
                              inline Matrix3x3&      transpose();
                              inline Matrix3x3&      invert();
                              inline const Matrix3x3 getInverse()     const;
   template< typename Other > inline Matrix3x3&      scale( Other scalar );
                              inline bool            isDiagonal()     const;
                              inline bool            isSymmetric()    const;
                              inline bool            isSingular()     const;
                              inline void            swap( Matrix3x3& m ) /* throw() */;
   //@}
   //**********************************************************************************************

   //**Expression template evaluation functions****************************************************
   /*!\name Expression template evaluation functions */
   //@{
   template< typename Other > inline bool isAliased( const Other* alias ) const;
   template< typename MT >    inline void assign   ( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline void assign   ( const SparseMatrix<MT>& rhs );
   template< typename MT >    inline void addAssign( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline void addAssign( const SparseMatrix<MT>& rhs );
   template< typename MT >    inline void subAssign( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline void subAssign( const SparseMatrix<MT>& rhs );
   //@}
   //**********************************************************************************************

   //**LSE functions*******************************************************************************
   /*!\name LSE functions
   //
   // The return type of the LSE functions depends on the involved data types of the
   // matrices and vectors (for further detail see the MathTrait class description).
   */
   //@{
   inline const Matrix3x3 getCholesky() const;

   template< typename Other >
   inline const Vector3< typename MathTrait<Type,Other>::MultType, false >
      solve( const Vector3<Other,false> &rhs ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Type v_[9];  //!< The nine statically allocated matrix elements.
                /*!< Access to the matrix elements is gained via the subscript or function call
                     operator. The order of the elements is
                     \f[\left(\begin{array}{*{3}{c}}
                     0 & 1 & 2 \\
                     3 & 4 & 5 \\
                     6 & 7 & 8 \\
                     \end{array}\right)\f] */
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_NOT_BE_CONST   ( Type );
   pe_CONSTRAINT_MUST_NOT_BE_VOLATILE( Type );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The default constructor for Matrix3x3.
 *
 * All matrix elements are initialized to the default value (i.e. 0 for integral data types).
 */
template< typename Type >  // Data type of the matrix
inline Matrix3x3<Type>::Matrix3x3()
{
   v_[0] = v_[1] = v_[2] = v_[3] = v_[4] = v_[5] = v_[6] = v_[7] = v_[8] = Type();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a homogenous initialization of all elements.
 *
 * \param init Initial value for all matrix elements.
 */
template< typename Type >  // Data type of the matrix
inline Matrix3x3<Type>::Matrix3x3( Type init )
{
   v_[0] = v_[1] = v_[2] = v_[3] = v_[4] = v_[5] = v_[6] = v_[7] = v_[8] = init;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a direct initialization of the diagonal matrix elements.
 *
 * \param xx Initial value for the xx-component.
 * \param yy Initial value for the yy-component.
 * \param zz Initial value for the zz-component.
 */
template< typename Type >  // Data type of the matrix
inline Matrix3x3<Type>::Matrix3x3( Type xx, Type yy, Type zz )
{
   v_[0] = xx;
   v_[4] = yy;
   v_[8] = zz;
   v_[1] = v_[2] = v_[3] = v_[5] = v_[6] = v_[7] = Type();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a direct initialization of all matrix elements.
 *
 * \param xx The initial value for the xx-component.
 * \param xy The initial value for the xy-component.
 * \param xz The initial value for the xz-component.
 * \param yx The initial value for the yx-component.
 * \param yy The initial value for the yy-component.
 * \param yz The initial value for the yz-component.
 * \param zx The initial value for the zx-component.
 * \param zy The initial value for the zy-component.
 * \param zz The initial value for the zz-component.
 */
template< typename Type >  // Data type of the matrix
inline Matrix3x3<Type>::Matrix3x3( Type xx, Type xy, Type xz,
                                   Type yx, Type yy, Type yz,
                                   Type zx, Type zy, Type zz )
{
   v_[0] = xx; v_[1] = xy; v_[2] = xz;
   v_[3] = yx; v_[4] = yy; v_[5] = yz;
   v_[6] = zx; v_[7] = zy; v_[8] = zz;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a direct initialization of the matrix columns.
 *
 * \param c1 The first column of the matrix.
 * \param c2 The second column of the matrix.
 * \param c3 The third column of the matrix.
 *
 * This constructor can be used to directly initialize the columns of the matrix using the
 * vectors \a c1, \a c2 and \a c3. The matrix is initialized as

                          \f[\left(\begin{array}{*{3}{c}}
                          c_{1,x} & c_{2,x} & c_{3,x} \\
                          c_{1,y} & c_{2,y} & c_{3,y} \\
                          c_{1,z} & c_{2,z} & c_{3,z} \\
                          \end{array}\right)\f]
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the column vectors
inline Matrix3x3<Type>::Matrix3x3( const Vector3<Other,false>& c1,
                                   const Vector3<Other,false>& c2,
                                   const Vector3<Other,false>& c3 )
{
   v_[0] = c1[0]; v_[1] = c2[0]; v_[2] = c3[0];
   v_[3] = c1[1]; v_[4] = c2[1]; v_[5] = c3[1];
   v_[6] = c1[2]; v_[7] = c2[2]; v_[8] = c3[2];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The copy constructor for Matrix3x3.
 *
 * \param m Matrix to be copied.
 *
 * The copy constructor is explicitly defined in order to enable/facilitate NRV optimization.
 */
template< typename Type >  // Data type of the matrix
inline Matrix3x3<Type>::Matrix3x3( const Matrix3x3& m )
{
   v_[0] = m.v_[0];
   v_[1] = m.v_[1];
   v_[2] = m.v_[2];
   v_[3] = m.v_[3];
   v_[4] = m.v_[4];
   v_[5] = m.v_[5];
   v_[6] = m.v_[6];
   v_[7] = m.v_[7];
   v_[8] = m.v_[8];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different Matrix3x3 instances.
 *
 * \param m Matrix to be copied.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the foreign matrix
inline Matrix3x3<Type>::Matrix3x3( const Matrix3x3<Other>& m )
{
   v_[0] = m[0];
   v_[1] = m[1];
   v_[2] = m[2];
   v_[3] = m[3];
   v_[4] = m[4];
   v_[5] = m[5];
   v_[6] = m[6];
   v_[7] = m[7];
   v_[8] = m[8];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different dense matrices.
 *
 * \param dm Dense matrix to be copied.
 * \exception std::invalid_argument Invalid setup of 3x3 matrix.
 *
 * This constructor initializes the 3x3 matrix from the given dense matrix. In case the size
 * of the given matrix does not match the size of the 3x3 matrix (i.e. the number of rows or
 * columns is not 3), a \a std::invalid_argument exception is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the foreign dense matrix
inline Matrix3x3<Type>::Matrix3x3( const DenseMatrix<MT>& dm )
{
   using pe::assign;

   if( (~dm).rows() != size_t(3) || (~dm).columns() != size_t(3) )
      throw std::invalid_argument( "Invalid setup of 3x3 matrix" );

   assign( *this, ~dm );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from sparse matrices.
 *
 * \param sm Sparse matrix to be copied.
 * \exception std::invalid_argument Invalid setup of 3x3 matrix.
 *
 * This constructor initializes the 3x3 matrix from the given sparse matrix. In case the size
 * of the given matrix does not match the size of the 3x3 matrix (i.e. the number of rows or
 * columns is not 3), a \a std::invalid_argument exception is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the foreign sparse matrix
inline Matrix3x3<Type>::Matrix3x3( const SparseMatrix<MT>& sm )
{
   using pe::assign;

   if( (~sm).rows() != size_t(3) || (~sm).columns() != size_t(3) )
      throw std::invalid_argument( "Invalid setup of 3x3 matrix" );

   if( IsBuiltin<Type>::value )
      reset();

   assign( *this, ~sm );
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Homogenous assignment to all matrix elements.
 *
 * \param set Scalar value to be assigned to all matrix elements.
 * \return Reference to the assigned matrix.
 */
template< typename Type >  // Data type of the matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::operator=( Type set )
{
   v_[0] = set;
   v_[1] = set;
   v_[2] = set;
   v_[3] = set;
   v_[4] = set;
   v_[5] = set;
   v_[6] = set;
   v_[7] = set;
   v_[8] = set;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy assignment operator for Matrix3x3.
 *
 * \param rhs Matrix to be copied.
 * \return Reference to the assigned matrix.
 *
 * Explicit definition of a copy assignment operator for performance reasons.
 */
template< typename Type >  // Data type of the matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::operator=( const Matrix3x3& rhs )
{
   // This implementation is faster than the synthesized default copy assignment operator and
   // faster than an implementation with the C library function 'memcpy' in combination with a
   // protection against self-assignment. Additionally, this version goes without a protection
   // against self-assignment.
   v_[0] = rhs.v_[0];
   v_[1] = rhs.v_[1];
   v_[2] = rhs.v_[2];
   v_[3] = rhs.v_[3];
   v_[4] = rhs.v_[4];
   v_[5] = rhs.v_[5];
   v_[6] = rhs.v_[6];
   v_[7] = rhs.v_[7];
   v_[8] = rhs.v_[8];
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for different Matrix3x3 instances.
 *
 * \param rhs Matrix to be copied.
 * \return Reference to the assigned matrix.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the foreign matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::operator=( const Matrix3x3<Other>& rhs )
{
   // This implementation is faster than the synthesized default copy assignment operator and
   // faster than an implementation with the C library function 'memcpy' in combination with a
   // protection against self-assignment. Additionally, this version goes without a protection
   // against self-assignment.
   v_[0] = rhs[0];
   v_[1] = rhs[1];
   v_[2] = rhs[2];
   v_[3] = rhs[3];
   v_[4] = rhs[4];
   v_[5] = rhs[5];
   v_[6] = rhs[6];
   v_[7] = rhs[7];
   v_[8] = rhs[8];
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for different dense matrices.
 *
 * \param rhs Dense matrix to be copied.
 * \return Reference to the assigned matrix.
 * \exception std::invalid_argument Invalid assignment to 3x3 matrix.
 *
 * This constructor initializes the matrix as a copy of the given dense matrix. In case the
 * number of rows or columns of the given matrix is not 3, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::operator=( const DenseMatrix<MT>& rhs )
{
   using pe::assign;

   if( (~rhs).rows() != size_t(3) || (~rhs).columns() != size_t(3) )
      throw std::invalid_argument( "Invalid assignment to 3x3 matrix" );

   if( IsExpression<MT>::value && (~rhs).isAliased( this ) ) {
      Matrix3x3 tmp( rhs );
      swap( tmp );
   }
   else {
      assign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for sparse matrices.
 *
 * \param rhs Sparse matrix to be copied.
 * \return Reference to the assigned matrix.
 * \exception std::invalid_argument Invalid assignment to 3x3 matrix.
 *
 * This constructor initializes the matrix as a copy of the given sparse matrix. In case the
 * number of rows or columns of the given matrix is not 3, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::operator=( const SparseMatrix<MT>& rhs )
{
   using pe::assign;

   if( (~rhs).rows() != size_t(3) || (~rhs).columns() != size_t(3) )
      throw std::invalid_argument( "Invalid assignment to 3x3 matrix" );

   reset();
   assign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 1D-access to the matrix elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..8]\f$.
 * \return Reference to the accessed value.
 *
 * In case pe_USER_ASSERT() is active, this operator performs an index check.
 */
template< typename Type >  // Data type of the matrix
inline Type& Matrix3x3<Type>::operator[]( size_t index )
{
   pe_USER_ASSERT( index < 9, "Invalid matrix access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 1D-access to the matrix elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..8]\f$.
 * \return Reference-to-const to the accessed value.
 *
 * In case pe_USER_ASSERT() is active, this operator performs an index check.
 */
template< typename Type >  // Data type of the matrix
inline const Type& Matrix3x3<Type>::operator[]( size_t index ) const
{
   pe_USER_ASSERT( index < 9, "Invalid matrix access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access to the matrix elements.
 *
 * \param i Access index for the row. The index has to be in the range [0..2].
 * \param j Access index for the column. The index has to be in the range [0..2].
 * \return Reference to the accessed value.
 */
template< typename Type >  // Data type of the matrix
inline Type& Matrix3x3<Type>::operator()( size_t i, size_t j )
{
   pe_USER_ASSERT( i<3 && j<3, "Invalid matrix access index" );
   return v_[i*3+j];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access to the matrix elements.
 *
 * \param i Access index for the row. The index has to be in the range [0..2].
 * \param j Access index for the column. The index has to be in the range [0..2].
 * \return Reference-to-const to the accessed value.
 */
template< typename Type >  // Data type of the matrix
inline const Type& Matrix3x3<Type>::operator()( size_t i, size_t j ) const
{
   pe_USER_ASSERT( i<3 && j<3, "Invalid matrix access index" );
   return v_[i*3+j];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition assignment operator for the addition of a dense matrix (\f$ A+=B \f$).
 *
 * \param rhs The right-hand side dense matrix to be added to the matrix.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two matrices don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::operator+=( const DenseMatrix<MT>& rhs )
{
   using pe::addAssign;

   if( (~rhs).rows() != size_t(3) || (~rhs).columns() != size_t(3) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   if( IsExpression<MT>::value && (~rhs).isAliased( this ) ) {
      Matrix3x3 tmp( rhs );
      v_[0] += tmp.v_[0];
      v_[1] += tmp.v_[1];
      v_[2] += tmp.v_[2];
      v_[3] += tmp.v_[3];
      v_[4] += tmp.v_[4];
      v_[5] += tmp.v_[5];
      v_[6] += tmp.v_[6];
      v_[7] += tmp.v_[7];
      v_[8] += tmp.v_[8];
   }
   else {
      addAssign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition assignment operator for the addition of a sparse matrix (\f$ A+=B \f$).
 *
 * \param rhs The right-hand side sparse matrix to be added to the matrix.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two matrices don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::operator+=( const SparseMatrix<MT>& rhs )
{
   using pe::addAssign;

   if( (~rhs).rows() != size_t(3) || (~rhs).columns() != size_t(3) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   addAssign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction assignment operator for the subtraction of a dense matrix (\f$ A-=B \f$).
 *
 * \param rhs The right-hand side dense matrix to be subtracted from the matrix.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two matrices don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::operator-=( const DenseMatrix<MT>& rhs )
{
   using pe::subAssign;

   if( (~rhs).rows() != size_t(3) || (~rhs).columns() != size_t(3) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   if( IsExpression<MT>::value && (~rhs).isAliased( this ) ) {
      Matrix3x3 tmp( rhs );
      v_[0] -= tmp.v_[0];
      v_[1] -= tmp.v_[1];
      v_[2] -= tmp.v_[2];
      v_[3] -= tmp.v_[3];
      v_[4] -= tmp.v_[4];
      v_[5] -= tmp.v_[5];
      v_[6] -= tmp.v_[6];
      v_[7] -= tmp.v_[7];
      v_[8] -= tmp.v_[8];
   }
   else {
      subAssign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction assignment operator for the subtraction of a sparse matrix (\f$ A-=B \f$).
 *
 * \param rhs The right-hand side sparse matrix to be subtracted from the matrix.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two matrices don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::operator-=( const SparseMatrix<MT>& rhs )
{
   using pe::subAssign;

   if( (~rhs).rows() != size_t(3) || (~rhs).columns() != size_t(3) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   subAssign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication of a dense matrix
 *        (\f$ A*=B \f$).
 *
 * \param rhs The right-hand side dense matrix for the multiplication.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two given matrices don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::operator*=( const DenseMatrix<MT>& rhs )
{
   if( (~rhs).rows() != size_t(3) || (~rhs).columns() != size_t(3) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   // Creating a temporary due to data dependencies
   Matrix3x3 tmp( v_[0]*(~rhs)(0,0) + v_[1]*(~rhs)(1,0) + v_[2]*(~rhs)(2,0),
                  v_[0]*(~rhs)(0,1) + v_[1]*(~rhs)(1,1) + v_[2]*(~rhs)(2,1),
                  v_[0]*(~rhs)(0,2) + v_[1]*(~rhs)(1,2) + v_[2]*(~rhs)(2,2),
                  v_[3]*(~rhs)(0,0) + v_[4]*(~rhs)(1,0) + v_[5]*(~rhs)(2,0),
                  v_[3]*(~rhs)(0,1) + v_[4]*(~rhs)(1,1) + v_[5]*(~rhs)(2,1),
                  v_[3]*(~rhs)(0,2) + v_[4]*(~rhs)(1,2) + v_[5]*(~rhs)(2,2),
                  v_[6]*(~rhs)(0,0) + v_[7]*(~rhs)(1,0) + v_[8]*(~rhs)(2,0),
                  v_[6]*(~rhs)(0,1) + v_[7]*(~rhs)(1,1) + v_[8]*(~rhs)(2,1),
                  v_[6]*(~rhs)(0,2) + v_[7]*(~rhs)(1,2) + v_[8]*(~rhs)(2,2) );

   return this->operator=( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication of a sparse matrix
 *        (\f$ A*=B \f$).
 *
 * \param rhs The right-hand side sparse matrix for the multiplication.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two given matrices don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::operator*=( const SparseMatrix<MT>& rhs )
{
   if( (~rhs).rows() != size_t(3) || (~rhs).columns() != size_t(3) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   typedef typename MT::ConstIterator  ConstIterator;

   const MT rhsT( trans( ~rhs ) );
   Matrix3x3 tmp;

   for( size_t i=0; i<3; ++i ) {
      for( size_t j=0; j<3; ++j ) {
         for( ConstIterator element=rhsT.begin(j); element!=rhsT.end(j); ++element ) {
            tmp[i*3+j] += v_[i*3+element->index()] * element->value();
         }
      }
   }

   return this->operator=( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication between a matrix and
 *        a scalar value (\f$ A*=s \f$).
 *
 * \param rhs The right-hand side scalar value for the multiplication.
 * \return Reference to the matrix.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, Matrix3x3<Type> >::Type&
   Matrix3x3<Type>::operator*=( Other rhs )
{
   v_[0] *= rhs;
   v_[1] *= rhs;
   v_[2] *= rhs;
   v_[3] *= rhs;
   v_[4] *= rhs;
   v_[5] *= rhs;
   v_[6] *= rhs;
   v_[7] *= rhs;
   v_[8] *= rhs;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Division assignment operator for the division of a matrix by a scalar value
 *        (\f$ A/=s \f$).
 *
 * \param rhs The right-hand side scalar value for the division.
 * \return Reference to the matrix.
 *
 * \b Note: A division by zero is only checked by an user assert.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, Matrix3x3<Type> >::Type&
   Matrix3x3<Type>::operator/=( Other rhs )
{
   pe_USER_ASSERT( rhs != Other(0), "Division by zero detected" );

   typedef typename MathTrait<Type,Other>::DivType  DT;

   // Depending on the two involved data types, an integer division is applied or a
   // floating point division is selected.
   if( IsNumeric<DT>::value && IsFloatingPoint<DT>::value ) {
      const DT tmp( DT(1)/static_cast<DT>( rhs ) );
      v_[0] = static_cast<Type>( static_cast<DT>( v_[0] ) * tmp );
      v_[1] = static_cast<Type>( static_cast<DT>( v_[1] ) * tmp );
      v_[2] = static_cast<Type>( static_cast<DT>( v_[2] ) * tmp );
      v_[3] = static_cast<Type>( static_cast<DT>( v_[3] ) * tmp );
      v_[4] = static_cast<Type>( static_cast<DT>( v_[4] ) * tmp );
      v_[5] = static_cast<Type>( static_cast<DT>( v_[5] ) * tmp );
      v_[6] = static_cast<Type>( static_cast<DT>( v_[6] ) * tmp );
      v_[7] = static_cast<Type>( static_cast<DT>( v_[7] ) * tmp );
      v_[8] = static_cast<Type>( static_cast<DT>( v_[8] ) * tmp );
      return *this;
   }
   else {
      v_[0] /= rhs;
      v_[1] /= rhs;
      v_[2] /= rhs;
      v_[3] /= rhs;
      v_[4] /= rhs;
      v_[5] /= rhs;
      v_[6] /= rhs;
      v_[7] /= rhs;
      v_[8] /= rhs;
      return *this;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the current number of rows of the matrix.
 *
 * \return The number of rows of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline size_t Matrix3x3<Type>::rows() const
{
   return size_t(3);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current number of columns of the matrix.
 *
 * \return The number of columns of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline size_t Matrix3x3<Type>::columns() const
{
   return size_t(3);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reset to the default initial values.
 *
 * \return void
 */
template< typename Type >  // Data type of the matrix
inline void Matrix3x3<Type>::reset()
{
   using pe::reset;
   reset( v_[0] );
   reset( v_[1] );
   reset( v_[2] );
   reset( v_[3] );
   reset( v_[4] );
   reset( v_[5] );
   reset( v_[6] );
   reset( v_[7] );
   reset( v_[8] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the determinant of the matrix.
 *
 * \return The determinant of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline Type Matrix3x3<Type>::getDeterminant() const
{
   return v_[0]*v_[4]*v_[8] + v_[1]*v_[5]*v_[6] + v_[2]*v_[3]*v_[7] -
          v_[6]*v_[4]*v_[2] - v_[7]*v_[5]*v_[0] - v_[8]*v_[3]*v_[1];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transposing the matrix.
 *
 * \return Reference to the transposed matrix.
 */
template< typename Type >  // Data type of the matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::transpose()
{
   std::swap( v_[1], v_[3] );
   std::swap( v_[2], v_[6] );
   std::swap( v_[5], v_[7] );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inverting the matrix.
 *
 * \return Reference to the inverted matrix.
 *
 * The calculation is performed with the matrix inversion by Cramer.
 *
 * \b Note: This function is only defined for matrices of floating point type. The attempt to
 * use this function with matrices of integral data types will result in a compile time error.
 */
template< typename Type >  // Data type of the matrix
inline Matrix3x3<Type>& Matrix3x3<Type>::invert()
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   Type det = v_[0] * ( ( v_[4] * v_[8] ) - ( v_[7] * v_[5] ) ) +
              v_[1] * ( ( v_[5] * v_[6] ) - ( v_[8] * v_[3] ) ) +
              v_[2] * ( ( v_[3] * v_[7] ) - ( v_[4] * v_[6] ) );

   pe_USER_ASSERT( det != Type(0), "Matrix is singular and cannot be inverted" );

   det = Type(1) / det;

   // Creating a temporary due to data dependencies
   return *this = Matrix3x3( det * ( ( v_[4]*v_[8] ) - ( v_[5]*v_[7] ) ),
                             det * ( ( v_[7]*v_[2] ) - ( v_[8]*v_[1] ) ),
                             det * ( ( v_[1]*v_[5] ) - ( v_[2]*v_[4] ) ),
                             det * ( ( v_[5]*v_[6] ) - ( v_[3]*v_[8] ) ),
                             det * ( ( v_[8]*v_[0] ) - ( v_[6]*v_[2] ) ),
                             det * ( ( v_[2]*v_[3] ) - ( v_[0]*v_[5] ) ),
                             det * ( ( v_[3]*v_[7] ) - ( v_[4]*v_[6] ) ),
                             det * ( ( v_[6]*v_[1] ) - ( v_[7]*v_[0] ) ),
                             det * ( ( v_[0]*v_[4] ) - ( v_[1]*v_[3] ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the inverse of the matrix.
 *
 * \return The inverse of the matrix.
 *
 * The calculation is performed with the matrix inversion by Cramer.
 *
 * \b Note: This function is only defined for matrices of floating point type. The attempt to
 * use this function with matrices of integral data types will result in a compile time error.
 */
template< typename Type >  // Data type of the matrix
inline const Matrix3x3<Type> Matrix3x3<Type>::getInverse() const
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   Type det = v_[0] * ( ( v_[4] * v_[8] ) - ( v_[7] * v_[5] ) ) +
              v_[1] * ( ( v_[5] * v_[6] ) - ( v_[8] * v_[3] ) ) +
              v_[2] * ( ( v_[3] * v_[7] ) - ( v_[4] * v_[6] ) );

   pe_USER_ASSERT( det != Type(0), "Matrix is singular and cannot be inverted" );

   det = Type(1) / det;

   return Matrix3x3( det * ( ( v_[4]*v_[8] ) - ( v_[5]*v_[7] ) ),
                     det * ( ( v_[7]*v_[2] ) - ( v_[8]*v_[1] ) ),
                     det * ( ( v_[1]*v_[5] ) - ( v_[2]*v_[4] ) ),
                     det * ( ( v_[5]*v_[6] ) - ( v_[3]*v_[8] ) ),
                     det * ( ( v_[8]*v_[0] ) - ( v_[6]*v_[2] ) ),
                     det * ( ( v_[2]*v_[3] ) - ( v_[0]*v_[5] ) ),
                     det * ( ( v_[3]*v_[7] ) - ( v_[4]*v_[6] ) ),
                     det * ( ( v_[6]*v_[1] ) - ( v_[7]*v_[0] ) ),
                     det * ( ( v_[0]*v_[4] ) - ( v_[1]*v_[3] ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scaling of the matrix by the scalar value \a scalar (\f$ A*=s \f$).
 *
 * \param scalar The scalar value for the matrix scaling.
 * \return Reference to the matrix.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the scalar value
inline Matrix3x3<Type>& Matrix3x3<Type>::scale( Other scalar )
{
   v_[0] *= scalar;
   v_[1] *= scalar;
   v_[2] *= scalar;
   v_[3] *= scalar;
   v_[4] *= scalar;
   v_[5] *= scalar;
   v_[6] *= scalar;
   v_[7] *= scalar;
   v_[8] *= scalar;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks if the matrix is diagonal.
 *
 * \return \a true if the matrix is diagonal, \a false if not.
 *
 * This function tests whether the matrix is diagonal, i.e. if the non-diagonal elements are
 * default elements. In case of integral or floating point data types, a diagonal matrix has
 * the form

                          \f[\left(\begin{array}{*{3}{c}}
                          xx & 0  & 0  \\
                          0  & yy & 0  \\
                          0  & 0  & zz \\
                          \end{array}\right)\f]
 */
template< typename Type >  // Data type of the matrix
inline bool Matrix3x3<Type>::isDiagonal() const
{
   if( !isDefault( v_[1] ) || !isDefault( v_[2] ) || !isDefault( v_[3] ) ||
       !isDefault( v_[5] ) || !isDefault( v_[6] ) || !isDefault( v_[7] ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks if the matrix is symmetric.
 *
 * \return \a true if the matrix is symmetric, \a false if not.
 */
template< typename Type >  // Data type of the matrix
inline bool Matrix3x3<Type>::isSymmetric() const
{
   if( !equal( v_[1], v_[3] ) || !equal( v_[2], v_[6] ) || !equal( v_[5], v_[7] ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Singularity check for the matrix (det=0).
 *
 * \return \a true if the matrix is singular, \a false if not.
 */
template< typename Type >  // Data type of the matrix
inline bool Matrix3x3<Type>::isSingular() const
{
   if( isDefault( getDeterminant() ) )
      return true;
   else return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two 3x3 matrices.
 *
 * \param m The matrix to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type >  // Data type of the matrix
inline void Matrix3x3<Type>::swap( Matrix3x3& m ) /* throw() */
{
   std::swap( v_[0], m.v_[0] );
   std::swap( v_[1], m.v_[1] );
   std::swap( v_[2], m.v_[2] );
   std::swap( v_[3], m.v_[3] );
   std::swap( v_[4], m.v_[4] );
   std::swap( v_[5], m.v_[5] );
   std::swap( v_[6], m.v_[6] );
   std::swap( v_[7], m.v_[7] );
   std::swap( v_[8], m.v_[8] );
}
//*************************************************************************************************




//=================================================================================================
//
//  EXPRESSION TEMPLATE EVALUATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the matrix is aliased with the given address \a alias.
 *
 * \param alias The alias to be checked.
 * \return \a true in case the alias corresponds to this matrix, \a false if not.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the foreign expression
inline bool Matrix3x3<Type>::isAliased( const Other* alias ) const
{
   return static_cast<const void*>( this ) == static_cast<const void*>( alias );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a dense matrix.
 *
 * \param rhs The right-hand side dense matrix to be assigned.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void Matrix3x3<Type>::assign( const DenseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 3 && (~rhs).columns() == 3, "Invalid matrix size" );

   v_[0] = (~rhs)(0,0);
   v_[1] = (~rhs)(0,1);
   v_[2] = (~rhs)(0,2);
   v_[3] = (~rhs)(1,0);
   v_[4] = (~rhs)(1,1);
   v_[5] = (~rhs)(1,2);
   v_[6] = (~rhs)(2,0);
   v_[7] = (~rhs)(2,1);
   v_[8] = (~rhs)(2,2);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a sparse matrix.
 *
 * \param rhs The right-hand side sparse matrix to be assigned.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline void Matrix3x3<Type>::assign( const SparseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 3 && (~rhs).columns() == 3, "Invalid matrix size" );

   typedef typename MT::ConstIterator  ConstIterator;

   for( size_t i=0; i<3; ++i )
      for( ConstIterator element=(~rhs).begin(i); element!=(~rhs).end(i); ++element )
         v_[i*3+element->index()] = element->value();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a dense matrix.
 *
 * \param rhs The right-hand side dense matrix to be added.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void Matrix3x3<Type>::addAssign( const DenseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 3 && (~rhs).columns() == 3, "Invalid matrix size" );

   v_[0] += (~rhs)(0,0);
   v_[1] += (~rhs)(0,1);
   v_[2] += (~rhs)(0,2);
   v_[3] += (~rhs)(1,0);
   v_[4] += (~rhs)(1,1);
   v_[5] += (~rhs)(1,2);
   v_[6] += (~rhs)(2,0);
   v_[7] += (~rhs)(2,1);
   v_[8] += (~rhs)(2,2);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a sparse matrix.
 *
 * \param rhs The right-hand side sparse matrix to be added.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline void Matrix3x3<Type>::addAssign( const SparseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 3 && (~rhs).columns() == 3, "Invalid matrix size" );

   typedef typename MT::ConstIterator  ConstIterator;

   for( size_t i=0; i<3; ++i )
      for( ConstIterator element=(~rhs).begin(i); element!=(~rhs).end(i); ++element )
         v_[i*3+element->index()] += element->value();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a dense matrix.
 *
 * \param rhs The right-hand side dense matrix to be subtracted.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void Matrix3x3<Type>::subAssign( const DenseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 3 && (~rhs).columns() == 3, "Invalid matrix size" );

   v_[0] -= (~rhs)(0,0);
   v_[1] -= (~rhs)(0,1);
   v_[2] -= (~rhs)(0,2);
   v_[3] -= (~rhs)(1,0);
   v_[4] -= (~rhs)(1,1);
   v_[5] -= (~rhs)(1,2);
   v_[6] -= (~rhs)(2,0);
   v_[7] -= (~rhs)(2,1);
   v_[8] -= (~rhs)(2,2);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a sparse matrix.
 *
 * \param rhs The right-hand side sparse matrix to be subtracted.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline void Matrix3x3<Type>::subAssign( const SparseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 3 && (~rhs).columns() == 3, "Invalid matrix size" );

   typedef typename MT::ConstIterator  ConstIterator;

   for( size_t i=0; i<3; ++i )
      for( ConstIterator element=(~rhs).begin(i); element!=(~rhs).end(i); ++element )
         v_[i*3+element->index()] -= element->value();
}
//*************************************************************************************************




//=================================================================================================
//
//  LSE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Cholesky decomposition of the matrix (\f$ A = LR \f$).
 *
 * \return The decomposed matrix \f$ L \f$.
 *
 * \b Note: This function is only defined for matrices of floating point type. The attempt to
 * use this function with matrices of integral data type will result in a compile time error.
 */
template< typename Type >  // Data type of the matrix
inline const Matrix3x3<Type> Matrix3x3<Type>::getCholesky() const
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   Matrix3x3 tmp( Type(0) );

   pe_USER_ASSERT( !isSingular(), "Matrix is singular and cannot be decomposed" );

   Type sum( 0 );

   for( int k=0; k<3; ++k ) {
      for( int p=0; p<k; ++p ) {
         sum += tmp.v_[k*3+p] * tmp.v_[k*3+p];
      }
      tmp.v_[k*4] = std::sqrt( v_[k*4]-sum );
      sum = Type(0);
      for( int i=(k+1); i<3; ++i ) {
         for( int p=0; p<k; ++p ) {
            sum += tmp.v_[k*3+p] * tmp.v_[i*3+p];
         }
         tmp.v_[i*3+k] = (v_[i*3+k]-sum) / tmp.v_[k*4];
         sum = Type(0);
      }
   }

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Solving the linear system of equations \f$A*x=b\f$ with the decomposed matrix
 *        \f$ LR = A \f$.
 *
 * \param rhs The right-hand side of the linear system of equations.
 * \return The solution of the linear system of equations.
 *
 * The function is selected for vectors of different data type (in case \a Type and \a Other
 * are supported by the MathTrait class). The operator returns a vector of the higher-order
 * data type of the two involved data types.
 *
 * \b Note: This function is only defined for matrices of floating point type. The attempt to
 * use this function with matrices of integral data type will result in a compile time error.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the right-hand side vector
inline const Vector3< typename MathTrait<Type,Other>::MultType, false >
   Matrix3x3<Type>::solve( const Vector3<Other,false> &rhs ) const
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type  );
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Other );

   typedef typename MathTrait<Type,Other>::MultType  MT;

   Vector3<MT,false> tmp1, tmp2;
   MT sum;

   // Solving the equation L*y = b
   for( int i=0; i<3; ++i ) {
      sum = rhs[i];
      for( int j=0; j<i; ++j ) {
         sum -= v_[i*3+j] * tmp1[j];
      }
      tmp1[i] = sum / v_[i*4];
   }

   // Solving the equation R*x = y
   for( int i=2; i>=0; --i ) {
      sum = tmp1[i];
      for( int j=2; j>i ; --j ) {
         sum -= v_[j*3+i] * tmp2[j];
      }
      tmp2[i] = sum / v_[i*4];
   }

   return tmp2;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Matrix3x3 operators */
//@{
template< typename T1, typename T2 >
inline bool operator==( const Matrix3x3<T1>& lhs, const Matrix3x3<T2>& rhs );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator==( const Matrix3x3<T1>& mat, T2 scalar );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator==( T1 scalar, const Matrix3x3<T2>& mat );

template< typename T1, typename T2 >
inline bool operator!=( const Matrix3x3<T1>& lhs, const Matrix3x3<T2>& rhs );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator!=( const Matrix3x3<T1>& mat, T2 scalar );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator!=( T1 scalar, const Matrix3x3<T2>& mat );

template< typename Type >
std::ostream& operator<<( std::ostream& os, const Matrix3x3<Type>& m );

template< typename Type >
inline bool isnan( const Matrix3x3<Type>& m );

template< typename Type >
inline const Matrix3x3<Type> abs( const Matrix3x3<Type>& m );

template< typename Type >
inline const Matrix3x3<Type> fabs( const Matrix3x3<Type>& m );

template< typename Type >
inline void reset( Matrix3x3<Type>& m );

template< typename Type >
inline void clear( Matrix3x3<Type>& m );

template< typename Type >
inline bool isDefault( const Matrix3x3<Type>& m );

template< typename Type >
inline const Matrix3x3<Type> trans( const Matrix3x3<Type>& m );

template< typename Type >
inline const Matrix3x3<Type> inv( const Matrix3x3<Type>& m );

template< typename Type >
inline const Matrix3x3<typename MathTrait<Type,Type>::MultType> sq( const Matrix3x3<Type>& m );

template< typename Type >
inline void swap( Matrix3x3<Type>& a, Matrix3x3<Type>& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of two matrices.
 * \ingroup dense_matrix_3x3
 *
 * \param lhs The left-hand side matrix for the comparison.
 * \param rhs The right-hand side matrix for the comparison.
 * \return \a true if the two matrices are equal, \a false if not.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side matrix
inline bool operator==( const Matrix3x3<T1>& lhs, const Matrix3x3<T2>& rhs )
{
   // In order to compare the two matrices, the data values of the lower-order data
   // type are converted to the higher-order data type within the equal function.
   if( !equal( lhs[0], rhs[0] ) ||
       !equal( lhs[1], rhs[1] ) ||
       !equal( lhs[2], rhs[2] ) ||
       !equal( lhs[3], rhs[3] ) ||
       !equal( lhs[4], rhs[4] ) ||
       !equal( lhs[5], rhs[5] ) ||
       !equal( lhs[6], rhs[6] ) ||
       !equal( lhs[7], rhs[7] ) ||
       !equal( lhs[8], rhs[8] ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a matrix and a scalar value.
 * \ingroup dense_matrix_3x3
 *
 * \param mat The left-hand side matrix for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if all elements of the matrix are equal to the scalar, \a false if not.
 *
 * If all values of the matrix are equal to the scalar value, the equality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator==( const Matrix3x3<T1>& mat, T2 scalar )
{
   // In order to compare the matrix and the scalar value, the data values of the lower-order
   // data type are converted to the higher-order data type.
   if( !equal( mat[0], scalar ) ||
       !equal( mat[1], scalar ) ||
       !equal( mat[2], scalar ) ||
       !equal( mat[3], scalar ) ||
       !equal( mat[4], scalar ) ||
       !equal( mat[5], scalar ) ||
       !equal( mat[6], scalar ) ||
       !equal( mat[7], scalar ) ||
       !equal( mat[8], scalar ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a scalar value and a matrix.
 * \ingroup dense_matrix_3x3
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param mat The right-hand side matrix for the comparison.
 * \return \a true if all elements of the matrix are equal to the scalar, \a false if not.
 *
 * If all values of the matrix are equal to the scalar value, the equality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1    // Data type of the left-hand side scalar
        , typename T2 >  // Data type of the right-hand side matrix
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator==( T1 scalar, const Matrix3x3<T2>& mat )
{
   return ( mat == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of two matrices.
 * \ingroup dense_matrix_3x3
 *
 * \param lhs The left-hand side matrix for the comparison.
 * \param rhs The right-hand side matrix for the comparison.
 * \return \a true if the two matrices are not equal, \a false if they are equal.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side matrix
inline bool operator!=( const Matrix3x3<T1>& lhs, const Matrix3x3<T2>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a matrix and a scalar value.
 * \ingroup dense_matrix_3x3
 *
 * \param mat The left-hand side matrix for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if at least one element of the matrix is different from the scalar, \a false if not.
 *
 * If one value of the matrix is inequal to the scalar value, the inequality test returns
 * \a true, otherwise \a false. Note that this function can only be used with built-in,
 * numerical data types!
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator!=( const Matrix3x3<T1>& mat, T2 scalar )
{
   return !( mat == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a scalar value and a matrix.
 * \ingroup dense_matrix_3x3
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param mat The right-hand side matrix for the comparison.
 * \return \a true if at least one element of the matrix is different from the scalar, \a false if not.
 *
 * If one value of the matrix is inequal to the scalar value, the inequality test returns
 * \a true, otherwise \a false. Note that this function can only be used with built-in,
 * numerical data types!
 */
template< typename T1    // Data type of the left-hand side scalar
        , typename T2 >  // Data type of the right-hand side matrix
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator!=( T1 scalar, const Matrix3x3<T2>& mat )
{
   return !( mat == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for 3x3 matrices.
 * \ingroup dense_matrix_3x3
 *
 * \param os Reference to the output stream.
 * \param m Reference to a constant matrix object.
 * \return Reference to the output stream.
 */
template< typename Type >  // Data type of the matrix
std::ostream& operator<<( std::ostream& os, const Matrix3x3<Type>& m )
{
   return os << " ( " << m[0] << " , " << m[1] << " , " << m[2] << " )\n"
             << " ( " << m[3] << " , " << m[4] << " , " << m[5] << " )\n"
             << " ( " << m[6] << " , " << m[7] << " , " << m[8] << " )\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks the given matrix for not-a-number elements.
 * \ingroup dense_matrix_3x3
 *
 * \param m The matrix to be checked for not-a-number elements.
 * \return \a true if at least one element of the matrix is not-a-number, \a false otherwise.
 */
template< typename Type >  // Data type of the matrix
inline bool isnan( const Matrix3x3<Type>& m )
{
   if( isnan( m[0] ) || isnan( m[1] ) || isnan( m[2] ) ||
       isnan( m[3] ) || isnan( m[4] ) || isnan( m[5] ) ||
       isnan( m[6] ) || isnan( m[7] ) || isnan( m[8] ) )
      return true;
   else return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a matrix containing the absolute values of each single element of \a m.
 * \ingroup dense_matrix_3x3
 *
 * \param m The integral input matrix.
 * \return The absolute value of each single element of \a m.
 *
 * The \a abs function calculates the absolute value of each element of the input matrix \a m.
 */
template< typename Type >  // Data type of the matrix
inline const Matrix3x3<Type> abs( const Matrix3x3<Type>& m )
{
   using std::abs;
   return Matrix3x3<Type>( abs(m[0]), abs(m[1]), abs(m[2]),
                           abs(m[3]), abs(m[4]), abs(m[5]),
                           abs(m[6]), abs(m[7]), abs(m[8]) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a matrix containing the absolute values of each single element of \a m.
 * \ingroup dense_matrix_3x3
 *
 * \param m The floating point input matrix.
 * \return The absolute value of each single element of \a m.
 *
 * The \a fabs function calculates the absolute value of each element of the input matrix \a m.
 */
template< typename Type >  // Data type of the matrix
inline const Matrix3x3<Type> fabs( const Matrix3x3<Type>& m )
{
   using std::fabs;
   return Matrix3x3<Type>( fabs(m[0]), fabs(m[1]), fabs(m[2]),
                           fabs(m[3]), fabs(m[4]), fabs(m[5]),
                           fabs(m[6]), fabs(m[7]), fabs(m[8]) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the given 3x3 matrix.
 * \ingroup dense_matrix_3x3
 *
 * \param m The matrix to be resetted.
 * \return void
 */
template< typename Type >  // Data type of the matrix
inline void reset( Matrix3x3<Type>& m )
{
   m.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the given 3x3 matrix.
 * \ingroup dense_matrix_3x3
 *
 * \param m The matrix to be cleared.
 * \return void
 *
 * Clearing a 3x3 matrix is equivalent to resetting it via the reset() function.
 */
template< typename Type >  // Data type of the matrix
inline void clear( Matrix3x3<Type>& m )
{
   m.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given 3x3 matrix is in default state.
 * \ingroup dense_matrix_3x3
 *
 * \param m The matrix to be tested for its default state.
 * \return \a true in case the given matrix is component-wise zero, \a false otherwise.
 */
template< typename Type >  // Data type of the matrix
inline bool isDefault( const Matrix3x3<Type>& m )
{
   return isDefault( m[0] ) && isDefault( m[1] ) && isDefault( m[2] ) &&
          isDefault( m[3] ) && isDefault( m[4] ) && isDefault( m[5] ) &&
          isDefault( m[6] ) && isDefault( m[7] ) && isDefault( m[8] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the transpose of the matrix.
 * \ingroup dense_matrix_3x3
 *
 * \param m The matrix to be transposed.
 * \return The transpose of the matrix.
 *
 * This function returns the transpose of the given 3x3 matrix:

   \code
   pe::Mat3 A, B;
   // ... Initialization
   B = trans( A );
   \endcode
 */
template< typename Type >  // Data type of the matrix
inline const Matrix3x3<Type> trans( const Matrix3x3<Type>& m )
{
   return Matrix3x3<Type>( m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the inverse of the matrix.
 * \ingroup dense_matrix_3x3
 *
 * \param m The matrix to be inverted.
 * \return The inverse of the matrix.
 *
 * This function returns the inverse of the given 3x3 matrix. It has the same effect as calling
 * the getInverse() member function of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline const Matrix3x3<Type> inv( const Matrix3x3<Type>& m )
{
   return m.getInverse();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Squaring the given 3x3 matrix.
 * \ingroup dense_matrix_3x3
 *
 * \param m The 3x3 matrix to be squared.
 * \return The result of the square operation.
 *
 * This function squares the given 3x3 matrix \a m. This function has the same effect as
 * multiplying the matrix with itself (\f$ m * m \f$).
 */
template< typename Type >  // Data type of the matrix
inline const Matrix3x3<typename MathTrait<Type,Type>::MultType> sq( const Matrix3x3<Type>& m )
{
   return m * m;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two 3x3 matrices.
 * \ingroup dense_matrix_3x3
 *
 * \param a The first matrix to be swapped.
 * \param b The second matrix to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type >  // Data type of the matrix
inline void swap( Matrix3x3<Type>& a, Matrix3x3<Type>& b ) /* throw() */
{
   a.swap( b );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL UNARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Matrix3x3 unary arithmetic operators */
//@{
template< typename Type >
inline const Matrix3x3<Type> operator-( const Matrix3x3<Type>& m );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unary minus operator for the negation of a 3x3 matrix (\f$ A = -B \f$).
 * \ingroup dense_matrix_3x3
 *
 * \param m The 3x3 matrix to be negated.
 * \return The negation of the matrix.
 *
 * This operator represents the negation of a 3x3 matrix:

   \code
   pe::Mat3 A, B;
   // ... Initialization
   B = -A;
   \endcode
 */
template< typename Type >  // Data type of the matrix
inline const Matrix3x3<Type> operator-( const Matrix3x3<Type>& m )
{
   return Matrix3x3<Type>( -m[0], -m[1], -m[2],
                           -m[3], -m[4], -m[5],
                           -m[6], -m[7], -m[8] );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Matrix3x3 binary arithmetic operators */
//@{
template< typename T1, typename T2 >
inline const Matrix3x3< typename MathTrait<T1,T2>::AddType >
   operator+( const Matrix3x3<T1>& lhs, const Matrix3x3<T2>& rhs );

template< typename T1, typename T2 >
inline const Matrix3x3< typename MathTrait<T1,T2>::SubType >
   operator-( const Matrix3x3<T1>& lhs, const Matrix3x3<T2>& rhs );

template< typename T1, typename T2 >
inline const Matrix3x3< typename MathTrait<T1,T2,IsNumeric<T2>::value>::MultType >
   operator*( const Matrix3x3<T1>& mat, T2 scalar );

template< typename T1, typename T2 >
inline const Matrix3x3< typename MathTrait<T1,T2,IsNumeric<T1>::value>::MultType >
   operator*( T1 scalar, const Matrix3x3<T2>& mat );

template< typename T1, typename T2 >
inline const Vector3< typename MathTrait<T1,T2>::MultType, false >
   operator*( const Matrix3x3<T1>& lhs, const Vector3<T2,false>& rhs );

template< typename T1, typename T2 >
inline const Vector3< typename MathTrait<T1,T2>::MultType, true >
   operator*( const Vector3<T1,true>& lhs, const Matrix3x3<T2>& rhs );

template< typename T1, typename T2 >
inline const Matrix3x3< typename MathTrait<T1,T2>::MultType >
   operator*( const Matrix3x3<T1>& lhs, const Matrix3x3<T2>& rhs );

template< typename T1, typename T2 >
inline const Matrix3x3< typename MathTrait< T1, T2, IsNumeric<T1>::value && IsNumeric<T2>::value >::DivType >
   operator/( const Matrix3x3<T1>& mat, T2 scalar );

template< typename T1, typename T2 >
inline const Matrix3x3< typename MathTrait<T1,T2>::MultType >
   operator%( const Matrix3x3<T1>& mat, const Vector3<T2,false>& vec );

template< typename T1, typename T2 >
inline const Matrix3x3< typename MathTrait<T1,T2>::MultType >
   operator%( const Vector3<T1,false>& vec, const Matrix3x3<T2>& mat );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition operator for the addition of two matrices (\f$ A=B+C \f$).
 * \ingroup dense_matrix_3x3
 *
 * \param lhs The left-hand side matrix for the matrix addition.
 * \param rhs The right-hand side matrix to be added to the left-hand side matrix.
 * \return The sum of the two matrices.
 *
 * This operator represents the addition of two 3x3 matrices:

   \code
   pe::Mat3 A, B, C;
   // ... Initialization
   C = A + B;
   \endcode

 * The operator returns a 3x3 matrix of the higher-order element type of the two involved matrix
 * element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be supported by the
 * MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side matrix
inline const Matrix3x3< typename MathTrait<T1,T2>::AddType >
   operator+( const Matrix3x3<T1>& lhs, const Matrix3x3<T2>& rhs )
{
   typedef typename MathTrait<T1,T2>::AddType  AT;
   return Matrix3x3<AT>( lhs[0] + rhs[0],  lhs[1] + rhs[1],  lhs[2] + rhs[2],
                         lhs[3] + rhs[3],  lhs[4] + rhs[4],  lhs[5] + rhs[5],
                         lhs[6] + rhs[6],  lhs[7] + rhs[7],  lhs[8] + rhs[8] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction operator for the subtraction of two matrices (\f$ A=B-C \f$).
 * \ingroup dense_matrix_3x3
 *
 * \param lhs The left-hand side matrix for the matrix subtraction.
 * \param rhs The right-hand side matrix to be subtracted from the left-hand side matrix.
 * \return The difference of the two matrices.
 *
 * This operator represents the subtraction of two 3x3 matrices:

   \code
   pe::Mat3 A, B, C;
   // ... Initialization
   C = A - B;
   \endcode

 * The operator returns a 3x3 matrix of the higher-order element type of the two involved matrix
 * element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be supported by the
 * MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side matrix
inline const Matrix3x3< typename MathTrait<T1,T2>::SubType >
   operator-( const Matrix3x3<T1>& lhs, const Matrix3x3<T2>& rhs )
{
   typedef typename MathTrait<T1,T2>::SubType  ST;
   return Matrix3x3<ST>( lhs[0] - rhs[0],  lhs[1] - rhs[1],  lhs[2] - rhs[2],
                         lhs[3] - rhs[3],  lhs[4] - rhs[4],  lhs[5] - rhs[5],
                         lhs[6] - rhs[6],  lhs[7] - rhs[7],  lhs[8] - rhs[8] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a matrix and a scalar value
 *        (\f$ A=B*s \f$).
 * \ingroup dense_matrix_3x3
 *
 * \param mat The left-hand side matrix for the multiplication.
 * \param scalar The right-hand side scalar value for the multiplication.
 * \return The scaled result matrix.
 *
 * This operator represents the multiplication between a 3x3 matrix and a scalar value:

   \code
   pe::Mat3 A, B;
   // ... Initialization
   B = A * 1.25;
   \endcode

 * The operator returns a 3x3 matrix of the higher-order element type of the involved data
 * types \a T1 and \a T2. Both data types \a T1 and \a T2 have to be supported by the MathTrait
 * class template. Note that this operator only works for scalar values of built-in data type.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side scalar
inline const Matrix3x3< typename MathTrait<T1,T2,IsNumeric<T2>::value>::MultType >
   operator*( const Matrix3x3<T1>& mat, T2 scalar )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;
   return Matrix3x3<MT>( mat[0]*scalar, mat[1]*scalar, mat[2]*scalar,
                         mat[3]*scalar, mat[4]*scalar, mat[5]*scalar,
                         mat[6]*scalar, mat[7]*scalar, mat[8]*scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a scalar value and a matrix
 *        (\f$ A=s*B \f$).
 * \ingroup dense_matrix_3x3
 *
 * \param scalar The left-hand side scalar value for the multiplication.
 * \param mat The right-hand side matrix for the multiplication.
 * \return The scaled result matrix.
 *
 * This operator represents the multiplication between a scalar value and a 3x3 matrix:

   \code
   pe::Mat3 A, B;
   // ... Initialization
   B = 1.25 * A;
   \endcode

 * The operator returns a 3x3 matrix of the higher-order element type of the involved data
 * types \a T1 and \a T2. Both data types \a T1 and \a T2 have to be supported by the MathTrait
 * class template. Note that this operator only works for scalar values of built-in data type.
 */
template< typename T1    // Data type of the right-hand side scalar
        , typename T2 >  // Data type of the left-hand side matrix
inline const Matrix3x3< typename MathTrait<T1,T2,IsNumeric<T1>::value>::MultType >
   operator*( T1 scalar, const Matrix3x3<T2>& mat )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;
   return Matrix3x3<MT>( mat[0]*scalar, mat[1]*scalar, mat[2]*scalar,
                         mat[3]*scalar, mat[4]*scalar, mat[5]*scalar,
                         mat[6]*scalar, mat[7]*scalar, mat[8]*scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a matrix and a vector
 *        (\f$ \vec{a}=B*\vec{c} \f$).
 * \ingroup dense_matrix_3x3
 *
 * \param lhs The left-hand side matrix for the multiplication.
 * \param rhs The right-hand side vector for the multiplication.
 * \return The resulting vector.
 *
 * This operator represents the multiplication between a 3x3 matrix and a 3D vector:

   \code
   pe::Mat3 A;
   pe::Vec3 x, y;
   // ... Initialization
   y = A * x;
   \endcode

 * The operator returns a 3D vector of the higher order element type of the two involved
 * element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be supported
 * by the MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side vector
inline const Vector3< typename MathTrait<T1,T2>::MultType, false >
   operator*( const Matrix3x3<T1>& lhs, const Vector3<T2,false>& rhs )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;
   return Vector3<MT,false>( lhs[0]*rhs[0] + lhs[1]*rhs[1] + lhs[2]*rhs[2],
                             lhs[3]*rhs[0] + lhs[4]*rhs[1] + lhs[5]*rhs[2],
                             lhs[6]*rhs[0] + lhs[7]*rhs[1] + lhs[8]*rhs[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a vector and a matrix
 *        (\f$ \vec{a}^T=\vec{b}^T*C \f$).
 * \ingroup dense_matrix_3x3
 *
 * \param lhs The left-hand side transpose vector for the multiplication.
 * \param rhs The right-hand side matrix for the multiplication.
 * \return The resulting transpose vector.
 *
 * This operator represents the multiplication between a transpose 3D vector and a 3x3
 * matrix:

   \code
   pe::Mat3 A;
   pe::Vec3T x, y;
   // ... Initialization
   y = x * A;
   \endcode

 * The operator returns a transpose 3D vector of the higher-order element type of the two
 * involved element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be
 * supported by the MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side vector
        , typename T2 >  // Data type of the right-hand side matrix
inline const Vector3< typename MathTrait<T1,T2>::MultType, true >
   operator*( const Vector3<T1,true>& lhs, const Matrix3x3<T2>& rhs )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;
   return Vector3<MT,true>( lhs[0]*rhs[0] + lhs[1]*rhs[3] + lhs[2]*rhs[6],
                            lhs[0]*rhs[1] + lhs[1]*rhs[4] + lhs[2]*rhs[7],
                            lhs[0]*rhs[2] + lhs[1]*rhs[5] + lhs[2]*rhs[8] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of two matrices (\f$ A=B*C \f$).
 * \ingroup dense_matrix_3x3
 *
 * \param lhs The left-hand side matrix for the matrix multiplication.
 * \param rhs The right-hand side matrix for the matrix multiplication.
 * \return The resulting matrix.
 *
 * This operator represents the multiplication of two 3x3 matrices:

   \code
   pe::Mat3 A, B, C;
   // ... Initialization
   C = A * B;
   \endcode

 * The operator returns a 3x3 matrix of the higher-order element type of the two involved
 * matrix element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be
 * supported by the MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side matrix
inline const Matrix3x3< typename MathTrait<T1,T2>::MultType >
   operator*( const Matrix3x3<T1>& lhs, const Matrix3x3<T2>& rhs )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;
   return Matrix3x3<MT>( lhs[0]*rhs[0] + lhs[1]*rhs[3] + lhs[2]*rhs[6],
                         lhs[0]*rhs[1] + lhs[1]*rhs[4] + lhs[2]*rhs[7],
                         lhs[0]*rhs[2] + lhs[1]*rhs[5] + lhs[2]*rhs[8],
                         lhs[3]*rhs[0] + lhs[4]*rhs[3] + lhs[5]*rhs[6],
                         lhs[3]*rhs[1] + lhs[4]*rhs[4] + lhs[5]*rhs[7],
                         lhs[3]*rhs[2] + lhs[4]*rhs[5] + lhs[5]*rhs[8],
                         lhs[6]*rhs[0] + lhs[7]*rhs[3] + lhs[8]*rhs[6],
                         lhs[6]*rhs[1] + lhs[7]*rhs[4] + lhs[8]*rhs[7],
                         lhs[6]*rhs[2] + lhs[7]*rhs[5] + lhs[8]*rhs[8] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Division operator for the divison of a matrix by a scalar value
 *        (\f$ \vec{a}=\vec{b}/s \f$).
 * \ingroup dense_matrix_3x3
 *
 * \param mat The left-hand side matrix for the division.
 * \param scalar The right-hand side scalar value for the division.
 * \return The scaled result matrix.
 *
 * This operator represents the division of a 3x3 matrix by a scalar value:

   \code
   pe::Mat3 A, B;
   // ... Initialization
   B = A / 0.24;
   \endcode

 * The operator returns a 3x3 matrix of the higher-order element type of the involved data types
 * \a T1 and \a T2. Both data types \a T1 and \a T2 have to be supported by the MathTrait class
 * template. Note that this operator is only selected in case a 3x3 matrix with either integral
 * or floating point data elements is divided by a scalar value of built-in data type.
 *
 * \b Note: A division by zero is only checked by an user assert.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side scalar
inline const Matrix3x3< typename MathTrait< T1, T2, IsNumeric<T1>::value && IsNumeric<T2>::value >::DivType >
   operator/( const Matrix3x3<T1>& mat, T2 scalar )
{
   pe_USER_ASSERT( scalar != T2(0), "Division by zero detected" );

   typedef typename MathTrait<T1,T2>::DivType  DT;

   // Depending on the two involved data types, either an integer division is applied
   // or a floating point division is selected.
   if( IsNumeric<DT>::value && IsFloatingPoint<DT>::value ) {
      const DT tmp( DT(1)/static_cast<DT>( scalar ) );
      return Matrix3x3<DT>( mat[0]*tmp, mat[1]*tmp, mat[2]*tmp,
                            mat[3]*tmp, mat[4]*tmp, mat[5]*tmp,
                            mat[6]*tmp, mat[7]*tmp, mat[8]*tmp );
   }
   else {
      return Matrix3x3<DT>( mat[0]/scalar, mat[1]/scalar, mat[2]/scalar,
                            mat[3]/scalar, mat[4]/scalar, mat[5]/scalar,
                            mat[6]/scalar, mat[7]/scalar, mat[8]/scalar );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Cross product (outer product) between a matrix and a vector (\f$ R = M \cdot r^{\times} \f$).
 * \ingroup dense_matrix_3x3
 *
 * \param mat The left-hand side matrix for the skew-symmetric cross product.
 * \param vec The right-hand side vector for the skew-symmetric cross product.
 * \return The matrix \f$ M \cdot r^{\times} \f$.
 *
 * This operator multiplies the matrix with the skew-symmetric matrix resulting from the
 * vector, which corresponds to a cross product between the columns vectors of the matrix
 * and the right-hand side vector:

                           \f[
                           M \cdot r^{\times} =

                           \left(\begin{array}{*{3}{c}}
                           m_{00} & m_{01} & m_{02} \\
                           m_{10} & m_{11} & m_{12} \\
                           m_{20} & m_{21} & m_{22} \\
                           \end{array}\right) \cdot

                           \left(\begin{array}{*{1}{c}}
                           r_0 \\
                           r_1 \\
                           r_2 \\
                           \end{array}\right)^{\times} =

                           \left(\begin{array}{*{3}{c}}
                           m_{00} & m_{01} & m_{02} \\
                           m_{10} & m_{11} & m_{12} \\
                           m_{20} & m_{21} & m_{22} \\
                           \end{array}\right) \cdot

                           \left(\begin{array}{*{3}{c}}
                           0    & -r_2 & r_1  \\
                           r_2  & 0    & -r_0 \\
                           -r_1 & r_0  & 0    \\
                           \end{array}\right)
                           \f]

 * The operator returns a matrix of the higher-order data type of the two involved data types
 * \a T1 and \a T2. In case \a T1 and \a T2 match, the operator works for any data type as long
 * as the data type has a multiplication and subtraction operator. In case \a T1 and \a T2 differ,
 * the operator only works for data types supported by the MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side vector
inline const Matrix3x3< typename MathTrait<T1,T2>::MultType >
   operator%( const Matrix3x3<T1>& mat, const Vector3<T2,false>& vec )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;
   return Matrix3x3<MT>( mat[1] * vec[2] - mat[2] * vec[1],
                         mat[2] * vec[0] - mat[0] * vec[2],
                         mat[0] * vec[1] - mat[1] * vec[0],
                         mat[4] * vec[2] - mat[5] * vec[1],
                         mat[5] * vec[0] - mat[3] * vec[2],
                         mat[3] * vec[1] - mat[4] * vec[0],
                         mat[7] * vec[2] - mat[8] * vec[1],
                         mat[8] * vec[0] - mat[6] * vec[2],
                         mat[6] * vec[1] - mat[7] * vec[0] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Cross product (outer product) between a vector and a matrix (\f$ R = r^{\times} \cdot M \f$).
 * \ingroup dense_matrix_3x3
 *
 * \param vec The left-hand side vector for the skew-symmetric product.
 * \param mat The right-hand side matrix for the skew-symmetric product.
 * \return The matrix \f$ r^{\times} \cdot M \f$.
 *
 * This operator multiplies the skew-symmetric matrix resulting from the vector with the
 * right-hand side matrix, which corresponds to a cross product between the vector and the
 * columns vectors of the matrix:

                           \f[
                           r^{\times} \cdot M =

                           \left(\begin{array}{*{1}{c}}
                           r_0 \\
                           r_1 \\
                           r_2 \\
                           \end{array}\right)^{\times} \cdot

                           \left(\begin{array}{*{3}{c}}
                           m_{00} & m_{01} & m_{02} \\
                           m_{10} & m_{11} & m_{12} \\
                           m_{20} & m_{21} & m_{22} \\
                           \end{array}\right) =

                           \left(\begin{array}{*{3}{c}}
                           0    & -r_2 & r_1  \\
                           r_2  & 0    & -r_0 \\
                           -r_1 & r_0  & 0    \\
                           \end{array}\right) \cdot

                           \left(\begin{array}{*{3}{c}}
                           m_{00} & m_{01} & m_{02} \\
                           m_{10} & m_{11} & m_{12} \\
                           m_{20} & m_{21} & m_{22} \\
                           \end{array}\right)
                           \f]

 * The operator returns a matrix of the higher-order data type of the two involved data types
 * \a T1 and \a T2. In case \a T1 and \a T2 match, the operator works for any data type as long
 * as the data type has a multiplication and subtraction operator. In case \a T1 and \a T2 differ,
 * the operator only works for data types supported by the MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side vector
        , typename T2 >  // Data type of the right-hand side matrix
inline const Matrix3x3< typename MathTrait<T1,T2>::MultType >
   operator%( const Vector3<T1,false>& vec, const Matrix3x3<T2>& mat )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;
   return Matrix3x3<MT>( vec[1] * mat[6] - vec[2] * mat[3],
                         vec[1] * mat[7] - vec[2] * mat[4],
                         vec[1] * mat[8] - vec[2] * mat[5],
                         vec[2] * mat[0] - vec[0] * mat[6],
                         vec[2] * mat[1] - vec[0] * mat[7],
                         vec[2] * mat[2] - vec[0] * mat[8],
                         vec[0] * mat[3] - vec[1] * mat[0],
                         vec[0] * mat[4] - vec[1] * mat[1],
                         vec[0] * mat[5] - vec[1] * mat[2] );
}
//*************************************************************************************************




//=================================================================================================
//
//  MATHTRAIT SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
template< typename T1, typename T2 >
struct MathTrait< Matrix3x3<T1>, T2 >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::DivType  >  DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T2 );
};

template< typename T1, typename T2 >
struct MathTrait< T1, Matrix3x3<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T1 );
};

template< typename T1, typename T2 >
struct MathTrait< Matrix3x3<T1>, Vector3<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef Vector3< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector3<T1,true>, Matrix3x3<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef Vector3< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix3x3<T1>, VectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef Vector3< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,true>, Matrix3x3<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef Vector3< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix3x3<T1>, SparseVectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef Vector3< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,true>, Matrix3x3<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef Vector3< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix3x3<T1>, Matrix3x3<T2> >
{
   typedef Matrix3x3< typename MathTrait<T1,T2>::HighType >  HighType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::LowType  >  LowType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief 3x3 real matrix.
 * \ingroup dense_matrix_3x3
 */
typedef Matrix3x3<real>  Mat3;
//*************************************************************************************************

} // namespace pe

#endif
