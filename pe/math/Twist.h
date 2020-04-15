//=================================================================================================
/*!
 *  \file pe/math/Twist.h
 *  \brief Header file for the implementation of a 6D twist
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

#ifndef _PE_MATH_TWIST_H_
#define _PE_MATH_TWIST_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <istream>
#include <ostream>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup twist Twist
 * \ingroup math
 */
/*!\brief Implementation of a 6-dimensional twist.
 * \ingroup twist
 *
 * The Twist class represents a 6-dimensional spatial velocity consisting of an angular and
 * a linear velocity part. The twist is built from 6 elements, where the first three elements
 * correspond to the angular velocity part and the last three elements to the linear velocity
 * part:

                            \f[\left(\begin{array}{*{6}{c}}
                            w_x & w_y & w_z & v_x & v_y & v_z \\
                            \end{array}\right)\f]
 */
class Twist
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline Twist();
   explicit inline Twist( real init );
   explicit inline Twist( const Vec3& w, const Vec3& v );
            inline Twist( const Twist& twist );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   inline Twist&      operator= ( real rhs );
   inline Twist&      operator= ( const Twist& rhs );
   inline real&       operator[]( size_t index );
   inline const real& operator[]( size_t index )      const;
   inline const Twist operator- ()                    const;
   inline Twist&      operator+=( const Twist& rhs );
   inline Twist&      operator-=( const Twist& rhs );
   inline Twist&      operator*=( real rhs );
   inline Twist&      operator/=( real rhs );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline Vec3&       angular();
   inline const Vec3& angular() const;
   inline Vec3&       linear ();
   inline const Vec3& linear () const;

   inline Twist&      set( real a, real b, real c, real d, real e, real f );
   inline void        reset();

   inline real        length()    const;
   inline real        sqrLength() const;

   inline Twist&      scale( real scalar );

   inline bool        equal( real scalar        ) const;
   inline bool        equal( const Twist& twist ) const;

   inline void        swap( Twist& twist ) /* throw() */;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 w_;  //!< The angular velocity part of the twist.
   Vec3 v_;  //!< The linear velocity part of the twist.
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
/*!\brief The default constructor for Twist.
 *
 * All twist elements are initialized to the default value (i.e. 0 for integral data types).
 */
inline Twist::Twist()
   : w_()  // The angular velocity part of the twist
   , v_()  // The linear velocity part of the twist
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a homogenous initialization of all elements.
 *
 * \param init Initial value for all twist elements.
 */
inline Twist::Twist( real init )
   : w_( init )  // The angular velocity part of the twist
   , v_( init )  // The linear velocity part of the twist
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a direct initialization of the twist.
 *
 * \param w Initialization of the angular part of the twist.
 * \param v Initialization of the linear part of the twist.
 */
inline Twist::Twist( const Vec3& w, const Vec3& v )
   : w_( w )  // The angular velocity part of the twist
   , v_( v )  // The linear velocity part of the twist
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The copy constructor for Twist.
 *
 * \param twist Twist to be copied.
 *
 * The copy constructor is explicitly defined in order to enable/facilitate NRV optimization.
 */
inline Twist::Twist( const Twist& twist )
   : w_( twist.w_ )  // The angular velocity part of the twist
   , v_( twist.v_ )  // The linear velocity part of the twist
{}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Homogenous assignment to all twist elements.
 *
 * \param rhs Scalar value to be assigned to all twist elements.
 * \return Reference to the assigned twist.
 */
inline Twist& Twist::operator=( real rhs )
{
   w_ = rhs;
   v_ = rhs;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy assignment operator for Twist.
 *
 * \param rhs Twist to be copied.
 * \return Reference to the assigned twist.
 *
 * Explicit definition of a copy assignment operator for performance reasons.
 */
inline Twist& Twist::operator=( const Twist& rhs )
{
   // This implementation is faster than the synthesized default copy assignment operator and
   // faster than an implementation with the C library function 'memcpy' in combination with a
   // protection against self-assignment. Additionally, this version goes without a protection
   // against self-assignment.
   w_ = rhs.w_;
   v_ = rhs.v_;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the twist elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..5]\f$.
 * \return Reference to the accessed value.
 *
 * In case pe_USER_ASSERT() is active, this operator performs an index check.
 */
inline real& Twist::operator[]( size_t index )
{
   pe_USER_ASSERT( index < 6, "Invalid twist access index" );
   if( index < 3 ) return w_[index  ];
   else            return v_[index-3];
   //return *( reinterpret_cast<Type*>( this ) + index );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the twist elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..5]\f$.
 * \return Reference to the accessed value.
 *
 * In case pe_USER_ASSERT() is active, this operator performs an index check.
 */
inline const real& Twist::operator[]( size_t index ) const
{
   pe_USER_ASSERT( index < 6, "Invalid twist access index" );
   if( index < 3 ) return w_[index  ];
   else            return v_[index-3];
   //return *( reinterpret_cast<const Type*>( this ) + index );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unary minus operator for the inversion of a twist.
 *
 * \return The inverse of the twist.
 */
inline const Twist Twist::operator-() const
{
   return Twist( -w_, -v_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition assignment operator for the addition of two twists.
 *
 * \param rhs The right-hand side twist to be added to the twist.
 * \return Reference to the twist.
 */
inline Twist& Twist::operator+=( const Twist& rhs )
{
   w_ += rhs.w_;
   v_ += rhs.v_;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction assignment operator for the subtraction of two twists.
 *
 * \param rhs The right-hand side twist to be subtracted from the twist.
 * \return Reference to the twist.
 */
inline Twist& Twist::operator-=( const Twist& rhs )
{
   w_ -= rhs.w_;
   v_ -= rhs.v_;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication between a twist and
 *        a scalar value.
 *
 * \param rhs The right-hand side scalar value for the multiplication.
 * \return Reference to the twist.
 */
inline Twist& Twist::operator*=( real rhs )
{
   w_ *= rhs;
   v_ *= rhs;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Division assignment operator for the division of a twist by a scalar value.
 *
 * \param rhs The right-hand side scalar value for the division.
 * \return Reference to the twist.
 *
 * \b Note: A division by zero is only checked by an user assert.
 */
inline Twist& Twist::operator/=( real rhs )
{
   pe_USER_ASSERT( rhs != real(0), "Division by zero detected" );

   const real tmp( real(1)/rhs );
   w_ *= tmp;
   v_ *= tmp;
   return *this;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the angular part of the twist.
 *
 * \return The angular part of the twist.
 */
inline Vec3& Twist::angular()
{
   return w_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the angular part of the twist.
 *
 * \return The angular part of the twist.
 */
inline const Vec3& Twist::angular() const
{
   return w_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the linear part of the twist.
 *
 * \return The linear part of the twist.
 */
inline Vec3& Twist::linear()
{
   return v_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the linear part of the twist.
 *
 * \return The linear part of the twist.
 */
inline const Vec3& Twist::linear() const
{
   return v_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the value of the twist elements.
 *
 * \param a The value for the x-component of the angular velocity part.
 * \param b The value for the y-component of the angular velocity part.
 * \param c The value for the z-component of the angular velocity part.
 * \param d The value for the x-component of the linear velocity part.
 * \param e The value for the y-component of the linear velocity part.
 * \param f The value for the z-component of the linear velocity part.
 */
inline Twist& Twist::set( real a, real b, real c, real d, real e, real f )
{
   w_[0] = a;
   w_[1] = b;
   w_[2] = c;
   v_[0] = d;
   v_[1] = e;
   v_[2] = f;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reset to the default initial values.
 *
 * \return void
 */
inline void Twist::reset()
{
   w_.reset();
   v_.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the twist length.
 *
 * \return The length of the twist.
 */
inline real Twist::length() const
{
   return std::sqrt( sqrLength() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the twist square length.
 *
 * \return The square length of the twist.
 */
inline real Twist::sqrLength() const
{
   return w_.sqrLength() + v_.sqrLength();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scaling of the twist by the scalar value \a scalar.
 *
 * \param scalar The scalar value for the twist scaling.
 * \return Reference to the twist.
 */
inline Twist& Twist::scale( real scalar )
{
   w_ *= scalar;
   v_ *= scalar;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison of a twist and a scalar value.
 *
 * \param scalar The scalar value for the comparison.
 * \return \a true if all elements of the twist are equal to the scalar, \a false if not.
 *
 * If all values of the twist are equal to the scalar value, the equality test returns true,
 * otherwise false.
 */
inline bool Twist::equal( real scalar ) const
{
   if( v_ != scalar || w_ != scalar )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison of two twists.
 *
 * \param twist The right-hand side twist for the comparison.
 * \return \a true if the two twists are equal, \a false if not.
 */
inline bool Twist::equal( const Twist& twist ) const
{
   if( v_ != twist.v_ || w_ != twist.w_ )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two twists.
 *
 * \param twist The twist to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
inline void Twist::swap( Twist& twist ) /* throw() */
{
   pe::swap( v_, twist.v_ );
   pe::swap( w_, twist.w_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Twist operators */
//@{
inline bool operator==( const Twist& lhs, const Twist& rhs );
inline bool operator==( const Twist& twist, real scalar );
inline bool operator==( real scalar, const Twist& twist );

inline bool operator!=( const Twist& lhs, const Twist& rhs );
inline bool operator!=( const Twist& twist, real scalar );
inline bool operator!=( real scalar, const Twist& twist );

inline std::ostream& operator<<( std::ostream& os, const Twist& t );
inline std::istream& operator>>( std::istream& is, Twist& t );

inline void reset( Twist& twist );
inline void clear( Twist& twist );
inline void swap ( Twist& a, Twist& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of two twists.
 * \ingroup twist
 *
 * \param lhs The left-hand side twist for the comparison.
 * \param rhs The right-hand side twist for the comparison.
 * \return \a true if the two twists are equal, \a false if not.
 */
inline bool operator==( const Twist& lhs, const Twist& rhs )
{
   return lhs.equal( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a twist and a scalar value.
 * \ingroup twist
 *
 * \param twist The left-hand side twist for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if all elements of the twist are equal to the scalar, \a false if not.
 *
 * If all values of the twist are equal to the scalar value, the equality test returns true,
 * otherwise false.
 */
inline bool operator==( const Twist& twist, real scalar )
{
   return twist.equal( scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a scalar value and a twist.
 * \ingroup twist
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param twist The right-hand side twist for the comparison.
 * \return \a true if all elements of the twist are equal to the scalar, \a false if not.
 *
 * If all values of the twist are equal to the scalar value, the equality test returns true,
 * otherwise false.
 */
inline bool operator==( real scalar, const Twist& twist )
{
   return twist.equal( scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of two twists.
 * \ingroup twist
 *
 * \param lhs The left-hand side twist for the comparison.
 * \param rhs The right-hand side twist for the comparison.
 * \return \a true if the two twists are not equal, \a false if they are equal.
 */
inline bool operator!=( const Twist& lhs, const Twist& rhs )
{
   return !lhs.equal( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a twist and a scalar value.
 * \ingroup twist
 *
 * \param twist The left-hand side twist for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if at least one element of the twist is different from the scalar, \a false if not.
 *
 * If one value of the twist is inequal to the scalar value, the inequality test returns true,
 * otherwise false.
 */
inline bool operator!=( const Twist& twist, real scalar )
{
   return !twist.equal( scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a scalar value and a twist.
 * \ingroup twist
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param twist The right-hand side twist for the comparison.
 * \return \a true if at least one element of the twist is different from the scalar, \a false if not.
 *
 * If one value of the twist is inequal to the scalar value, the inequality test returns true,
 * otherwise false.
 */
inline bool operator!=( real scalar, const Twist& twist )
{
   return !twist.equal( scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for 6-dimensional twists.
 * \ingroup twist
 *
 * \param os Reference to the output stream.
 * \param t Reference to a constant twist object.
 * \return Reference to the output stream.
 */
inline std::ostream& operator<<( std::ostream& os, const Twist& t )
{
   return os << t.angular() << "," << t.linear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global input operator for 6-dimensional twists.
 * \ingroup twist
 *
 * \param is Reference to the input stream.
 * \param t Reference to a twist object.
 * \return The input stream.
 */
inline std::istream& operator>>( std::istream& is, Twist& t )
{
   if( !is ) return is;

   char b1, b2, b3, b4;      // Temporary variables for brackets
   char c1, c2, c3, c4, c5;  // Temporary vairables for commas
   real a(0), b(0), c(0), d(0), e(0), f(0);
   const std::istream::pos_type pos( is.tellg() );
   const std::istream::fmtflags oldFlags( is.flags() );

   // Setting the 'skip whitespaces' flag
   is >> std::skipws;

   // Extracting the vector
   if( !(is >> b1 >> a >> c1 >> b >> c2 >> c >> b2 >> c3 >> b3 >> d >> c4 >> e >> c5 >> f >> b4) ||
       b1 != '<' || c1 != ',' || c2 != ',' || c3 != ',' || c4 != ',' || c5 != ',' || b2 != '>' ) {
      is.clear();
      is.seekg( pos );
      is.setstate( std::istream::failbit );
      is.flags( oldFlags );
      return is;
   }

   // Transfering the input to the vector values
   t.set( a, b, c, d, e, f );

   // Resetting the flags
   is.flags( oldFlags );

   return is;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the given twist.
 * \ingroup twist
 *
 * \param twist The twist to be resetted.
 * \return void
 */
inline void reset( Twist& twist )
{
   twist.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the given twist.
 * \ingroup twist
 *
 * \param twist The twist to be cleared.
 * \return void
 *
 * Clearing a twist is equivalent to resetting it via the reset() function.
 */
inline void clear( Twist& twist )
{
   twist.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two twists.
 * \ingroup twist
 *
 * \param a The first twist to be swapped.
 * \param b The second twist to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
inline void swap( Twist& a, Twist& b ) /* throw() */
{
   a.swap( b );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Twist arithmetic operators */
//@{
inline const Twist operator+( const Twist& lhs, const Twist& rhs );
inline const Twist operator-( const Twist& lhs, const Twist& rhs );
inline real        operator*( const Twist& lhs, const Twist& rhs );
inline const Twist operator*( const Twist& twist, real scalar );
inline const Twist operator*( real scalar, const Twist& twist );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition operator for the addition of two twists.
 * \ingroup twist
 *
 * \param lhs The left-hand side twist for the twist addition.
 * \param rhs The right-hand side twist for the twist addition.
 * \return The sum of the two twists.
 */
inline const Twist operator+( const Twist& lhs, const Twist& rhs )
{
   return Twist( lhs.angular() + rhs.angular(), lhs.linear() + rhs.linear() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction operator for the subtraction of two twists.
 * \ingroup twist
 *
 * \param lhs The left-hand side twist for the twist subtraction.
 * \param rhs The right-hand side twist to be subtracted from the left-hand side twist.
 * \return The difference of the two twists.
 */
inline const Twist operator-( const Twist& lhs, const Twist& rhs )
{
   return Twist( lhs.angular() - rhs.angular(), lhs.linear() - rhs.linear() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the scalar product of two twists.
 * \ingroup twist
 *
 * \param lhs The left-hand side twist for the scalar product.
 * \param rhs The right-hand side twist for the scalar product.
 * \return The scalar product.
 */
inline real operator*( const Twist& lhs, const Twist& rhs )
{
   return trans( lhs.angular() ) * rhs.angular() + trans( lhs.linear() ) * rhs.linear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a twist and a scalar value.
 * \ingroup twist
 *
 * \param twist The left-hand side twist for the multiplication.
 * \param scalar The right-hand side scalar value for the multiplication.
 * \return The scaled result twist.
 */
inline const Twist operator*( const Twist& twist, real scalar )
{
   return Twist( twist.angular()*scalar, twist.linear()*scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a scalar value and a twist.
 * \ingroup twist
 *
 * \param scalar The left-hand side scalar value for the multiplication.
 * \param twist The right-hand side twist for the multiplication.
 * \return The scaled result twist.
 */
inline const Twist operator*( real scalar, const Twist& twist )
{
   return Twist( twist.angular()*scalar, twist.linear()*scalar );
}
//*************************************************************************************************

} // namespace pe

#endif
