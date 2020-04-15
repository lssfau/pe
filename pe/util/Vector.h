//=================================================================================================
/*!
 *  \file pe/util/Vector.h
 *  \brief Implementation of a vector container
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

#ifndef _PE_UTIL_VECTOR_H_
#define _PE_UTIL_VECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <vector>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector container.
 * \ingroup util
 *
 * The Vector class represents a vector container after the requirements of the Standard Library.
 * TODO
 */
template< typename T                      // Type of the vector elements
        , typename A=std::allocator<T> >  // Type of the allocator
class Vector
{
public:
   //**Type definitions****************************************************************************
   // STL naming conventions
   /*! \cond PE_INTERNAL */
   typedef std::vector<T,A>                             VectorType;
   typedef typename VectorType::value_type              value_type;
   typedef typename VectorType::pointer                 pointer;
   typedef typename VectorType::const_pointer           const_pointer;
   typedef typename VectorType::reference               reference;
   typedef typename VectorType::const_reference         const_reference;
   typedef typename VectorType::iterator                iterator;
   typedef typename VectorType::const_iterator          const_iterator;
   typedef typename VectorType::reverse_iterator        reverse_iterator;
   typedef typename VectorType::const_reverse_iterator  const_reverse_iterator;
   typedef typename VectorType::size_type               size_type;
   typedef typename VectorType::difference_type         difference_type;
   /*! \endcond */

   // pe naming convention
   typedef value_type              ValueType;             //!< Type of the vector elements.
   typedef pointer                 Pointer;               //!< Pointer to a non-const element.
   typedef const_pointer           ConstPointer;          //!< Pointer to a const element.
   typedef reference               Reference;             //!< Reference to a non-const element.
   typedef const_reference         ConstReference;        //!< Reference to a const element.
   typedef iterator                Iterator;              //!< Iterator over non-const elements.
   typedef const_iterator          ConstIterator;         //!< Iterator over const elements.
   typedef reverse_iterator        ReverseIterator;       //!< Reverse iterator over non-const elements.
   typedef const_reverse_iterator  ConstReverseIterator;  //!< Reverse iterator over const elements.
   typedef size_type               SizeType;              //!< Size type of the vector.
   typedef difference_type         DifferenceType;        //!< Difference type of the vector.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
                           explicit inline Vector();
                           explicit inline Vector( SizeType n, const T& value=T() );
   template< typename IT > explicit inline Vector( IT first, IT last );
                                    inline Vector( const Vector& vector );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Assignment operators************************************************************************
   /*!\name Assignment operators */
   //@{
   inline Vector& operator=( const Vector& vector );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline SizeType maxSize()  const;
   inline SizeType size()     const;
   inline SizeType capacity() const;
   inline bool     isEmpty()  const;
   //@}
   //**********************************************************************************************

   //**Access functions****************************************************************************
   /*!\name Access functions */
   //@{
   inline Reference      operator[]( SizeType index );
   inline ConstReference operator[]( SizeType index ) const;
   inline Reference      front();
   inline ConstReference front() const;
   inline Reference      back();
   inline ConstReference back()  const;
   //@}
   //**********************************************************************************************

   //**Iterator functions**************************************************************************
   /*!\name Iterator functions */
   //@{
   inline Iterator             begin();
   inline ConstIterator        begin() const;
   inline Iterator             end();
   inline ConstIterator        end()   const;

   inline ReverseIterator      rbegin();
   inline ConstReverseIterator rbegin() const;
   inline ReverseIterator      rend();
   inline ConstReverseIterator rend()   const;
   //@}
   //**********************************************************************************************

   //**Element functions***************************************************************************
   /*!\name Element functions */
   //@{
                           inline void     pushBack( const ValueType& value );
                           inline void     popBack();

                           inline void     assign( SizeType n, const ValueType& value );
   template< typename IT > inline void     assign( IT first, IT last );

                           inline Iterator insert( Iterator pos, const ValueType& value );
                           inline void     insert( Iterator pos, SizeType n, const ValueType& value );
   template< typename IT > inline void     insert( Iterator pos, IT first, IT last );

                           inline Iterator erase( Iterator pos );
                           inline Iterator erase( Iterator first, Iterator last );

                           inline void     clear();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void reserve( SizeType n );
   inline void resize ( SizeType n, const ValueType& value=ValueType() );
   inline void swap( Vector& vector ) /* throw() */;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   VectorType vector_;  //!< The wrapped vector container.
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
/*!\brief Default constructor for Vector.
 *
 * The default constructor creates an empty vector.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline Vector<T,A>::Vector()
   : vector_()  // The wrapped vector container
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a vector with a number of copies of the given element.
 *
 * \param n The number of elements to be initially created.
 * \param value The initial element for the vector.
 *
 * This constructor creates a vector with \a n copies of the given element \a value.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline Vector<T,A>::Vector( SizeType n, const T& value )
   : vector_( n, value )  // The wrapped vector container
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a vector from a range of elements.
 *
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 *
 * This constructor creates a vector with copies of the elements in the range \f$ [first,last) \f$.
 */
template< typename T     // Type of the vector elements
        , typename A >   // Type of the allocator
template< typename IT >  // Type of the iterators
inline Vector<T,A>::Vector( IT first, IT last )
   : vector_( first, last )  // The wrapped vector container
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy constructor for Vector.
 *
 * \param vector The vector to be copied.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline Vector<T,A>::Vector( const Vector& vector )
   : vector_( vector.vector_ )  // The wrapped vector container
{}
//*************************************************************************************************




//=================================================================================================
//
//  ASSIGNMENT OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Copy assignment operator for Vector.
 *
 * \param vector The vector to be copied.
 * \return Reference to the assigned vector.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline Vector<T,A>& Vector<T,A>::operator=( const Vector& vector )
{
   vector_ = vector.vector_;
   return *this;
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the maximum possible size of a vector.
 *
 * \return The maximum possible size.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::SizeType Vector<T,A>::maxSize() const
{
   return vector_.max_size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current size of the vector.
 *
 * \return The current size.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::SizeType Vector<T,A>::size() const
{
   return vector_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the capacity of the vector.
 *
 * \return The capacity.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::SizeType Vector<T,A>::capacity() const
{
   return vector_.capacity();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns \a true if the vector has no elements.
 *
 * \return \a true if the vector is empty, \a false if it is not.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline bool Vector<T,A>::isEmpty() const
{
   return vector_.empty();
}
//*************************************************************************************************




//=================================================================================================
//
//  ACCESS FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the vector elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Handle to the accessed element.
 *
 * \b Note: No runtime check is performed to insure the validity of the access index.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::Reference Vector<T,A>::operator[]( SizeType index )
{
   return vector_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the vector elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Handle to the accessed element.
 *
 * \b Note: No runtime check is performed to insure the validity of the access index.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::ConstReference Vector<T,A>::operator[]( SizeType index ) const
{
   return vector_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the first element of the vector.
 *
 * \return Handle to the first element.
 *
 * \b Note: No runtime check is performed if the first element exists!
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::Reference Vector<T,A>::front()
{
   pe_USER_ASSERT( size() > 0, "Vector is empty, invalid access to the front element" );
   return vector_.front();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the first element of the vector.
 *
 * \return Handle to the first element.
 *
 * \b Note: No runtime check is performed if the first element exists!
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::ConstReference Vector<T,A>::front() const
{
   pe_USER_ASSERT( size() > 0, "Vector is empty, invalid access to the front element" );
   return vector_.front();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the last element of the vector.
 *
 * \return Handle to the last element.
 *
 * \b Note: No runtime check is performed if the last element exists!
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::Reference Vector<T,A>::back()
{
   pe_USER_ASSERT( size() > 0, "Vector is empty, invalid access to the back element" );
   return vector_.back();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the last element of the vector.
 *
 * \return Handle to the last element.
 *
 * \b Note: No runtime check is performed if the last element exists!
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::ConstReference Vector<T,A>::back() const
{
   pe_USER_ASSERT( size() > 0, "Vector is empty, invalid access to the back element" );
   return vector_.back();
}
//*************************************************************************************************




//=================================================================================================
//
//  ITERATOR FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns an iterator to the beginning of the vector.
 *
 * \return Iterator to the beginning of the vector.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::Iterator Vector<T,A>::begin()
{
   return vector_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the beginning of the vector.
 *
 * \return Iterator to the beginning of the vector.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::ConstIterator Vector<T,A>::begin() const
{
   return vector_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last element of the vector.
 *
 * \return Iterator just past the last element of the vector.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::Iterator Vector<T,A>::end()
{
   return vector_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last element of the vector.
 *
 * \return Iterator just past the last element of the vector.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::ConstIterator Vector<T,A>::end() const
{
   return vector_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator to the last element of the vector.
 *
 * \return Reverse iterator to the last element of the vector.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::ReverseIterator Vector<T,A>::rbegin()
{
   return vector_.rbegin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator to the last element of the vector.
 *
 * \return Reverse iterator to the last element of the vector.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::ConstReverseIterator Vector<T,A>::rbegin() const
{
   return vector_.rbegin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator just one before the first element of the vector.
 *
 * \return Reverse iterator just one before the first element of the vector.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::ReverseIterator Vector<T,A>::rend()
{
   return vector_.rend();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator just one before the first element of the vector.
 *
 * \return Reverse iterator just one before the first element of the vector.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::ConstReverseIterator Vector<T,A>::rend() const
{
   return vector_.rend();
}
//*************************************************************************************************




//=================================================================================================
//
//  ELEMENT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding an element to the end of the vector.
 *
 * \param value The element to be added to the end of the vector.
 * \return void
 *
 * The pushBack function runs in constant time.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline void Vector<T,A>::pushBack( const ValueType& value )
{
   vector_.push_back( value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing an element from the end of the vector.
 *
 * \return void
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline void Vector<T,A>::popBack()
{
   vector_.pop_back();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assigning a given element to the vector.
 *
 * \param n The number of elements to be assigned.
 * \param value The element to be assigned to the vector.
 * \return void
 *
 * This function fills a vector with \a n copies of the given element \a value. Note that
 * the assignment completely changes the vector and that the resulting vector's size is the
 * same as the number of elements assigned. The assign function runs in linear time.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline void Vector<T,A>::assign( SizeType n, const ValueType& value )
{
   vector_.assign( n, value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assigning a range of elements to the vector.
 *
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 * \return void
 *
 * This functions assigns the elements in the range \f$ [first,last) \f$ to the vector.
 * All elements previously contained in the vector are removed. The assign function runs
 * in linear time.
 */
template< typename T     // Type of the vector elements
        , typename A >   // Type of the allocator
template< typename IT >  // Type of the iterators
inline void Vector<T,A>::assign( IT first, IT last )
{
   vector_.assign( first, last );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting an element into the vector.
 *
 * \param pos The position before which the element is inserted.
 * \param value The element to be inserted into the vector.
 * \return Iterator to the inserted element.
 *
 * The insert function runs in linear time. Note however that inserting elements into a vector
 * can be a relatively time-intensive operation.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::Iterator Vector<T,A>::insert( Iterator pos, const ValueType& value )
{
   return vector_.insert( pos, value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting a number of copies of the given element into the vector.
 *
 * \param pos The position before which the element is inserted.
 * \param n The number of elements to be inserted.
 * \param value The element to be inserted into the vector.
 * \return void
 *
 * This function inserts a specified number of copies of the given element before the location
 * specified by \a pos. The insert function runs in linear time. Note however that inserting
 * elements into a vector can be a relatively time-intensive operation.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline void Vector<T,A>::insert( Iterator pos, SizeType n, const ValueType& value )
{
   return vector_.insert( pos, n, value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting a range of elements into the vector.
 *
 * \param pos The position before which the elements are inserted.
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 * \return void
 *
 * This functions inserts the elements in the range \f$ [first,last) \f$ into the vector. The
 * insert function runs in linear time. Note however that inserting elements into a vector can
 * be a relatively time-intensive operation.
 */
template< typename T     // Type of the vector elements
        , typename A >   // Type of the allocator
template< typename IT >  // Type of the iterators
inline void Vector<T,A>::insert( Iterator pos, IT first, IT last )
{
   vector_.insert( pos, first, last );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single element from the vector.
 *
 * \param pos The position of the element to be removed.
 * \return Iterator to the element after the erased element.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::Iterator Vector<T,A>::erase( Iterator pos )
{
   return vector_.erase( pos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing the a range of elements.
 *
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 * \return Iterator to the element after the erased elements.
 *
 * This function erases all elements in the range \f$ [first,last) \f$ and returns an iterator
 * to the element after the erased elements.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline typename Vector<T,A>::Iterator Vector<T,A>::erase( Iterator first, Iterator last )
{
   return vector_.erase( first, last );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing all elements from the vector.
 *
 * \return void
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline void Vector<T,A>::clear()
{
   vector_.clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the minimum capacity of the vector.
 *
 * \param n The new minimum capacity of the vector.
 * \return void
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline void Vector<T,A>::reserve( SizeType n )
{
   vector_.reserve( n );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resizing the vector to the specified number of elements.
 *
 * \param n The new number of elements.
 * \param value Initialization value for new vector elements.
 * \return void
 *
 * This function resizes the vector to the specified number of elements. If \a n is smaller than
 * the current size of the vector, the vector is truncated, otherwise the vector is extended and
 * the new elements are populated with the given element \a value.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline void Vector<T,A>::resize( SizeType n, const ValueType& value )
{
   vector_.resize( n, value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two vectors.
 *
 * \param vector The vector to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline void Vector<T,A>::swap( Vector& vector ) /* throw() */
{
   vector_.swap( vector.vector_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Vector operators */
//@{
template< typename T, typename A >
inline bool operator==( const Vector<T,A>& lhs, const Vector<T,A>& rhs );

template< typename T, typename A >
inline bool operator!=( const Vector<T,A>& lhs, const Vector<T,A>& rhs );

template< typename T, typename A >
inline bool operator<( const Vector<T,A>& lhs, const Vector<T,A>& rhs );

template< typename T, typename A >
inline bool operator>( const Vector<T,A>& lhs, const Vector<T,A>& rhs );

template< typename T, typename A >
inline bool operator<=( const Vector<T,A>& lhs, const Vector<T,A>& rhs );

template< typename T, typename A >
inline bool operator>=( const Vector<T,A>& lhs, const Vector<T,A>& rhs );

template< typename T, typename A >
inline void swap( Vector<T,A>& a, Vector<T,A>& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between two vectors.
 *
 * \param lhs The left hand side vector.
 * \param rhs The right hand side vector.
 * \return \a true if the two vectors are equal, \a false if they are not.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline bool operator==( const Vector<T,A>& lhs, const Vector<T,A>& rhs )
{
   return lhs.size() == rhs.size() && std::equal( lhs.begin(), lhs.end(), rhs.begin() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between two vectors.
 *
 * \param lhs The left hand side vector.
 * \param rhs The right hand side vector.
 * \return \a true if the two vectors are inequal, \a false if they are not.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline bool operator!=( const Vector<T,A>& lhs, const Vector<T,A>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Less-than comparison between two vectors.
 *
 * \param lhs The left hand side vector.
 * \param rhs The right hand side vector.
 * \return \a true if \a lhs is lexicographically smaller than \a rhs, \a false if not.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline bool operator<( const Vector<T,A>& lhs, const Vector<T,A>& rhs )
{
   return std::lexicographical_compare( lhs.begin(), lhs.end(), rhs.begin(), rhs.end() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Greater-than comparison between two vectors.
 *
 * \param lhs The left hand side vector.
 * \param rhs The right hand side vector.
 * \return \a true if \a lhs is lexicographically greater than \a rhs, \a false if not.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline bool operator>( const Vector<T,A>& lhs, const Vector<T,A>& rhs )
{
   return rhs < lhs;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Less-or-equal-than comparison between two vectors.
 *
 * \param lhs The left hand side vector.
 * \param rhs The right hand side vector.
 * \return \a true if \a lhs is lexicographically less-or-equal than \a rhs, \a false if not.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline bool operator<=( const Vector<T,A>& lhs, const Vector<T,A>& rhs )
{
   return !( rhs < lhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Greater-or-equal-than comparison between two vectors.
 *
 * \param lhs The left hand side vector.
 * \param rhs The right hand side vector.
 * \return \a true if \a lhs is lexicographically greater-or-equal than \a rhs, \a false if not.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline bool operator>=( const Vector<T,A>& lhs, const Vector<T,A>& rhs )
{
   return !( lhs < rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two vectors.
 *
 * \param a The first vector to be swapped.
 * \param b The second vector to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename T    // Type of the vector elements
        , typename A >  // Type of the allocator
inline void swap( Vector<T,A>& a, Vector<T,A>& b ) /* throw() */
{
   a.swap( b );
}
//*************************************************************************************************

} // namespace pe

#endif
