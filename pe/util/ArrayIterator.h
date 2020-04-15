//=================================================================================================
/*!
 *  \file pe/util/ArrayIterator.h
 *  \brief Iterator class for array data structures
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

#ifndef _PE_UTIL_ARRAYITERATOR_H_
#define _PE_UTIL_ARRAYITERATOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iterator>
#include <pe/util/Null.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of an iterator for array data structures.
 * \ingroup util
 *
 * The ArrayIterator class implements the classical random-acess iterator concept from the STL.
 * It provides all required functionality for a random access iterator like forward and backward
 * movements in the range of elements, integer arithmetic with iterators, and equality, inequality
 * and less-than, less-or-equal-than, greater-than, and greater-or-equal-than comparisons between
 * iterators.
 */
template< typename Type >
class ArrayIterator
{
public:
   //**Type definitions****************************************************************************
   // pe naming convention
   typedef Type*                            IteratorType;      //!< Type of the iterator.
   typedef std::random_access_iterator_tag  IteratorCategory;  //!< Category of the iterator.
   typedef Type                             ValueType;         //!< Type of the underlying elements.
   typedef ValueType*                       PointerType;       //!< Pointer return type.
   typedef ValueType&                       ReferenceType;     //!< Reference return type.
   typedef std::ptrdiff_t                   DifferenceType;    //!< Difference between two iterators.

   // STL iterator requirements
   /*! \cond PE_INTERNAL */
   typedef IteratorType      iterator_type;
   typedef IteratorCategory  iterator_category;
   typedef ValueType         value_type;
   typedef PointerType       pointer;
   typedef ReferenceType     reference;
   typedef DifferenceType    difference_type;
   /*! \endcond */
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
            inline ArrayIterator();
   explicit inline ArrayIterator( const IteratorType& it );

   template< typename Other >
   inline ArrayIterator( const ArrayIterator<Other>& it );

   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   inline ArrayIterator& operator++();
   inline ArrayIterator  operator++( int );
   inline ArrayIterator& operator--();
   inline ArrayIterator  operator--( int );
   inline ArrayIterator& operator+=( DifferenceType n );
   inline ArrayIterator  operator+ ( DifferenceType n )        const;
   inline ArrayIterator& operator-=( DifferenceType n );
   inline ArrayIterator  operator- ( DifferenceType n )        const;
   inline DifferenceType operator- ( const ArrayIterator& it ) const;
   //@}
   //**********************************************************************************************

   //**Access operators****************************************************************************
   /*!\name Access operators */
   //@{
   inline ReferenceType operator[]( DifferenceType n ) const;
   inline ReferenceType operator*()                    const;
   inline PointerType   operator->()                   const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline const IteratorType& base() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   IteratorType it_;  //!< Pointer to the current memory location.
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
/*!\brief Default constructor for ArrayIterator.
 */
template< typename Type >
inline ArrayIterator<Type>::ArrayIterator()
   : it_(NULL)  // Pointer to the current memory location
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Standard constructor for ArrayIterator.
 *
 * \param it The value of the iterator.
 */
template< typename Type >
inline ArrayIterator<Type>::ArrayIterator( const IteratorType& it )
   : it_(it)  // Pointer to the current memory location
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different ArrayIterator instances.
 *
 * \param it The foreign ArrayIterator instance to be copied.
 */
template< typename Type >
template< typename Other >
inline ArrayIterator<Type>::ArrayIterator( const ArrayIterator<Other>& it )
   : it_( it.base() )  // Pointer to the current memory location
{}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Pre-increment operator.
 *
 * \return Reference to the incremented array iterator.
 */
template< typename Type >
inline ArrayIterator<Type>& ArrayIterator<Type>::operator++()
{
   ++it_;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Post-increment operator.
 *
 * \return The incremented array iterator.
 */
template< typename Type >
inline ArrayIterator<Type> ArrayIterator<Type>::operator++( int )
{
   ArrayIterator tmp( *this );
   ++it_;
   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Pre-decrement operator.
 *
 * \return Reference to the decremented array iterator.
 */
template< typename Type >
inline ArrayIterator<Type>& ArrayIterator<Type>::operator--()
{
   --it_;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Post-decrement operator.
 *
 * \return The decremented array iterator.
 */
template< typename Type >
inline ArrayIterator<Type> ArrayIterator<Type>::operator--( int )
{
   ArrayIterator tmp( *this );
   --it_;
   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Shifting the iterator by \a n elements to the higher elements.
 *
 * \param n The number of elements.
 * \return Reference to the shifted array iterator.
 */
template< typename Type >
inline ArrayIterator<Type>& ArrayIterator<Type>::operator+=( DifferenceType n )
{
   it_ += n;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Shifting the iterator by \a n elements to the higher elements.
 *
 * \param n The number of elements.
 * \return The shifted array iterator.
 */
template< typename Type >
inline ArrayIterator<Type> ArrayIterator<Type>::operator+( DifferenceType n ) const
{
   return ArrayIterator( it_ + n );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Shifting the iterator by \a n elements to the lower elements.
 *
 * \param n The number of elements.
 * \return Reference to the shifted array iterator.
 */
template< typename Type >
inline ArrayIterator<Type>& ArrayIterator<Type>::operator-=( DifferenceType n )
{
   it_ -= n;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Shifting the iterator by \a n elements to the lower elements.
 *
 * \param n The number of elements.
 * \return The shifted array iterator.
 */
template< typename Type >
inline ArrayIterator<Type> ArrayIterator<Type>::operator-( DifferenceType n ) const
{
   return ArrayIterator( it_ - n );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating the number of elements between two array iterators.
 *
 * \param it The right-hand side iterator.
 * \return The number of elements between the two array iterators.
 */
template< typename Type >
inline typename ArrayIterator<Type>::DifferenceType ArrayIterator<Type>::operator-( const ArrayIterator& it ) const
{
   return it_ - it.it_;
}
//*************************************************************************************************




//=================================================================================================
//
//  ACCESS OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Subscript operator for the direct element access.
 *
 * \param index Access index. Accesses the element \a index elements away from the current iterator position.
 * \return Handle to the accessed element.
 */
template< typename Type >
inline typename ArrayIterator<Type>::ReferenceType ArrayIterator<Type>::operator[]( DifferenceType index ) const
{
   return it_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a handle to the element at the current iterator position.
 *
 * \return Handle to the element at the current iterator position.
 */
template< typename Type >
inline typename ArrayIterator<Type>::ReferenceType ArrayIterator<Type>::operator*() const
{
   return *it_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Direct access to the element at the current iterator position.
 *
 * \return Reference to the element at the current iterator position.
 */
template< typename Type >
inline typename ArrayIterator<Type>::PointerType ArrayIterator<Type>::operator->() const
{
   return it_;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Access to the underlying member of the array iterator.
 *
 * \return Pointer to the current memory location.
 */
template< typename Type >
inline const typename ArrayIterator<Type>::IteratorType& ArrayIterator<Type>::base() const
{
   return it_;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name ArrayIterator operators */
//@{
template< typename TypeL, typename TypeR >
inline bool operator==( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs );

template< typename TypeL, typename TypeR >
inline bool operator!=( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs );

template< typename TypeL, typename TypeR >
inline bool operator<( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs );

template< typename TypeL, typename TypeR >
inline bool operator>( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs );

template< typename TypeL, typename TypeR >
inline bool operator<=( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs );

template< typename TypeL, typename TypeR >
inline bool operator>=( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between two ArrayIterator objects.
 *
 * \param lhs The left-hand side array iterator.
 * \param rhs The right-hand side array iterator.
 * \return \a true if the iterators point to the same element, \a false if not.
 */
template< typename TypeL, typename TypeR >
inline bool operator==( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs )
{
   return lhs.base() == rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between two ArrayIterator objects.
 *
 * \param lhs The left-hand side array iterator.
 * \param rhs The right-hand side array iterator.
 * \return \a true if the iterators don't point to the same element, \a false if they do.
 */
template< typename TypeL, typename TypeR >
inline bool operator!=( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs )
{
   return lhs.base() != rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Less-than comparison between two ArrayIterator objects.
 *
 * \param lhs The left-hand side array iterator.
 * \param rhs The right-hand side array iterator.
 * \return \a true if the left-hand side iterator points to a lower element, \a false if not.
 */
template< typename TypeL, typename TypeR >
inline bool operator<( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs )
{
   return lhs.base() < rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Greater-than comparison between two ArrayIterator objects.
 *
 * \param lhs The left-hand side array iterator.
 * \param rhs The right-hand side array iterator.
 * \return \a true if the left-hand side iterator points to a higher element, \a false if not.
 */
template< typename TypeL, typename TypeR >
inline bool operator>( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs )
{
   return lhs.base() > rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Less-or-equal-than comparison between two ArrayIterator objects.
 *
 * \param lhs The left-hand side array iterator.
 * \param rhs The right-hand side array iterator.
 * \return \a true if the left-hand side iterator points to a lower or the same element, \a false if not.
 */
template< typename TypeL, typename TypeR >
inline bool operator<=( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs )
{
   return lhs.base() <= rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Greater-or-equal-than comparison between two ArrayIterator objects.
 *
 * \param lhs The left-hand side array iterator.
 * \param rhs The right-hand side array iterator.
 * \return \a true if the left-hand side iterator points to a higher or the same element, \a false if not.
 */
template< typename TypeL, typename TypeR >
inline bool operator>=( const ArrayIterator<TypeL>& lhs, const ArrayIterator<TypeR>& rhs )
{
   return lhs.base() >= rhs.base();
}
//*************************************************************************************************

} // namespace pe

#endif
