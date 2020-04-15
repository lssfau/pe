//=================================================================================================
/*!
 *  \file pe/util/SmartIterator.h
 *  \brief Iterator class for smart pointer vectors
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

#ifndef _PE_UTIL_SMARTITERATOR_H_
#define _PE_UTIL_SMARTITERATOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iterator>
#include <pe/util/constraints/Comparable.h>
#include <pe/util/constraints/Convertible.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of an iterator for smart pointer vectors.
 * \ingroup util
 *
 * The SmartIterator class follows the example of the random-access iterator classes of the STL.
 * However, the focus of this iterator implementation is the use with smart pointers. Since the
 * physics engine relies on smart pointers like the SharedPtr, this implementation eases the use
 * of iterators over a range of smart pointers and improves the semantics on these pointers.\n
 *
 * In contrast to the STL iterators, the SmartIterator class slightly changes the meaning of the
 * access operators. Consider the following example:

   \code
   // Definition of class A
   class A
   {
    public:
      A( int i=0 ):i_(i) {}

      void set( int i )       { i_ = i; }
      int  get()        const { return i_; }

    private:
      int i_;
   };

   // Definition of a smart pointer and smart pointer vector for class A
   typedef pe::SharedPtr<A>                  APtr;
   typedef pe::SmartVector<pe::SharedPtr,A>  AVector;

   AVector vector;
   AVector::Iterator it = vector.begin();

   // The subscript operator returns a handle to the underlying object
   APtr a1 = it[0];

   // The dereference operator returns a handle to the underlying object
   APtr a2 = *it;

   // The member access operator offers direct access to the underlying object
   it->set( 2 );
   \endcode

 * The constant iterators (iterator over constant objects) prohibit the access to non-const
 * member functions. Therefore the following operation results in a compile-time error:

   \code
   AVector vector;
   AVector::ConstIterator it = vector.begin();

   it->set( 2 );  // Compile-time error!
   \endcode
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
class SmartIterator
{
public:
   //**Type definitions****************************************************************************
   // pe naming convention
   typedef const S<T>      ValueType;       //!< Type of the underlying smart pointers.
   typedef ValueType&      ReferenceType;   //!< Reference return type.
   typedef ValueType*      IteratorType;    //!< Type of the internal pointer.
   typedef std::ptrdiff_t  DifferenceType;  //!< Difference between two iterators.

   // STL iterator requirements
   /*! \cond PE_INTERNAL */
   typedef std::random_access_iterator_tag  iterator_category;
   typedef ValueType                        value_type;
   typedef DifferenceType                   difference_type;
   typedef IteratorType                     pointer;
   typedef ReferenceType                    reference;
   /*! \endcond */
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   template< typename Other >
   inline SmartIterator( const S<Other>* it );

   template< typename Other >
   inline SmartIterator( const SmartIterator<S,Other>& it );

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
   inline SmartIterator& operator++();
   inline SmartIterator  operator++( int );
   inline SmartIterator& operator--();
   inline SmartIterator  operator--( int );
   inline SmartIterator& operator+=( DifferenceType n );
   inline SmartIterator  operator+ ( DifferenceType n )         const;
   inline SmartIterator& operator-=( DifferenceType n );
   inline SmartIterator  operator- ( DifferenceType n )         const;
   inline DifferenceType operator- ( const SmartIterator& it ) const;
   //@}
   //**********************************************************************************************

   //**Access operators****************************************************************************
   /*!\name Access operators */
   //@{
   inline ReferenceType operator[]( DifferenceType n ) const;
   inline ReferenceType operator*()                    const;
   inline ValueType     operator->()                   const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline IteratorType base() const;
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
/*!\brief Standard constructor for SmartIterator.
 *
 * \param it The value of the iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
template< typename Other >              // Type of the foreign smart pointer elements
inline SmartIterator<S,T>::SmartIterator( const S<Other>* it )
   : it_( reinterpret_cast<IteratorType>( it ) )
{
   pe_CONSTRAINT_MUST_BE_CONVERTIBLE( Other*, T* );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different SmartIterator instances.
 *
 * \param it The foreign SmartIterator instance to be copied.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
template< typename Other >              // Type of the foreign smart pointer elements
inline SmartIterator<S,T>::SmartIterator( const SmartIterator<S,Other>& it )
   : it_( reinterpret_cast<IteratorType>( it.base() ) )
{
   pe_CONSTRAINT_MUST_BE_CONVERTIBLE( Other*, T* );
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Pre-increment operator.
 *
 * \return Reference to the incremented smart pointer iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline SmartIterator<S,T>& SmartIterator<S,T>::operator++()
{
   ++it_;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Post-increment operator.
 *
 * \return The incremented smart pointer iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline SmartIterator<S,T> SmartIterator<S,T>::operator++( int )
{
   SmartIterator tmp( *this );
   ++it_;
   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Pre-decrement operator.
 *
 * \return Reference to the decremented smart pointer iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline SmartIterator<S,T>& SmartIterator<S,T>::operator--()
{
   --it_;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Post-decrement operator.
 *
 * \return The decremented smart pointer iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline SmartIterator<S,T> SmartIterator<S,T>::operator--( int )
{
   SmartIterator tmp( *this );
   --it_;
   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Shifting the iterator by \a n elements to the higher elements.
 *
 * \param n The number of elements.
 * \return Reference to the shifted smart pointer iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline SmartIterator<S,T>& SmartIterator<S,T>::operator+=( DifferenceType n )
{
   it_ += n;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Shifting the iterator by \a n elements to the higher elements.
 *
 * \param n The number of elements.
 * \return The shifted smart pointer iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline SmartIterator<S,T> SmartIterator<S,T>::operator+( DifferenceType n ) const
{
   return SmartIterator( it_ + n );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Shifting the iterator by \a n elements to the lower elements.
 *
 * \param n The number of elements.
 * \return Reference to the shifted smart pointer iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline SmartIterator<S,T>& SmartIterator<S,T>::operator-=( DifferenceType n )
{
   it_ -= n;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Shifting the iterator by \a n elements to the lower elements.
 *
 * \param n The number of elements.
 * \return The shifted smart pointer iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline SmartIterator<S,T> SmartIterator<S,T>::operator-( DifferenceType n ) const
{
   return SmartIterator( it_ - n );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating the number of elements between two pointer iterators.
 *
 * \param it The right hand side iterator.
 * \return The number of elements between the two smart pointer iterators.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline typename SmartIterator<S,T>::DifferenceType SmartIterator<S,T>::operator-( const SmartIterator& it ) const
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
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline typename SmartIterator<S,T>::ReferenceType SmartIterator<S,T>::operator[]( DifferenceType index ) const
{
   return it_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a handle to the element at the current iterator position.
 *
 * \return Handle to the element at the current iterator position.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline typename SmartIterator<S,T>::ReferenceType SmartIterator<S,T>::operator*() const
{
   return *it_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Direct access to the element at the current iterator position.
 *
 * \return Reference to the element at the current iterator position.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline typename SmartIterator<S,T>::ValueType SmartIterator<S,T>::operator->() const
{
   return *it_;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Access to the underlying member of the smart pointer iterator.
 *
 * \return Pointer to the current memory location.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T >                  // Type of the smart pointer elements
inline typename SmartIterator<S,T>::IteratorType SmartIterator<S,T>::base() const
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
/*!\name SmartIterator operators */
//@{
template< template< typename > class S, typename L, typename R >
inline bool operator==( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs );

template< template< typename > class S, typename L, typename R >
inline bool operator!=( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs );

template< template< typename > class S, typename L, typename R >
inline bool operator<( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs );

template< template< typename > class S, typename L, typename R >
inline bool operator>( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs );

template< template< typename > class S, typename L, typename R >
inline bool operator<=( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs );

template< template< typename > class S, typename L, typename R >
inline bool operator>=( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between two SmartIterator objects.
 *
 * \param lhs The left hand side smart pointer iterator.
 * \param rhs The right hand side smart pointer iterator.
 * \return \a true if the iterators point to the same element, \a false if not.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename L                    // Type of the smart pointer element of the left operand
        , typename R >                  // Type of the smart pointer element of the right operand
inline bool operator==( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs )
{
   pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
   return static_cast<const void*>( lhs.base() ) == static_cast<const void*>( rhs.base() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between two SmartIterator objects.
 *
 * \param lhs The left hand side smart pointer iterator.
 * \param rhs The right hand side smart pointer iterator.
 * \return \a true if the iterators don't point to the same element, \a false if they do.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename L                    // Type of the smart pointer element of the left operand
        , typename R >                  // Type of the smart pointer element of the right operand
inline bool operator!=( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs )
{
   pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
   return static_cast<const void*>( lhs.base() ) != static_cast<const void*>( rhs.base() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Less-than comparison between two SmartIterator objects.
 *
 * \param lhs The left hand side smart pointer iterator.
 * \param rhs The right hand side smart pointer iterator.
 * \return \a true if the left hand side iterator points to a lower element, \a false if not.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename L                    // Type of the smart pointer element of the left operand
        , typename R >                  // Type of the smart pointer element of the right operand
inline bool operator<( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs )
{
   pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
   return static_cast<const void*>( lhs.base() ) < static_cast<const void*>( rhs.base() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Greater-than comparison between two SmartIterator objects.
 *
 * \param lhs The left hand side smart pointer iterator.
 * \param rhs The right hand side smart pointer iterator.
 * \return \a true if the left hand side iterator points to a higher element, \a false if not.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename L                    // Type of the smart pointer element of the left operand
        , typename R >                  // Type of the smart pointer element of the right operand
inline bool operator>( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs )
{
   pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
   return static_cast<const void*>( lhs.base() ) > static_cast<const void*>( rhs.base() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Less-or-equal-than comparison between two SmartIterator objects.
 *
 * \param lhs The left hand side smart pointer iterator.
 * \param rhs The right hand side smart pointer iterator.
 * \return \a true if the left hand side iterator points to a lower or the same element, \a false if not.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename L                    // Type of the smart pointer element of the left operand
        , typename R >                  // Type of the smart pointer element of the right operand
inline bool operator<=( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs )
{
   pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
   return static_cast<const void*>( lhs.base() ) <= static_cast<const void*>( rhs.base() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Greater-or-equal-than comparison between two SmartIterator objects.
 *
 * \param lhs The left hand side smart pointer iterator.
 * \param rhs The right hand side smart pointer iterator.
 * \return \a true if the left hand side iterator points to a higher or the same element, \a false if not.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename L                    // Type of the smart pointer element of the left operand
        , typename R >                  // Type of the smart pointer element of the right operand
inline bool operator>=( const SmartIterator<S,L>& lhs, const SmartIterator<S,R>& rhs )
{
   pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
   return static_cast<const void*>( lhs.base() ) >= static_cast<const void*>( rhs.base() );
}
//*************************************************************************************************

} // namespace pe

#endif
