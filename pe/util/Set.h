//=================================================================================================
/*!
 *  \file pe/util/Set.h
 *  \brief Implementation of a set associative container
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

#ifndef _PE_UTIL_SET_H_
#define _PE_UTIL_SET_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <set>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a set associative container.
 * \ingroup util
 *
 * The Set class represents a set associative container after the requirements of the Standard
 * Library.
 * TODO
 */
template< typename K                      // Type of the set elements
        , typename C=std::less<K>         // Comparison function object type
        , typename A=std::allocator<K> >  // Type of the allocator
class Set
{
public:
   public:
   //**Type definitions****************************************************************************
   // STL naming conventions
   /*! \cond PE_INTERNAL */
   typedef std::set<K,C,A>                           SetType;
   typedef typename SetType::key_type                key_type;
   typedef typename SetType::value_type              value_type;
   typedef typename SetType::key_compare             key_compare;
   typedef typename SetType::value_compare           value_compare;
   typedef typename SetType::allocator_type          allocator_type;
   typedef typename SetType::pointer                 pointer;
   typedef typename SetType::const_pointer           const_pointer;
   typedef typename SetType::reference               reference;
   typedef typename SetType::const_reference         const_reference;
   typedef typename SetType::iterator                iterator;
   typedef typename SetType::const_iterator          const_iterator;
   typedef typename SetType::reverse_iterator        reverse_iterator;
   typedef typename SetType::const_reverse_iterator  const_reverse_iterator;
   typedef typename SetType::size_type               size_type;
   typedef typename SetType::difference_type         difference_type;
   /*! \endcond */

   // pe naming convention
   typedef key_type                KeyType;               //!< Type of the set keys.
   typedef value_type              ValueType;             //!< Type of the set elements.
   typedef key_compare             KeyCompare;            //!< Type of the key comparator.
   typedef value_compare           ValueCompare;          //!< Type of the value comparator.
   typedef allocator_type          AllocatorType;         //!< Type of the allocator.
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

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline Set();
            inline Set( const Set& set );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Assignment operators************************************************************************
   /*!\name Assignment operators */
   //@{
   inline Set& operator=( const Set& set );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline SizeType maxSize()                   const;
   inline SizeType size()                      const;
   inline SizeType count( const KeyType& key ) const;
   inline bool     isEmpty()                   const;
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
                           inline std::pair<Iterator,bool> insert( const ValueType& value );
                           inline Iterator insert( Iterator pos, const ValueType& value );
   template< typename IT > inline void     insert( IT first, IT last );

                           inline void     erase( Iterator pos );
                           inline SizeType erase( const KeyType& key );
                           inline void     erase( Iterator first, Iterator last );

                           inline void     clear();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline Iterator                     find      ( const KeyType& key );
   inline std::pair<Iterator,Iterator> equalRange( const KeyType& key );
   inline Iterator                     lowerBound( const KeyType& key );
   inline Iterator                     upperBound( const KeyType& key );
   inline KeyCompare                   keyComp   () const;
   inline ValueCompare                 valueComp () const;
   inline void                         swap( Set& set ) /* throw() */;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   SetType set_;  //!< The wrapped set container.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   template< typename K2, typename C2, typename A2 >
   friend bool operator==( const Set<K2,C2,A2>& lhs, const Set<K2,C2,A2>& rhs );

   template< typename K2, typename C2, typename A2 >
   friend bool operator<( const Set<K2,C2,A2>& lhs, const Set<K2,C2,A2>& rhs );
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
/*!\brief Default constructor for Set.
 *
 * The default constructor creates an empty set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline Set<K,C,A>::Set()
   : set_()  // The wrapped set container.
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy constructor for Set.
 *
 * \param set The set to be copied.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline Set<K,C,A>::Set( const Set& set )
   : set_( set.set_ )  // The wrapped set container
{}
//*************************************************************************************************




//=================================================================================================
//
//  ASSIGNMENT OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Copy assignment operator for Set.
 *
 * \param set The set to be copied.
 * \return Reference to the assigned set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline Set<K,C,A>& Set<K,C,A>::operator=( const Set& set )
{
   set_ = set.set_;
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the maximum possible size of a set.
 *
 * \return The maximum possible size.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::SizeType Set<K,C,A>::maxSize() const
{
   return set_.max_size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current size of the set.
 *
 * \return The current size.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::SizeType Set<K,C,A>::size() const
{
   return set_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Counts the number of elements for the given key.
 *
 * \param key Key of the element(s) to be counted.
 * \return The number of elements located by the given key.
 *
 * This function will always either return 1 (in case the element is present) or 0 (in case
 * the element is not present.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::SizeType Set<K,C,A>::count( const KeyType& key ) const
{
   return set_.count( key );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns \a true if the set has no elements.
 *
 * \return \a true if the set is empty, \a false if it is not.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline bool Set<K,C,A>::isEmpty() const
{
   return set_.empty();
}
//*************************************************************************************************




//=================================================================================================
//
//  ITERATOR FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns an iterator to the beginning of the set.
 *
 * \return Iterator to the beginning of the set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::Iterator Set<K,C,A>::begin()
{
   return set_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the beginning of the set.
 *
 * \return Iterator to the beginning of the set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::ConstIterator Set<K,C,A>::begin() const
{
   return set_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last element of the set.
 *
 * \return Iterator just past the last element of the set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::Iterator Set<K,C,A>::end()
{
   return set_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last element of the set.
 *
 * \return Iterator just past the last element of the set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::ConstIterator Set<K,C,A>::end() const
{
   return set_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator to the last element of the set.
 *
 * \return Reverse iterator to the last element of the set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::ReverseIterator Set<K,C,A>::rbegin()
{
   return set_.rbegin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator to the last element of the set.
 *
 * \return Reverse iterator to the last element of the set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::ConstReverseIterator Set<K,C,A>::rbegin() const
{
   return set_.rbegin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator just one before the first element of the set.
 *
 * \return Reverse iterator just one before the first element of the set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::ReverseIterator Set<K,C,A>::rend()
{
   return set_.rend();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator just one before the first element of the set.
 *
 * \return Reverse iterator just one before the first element of the set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::ConstReverseIterator Set<K,C,A>::rend() const
{
   return set_.rend();
}
//*************************************************************************************************




//=================================================================================================
//
//  ELEMENT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Attempts to insert an element into the set.
 *
 * \param value The element to be inserted into the set.
 * \return Pair containing the position of the (possibly) inserted element and a bool.
 *
 * This function attempts to insert an element into the set. Since a set relies on unique keys,
 * the element is only inserted if it is not already present in the set. The return value of
 * the function is a pair, of which the first element is an iterator that points to the possibly
 * inserted element, and the second is a boolean value that is \a true in case the element was
 * actually inserted and \a false if it wasn't. The insertion runs in logarithmic time.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline std::pair<typename Set<K,C,A>::Iterator,bool> Set<K,C,A>::insert( const ValueType& value )
{
   return set_.insert( value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting an element into the set.
 *
 * \param pos Iterator that serves as a hint as to where the element should be inserted.
 * \param value The element to be inserted into the set.
 * \return Iterator to the inserted element.
 *
 * This function is not concerned about whether the insertion tool place and thus does not
 * return a boolean like the single-argument insert() does. Note that the first parameter
 * is only a hint and can potentially improve the performance of the insertion process. A
 * bad hint would cause no gains in efficiency. The insertion process runs in logarithmic
 * time (in case the hint is not taken).
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::Iterator Set<K,C,A>::insert( Iterator pos, const ValueType& value )
{
   return set_.insert( pos, value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting a range of elements into the set.
 *
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 * \return void
 *
 * This functions inserts the elements in the range \f$ [first,last) \f$ into the set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
template< typename IT >  // Type of the iterators
inline void Set<K,C,A>::insert( IT first, IT last )
{
   set_.insert( first, last );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single element from the set.
 *
 * \param pos The position of the element to be removed.
 * \return void
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline void Set<K,C,A>::erase( Iterator pos )
{
   set_.erase( pos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Erases elements according to the provided key.
 *
 * \param key Key of the element(s) to be erased.
 * \return The number of elements erased.
 *
 * This function erases all the elements located by the given key from the set.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::SizeType Set<K,C,A>::erase( const KeyType& key )
{
   return set_.erase( key );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a range of elements.
 *
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 * \return void
 *
 * This function erases all elements in the range \f$ [first,last) \f$.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline void Set<K,C,A>::erase( Iterator first, Iterator last )
{
   set_.erase( first, last );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing all elements from the set.
 *
 * \return void
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline void Set<K,C,A>::clear()
{
   set_.clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Locating an element in the set.
 *
 * \param key The element to be located.
 * \return Iterator to the located element.
 *
 * This function locates the element matching the given key \a key. In case the element is
 * found in the set, an iterator to the element is returned. In case the element is not
 * found, the function returns the past-the-end iterator (see end()).
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::Iterator Set<K,C,A>::find( const KeyType& key )
{
   return set_.find( key );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finds a subsequence matching the given key.
 *
 * \param key The key to be located.
 * \return Pair of iterators that point to the detected subsequence.
 *
 * This function returns a pair of iterators to the subsequence of elements equal to key.
 * For the Set, this function will always return a subsequence of length 1 (in case the
 * key is present) or 0 (in case the key is not present).
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline std::pair<typename Set<K,C,A>::Iterator,typename Set<K,C,A>::Iterator>
   Set<K,C,A>::equalRange( const KeyType& key )
{
   return set_.equal_range( key );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finds the beginning of a subsequence matching the given key.
 *
 * \param key The key to be located.
 * \return Iterator to the first element equal or greater than the given key.
 *
 * This function returns the first element of a subsequence of elements that matches the
 * given key.  If unsuccessful it returns an iterator pointing to the first element that
 * has a greater value than given key or end() if no such element exists.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::Iterator Set<K,C,A>::lowerBound( const KeyType& key )
{
   return set_.lower_bound( key );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finds the end of a subsequence matching the given key.
 *
 * \param key The key to be located.
 * \return Iterator to the first element greater than key.
 *
 * This function returns an iterator to the first element greater than the given key or
 * end().
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::Iterator Set<K,C,A>::upperBound( const KeyType& key )
{
   return set_.upper_bound( key );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the key comparison object of the set.
 *
 * \return The key comparsion object.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::KeyCompare Set<K,C,A>::keyComp() const
{
   return set_.key_comp();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the value comparison object of the set.
 *
 * \return The value comparsion object.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline typename Set<K,C,A>::ValueCompare Set<K,C,A>::valueComp() const
{
   return set_.value_comp();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two sets.
 *
 * \param set The set to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline void Set<K,C,A>::swap( Set& set ) /* throw() */
{
   set_.swap( set.set_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Set operators */
//@{
template< typename K, typename C, typename A >
inline bool operator==( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs );

template< typename K, typename C, typename A >
inline bool operator!=( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs );

template< typename K, typename C, typename A >
inline bool operator<( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs );

template< typename K, typename C, typename A >
inline bool operator>( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs );

template< typename K, typename C, typename A >
inline bool operator<=( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs );

template< typename K, typename C, typename A >
inline bool operator>=( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs );

template< typename K, typename C, typename A >
inline void swap( Set<K,C,A>& a, Set<K,C,A>& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between two sets.
 *
 * \param lhs The left hand side set.
 * \param rhs The right hand side set.
 * \return \a true if the two sets are equal, \a false if they are not.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline bool operator==( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs )
{
   return ( lhs.set_ == rhs.set_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between two sets.
 *
 * \param lhs The left hand side set.
 * \param rhs The right hand side set.
 * \return \a true if the two sets are inequal, \a false if they are not.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline bool operator!=( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Less-than comparison between two sets.
 *
 * \param lhs The left hand side set.
 * \param rhs The right hand side set.
 * \return \a true if \a lhs is lexicographically smaller than \a rhs, \a false if not.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline bool operator<( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs )
{
   return lhs.set_ < rhs.set_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Greater-than comparison between two sets.
 *
 * \param lhs The left hand side set.
 * \param rhs The right hand side set.
 * \return \a true if \a lhs is lexicographically greater than \a rhs, \a false if not.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline bool operator>( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs )
{
   return rhs < lhs;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Less-or-equal-than comparison between two sets.
 *
 * \param lhs The left hand side set.
 * \param rhs The right hand side set.
 * \return \a true if \a lhs is lexicographically less-or-equal than \a rhs, \a false if not.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline bool operator<=( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs )
{
   return !( rhs < lhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Greater-or-equal-than comparison between two sets.
 *
 * \param lhs The left hand side set.
 * \param rhs The right hand side set.
 * \return \a true if \a lhs is lexicographically greater-or-equal than \a rhs, \a false if not.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline bool operator>=( const Set<K,C,A>& lhs, const Set<K,C,A>& rhs )
{
   return !( lhs < rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two sets.
 *
 * \param a The first set to be swapped.
 * \param b The second set to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename K    // Type of the set elements
        , typename C    // Comparison function object type
        , typename A >  // Type of the allocator
inline void swap( Set<K,C,A>& a, Set<K,C,A>& b ) /* throw() */
{
   a.swap( b );
}
//*************************************************************************************************

} // namespace pe

#endif
