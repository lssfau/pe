//=================================================================================================
/*!
 *  \file pe/util/List.h
 *  \brief Implementation of a list container
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

#ifndef _PE_UTIL_LIST_H_
#define _PE_UTIL_LIST_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <list>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a list container.
 * \ingroup util
 *
 * The List class represents a list container after the requirements of the Standard Library.
 * TODO
 */
template< typename T                      // Type of the list elements
        , typename A=std::allocator<T> >  // Type of the allocator
class List
{
public:
   //**Type definitions****************************************************************************
   // STL naming conventions
   /*! \cond PE_INTERNAL */
   typedef std::list<T,A>                             ListType;
   typedef typename ListType::value_type              value_type;
   typedef typename ListType::pointer                 pointer;
   typedef typename ListType::const_pointer           const_pointer;
   typedef typename ListType::reference               reference;
   typedef typename ListType::const_reference         const_reference;
   typedef typename ListType::iterator                iterator;
   typedef typename ListType::const_iterator          const_iterator;
   typedef typename ListType::reverse_iterator        reverse_iterator;
   typedef typename ListType::const_reverse_iterator  const_reverse_iterator;
   typedef typename ListType::size_type               size_type;
   typedef typename ListType::difference_type         difference_type;
   /*! \endcond */

   // pe naming convention
   typedef value_type              ValueType;             //!< Type of the list elements.
   typedef pointer                 Pointer;               //!< Pointer to a non-const element.
   typedef const_pointer           ConstPointer;          //!< Pointer to a const element.
   typedef reference               Reference;             //!< Reference to a non-const element.
   typedef const_reference         ConstReference;        //!< Reference to a const element.
   typedef iterator                Iterator;              //!< Iterator over non-const elements.
   typedef const_iterator          ConstIterator;         //!< Iterator over const elements.
   typedef reverse_iterator        ReverseIterator;       //!< Reverse iterator over non-const elements.
   typedef const_reverse_iterator  ConstReverseIterator;  //!< Reverse iterator over const elements.
   typedef size_type               SizeType;              //!< Size type of the list.
   typedef difference_type         DifferenceType;        //!< Difference type of the list.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
                           explicit inline List();
                           explicit inline List( SizeType n, const T& value=T() );
   template< typename IT > explicit inline List( IT first, IT last );
                                    inline List( const List& list );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Assignment operators************************************************************************
   /*!\name Assignment operators */
   //@{
   inline List& operator=( const List& list );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline SizeType maxSize() const;
   inline SizeType size()    const;
   inline bool     isEmpty() const;
   //@}
   //**********************************************************************************************

   //**Access functions****************************************************************************
   /*!\name Access functions */
   //@{
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
                           inline void     pushFront( const ValueType& value );
                           inline void     pushBack ( const ValueType& value );
                           inline void     popFront ();
                           inline void     popBack  ();

                           inline void     assign( SizeType n, const ValueType& value );
   template< typename IT > inline void     assign( IT first, IT last );

                           inline Iterator insert( Iterator pos, const ValueType& value );
                           inline void     insert( Iterator pos, SizeType n, const ValueType& value );

   template< typename IT > inline void     insert( Iterator pos, IT first, IT last );

                           inline Iterator erase( Iterator pos );
                           inline Iterator erase( Iterator first, Iterator last );

                           inline void     remove( const ValueType& value );

   template< typename UP > inline void     removeIf( UP up );

                           inline void     clear();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
                           inline void resize ( SizeType n, const ValueType& value=ValueType() );
                           inline void reverse();

                           inline void sort();
   template< typename BP > inline void sort( BP bp );

                           inline void merge( List& list );
   template< typename BP > inline void merge( List& list, BP bp );

                           inline void splice( Iterator pos, List& list );
                           inline void splice( Iterator pos, List& list, Iterator del );
                           inline void splice( Iterator pos, List& list, Iterator first, Iterator last );

                           inline void unique();
   template< typename BP > inline void unique( BP bp );

                           inline void swap( List& list ) /* throw() */;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   ListType list_;  //!< The wrapped list container.
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
/*!\brief Default constructor for List.
 *
 * The default constructor creates an empty list.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline List<T,A>::List()
   : list_()  // The wrapped list container
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a list with a number of copies of the given element.
 *
 * \param n The number of elements to be initially created.
 * \param value The initial element for the list.
 *
 * This constructor creates a list with \a n copies of the given element \a value.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline List<T,A>::List( SizeType n, const T& value )
   : list_( n, value )  // The wrapped list container
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a list from a range of elements.
 *
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 *
 * This constructor creates a list with copies of the elements in the range \f$ [first,last) \f$.
 */
template< typename T     // Type of the list elements
        , typename A >   // Type of the allocator
template< typename IT >  // Type of the iterators
inline List<T,A>::List( IT first, IT last )
   : list_( first, last )  // The wrapped list container
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy constructor for List.
 *
 * \param list The list to be copied.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline List<T,A>::List( const List& list )
   : list_( list.list_ )  // The wrapped list container
{}
//*************************************************************************************************




//=================================================================================================
//
//  ASSIGNMENT OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Copy assignment operator for List.
 *
 * \param list The list to be copied.
 * \return Reference to the assigned list.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline List<T,A>& List<T,A>::operator=( const List& list )
{
   list_ = list.list_;
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the maximum possible size of a list.
 *
 * \return The maximum possible size.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::SizeType List<T,A>::maxSize() const
{
   return list_.max_size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current size of the list.
 *
 * \return The current size.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::SizeType List<T,A>::size() const
{
   return list_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns \a true if the list has no elements.
 *
 * \return \a true if the list is empty, \a false if it is not.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline bool List<T,A>::isEmpty() const
{
   return list_.empty();
}
//*************************************************************************************************




//=================================================================================================
//
//  ACCESS FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a reference to the first element of the list.
 *
 * \return Handle to the first element.
 *
 * \b Note: No runtime check is performed if the first element exists!
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::Reference List<T,A>::front()
{
   pe_USER_ASSERT( size() > 0, "List is empty, invalid access to the front element" );
   return list_.front();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the first element of the list.
 *
 * \return Handle to the first element.
 *
 * \b Note: No runtime check is performed if the first element exists!
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::ConstReference List<T,A>::front() const
{
   pe_USER_ASSERT( size() > 0, "List is empty, invalid access to the front element" );
   return list_.front();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the last element of the list.
 *
 * \return Handle to the last element.
 *
 * \b Note: No runtime check is performed if the last element exists!
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::Reference List<T,A>::back()
{
   pe_USER_ASSERT( size() > 0, "Listist is empty, invalid access to the back element" );
   return list_.back();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the last element of the list.
 *
 * \return Handle to the last element.
 *
 * \b Note: No runtime check is performed if the last element exists!
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::ConstReference List<T,A>::back() const
{
   pe_USER_ASSERT( size() > 0, "List is empty, invalid access to the back element" );
   return list_.back();
}
//*************************************************************************************************




//=================================================================================================
//
//  ITERATOR FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns an iterator to the beginning of the list.
 *
 * \return Iterator to the beginning of the list.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::Iterator List<T,A>::begin()
{
   return list_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the beginning of the list.
 *
 * \return Iterator to the beginning of the list.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::ConstIterator List<T,A>::begin() const
{
   return list_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last element of the list.
 *
 * \return Iterator just past the last element of the list.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::Iterator List<T,A>::end()
{
   return list_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last element of the list.
 *
 * \return Iterator just past the last element of the list.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::ConstIterator List<T,A>::end() const
{
   return list_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator to the last element of the list.
 *
 * \return Reverse iterator to the last element of the list.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::ReverseIterator List<T,A>::rbegin()
{
   return list_.rbegin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator to the last element of the list.
 *
 * \return Reverse iterator to the last element of the list.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::ConstReverseIterator List<T,A>::rbegin() const
{
   return list_.rbegin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator just one before the first element of the list.
 *
 * \return Reverse iterator just one before the first element of the list.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::ReverseIterator List<T,A>::rend()
{
   return list_.rend();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reverse iterator just one before the first element of the list.
 *
 * \return Reverse iterator just one before the first element of the list.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::ConstReverseIterator List<T,A>::rend() const
{
   return list_.rend();
}
//*************************************************************************************************




//=================================================================================================
//
//  ELEMENT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding an element to the front of the list.
 *
 * \param value The element to be added to the front of the list.
 * \return void
 *
 * The pushFront function runs in constant time.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::pushFront( const ValueType& value )
{
   list_.push_front( value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adding an element to the end of the list.
 *
 * \param value The element to be added to the end of the list.
 * \return void
 *
 * The pushBack function runs in constant time.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::pushBack( const ValueType& value )
{
   list_.push_back( value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing an element from the front of the list.
 *
 * \return void
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::popFront()
{
   list_.pop_front();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing an element from the end of the list.
 *
 * \return void
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::popBack()
{
   list_.pop_back();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assigning a given element to the list.
 *
 * \param n The number of elements to be assigned.
 * \param value The element to be assigned to the list.
 * \return void
 *
 * This function fills a list with \a n copies of the given element \a value. Note that
 * the assignment completely changes the list and that the resulting list's size is the
 * same as the number of elements assigned. The assign function runs in linear time.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::assign( SizeType n, const ValueType& value )
{
   list_.assign( n, value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assigning a range of elements to the list.
 *
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 * \return void
 *
 * This functions assigns the elements in the range \f$ [first,last) \f$ to the list.
 * All elements previously contained in the list are removed. The assign function runs
 * in linear time.
 */
template< typename T     // Type of the list elements
        , typename A >   // Type of the allocator
template< typename IT >  // Type of the iterators
inline void List<T,A>::assign( IT first, IT last )
{
   list_.assign( first, last );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting an element into the list.
 *
 * \param pos The position before which the element is inserted.
 * \param value The element to be inserted into the list.
 * \return Iterator to the inserted element.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::Iterator List<T,A>::insert( Iterator pos, const ValueType& value )
{
   return list_.insert( pos, value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting a number of copies of the given element into the list.
 *
 * \param pos The position before which the element is inserted.
 * \param n The number of elements to be inserted.
 * \param value The element to be inserted into the list.
 * \return void
 *
 * This function inserts a specified number of copies of the given element before the location
 * specified by \a pos.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::insert( Iterator pos, SizeType n, const ValueType& value )
{
   return list_.insert( pos, n, value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting a range of elements into the list.
 *
 * \param pos The position before which the elements are inserted.
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 * \return void
 *
 * This functions inserts the elements in the range \f$ [first,last) \f$ into the list.
 */
template< typename T     // Type of the list elements
        , typename A >   // Type of the allocator
template< typename IT >  // Type of the iterators
inline void List<T,A>::insert( Iterator pos, IT first, IT last )
{
   list_.insert( pos, first, last );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single element from the list.
 *
 * \param pos The position of the element to be removed.
 * \return Iterator to the element after the erased element.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::Iterator List<T,A>::erase( Iterator pos )
{
   return list_.erase( pos );
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
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline typename List<T,A>::Iterator List<T,A>::erase( Iterator first, Iterator last )
{
   return list_.erase( first, last );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing all elements equal to the given element from the list.
 *
 * \param value The element to be removed from the list.
 * \return void
 *
 * This function removes all instances of the given element \a value from the list.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::remove( const ValueType& value )
{
   list_.remove( value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing all elements for which the unary predicate \a p evaluates to \a true
 *
 * \param up The unary predicate.
 * \return void
 *
 * This function removes elements for which the unary predicate \a p evaluates to true
 * from the list.
 */
template< typename T     // Type of the list elements
        , typename A >   // Type of the allocator
template< typename UP >  // Type of the unary predicate
inline void List<T,A>::removeIf( UP up )
{
   list_.remove_if( up );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing all elements from the list.
 *
 * \return void
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::clear()
{
   list_.clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Resizing the list to the specified number of elements.
 *
 * \param n The new number of elements.
 * \param value Initialization value for new list elements.
 * \return void
 *
 * This function resizes the list to the specified number of elements. If \a n is smaller than
 * the current size of the list, the list is truncated, otherwise the list is extended and the
 * new elements are populated with the given element \a value.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::resize( SizeType n, const ValueType& value )
{
   list_.resize( n, value );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reversing the content of the list.
 *
 * \return void
 *
 * This function reverses the content of the list in linear time.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::reverse()
{
   list_.reverse();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sorting the elements contained in the list.
 *
 * \return void
 *
 * This function sorts the elements contained in the list in ascending order using the
 * \< operator. Sorting the list takes NlogN time.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::sort()
{
   list_.sort();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sorting the elements contained in the list.
 *
 * \param bp Binary predicate for the sorting process.
 * \return void
 *
 * This function sorts the elements contained in the list according to the given binary
 * predicate \a bp. Sorting the list takes NlogN time.
 */
template< typename T     // Type of the list elements
        , typename A >   // Type of the allocator
template< typename BP >  // Type of the binary predicate
inline void List<T,A>::sort( BP bp )
{
   list_.sort( bp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merging of two lists.
 *
 * \param list The list to be merged.
 * \return void
 *
 * This function merges the two lists, producing a combined list that is ordered with respect
 * to the \< operator. The merge() function runs in linear time.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::merge( List& list )
{
   list_.merge( list );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merging of two lists.
 *
 * \param list The list to be merged.
 * \param bp The binary predicate for the sorting process.
 * \return void
 *
 * This function merges the two lists, producing a combined list that is ordered according
 * to the given binary predicate \a bp. The merge() function runs in linear time.
 */
template< typename T     // Type of the list elements
        , typename A >   // Type of the allocator
template< typename BP >  // Type of the binary predicate
inline void List<T,A>::merge( List& list, BP bp )
{
   list_.merge( list, bp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting one list into another list.
 *
 * \param pos The position before which the elements of the second list are inserted.
 * \param list The source list.
 * \return void
 *
 * This function inserts all elements from the given list \a list before the element at position
 * \a pos. The elements from the second list are simply moved from one list to the other. This
 * does not involve copy or delete operations. Therefore the splice() function runs in constant
 * time.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::splice( Iterator pos, List& list )
{
   list_.splice( pos, list );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting a single element from another list.
 *
 * \param pos The position before which the element of the second list is inserted.
 * \param list The source list.
 * \param del The position of the element to be moved to the first list.
 * \return void
 *
 * Removes the element at position \a del in the given list \a list and inserts it into
 * the current list before \a pos. The element from the second list is simply moved from
 * one list to the other. This does not involve copy or delete operations. Therefore the
 * splice() function runs in constant time.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::splice( Iterator pos, List& list, Iterator del )
{
   list_.splice( pos, list, del );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting a range of elements from another list.
 *
 * \param pos The position before which the elements of the second list are inserted.
 * \param list The source list.
 * \param first Iterator to the first element of the element range of \a list.
 * \param last Iterator to the element one past the last element of the element range of \a list.
 * \return void
 *
 * Removes the elements in the range \f$ [first..last) \f$ from the second list and inserts
 * them into the current list before \a pos. The elements from the second list are simply moved
 * from one list to the other. This does not involve copy or delete operations. Therefore the
 * splice() function runs in constant time.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::splice( Iterator pos, List& list, Iterator first, Iterator last )
{
   list_.splice( pos, list, first, last );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing duplicate elements from the list.
 *
 * \return void
 *
 * The unique() function removes all consecutive duplicate elements from the list. Note that
 * only consecutive duplicates are removed, which may require a sorting of the list first.
 * The equality of the elements is tested by the == operator. The function runs in linear time.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::unique()
{
   list_.unique();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing duplicate elements from the list.
 *
 * \param bp Binary predicate for the element comparison.
 * \return void
 *
 * The unique() function removes all consecutive "duplicate" elements from the list, where
 * the given binary predicate is used to qualify elements as "duplicate". Note that only
 * consecutive duplicates are removed, which may require a sorting of the list first.
 * The function runs in linear time.
 */
template< typename T     // Type of the list elements
        , typename A >   // Type of the allocator
template< typename BP >  // Type of the binary predicate
inline void List<T,A>::unique( BP bp )
{
   list_.unique( bp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two lists.
 *
 * \param list The list to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void List<T,A>::swap( List& list ) /* throw() */
{
   list_.swap( list.list_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name List operators */
//@{
template< typename T, typename A >
inline bool operator==( const List<T,A>& lhs, const List<T,A>& rhs );

template< typename T, typename A >
inline bool operator!=( const List<T,A>& lhs, const List<T,A>& rhs );

template< typename T, typename A >
inline bool operator<( const List<T,A>& lhs, const List<T,A>& rhs );

template< typename T, typename A >
inline bool operator>( const List<T,A>& lhs, const List<T,A>& rhs );

template< typename T, typename A >
inline bool operator<=( const List<T,A>& lhs, const List<T,A>& rhs );

template< typename T, typename A >
inline bool operator>=( const List<T,A>& lhs, const List<T,A>& rhs );

template< typename T, typename A >
inline void swap( List<T,A>& a, List<T,A>& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between two lists.
 *
 * \param lhs The left hand side list.
 * \param rhs The right hand side list.
 * \return \a true if the two lists are equal, \a false if they are not.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline bool operator==( const List<T,A>& lhs, const List<T,A>& rhs )
{
   typedef typename List<T,A>::ConstIterator  ConstIterator;
   ConstIterator end1( lhs.end() );
   ConstIterator end2( rhs.end() );

   ConstIterator i1( lhs.begin() );
   ConstIterator i2( rhs.begin() );
   while( i1 != end1 && i2 != end2 && *i1 == *i2 ) {
      ++i1;
      ++i2;
   }
   return i1 == end1 && i2 == end2;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between two lists.
 *
 * \param lhs The left hand side list.
 * \param rhs The right hand side list.
 * \return \a true if the two lists are inequal, \a false if they are not.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline bool operator!=( const List<T,A>& lhs, const List<T,A>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Less-than comparison between two lists.
 *
 * \param lhs The left hand side list.
 * \param rhs The right hand side list.
 * \return \a true if \a lhs is lexicographically smaller than \a rhs, \a false if not.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline bool operator<( const List<T,A>& lhs, const List<T,A>& rhs )
{
   return std::lexicographical_compare( lhs.begin(), lhs.end(), rhs.begin(), rhs.end() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Greater-than comparison between two lists.
 *
 * \param lhs The left hand side list.
 * \param rhs The right hand side list.
 * \return \a true if \a lhs is lexicographically greater than \a rhs, \a false if not.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline bool operator>( const List<T,A>& lhs, const List<T,A>& rhs )
{
   return rhs < lhs;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Less-or-equal-than comparison between two lists.
 *
 * \param lhs The left hand side list.
 * \param rhs The right hand side list.
 * \return \a true if \a lhs is lexicographically less-or-equal than \a rhs, \a false if not.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline bool operator<=( const List<T,A>& lhs, const List<T,A>& rhs )
{
   return !( rhs < lhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Greater-or-equal-than comparison between two lists.
 *
 * \param lhs The left hand side list.
 * \param rhs The right hand side list.
 * \return \a true if \a lhs is lexicographically greater-or-equal than \a rhs, \a false if not.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline bool operator>=( const List<T,A>& lhs, const List<T,A>& rhs )
{
   return !( lhs < rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two lists.
 *
 * \param a The first list to be swapped.
 * \param b The second list to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename T    // Type of the list elements
        , typename A >  // Type of the allocator
inline void swap( List<T,A>& a, List<T,A>& b ) /* throw() */
{
   a.swap( b );
}
//*************************************************************************************************

} // namespace pe

#endif
