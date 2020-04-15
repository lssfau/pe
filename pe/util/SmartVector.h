//=================================================================================================
/*!
 *  \file pe/util/SmartVector.h
 *  \brief Implementation of a vector for (polymorhpic) smart pointers
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

#ifndef _PE_UTIL_SMARTVECTOR_H_
#define _PE_UTIL_SMARTVECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <stdexcept>
#include <pe/util/Assert.h>
#include <pe/util/policies/OptimalGrowth.h>
#include <pe/util/SmartIterator.h>
#include <pe/util/Template.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector for (polymorphic) smart pointers.
 * \ingroup util
 *
 * \section basics Basic usage
 *
 * The SmartVector class is a special container for smart pointers to polymorphic objects. It
 * follows the example of the \a std::vector class but provides the necessary extensions to work
 * with smart pointers to (polymorphic) objects. Let's consider the following example using the
 * two classes \a A and \a B:

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

   // Definition of class B
   class B : public A { ... };

   // Definition of a smart pointer and smart pointer vector for class A
   typedef pe::SharedPtr<A>                 APtr;
   typedef pe::SmartVector<pe::SharedPtr,A> AVector;
   \endcode

 * Using the STL vector for pointers results in the following problem:

   \code
   APtr a;
   std::vector<APtr> vector;
   vector.push_back( a );
   std::vector<APtr>::const_iterator it = vector.begin();
   (*it)->set( 2 );  // Works unfortunately!
   \endcode

 * Although we use a \a const_iterator, the set() function of class \a A can be called in order
 * to change the internal integer member. This is because only the smart pointer to the \a A
 * object is constant, but not the object behind the pointer. The SmartVector removes this
 * inconvenience:

   \code
   APtr a;
   pe::SmartVector<pe::SharedPtr,A> vector;
   vector.pushBack( a );
   pe::SmartVector<pe::SharedPtr,A>::ConstIterator it = vector.begin();
   it->set( 2 );  // Compile-time error!
   \endcode

 * Notice the difference in the usage of the iterator. As an additional feature, the underlying
 * iterator adds an additional dereference to all access operators, which eases the access to
 * the underlying objects:

   \code
   // STL style:
   (*it)->set( 2 );

   // pe style:
   it->set( 2 );
   \endcode

 * An additional difference between the STL vector and the pe smart vector are the template
 * arguments: whereas the STL iterator requires only a single template parameter, the smart
 * vector requires two parameters:

   \code
   // STL vector
   std::vector<A*> vector;

   // pe smart vector
   pe::SmartVector<SharedPtr,A> vector;
   \endcode

 * Another difference in contrast to the \a std::vector is that the functions of SmartVector follow
 * the naming convention of the physics engine (i.e. pushBack instead of push_back).\n\n
 *
 *
 * \section polymorphic Polymorphic pointers
 *
 * For smart pointers to polymorphic objects, the SmartVector class additionally offers two special
 * iterators to iterate over all objects of a specific type: the CastIterator and ConstCastIterator.

   \code
   // Definition of function f for non-const smart pointer vectors
   void f( AVector& vector )
   {
      AVector::CastIterator<B> begin = vector.begin<B>();
      AVector::CastIterator<B> end   = vector.end<B>();

      // Loop over all objects of type B contained in the vector
      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for const smart pointer vectors
   void f( const AVector& vector )
   {
      AVector::ConstCastIterator<B> begin = vector.begin<B>();
      AVector::ConstCastIterator<B> end   = vector.end<B>();

      // Loop over all objects of type B contained in the vector
      for( ; begin!=end; ++begin )
   }
   \endcode

 * In the example, the cast iterators are used to iterate over all objects of type \a B within
 * the smart pointer vector, where \a B must be a type derived from \a A. The attempt to use these
 * iterators for types that are not derived from \a A results in a compile time error. Note that
 * the usage of the cast iterators is computaionally more expensive than the use of the standard
 * iterators. Therefore these iterators should not be used unless a down-cast is really necessary,
 * e.g. in order to access a type specific function.\n\n
 *
 *
 * \section container Using a smart pointer vector within other container classes
 *
 * If a smart pointer vector is used within an other container and is used to store smart pointers
 * to polymorhpic objects, you might face the problem of not being able to create type definitions
 * for the cast iterators. Whereas it is possible to create typedefs for the standard iterators, it
 * is unfortunately not possible (yet) to create type definitions for template classes. In order to
 * create a new return type within the container, the following approach could be taken:

   \code
   template< typename A >
   class Container
   {
    public:
      template< typename C >
      struct CastIterator : public pe::SmartVector<SharedPtr,A>::CastIterator<C>
      {
         CastIterator( const pe::SmartVector<SharedPtr,A>::CastIterator<C>& it )
            : pe::SmartVector<SharedPtr,A>::CastIterator<C>( it )  // Initializing the base class
         {}
      };

      template< typename C >
      CastIterator<C> begin();

      template< typename C >
      CastIterator<C> end();

    private:
      pe::SmartVector<SharedPtr,A> vector_;
   };
   \endcode

 * Instead of a typedef within the Container class, a new class CastIterator is derived from the
 * SmartVector::CastIterator class. This approach acts similar as the typedef as a user can now
 * use the Container as follows:

   \code
   class A { ... };
   class B : public A { ... };

   Container<A>::CastIterator<B> begin;
   \endcode

 * This provides the same abstraction from the internal implementation as the desired typedef.
 * The same approach could be taken for a ConstCastIterator definition.\n\n
 *
 *
 * \section adaptions Adapting a smart pointer vector
 *
 * The growth rate of the SmartVector class can be adapted to any specific task. The third
 * template argument of the SmartVector specifies the growth rate. The following growth rates
 * can be selected:
 *
 *  - ConstantGrowth
 *  - LinearGrowth (the default behavior)
 *
 * Both growth rates take two template arguments: the first specifies the growth factor for the
 * selected bahavior (additive for ConstantGrowth, multiplicative for LinearGrowth), whereas the
 * second specifies the initial capacity of the smart pointer vector.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G = OptimalGrowth >  // Growth policy
class SmartVector
{
private:
   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   template< template< typename > class S2, typename T2, typename G2 > friend class SmartVector;
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef S<T>                      ValueType;           //!< Type of the vector elements.
   typedef S<T>&                     ReferenceType;       //!< Reference return type.
   typedef const S<T>&               ConstReferenceType;  //!< Reference-to-const return type.
   typedef size_t                    SizeType;            //!< Size type of the smart pointer vector.
   typedef SmartIterator<S,T>        Iterator;            //!< Iterator over non-const objects.
   typedef SmartIterator<S,const T>  ConstIterator;       //!< Iterator over const objects.
   typedef G                         GrowthPolicy;        //!< Type of the growth policy.
   //**********************************************************************************************

   //**Forward declarations for nested classes*****************************************************
   template< typename C > class CastIterator;
   template< typename C > class ConstCastIterator;
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline SmartVector( SizeType initCapacity = 0 );
            inline SmartVector( const SmartVector& sv );

   template< typename T2, typename G2 >
            inline SmartVector( const SmartVector<S,T2,G2>& sv );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~SmartVector();
   //@}
   //**********************************************************************************************

   //**Assignment operators************************************************************************
   /*!\name Assignment operators */
   //@{
   SmartVector& operator=( const SmartVector& sv );

   template< typename T2, typename G2 >
   SmartVector& operator=( const SmartVector<S,T2,G2>& sv );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
                          inline SizeType maxSize()  const;
                          inline SizeType size()     const;
   template< typename C > inline SizeType size()     const;
                          inline SizeType capacity() const;
                          inline bool     isEmpty()  const;
   //@}
   //**********************************************************************************************

   //**Access functions****************************************************************************
   /*!\name Access functions */
   //@{
   inline ReferenceType      operator[]( SizeType index );
   inline ConstReferenceType operator[]( SizeType index ) const;
   inline ReferenceType      front();
   inline ConstReferenceType front() const;
   inline ReferenceType      back();
   inline ConstReferenceType back()  const;
   //@}
   //**********************************************************************************************

   //**Iterator functions**************************************************************************
   /*!\name Iterator functions */
   //@{
                          inline Iterator             begin();
                          inline ConstIterator        begin() const;
   template< typename C > inline CastIterator<C>      begin();
   template< typename C > inline ConstCastIterator<C> begin() const;

                          inline Iterator             end();
                          inline ConstIterator        end()   const;
   template< typename C > inline CastIterator<C>      end();
   template< typename C > inline ConstCastIterator<C> end()   const;
   //@}
   //**********************************************************************************************

   //**Element functions***************************************************************************
   /*!\name Element functions */
   //@{
   inline void     pushBack( const ValueType& sp );
   inline void     popBack();
   inline Iterator insert( Iterator pos, const ValueType& sp );

   template< typename IteratorType >
   inline void     insert( Iterator pos, IteratorType first, IteratorType last );

   /*! \cond PE_INTERNAL */
   template< typename IteratorType >
   inline void     insert( Iterator pos, IteratorType* first, IteratorType* last );
   /*! \endcond */

   inline Iterator erase( Iterator pos );
   inline void     clear();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
          void reserve( SizeType newCapacity );
   inline void swap( SmartVector& sv ) /* throw() */;
   //@}
   //**********************************************************************************************

private:
   //**Helper functions****************************************************************************
   /*!\name Helper functions */
   //@{
   inline size_t calcCapacity( size_t minCapacity ) const;
   //@}
   //**********************************************************************************************

   //**Insertion helper functions******************************************************************
   /*!\name Insertion helper functions */
   //@{
          void insert( ValueType* const pos, const ValueType& sp );

   /*! \cond PE_INTERNAL */
   template< typename IteratorType >
   inline void insert( Iterator pos, IteratorType first, IteratorType last, std::input_iterator_tag );

   template< typename IteratorType >
   inline void insert( Iterator pos, IteratorType first, IteratorType last, std::random_access_iterator_tag );
   /*! \endcond */

   template< typename IteratorType >
          void insert( ValueType* pos, IteratorType first, IteratorType last, SizeType n );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   SizeType size_;      //!< The current size of the smart pointer vector.
   SizeType capacity_;  //!< The capacity of the smart pointer vector.
   ValueType* begin_;   //!< Pointer to the first element of the smart pointer vector.
   ValueType* end_;     //!< Pointer to the last element of the smart pointer vector.
   //@}
   //**********************************************************************************************

public:
   //**CastIterator/ConstCastIterator comparison operators*****************************************
   // The following comparison operators cannot be defined as namespace or member functions
   // but have to be injected into the surrounding scope via the Barton-Nackman trick since
   // the template arguments of nested templated cannot be deduced (C++ standard 14.8.2.4/4).
   /*!\name CastIterator/ConstCastIterator comparison operators */
   //@{

   //**********************************************************************************************
   /*!\brief Equality comparison between two CastIterator objects.
   //
   // \param lhs The left hand side cast iterator.
   // \param rhs The right hand side cast iterator.
   // \return \a true if the iterators point to the same element, \a false if not.
   */
   template< typename L, typename R >
   friend inline bool operator==( const CastIterator<L>& lhs, const CastIterator<R>& rhs )
   {
      pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
      return static_cast<const void*>( lhs.base() ) == static_cast<const void*>( rhs.base() );
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Equality comparison between a CastIterator and a ConstCastIterator.
   //
   // \param lhs The left hand side cast iterator.
   // \param rhs The right hand side constant cast iterator.
   // \return \a true if the iterators point to the same element, \a false if not.
   */
   template< typename L, typename R >
   friend inline bool operator==( const CastIterator<L>& lhs, const ConstCastIterator<R>& rhs )
   {
      pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
      return static_cast<const void*>( lhs.base() ) == static_cast<const void*>( rhs.base() );
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Equality comparison between a ConstCastIterator and a CastIterator.
   //
   // \param lhs The left hand side constant cast iterator.
   // \param rhs The right hand side cast iterator.
   // \return \a true if the iterators point to the same element, \a false if not.
   */
   template< typename L, typename R >
   friend inline bool operator==( const ConstCastIterator<L>& lhs, const CastIterator<R>& rhs )
   {
      pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
      return static_cast<const void*>( lhs.base() ) == static_cast<const void*>( rhs.base() );
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Equality comparison between two ConstCastIterator objects.
   //
   // \param lhs The left hand side constant cast iterator.
   // \param rhs The right hand side constant cast iterator.
   // \return \a true if the iterators point to the same element, \a false if not.
   */
   template< typename L, typename R >
   friend inline bool operator==( const ConstCastIterator<L>& lhs, const ConstCastIterator<R>& rhs )
   {
      pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
      return static_cast<const void*>( lhs.base() ) == static_cast<const void*>( rhs.base() );
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Inequality comparison between two CastIterator objects.
   //
   // \param lhs The left hand side cast iterator.
   // \param rhs The right hand side cast iterator.
   // \return \a true if the iterators don't point to the same element, \a false if they do.
   */
   template< typename L, typename R >
   friend inline bool operator!=( const CastIterator<L>& lhs, const CastIterator<R>& rhs )
   {
      pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
      return static_cast<const void*>( lhs.base() ) != static_cast<const void*>( rhs.base() );
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Inequality comparison between a CastIterator and a ConstCastIterator.
   //
   // \param lhs The left hand side cast iterator.
   // \param rhs The right hand side constant cast iterator.
   // \return \a true if the iterators don't point to the same element, \a false if they do.
   */
   template< typename L, typename R >
   friend inline bool operator!=( const CastIterator<L>& lhs, const ConstCastIterator<R>& rhs )
   {
      pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
      return static_cast<const void*>( lhs.base() ) != static_cast<const void*>( rhs.base() );
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Inequality comparison between a ConstCastIterator and a CastIterator.
   //
   // \param lhs The left hand side constant cast iterator.
   // \param rhs The right hand side cast iterator.
   // \return \a true if the iterators don't point to the same element, \a false if they do.
   */
   template< typename L, typename R >
   friend inline bool operator!=( const ConstCastIterator<L>& lhs, const CastIterator<R>& rhs )
   {
      pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
      return static_cast<const void*>( lhs.base() ) != static_cast<const void*>( rhs.base() );
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Inequality comparison between two ConstCastIterator objects.
   //
   // \param lhs The left hand side constant cast iterator.
   // \param rhs The right hand side constant cast iterator.
   // \return \a true if the iterators don't point to the same element, \a false if they do.
   */
   template< typename L, typename R >
   friend inline bool operator!=( const ConstCastIterator<L>& lhs, const ConstCastIterator<R>& rhs )
   {
      pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE( L, R );
      return static_cast<const void*>( lhs.base() ) != static_cast<const void*>( rhs.base() );
   }
   //**********************************************************************************************

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
/*!\brief Standard constructor for SmartVector.
 *
 * \param initCapacity The initial capacity of the smart pointer vector.
 *
 * The initial capacity of the smart pointer vector is specified by the selected growth policy.
 */
template< template< typename > class S      // Type of the smart pointer
        , typename T                        // Type of the smart pointer elements
        , typename G >                      // Growth policy
inline SmartVector<S,T,G>::SmartVector( SizeType initCapacity )
   : size_( 0 )                             // Current size of the smart pointer vector
   , capacity_( initCapacity )              // Capacity of the smart pointer vector
   , begin_( new ValueType[initCapacity] )  // Pointer to the first element
   , end_( begin_ )                         // Pointer to the last element
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy constructor for SmartVector.
 *
 * \param sv The smart pointer vector to be copied.
 */
template< template< typename > class S   // Type of the smart pointer
        , typename T                     // Type of the smart pointer elements
        , typename G >                   // Growth policy
inline SmartVector<S,T,G>::SmartVector( const SmartVector& sv )
   : size_( sv.size_ )                   // Current size of the smart pointer vector
   , capacity_( sv.size_ )               // Capacity of the smart pointer vector
   , begin_( new ValueType[capacity_] )  // Pointer to the first element
   , end_( begin_+size_ )                // Pointer to the last element
{
   for( SizeType i=0; i<size_; ++i )
      begin_[i] = sv.begin_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different SmartVector instances.
 *
 * \param sv The smart pointer vector to be copied.
 */
template< template< typename > class S   // Type of the smart pointer
        , typename T                     // Type of the smart pointer elements
        , typename G >                   // Growth policy
template< typename T2                    // Type of the foreign smart pointer vector
        , typename G2 >                  // Growth policy of the foreign smart pointer vector
inline SmartVector<S,T,G>::SmartVector( const SmartVector<S,T2,G2>& sv )
   : size_( sv.size() )                  // Current size of the smart pointer vector
   , capacity_( sv.size() )              // Capacity of the smart pointer vector
   , begin_( new ValueType[capacity_] )  // Pointer to the first element
   , end_( begin_+size_ )                // Pointer to the last element
{
   for( SizeType i=0; i<size_; ++i )
      begin_[i] = sv[i];
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for SmartVector.
 *
 * In the destructor, the selected deletion policy is applied to all elements of the pointer
 * vector.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline SmartVector<S,T,G>::~SmartVector()
{
   delete [] begin_;
}
//*************************************************************************************************




//=================================================================================================
//
//  COPY ASSIGNMENT OPERATOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Copy assignment operator for SmartVector.
 *
 * \param sv The smart pointer vector to be copied.
 * \return Reference to the assigned smart pointer vector.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
SmartVector<S,T,G>& SmartVector<S,T,G>::operator=( const SmartVector& sv )
{
   if( &sv == this ) return *this;

   if( sv.size_ > capacity_ ) {
      ValueType* newBegin( new ValueType[sv.size_] );
      end_ = std::copy( sv.begin_, sv.end_, newBegin );
      std::swap( begin_, newBegin );
      delete [] newBegin;

      size_ = sv.size_;
      capacity_ = sv.size_;
   }
   else if( size_ > sv.size_ ) {
      ValueType* newEnd = std::copy( sv.begin_, sv.end_, begin_ );
      for( ; end_!=newEnd; )
         (--end_)->reset();
      size_ = sv.size_;
   }
   else {
      end_  = std::copy( sv.begin_, sv.end_, begin_ );
      size_ = sv.size_;
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for different SmartVector instances.
 *
 * \param sv The smart pointer vector to be copied.
 * \return Reference to the assigned smart pointer vector.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename T2                   // Type of the foreign smart pointer vector
        , typename G2 >                 // Growth policy of the foreign smart pointer vector
SmartVector<S,T,G>& SmartVector<S,T,G>::operator=( const SmartVector<S,T2,G2>& sv )
{
   if( sv.size_ > capacity_ ) {
      ValueType* newBegin( new ValueType[sv.size_] );
      end_ = std::copy( sv.begin_, sv.end_, newBegin );
      std::swap( begin_, newBegin );
      delete [] newBegin;

      size_ = sv.size_;
      capacity_ = sv.size_;
   }
   else if( size_ > sv.size_ ) {
      ValueType* newEnd = std::copy( sv.begin_, sv.end_, begin_ );
      for( ; end_!=newEnd; )
         (--end_)->reset();
      size_ = sv.size_;
   }
   else {
      end_  = std::copy( sv.begin_, sv.end_, begin_ );
      size_ = sv.size_;
   }

   return *this;
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the maximum possible size of a smart pointer vector.
 *
 * \return The maximum possible size.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::SizeType SmartVector<S,T,G>::maxSize() const
{
   return SizeType(-1) / sizeof(ValueType);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current size of the smart pointer vector.
 *
 * \return The current size.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::SizeType SmartVector<S,T,G>::size() const
{
   return size_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the total number of objects of type \a C contained in the smart pointer vector.
 *
 * \return The total number of objects of type \a C.
 *
 * This function calculates the total number of objects of type \a C within the smart pointer
 * vector, where \a C is a type derived from the type \a T of objects contained in the smart
 * pointer vector. The attempt to use this function for types that are not derived from \a T
 * results in a compile time error.

   \code
   // Definition of class A and the derived type B
   class A { ... };
   class B : public A { ... };

   // Definition of a smart pointer vector for class A
   typedef pe::SmartVector<SharedPtr,A> AVector;
   AVector vector;

   AVector::SizeType total = vector.size();     // Calculating the total number of pointers
   AVector::SizeType numB  = vector.size<B>();  // Calculating the total number of B objects
   \endcode

 * \b Note: The total number of objects of type \a C is not cached inside the smart pointer
 * vector but is calculated each time the function is called. Using the templated version of
 * size() to calculate the total number objects of type \a C is therefore more expensive than
 * using the non-template version of size() to get the total number of smart pointers in the
 * vector!
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::SizeType SmartVector<S,T,G>::size() const
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( C, T );

   SizeType count( 0 );
   for( ValueType* it=begin_; it!=end_; ++it )
      if( dynamic_cast<C*>( it->get() ) ) ++count;
   return count;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the capacity of the smart pointer vector.
 *
 * \return The capacity.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::SizeType SmartVector<S,T,G>::capacity() const
{
   return capacity_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns \a true if the smart pointer vector has no elements.
 *
 * \return \a true if the smart pointer vector is empty, \a false if it is not.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline bool SmartVector<S,T,G>::isEmpty() const
{
   return size_ == 0;
}
//*************************************************************************************************




//=================================================================================================
//
//  ACCESS FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the smart pointer vector elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Handle to the accessed element.
 *
 * \b Note: No runtime check is performed to insure the validity of the access index.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::ReferenceType SmartVector<S,T,G>::operator[]( SizeType index )
{
   return *(begin_+index);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the smart pointer vector elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Handle to the accessed element.
 *
 * \b Note: No runtime check is performed to insure the validity of the access index.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::ConstReferenceType SmartVector<S,T,G>::operator[]( SizeType index ) const
{
   return *(begin_+index);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the first element of the smart pointer vector.
 *
 * \return Handle to the first element.
 *
 * \b Note: No runtime check is performed if the first element exists!
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::ReferenceType SmartVector<S,T,G>::front()
{
   pe_USER_ASSERT( size_ > 0, "Smart pointer vector is empty, invalid access to the front element" );
   return *begin_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the first element of the smart pointer vector.
 *
 * \return Handle to the first element.
 *
 * \b Note: No runtime check is performed if the first element exists!
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::ConstReferenceType SmartVector<S,T,G>::front() const
{
   pe_USER_ASSERT( size_ > 0, "Smart pointer vector is empty, invalid access to the front element" );
   return *begin_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the last element of the smart pointer vector.
 *
 * \return Handle to the last element.
 *
 * \b Note: No runtime check is performed if the last element exists!
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::ReferenceType SmartVector<S,T,G>::back()
{
   pe_USER_ASSERT( size_ > 0, "Smart pointer vector is empty, invalid access to the back element" );
   return *(end_-1);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the last element of the smart pointer vector.
 *
 * \return Handle to the last element.
 *
 * \b Note: No runtime check is performed if the last element exists!
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::ConstReferenceType SmartVector<S,T,G>::back() const
{
   pe_USER_ASSERT( size_ > 0, "Smart pointer vector is empty, invalid access to the back element" );
   return *(end_-1);
}
//*************************************************************************************************




//=================================================================================================
//
//  ITERATOR FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns an iterator to the beginning of the smart pointer vector.
 *
 * \return Iterator to the beginning of the smart pointer vector.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::Iterator SmartVector<S,T,G>::begin()
{
   return Iterator( begin_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the beginning of the smart pointer vector.
 *
 * \return Iterator to the beginning of the smart pointer vector.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::ConstIterator SmartVector<S,T,G>::begin() const
{
   return ConstIterator( begin_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first element of type \a C within the smart pointer vector.
 *
 * \return Iterator to the first element of type \a C.
 *
 * This function returns an iterator to the first element of type \a C within in the smart
 * pointer vector, where \a C is a type derived from the type \a T of objects contained in the
 * smart pointer vector. In case there is no element of type \a C contained in the vector, an
 * iterator just past the last element of the smart pointer vector is returned. In combination
 * with the according end function (see example), this iterator allows to iterate over all
 * objects of type \a C in the range of the smart pointer vector. The attempt to use this
 * function for types that are not derived from \a T results in a compile time error.

   \code
   // Definition of class A and the derived type B
   class A { ... };
   class B : public A { ... };

   // Definition of a smart pointer vector for class A
   typedef pe::SmartVector<SharedPtr,A> AVector;

   // Definition of function f for non-const smart pointer vectors
   void f( AVector& vector )
   {
      AVector::CastIterator<B> begin = vector.begin<B>();
      AVector::CastIterator<B> end   = vector.end<B>();

      // Loop over all objects of type B contained in the vector
      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for const smart pointer vectors
   void f( const AVector& vector )
   {
      AVector::ConstCastIterator<B> begin = vector.begin<B>();
      AVector::ConstCastIterator<B> end   = vector.end<B>();

      // Loop over all objects of type B contained in the vector
      for( ; begin!=end; ++begin )
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all elements of
 * type \a C in the element range of the smart pointer vector is more expensive than using
 * the non-template versions to traverse the entire range of elements. Use this function only
 * if you require a type-specific member of type \a C.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE CastIterator<C> SmartVector<S,T,G>::begin()
{
   return CastIterator<C>( begin_, end_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first element of type \a C within the smart pointer vector.
 *
 * \return Iterator to the first element of type \a C.
 *
 * This function returns an iterator to the first element of type \a C within in the smart
 * pointer vector, where \a C is a type derived from the type \a T of objects contained in the
 * smart pointer vector. In case there is no element of type \a C contained in the vector, an
 * iterator just past the last element of the smart pointer vector is returned. In combination
 * with the according end function (see example), this iterator allows to iterate over all
 * objects of type \a C in the range of the smart pointer vector. The attempt to use this
 * function for types that are not derived from \a T results in a compile time error.

   \code
   // Definition of class A and the derived type B
   class A { ... };
   class B : public A { ... };

   // Definition of a smart pointer vector for class A
   typedef pe::SmartVector<SharedPtr,A> AVector;

   // Definition of function f for non-const smart pointer vectors
   void f( AVector& vector )
   {
      AVector::CastIterator<B> begin = vector.begin<B>();
      AVector::CastIterator<B> end   = vector.end<B>();

      // Loop over all objects of type B contained in the vector
      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for const smart pointer vectors
   void f( const AVector& vector )
   {
      AVector::ConstCastIterator<B> begin = vector.begin<B>();
      AVector::ConstCastIterator<B> end   = vector.end<B>();

      // Loop over all objects of type B contained in the vector
      for( ; begin!=end; ++begin )
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all elements of
 * type \a C in the element range of the smart pointer vector is more expensive than using
 * the non-template versions to traverse the entire range of elements. Use this function only
 * if you require a type-specific member of type \a C.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE ConstCastIterator<C> SmartVector<S,T,G>::begin() const
{
   return ConstCastIterator<C>( begin_, end_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last element of the smart pointer vector.
 *
 * \return Iterator just past the last element of the smart pointer vector.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::Iterator SmartVector<S,T,G>::end()
{
   return Iterator( end_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last element of the smart pointer vector.
 *
 * \return Iterator just past the last element of the smart pointer vector.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::ConstIterator SmartVector<S,T,G>::end() const
{
   return ConstIterator( end_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last element of the smart pointer vector.
 *
 * \return Iterator just past the last element of the smart pointer vector.
 *
 * This function returns an iterator just past the last element of the smart pointer vector. In
 * combination with the according begin function (see example), this iterator allows to iterate
 * over all objects of type \a C in the range of the smart pointer vector. The attempt to use
 * this function for types that are not derived from \a T results in a compile time error.

   \code
   // Definition of class A and the derived type B
   class A { ... };
   class B : public A { ... };

   // Definition of a smart pointer vector for class A
   typedef pe::SmartVector<SharedPtr,A> AVector;

   // Definition of function f for non-const pointer vectors
   void f( AVector& vector )
   {
      AVector::CastIterator<B> begin = vector.begin<B>();
      AVector::CastIterator<B> end   = vector.end<B>();

      // Loop over all objects of type B contained in the vector
      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for const pointer vectors
   void f( const AVector& vector )
   {
      AVector::ConstCastIterator<B> begin = vector.begin<B>();
      AVector::ConstCastIterator<B> end   = vector.end<B>();

      // Loop over all objects of type B contained in the vector
      for( ; begin!=end; ++begin )
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all elements of
 * type \a C in the element range of the smart pointer vector is more expensive than using
 * the non-template versions to traverse the entire range of elements. Use this function only
 * if you require a type-specific member of type \a C.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE CastIterator<C> SmartVector<S,T,G>::end()
{
   return CastIterator<C>( end_, end_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last element of the smart pointer vector.
 *
 * \return Iterator just past the last element of the smart pointer vector.
 *
 * This function returns an iterator just past the last element of the smart pointer vector. In
 * combination with the according begin function (see example), this iterator allows to iterate
 * over all objects of type \a C in the range of the smart pointer vector. The attempt to use
 * this function for types that are not derived from \a T results in a compile time error.

   \code
   // Definition of class A and the derived type B
   class A { ... };
   class B : public A { ... };

   // Definition of a smart pointer vector for class A
   typedef pe::SmartVector<SharedPtr,A> AVector;

   // Definition of function f for non-const pointer vectors
   void f( AVector& vector )
   {
      AVector::CastIterator<B> begin = vector.begin<B>();
      AVector::CastIterator<B> end   = vector.end<B>();

      // Loop over all objects of type B contained in the vector
      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for const pointer vectors
   void f( const AVector& vector )
   {
      AVector::ConstCastIterator<B> begin = vector.begin<B>();
      AVector::ConstCastIterator<B> end   = vector.end<B>();

      // Loop over all objects of type B contained in the vector
      for( ; begin!=end; ++begin )
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all elements of
 * type \a C in the element range of the smart pointer vector is more expensive than using
 * the non-template versions to traverse the entire range of elements. Use this function only
 * if you require a type-specific member of type \a C.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE ConstCastIterator<C> SmartVector<S,T,G>::end() const
{
   return ConstCastIterator<C>( end_, end_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  ELEMENT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding an element to the end of the smart pointer vector.
 *
 * \param sp The smart pointer to be added to the end smart pointer vector.
 * \return void
 * \exception std::length_error Maximum smart pointer vector length exceeded.
 *
 * The PushBack function runs in constant time.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline void SmartVector<S,T,G>::pushBack( const ValueType& sp )
{
   if( size_ != capacity_ ) {
      *end_ = sp;
      ++end_;
      ++size_;
   }
   else {
      insert( end_, sp );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing an element from the end of the smart pointer vector.
 *
 * \return void
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline void SmartVector<S,T,G>::popBack()
{
   (--end_)->reset();
   --size_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting an element into the smart pointer vector.
 *
 * \param pos The position before which the element is inserted.
 * \param sp The pointer to be inserted into the smart pointer vector.
 * \return Iterator to the inserted element.
 * \exception std::length_error Maximum smart pointer vector length exceeded.
 *
 * The insert function runs in linear time. Note however that inserting elements into a smart
 * pointer vector can be a relatively time-intensive operation.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::Iterator SmartVector<S,T,G>::insert( Iterator pos, const ValueType& sp )
{
   ValueType* const base = const_cast<ValueType* const>( pos.base() );
   const SizeType diff( base - begin_ );

   if( size_ != capacity_ && base == end_ ) {
      *end_ = sp;
      ++end_;
      ++size_;
   }
   else {
      insert( base, sp );
   }

   return Iterator( begin_+diff );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting a range of elements into the smart pointer vector.
 *
 * \param pos The position before which the elements are inserted.
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 * \return void
 * \exception std::length_error Maximum smart pointer vector length exceeded.
 *
 * This functions inserts the elements in the range \f$ [first,last) \f$ into the smart pointer
 * vector. The insert function runs in linear time. Note however that inserting elements into a
 * smart pointer vector can be a relatively time-intensive operation.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename IteratorType >
inline void SmartVector<S,T,G>::insert( Iterator pos, IteratorType first, IteratorType last )
{
   insert( pos, first, last, typename IteratorType::iterator_category() );
}
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Inserting a range of elements into the smart pointer vector.
 *
 * \param pos The position before which the elements are inserted.
 * \param first Pointer to the first element of the element range.
 * \param last Pointer to the element one past the last element of the element range.
 * \return void
 * \exception std::length_error Maximum smart pointer vector length exceeded.
 *
 * This functions inserts the elements in the range \f$ [first,last) \f$ into the smart pointer
 * vector. The insert function runs in linear time. Note however that inserting elements into a
 * smart pointer vector can be a relatively time-intensive operation.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename IteratorType >
inline void SmartVector<S,T,G>::insert( Iterator pos, IteratorType* first, IteratorType* last )
{
   insert( pos, first, last, std::random_access_iterator_tag() );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing an element from the smart pointer vector.
 *
 * \param pos The position of the element to be removed.
 * \return Iterator to the element after the erased element.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline typename SmartVector<S,T,G>::Iterator SmartVector<S,T,G>::erase( Iterator pos )
{
   ValueType* const base = const_cast<ValueType* const>( pos.base() );
   std::copy( base+1, end_, base );

   (--end_)->reset();
   --size_;

   return pos;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing all elements from the smart pointer vector.
 *
 * \return void
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline void SmartVector<S,T,G>::clear()
{
   for( ; end_!=begin_; )
      (--end_)->reset();
   size_ = 0;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the minimum capacity of the smart pointer vector
 *
 * \param newCapacity The new minimum capacity of the smart pointer vector.
 * \return void
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
void SmartVector<S,T,G>::reserve( SizeType newCapacity )
{
   if( newCapacity > capacity_ )
   {
      // Calculating the new capacity
      newCapacity = calcCapacity( newCapacity );

      // Allocating a new array
      ValueType* tmp = new ValueType[newCapacity];

      // Replacing the old array
      std::copy( begin_, end_, tmp );
      std::swap( tmp, begin_ );
      capacity_ = newCapacity;
      end_ = begin_ + size_;
      delete [] tmp;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two smart pointer vectors.
 *
 * \param sv The smart pointer vector to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline void SmartVector<S,T,G>::swap( SmartVector& sv ) /* throw() */
{
   // By using the 'std::swap' function to swap all member variables,
   // the function can give the nothrow guarantee.
   std::swap( size_, sv.size_ );
   std::swap( capacity_, sv.capacity_ );
   std::swap( begin_, sv.begin_ );
   std::swap( end_, sv.end_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  HELPER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculating the new capacity of the vector based on its growth policy.
 *
 * \param minCapacity The minimum necessary capacity.
 * \return The new capacity.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline size_t SmartVector<S,T,G>::calcCapacity( size_t minCapacity ) const
{
   pe_INTERNAL_ASSERT( minCapacity > capacity_, "Invalid new vector capacity" );
   const size_t newCapacity( GrowthPolicy()( capacity_, minCapacity ) );
   pe_INTERNAL_ASSERT( newCapacity > capacity_, "Invalid new vector capacity" );
   return newCapacity;
}
//*************************************************************************************************




//=================================================================================================
//
//  INSERTION HELPER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Inserting an element into the smart pointer vector.
 *
 * \param pos The position before which the element is inserted.
 * \param p The pointer to be inserted into the smart pointer vector.
 * \return void
 * \exception std::length_error Maximum smart pointer vector length exceeded.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
void SmartVector<S,T,G>::insert( ValueType* const pos, const ValueType& p )
{
   if( size_ != capacity_ ) {
      std::copy_backward( pos, end_, end_+1 );
      *pos = p;
      ++end_;
      ++size_;
   }
   else if( size_ == maxSize() ) {
      throw std::length_error( "Maximum pointer vector length exceeded!" );
   }
   else {
      SizeType newCapacity( calcCapacity( capacity_+1 ) );
      if( newCapacity > maxSize() || newCapacity < capacity_ ) newCapacity = maxSize();

      ValueType* newBegin = new ValueType[newCapacity];
      ValueType* tmp = std::copy( begin_, pos, newBegin );
      *tmp = p;
      end_ = std::copy( pos, end_, ++tmp );

      std::swap( newBegin, begin_ );
      delete [] newBegin;
      capacity_ = newCapacity;
      ++size_;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Inserting a range of elements into the smart pointer vector.
 *
 * \param pos The position before which the elements are inserted.
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 * \return void
 * \exception std::length_error Maximum smart pointer vector length exceeded.
 *
 * This functions inserts the elements in the range \f$ [first,last) \f$ into the smart pointer
 * vector. The iterators are treated as input iterators.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename IteratorType >
inline void SmartVector<S,T,G>::insert( Iterator pos, IteratorType first, IteratorType last,
                                        std::input_iterator_tag )
{
   for( ; first!=last; ++first ) {
      pos = insert( pos, *first );
      ++pos;
   }
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Inserting a range of elements into the smart pointer vector.
 *
 * \param pos The position before which the elements are inserted.
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 * \return void
 * \exception std::length_error Maximum smart pointer vector length exceeded.
 *
 * This functions inserts the elements in the range \f$ [first,last) \f$ into the smart pointer
 * vector. The iterators are treated as random access iterators.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename IteratorType >
inline void SmartVector<S,T,G>::insert( Iterator pos, IteratorType first, IteratorType last,
                                        std::random_access_iterator_tag )
{
   ValueType* base = const_cast<ValueType*>( pos.base() );
   const SizeType diff( last - first );

   if( size_+diff <= capacity_ && base == end_ ) {
      for( ; first!=last; ++first, ++end_ ) {
         *end_ = *first;
      }
      size_ += diff;
   }
   else {
      insert( base, first, last, diff );
   }
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting a range of elements into the smart pointer vector.
 *
 * \param pos The position before which the elements are inserted.
 * \param first Iterator to the first element of the element range.
 * \param last Iterator to the element one past the last element of the element range.
 * \param n The number of elements to be inserted.
 * \return void
 * \exception std::length_error Maximum smart pointer vector length exceeded.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename IteratorType >
void SmartVector<S,T,G>::insert( ValueType* pos, IteratorType first, IteratorType last, SizeType n )
{
   const SizeType newSize( size_ + n );

   if( newSize <= capacity_ ) {
      std::copy_backward( pos, end_, end_+n );
      for( ; first!=last; ++first, ++pos ) {
         *pos = *first;
      }
      end_ += n;
      size_ = newSize;
   }
   else if( newSize > maxSize() || newSize < size_ ) {
      throw std::length_error( "Maximum pointer vector length exceeded!" );
   }
   else {
      ValueType* newBegin = new ValueType[newSize];
      ValueType* tmp = std::copy( begin_, pos, newBegin );

      for( ; first!=last; ++first, ++tmp ) {
         *tmp = *first;
      }

      end_ = std::copy( pos, end_, tmp );

      std::swap( newBegin, begin_ );
      delete [] newBegin;
      capacity_ = newSize;
      size_ = newSize;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name SmartVector operators */
//@{
template< template< typename > class S, typename TL, typename TR, typename GL, typename GR >
inline bool operator==( const SmartVector<S,TL,GL>& lhs, const SmartVector<S,TR,GR>& rhs );

template< template< typename > class S, typename TL, typename TR, typename GL, typename GR >
inline bool operator!=( const SmartVector<S,TL,GL>& lhs, const SmartVector<S,TR,GR>& rhs );

template< template< typename > class S, typename T, typename G >
inline void swap( SmartVector<S,T,G>& a, SmartVector<S,T,G>& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between two smart pointer vectors.
 *
 * \param lhs The left hand side smart pointer vector.
 * \param rhs The right hand side smart pointer vector.
 * \return \a true if the two smart pointer vectors are equal, \a false if they are not.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename TL                   // Type of the smart pointer elements of the left operand
        , typename TR                   // Type of the smart pointer elements of the right operand
        , typename GL                   // Growth policy of the left operand
        , typename GR >                 // Growth policy of the right operand
inline bool operator==( const SmartVector<S,TL,GL>& lhs, const SmartVector<S,TR,GR>& rhs )
{
   return lhs.size() == rhs.size() && std::equal( lhs.begin(), lhs.end(), rhs.begin() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between two smart pointer vectors.
 *
 * \param lhs The left hand side smart pointer vector.
 * \param rhs The right hand side smart pointer vector.
 * \return \a true if the two smart pointer vectors are inequal, \a false if they are not.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename TL                   // Type of the smart pointer elements of the left operand
        , typename TR                   // Type of the smart pointer elements of the right operand
        , typename GL                   // Growth policy of the left operand
        , typename GR >                 // Growth policy of the right operand
inline bool operator!=( const SmartVector<S,TL,GL>& lhs, const SmartVector<S,TR,GR>& rhs )
{
   return lhs.size() != rhs.size() || !std::equal( lhs.begin(), lhs.end(), rhs.begin() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two smart pointer vectors.
 *
 * \param a The first smart pointer vector to be swapped.
 * \param b The second smart pointer vector to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
inline void swap( SmartVector<S,T,G>& a, SmartVector<S,T,G>& b ) /* throw() */
{
   a.swap( b );
}
//*************************************************************************************************








//=================================================================================================
//
//  NESTED CLASS SMARTVECTOR::CASTITERATOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Dynamic cast iterator for polymorphic smart pointer vectors.
 * \ingroup util
 *
 * The CastIterator class is part of the SmartVector class and represent a forward iterator
 * over all elements of type \a C contained in a range of elements of type \a T, where \a C
 * is a type derived from \a T.

   \code
   class A { ... };
   class B : public class A { ... };

   pe::SmartVector<SharedPtr,A>::CastIterator<B> begin;
   pe::SmartVector<SharedPtr,A>::CastIterator<B> end;

   // Loop over all elements of type B within the range [begin..end)
   for( ; begin!=end; ++begin )
      ...
   \endcode

 * \b Note: Using a CastIterator is computationally more expensive than using a standard
 * iterator over all elements contained in the vector.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
class SmartVector<S,T,G>::CastIterator
{
public:
   //**Type definitions****************************************************************************
   // pe naming convention
   typedef const S<T>  ValueType;       //!< Type of the underlying smart pointers.
   typedef const S<C>  CastType;        //!< Type of the resulting/casted smart pointers.
   typedef ValueType&  ReferenceType;   //!< Reference return type.
   typedef ValueType*  IteratorType;    //!< Type of the internal pointer.
   typedef ptrdiff_t   DifferenceType;  //!< Difference between two iterators.

   // STL iterator requirements
   /*! \cond PE_INTERNAL */
   typedef std::forward_iterator_tag  iterator_category;
   typedef ValueType                  value_type;
   typedef DifferenceType             difference_type;
   typedef IteratorType               pointer;
   typedef ReferenceType              reference;
   /*! \endcond */
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   inline CastIterator( IteratorType begin, IteratorType end );

   template< typename Other >
   inline CastIterator( const CastIterator<Other>& it );

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
   inline CastIterator& operator++();
   inline CastIterator  operator++( int );
   //@}
   //**********************************************************************************************

   //**Access operators****************************************************************************
   /*!\name Access operators */
   //@{
   inline CastType operator*()  const;
   inline CastType operator->() const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline const IteratorType& base() const;
   inline const IteratorType& stop() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   IteratorType cur_;  //!< Pointer to the current memory location.
   IteratorType end_;  //!< Pointer to the element one past the last element in the element range.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Standard constructor for CastIterator.
 *
 * \param begin The beginning of the element range.
 * \param end The end of the element range.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline SmartVector<S,T,G>::CastIterator<C>::CastIterator( IteratorType begin, IteratorType end )
   : cur_(begin)  // Pointer to the current memory location
   , end_(end)    // Pointer to the element one past the last element in the element range
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( C, T );
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( ValueType, CastType );

   while( cur_ != end_ && !dynamic_cast<C*>( cur_->get() ) ) ++cur_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different CastIterator instances.
 *
 * \param it The foreign CastIterator instance to be copied.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
template< typename Other >              // The foreign cast iterator type
inline SmartVector<S,T,G>::CastIterator<C>::CastIterator( const CastIterator<Other>& it )
   : cur_( reinterpret_cast<IteratorType>( it.base() ) )  // Pointer to the current memory location
   , end_( reinterpret_cast<IteratorType>( it.stop() ) )  // Pointer to the element one past the last element in the element range
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( C, T );
   pe_CONSTRAINT_POINTER_MUST_BE_CONVERTIBLE( Other*, T* );
   pe_CONSTRAINT_POINTER_MUST_BE_CONVERTIBLE( Other*, C* );
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( ValueType, CastType );
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
 * \return Reference to the incremented cast iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE CastIterator<C>&
   SmartVector<S,T,G>::CastIterator<C>::operator++()
{
   while( ++cur_ != end_ && !dynamic_cast<C*>( cur_->get() ) );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Post-increment operator.
 *
 * \return The incremented cast iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE CastIterator<C>
   SmartVector<S,T,G>::CastIterator<C>::operator++( int )
{
   CastIterator tmp( *this );
   while( ++cur_ != end_ && !dynamic_cast<C*>( cur_->get() ) );
   return tmp;
}
//*************************************************************************************************




//=================================================================================================
//
//  ACCESS OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a handle to the element at the current iterator position.
 *
 * \return Handle to the element at the current iterator position.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE CastIterator<C>::CastType
   SmartVector<S,T,G>::CastIterator<C>::operator*() const
{
   return CastType( *reinterpret_cast<CastType*>( cur_ ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Direct access to the element at the current iterator position.
 *
 * \return Reference to the element at the current iterator position.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE CastIterator<C>::CastType
   SmartVector<S,T,G>::CastIterator<C>::operator->() const
{
   return CastType( *reinterpret_cast<CastType*>( cur_ ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Direct access to the current memory location of the cast iterator.
 *
 * \return Pointer to the current memory location.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline const typename SmartVector<S,T,G>::pe_TEMPLATE CastIterator<C>::IteratorType&
   SmartVector<S,T,G>::CastIterator<C>::base() const
{
   return cur_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Direct access to the final memory location of the cast iterator.
 *
 * \return Pointer to the final memory location.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline const typename SmartVector<S,T,G>::pe_TEMPLATE CastIterator<C>::IteratorType&
   SmartVector<S,T,G>::CastIterator<C>::stop() const
{
   return end_;
}
//*************************************************************************************************








//=================================================================================================
//
//  NESTED CLASS SMARTVECTOR::CONSTCASTITERATOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Dynamic cast iterator for polymorphic smart pointer vectors.
 * \ingroup util
 *
 * The ConstCastIterator class is part of the SmartVector class and represent a forward iterator
 * over all elements of type \a C contained in a range of elements of type \a T, where \a C
 * is a type derived from \a T. The ConstCastIterator is the counterpart of CastIterator for
 * constant vectors.

   \code
   class A { ... };
   class B : public class A { ... };

   pe::SmartVector<SharedPtr,A>::ConstCastIterator<B> begin;
   pe::SmartVector<SharedPtr,A>::ConstCastIterator<B> end;

   // Loop over all elements of type B within the range [begin..end)
   for( ; begin!=end; ++begin )
      ...
   \endcode

 * \b Note: Using a ConstCastIterator is computationally more expensive than using a standard
 * iterator over all elements contained in the vector.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
class SmartVector<S,T,G>::ConstCastIterator
{
public:
   //**Type definitions****************************************************************************
   // pe naming convention
   typedef const S<const T>  ValueType;       //!< Type of the underlying smart pointers.
   typedef const S<const C>  CastType;        //!< Type of the resulting/casted smart pointers.
   typedef ValueType&        ReferenceType;   //!< Reference return type.
   typedef ValueType*        IteratorType;    //!< Type of the internal pointer.
   typedef ptrdiff_t         DifferenceType;  //!< Difference between two iterators.

   // STL iterator requirements
   /*! \cond PE_INTERNAL */
   typedef std::forward_iterator_tag  iterator_category;
   typedef ValueType                  value_type;
   typedef DifferenceType             difference_type;
   typedef IteratorType               pointer;
   typedef ReferenceType              reference;
   /*! \endcond */
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   inline ConstCastIterator( IteratorType begin, IteratorType end );

   template< typename Other >
   inline ConstCastIterator( const ConstCastIterator<Other>& it );

   template< typename Other >
   inline ConstCastIterator( const typename SmartVector<S,T,G>::pe_TEMPLATE CastIterator<Other>& it );

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
   inline ConstCastIterator& operator++();
   inline ConstCastIterator  operator++( int );
   //@}
   //**********************************************************************************************

   //**Access operators****************************************************************************
   /*!\name Access operators */
   //@{
   inline CastType operator*()  const;
   inline CastType operator->() const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline const IteratorType& base() const;
   inline const IteratorType& stop() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   IteratorType cur_;  //!< Pointer to the current memory location.
   IteratorType end_;  //!< Pointer to the element one past the last element in the element range.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Standard constructor for ConstCastIterator.
 *
 * \param begin The beginning of the element range.
 * \param end The end of the element range.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline SmartVector<S,T,G>::ConstCastIterator<C>::ConstCastIterator( IteratorType begin, IteratorType end )
   : cur_(begin)  // Pointer to the current memory location
   , end_(end)    // Pointer to the element one past the last element in the element range
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( C, T );
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( ValueType, CastType );

   while( cur_ != end_ && !dynamic_cast<const C*>( cur_->get() ) ) ++cur_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different ConstCastIterator instances.
 *
 * \param it The foreign ConstCastIterator instance to be copied.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
template< typename Other >              // The foreign constant cast iterator type
inline SmartVector<S,T,G>::ConstCastIterator<C>::ConstCastIterator( const ConstCastIterator<Other>& it )
   : cur_( reinterpret_cast<IteratorType>( it.base() ) )  // Pointer to the current memory location
   , end_( reinterpret_cast<IteratorType>( it.stop() ) )  // Pointer to the element one past the last element in the element range
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( C, T );
   pe_CONSTRAINT_POINTER_MUST_BE_CONVERTIBLE( Other*, T* );
   pe_CONSTRAINT_POINTER_MUST_BE_CONVERTIBLE( Other*, C* );
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( ValueType, CastType );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from CastIterator instances.
 *
 * \param it The foreign CastIterator instance to be copied.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
template< typename Other >              // The foreign cast iterator type
inline SmartVector<S,T,G>::ConstCastIterator<C>::ConstCastIterator( const typename SmartVector<S,T,G>::pe_TEMPLATE CastIterator<Other>& it )
   : cur_( reinterpret_cast<IteratorType>( it.base() ) )  // Pointer to the current memory location
   , end_( reinterpret_cast<IteratorType>( it.stop() ) )  // Pointer to the element one past the last element in the element range
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( C, T );
   pe_CONSTRAINT_POINTER_MUST_BE_CONVERTIBLE( Other*, T* );
   pe_CONSTRAINT_POINTER_MUST_BE_CONVERTIBLE( Other*, C* );
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
 * \return Reference to the incremented cast iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE ConstCastIterator<C>&
   SmartVector<S,T,G>::ConstCastIterator<C>::operator++()
{
   while( ++cur_ != end_ && !dynamic_cast<const C*>( cur_->get() ) );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Post-increment operator.
 *
 * \return The incremented cast iterator.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE ConstCastIterator<C>
   SmartVector<S,T,G>::ConstCastIterator<C>::operator++( int )
{
   ConstCastIterator tmp( *this );
   while( ++cur_ != end_ && !dynamic_cast<const C*>( cur_->get() ) );
   return tmp;
}
//*************************************************************************************************




//=================================================================================================
//
//  ACCESS OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a handle to the element at the current iterator position.
 *
 * \return Handle to the element at the current iterator position.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE ConstCastIterator<C>::CastType
   SmartVector<S,T,G>::ConstCastIterator<C>::operator*() const
{
   return CastType( *reinterpret_cast<CastType*>( cur_ ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Direct access to the element at the current iterator position.
 *
 * \return Reference to the element at the current iterator position.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline typename SmartVector<S,T,G>::pe_TEMPLATE ConstCastIterator<C>::CastType
   SmartVector<S,T,G>::ConstCastIterator<C>::operator->() const
{
   return CastType( *reinterpret_cast<CastType*>( cur_ ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Direct access to the current memory location of the constant cast iterator.
 *
 * \return Pointer to the current memory location.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline const typename SmartVector<S,T,G>::pe_TEMPLATE ConstCastIterator<C>::IteratorType&
   SmartVector<S,T,G>::ConstCastIterator<C>::base() const
{
   return cur_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Direct access to the final memory location of the constant cast iterator.
 *
 * \return Pointer to the final memory location.
 */
template< template< typename > class S  // Type of the smart pointer
        , typename T                    // Type of the smart pointer elements
        , typename G >                  // Growth policy
template< typename C >                  // Cast type
inline const typename SmartVector<S,T,G>::pe_TEMPLATE ConstCastIterator<C>::IteratorType&
   SmartVector<S,T,G>::ConstCastIterator<C>::stop() const
{
   return end_;
}
//*************************************************************************************************

} // namespace pe

#endif
