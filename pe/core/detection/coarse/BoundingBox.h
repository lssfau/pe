//=================================================================================================
/*!
 *  \file pe/core/detection/coarse/BoundingBox.h
 *  \brief Headerfile for an axis-aligned bounding box for rigid bodies
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

#ifndef _PE_CORE_DETECTION_COARSE_BOUNDINGBOX_H_
#define _PE_CORE_DETECTION_COARSE_BOUNDINGBOX_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <ostream>
#include <pe/math/Functions.h>
#include <pe/math/Vector3.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

namespace detection {

namespace coarse {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Axis-aligned bounding box for rigid bodies.
 * \ingroup coarse_collision_detection
 *
 * An axis-aligned bounding box is a bounding volume for rigid bodies. One basic property of an
 * axis-aligned bounding box is that it is always aligned with the axes of the global world frame.
 * Therefore the bounding box can be represented by two vertices: the first gives the coordinates
 * of the bottom-left-front vertex \f$ (x_1,y_1,z_1) \f$, the second is the upper-right-back vertex
 * \f$ (x_2,y_2,z_2) \f$. Due to this arrangement, the x-, y- and z-component of the first vertex
 * have to be smaller than their counterparts in the second vertex, otherwise the bounding box is
 * invalid.
 *
 * \image html aabb.png
 * \image latex aabb.eps "Axis-aligned bounding box" width=800pt
 *
 * Example:

   \code
   using namespace pe;
   using namespace pe::detection::coarse;

   typedef BoundingBox<real> AABB;

   // Creating a default bounding box with the vertices (-1,-1,-1) and (1,1,1)
   AABB aabb1;

   // Creating a bounding box with the vertices (1,1,1) and (2,2,2)
   AABB aabb2( 1, 1, 1, 2, 2, 2 );

   // Merging the two bounding boxes results in a bounding box from (-1,-1,-1) to (2,2,2)
   AABB aabb3 = aabb1 + aabb2;
   \endcode
 */
template< typename Type >  // Element type of the axis-aligned bounding box
class BoundingBox
{
public:
   //**Type definitions****************************************************************************
   typedef Type ElementType;  //!< Type of the bounding box elements.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline BoundingBox();
   explicit inline BoundingBox( const Type& x1, const Type& y1, const Type& z1,
                                const Type& x2, const Type& y2, const Type& z2 );

   template< typename Other >
   explicit inline BoundingBox( const Vector3<Other>& a, const Vector3<Other>& b );

            inline BoundingBox( const BoundingBox& aabb );

   template< typename Other >
            inline BoundingBox( const BoundingBox<Other>& aabb );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Bounding box operators**********************************************************************
   /*!\name Bounding box operators */
   //@{
                              inline BoundingBox&      operator= ( const BoundingBox& rhs );
   template< typename Other > inline BoundingBox&      operator= ( const BoundingBox<Other>& rhs );
                              inline BoundingBox&      operator+=( const BoundingBox& rhs );
                              inline const BoundingBox operator+ ( const BoundingBox& rhs ) const;
                              inline Type&             operator[]( size_t index );
                              inline const Type&       operator[]( size_t index )           const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
                              inline bool isValid()                               const;
                              inline void reset();
   template< typename Other > inline bool equal( const BoundingBox<Other>& aabb ) const;

   template< typename Other > inline bool overlaps( const BoundingBox<Other>& aabb          ) const;
   template< typename Other > inline bool overlaps( const BoundingBox<Other>& aabb, real dx ) const;
                              inline bool contains( real px, real py, real pz               ) const;
                              inline bool contains( const Vec3& gpos                        ) const;
   template< typename Other > inline bool contains( const BoundingBox<Other>& aabb          ) const;
   template< typename Other > inline bool contains( const BoundingBox<Other>& aabb, real dx ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Type v_[6];  //!< Bottom-left-front and upper-right-back vertex.
                /*!< The first three values are used for the Bottom-left-front vertex <x1,y1,z1>
                     and the last three values for the upper-right-back vertex <x2,y2,z2>. */
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   template< typename Other > friend class BoundingBox;
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
/*!\brief Default constructor for the axis-aligned bounding box.
 *
 * This constructor initializes the axis-aligned bounding box to the default values
 * (-1,-1,-1) and (1,1,1).
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline BoundingBox<Type>::BoundingBox()
{
   reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the direct initialization of the bounding box.
 *
 * \param x1 x-component of the bottom-left-front vertex.
 * \param y1 y-component of the bottom-left-front vertex.
 * \param z1 z-component of the bottom-left-front vertex.
 * \param x2 x-component of the upper-right-back vertex.
 * \param y2 y-component of the upper-right-back vertex.
 * \param z2 z-component of the upper-right-back vertex.
 *
 * Direct initialization of the bounding box. The first vertex is initialized as (x1,y1,z1),
 * whereas the second bounding box is set to (x2,y2,z2).
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline BoundingBox<Type>::BoundingBox( const Type& x1, const Type& y1, const Type& z1,
                                       const Type& x2, const Type& y2, const Type& z2 )
{
   pe_USER_ASSERT( ( x1 < x2 && y1 < y2 && z1 < z2 ), "Invalid bounding box values" );

   v_[0] = x1;
   v_[1] = y1;
   v_[2] = z1;
   v_[3] = x2;
   v_[4] = y2;
   v_[5] = z2;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the direct initialization of the bounding box.
 *
 * \param a The bottom-left-front vertex.
 * \param b The upper-right-back vertex.
 *
 * Direct initialization of the bounding box. The first vertex is initialized with \a a,
 * whereas the second bounding box is set to \a b.
 */
template< typename Type >   // Element type of the axis-aligned bounding box
template< typename Other >  // Element type of the vectors
inline BoundingBox<Type>::BoundingBox( const Vector3<Other>& a, const Vector3<Other>& b )
{
   v_[0] = a[0];
   v_[1] = a[1];
   v_[2] = a[2];
   v_[3] = b[0];
   v_[4] = b[1];
   v_[5] = b[2];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy constructor for axis-aligned bounding boxes.
 *
 * \param aabb Bounding box to be copied.
 *
 * Explicit definition of a copy constructor in order to enable/facilitate NRV optimization.
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline BoundingBox<Type>::BoundingBox( const BoundingBox& aabb )
{
   v_[0] = aabb.v_[0];
   v_[1] = aabb.v_[1];
   v_[2] = aabb.v_[2];
   v_[3] = aabb.v_[3];
   v_[4] = aabb.v_[4];
   v_[5] = aabb.v_[5];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different BoundingBox instances.
 *
 * \param aabb Bounding box to be copied.
 */
template< typename Type >   // Element type of the axis-aligned bounding box
template< typename Other >  // Element type of the foreign axis-aligned bounding box
inline BoundingBox<Type>::BoundingBox( const BoundingBox<Other>& aabb )
{
   v_[0] = aabb.v_[0];
   v_[1] = aabb.v_[1];
   v_[2] = aabb.v_[2];
   v_[3] = aabb.v_[3];
   v_[4] = aabb.v_[4];
   v_[5] = aabb.v_[5];
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Copy assignment operator for axis-aligned bounding boxes.
 *
 * \param rhs Bounding box to be copied.
 * \return Reference to the assigned bounding box.
 *
 * Explicit definition of a copy assignment operator for performance reasons.
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline BoundingBox<Type>& BoundingBox<Type>::operator=( const BoundingBox& rhs )
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
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy assignment operator for different axis-aligned bounding box instances.
 *
 * \param rhs Bounding box to be copied.
 * \return Reference to the assigned bounding box.
 */
template< typename Type >   // Element type of the axis-aligned bounding box
template< typename Other >  // Element type of the foreign axis-aligned bounding box
inline BoundingBox<Type>& BoundingBox<Type>::operator=( const BoundingBox<Other>& rhs )
{
   v_[0] = rhs.v_[0];
   v_[1] = rhs.v_[1];
   v_[2] = rhs.v_[2];
   v_[3] = rhs.v_[3];
   v_[4] = rhs.v_[4];
   v_[5] = rhs.v_[5];
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merge operation for two axis-aligned bounding boxes.
 *
 * \param rhs Right-hand side bounding box to be merged into the left-hand side bounding box.
 * \return Reference to the (adjusted) left-hand side bounding box.
 *
 * Merging of a second axis-aligned bounding box into the left-hand side bounding box to form
 * a combined bounding box that contains both original bounding boxes.
 *
 * \b Note: The result of the merging process is undefined for invalid bounding boxes!
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline BoundingBox<Type>& BoundingBox<Type>::operator+=( const BoundingBox& rhs )
{
   v_[0] = min( v_[0], rhs.v_[0] );
   v_[1] = min( v_[1], rhs.v_[1] );
   v_[2] = min( v_[2], rhs.v_[2] );
   v_[3] = max( v_[3], rhs.v_[3] );
   v_[4] = max( v_[4], rhs.v_[4] );
   v_[5] = max( v_[5], rhs.v_[5] );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merge operation for two axis-aligned bounding boxes.
 *
 * \param rhs Right-hand side bounding box to be merged with the left-hand side bounding box.
 * \return The new combined bounding box.
 *
 * Merging of two axis-aligned bounding boxes into a new combined bounding box that contains
 * both original bounding boxes.
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline const BoundingBox<Type> BoundingBox<Type>::operator+( const BoundingBox& rhs ) const
{
   return BoundingBox( min( v_[0], rhs.v_[0] ),
                       min( v_[1], rhs.v_[1] ),
                       min( v_[2], rhs.v_[2] ),
                       max( v_[3], rhs.v_[3] ),
                       max( v_[4], rhs.v_[4] ),
                       max( v_[5], rhs.v_[5] ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Direct access to the bounding box vertices/values.
 *
 * \param index Index of the accessed bounding box value.
 * \return Reference to the accessed value.
 *
 * Direct access to the bounding box vertices. The first vertex is accessed with the indices
 * [0..2], whereas the second vertex is accessed with the indices [3..5].
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline Type& BoundingBox<Type>::operator[]( size_t index )
{
   pe_USER_ASSERT( index < 6, "Invalid bounding box access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Direct access to the bounding box vertices/values.
 *
 * \param index Index of the accessed bounding box value.
 * \return Reference-to-const to the accessed value.
 *
 * Direct access to the bounding box vertices. The first vertex is accessed with the indices
 * [0..2], whereas the second vertex is accessed with the indices [3..5].
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline const Type& BoundingBox<Type>::operator[]( size_t index ) const
{
   pe_USER_ASSERT( index < 6, "Invalid bounding box access index" );
   return v_[index];
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Validity check on a axis-aligned bounding box.
 *
 * \return \a true if the bounding box is valid, \a false if it is not.
 *
 * Checking the validity of the axis-aligned bounding box: all components of the first bottom-
 * left-front vertex have to be smaller than their counterparts in the second upper-right-back
 * vertex.
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline bool BoundingBox<Type>::isValid() const
{
   if( v_[0] >= v_[3] || v_[1] >= v_[4] || v_[2] >= v_[5] ) return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the axis-aligned bounding box to the default values.
 *
 * \return void
 *
 * This function resets the axis-aligned bounding box to its default values: the bottom-left-front
 * vertex is reset to (-1,-1,-1), the upper-right-back vertex is reset to (1,1,1).
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline void BoundingBox<Type>::reset()
{
   v_[0] = -1;
   v_[1] = -1;
   v_[2] = -1;
   v_[3] =  1;
   v_[4] =  1;
   v_[5] =  1;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison of two axis-aligned bounding boxes.
 *
 * \param rhs Right-hand side bounding box to be compared.
 * \return \a true if the two bounding boxes are equal, \a false if not.
 */
template< typename Type >   // Element type of the axis-aligned bounding box
template< typename Other >  // Element type of the foreign axis-aligned bounding box
inline bool BoundingBox<Type>::equal( const BoundingBox<Other>& rhs ) const
{
   if( !pe::equal( v_[0], rhs.v_[0] ) || !pe::equal( v_[1], rhs.v_[1] ) || !pe::equal( v_[2], rhs.v_[2] ) ||
       !pe::equal( v_[3], rhs.v_[3] ) || !pe::equal( v_[4], rhs.v_[4] ) || !pe::equal( v_[5], rhs.v_[5] ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between two axis-aligned bounding boxes.
 *
 * \param aabb The second axis-aligned bounding box.
 * \return \a true if the two bounding boxes overlap, \a false if not.
 *
 * This function tests, whether the bounding box overlaps with the second bounding box \a aabb.
 * In case the bounding box overlaps with \a aabb, the function returns \a true, otherwise the
 * function returns \a false.
 */
template< typename Type >   // Element type of the axis-aligned bounding box
template< typename Other >  // Element type of the foreign axis-aligned bounding box
inline bool BoundingBox<Type>::overlaps( const BoundingBox<Other>& aabb ) const
{
   pe_USER_ASSERT( isValid()     , "Invalid bounding box" );
   pe_USER_ASSERT( aabb.isValid(), "Invalid bounding box" );

   if( aabb.v_[0] > v_[3] || aabb.v_[1] > v_[4] || aabb.v_[2] > v_[5] ||
       aabb.v_[3] < v_[0] || aabb.v_[4] < v_[1] || aabb.v_[5] < v_[2] )
      return false;
   else
      return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between two axis-aligned bounding boxes.
 *
 * \param aabb The second axis-aligned bounding box.
 * \param dx Spacing between the two bounding boxes.
 * \return \a true if the two bounding boxes overlap, \a false if not.
 *
 * In this overlap test, the first bounding box is increased by \a dx in all directions. The
 * function detects an overlap, if the second bounding box overlaps the first, increased
 * bounding box. In this case, the function returns \a true, otherwise it returns \a false.
 */
template< typename Type >   // Element type of the axis-aligned bounding box
template< typename Other >  // Element type of the foreign axis-aligned bounding box
inline bool BoundingBox<Type>::overlaps( const BoundingBox<Other>& aabb, real dx ) const
{
   pe_USER_ASSERT( isValid()     , "Invalid bounding box" );
   pe_USER_ASSERT( aabb.isValid(), "Invalid bounding box" );

   if( aabb.v_[0] > v_[3]+dx || aabb.v_[1] > v_[4]+dx || aabb.v_[2] > v_[5]+dx ||
       aabb.v_[3] < v_[0]-dx || aabb.v_[4] < v_[1]-dx || aabb.v_[5] < v_[2]-dx )
      return false;
   else
      return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Intersection test for global coordinates.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the coordinate is contained in the bounding box, \a false if not.
 *
 * This function tests, whether the given global coordinate is contained in or on the surface
 * of the bounding box. In case the point is contained in the bounding box, the function returns
 * \a true. Otherwise \a false is returned.
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline bool BoundingBox<Type>::contains( real px, real py, real pz ) const
{
   if( px < v_[0] || py < v_[1] || pz < v_[2] ||
       px > v_[3] || py > v_[4] || pz > v_[5] )
      return false;
   else
      return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Intersection test for global coordinates.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained in the bounding box, \a false if not.
 *
 * This function tests, whether the given global coordinate is contained in or on the surface
 * of the bounding box. In case \a gpos is contained in the bounding box, the function returns
 * \a true. Otherwise \a false is returned.
 */
template< typename Type >  // Element type of the axis-aligned bounding box
inline bool BoundingBox<Type>::contains( const Vec3& gpos ) const
{
   if( gpos[0] < v_[0] || gpos[1] < v_[1] || gpos[2] < v_[2] ||
       gpos[0] > v_[3] || gpos[1] > v_[4] || gpos[2] > v_[5] )
      return false;
   else
      return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Intersection test for axis-aligned bounding boxes.
 *
 * \param aabb The second axis-aligned bounding box.
 * \return \a true if the bounding box contains the other bounding box, \a false if not.
 *
 * This function tests, whether the bounding box contains the second bounding box \a aabb.
 * In case \a aabb is completely contained in the bounding box, the function returns \a true.
 * Otherwise \a false is returned.
 */
template< typename Type >   // Element type of the axis-aligned bounding box
template< typename Other >  // Element type of the foreign axis-aligned bounding box
inline bool BoundingBox<Type>::contains( const BoundingBox<Other>& aabb ) const
{
   pe_USER_ASSERT( isValid()     , "Invalid bounding box" );
   pe_USER_ASSERT( aabb.isValid(), "Invalid bounding box" );

   if( aabb.v_[0] < v_[0] || aabb.v_[1] < v_[1] || aabb.v_[2] < v_[2] ||
       aabb.v_[3] > v_[3] || aabb.v_[4] > v_[4] || aabb.v_[5] > v_[5] )
      return false;
   else
      return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Intersection test for axis-aligned bounding boxes.
 *
 * \param aabb The second axis-aligned bounding box.
 * \param dx Spacing between the two bounding boxes.
 * \return \a true if the bounding box contains the other bounding box, \a false if not.
 *
 * In this intersection test, the first bounding box is increased by \a dx in all directions. The
 * function tests, if the increased axis-aligned bounding box contains the second bounding box.
 * In case \a aabb is completely contained in the bounding box, the function returns \a true.
 * Otherwise \a false is returned.
 */
template< typename Type >   // Element type of the axis-aligned bounding box
template< typename Other >  // Element type of the foreign axis-aligned bounding box
inline bool BoundingBox<Type>::contains( const BoundingBox<Other>& aabb, real dx ) const
{
   pe_USER_ASSERT( isValid()     , "Invalid bounding box" );
   pe_USER_ASSERT( aabb.isValid(), "Invalid bounding box" );

   if( aabb.v_[0] < v_[0]-dx || aabb.v_[1] < v_[1]-dx || aabb.v_[2] < v_[2]-dx ||
       aabb.v_[3] > v_[3]+dx || aabb.v_[4] > v_[4]+dx || aabb.v_[5] > v_[5]+dx )
      return false;
   else
      return true;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Axis-aligned bounding box operators */
//@{
template< typename T1, typename T2 >
inline bool operator==( const BoundingBox<T1>& lhs, const BoundingBox<T2>& rhs );

template< typename T1, typename T2 >
inline bool operator!=( const BoundingBox<T1>& lhs, const BoundingBox<T2>& rhs );

template< typename Type >
inline std::ostream& operator<<( std::ostream& os, const BoundingBox<Type>& aabb );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between two axis-aligned bounding boxes.
 * \ingroup coarse_collision_detection
 *
 * \param lhs Left-hand side bounding box to be compared.
 * \param rhs Right-hand side bounding box to be compared.
 * \return \a true if the two bounding boxes are equal, \a false if not.
 */
template< typename T1    // Element type of the left-hand side axis-aligned bounding box
        , typename T2 >  // Element type of the right-hand side axis-aligned bounding box
inline bool operator==( const BoundingBox<T1>& lhs, const BoundingBox<T2>& rhs )
{
   return lhs.equal( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between two axis-aligned bounding boxes.
 * \ingroup coarse_collision_detection
 *
 * \param lhs Left-hand side bounding box to be compared.
 * \param rhs Right-hand side bounding box to be compared.
 * \return \a true if the two bounding boxes are not equal, \a false if they are equal.
 */
template< typename T1    // Element type of the left-hand side axis-aligned bounding box
        , typename T2 >  // Element type of the right-hand side axis-aligned bounding box
inline bool operator!=( const BoundingBox<T1>& lhs, const BoundingBox<T2>& rhs )
{
   return !lhs.equal( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for axis-aligned bounding boxes.
 * \ingroup coarse_collision_detection
 *
 * \param os Reference to the output stream.
 * \param aabb Reference to an axis-aligned bounding box.
 * \return Reference to the output stream.
 */
template< typename Type >
inline std::ostream& operator<<( std::ostream& os, const BoundingBox<Type>& aabb )
{
   return os << "<" << aabb[0] << "," << aabb[1] << "," << aabb[2] << "> , "
             << "<" << aabb[3] << "," << aabb[4] << "," << aabb[5] << ">";
}
//*************************************************************************************************

} // namespace coarse

} // namespace detection

} // namespace pe

#endif
