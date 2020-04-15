//=================================================================================================
/*!
 *  \file pe/core/detection/coarse/BoundingSphere.h
 *  \brief Headerfile for a bounding sphere for rigid bodies
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

#ifndef _PE_CORE_DETECTION_COARSE_BOUNDINGSPHERE_H_
#define _PE_CORE_DETECTION_COARSE_BOUNDINGSPHERE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <ostream>
#include <pe/math/shims/Equal.h>
#include <pe/math/shims/Square.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
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
/*!\brief Bounding sphere for rigid bodies.
 * \ingroup coarse_collision_detection
 *
 * A bounding sphere is a bounding volume for rigid bodies. The radius of a bounding sphere
 * is chosen such that it contains the entire rigid body (see the example illustrations).
 * The bounding sphere merely consists of its radius and the global position of its center.
 *
 * \image html boundingsphere.png
 * \image latex boundingsphere.eps "Bounding sphere" width=750pt
 */
class BoundingSphere
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline BoundingSphere();
   explicit inline BoundingSphere( real x, real y, real z, real radius );
   explicit inline BoundingSphere( const Vec3& center, real radius );
            inline BoundingSphere( const BoundingSphere& bs );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Bounding sphere operators*******************************************************************
   /*!\name Bounding sphere operators */
   //@{
   inline BoundingSphere&      operator= ( const BoundingSphere& rhs );
   inline bool                 operator==( const BoundingSphere& rhs ) const;
   inline bool                 operator!=( const BoundingSphere& rhs ) const;
   inline BoundingSphere&      operator+=( const BoundingSphere& rhs );
   inline const BoundingSphere operator+ ( const BoundingSphere& rhs ) const;
   inline real&                operator[]( size_t index );
   inline const real&          operator[]( size_t index )         const;
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline Vec3&       getCenter();
   inline const Vec3& getCenter() const;
   inline real        getRadius() const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void        setRadius( real radius );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool isValid() const;
   inline void reset();

   inline bool overlaps( real x, real y, real z, real radius, real dx=0 ) const;
   inline bool overlaps( const BoundingSphere& bs, real dx=0 )            const;
   inline bool contains( real px, real py, real pz )                      const;
   inline bool contains( const Vec3& gpos )                               const;
   inline bool contains( real x, real y, real z, real radius, real dx=0 ) const;
   inline bool contains( const BoundingSphere& bs, real dx=0 )            const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 center_;  //!< Geometric center of the bounding sphere.
   real radius_;  //!< Radius of the bounding sphere.
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
/*!\brief Default constructor for the bounding sphere.
 *
 * The default constructor for the BoundingSphere class initializes the center of the bounding
 * sphere to (0,0,0) and the radius to 0.
 */
inline BoundingSphere::BoundingSphere()
   : center_()     // Geometric center of the bounding sphere
   , radius_( 0 )  // Radius of the bounding sphere
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the direct initialization of the bounding sphere.
 *
 * \param x x-component of the bounding sphere's center.
 * \param y y-component of the bounding sphere's center.
 * \param z z-component of the bounding sphere's center.
 * \param radius Radius of the bouding sphere \f$ (0..\infty) \f$.
 */
inline BoundingSphere::BoundingSphere( real x, real y, real z, real radius )
   : center_( x, y, z )  // Geometric center of the bounding sphere
   , radius_( radius )   // Radius of the bounding sphere
{
   pe_USER_ASSERT( radius > real(0), "Invalid bounding sphere radius" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the direct initialization of the bounding sphere.
 *
 * \param center The center of the bounding sphere.
 * \param radius Radius of the bouding sphere \f$ (0..\infty) \f$.
 */
inline BoundingSphere::BoundingSphere( const Vec3& center, real radius )
   : center_( center )  // Geometric center of the bounding sphere
   , radius_( radius )  // Radius of the bounding sphere
{
   pe_USER_ASSERT( radius > real(0), "Invalid bounding sphere radius" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy constructor for bounding spheres.
 *
 * \param bs The bounding sphere to be copied.
 *
 * Explicit definition of a copy constructor in order to enable/facilitate NRV optimization.
 */
inline BoundingSphere::BoundingSphere( const BoundingSphere& bs )
{
   center_ = bs.center_;
   radius_ = bs.radius_;
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Copy assignment operator for bounding spheres.
 *
 * \param rhs Bounding sphere to be copied.
 * \return Reference to the assigned bounding sphere.
 *
 * Explicit definition of a copy assignment operator for performance reasons.
 */
inline BoundingSphere& BoundingSphere::operator=( const BoundingSphere& rhs )
{
   // This implementation is faster than the synthesized default copy assignment operator and
   // faster than an implementation with the C library function 'memcpy' in combination with a
   // protection against self-assignment. Additionally, this version goes without a protection
   // against self-assignment.
   center_ = rhs.center_;
   radius_ = rhs.radius_;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between two bounding spheres.
 *
 * \param rhs Right-hand side bounding sphere to be compared.
 * \return \a true if the two bounding spheres are equal, \a false if not.
 */
inline bool BoundingSphere::operator==( const BoundingSphere& rhs ) const
{
   if( center_ != rhs.center_ || !equal( radius_, rhs.radius_ ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between two bounding spheres.
 *
 * \param rhs Right-hand side bounding sphere to be compared.
 * \return \a true if the two bounding spheres are not equal, \a false if they are equal.
 */
inline bool BoundingSphere::operator!=( const BoundingSphere& rhs ) const
{
   if( center_ != rhs.center_ || !equal( radius_, rhs.radius_ ) )
      return true;
   else return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merge operation for two bounding spheres.
 *
 * \param rhs Right-hand side bounding sphere to be merged into the left-hand side bounding sphere.
 * \return Reference to the (adjusted) left-hand side bounding sphere.
 *
 * Merging of a second bounding sphere into the left-hand side bounding sphere to form a combined
 * bounding sphere that contains both original bounding spheres.
 */
inline BoundingSphere& BoundingSphere::operator+=( const BoundingSphere& rhs )
{
   const Vec3 diff  ( rhs.center_ - center_ );
   const real dist  ( diff.length() );
   const Vec3 dir   ( diff / dist );
   const real extent( dist + radius_ + rhs.radius_ );

   center_ = rhs.center_ + rhs.radius_*dir - real(0.5)*extent*dir;
   radius_ = real(0.5)*extent;

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merge operation for two bounding spheres.
 *
 * \param rhs Right-hand side bounding sphere to be merged with the left-hand side bounding sphere.
 * \return The new combined bounding sphere.
 *
 * Merging of two bounding spheres into a new combined bounding sphere that contains both
 * original bounding spheres.
 */
inline const BoundingSphere BoundingSphere::operator+( const BoundingSphere& rhs ) const
{
   const Vec3 diff  ( rhs.center_ - center_ );
   const real dist  ( diff.length() );
   const Vec3 dir   ( diff / dist );
   const real extent( dist + radius_ + rhs.radius_ );

   return BoundingSphere( rhs.center_ + rhs.radius_*dir - real(0.5)*extent*dir,
                          real(0.5)*extent );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Direct access to the bounding sphere center values.
 *
 * \param index Index of the accessed bounding sphere center value \f$[0..2]\f$.
 * \return Reference to the accessed value.
 */
inline real& BoundingSphere::operator[]( size_t index )
{
   pe_USER_ASSERT( index < 3, "Invalid bounding sphere access index" );
   return center_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Direct access to the bounding sphere center values.
 *
 * \param index Index of the accessed bounding sphere center value \f$[0..2]\f$.
 * \return Reference-to-const to the accessed value.
 */
inline const real& BoundingSphere::operator[]( size_t index ) const
{
   pe_USER_ASSERT( index < 3, "Invalid bounding sphere access index" );
   return center_[index];
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the center of the bounding sphere.
 *
 * \return The center of the bounding sphere.
 */
inline Vec3& BoundingSphere::getCenter()
{
   return center_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the center of the bounding sphere.
 *
 * \return The center of the bounding sphere.
 */
inline const Vec3& BoundingSphere::getCenter() const
{
   return center_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the radius of the bounding sphere.
 *
 * \return The radius of the bounding sphere.
 */
inline real BoundingSphere::getRadius() const
{
   return radius_;
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the radius of the bounding sphere.
 *
 * \param radius The radius of the bounding sphere \f$ (0..\infty) \f$.
 * \return void
 */
inline void BoundingSphere::setRadius( real radius )
{
   pe_USER_ASSERT( radius > real(0), "Invalid bounding sphere radius" );
   radius_ = radius;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Validity check on a bounding sphere.
 *
 * \return \a true if the bounding sphere is valid, \a false if it is not.
 *
 * Checking the validity of the bouding sphere: the radius of the bounding sphere has to
 * be positive.
 */
inline bool BoundingSphere::isValid() const
{
   if( radius_ > real(0) ) return true;
   else return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the bounding sphere to the default values.
 *
 * \return void
 *
 * Resetting the center to (0,0,0) and the radius to 0.
 */
inline void BoundingSphere::reset()
{
   center_.reset();
   radius_ = real(0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between two bounding spheres.
 *
 * \param x x-component of the bounding sphere's center.
 * \param y y-component of the bounding sphere's center.
 * \param z z-component of the bounding sphere's center.
 * \param radius Radius of the bouding sphere \f$ (0..\infty) \f$.
 * \param dx Spacing between the two bounding spheres.
 * \return \a true if the two bounding spheres overlap, \a false if not.
 *
 * In this overlap test, the radius of the first bounding sphere is increased by \a dx. The
 * function detects an overlap, if the second bounding sphere at position (x,y,z) with radius
 * \a radius overlaps the first, increased bounding sphere.
 *
 * \b Note: In debug mode the validity of both bounding spheres is asserted by pe_USER_ASSERTs.
 */
inline bool BoundingSphere::overlaps( real x, real y, real z, real radius, real dx ) const
{
   pe_USER_ASSERT( isValid(), "Invalid bounding sphere" );
   pe_USER_ASSERT( radius > real(0), "Invalid bounding sphere radius" );

   if( sq(center_[0]-x) + sq(center_[1]-y) + sq(center_[2]-z) > sq( radius_ + radius + dx ) )
      return false;
   else
      return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between two bounding spheres.
 *
 * \param bs The second bounding sphere.
 * \param dx Spacing between the two bounding spheres.
 * \return \a true if the two bounding spheres overlap, \a false if not.
 *
 * In this overlap test, the radius of the first bounding sphere is increased by \a dx. The
 * function detects an overlap, if the second bounding sphere overlaps the first, increased
 * bounding sphere.
 *
 * \b Note: In debug mode the validity of both bounding spheres is asserted by pe_USER_ASSERTs.
 */
inline bool BoundingSphere::overlaps( const BoundingSphere& bs, real dx ) const
{
   pe_USER_ASSERT( isValid()   , "Invalid bounding sphere" );
   pe_USER_ASSERT( bs.isValid(), "Invalid bounding sphere" );

   if( ( center_ - bs.center_ ).sqrLength() > sq( radius_ + bs.radius_ + dx ) )
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
 * \return \a true if the coordinate is contained in the bounding sphere, \a false if not.
 *
 * This function tests, whether the given global coordinate is contained in or on the surface
 * of the bounding sphere. In case the point is contained in the bounding sphere, the function
 * returns \a true. Otherwise \a false is returned.
 */
inline bool BoundingSphere::contains( real px, real py, real pz ) const
{
   if( sq(center_[0]-px) + sq(center_[1]-py) + sq(center_[2]-pz) > radius_*radius_ )
      return false;
   else
      return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Intersection test for global coordinates.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained in the bounding sphere, \a false if not.
 *
 * This function tests, whether the given global coordinate is contained in or on the surface
 * of the bounding sphere. In case \a gpos is contained in the bounding sphere, the function
 * returns \a true. Otherwise \a false is returned.
 */
inline bool BoundingSphere::contains( const Vec3& gpos ) const
{
   if( sq(center_[0]-gpos[0]) + sq(center_[1]-gpos[1]) + sq(center_[2]-gpos[2]) > radius_*radius_ )
      return false;
   else
      return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Intersection test for bounding spheres.
 *
 * \param x x-component of the bounding sphere's center.
 * \param y y-component of the bounding sphere's center.
 * \param z z-component of the bounding sphere's center.
 * \param radius Radius of the bouding sphere \f$ (0..\infty) \f$.
 * \param dx Spacing between the two bounding spheres.
 * \return \a true if the bounding sphere contains the other bounding sphere, \a false if not.
 *
 * In this intersection test, the radius of the first bounding sphere is increased by \a dx.
 * The function tests, if the increased bounding sphere contains the second bounding sphere
 * at (x,y,z) with radius \a radius.
 *
 * \b Note: In debug mode the validity of both bounding spheres is asserted by pe_USER_ASSERTs.
 */
inline bool BoundingSphere::contains( real x, real y, real z, real radius, real dx ) const
{
   pe_USER_ASSERT( isValid(), "Invalid bounding sphere" );
   pe_USER_ASSERT( radius > real(0), "Invalid bounding sphere radius" );

   if( sq(center_[0]-x) + sq(center_[1]-y) + sq(center_[2]-z) + radius > radius_ + dx )
      return false;
   else
      return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Intersection test for axis-aligned bounding boxes.
 *
 * \param bs The second bounding sphere.
 * \param dx Spacing between the two bounding spheres.
 * \return \a true if the bounding sphere contains the other bounding sphere, \a false if not.
 *
 * In this intersection test, the radius of the first bounding sphere is increased by \a dx.
 * The function tests, if the increased bounding sphere contains the second bounding sphere.
 *
 * \b Note: In debug mode the validity of both bounding spheres is asserted by pe_USER_ASSERTs.
 */
inline bool BoundingSphere::contains( const BoundingSphere& bs, real dx ) const
{
   pe_USER_ASSERT( isValid()   , "Invalid bounding sphere" );
   pe_USER_ASSERT( bs.isValid(), "Invalid bounding sphere" );

   if( ( center_ - bs.center_ ).sqrLength() + bs.radius_ > radius_ + dx )
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
/*!\name Bounding sphere operators */
//@{
inline std::ostream& operator<<( std::ostream& os, const BoundingSphere& bs );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for bounding spheres.
 * \ingroup coarse_collision_detection
 *
 * \param os Reference to the output stream.
 * \param bs Reference to a bounding sphere.
 * \return Reference to the output stream.
 */
inline std::ostream& operator<<( std::ostream& os, const BoundingSphere& bs )
{
   return os << bs.getCenter() << ", " << bs.getRadius();
}
//*************************************************************************************************

} // namespace coarse

} // namespace detection

} // namespace pe

#endif
