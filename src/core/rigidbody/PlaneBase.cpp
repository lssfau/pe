//=================================================================================================
/*!
 *  \file src/core/rigidbody/PlaneBase.cpp
 *  \brief Base class for the plane geometry
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/GlobalSection.h>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/PlaneBase.h>
#include <pe/core/Thresholds.h>
#include <pe/math/Accuracy.h>
#include <pe/math/Infinity.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Quaternion.h>
#include <pe/math/RotationMatrix.h>
#include <pe/math/shims/Equal.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the PlaneBase class.
 *
 * \param sid Unique system-specific ID for the plane.
 * \param uid User-specific ID for the plane.
 * \param gpos The global position (anchor point) of the plane.
 * \param normal The plane's normal in reference to the global world frame, \f$ |n| = 1 \f$.
 * \param d The displacement of the plane.
 * \param material The material of the plane.
 * \param visible Specifies if the plane is visible in a visualization.
 *
 * The plane equation is: \f$ ax + by + cz = d \f$.\n
 * \a a, \a b and \a c are the x, y and z coordinate of the normal vector and \a d is the distance
 * from the origin to the plane.
 */
PlaneBase::PlaneBase( id_t sid, id_t uid, const Vec3& gpos, const Vec3& normal, real d,
                      MaterialID material, bool visible )
   : GeomPrimitive( planeType, false, visible, sid, uid, material )  // Initializing the base object
   , normal_( normal )                                               // Normal of the plane
   , d_( d )                                                         // Plane displacement from the origin
{
   // Checking the mass properties of the plane
   pe_INTERNAL_ASSERT( mass_    == real(0), "Mass of plane is not 0" );
   pe_INTERNAL_ASSERT( invMass_ == real(0), "Inverse mass of plane is not 0" );
   pe_INTERNAL_ASSERT( I_       == Mat3(0), "Moment of inertia of plane is not 0" );
   pe_INTERNAL_ASSERT( Iinv_    == Mat3(0), "Inverse moment of inertia of plane is not 0" );

   // Checking the plane normal
   // Since the plane constructor is never directly called but only used in a small number
   // of functions that already check the plane arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( equal( normal.sqrLength(), real(1) ), "Invalid plane normal" );

   // Checking if the plane is created inside a global section
   if( GlobalSection::isActive() )
      global_ = true;

   // Setting the global position (anchor point) of the plane
   gpos_ = gpos;

   // Setting the axis-aligned bounding box
   PlaneBase::calcBoundingBox();
}
//*************************************************************************************************




//*************************************************************************************************
/*!\brief Destructor for the PlaneBase class.
 */
PlaneBase::~PlaneBase()
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the bounding box of the plane.
 *
 * \return void
 *
 * This function updates the axis-aligned bounding box of the plane primitive according to the
 * current position and orientation of the plane. Note that the bounding box is increased in
 * all dimensions by pe::contactThreshold to guarantee that rigid bodies in close proximity of
 * the plane are also considered during the collision detection process.
 */
void PlaneBase::calcBoundingBox()
{
   aabb_[0] = ( normal_[0] <  real(0) && normal_[1] == real(0) && normal_[2] == real(0) )
                 ? ( -d_ - contactThreshold ) : ( -inf );
   aabb_[1] = ( normal_[0] == real(0) && normal_[1] <  real(0) && normal_[2] == real(0) )
                 ? ( -d_ - contactThreshold ) : ( -inf );
   aabb_[2] = ( normal_[0] == real(0) && normal_[1] == real(0) && normal_[2] <  real(0) )
                 ? ( -d_ - contactThreshold ) : ( -inf );
   aabb_[3] = ( normal_[0] >  real(0) && normal_[1] == real(0) && normal_[2] == real(0) )
                 ? (  d_ + contactThreshold ) : (  inf );
   aabb_[4] = ( normal_[0] == real(0) && normal_[1] >  real(0) && normal_[2] == real(0) )
                 ? (  d_ + contactThreshold ) : (  inf );
   aabb_[5] = ( normal_[0] == real(0) && normal_[1] == real(0) && normal_[2] >  real(0) )
                 ? (  d_ + contactThreshold ) : (  inf );

   pe_INTERNAL_ASSERT( aabb_.isValid(), "Invalid bounding box detected" );
}
//*************************************************************************************************

} // namespace pe
