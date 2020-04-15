//=================================================================================================
/*!
 *  \file src/core/rigidbody/CapsuleBase.cpp
 *  \brief Base class for the capsule geometry
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
#include <pe/core/rigidbody/CapsuleBase.h>
#include <pe/core/Materials.h>
#include <pe/core/MPI.h>
#include <pe/core/Thresholds.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/RotationMatrix.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the CapsuleBase class.
 *
 * \param sid Unique system-specific ID for the capsule.
 * \param uid User-specific ID for the capsule.
 * \param gpos Global geometric center of the capsule.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part \f$ (0..\infty) \f$.
 * \param material The material of the capsule.
 * \param visible Specifies if the capsule is visible in a visualization.
 *
 * The capsule is created lying along the x-axis.
 */
CapsuleBase::CapsuleBase( id_t sid, id_t uid, const Vec3& gpos, real radius,
                          real length, MaterialID material, bool visible )
   : GeomPrimitive( capsuleType, true, visible, sid, uid, material )  // Initializing the base object
   , radius_(radius)                                                  // Radius of the capsule
   , length_(length)                                                  // Length of the capsule
{
   // Checking the radius and the length
   // Since the capsule constructor is never directly called but only used in a small number
   // of functions that already check the capsule arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( radius  > real(0), "Invalid capsule radius"  );
   pe_INTERNAL_ASSERT( length  > real(0), "Invalid capsule length"  );

   // Setting the center of the capsule
   gpos_ = gpos;

   // Calculating the capsule mass
   mass_ = radius*radius * M_PI * ( real(4)/real(3) * radius + length ) * Material::getDensity( material );
   invMass_ = real(1) / mass_;

   // Calculating the moment of inertia
   calcInertia();

   // Setting the axis-aligned bounding box
   CapsuleBase::calcBoundingBox();
}
//*************************************************************************************************




//*************************************************************************************************
/*!\brief Destructor for the CapsuleBase class.
 */
CapsuleBase::~CapsuleBase()
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the bounding box of the capsule.
 *
 * \return void
 *
 * This function updates the axis-aligned bounding box of the capsule primitive according to the
 * current position and orientation of the capsule. Note that the bounding box is increased in
 * all dimensions by pe::contactThreshold to guarantee that rigid bodies in close proximity of
 * the capsule are also considered during the collision detection process.
 */
void CapsuleBase::calcBoundingBox()
{
   const real xlength( std::fabs( R_[0]*length_ )*real(0.5) + radius_ + contactThreshold );
   const real ylength( std::fabs( R_[3]*length_ )*real(0.5) + radius_ + contactThreshold );
   const real zlength( std::fabs( R_[6]*length_ )*real(0.5) + radius_ + contactThreshold );
   aabb_[0] = gpos_[0] - xlength;
   aabb_[1] = gpos_[1] - ylength;
   aabb_[2] = gpos_[2] - zlength;
   aabb_[3] = gpos_[0] + xlength;
   aabb_[4] = gpos_[1] + ylength;
   aabb_[5] = gpos_[2] + zlength;

   pe_INTERNAL_ASSERT( aabb_.isValid()        , "Invalid bounding box detected" );
   pe_INTERNAL_ASSERT( aabb_.contains( gpos_ ), "Invalid bounding box detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the moment of inertia in reference to the body frame of the capsule.
 *
 * \return void
 */
void CapsuleBase::calcInertia()
{
   const real density( calcDensity( radius_, length_, mass_ ) );
   const real sphereMass( real(4)/real(3) * M_PI * radius_*radius_*radius_ * density );
   const real cylinderMass( M_PI * radius_*radius_ * length_ * density );

   // 'Ia' represent the moment of inertia along the x-axis. 'Ia' contains the following two parts:
   //  - cylinder :  I = (1/2)*mass*radius^2
   //  - sphere   :  I = (2/5)*mass*radius^2
   const real Ia( radius_*radius_ * ( real(0.5)*cylinderMass + real(0.4)*sphereMass ) );

   // 'Ib' represent the moment of inertia along the y- and z-axis. 'Ib' contains the following two parts,
   // where full_length is the full length of the cylinder part and half_length is (1/2)*full_length:
   //  - cylinder :  I = mass*( (1/4)*radius^2 + (1/12)*full_length^2 )
   //  - sphere   :  I = mass*( (2/5)*radius^2 + half_length^2 + (3/4)*half_length*radius )
   const real Ib( cylinderMass*( real(0.25)*radius_*radius_ + real(1)/real(12)*length_*length_ ) +
                  sphereMass*( real(0.4)*radius_*radius_ + real(0.375)*radius_*length_ + real(0.25)*length_*length_ ) );

   // Setting the moment of inertia (capsule is aligned along the x-axis)
   I_[0] = Ia;
   I_[4] = Ib;
   I_[8] = Ib;
   Iinv_ = I_.getInverse();
}
//*************************************************************************************************

} // namespace pe
