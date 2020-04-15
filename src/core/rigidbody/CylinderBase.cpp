//=================================================================================================
/*!
 *  \file src/core/rigidbody/CylinderBase.cpp
 *  \brief Base class for the cylinder geometry
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
#include <pe/core/rigidbody/CylinderBase.h>
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
/*!\brief Constructor for the CylinderBase class.
 *
 * \param sid Unique system-specific ID for the cylinder.
 * \param uid User-specific ID for the cylinder.
 * \param gpos Global geometric center of the cylinder.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part \f$ (0..\infty) \f$.
 * \param material The material of the cylinder.
 * \param visible Specifies if the cylinder is visible in a visualization.
 *
 * The cylinder is created lying along the x-axis.
 */
CylinderBase::CylinderBase( id_t sid, id_t uid, const Vec3& gpos, real radius,
                            real length, MaterialID material, bool visible )
   : GeomPrimitive( cylinderType, true, visible, sid, uid, material )  // Initializing the base object
   , radius_(radius)                                                   // Radius of the cylinder
   , length_(length)                                                   // Length of the cylinder
{
   // Checking the radius and the length
   // Since the cylinder constructor is never directly called but only used in a small number
   // of functions that already check the cylinder arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( radius  > real(0), "Invalid cylinder radius"  );
   pe_INTERNAL_ASSERT( length  > real(0), "Invalid cylinder length"  );

   // Setting the center of the cylinder
   gpos_ = gpos;

   // Calculating the cylinder mass
   mass_ = radius*radius * M_PI * length * Material::getDensity( material );
   invMass_ = real(1) / mass_;

   // Calculating the moment of inertia
   calcInertia();

   // Setting the axis-aligned bounding box
   CylinderBase::calcBoundingBox();
}
//*************************************************************************************************




//*************************************************************************************************
/*!\brief Destructor for the CylinderBase class.
 */
CylinderBase::~CylinderBase()
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the bounding box of the cylinder.
 *
 * \return void
 *
 * This function updates the axis-aligned bounding box of the cylinder primitive according to the
 * current position and orientation of the cylinder. Note that the bounding box is increased in
 * all dimensions by pe::contactThreshold to guarantee that rigid bodies in close proximity of
 * the cylinder are also considered during the collision detection process.
 */
void CylinderBase::calcBoundingBox()
{
   using std::fabs;

   // An exact bounding box for a cylinder can be calculated by projecting the center line on the
   // x-, y-, and z-axes and adding the projection of the two cap circles. Knowing the two cylinder
   // end points A and B and the radius r, the bounding box offset from the center of mass is
   // calculated by:
   //
   //   dx = (1/2)*fabs( R_[0] )*length_ + kx * radius;
   //   dy = (1/2)*fabs( R_[3] )*length_ + ky * radius;
   //   dz = (1/2)*fabs( R_[6] )*length_ + kz * radius;
   //
   // where
   //
   //   kx = sqrt( ( (A[1]-B[1])^2 + (A[2]-B[2])^2 ) / ( (A[0]-B[0])^2 + (A[1]-B[1])^2 + (A[2]-B[2])^2 ) )
   //   ky = sqrt( ( (A[0]-B[0])^2 + (A[2]-B[2])^2 ) / ( (A[0]-B[0])^2 + (A[1]-B[1])^2 + (A[2]-B[2])^2 ) )
   //   kz = sqrt( ( (A[0]-B[0])^2 + (A[1]-B[1])^2 ) / ( (A[0]-B[0])^2 + (A[1]-B[1])^2 + (A[2]-B[2])^2 ) )
   //
   // Alternatively, an exact bounding box can be calculated using the rotation matrix:
   //
   //  dx = (1/2)*fabs( R_[0] )*length_ + sqrt( R_[1]^2 + R_[2]^2 )*radius_
   //  dy = (1/2)*fabs( R_[3] )*length_ + sqrt( R_[4]^2 + R_[5]^2 )*radius_
   //  dz = (1/2)*fabs( R_[6] )*length_ + sqrt( R_[7]^2 + R_[8]^2 )*radius_
   //
   // However, in order to avoid the three costly square root calculations (in both formulations),
   // a loosely fitting bounding box is calculated instead by the three following equations:
   //
   //  dx = (1/2)*fabs( R_[0] )*length_ + fabs( R_[1] )*radius_ + fabs( R_[2] )*radius_
   //  dx = (1/2)*fabs( R_[3] )*length_ + fabs( R_[4] )*radius_ + fabs( R_[5] )*radius_
   //  dx = (1/2)*fabs( R_[6] )*length_ + fabs( R_[7] )*radius_ + fabs( R_[8] )*radius_

   const real xlength( real(0.5)*fabs(R_[0])*length_ + ( fabs(R_[1]) + fabs(R_[2]) )*radius_ + contactThreshold );
   const real ylength( real(0.5)*fabs(R_[3])*length_ + ( fabs(R_[4]) + fabs(R_[5]) )*radius_ + contactThreshold );
   const real zlength( real(0.5)*fabs(R_[6])*length_ + ( fabs(R_[7]) + fabs(R_[8]) )*radius_ + contactThreshold );
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
/*!\brief Calculation of the moment of inertia in reference to the body frame of the cylinder.
 *
 * \return void
 */
void CylinderBase::calcInertia()
{
   // 'Ia' represent the moment of inertia along the x-axis. For a solid cylinder it is
   // calculated by  I = (1/2)*mass*radius^2
   const real Ia( real(0.5)*mass_*radius_*radius_ );

   // 'Ib' represent the moment of inertia along the y- and z-axis. For a solid cylinder it is
   // calculated by  I = mass*( (1/4)*radius^2 + (1/12)*length^2 )
   const real Ib( mass_*( real(0.25)*radius_*radius_ + real(1)/real(12)*length_*length_ ) );

   // Setting the moment of inertia (cylinder is aligned along the x-axis)
   I_[0] = Ia;
   I_[4] = Ib;
   I_[8] = Ib;
   Iinv_ = I_.getInverse();
}
//*************************************************************************************************

} // namespace pe
