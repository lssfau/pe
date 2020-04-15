//=================================================================================================
/*!
 *  \file src/core/rigidbody/BoxBase.cpp
 *  \brief Base class for the box geometry
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
#include <pe/core/rigidbody/BoxBase.h>
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
/*!\brief Constructor for the BoxBase class.
 *
 * \param sid Unique system-specific ID for the box.
 * \param uid User-specific ID for the box.
 * \param gpos Global geometric center of the box.
 * \param lengths Side lengths of the box \f$ (0..\infty) \f$.
 * \param material The material of the box.
 * \param visible Specifies if the box is visible in a visualization.
 */
BoxBase::BoxBase( id_t sid, id_t uid, const Vec3& gpos,
                  const Vec3& lengths, MaterialID material, bool visible )
   : GeomPrimitive( boxType, true, visible, sid, uid, material )  // Initializing the base object
   , lengths_( lengths )                                          // Side lengths of the box
{
   // Checking the side lengths
   // Since the box constructor is never directly called but only used in a small number
   // of functions that already check the box arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( lengths[0] > real(0), "Invalid side length in x-dimension" );
   pe_INTERNAL_ASSERT( lengths[1] > real(0), "Invalid side length in y-dimension" );
   pe_INTERNAL_ASSERT( lengths[2] > real(0), "Invalid side length in z-dimension" );

   // Setting the center of the box
   gpos_ = gpos;

   // Calculating the box mass
   mass_ = calcMass( lengths, Material::getDensity( material ) );
   invMass_ = real(1) / mass_;

   // Calculating the moment of inertia
   calcInertia();

   // Setting the axis-aligned bounding box
   BoxBase::calcBoundingBox();
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the BoxBase class.
 */
BoxBase::~BoxBase()
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the bounding box of the box.
 *
 * \return void
 *
 * This function updates the axis-aligned bounding box of the box primitive according to the
 * current position and orientation of the box. Note that the bounding box is increased in
 * all dimensions by pe::contactThreshold to guarantee that rigid bodies in close proximity
 * of the box are also considered during the collision detection process.
 */
void BoxBase::calcBoundingBox()
{
   using std::fabs;

   const real xlength( real(0.5) * ( fabs(R_[0]*lengths_[0]) + fabs(R_[1]*lengths_[1]) + fabs(R_[2]*lengths_[2]) ) + contactThreshold );
   const real ylength( real(0.5) * ( fabs(R_[3]*lengths_[0]) + fabs(R_[4]*lengths_[1]) + fabs(R_[5]*lengths_[2]) ) + contactThreshold );
   const real zlength( real(0.5) * ( fabs(R_[6]*lengths_[0]) + fabs(R_[7]*lengths_[1]) + fabs(R_[8]*lengths_[2]) ) + contactThreshold );
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
/*!\brief Calculation of the moment of inertia in reference to the body frame of the box.
 *
 * \return void
 */
void BoxBase::calcInertia()
{
   I_[0] = mass_/static_cast<real>( 12 ) * ( lengths_[1]*lengths_[1] + lengths_[2]*lengths_[2] );
   I_[4] = mass_/static_cast<real>( 12 ) * ( lengths_[0]*lengths_[0] + lengths_[2]*lengths_[2] );
   I_[8] = mass_/static_cast<real>( 12 ) * ( lengths_[0]*lengths_[0] + lengths_[1]*lengths_[1] );
   Iinv_ = I_.getInverse();
}
//*************************************************************************************************

} // namespace pe
