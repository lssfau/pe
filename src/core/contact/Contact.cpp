//*************************************************************************************************
/*!
 *  \file src/core/contact/Contact.cpp
 *  \brief Source file for the Contact class
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
//*************************************************************************************************


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/contact/Contact.h>
#include <pe/core/Materials.h>
#include <pe/core/MPI.h>
#include <pe/math/Accuracy.h>
#include <pe/util/Assert.h>
#include <pe/util/logging/DebugSection.h>


namespace pe {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

Contact::ContactPool Contact::contactPool_;




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Contact constructor for a vertex/face contact.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 The second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \param dist The distance between the surfaces of the contacting rigid bodies.
 */
Contact::Contact( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal, real dist )
   : Parent( g1, g2, gpos, normal, dist )  // Initialization of the parent class
{
   pe_INTERNAL_ASSERT( ( real(1) - normal.sqrLength() ) < accuracy, "Invalid contact normal" );
   pe_INTERNAL_ASSERT( b1_->getSuperBody() == b1_, "Invalid superordinate rigid body" );
   pe_INTERNAL_ASSERT( b2_->getSuperBody() == b2_, "Invalid superordinate rigid body" );
   pe_INTERNAL_ASSERT( !( b1_->isFixed() && b2_->isFixed() ), "Invalid contact between two fixed rigid bodies" );

   // Calculating the relative velocity
   const Vec3 rvel( b1_->velFromWF( gpos_ ) - b2_->velFromWF( gpos_ ) );  // Relative velocity
   const real nvel( trans(normal_) * rvel );                              // Normal relative velocity
   const real tvel( trans(normal_) * ( rvel - normal_ * nvel ) );         // Tangential relative velocity

   // Calculating the coefficient of restitution between the two colliding (sub-)bodies
   // In case the normal relative velocity between the two colliding rigid bodies is smaller than the
   // restitution threshold, a coefficient of restitution of 0 is used to prevent an infinite number
   // of collisions during a single time step.
   if( std::fabs( nvel ) > restitutionThreshold ) {
      restitution_ = Material::getRestitution( g1->getMaterial(), g2->getMaterial() );
   }

   // Calculate the stiffness and damping parameters
   stiffness_ = Material::getStiffness( g1->getMaterial(), g2->getMaterial() );
   dampingN_  = Material::getDampingN ( g1->getMaterial(), g2->getMaterial() );
   dampingT_  = Material::getDampingT ( g1->getMaterial(), g2->getMaterial() );

   // Calculating the coefficient of friction
   // In case the tangential relative velocity between the two colliding rigid bodies is smaller than
   // the friction threshold, the coefficient of static friction is used. Otherwise, the coefficient
   // of dynamic friction is used.
   if( std::fabs( tvel ) > frictionThreshold )
      friction_ = Material::getDynamicFriction( g1->getMaterial(), g2->getMaterial() );
   else
      friction_ = Material::getStaticFriction( g1->getMaterial(), g2->getMaterial() );

   // Registering the contact with both attached rigid bodies
   if( !b1_->isFixed() ) b1_->registerContact( this );
   if( !b2_->isFixed() ) b2_->registerContact( this );

   // Merging the contact graph
   if( !b1_->isFixed() && !b2_->isFixed() )
      mergeNodes( b1_, b2_ );

   // Debugging output
   pe_LOG_DEBUG_SECTION( log ) {
      log << "         => contact-id = " << id_;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact constructor for an edge/edge contact.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 The second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \param e1 Edge direction of the colliding edge of the first colliding rigid body.
 * \param e2 Edge direction of the colliding edge of the second colliding rigid body.
 * \param dist The distance between the surfaces of the contacting rigid bodies.
 */
Contact::Contact( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal,
                  const Vec3& e1, const Vec3& e2, real dist )
   : Parent ( g1, g2, gpos, normal, e1, e2, dist )  // Initialization of the parent class
{
   pe_INTERNAL_ASSERT( ( real(1) - normal.sqrLength() ) < real(1E-8), "Invalid contact normal" );
   pe_INTERNAL_ASSERT( e1.sqrLength() > real(0), "Invalid colliding edge" );
   pe_INTERNAL_ASSERT( e2.sqrLength() > real(0), "Invalid colliding edge" );
   pe_INTERNAL_ASSERT( b1_->getSuperBody() == b1_, "Invalid superordinate rigid body" );
   pe_INTERNAL_ASSERT( b2_->getSuperBody() == b2_, "Invalid superordinate rigid body" );
   pe_INTERNAL_ASSERT( !( b1_->isFixed() && b2_->isFixed() ), "Invalid contact between two fixed rigid bodies" );

   // Calculating the relative velocity
   const Vec3 rvel( b1_->velFromWF( gpos_ ) - b2_->velFromWF( gpos_ ) );  // Relative velocity
   const real nvel( trans(normal_) * rvel );                              // Normal relative velocity
   const real tvel( trans(normal_) * ( rvel - normal_ * nvel ) );         // Tangential relative velocity

   // Calculating the coefficient of restitution between the two colliding (sub-)bodies
   // In case the normal relative velocity between the two colliding rigid bodies is smaller than the
   // restitution threshold, a coefficient of restitution of 0 is used to prevent an infinite number
   // of collisions during a single time step.
   if( std::fabs( nvel ) > restitutionThreshold ) {
      restitution_ = Material::getRestitution( g1->getMaterial(), g2->getMaterial() );
   }

   // Calculating the coefficient of friction
   // In case the tangential relative velocity between the two colliding rigid bodies is smaller than
   // the friction threshold, the coefficient of static friction is used. Otherwise, the coefficient
   // of dynamic friction is used.
   if( std::fabs( tvel ) > frictionThreshold )
      friction_ = Material::getDynamicFriction( g1->getMaterial(), g2->getMaterial() );
   else
      friction_ = Material::getStaticFriction( g1->getMaterial(), g2->getMaterial() );

   // Registering the contact with both attached rigid bodies
   if( !b1_->isFixed() ) b1_->registerContact( this );
   if( !b2_->isFixed() ) b2_->registerContact( this );

   // Merging the contact graph
   if( !b1_->isFixed() && !b2_->isFixed() )
      mergeNodes( b1_, b2_ );

   // Debugging output
   pe_LOG_DEBUG_SECTION( log ) {
      log << "         => contact-id = " << id_;
   }
}
//*************************************************************************************************

} // namespace pe
