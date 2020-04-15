//=================================================================================================
/*!
 *  \file src/core/Link.cpp
 *  \brief Source file for the link class
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <iostream>
#include <set>
#include <stdexcept>
#include <pe/core/contact/ContactVector.h>
#include <pe/core/detection/fine/MaxContacts.h>
#include <pe/core/Link.h>
#include <pe/core/LinkContact.h>
#include <pe/core/MPI.h>
#include <pe/core/Overlap.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/math/MatrixMxN.h>
#include <pe/system/Precision.h>
#include <pe/system/VerboseMode.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/Null.h>
#include <pe/util/policies/PtrDelete.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Link class.
 *
 * \param id User-specific ID for the sphere.
 * \param sb The superordinate union.
 * \param b1 The first directly attached rigid body.
 * \param b2 The second directly attached rigid body.
 *
 * Constructing the link \a id within the Union \a sb between the two rigid bodies \a b1 and
 * \a b2. The given global link normal is transfered to the body frame of the union.
 */
Link::Link( id_t id, UnionID sb, BodyID b1, BodyID b2 )
   : valid_(false)  // Validity flag
   , id_(id)        // User-specific link ID
   , sb_(sb)        // Superordinate union containing this link
   , b1_(b1)        // The first directly attached rigid body
   , b2_(b2)        // The second directly attached rigid body
   , sec1_(sb)      // The first attached section
   , sec2_(sb)      // The second attached section
   , gPos_()        // Global position
   , rPos_()        // Relative position
   , normal_()      // Relative body frame normal of the link
   , force1_()      // Total force acting on section 1
   , torque1_()     // Total torque acting on section 1
   , nForce1_()     // Total normal force acting on section 1
   , tForce1_()     // Total tangential force acting on section 1
   , force2_()      // Total force acting on section 2
   , torque2_()     // Total torque acting on section 2
   , nForce2_()     // Total normal force acting on section 2
   , tForce2_()     // Total tangential force acting on section 2
{}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the normal of the link in reference to the global world frame.
 *
 * \return The global normal of the link.
 */
const Vec3 Link::getNormal() const
{
   return sb_->getRotation() * normal_;
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of the attached sections.
 *
 * \param bodies The contained rigid bodies of the superordinate union.
 *
 * This functions tries to set up the two attached sections of the link. Whenever a structural
 * change in the superordinate union is done, this function is triggered to adapt the link to
 * the new union structure. In order to be able to calculate contact forces and torques, the
 * structure of the superordinate union must be separable in two distinct bodies, which are
 * only touching each other at the position of the link. If no clean separation is possible,
 * the link is set invalid and no contact forces and torques are calculated.
 */
void Link::setupLink( Bodies& bodies )
{
   // Resetting the link
   sec1_.clearSection();
   sec2_.clearSection();
   gPos_   = real(0);
   rPos_   = real(0);
   normal_ = real(0);
   valid_  = false;

   // Removing the link force and torque
   force1_ = torque1_ = nForce1_ = tForce1_ = real(0);
   force2_ = torque2_ = nForce2_ = tForce2_ = real(0);

   // Checking the number of bodies in the union and if the union is finite or infinite
   if( bodies.size() == 0 || !sb_->isFinite() ) {
      return;
   }

   // Searching for the two directly attached bodies and determining their indices
   const Bodies::SizeType numBodies( bodies.size() );
   Bodies::SizeType index1(0), index2(0);

   for( ; index1<numBodies; ++index1 ) {
      if( bodies[index1] == b1_ ) break;
   }
   if( index1 == numBodies ) return;

   for( ; index2<numBodies; ++index2 ) {
      if( bodies[index2] == b2_ ) break;
   }
   if( index2 == numBodies ) return;

   // Local variables
   Bodies::SizeType index;
   MatrixMxN<bool> M( numBodies, numBodies, false );
   Bodies section1, section2;
   std::set<Bodies::SizeType> toCheck;
   std::set<Bodies::SizeType>::iterator it;

   // Creating a link matrix
   for( Bodies::SizeType i=0; i<numBodies; ++i ) {
      for( Bodies::SizeType j=i+1; j<numBodies; ++j ) {
         if( overlap( bodies[i], bodies[j] ) ) {
            M(i,j) = M(j,i) = true;
         }
      }
   }

   // Checking for an overlap between body 1 and body 2
   if( M(index1,index2) == false ) return;

   // Removing the link between body 1 and body 2
   M(index1,index2) = M(index2,index1) = false;

   // Determining section 1
   section1.pushBack( b1_ );
   toCheck.insert( index1 );

   while( !toCheck.empty() ) {
      it = toCheck.begin();
      index = *it;
      toCheck.erase( it );
      for( Bodies::SizeType j=0; j<numBodies; ++j ) {
         if( M(index,j) && std::find( section1.begin(), section1.end(), bodies[j] ) == section1.end() ) {
            section1.pushBack( bodies[j] );
            toCheck.insert( j );
         }
      }
   }

   // Determining section 2
   section2.pushBack( b2_ );
   toCheck.insert( index2 );

   while( !toCheck.empty() ) {
      it = toCheck.begin();
      index = *it;
      toCheck.erase( it );
      for( Bodies::SizeType j=0; j<numBodies; ++j ) {
         if( M(index,j) && std::find( section2.begin(), section2.end(), bodies[j] ) == section2.end() ) {
            section2.pushBack( bodies[j] );
            toCheck.insert( j );
         }
      }
   }

   // Checking the structure of the union: if the number of bodies in 'section1' and 'section2'
   // is smaller than the total number of bodies in the superordinate union, then the structure
   // is disjoint. If the number of bodies in 'section1' and 'section2' is larger than the total
   // number of bodies, then the link is placed in a cyclic structure.
   if( section1.size() + section2.size() != numBodies ) return;

   // Setting the two attached sections
   sec1_.setSection( section1 );
   sec2_.setSection( section2 );

   // Calculating contact points between body 1 and body 2
   // Since an overlap between the two objects has already been detected, no further coarse
   // collision detection has to be performed. Additionally, the number of contacts will always
   // be larger than zero and it is therefore safe to divide by the number of contacts.
   typedef ContactVector<LinkContact,PtrDelete>  Contacts;
   Contacts contacts;
   detection::fine::MaxContacts::collide( b1_, b2_, contacts );

   // Calculating the global position of the link
   for( Contacts::ConstIterator c=contacts.begin(); c!=contacts.end(); ++c )
      gPos_ += c->getPosition();
   gPos_ /= contacts.size();

   // Calculating the relative position within the superordinate body
   rPos_ = trans( sb_->getRotation() )*( gPos_ - sb_->getPosition() );

   // Calculating the global normal of the link
   for( Contacts::ConstIterator c=contacts.begin(); c!=contacts.end(); ++c )
      normal_ += c->getNormal();
   normal_ /= contacts.size();

   if( trans(normal_) * ( b2_->getPosition() - b1_->getPosition() ) < real(0) )
      normal_ *= real(-1);

   // Making the link valid
   valid_ = true;
}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation update of the link and the attached sections.
 *
 * \param dp Change in the global position of the superordinate union.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a translational
 * movement. In case of a finite superordinate body, this movement only changes the global
 * position. In case of an infinite superordinate body, the fixed point of reference (the
 * origin of the global world frame) remains unchanged. Therefore the relative position is
 * changed additionally.
 */
void Link::update( const Vec3& dp )
{
   if( valid_ )
   {
      pe_INTERNAL_ASSERT( sb_->isFinite(), "Infinite superordinate union detected" );

      // Updating the global position
      gPos_ += dp;

      // Updating the two attached sections
      sec1_.update( dp );
      sec2_.update( dp );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation update of the link and the attached sections.
 *
 * \param dq Change in the orientation of the superordinate union.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a rotational
 * movement. This movement may cause a change in the global position of the link.
 */
void Link::update( const Quat& dq )
{
   if( valid_ )
   {
      pe_INTERNAL_ASSERT( sb_->isFinite(), "Infinite superordinate union detected" );

      // Updating the global position
      gPos_ = sb_->getPosition() + ( sb_->getRotation() * rPos_ );

      // Updating the two attached sections
      sec1_.update( dq );
      sec2_.update( dq );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the link forces and torques on both attached sections.
 *
 * \param vdot The linear acceleration of the superordinate union.
 * \param wdot The angular acceleration of the superordinate union.
 *
 * The calculation of the link forces for both sections is performed using the following
 * equation:

                       \f[ F_{link}(t) = M_s \dot{v}(t) - F_s(t), \f]

 * where \f$ M_s \f$ is the total section mass, \f$ \dot{v} \f$ is the section's linear
 * acceleration and \f$ F_s \f$ is the force acting on the section. Additionally, the link
 * force is split into the normal and tangential force component.\n
 * The calculation of the link torques is done using the following equation:

           \f[
               T_{link}(t) = I_s(t) \cdot \dot{\omega}(t) +
                             \omega(t) \times ( I_s(t) \cdot \omega(t) ) -
                             d_{link} \times F_{link}(t) -
                             T_{link}(t),
           \f]

 * where \f$ I_s(t) \f$ is the section's moment of inertia in reference to the global world frame,
 * \f$ \dot{\omega} \f$ is the section's angular acceleration, \f$ \omega \f$ is the section's
 * angular velocity, \f$ d_{link} = x_s(t) - x_{link}(t) \f$ is the distance from the section's
 * center of mass to the global position of the link and \f$ T_{link} \f$ is the torque acting
 * on the section.\n
 * The link forces and torques can only be calculated if the superordinate union can be separated
 * in two distinct sections. Otherwise, the link is invalidated and no force computations are
 * performed.
 */
void Link::calcForce( const Vec3& vdot, const Vec3& wdot )
{
   // Checking the validity of the link
   if( !valid_ ) return;

   // Local variables
   real tmp;
   Vec3 dist, acc, cTorque;
   Mat3 I;
   const Vec3& w( sb_->getAngularVel() );

   // Calculating the normal of the link
   const Vec3 normal( getNormal() );


   ///////////////////////////////////////////////////
   // Calculating the force and torque on section 1

   // Distance between section 1 and the union's center of mass
   dist = sec1_.getPosition() - sb_->getPosition();

   // Calculating the linear acceleration of section 1
   acc = vdot + wdot % dist + w % ( w % dist );

   // Contact force at joint
   force1_ = sec1_.getMass() * acc - sec1_.getForce();

   // Calculation of the normal and tangential force of section 1
   tmp = trans(force1_) * normal;
   nForce1_ = normal * tmp;
   tForce1_ = force1_ - nForce1_;

   // Torque caused by contact force
   cTorque = ( gPos_ - sec1_.getPosition() ) % force1_;

   // Calculating the moment of inertia of section 1
   I = sec1_.getInertia();

   // Overall torque at joint
   torque1_ = I * wdot + w % ( I * w ) - cTorque - sec1_.getTorque();


   ///////////////////////////////////////////////////
   // Calculating the force and torque on section 2

   // Distance between section 2 and the particle's center of mass
   dist = sec2_.getPosition() - sb_->getPosition();

   //Calculating the linear acceleration of section 2
   acc = vdot + wdot % dist + w % ( w % dist );

   // Contact force at joint
   force2_ = sec2_.getMass() * acc - sec2_.getForce();

   // Calculation of the normal and tangential force of section 2
   tmp = trans(force2_) * normal;
   nForce2_ = normal * tmp;
   tForce2_ = force2_ - nForce2_;

   // Torque caused by contact force
   cTorque = ( gPos_ - sec2_.getPosition() ) % force2_;

   // Calculating the moment of inertia of section 2
   I = sec2_.getInertia();

   // Overall torque at joint
   torque2_ = I * wdot + w % ( I * w ) - cTorque - sec2_.getTorque();


   ////////////////////////////////////////
   // Controlling the forces and torques

   pe_INTERNAL_ASSERT( force1_  == -force2_ , "Link forces are not contrary to each other"  );
   pe_INTERNAL_ASSERT( torque1_ == -torque2_, "Link torques are not contrary to each other" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a link.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the link output.
 * \return void
 */
void Link::print( std::ostream& os, const char* tab ) const
{
   if( valid_ ) {
      os << tab << " Link " << id_ << " connecting body " << b1_->getID() << " and " << b2_->getID() << "\n";

      if( verboseMode )
      {
         os << tab << "   Global Position   = " << getPosition() << "\n"
            << tab << "   Relative Position = " << rPos_ << "\n"
            << tab << "   Global Normal     = " << getNormal() << "\n"
            << tab << "   Relative Normal   = " << normal_ << "\n";
      }
   }
   else {
      os << tab << " Invalidated link " << id_ << "\n";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  LINK SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new link within a union.
 *
 * \param u The union in which the new link is created.
 * \param id The user-specific ID of the link.
 * \param b1 The first directly attached rigid body.
 * \param b2 The second directly attached rigid body.
 * \return Handle for the new link.
 * \exception std::logic_error Cannot create link in MPI parallel simulation.
 * \exception std::invalid_argument Invalid link parameters.
 *
 * Creating a new link between the two rigid bodie \a b1 and \a b2 within the union \a u.
 * Both bodies have to touch each other or the setup of the link fails, which will result
 * in a \a std::invalid_argument exception. Additionally it will be checked if previously
 * an identical link has been defined, which will also result in a \a std::invalid_argument
 * exception. If the structure of the union can be uniquely separated into two distinct,
 * non-disjoint and non-cyclic sections (where the first body will be in section 1 and the
 * second body in section 2), then in each time step the forces and torques in the link
 * will be calculated.
 *
 * \b Note: Currently it is not possible to create links in MPI parallel simulations (i.e.,
 * in case the simulation uses more than a single process). The attempt to create a link in
 * a parallel simulation results results in a \a std::logic_error exception.
 */
LinkID createLink( UnionID u, id_t id, BodyID b1, BodyID b2 )
{
   // Checking the number of active MPI processes
   if( MPISettings::size() > 1 )
      throw std::logic_error( "Cannot create link in MPI parallel simulation" );

   // Checking the union and body IDs
   if( u == NULL || b1 == NULL || b2 == NULL )
      throw std::invalid_argument( "Invalid rigid body ID!" );

   return u->createLink( id, b1, b2 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Destroys the given link.
 *
 * \param link The link to be destroyed.
 *
 * \b Note: Destroying the link invalidates all remaining LinkID referencing the link!
 */
void destroy( LinkID link )
{
   link->getUnion()->destroyLink( link );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for spheres.
 * \ingroup link
 *
 * \param os Reference to the output stream.
 * \param l Reference to a constant link object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Link& l )
{
   os << "--" << pe_BROWN << "LINK PARAMETERS" << pe_OLDCOLOR
      << "---------------------------------------------------------------\n";
   l.print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for link handles.
 * \ingroup link
 *
 * \param os Reference to the output stream.
 * \param l Constant link handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstLinkID l )
{
   os << "--" << pe_BROWN << "LINK PARAMETERS" << pe_OLDCOLOR
      << "---------------------------------------------------------------\n";
   l->print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
