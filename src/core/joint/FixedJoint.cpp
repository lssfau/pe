//=================================================================================================
/*!
 *  \file src/core/joint/FixedJoint.cpp
 *  \brief Source file for the FixedJoint class
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

#include <stdexcept>
#include <pe/core/CollisionSystem.h>
#include <pe/config/VerboseMode.h>
#include <pe/core/joint/FixedJoint.h>
#include <pe/core/joint/JointType.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/core/response/typetraits/IsSelectedSolver.h>
#include <pe/core/TimeStep.h>
#include <pe/core/Visualization.h>
#include <pe/system/Collisions.h>
#include <pe/system/ConstraintConfig.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the FixedJoint class.
 *
 * \param sid The unique system-specific ID.
 * \param body1 The first body to which the joint is attached.
 * \param anchor1 The first body's anchor point in body relative coordinates.
 * \param body2 The second body to which the joint is attached.
 * \param anchor2 The second body's anchor point in body relative coordinates.
 * \param scale Scaling parameter for the visualization of the fixed joint \f$[0..1] \f$.
 */
FixedJoint::FixedJoint( id_t sid, BodyID body1, const Vec3& anchor1,
                        BodyID body2, const Vec3& anchor2, real scale )
   : Joint( fixedType, size_t(6), body1, body2, scale, sid )  // Initialization of the Joint base object
   , length_ ( 0 )                                       // The constant distance between the two rigid bodies
   , anchor1_( anchor1 )                                 // The first body's anchor point in body relative coordinates
   , anchor2_( anchor2 )                                 // The second body's anchor point in body relative coordinates
   , mid1_   ()                                          // The midpoint between the two bodies wrt. the first body
   , mid2_   ()                                          // The midpoint between the two bodies wrt. the second body
   , qini_   ()                                          // The initial relative orientation of the two bodies
{
   // Checking the anchor points and the scaling parameter of the fixed joint
   // Since the fixed joint constructor is never directly called but only used in a small number
   // of functions that already check the fixed joint arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( body1->containsRelPoint( anchor1 )  , "Invalid anchor point for body 1" );
   pe_INTERNAL_ASSERT( body2->containsRelPoint( anchor2 )  , "Invalid anchor point for body 2" );
   pe_INTERNAL_ASSERT( scale >= real(0) && scale <= real(1), "Invalid scaling parameter"       );

   // Calculating the length of the fixed joint
   length_ = ( body2->pointFromBFtoWF( anchor2 ) - body1->pointFromBFtoWF( anchor1 ) ).length();
   pe_INTERNAL_ASSERT( length_ >= real(0), "Invalid fixed joint length" );

   // Calculating the initial relative orientation of the two bodies
   qini_ = body2_->getQuaternion().getInverse() * body1_->getQuaternion();

   // Calculating the midpoints of the two bodies in body relative coordinates
   const Vec3 offset( real(0.5) * ( body2->getPosition() + body1->getPosition() ) );
   mid1_ = body1->pointFromWFtoBF( offset );
   mid2_ = body2->pointFromWFtoBF( offset );

   // Registering the fixed joint with both attached rigid bodies
   //body1->registerJoint( this );  TODO
   //body2->registerJoint( this );  TODO

   // Registering the fixed joint for visualization
   //Visualization::add( this );  TODO
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the FixedJoint class.
 */
FixedJoint::~FixedJoint()
{
   // Deregistering the fixed joint from the visualization
   // Visualization::remove( this );  TODO
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the error correction parameter.
 *
 * \return The error correction parameter.
 */
real FixedJoint::getCorParam() const
{
   return ( pe::errCorFixed / TimeStep::size() );
}
//*************************************************************************************************




//=================================================================================================
//
//  CALCULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculating the matrices for the fixed joint.
 *
 * \param JLin1 The linear Jacobian matrix for body 1.
 * \param JAng1 The angular Jacobian matrix for body 1.
 * \param JLin2 The linear Jacobian matrix for body 2.
 * \param JAng2 The angular Jacobian matrix for body 2.
 * \return void
 *
 * This function calculates the Jacobian matrices for the fixed joint.
 */
void FixedJoint::calcMat( MatN& JLin1, MatN& JAng1, MatN& JLin2, MatN& JAng2 ) const
{
   const real initLin1[6][3] = { { 1, 0, 0 },
                                 { 0, 1, 0 },
                                 { 0, 0, 1 },
                                 { 0, 0, 0 },
                                 { 0, 0, 0 },
                                 { 0, 0, 0 } };
   JLin1 = initLin1;

   const Vec3 temp( body2_->getPosition() - body1_->getPosition() );

   const real initAng1[6][3] = { {  0      ,  temp[2], -temp[1]},
                                 { -temp[2],  0      ,  temp[0] },
                                 {  temp[1], -temp[0],  0       },
                                 { 1, 0, 0 },
                                 { 0, 1, 0 },
                                 { 0, 0, 1 } };
   JAng1 = initAng1;

   const real initLin2[6][3] = { { -1,  0,  0 },
                                 {  0, -1,  0 },
                                 {  0,  0, -1 },
                                 {  0,  0,  0 },
                                 {  0,  0,  0 },
                                 {  0,  0,  0 } };
   JLin2 = initLin2;

   const real initAng2[6][3] = { {  0,  0,  0 },
                                 {  0,  0,  0 },
                                 {  0,  0,  0 },
                                 { -1,  0,  0 },
                                 {  0, -1,  0 },
                                 {  0,  0, -1 } };
   JAng2 = initAng2;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating the error vector for the fixed joint.
 *
 * \param bError The velocity error correcting vector.
 * \return void
 *
 * This function calculates the velocity error correcting term for the fixed joint.
 */
void FixedJoint::calcErrorVec( VecN& bError ) const
{
   // Computing the translational error between both bodies
   const Vec3 linErr( getCorParam() * ( getMid2WF() - getMid1WF() ) );

   // Computing the rotational error between both bodies
   const Quat q( getErrOrient() );
   const Vec3 rotErr( real(2) * ( pe::errCorFixed / TimeStep::size() ) * Vec3( q[1], q[2], q[3] ) );

   const real init[6] = { linErr[0], linErr[1], linErr[2], rotErr[0], rotErr[1], rotErr[2] };

   bError = init;
}
//*************************************************************************************************




//=================================================================================================
//
//  LIMIT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the lower joint limit is violated.
 *
 * \return \a true in case of a violated lower joint limit, \a false otherwise.
 *
 * This function returns whether the lower joint limit is violated. In case of the fixed
 * joint this function always returns \a false.
 */
inline bool FixedJoint::isViolatedLow() const
{
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the upper joint limit is violated.
 *
 *\return \a true in case of a violated upper joint limit, \a false otherwise.
 *
 * This function returns whether the upper joint limit is violated. In case of the fixed
 * joint this function always returns \a false.
 */
inline bool FixedJoint::isViolatedHigh() const
{
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating the limit matrices for the fixed joint.
 *
 * \param JLin1L The linear limit Jacobian matrix for body 1.
 * \param JAng1L The angular limit Jacobian matrix for body 1.
 * \param JLin2L The linear limit Jacobian matrix for body 2.
 * \param JAng2L The angular limit Jacobian matrix for body 2.
 * \return void
 *
 * This function calculates the limit matrices for the joint. In case of the fixed joint
 * this function does not perform any action.
 */
void FixedJoint::calcLimitMat( MatN& /*JLin1L*/, MatN& /*JAng1L*/, MatN& /*JLin2L*/, MatN& /*JAng2L*/ ) const
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating the error term for the lower limit of the fixed joint.
 *
 * \return The velocity error correcting term for the lower limit.
 *
 * This function calculates the velocity error correcting term for the lower limit of the
 * joint. In case of the fixed joint the function always returns 0.
 */
inline real FixedJoint::calcLimitErrorLo() const
{
   return real(0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating the error vector for the upper limit of the fixed joint.
 *
 * \return The velocity error correcting term for the upper limit.
 *
 * This function calculates the velocity error correcting term for the upper limit of the
 * joint. In case of the fixed joint the function always returns 0.
 */
inline real FixedJoint::calcLimitErrorHi() const
{
   return real(0);
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of the fixed joint.
 *
 * \param os Reference to the output stream.
 * \return void
 */
void FixedJoint::print( std::ostream& os ) const
{
   os << " Fixed joint between the rigid body " << body1_->getID() << " and " << body2_->getID() << "\n";

   if( verboseMode ) {
      os << "   1. anchor point: global             = " << getAnchor1WF() << "\n"
         << "                  : relative           = " << getAnchor1BF() << "\n"
         << "   2. anchor point: global             = " << getAnchor2WF() << "\n"
         << "                  : relative           = " << getAnchor2BF() << "\n"
         << "   1. midpoint    : global             = " << getMid1WF() << "\n"
         << "                  : relative           = " << getMid1BF() << "\n"
         << "   2. midpoint    : global             = " << getMid2WF() << "\n"
         << "                  : relative           = " << getMid2BF() << "\n"
         << "   Initial distance between the bodies = " << getIniLength() << "\n"
         << "   Current distance between the bodies = " << getCurLength() << "\n";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  FIXED JOINT SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new fixed joint between two rigid bodies.
 * \ingroup joint
 *
 * \param body1 The first body to which the joint is attached.
 * \param anchor1 The first body's anchor point in body relative coordinates.
 * \param body2 The second body to which the joint is attached.
 * \param anchor2 The second body's anchor point in body relative coordinates.
 * \param scale Visualization parameter for the scaling of the fixed joint \f$[0..1] \f$.
 * \return Handle for the new fixed joint.
 * \exception std::logic_error Cannot create fixed joint in MPI parallel simulation.
 * \exception std::logic_error Selected constraint solver is incapable of handling fixed joints.
 * \exception std::invalid_argument Invalid anchor point for body 1.
 * \exception std::invalid_argument Invalid anchor point for body 2.
 * \exception std::invalid_argument Invalid scaling parameter.
 *
 * This function creates a new fixed joint between the two rigid bodies \a body1 and \a body2,
 * which prevents any relative motion between the two bodies. \a anchor1 is the first anchor
 * point given in coordinates relative to the body frame of \a body1, \a anchor2 is the second
 * anchor point given in coordinates relative to the body frame of \a body2. The \a scale
 * parameter specifies the size of the fixed joint in all active visualization systems (the
 * default is 1).
 *
 * \image html fixedJoint.png
 * \image latex fixedJoint.eps "Fixed Joint" width=400pt
 *
 * The following code example illustrates the setup of a fixed joint between a sphere and
 * a box primitive:

   \code
   // Creating the iron sphere 1 with a radius of 2.5 at the global position (2,3,4).
   SphereID sphere = createSphere( 1, Vec3(2,3,4), 2.5, iron );

   // Creating the granite box 2 with side lengths (6,5,4) at the global position (10,3,4)
   BoxID box = createBox( 2, Vec3(10,3,4), Vec3(6,5,4), granite );

   // Creating a fixed joint between the sphere and the box primitive
   // The fixed joint's anchor point on the sphere is (2.5,0,0) in body relative coordinates
   // and its anchor point on the box is (-3,0,1) (again in body relative coordinates).
   // In all active visualizations, the scaling factor of 0.5 will be used to draw the
   // fixed joint.
   FixedJointID joint = attachFixedJoint( sphere, Vec3(2.5,0,0),
                                          box   , Vec3( -3,0,1), 0.5 );
   \endcode

 * \b Note: Currently it is not possible to use fixed joints in MPI parallel simulations (i.e.,
 * in case the simulation uses more than a single process). The attempt to create a fixed joint
 * in a parallel simulation results results in a \a std::logic_error exception. Additionally,
 * it is not possible to use fixed joints in combination with the PolyhedralFrictionSolver or
 * the FFDSolver. The attempt to create a fixed joint in case any of these constraint solvers
 * is selected (see pe::pe_CONSTRAINT_SOLVER) results in a \a std::logic_error exception.
 */
FixedJointID attachFixedJoint( BodyID body1, const Vec3& anchor1,
                               BodyID body2, const Vec3& anchor2, real scale )
{
   using namespace pe::response;

   // Checking the number of active MPI processes
   if( MPISettings::size() > 1 )
      throw std::logic_error( "Cannot create fixed joint in MPI parallel simulation" );

   // Checking the selected constraint solver
   if( IsSelectedSolver<PolyhedralFrictionSolver>::value ||
       IsSelectedSolver<FFDSolver>::value )
      throw std::logic_error( "Selected constraint solver is incapable of handling fixed joints" );

   // Checking whether the bodies contain the anchor points
   if( !( body1->containsRelPoint( anchor1 ) ) )
      throw std::invalid_argument( "Invalid anchor point for body 1" );
   if( !( body2->containsRelPoint( anchor2 ) ) )
      throw std::invalid_argument( "Invalid anchor point for body 2" );

   // Checking the scaling parameter
   if( scale < real(0) || scale > real(1) )
      throw std::invalid_argument( "Invalid scaling parameter" );

   // Creating a new fixed joint
   const id_t sid( UniqueID<Joint>::create() );
   FixedJointID fixedJoint = new FixedJoint( sid, body1, anchor1, body2, anchor2, scale );

   // WARNING: Using friend relationship to add joint to joint storage.
   theCollisionSystem()->jointstorage_.add( fixedJoint );

   return fixedJoint;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for fixed joints.
 * \ingroup fixed_joint
 *
 * \param os Reference to the output stream.
 * \param joint Reference to a constant fixed joint object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const FixedJoint& joint )
{
   os << "--" << pe_BROWN << "FIXED JOINT PARAMETERS" << pe_OLDCOLOR
      << "--------------------------------------------------------\n";
   joint.print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for fixed joint handles.
 * \ingroup fixed_joint
 *
 * \param os Reference to the output stream.
 * \param joint Constant fixed joint handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstFixedJointID joint )
{
   os << "--" << pe_BROWN << "FIXED JOINT PARAMETERS" << pe_OLDCOLOR
      << "--------------------------------------------------------\n";
   joint->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} //namespace pe
