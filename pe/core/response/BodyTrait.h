//=================================================================================================
/*!
 *  \file pe/core/response/BodyTrait.h
 *  \brief Header file for the BodyTrait class
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

#ifndef _PE_CORE_RESPONSE_BODYTRAIT_H_
#define _PE_CORE_RESPONSE_BODYTRAIT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/response/Types.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rigid body customization class for the collision response.
 * \ingroup collision_response
 *
 * The BodyTrait class template is used to adapt the RigidBody class to the used collision
 * response algorithm. Depending on the used algorithm, a body may require additional data
 * or functionality to efficiently support the collision response calculations.\n
 * In order to adapt the RigidBody class to a particular algorithm, the base template needs
 * to be specialized.
 */
template< typename C >  // Type of the configuration
class BodyTrait
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline BodyTrait();
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~BodyTrait();
   //@}
   //**********************************************************************************************

public:
   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void move( real dt );  // Performing a time step of size dt
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the BodyTrait constructor.
 */
template< typename C >  // Type of the configuration
inline BodyTrait<C>::BodyTrait()
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the BodyTrait destructor.
 */
template< typename C >  // Type of the configuration
BodyTrait<C>::~BodyTrait()
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the move function.
 *
 * \param dt Time step size.
 * \return void
 *
 * The move() function calculates a single time step of size \a dt for the rigid body. All
 * derived rigid body classes have to implement this function to update the properties of
 * the rigid body according to the currently acting forces and torques. The following
 * properties of the rigid body might change due to acting forces and torques. All derived
 * classes have to make sure these properties are updated correctly:
 *
 *   - the global position
 *   - the linear and angular velocity (including damping due to drag)
 *   - the orientation/rotation (i.e. the quaternion and the rotation matrix)
 *   - the axis-aligned bounding box
 *   - the current motion (in order to be able to put a body to sleep)
 *
 * Furthermore, the contact graph has to be reset as well as the acting forces and torques
 * if the force reset flag is active.
 */
template< typename C >  // Type of the configuration
void BodyTrait<C>::move( real /*dt*/ )
{
   pe_INTERNAL_ASSERT( false, "Undefined body movement" );
}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE FAST FRICTIONAL DYNAMICS SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the BodyTrait class template for the fast-frictional-dynamics solver.
 * \ingroup collision_response
 *
 * This specialization of the BodyTrait class template adapts bodies to the fast frictional
 * dynamics solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
struct BodyTrait< C<CD,FD,BG,FFDSolver> >
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline BodyTrait();
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~BodyTrait();
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline bool isUpdated() const;
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void setUpdated( bool updated );
   //@}
   //**********************************************************************************************

   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void firstPositionHalfStep ( real dt );
   virtual void secondPositionHalfStep( real dt );
   //@}
   //**********************************************************************************************


protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool updated_;  //!< Update flag.
                   /*!< The update flag is used during the MPI parallel execution of
                        the FFD algorithm to mark remote rigid bodies that have already
                        been updated by a remote MPI process. */
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the FFDSolver specialization of the BodyTrait class template.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline BodyTrait< C<CD,FD,BG,FFDSolver> >::BodyTrait()
   : updated_( false )  // Update flag
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Destructor for the FFDSolver specialization of the BodyTrait class template.
 *
 * Explicit definition to avoid inline warnings!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
BodyTrait< C<CD,FD,BG,FFDSolver> >::~BodyTrait()
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the remote rigid body has been updated.
 *
 * \return \a true in case of an updated rigid body, \a false otherwise.
 *
 * This function returns whether the rigid body has already received an update from a remote
 * MPI process. This function doesn't have to be called explicitly. It is exclusively used
 * internally for the execution of the MPI parallel FFD algorithm.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline bool BodyTrait< C<CD,FD,BG,FFDSolver> >::isUpdated() const
{
   return updated_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the update flag of the rigid body.
 *
 * \param updated \a true to mark the rigid body as updated, \a false to mark it as not updated.
 * \return void
 *
 * This function sets the update flag of the rigid body. The flag indicates whether the rigid
 * body has already received an update from a remote MPI process. This function doesn't have
 * to be called explicitly. It is exclusively used internally for the execution of the MPI
 * parallel FFD algorithm.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void BodyTrait< C<CD,FD,BG,FFDSolver> >::setUpdated( bool updated )
{
   updated_ = updated;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the first position half step.
 *
 * \param dt Time step size.
 * \return void
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void BodyTrait< C<CD,FD,BG,FFDSolver> >::firstPositionHalfStep( real /*dt*/ )
{
   pe_INTERNAL_ASSERT( false, "Undefined first position half step" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the second position half step.
 *
 * \param dt Time step size.
 * \return void
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void BodyTrait< C<CD,FD,BG,FFDSolver> >::secondPositionHalfStep( real /*dt*/ )
{
   pe_INTERNAL_ASSERT( false, "Undefined second position half step" );
}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE OPENCL SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rigid body customization class for the collision response.
 * \ingroup collision_response
 *
 * The BodyTrait class template is used to adapt the RigidBody class to the used collision
 * response algorithm. Depending on the used algorithm, a body may require additional data
 * or functionality to efficiently support the collision response calculations.\n
 * In order to adapt the RigidBody class to a particular algorithm, the base template needs
 * to be specialized.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BodyTrait< C<CD,FD,BG,OpenCLSolver> >
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline BodyTrait();
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~BodyTrait();
   //@}
   //**********************************************************************************************

public:
   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void integrateVelocity( real dt );
   virtual void integratePosition( real dt );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the BodyTrait constructor.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline BodyTrait< C<CD,FD,BG,OpenCLSolver> >::BodyTrait()
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the BodyTrait destructor.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
BodyTrait< C<CD,FD,BG,OpenCLSolver> >::~BodyTrait()
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the velocity integration function.
 *
 * \param dt Time step size.
 * \return void
 *
 * The function integrates the velocity for a single time step of size \a dt for the rigid body under
 * the assumption, that no contact impulses are present. All derived rigid body classes have to
 * implement this function to update the properties of the rigid body according to the currently
 * acting forces and torques. The following properties of the rigid body might change due to acting
 * forces and torques. All derived classes have to make sure these properties are updated correctly:
 *
 *   - the linear and angular velocity
 *
 * Furthermore, the acting forces and torques have to be reset if the force reset flag is active.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void BodyTrait< C<CD,FD,BG,OpenCLSolver> >::integrateVelocity( real /*dt*/ )
{
   pe_INTERNAL_ASSERT( false, "Undefined body velocity integration" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the position integration function.
 *
 * \param dt Time step size.
 * \return void
 *
 * The function integrates the position and orientation for a single time step of size \a dt for
 * the rigid body. All derived rigid body classes have to implement this function to update the
 * properties of the rigid body according to the current linear and angular velocities. The
 * following properties of the rigid body might change. All derived classes have to make sure these
 * properties are updated correctly:
 *
 *   - the global position
 *   - the linear and angular velocity due to damping
 *   - the orientation/rotation (i.e. the quaternion and the rotation matrix)
 *   - the axis-aligned bounding box
 *   - the current motion (in order to be able to put a body to sleep)
 *
 * Furthermore, the contact graph has to be reset.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void BodyTrait< C<CD,FD,BG,OpenCLSolver> >::integratePosition( real /*dt*/ )
{
   pe_INTERNAL_ASSERT( false, "Undefined body position integration" );
}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE OLD DISCRETE ELEMENT SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the BodyTrait class template for the discrete element solver.
 * \ingroup collision_response
 *
 * This specialization of the BodyTrait class template adapts bodies to the discrete element solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
struct BodyTrait< C<CD,FD,BG,DEMSolverObsolete> >
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline BodyTrait();
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~BodyTrait();
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline bool isUpdated() const;
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void setUpdated( bool updated );
   //@}
   //**********************************************************************************************

   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void move( real dt );
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool updated_;  //!< Update flag.
                   /*!< The update flag is used during the MPI parallel execution of
                        the DEM algorithm to mark remote rigid bodies that have already
                        been updated by a remote MPI process. */
   //**********************************************************************************************
};

//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the DEMSolverObsolete specialization of the BodyTrait class template.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline BodyTrait< C<CD,FD,BG,DEMSolverObsolete> >::BodyTrait()
   : updated_( false )  // Update flag
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Destructor for the DEMSolverObsolete specialization of the BodyTrait class template.
 *
 * Explicit definition to avoid inline warnings!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
BodyTrait< C<CD,FD,BG,DEMSolverObsolete> >::~BodyTrait()
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the remote rigid body has been updated.
 *
 * \return \a true in case of an updated rigid body, \a false otherwise.
 *
 * This function returns whether the rigid body has already received an update from a remote
 * MPI process. This function doesn't have to be called explicitly. It is exclusively used
 * internally for the execution of the MPI parallel DEM algorithm.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline bool BodyTrait< C<CD,FD,BG,DEMSolverObsolete> >::isUpdated() const
{
   return updated_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the update flag of the rigid body.
 *
 * \param updated \a true to mark the rigid body as updated, \a false to mark it as not updated.
 * \return void
 *
 * This function sets the update flag of the rigid body. The flag indicates whether the rigid
 * body has already received an update from a remote MPI process. This function doesn't have
 * to be called explicitly. It is exclusively used internally for the execution of the MPI
 * parallel DEM algorithm.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void BodyTrait< C<CD,FD,BG,DEMSolverObsolete> >::setUpdated( bool updated )
{
   updated_ = updated;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the move function.
 *
 * \param dt Time step size.
 * \return void
 *
 * The move() function calculates a single time step of size \a dt for the rigid body. All
 * derived rigid body classes have to implement this function to update the properties of
 * the rigid body according to the currently acting forces and torques. The following
 * properties of the rigid body might change due to acting forces and torques. All derived
 * classes have to make sure these properties are updated correctly:
 *
 *   - the global position
 *   - the linear and angular velocity (including damping due to drag)
 *   - the orientation/rotation (i.e. the quaterion and the rotation matrix)
 *   - the axis-aligned bounding box
 *   - the current motion (in order to be able to put a body to sleep)
 *
 * Furthermore, the contact graph has to be reset as well as the acting forces and torques
 * if the force reset flag is active.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >
void BodyTrait< C<CD,FD,BG,DEMSolverObsolete> >::move( real /*dt*/ )
{
   pe_INTERNAL_ASSERT( false, "Undefined body movement" );
}
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
