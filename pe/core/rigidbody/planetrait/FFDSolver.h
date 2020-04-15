//=================================================================================================
/*!
 *  \file pe/core/rigidbody/planetrait/FFDSolver.h
 *  \brief Specialization of the PlaneTrait class template for the fast frictional dynamics solver
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

#ifndef _PE_CORE_RIGIDBODY_PLANETRAIT_FFDSOLVER_H_
#define _PE_CORE_RIGIDBODY_PLANETRAIT_FFDSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/PlaneBase.h>
#include <pe/core/rigidbody/planetrait/Default.h>
#include <pe/core/response/Types.h>
#include <pe/core/Settings.h>
#include <pe/core/Types.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the PlaneTrait class template for the fast frictional dynamics solver.
 * \ingroup plane
 *
 * This specialization of the PlaneTrait class template adapts the plane geometry to the
 * requirements of the fast frictional dynamics solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class PlaneTrait< C<CD,FD,BG,response::FFDSolver> > : public PlaneBase
{
protected:
   //**Type definitions****************************************************************************
   typedef PlaneBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit PlaneTrait( id_t sid, id_t uid, const Vec3& gpos, const Vec3& normal,
                        real d, MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~PlaneTrait() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void firstPositionHalfStep ( real dt );
   virtual void secondPositionHalfStep( real dt );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the FFDSolver specialization of the PlaneTrait class template.
 *
 * \param sid System-specific ID for the plane.
 * \param uid User-specific ID for the plane.
 * \param gpos The global position (anchor point) of the plane.
 * \param normal The plane's normal in reference to the global world frame, \f$ |n| = 1 \f$.
 * \param d The displacement of the plane.
 * \param material The material of the plane.
 * \param visible Specifies if the plane is visible in a visualization.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
PlaneTrait< C<CD,FD,BG,response::FFDSolver> >::PlaneTrait( id_t sid, id_t uid, const Vec3& gpos, const Vec3& normal,
                                                           real d, MaterialID material, bool visible )
   : Parent( sid, uid, gpos, normal, d, material, visible )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the FFDSolver specialization of the PlaneTrait class template.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
PlaneTrait< C<CD,FD,BG,response::FFDSolver> >::~PlaneTrait()
{}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the first position half step of size \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. As an
 * infinite rigid body, a plane is not allowed to move.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void PlaneTrait< C<CD,FD,BG,response::FFDSolver> >::firstPositionHalfStep( real /*dt*/ )
{
   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants()      , "Invalid plane state detected"        );
   pe_INTERNAL_ASSERT( !hasSuperBody()        , "Invalid superordinate body detected" );
   pe_INTERNAL_ASSERT( !hasContacts()         , "Invalid contacts detected"           );
   pe_INTERNAL_ASSERT( !this->hasConstraints(), "Invalid constraints detected"        );
   pe_INTERNAL_ASSERT( !remote_               , "Invalid remote plane detected"       );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the second position half step of size \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. As an
 * infinite rigid body, a plane is not allowed to move.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void PlaneTrait< C<CD,FD,BG,response::FFDSolver> >::secondPositionHalfStep( real /*dt*/ )
{
   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants()      , "Invalid plane state detected"        );
   pe_INTERNAL_ASSERT( !hasSuperBody()        , "Invalid superordinate body detected" );
   pe_INTERNAL_ASSERT( !hasContacts()         , "Invalid contacts detected"           );
   pe_INTERNAL_ASSERT( !this->hasConstraints(), "Invalid constraints detected"        );
   pe_INTERNAL_ASSERT( !remote_               , "Invalid remote plane detected"       );

   // Resetting the acting forces
   if( Settings::forceReset() )
      RigidBody::resetForce();

   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid plane state detected" );
}
//*************************************************************************************************

} // namespace pe

#endif
