//=================================================================================================
/*!
 *  \file pe/core/rigidbody/rigidbodytrait/DEMSolverObsolete.h
 *  \brief Specialization of the RigidBodyTrait class template for the DEM solver.
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

#ifndef _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_DEMSOLVEROBSOLETE_H_
#define _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_DEMSOLVEROBSOLETE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <cmath>
#include <pe/core/rigidbody/RigidBodyBase.h>
#include <pe/core/response/Types.h>
#include <pe/core/Settings.h>
#include <pe/core/Thresholds.h>
#include <pe/math/Accuracy.h>
#include <pe/math/Twist.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>
#include <pe/util/Types.h>
#include <pe/util/Vector.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the RigidBodyTrait class template for the discrete element solver.
 * \ingroup rigid_body
 *
 * This specialization of the RigidBodyTrait class template adapts all rigid bodies to the requirements
 * of the discrete element solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> > : public RigidBodyBase
{
protected:
   //**Type definitions****************************************************************************
   typedef RigidBodyBase                    Parent;          //!< The type of the parent class.
   typedef C<CD,FD,BG,response::DEMSolverObsolete>  Config;          //!< Type of the configuration.
   typedef Process*                         ProcessID;       //!< Handle for a remote process.
   typedef const Process*                   ConstProcessID;  //!< Handle for a constant remote process.

   //! Vector for remote MPI processes the rigid body is contained in.
   typedef PtrVector<Process,NoDelete>  Processes;
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef typename Processes::Iterator       ProcessIterator;       //!< Iterator over the connected processes.
   typedef typename Processes::ConstIterator  ConstProcessIterator;  //!< ConstIterator over the connected processes.
   typedef size_t                             SizeType;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit RigidBodyTrait( BodyID body );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~RigidBodyTrait() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vec3& getLinearAcc () const;
   inline const Vec3& getAngularAcc() const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void setLinearAcc ( const Vec3& lacc );
   inline void setAngularAcc( const Vec3& aacc );
   //@}
   //**********************************************************************************************

protected:
   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void resetGlobalVelocity();
   inline void setGlobalVelocity  ( const Vec3& v, const Vec3& w );
   //@}
   //**********************************************************************************************

public:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool hasSuperBody() const;
   //@}
   //**********************************************************************************************

   //**Process functions***************************************************************************
   /*!\name Process functions */
   //@{
   inline void                 registerProcess  ( ProcessID process );
   inline void                 deregisterProcess( ProcessID process );
   inline bool                 isRegistered( ConstProcessID process ) const;
   inline bool                 hasProcesses  () const;
   inline ProcessIterator      beginProcesses();
   inline ConstProcessIterator beginProcesses() const;
   inline ProcessIterator      endProcesses  ();
   inline ConstProcessIterator endProcesses  () const;
   inline SizeType             sizeProcesses () const;
   inline void                 clearProcesses();
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 vdot_;  //!< The linear acceleration of this body.
   Vec3 wdot_;  //!< The angular acceleration of this body.
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Processes processes_;  //!< Vector of all processes the rigid body is currently contained in.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   template<typename,typename,typename> friend class response::DEMSolverObsolete;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the RigidBodyTrait constructor.
 *
 * \param body The body ID of this rigid body.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::RigidBodyTrait( BodyID body )
   : Parent( body )  // Initialization of the parent class
   , vdot_ ()        // The linear acceleration of this body
   , wdot_ ()        // The angular acceleration of this body
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the RigidBodyTrait destructor.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::~RigidBodyTrait()
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the global velocity of the rigid body.
 *
 * \return void
 *
 * This function reset the global linear and angular velocity of the rigid body to zero.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::resetGlobalVelocity()
{
   v_.reset();  // Resetting the global linear velocity to zero
   w_.reset();  // Resetting the global angular velocity to zero
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global velocity of the rigid body.
 *
 * \param v The global linear velocity.
 * \param w The global angular velocity.
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
inline void RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::setGlobalVelocity( const Vec3& v, const Vec3& w )
{
   v_ = v;  // Setting the global linear velocity
   w_ = w;  // Setting the global angular velocity
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a new remote process the rigid body is contained in.
 *
 * \param process The remote process to be registered with the rigid body.
 * \return void
 *
 * This function registers the given remote process with the rigid body. In case the process is
 * already registered, it is not registered again.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::registerProcess( ProcessID process )
{
   if( !isRegistered( process ) )
      processes_.pushBack( process );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistering a remote process from the rigid body.
 *
 * \param process The remote process to be deregistered from the rigid body.
 * \return void
 *
 * This function deregisters the given remote process from the rigid body.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::deregisterProcess( ProcessID process )
{
   ProcessIterator pos = std::find( processes_.begin(), processes_.end(), process );
   if( pos != processes_.end() )
      processes_.erase( pos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks whether the given remote process is registered with the rigid body.
 *
 * \param process The remote process that possibly registered with the rigid bdoy.
 * \return \a true if the given process is registered with the rigid body, \a false if not.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline bool RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::isRegistered( ConstProcessID process ) const
{
   if( std::find( processes_.begin(), processes_.end(), process ) == processes_.end() )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether any processes are registered with the rigid body.
 *
 * \return \a true if at least one process is registered with the rigid body, \a false if not.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline bool RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::hasProcesses() const
{
   return !processes_.isEmpty();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first remote process the rigid body is contained in.
 *
 * \return Iterator to the first remote process.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::ProcessIterator
   RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::beginProcesses()
{
   return processes_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first remote process the rigid body is contained in.
 *
 * \return Iterator to the first remote process.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::ConstProcessIterator
   RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::beginProcesses() const
{
   return processes_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last process the rigid body is contained in.
 *
 * \return Iterator just past the last remote process.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::ProcessIterator
   RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::endProcesses()
{
   return processes_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last process the rigid body is contained in.
 *
 * \return Iterator just past the last remote process.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::ConstProcessIterator
   RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::endProcesses() const
{
   return processes_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of registered processes.
 *
 * \return The number of registered processes.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::SizeType RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::sizeProcesses() const
{
   return processes_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing all registered remote processes from the rigid body.
 *
 * \return void
 *
 * This function clears all references to remote processes the rigid body may be contained in.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::clearProcesses()
{
   processes_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the linear acceleration of the rigid body.
 *
 * \return linear acceleration
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Vec3& RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::getLinearAcc() const
{
   // TODO
//    if( hasSuperBody() )
//       vdot_ = sb_->getLinearAcc();
   return vdot_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the angular acceleration of the rigid body.
 *
 * \return linear acceleration
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Vec3& RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::getAngularAcc() const
{
   // TODO
//    if( hasSuperBody() )
//       wdot_ = sb_->getAngularAcc();
   return wdot_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global linear acceleration of the rigid body.
 *
 * \param lacc The global linear acceleration.
 * \return void
 *
 * This function sets the linear velocity of the rigid body. If the body is infinite (as for
 * instance a plane or an union containing a plane), if the global position is fixed, or if
 * the body is contained in a superordinate body the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::setLinearAcc( const Vec3& lacc )
{
   if( !hasSuperBody() && !fixed_ ) {
      vdot_ = lacc;
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global angular acceleration of the rigid body.
 *
 * \param aacc The global angular acceleration.
 * \return void
 *
 * This function sets the linear velocity of the rigid body. If the body is infinite (as for
 * instance a plane or an union containing a plane), if the global position is fixed, or if
 * the body is contained in a superordinate body the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::setAngularAcc( const Vec3& aacc )
{
   if( !hasSuperBody() && !fixed_ ) {
      wdot_ = aacc;
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief This function assures that any of the derived classes has implemented the hasSuperBody()
 * function
 *
 * \return bool
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
bool RigidBodyTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::hasSuperBody() const
{
   pe_INTERNAL_ASSERT( false, "Undefined use of the hasSuperBody() function" );
   return false;
}
//*************************************************************************************************

} // namespace pe

#endif
