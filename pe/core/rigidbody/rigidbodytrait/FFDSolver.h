//=================================================================================================
/*!
 *  \file pe/core/rigidbody/rigidbodytrait/FFDSolver.h
 *  \brief Specialization of the RigidBodyTrait class template for the FFD solver.
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

#ifndef _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_FFDSOLVER_H_
#define _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_FFDSOLVER_H_


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
#include <pe/math/Matrix6x6.h>
#include <pe/math/Twist.h>
#include <pe/system/FFDConfig.h>
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
/*!\brief Specialization of the RigidBodyTrait class template for the fast frictional dynamics solver.
 * \ingroup rigid_body
 *
 * This specialization of the RigidBodyTrait class template adapts all rigid bodies to the requirements
 * of the fast frictional dynamics solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> > : public RigidBodyBase
{
protected:
   //**Type definitions****************************************************************************
   typedef C<CD,FD,BG,response::FFDSolver>  Config;          //!< Type of the configuration.
   typedef Process*                         ProcessID;       //!< Handle for a remote process.
   typedef const Process*                   ConstProcessID;  //!< Handle for a constant remote process.

   //! Vector for remote MPI processes the rigid body is contained in.
   typedef PtrVector<Process,NoDelete>  Processes;

   typedef RigidBodyBase  Parent;       //!< The type of the parent class.
   typedef Vector<Twist>  Constraints;  //!< Container type for constraint normals.
   typedef Vector<real>   Offsets;      //!< Container type for constraint offsets.
   typedef Vector<real>   CoFs;         //!< Container for the coefficients of friction.
   typedef Vector<Twist>  Bounds;       //!< Container type for friction bounds.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef typename Processes::Iterator       ProcessIterator;       //!< Iterator over the connected processes.
   typedef typename Processes::ConstIterator  ConstProcessIterator;  //!< ConstIterator over the connected processes.
   typedef Matrix6x6<real>                    FrictionBase;          //!< Type of the friction base.
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
   inline bool    hasConstraints   () const;
   inline size_t  countConstraints () const;
   inline size_t  countCoFs        () const;
   inline size_t  countBounds      () const;

   inline size_t              getDimension   ()           const;
   inline real                getWeightedCV  ()           const;
   inline real                getCV          ()           const;
   inline const Twist&        getConstraint  ( size_t i ) const;
   inline real                getOffset      ( size_t i ) const;
   inline real                getCoF         ( size_t i ) const;
   inline const Twist&        getBound       ( size_t i ) const;
   inline const FrictionBase& getFrictionBase()           const;
   //@}
   //**********************************************************************************************

protected:
   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void resetGlobalVelocity();
   inline void setGlobalVelocity( const Vec3& v, const Vec3& w );
   //@}
   //**********************************************************************************************

public:
   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   void firstVelocityHalfStep ( real dt );
   void secondVelocityHalfStep( real dt );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void addWeightedCV   ( real wcv );
   inline void addCV           ( real cv );
   inline void addConstraint   ( const Twist& constraint );
   inline void addOffset       ( real offset );
   inline void addCoF          ( real cof );
   inline void addBound        ( const Twist& bound );
   inline void resetConstraints();
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
   inline void                 clearProcesses();
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline const Twist toFrictionBase( const Twist& bound ) const;
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   real wcv_;                 //!< The sum of the weighted constraint violations.
                              /*!< This value is the enumerator for the calculation of the
                                   resulting coefficient of restitution. */
   real cv_;                  //!< The total sum of all constraint violations.
                              /*!< This value is the denominator for the calculation of the
                                   resulting coefficient of restitution. */
   Constraints constraints_;  //!< The currently active constraint normals.
   Offsets offsets_;          //!< The currently active constraint offsets.
   CoFs cofs_;                //!< The coefficients of friction.
   Bounds bounds_;            //!< The currently active friction bounds.
   size_t dimension_;         //!< The current dimension of the friction base.
   FrictionBase base_;        //!< Friction base for the estimation of the frictional response.
   Processes processes_;      //!< Vector of all processes the rigid body is currently contained in.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   template<typename,typename,typename> friend class response::FFDSolver;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief RigidBodyTrait constructor for the FFDSolver specialization.
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
RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::RigidBodyTrait( BodyID body )
   : Parent( body )  // Initialization of the parent class
   , wcv_(0)         // The sum of the weighted constraint violations
   , cv_(0)          // The total sum of all constraint violations
   , constraints_()  // The currently active constraint normals
   , offsets_()      // The currently active constraint offsets
   , cofs_()         // The coefficients of friction
   , bounds_()       // The currently active friction bounds
   , dimension_(0)   // The current dimension of the friction base
   , base_()         // Friction base for the estimation of the frictional response
{
   for( size_t i=0; i<6; ++i )
      base_(i,i) = real(1);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief RigidBodyTrait destructor for the FFDSolver specialization.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::~RigidBodyTrait()
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the rigid body currently has an active constraint.
 *
 * \return \a true in case the body has active constraints, \a false if it has not.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline bool RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::hasConstraints() const
{
   using namespace pe::response::ffd;

   // Checking for the correct number of normal constraints, constraint offsets, friction
   // coefficients, and friction bounds
   pe_INTERNAL_ASSERT( constraints_.size() == offsets_.size(), "Invalid number of constraints" );
   pe_INTERNAL_ASSERT( !friction || cofs_.size() == constraints_.size(), "Invalid number of constraints" );
   pe_INTERNAL_ASSERT( !friction || cofs_.size() == offsets_.size(), "Invalid number of constraint offsets" );
   pe_INTERNAL_ASSERT( cofs_.size()*frictionSamples == bounds_.size(), "Invalid number of friction bounds" );

   return constraints_.size() > 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of currently active constraints.
 *
 * \return The number of currently active constraints.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline size_t RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::countConstraints() const
{
   return constraints_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of friction coefficients.
 *
 * \return The number of friction coefficients.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline size_t RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::countCoFs() const
{
   return cofs_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of currently active friction bounds.
 *
 * \return The number of currently active friction bounds.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline size_t RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::countBounds() const
{
   return bounds_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current dimension of the friction base.
 *
 * \return The current dimension of the friction base.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline size_t RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::getDimension() const
{
   return dimension_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current sum of weighted constraint violations.
 *
 * \return The current sum of weighted constraint violations.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline real RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::getWeightedCV() const
{
   return wcv_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current sum of all constraint violations.
 *
 * \return The current sum of all constraint violations.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline real RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::getCV() const
{
   return cv_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the indexed constraint normal.
 *
 * \param i Constraint access index. The index has to be in the range \f$[0..n)\f$ (with n the number of constraints).
 * \return The indexed constraint normal.
 *
 * In case pe_INTERNAL_ASSERT() is active, this access function performs an index check.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Twist& RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::getConstraint( size_t i ) const
{
   pe_INTERNAL_ASSERT( i < constraints_.size(), "Invalid constraint access index" );
   return constraints_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the indexed constraint offset.
 *
 * \param i Offset access index. The index has to be in the range \f$[0..n)\f$ (with n the number of offsets).
 * \return The indexed constraint offset.
 *
 * In case pe_INTERNAL_ASSERT() is active, this access function performs an index check.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline real RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::getOffset( size_t i ) const
{
   pe_INTERNAL_ASSERT( i < offsets_.size(), "Invalid offset access index" );
   return offsets_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the indexed coefficient of friction.
 *
 * \param i Access index. The index has to be in the range \f$[0..n)\f$ (with n the number of friction coefficients).
 * \return The indexed coefficient of friction.
 *
 * In case pe_INTERNAL_ASSERT() is active, this access function performs an index check.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline real RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::getCoF( size_t i ) const
{
   pe_INTERNAL_ASSERT( i < cofs_.size(), "Invalid coefficient of friction access index" );
   return cofs_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the indexed friction bound.
 *
 * \param i Constraint access index. The index has to be in the range \f$[0..n)\f$ (with n the number of friction bounds).
 * \return The indexed friction bound.
 *
 * In case pe_INTERNAL_ASSERT() is active, this access function performs an index check.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Twist& RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::getBound( size_t i ) const
{
   pe_INTERNAL_ASSERT( i < bounds_.size(), "Invalid friction bound access index" );
   return bounds_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the friction base for the estimation of the frictional collision response.
 *
 * \return The friction base.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const typename RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::FrictionBase&
   RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::getFrictionBase() const
{
   return base_;
}
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::resetGlobalVelocity()
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::setGlobalVelocity( const Vec3& v, const Vec3& w )
{
   v_ = v;  // Setting the global linear velocity
   w_ = w;  // Setting the global angular velocity
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a weighted constraint violation to the rigid body.
 *
 * \param wcv The weighted constraint violation.
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::addWeightedCV( real wcv )
{
   wcv_ += wcv;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a constraint violation to the rigid body.
 *
 * \param cv The constraint violation.
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::addCV( real cv )
{
   cv_ += cv;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a normal constraint to the motion of the rigid body.
 *
 * \param constraint The new normal constraint.
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::addConstraint( const Twist& constraint )
{
   constraints_.pushBack( constraint );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a constraint offset to the rigid body.
 *
 * \param offset The new constraint offset.
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::addOffset( real offset )
{
   offsets_.pushBack( offset );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a coefficient of friction for the calculation of the frictional response.
 *
 * \param cof The coefficient of friction.
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::addCoF( real cof )
{
   cofs_.pushBack( cof );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Converts the given twist to the current friction base.
 *
 * \param twist The twist to be converted.
 * \return The converted twist.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Twist RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::toFrictionBase( const Twist& twist ) const
{
   Twist tmp( 0 );

   for( size_t i=0; i<6; ++i ) {
      for( size_t j=0; j<6; ++j ) {
         tmp[i] += base_[j*6+i] * twist[j];
      }
   }

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a friction bound to the motion of the rigid body.
 *
 * \param bound The new friction bound.
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::addBound( const Twist& bound )
{
   bounds_.pushBack( bound );

   Twist sbase( Iinv_ * bound.angular(), invMass_ * bound.linear() );

   if( this->dimension_ > 0 )
      sbase = toFrictionBase( sbase );

   bool uniqueFriction( false );
   for( int i=dimension_; i<6; ++i ) {
      if( std::fabs( sbase[i] ) > accuracy )
         uniqueFriction = true;
   }

   if( !uniqueFriction ) return;

   real w1(0), w2(0), mu(0), omega(0), s(0), c(0), tmp1(0), tmp2(0), v(0);

   for( size_t i=5; i>dimension_; --i )
   {
      if( std::fabs( sbase[i] ) > accuracy ) {
         w1   = sbase[i-1];
         mu   = std::fabs( w1 );
         w2   = sbase[i];
         tmp1 = std::fabs( w2 );
         if( tmp1 > mu ) mu = tmp1;
         tmp1   = w1/mu;
         omega  = tmp1*tmp1;
         tmp1   = w2/mu;
         omega += tmp1*tmp1;
         omega  = mu * std::sqrt( omega );
         if( w1 < real(0) ) omega = -omega;
         c = w1/omega;
         s = w2/omega;

         sbase[i-1] = omega;
         sbase[i  ] = real(0);

         v = s/(1+c);
         for( size_t j=0; j<6; ++j ) {
            tmp1 = base_(j,i-1);
            tmp2 = base_(j,i  );
            base_(j,i-1) = c*tmp1 + s*tmp2;
            base_(j,i  ) = v * ( tmp1 + base_(j,i-1) ) - tmp2;
         }
      }
   }

   ++dimension_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resets the currently active constraints.
 *
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::resetConstraints()
{
   wcv_ = real(0);
   cv_  = real(0);

   constraints_.clear();
   offsets_.clear();
   bounds_.clear();
   cofs_.clear();

   dimension_ = 0;
   base_.reset();
   for( size_t i=0; i<6; ++i )
      base_(i,i) = real(1);
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::registerProcess( ProcessID process )
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::deregisterProcess( ProcessID process )
{
   ProcessIterator pos = std::find( processes_.begin(), processes_.end(), process );
   if( pos != processes_.end() )
      processes_.erase( pos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks whether the given remote process is registered with the rigid body.
 *
 * \param process The remote process that possibly registered with the rigid body.
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
inline bool RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::isRegistered( ConstProcessID process ) const
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
inline bool RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::hasProcesses() const
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
inline typename RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::ProcessIterator
   RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::beginProcesses()
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
inline typename RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::ConstProcessIterator
   RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::beginProcesses() const
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
inline typename RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::ProcessIterator
   RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::endProcesses()
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
inline typename RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::ConstProcessIterator
   RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::endProcesses() const
{
   return processes_.end();
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
inline void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::clearProcesses()
{
   processes_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Velocity update of a rigid body.
 *
 * \param dt Time step size.
 * \return void
 *
 * This function performs the first velocity half step for a rigid body and updates the
 * velocity according to the currently acting forces.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::firstVelocityHalfStep( real dt )
{
   // Don't move a fixed rigid body
   pe_INTERNAL_ASSERT( !fixed_, "Fixed rigid body detected" );

   // Calculating the linear acceleration by the equation
   //   force * m^(-1) + 2*gravity
   const Vec3 vdot( force_ * invMass_ + real(2)*Settings::gravity() );

   // Calculating the angular acceleration (for general torques and angular velocities) by the equation
   //   R * Iinv * R^T * ( torque - w x ( R * I * R^T * w ) )
   const Mat3 R_transposed = trans( R_ );
   const Vec3 wdot( R_ * ( Iinv_ * ( R_transposed * ( torque_ - ( w_ % ( R_ * ( I_ * ( R_transposed * w_ ) ) ) ) ) ) ) );

   // Updating the linear velocity
   v_ += vdot * dt;

   // Updating the angular velocity
   w_ += wdot * dt;

   // Damping the movement for the entire time step size of 2*dt
   if( Settings::damping() < real(1) ) {
      const real drag( std::pow( Settings::damping(), real(2)*dt ) );
      v_ *= drag;
      w_ *= drag;
   }

   // Calculating the current motion of the rigid body
   //calcMotion();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Velocity update of a rigid body.
 *
 * \param dt Time step size.
 * \return void
 *
 * This function performs the second velocity half step for a rigid body and updates the
 * velocity according to the currently acting forces.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void RigidBodyTrait< C<CD,FD,BG,response::FFDSolver> >::secondVelocityHalfStep( real dt )
{
   // Don't move a fixed rigid body
   pe_INTERNAL_ASSERT( !fixed_, "Fixed rigid body detected" );

   // Calculating the linear acceleration by the equation
   //   force * m^(-1)
   const Vec3 vdot( force_ * invMass_ );

   // Calculating the angular acceleration by the equation (for general torques and angular velocities)
   //   R * Iinv * R^T * ( torque - w x ( R * I * R^T * w ) )
   const Vec3 wdot( R_ * ( Iinv_ * ( trans(R_) * (torque_- ( w_ % ( R_ * ( I_ * ( trans( R_ ) * w_ ) ) ) ) ) ) ) );

   // Updating the linear velocity
   v_ += vdot * dt;

   // Updating the angular velocity
   w_ += wdot * dt;

   // Calculating the current motion of the rigid body
   //calcMotion();
}
//*************************************************************************************************

} // namespace pe

#endif
