//=================================================================================================
/*!
 *  \file pe/core/domaindecomp/Domain.h
 *  \brief Header file for the domain of the rigid body simulation world
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

#ifndef _PE_CORE_DOMAINDECOMP_DOMAIN_H_
#define _PE_CORE_DOMAINDECOMP_DOMAIN_H_


//*************************************************************************************************
// MPI Includes
//*************************************************************************************************

#include <pe/core/MPISettings.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <memory>
#include <pe/core/ParallelTrait.h>
#include <pe/core/domaindecomp/HalfSpace.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/domaindecomp/ProcessGeometry.h>
#include <pe/core/domaindecomp/ProcessStorage.h>
#include <pe/core/Types.h>
#include <pe/math/Infinity.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Unused.h>



namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Domain of the rigid body simulation world.
 * \ingroup core
 *
 * The Domain class represent the local part of the domain of the rigid body simulation world
 * (see the World class description). In case of a non-parallel simulation, the local domain
 * spans the entire domain in the range \f$ (-\infty..\infty) \f$ on all three coordinate axes.
 * In case of MPI parallel simulations the local domain also spans by default the entire domain but
 * can be redefined by \a defineLocalDomain() so that it excludes the subdomains of the connected
 * remote MPI processes.\n
 */
class PE_PROTECTED Domain
{
private:
   //**Type definitions****************************************************************************
   typedef std::auto_ptr<ProcessGeometry>  Geometry;  //!< Handle for the process geometry.
   typedef ProcessStorage<Config>  PS;
   //**********************************************************************************************

public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   inline explicit Domain( PS& processstorage );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const ProcessGeometry* getGeometry() const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool containsPoint        ( const Vec3& gpos ) const;
   inline bool containsPointStrictly( const Vec3& gpos ) const;
   inline bool ownsPoint            ( const Vec3& gpos ) const;

   inline bool intersectsWith( ConstBodyID     o ) const;
   inline bool intersectsWith( ConstSphereID   o ) const;
   inline bool intersectsWith( ConstBoxID      o ) const;
   inline bool intersectsWith( ConstCapsuleID  o ) const;
   inline bool intersectsWith( ConstCylinderID o ) const;
   inline bool intersectsWith( ConstUnionID    o ) const;

          bool requires      ( ConstBodyID body ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Geometry geometry_;  //!< The geometry of the local process.
                        /*!< The geometry of the local process, i.e. the physical expansion of
                             the process, is used to evaluate whether rigid bodies are (partially)
                             contained in the local process or whether global coordinates fall
                             into the space occupied by the local process. The geometry/expansion
                             is assigned during construction of the process according to the
                             strategy pattern and managed by this handle. */
   PS& processstorage_;
   //@}
   //**********************************************************************************************

   //**Local process setup functions***************************************************************
   /*! \cond PE_INTERNAL */
   template< typename Geometry >
   friend void defineLocalDomain( const Geometry& geometry );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Domain class.
 *
 * \param processstorage Reference to the central process storage.
 */
inline Domain::Domain( PS& processstorage )
   : geometry_( new HalfSpace( Vec3(1, 0, 0), -inf ) )
   , processstorage_( processstorage )
{
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a read-only pointer to the local subdomain geometry.
 *
 * \return A read-only pointer to the local subdomain geometry.
 */
inline const ProcessGeometry* Domain::getGeometry() const
{
   return geometry_.get();
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Tests if a global coordinate is contained in the local domain of the simulation world.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained in the local domain, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the local domain of
 * the simulation world. In case the coordinate is within the bounds of the domain, the function
 * returns \a true, otherwise it returns \a false. If the point is located on the surface of the
 * local domain it is considered to be contained. Thus a point can be contained in several process
 * subdomains. Note that for a non-parallel simulation, this
 * function always returns \a true.
 */
inline bool Domain::containsPoint( const Vec3& gpos ) const
{
#if HAVE_MPI
   return geometry_->containsPoint( gpos );
#else
   UNUSED( gpos );
   return true;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a global coordinate is contained strictly in the local domain of the simulation world.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained strictly in the local domain, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the local domain of
 * the simulation world. In case the coordinate is within the bounds of the domain, the function
 * returns \a true, otherwise it returns \a false. If the point is located on the surface of the
 * local domain it is considered to be \em not contained. Thus a point cannot be contained in
 * several process subdomains. Note that for a non-parallel simulation, this
 * function always returns \a true.
 */
inline bool Domain::containsPointStrictly( const Vec3& gpos ) const
{
#if HAVE_MPI
   return geometry_->containsPointStrictly( gpos );
#else
   UNUSED( gpos );
   return true;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a global coordinate is owned by the local domain of the simulation world.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is owned by the local domain, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the local domain of
 * the simulation world and that it is contained in no remote domain with lower rank. It differs
 * from \a containsPoint in cases where the point is located on a boundary surface. In such cases
 * the point is defined to be part of the process of lowest rank. Note that for a non-parallel
 * simulation, this function always returns \a true.
 */
inline bool Domain::ownsPoint( const Vec3& gpos ) const
{
#if HAVE_MPI
   if( containsPointStrictly( gpos ) )
      return true;
   if( !containsPoint( gpos ) )
      return false;

   const int rank( MPISettings::rank() );

   const PS::ConstIterator pbegin( processstorage_.begin() );
   const PS::ConstIterator pend  ( processstorage_.end()   );

   for( PS::ConstIterator process=pbegin; process!=pend; ++process ) {
      if( process->getRank() < rank && process->containsPoint( gpos ) )
         return false;
   }

   return true;
#else
   UNUSED( gpos );
   return true;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given rigid body intersects with the local process.
 *
 * \param o The rigid body to be tested.
 * \return \a true if the rigid body intersects with the local process, \a false if not.
 * \exception std::invalid_argument Invalid infinite rigid body detected.
 *
 * This function tests whether the given rigid body is partially contained in the local
 * process. In case the body is partially contained in the local process (i.e. overlaps
 * the boundary to the local process) the function returns \a true, otherwise it returns
 * \a false. Note that it is not possible to test infinite rigid bodies (as for instance
 * planes). The attempt to test an infinite rigid body results in a \a std::invalid_argument
 * exception.
 */
inline bool Domain::intersectsWith( ConstBodyID o ) const
{
#if HAVE_MPI
   return geometry_->intersectsWith( o );
#else
   UNUSED( o );
   return true;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given sphere intersects with the local process.
 *
 * \param o The sphere to be tested.
 * \return \a true if the sphere intersects with the local process, \a false if not.
 *
 * This function tests whether the given sphere is partially contained in the local
 * process. In case the sphere is partially contained in the local process (i.e. overlaps
 * the boundary to the local process) the function returns \a true, otherwise it returns
 * \a false.
 */
inline bool Domain::intersectsWith( ConstSphereID o ) const
{
#if HAVE_MPI
   return geometry_->intersectsWith( o );
#else
   UNUSED( o );
   return true;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given box intersects with the local process.
 *
 * \param o The box to be tested.
 * \return \a true if the box intersects with the local process, \a false if not.
 *
 * This function tests whether the given box is partially contained in the local
 * process. In case the box is partially contained in the local process (i.e. overlaps
 * the boundary to the local process) the function returns \a true, otherwise it returns
 * \a false.
 */
inline bool Domain::intersectsWith( ConstBoxID o ) const
{
#if HAVE_MPI
   return geometry_->intersectsWith( o );
#else
   UNUSED( o );
   return true;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given capsule intersects with the local process.
 *
 * \param o The capsule to be tested.
 * \return \a true if the capsule intersects with the local process, \a false if not.
 *
 * This function tests whether the given capsule is partially contained in the local
 * process. In case the capsule is partially contained in the local process (i.e. overlaps
 * the boundary to the local process) the function returns \a true, otherwise it returns
 * \a false.
 */
inline bool Domain::intersectsWith( ConstCapsuleID o ) const
{
#if HAVE_MPI
   return geometry_->intersectsWith( o );
#else
   UNUSED( o );
   return true;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given cylinder intersects with the local process.
 *
 * \param o The cylinder to be tested.
 * \return \a true if the cylinder intersects with the local process, \a false if not.
 *
 * This function tests whether the given cylinder is partially contained in the local
 * process. In case the cylinder is partially contained in the local process (i.e. overlaps
 * the boundary to the local process) the function returns \a true, otherwise it returns
 * \a false.
 */
inline bool Domain::intersectsWith( ConstCylinderID o ) const
{
#if HAVE_MPI
   return geometry_->intersectsWith( o );
#else
   UNUSED( o );
   return true;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given union intersects with the local process.
 *
 * \param o The union to be tested.
 * \return \a true if the union intersects with the local process, \a false if not.
 *
 * This function tests whether the given union is partially contained in the local
 * process. In case the union is partially contained in the local process (i.e. overlaps
 * the boundary to the local process) the function returns \a true, otherwise it returns
 * \a false.
 */
inline bool Domain::intersectsWith( ConstUnionID o ) const
{
#if HAVE_MPI
   return geometry_->intersectsWith( o );
#else
   UNUSED( o );
   return true;
#endif
}
//*************************************************************************************************

} // namespace pe

#endif
