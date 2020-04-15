//=================================================================================================
/*!
 *  \file pe/core/detection/coarse/BodyTrait.h
 *  \brief Rigid body customization class for the coarse collision detection
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

#ifndef _PE_CORE_DETECTION_COARSE_BODYTRAIT_H_
#define _PE_CORE_DETECTION_COARSE_BODYTRAIT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <vector>
#include <pe/core/detection/coarse/BoundingBox.h>
#include <pe/core/detection/coarse/IntervalEndpoint.h>
#include <pe/core/detection/coarse/Types.h>
#include <pe/core/Types.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>


namespace pe {

namespace detection {

namespace coarse {

//=================================================================================================
//
//  COARSE COLLISION DETECTION BODY TRAIT
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rigid body customization class for the coarse collision detection.
 * \ingroup coarse_collision_detection
 *
 * The coarse detection module BodyTrait class template is used to adapt the RigidBody class
 * to the used coarse collision detection algorithm. For instance, this class configures the
 * axis-aligned bounding box for rigid bodies depending on the requirements of the selected
 * coarse collision detection algorithm.\n
 * In order to adapt the RigidBody class to a particular algorithm, the base template needs
 * to be specialized.
 */
template< typename C >  // Type of the configuration
class BodyTrait
{
public:
   //**Type definitions****************************************************************************
   typedef BoundingBox<real>  AABB;  //!< Type of the axis-aligned bounding box.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline BodyTrait( BodyID body );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   AABB aabb_;  //!< Axis-aligned bounding box for the rigid body.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the BodyTrait constructor.
 *
 * \param body The rigid body containing this bounding box.
 */
template< typename C >  // Type of the configuration
inline BodyTrait<C>::BodyTrait( BodyID /*body*/ )
   : aabb_()  // Axis-aligned bounding box for the rigid body
{}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE HIERARCHICAL HASH GRID ALGORITHM
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the BodyTrait class template for the 'hierarchical hash-grid' algorithm.
 * \ingroup coarse_collision_detection
 *
 * This specialization of the BodyTrait class template adapts rigid bodies to the hierarchical hash-grid
 * coarse collision detection algorithm.
 */
template< typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BodyTrait< C<HashGrids,FD,BG,CR> >
{
public:
   //**Type definitions****************************************************************************
   typedef BoundingBox<real>  AABB;  //!< Type of the axis-aligned bounding box.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline BodyTrait( BodyID body );
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline real   getAABBSize() const;
   inline void*  getGrid    () const;
   inline size_t getHash    () const;
   inline size_t getCellId  () const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void setGrid  ( void*  grid );
   inline void setHash  ( size_t hash );
   inline void setCellId( size_t cell );
   //@}
   //**********************************************************************************************   

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   AABB   aabb_;    //!< Axis-aligned bounding box for the rigid body.
   void*  grid_;    //!< Pointer to the hash grid this rigid body is currently assigned to.
   size_t hash_;    //!< Current hash value of this rigid body.
   size_t cellId_;  //!< The body's index in the body container of the grid cell this rigid body is currently assigned to.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the BodyTrait<HashGrids> specialization.
 *
 * \param body The rigid body containing this bounding box.
 */
template< typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline BodyTrait< C<HashGrids,FD,BG,CR> >::BodyTrait( BodyID /*body*/ )
   : aabb_  ()   // Axis-aligned bounding box for the rigid body
   , grid_  (0)  // Pointer to the hash grid this rigid body is currently assigned to
   , hash_  (0)  // Current hash value of this rigid body
   , cellId_(0)  // The body's index in the body container of the grid cell this rigid body is currently assigned to
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the length of the longest side of the AABB of the rigid body.
 *
 * \return The length of the longest side of the AABB of the rigid body.
 */
template< typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline real BodyTrait< C<HashGrids,FD,BG,CR> >::getAABBSize() const
{
   real size  = aabb_[3] - aabb_[0];
   real yDiff = aabb_[4] - aabb_[1];
   real zDiff = aabb_[5] - aabb_[2];

   if( yDiff > size ) size = yDiff;
   if( zDiff > size ) size = zDiff;

   return size;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the grid the rigid body is currently assigned to.
 *
 * \return The grid the rigid body is currently assigned to.
 */
template< typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void* BodyTrait< C<HashGrids,FD,BG,CR> >::getGrid() const
{
   return grid_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current hash value of the rigid body.
 *
 * \return The current hash value of the rigid body.
 */
template< typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline size_t BodyTrait< C<HashGrids,FD,BG,CR> >::getHash() const
{
   return hash_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current body container index within the cell the body is currently assigned to.
 *
 * \return The current body container index.
 */
template< typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline size_t BodyTrait< C<HashGrids,FD,BG,CR> >::getCellId() const
{
   return cellId_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the grid the rigid body is associated with.
 *
 * \param grid The grid the rigid body is assigned to.
 * \return void
 */
template< typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void BodyTrait< C<HashGrids,FD,BG,CR> >::setGrid( void* grid )
{
   grid_ = grid;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the hash value of the rigid body.
 *
 * \param hash The new hash value of the rigid body.
 * \return void
 */
template< typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void BodyTrait< C<HashGrids,FD,BG,CR> >::setHash( size_t hash )
{
   hash_ = hash;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the body container index within the cell the body is currently assigned to.
 *
 * \param cell The new body container index.
 * \return void
 */
template< typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void BodyTrait< C<HashGrids,FD,BG,CR> >::setCellId( size_t cell )
{
   cellId_ = cell;
}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE SWEEP AND PRUNE ALGORITHM
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the BodyTrait class template for the 'sweep-and-prune' algorithm.
 * \ingroup coarse_collision_detection
 *
 * This specialization of the BodyTrait class template adapts rigid bodies to the sweep-and-prune
 * coarse collision detection algorithm.
 */
template< typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BodyTrait< C<SweepAndPrune,FD,BG,CR> >
{
public:
   //**Type definitions****************************************************************************
   typedef BoundingBox<IntervalEndpoint>  AABB;  //!< Type of the axis-aligned bounding box.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline BodyTrait( BodyID body );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   AABB aabb_;  //!< Axis-aligned bounding box for the rigid body
   //@}
   //**********************************************************************************************

private:
   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   template<typename> friend class SweepAndPrune;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the BodyTrait<SweepAndPrune> specialization.
 *
 * \param body The rigid body containing this bounding box.
 */
template< typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline BodyTrait< C<SweepAndPrune,FD,BG,CR> >::BodyTrait( BodyID body )
   : aabb_()  // Axis-aligned bounding box for the rigid body
{
   aabb_[0].type_ = IntervalEndpoint::start;
   aabb_[0].body_ = body;
   aabb_[1].type_ = IntervalEndpoint::start;
   aabb_[1].body_ = body;
   aabb_[2].type_ = IntervalEndpoint::start;
   aabb_[2].body_ = body;
   aabb_[3].type_ = IntervalEndpoint::end;
   aabb_[3].body_ = body;
   aabb_[4].type_ = IntervalEndpoint::end;
   aabb_[4].body_ = body;
   aabb_[5].type_ = IntervalEndpoint::end;
   aabb_[5].body_ = body;
}
//*************************************************************************************************

} // namespace coarse

} // namespace detection

} // namespace pe

#endif
