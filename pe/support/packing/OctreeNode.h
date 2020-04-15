//=================================================================================================
/*!
 *  \file pe/support/packing/OctreeNode.h
 *  \brief Header file for the implementation of an octree node.
 *
 *  Copyright (C) 2016 Tobias Preclik
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

#ifndef _PE_SUPPORT_PACKING_OCTREENODE_H_
#define _PE_SUPPORT_PACKING_OCTREENODE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/RigidBody.h>
#include <pe/math/Vector3.h>
#include <pe/support/packing/SolidVolumeBoundingBox.h>
#include <memory>
#include <vector>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
class PE_PRIVATE OctreeNode
{
   SolidVolumeBoundingBox regionBB_;
   std::vector<ConstBodyID> bodies_;
   std::auto_ptr<OctreeNode> children_[8];
   std::size_t depth_;
   bool leaf_;
   const real fraction_;
   const std::size_t splitThreshold_;

public:

   OctreeNode( const SolidVolumeBoundingBox& regionBB, std::size_t depth ) : regionBB_( regionBB ), depth_( depth ), leaf_( true ), fraction_( 0.75 ), splitThreshold_( 8 )
   {
   }

   void insert( const std::vector<ConstBodyID>& bodies )
   {
      for( std::vector<ConstBodyID>::const_iterator it = bodies.begin(); it != bodies.end(); ++it )
      {
         insert( *it );
      }
   }

   std::size_t insert( ConstBodyID body )
   {
      if( body->isGlobal() )
         return 0;

      SolidVolumeBoundingBox bodyBB = SolidVolumeBoundingBox( body->getAABB() ).intersected( regionBB_ );
      if( bodyBB.empty() )
         return 0;

      if( bodyBB.volume() > fraction_ * regionBB_.volume() || ( leaf_ && bodies_.size() < splitThreshold_ ) || depth_ == 0 )
      {
         bodies_.push_back( body );
         return 1;
      }

      split();

      // Perform requested insertion into children nodes.
      std::size_t n = 0;
      for( std::size_t i = 0; i < 8; ++i )
         n += children_[i]->insert( body );

      return n;
   }

   ConstBodyID query( const Vec3& p ) const
   {
      if( !regionBB_.contains( p ) )
         return 0;

      for( std::vector<ConstBodyID>::const_iterator it = bodies_.begin(); it != bodies_.end(); ++it )
      {
         if( (*it)->getAABB().contains( p ) && (*it)->containsPoint( p ) )
            return *it;
      }

      if( leaf_ )
         return 0;

      for( std::size_t i = 0; i < 8; ++i )
      {
         ConstBodyID ret = children_[i]->query( p );
         if( ret )
            return ret;
      }

      return 0;
   }

   std::size_t size() const
   {
      if( leaf_ )
         return bodies_.size();

      std::size_t n = bodies_.size();
      for( std::size_t i = 0; i < 8; ++i )
         n += children_[i]->size();

      return n;
   }

   std::size_t size( std::size_t level ) const
   {
      if( depth_ == level )
         return bodies_.size();

      if( leaf_ )
         return 0;

      std::size_t n = 0;
      for( std::size_t i = 0; i < 8; ++i )
         n += children_[i]->size( level );

      return n;
   }

private:

   void split()
   {
      if( !leaf_ )
         return;

      // Split node.
      Vec3 midpoint( 0.5 * regionBB_.lbound() + 0.5 * regionBB_.ubound() );

      children_[0].reset( new OctreeNode( SolidVolumeBoundingBox( Vec3( regionBB_.lbound()[0], regionBB_.lbound()[1], regionBB_.lbound()[2] ), Vec3( midpoint[0],           midpoint[1],           midpoint[2]           ) ), depth_ - 1 ) );
      children_[1].reset( new OctreeNode( SolidVolumeBoundingBox( Vec3( midpoint[0],           regionBB_.lbound()[1], regionBB_.lbound()[2] ), Vec3( regionBB_.ubound()[0], midpoint[1],           midpoint[2]           ) ), depth_ - 1 ) );
      children_[2].reset( new OctreeNode( SolidVolumeBoundingBox( Vec3( regionBB_.lbound()[0], midpoint[1],           regionBB_.lbound()[2] ), Vec3( midpoint[0],           regionBB_.ubound()[1], midpoint[2]           ) ), depth_ - 1 ) );
      children_[3].reset( new OctreeNode( SolidVolumeBoundingBox( Vec3( midpoint[0],           midpoint[1],           regionBB_.lbound()[2] ), Vec3( regionBB_.ubound()[0], regionBB_.ubound()[1], midpoint[2]           ) ), depth_ - 1 ) );
      children_[4].reset( new OctreeNode( SolidVolumeBoundingBox( Vec3( regionBB_.lbound()[0], regionBB_.lbound()[1], midpoint[2]           ), Vec3( midpoint[0],           midpoint[1],           regionBB_.ubound()[2] ) ), depth_ - 1 ) );
      children_[5].reset( new OctreeNode( SolidVolumeBoundingBox( Vec3( midpoint[0],           regionBB_.lbound()[1], midpoint[2]           ), Vec3( regionBB_.ubound()[0], midpoint[1],           regionBB_.ubound()[2] ) ), depth_ - 1 ) );
      children_[6].reset( new OctreeNode( SolidVolumeBoundingBox( Vec3( regionBB_.lbound()[0], midpoint[1],           midpoint[2]           ), Vec3( midpoint[0],           regionBB_.ubound()[1], regionBB_.ubound()[2] ) ), depth_ - 1 ) );
      children_[7].reset( new OctreeNode( SolidVolumeBoundingBox( Vec3( midpoint[0],           midpoint[1],           midpoint[2]           ), Vec3( regionBB_.ubound()[0], regionBB_.ubound()[1], regionBB_.ubound()[2] ) ), depth_ - 1 ) );

      // Move bodies into leaves.
      std::vector<ConstBodyID> bodiesStaying;
      real V = regionBB_.volume();
      for( std::vector<ConstBodyID>::iterator it = bodies_.begin(); it != bodies_.end(); ++it )
      {
         if( SolidVolumeBoundingBox( (*it)->getAABB() ).intersected( regionBB_ ).volume() > fraction_ * V )
         {
            bodiesStaying.push_back( *it );
            continue;
         }

         for( std::size_t i = 0; i < 8; ++i )
         {
            children_[i]->insert( *it );
         }
      }

      bodies_ = bodiesStaying;
      leaf_ = false;
   }
};
//*************************************************************************************************

} // namespace pe

#endif
