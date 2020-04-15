//=================================================================================================
/*!
 *  \file pe/support/packing/Octree.h
 *  \brief Header file for the implementation of an octree.
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

#ifndef _PE_SUPPORT_PACKING_OCTREE_H_
#define _PE_SUPPORT_PACKING_OCTREE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/support/packing/OctreeNode.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
class PE_PUBLIC Octree
{
   std::auto_ptr<OctreeNode> root_;

public:

   Octree()
   {
   }

   Octree( const std::vector<ConstBodyID>& bodies, std::size_t depth )
   {
      SolidVolumeBoundingBox solidVolumeBoundingBox( Vec3( std::numeric_limits<real>::infinity(), std::numeric_limits<real>::infinity(), std::numeric_limits<real>::infinity() ), Vec3( -std::numeric_limits<real>::infinity(), -std::numeric_limits<real>::infinity(), -std::numeric_limits<real>::infinity() ) );

      for( std::vector<ConstBodyID>::const_iterator it = bodies.begin(); it != bodies.end(); ++it )
      {
         if( (*it)->isGlobal() )
            continue;

         solidVolumeBoundingBox.unify( (*it)->getAABB() );
      }

      if( !solidVolumeBoundingBox.empty() && depth > 0 )
      {
         root_.reset( new OctreeNode( solidVolumeBoundingBox, depth - 1 ) );
         root_->insert( bodies );
      }
   }

   ConstBodyID query( const Vec3& p ) const
   {
      if( root_.get() == 0 )
         return 0;

      return root_->query( p );
   }
};
//*************************************************************************************************

} // namespace pe

#endif
