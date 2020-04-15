//=================================================================================================
/*!
 *  \file pe/support/packing/SolidVolumeBoundingBox.h
 *  \brief Header file for the implementation of a solid volume bounding box.
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

#ifndef _PE_SUPPORT_PACKING_SOLIDVOLUMEBOUNDINGBOX_H_
#define _PE_SUPPORT_PACKING_SOLIDVOLUMEBOUNDINGBOX_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/Vector3.h>
#include <pe/core/detection/coarse/BoundingBox.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
class PE_PUBLIC SolidVolumeBoundingBox {
   Vec3 lb_, ub_;

public:
   SolidVolumeBoundingBox( const Vec3& lb, const Vec3& ub ) : lb_(lb), ub_(ub)
   {
   }

   SolidVolumeBoundingBox( const pe::detection::coarse::BoundingBox<real>& bb ) : lb_( Vec3( bb[0], bb[1], bb[2] ) ), ub_( Vec3( bb[3], bb[4], bb[5] ) )
   {
   }

   bool empty() const
   {
      return (lb_[0] >= ub_[0] || lb_[1] >= ub_[1] || lb_[2] >= ub_[2]);
   }

   const Vec3& lbound() const
   {
      return lb_;
   }

   const Vec3& ubound() const
   {
      return ub_;
   }

   void expand( real margin )
   {
      lb_[0] -= margin;
      lb_[1] -= margin;
      lb_[2] -= margin;
      ub_[0] += margin;
      ub_[1] += margin;
      ub_[2] += margin;
   }

   void intersect( const SolidVolumeBoundingBox& sv )
   {
      lb_[0] = std::max(lb_[0], sv.lb_[0]);
      lb_[1] = std::max(lb_[1], sv.lb_[1]);
      lb_[2] = std::max(lb_[2], sv.lb_[2]);

      ub_[0] = std::min(ub_[0], sv.ub_[0]);
      ub_[1] = std::min(ub_[1], sv.ub_[1]);
      ub_[2] = std::min(ub_[2], sv.ub_[2]);
   }

   SolidVolumeBoundingBox intersected( const SolidVolumeBoundingBox& sv ) const
   {
      return SolidVolumeBoundingBox( Vec3( std::max(lb_[0], sv.lb_[0]), std::max(lb_[1], sv.lb_[1]), std::max(lb_[2], sv.lb_[2]) ), Vec3( std::min(ub_[0], sv.ub_[0]), std::min(ub_[1], sv.ub_[1]), std::min(ub_[2], sv.ub_[2]) ) );
   }

   real volume() const
   {
      if (empty())
         return 0.0;

      return (ub_[0] - lb_[0]) * (ub_[1] - lb_[1]) * (ub_[2] - lb_[2]);
   }

   void unify( const SolidVolumeBoundingBox& sv )
   {
      lb_[0] = std::min(lb_[0], sv.lb_[0]);
      lb_[1] = std::min(lb_[1], sv.lb_[1]);
      lb_[2] = std::min(lb_[2], sv.lb_[2]);

      ub_[0] = std::max(ub_[0], sv.ub_[0]);
      ub_[1] = std::max(ub_[1], sv.ub_[1]);
      ub_[2] = std::max(ub_[2], sv.ub_[2]);
   }

   bool contains( const Vec3& p ) const
   {
      return p[0] >= lb_[0] && p[0] <= ub_[0] && p[1] >= lb_[1] && p[1] <= ub_[1] && p[2] >= lb_[2] && p[2] <= ub_[2];
   }
};
//*************************************************************************************************


//*************************************************************************************************
inline std::ostream& operator<<( std::ostream& out, const SolidVolumeBoundingBox& solidVolumeBoundingBox )
{
   out << "[" << solidVolumeBoundingBox.lbound()[0] << "; " << solidVolumeBoundingBox.ubound()[0] << "] x [" << solidVolumeBoundingBox.lbound()[1] << "; " << solidVolumeBoundingBox.ubound()[1] << "] x [" << solidVolumeBoundingBox.lbound()[2] << "; " << solidVolumeBoundingBox.ubound()[2] << "]";
   return out;
}
//*************************************************************************************************

} // namespace pe

#endif
