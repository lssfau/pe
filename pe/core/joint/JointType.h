//=================================================================================================
/*!
 *  \file pe/core/joint/JointType.h
 *  \brief Header file for the rigid body joint types
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

#ifndef _PE_CORE_JOINT_JOINTTYPE_H_
#define _PE_CORE_JOINT_JOINTTYPE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>


namespace pe {

//=================================================================================================
//
//  RIGID BODY JOINT TYPES
//
//=================================================================================================

//*************************************************************************************************
//! Joint types of the rigid bodies.
enum JointType {
   fixedType  = 0,  //!< Code for fixed joints.
   sliderType = 1,  //!< Code for slider joints.
   hingeType  = 2,  //!< Code for hinge joints.
   ballType   = 3   //!< Code for ball-in-socket joints.
};
//*************************************************************************************************




//=================================================================================================
//
//  JOINT TYPE UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Joint type utility functions */
//@{
std::ostream& operator<<( std::ostream& os, JointType type );
//@}
//*************************************************************************************************

} // namespace pe

#endif
