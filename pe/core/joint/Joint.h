//=================================================================================================
/*!
 *  \file pe/core/joint/Joint.h
 *  \brief Header file for the Joint class
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

#ifndef _PE_CORE_JOINT_JOINT_H_
#define _PE_CORE_JOINT_JOINT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/Configuration.h>
#include <pe/core/joint/JointTrait.h>
#include <pe/core/joint/JointType.h>
#include <pe/core/Types.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup joints Joints
 * \ingroup core
 */
/*!\brief Joint between two connected primitives.
 * \ingroup joints
 *
 * The Joint class is the base class for all types of joints between rigid bodies in the
 * simulation system.
 */
class Joint : public JointTrait<Config>
{
private:
   //**Type definitions****************************************************************************
   typedef JointTrait<Config>  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Joint( JointType type, size_t rows,  BodyID body1, BodyID body2, real scale, id_t sid );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Joint();
   //@}
   //**********************************************************************************************

private:
   //**Joint setup functions***********************************************************************
   /*! \cond PE_INTERNAL */
   friend void detach( JointID joint );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Joint class.
 */
inline Joint::~Joint()
{}
//*************************************************************************************************




//=================================================================================================
//
//  JOINT SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Joint setup functions */
//@{
void detach( JointID joint );
//@}
//*************************************************************************************************

} // namespace pe

#endif
