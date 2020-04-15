//=================================================================================================
/*!
 *  \file pe/core/attachable/Attachable.h
 *  \brief Header file for the Attachable class
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

#ifndef _PE_CORE_ATTACHABLE_ATTACHABLE_H_
#define _PE_CORE_ATTACHABLE_ATTACHABLE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/attachable/AttachableType.h>
#include <pe/core/Configuration.h>
#include <pe/core/response/AttachableTrait.h>
#include <pe/core/Types.h>
#include <pe/core/UniqueID.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Attachable interface class.
 * \ingroup core
 *
 * The Attachable class is the base class for the Attachable concept of the physics engine.
 * Classes deriving from this interface class (which are simply called Attachables) can be
 * attached to rigid bodies and are notified in case the rigid body is destroyed. This
 * concept is for example used for all force generators (see the ForceGenerator class
 * description).
 */
class Attachable : public AttachableTrait<Config>
{
private:
   //**Type definitions****************************************************************************
   typedef pe::AttachableTrait<Config>  Parent;  //!< The type of the parent class of the Attachable class.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Attachable( AttachableType type, id_t sid );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Attachable();
   //@}
   //**********************************************************************************************

public:
   //**MPI functions*******************************************************************************
   /*!\name MPI functions */
   //@{
   virtual bool isRemote() const = 0;
   //@}
   //**********************************************************************************************

private:
   //**Attachable setup functions******************************************************************
   /*! \cond PE_INTERNAL */
   friend void detach( AttachableID attachable );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  ATTACHABLE SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Attachable setup functions */
//@{
void detach( AttachableID attachable );
//@}
//*************************************************************************************************

} // namespace pe

#endif
