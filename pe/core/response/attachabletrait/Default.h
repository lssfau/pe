//=================================================================================================
/*!
 *  \file pe/core/response/attachabletrait/Default.h
 *  \brief Header file for the default implementation of the AttachableTrait class template
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

#ifndef _PE_CORE_RESPONSE_ATTACHABLETRAIT_DEFAULT_H_
#define _PE_CORE_RESPONSE_ATTACHABLETRAIT_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/attachable/AttachableBase.h>
#include <pe/core/attachable/AttachableType.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Attachable customization class for the collision response.
 * \ingroup core
 *
 * The AttachableTrait class template is a customization class for attachables in general.
 * Its main purpose is the customization of the Attachable class for the selected collision
 * response algorithm (see pe::pe_CONSTRAINT_SOLVER).
 */
template< typename C >  // Type of the configuration
class AttachableTrait : public AttachableBase
{
private:
   //**Type definitions****************************************************************************
   typedef AttachableBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit AttachableTrait( AttachableType type, id_t sid );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~AttachableTrait();
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the AttachableTrait constructor.
 *
 * \param type The type of the attachable.
 * \param sid The unique system-specific ID of the attachable.
 */
template< typename C >  // Type of the configuration
AttachableTrait<C>::AttachableTrait( AttachableType type, id_t sid )
   : Parent( type, sid )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the AttachableTrait destructor.
 */
template< typename C >  // Type of the configuration
AttachableTrait<C>::~AttachableTrait()
{}
//*************************************************************************************************

} // namespace pe

#endif
