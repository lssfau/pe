//=================================================================================================
/*!
 *  \file pe/core/response/gravitytrait/Default.h
 *  \brief Header file for the default implementation of the GravityTrait class template
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

#ifndef _PE_CORE_RESPONSE_GRAVITYTRAIT_DEFAULT_H_
#define _PE_CORE_RESPONSE_GRAVITYTRAIT_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/attachable/GravityBase.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Gravity customization class for the collision response.
 * \ingroup core
 *
 * The GravityTrait class template is a customization class for the gravity force generator.
 * Its main purpose is the customization of the Gravity class for the selected collision
 * response algorithm (see pe::pe_CONSTRAINT_SOLVER).
 */
template< typename C >  // Type of the configuration
class GravityTrait : public GravityBase
{
private:
   //**Type definitions****************************************************************************
   typedef GravityBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit GravityTrait( id_t sid, BodyID body, const Vec3& gravity );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~GravityTrait() = 0;
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
/*!\brief Default implementation of the GravityTrait constructor.
 *
 * \param sid The unique system-specific ID of the gravity force generator.
 * \param body The rigid body the force generator is attached to.
 * \param gravity The exerted gravity.
 */
template< typename C >  // Type of the configuration
GravityTrait<C>::GravityTrait( id_t sid, BodyID body, const Vec3& gravity )
   : Parent( sid, body, gravity )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the GravityTrait destructor.
 */
template< typename C >  // Type of the configuration
GravityTrait<C>::~GravityTrait()
{}
//*************************************************************************************************

} // namespace pe

#endif
