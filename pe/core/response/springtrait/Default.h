//=================================================================================================
/*!
 *  \file pe/core/response/springtrait/Default.h
 *  \brief Header file for the default implementation of the SpringTrait class template
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

#ifndef _PE_CORE_RESPONSE_SPRINGTRAIT_DEFAULT_H_
#define _PE_CORE_RESPONSE_SPRINGTRAIT_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/attachable/SpringBase.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Spring customization class for the collision response.
 * \ingroup core
 *
 * The SpringTrait class template is a customization class for the spring force generator.
 * Its main purpose is the customization of the Spring class for the selected collision
 * response algorithm (see pe::pe_CONSTRAINT_SOLVER).
 */
template< typename C >  // Type of the configuration
class SpringTrait : public SpringBase
{
private:
   //**Type definitions****************************************************************************
   typedef SpringBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit SpringTrait( id_t sid, BodyID body1, BodyID body2, const Vec3& anchor1, const Vec3& anchor2,
                         real stiffness, real damping, real length, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~SpringTrait() = 0;
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
/*!\brief Default implementation of the SpringTrait constructor.
 *
 * \param sid The unique system-specific ID of the spring.
 * \param body1 The first rigid body anchored to the spring.
 * \param body2 The second rigid body anchored to the spring.
 * \param anchor1 The first body's anchor point in body relative coordinates.
 * \param anchor2 The second body's anchor point in body relative coordinates.
 * \param stiffness The stiffness of the spring \f$ (0..\infty) \f$.
 * \param damping The damping of the spring \f$ [0..\infty) \f$.
 * \param length The rest length of the spring \f$ (0..\infty) \f$.
 * \param visible Specifies if the spring is visible in a visualization.
 */
template< typename C >  // Type of the configuration
SpringTrait<C>::SpringTrait( id_t sid, BodyID body1, BodyID body2, const Vec3& anchor1, const Vec3& anchor2,
                             real stiffness, real damping, real length, bool visible )
   : Parent( sid, body1, body2, anchor1, anchor2, stiffness, damping, length, visible )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the SpringTrait destructor.
 */
template< typename C >  // Type of the configuration
SpringTrait<C>::~SpringTrait()
{}
//*************************************************************************************************

} // namespace pe

#endif
