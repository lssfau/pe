//=================================================================================================
/*!
 *  \file pe/core/contact/contacttrait/Default.h
 *  \brief Header file for the default implementation of the ContactTrait class template.
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

#ifndef _PE_CORE_CONTACT_CONTACTTRAIT_DEFAULT_H_
#define _PE_CORE_CONTACT_CONTACTTRAIT_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/contact/ContactBase.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Contact customization class for the collision response.
 * \ingroup contact
 *
 * The ContactTrait class template is a customization class for contacts in general.
 * Its main purpose is the customization of the Contact class for the selected collision
 * response algorithm (see pe::pe_CONSTRAINT_SOLVER).\n
 * Depending on the used algorithm, a contact may require additional data or functionality
 * to efficiently support the collision response calculations. In order to add this specific
 * functionality or data and to adapt contacts to a particular algorithm, the base template
 * needs to be specialized.
 */
template< typename C >  // Type of the configuration
class ContactTrait : public ContactBase
{
protected:
   //**Type definitions****************************************************************************
   typedef ContactBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   inline ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal, real dist );
   inline ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal,
                        const Vec3& e1, const Vec3& e2, real dist );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~ContactTrait();
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the ContactTrait constructor for vertex-face contacts.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 The second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \param dist The distance between the surfaces of the contacting rigid bodies.
 * \return void
 *
 * Default implementation of the ContactTrait constructor for vertex-face contacts.
 */
template< typename C >  // Type of the configuration
inline ContactTrait<C>::ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal, real dist )
   : ContactBase( g1, g2, gpos, normal, dist )  // Initialization of the ContactBase base object
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the ContactTrait constructor for edge/edge contacts.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 The second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \param e1 Edge direction of the colliding edge of the first colliding rigid body.
 * \param e2 Edge direction of the colliding edge of the second colliding rigid body.
 * \param dist The distance between the surfaces of the contacting rigid bodies.
 */
template< typename C >  // Type of the configuration
inline ContactTrait<C>::ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal,
                                      const Vec3& e1, const Vec3& e2, real dist )
   : ContactBase( g1, g2, gpos, normal, e1, e2, dist )  // Initialization of the ContactBase base object
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the ContactTrait destructor.
 */
template< typename C >  // Type of the configuration
inline ContactTrait<C>::~ContactTrait()
{}
//*************************************************************************************************

} // namespace pe

#endif
