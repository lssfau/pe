//*************************************************************************************************
/*!
 *  \file pe/core/LinkContact.h
 *  \brief Header file for the LinkContact class.
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
//*************************************************************************************************

#ifndef _PE_CORE_LINKCONTACT_H_
#define _PE_CORE_LINKCONTACT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>


namespace pe {

//*************************************************************************************************
/*!\brief Contact for the link setup process.
 * \ingroup core
 *
 * The LinkContact class is used for the link setup process to determine the global position
 * and normal of a newly created link.
 */
class LinkContact
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   inline LinkContact( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal, real dist );
   inline LinkContact( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal,
                       const Vec3& e1, const Vec3& e2, real dist );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~LinkContact();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vec3& getPosition() const;
   inline const Vec3& getNormal()   const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 gpos_;         //!< The global position of the contact.
   Vec3 normal_;       //!< Normal of the contact.
                       /*!< The normal is defined within the global world frame and points
                            from body 2 to body 1. */
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief LinkContact constructor for a vertex/face contact.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 The second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \param dist The distance between the surfaces of the contacting rigid bodies.
 */
inline LinkContact::LinkContact( GeomID /*g1*/, GeomID /*g2*/, const Vec3& gpos, const Vec3& normal, real /*dist*/ )
   : gpos_(gpos)      // The global position of the contact
   , normal_(normal)  // Normal of the contact
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief LinkContact constructor for an edge/edge contact.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 The second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \param e1 Edge direction of the colliding edge of the first colliding rigid body.
 * \param e2 Edge direction of the colliding edge of the second colliding rigid body.
 * \param dist The distance between the surfaces of the contacting rigid bodies.
 */
inline LinkContact::LinkContact( GeomID /*g1*/, GeomID /*g2*/, const Vec3& gpos, const Vec3& normal,
                                 const Vec3& /*e1*/, const Vec3& /*e2*/, real /*dist*/ )
   : gpos_(gpos)      // The global position of the contact
   , normal_(normal)  // Normal of the contact
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the LinkContact class.
 */
inline LinkContact::~LinkContact()
{}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the global position of the contact.
 *
 * \return Global position of the contact.
 */
inline const Vec3& LinkContact::getPosition() const
{
   return gpos_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the global normal of the contact.
 *
 * \return Global normal of the contact.
 */
inline const Vec3& LinkContact::getNormal() const
{
   return normal_;
}
//*************************************************************************************************

} // namespace pe

#endif
