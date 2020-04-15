//=================================================================================================
/*!
 *  \file pe/core/contact/ContactVector.h
 *  \brief Implementation of a vector for contact handles
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

#ifndef _PE_CORE_CONTACT_CONTACTVECTOR_H_
#define _PE_CORE_CONTACT_CONTACTVECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector for contact handles.
 * \ingroup core
 *
 * The ContactVector class is a container class for contact handles of various type (e.g.
 * pe::Contact, ...). It offers easy and fast access to the contained contacts:

   \code
   // Type definition for a Contact vector
   typedef pe::ContactVector<Contact>  Contacts;

   Contacts contacts;

   // Calculating the total number of contacts contained in the contact vector.
   Contacts::SizeType num = contacts.size();

   // Loop over all contacts contained in the contact vector.
   Contacts::Iterator begin = contacts.begin();
   Contacts::Iterator end   = contacts.end();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
template< typename C                    // Type of the contact
        , typename D = NoDelete         // Deletion policy
        , typename G = OptimalGrowth >  // Growth policy
class ContactVector : public PtrVector<C,D,G>
{
private:
   //**Type definitions****************************************************************************
   typedef PtrVector<C,D,G>  Base;  //!< Type of the base class.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef C                     Contact;           //!< Type of the contact
   typedef C*                    PointerType;       //!< Pointer to a non-const object.
   typedef const C*              ConstPointerType;  //!< Pointer to a const object.
   typedef size_t                SizeType;          //!< Size type of the pointer vector.
   typedef PtrIterator<C>        Iterator;          //!< Iterator over non-const objects.
   typedef PtrIterator<const C>  ConstIterator;     //!< Iterator over const objects.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline ContactVector( SizeType initCapacity = 100 );
   //@}
   //**********************************************************************************************

   //**Contact setup functions*********************************************************************
   /*!\name Contact setup functions */
   //@{
   inline void addVertexFaceContact( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal, real dist );
   inline void addEdgeEdgeContact( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal,
                                   const Vec3& e1, const Vec3& e2, real dist );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool isActive() const;
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
/*!\brief Standard constructor for ContactVector.
 *
 * \param initCapacity The initial capacity of the contact vector.
 */
template< typename C    // Type of the contact
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline ContactVector<C,D,G>::ContactVector( SizeType initCapacity )
   : Base( initCapacity )  // Base class initialization
{}
//*************************************************************************************************




//=================================================================================================
//
//  CONTACT SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a new vertex/face contact to the contact vector.
 *
 * \param g1 The first colliding geometric primitive.
 * \param g2 The second colliding geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The normal of the contact.
 * \param dist The distance between the surfaces of the two colliding rigid bodies.
 * \return void
 *
 * Creating a vertex/face contact between the superordinate bodies of the (potentially)
 * subordinate bodies \a b1 and \a b2 and adding it to the contact vector.
 */
template< typename C    // Type of the contact
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline void ContactVector<C,D,G>::addVertexFaceContact( GeomID g1, GeomID g2, const Vec3& gpos,
                                                        const Vec3& normal, real dist )
{
   // Creating a new vertex/face contact
   Base::pushBack(
      new Contact(
         g1,      // First colliding geometric primitive
         g2,      // Second colliding geometric primitive
         gpos,    // Global position of the contact
         normal,  // Normal of the contact (from body 2 to body 1)
         dist     // Distance between the surfaces (negative distance means penetration)
      )
   );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adding a new edge/edge contact to the contact vector.
 *
 * \param g1 The first colliding geometric primitive.
 * \param g2 The second colliding geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The normal of the contact.
 * \param e1 Edge direction of the colliding edge of the first colliding rigid body.
 * \param e2 Edge direction of the colliding edge of the second colliding rigid body.
 * \param dist The distance between the surfaces of the two colliding rigid bodies.
 * \return void
 *
 * Creating an edge/edge contact between the superordinate bodies of the (potentially)
 * subordinate bodies \a b1 and \a b2 and adding it to the contact vector.
 */
template< typename C    // Type of the contact
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline void ContactVector<C,D,G>::addEdgeEdgeContact( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal,
                                                      const Vec3& e1, const Vec3& e2, real dist )
{
   // Creating a new edge/edge contact
   Base::pushBack(
      new Contact(
         g1,      // First colliding geometric primitive
         g2,      // Second colliding geometric primitive
         gpos,    // Global position of the contact
         normal,  // Normal of the contact (from body 2 to body 1)
         e1,      // Edge direction of the colliding edge of body 1
         e2,      // Edge direction of the colliding edge of body 2
         dist     // Distance between the surfaces (negative distance means penetration)
      )
   );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the contact vector contains an active contact.
 *
 * \return \a true if at least one contact is active, \a false if all contacts are inactive. 
 *
 * The function returns whether the contact vector contains at least a single active contact or
 * if all the contained contacts are inactive. A contact is considered to be active if at least
 * one of the attached rigid bodies is active.
 */
template< typename C    // Type of the contact
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline bool ContactVector<C,D,G>::isActive() const
{
   for( ConstIterator c=this->begin(); c!=this->end(); ++c ) {
      if( c->isActive() ) return true;
   }
   return false;
}
//*************************************************************************************************

} // namespace pe

#endif
