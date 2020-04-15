//*************************************************************************************************
/*!
 *  \file pe/core/contact/Contact.h
 *  \brief Header file for the Contact class
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

#ifndef _PE_CORE_CONTACT_CONTACT_H_
#define _PE_CORE_CONTACT_CONTACT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/Configuration.h>
#include <pe/core/contact/ContactTrait.h>
#include <pe/core/rigidbody/GeomPrimitive.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/system/VerboseMode.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/MemoryPool.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup contact Contact
 * \ingroup core
 */
/*!\brief Contact between rigid bodies.
 * \ingroup contact
 *
 * The Contact class is the base class for all types of contacts between rigid bodies in the
 * simulation system. Contacts between rigid bodies are classified depending on the relative
 * velocity between the touching rigid bodies:
 *
 *  - \f$ v_{rel} \f$ > 0: separating contact
 *  - \f$ v_{rel} \f$ = 0: resting contact
 *  - \f$ v_{rel} \f$ < 0: colliding contact\n
 *
 * (in order to classify the contact, pe::collisionThreshold is used as tolerance level).
 */
class Contact : public ContactTrait<Config>
{
private:
   //**Type definitions****************************************************************************
   //!< The type of the parent class.
   typedef pe::ContactTrait<Config>  Parent;

   //! Contact Pool for contacts.
   typedef MemoryPool<Contact, 10000>  ContactPool;
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Contact( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal, real dist );
   explicit Contact( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal,
                     const Vec3& e1, const Vec3& e2, real dist );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~Contact();
   //@}
   //**********************************************************************************************

   //**Memory management functions*****************************************************************
   /*!\name Memory management functions */
   //@{
   static inline void* operator new   ( size_t size );
   static inline void  operator delete( void* rawMemory );
   //@}
   //**********************************************************************************************

   //**Impulse and force functions*****************************************************************
   /*!\name Impulse and force functions */
   //@{
   inline void applyForce  ( const Vec3& f ) const;
   inline void applyImpulse( const Vec3& j ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static ContactPool contactPool_;  //!< Memory pool for contacts.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Contact class.
 */
inline Contact::~Contact()
{}
//*************************************************************************************************




//=================================================================================================
//
//  MEMORY MANAGEMENT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Dynamic allocation of a single Contact object.
 *
 * \param size (Implicit) size argument of the new operator.
 * \return Pointer to the raw memory.
 */
inline void* Contact::operator new( size_t /*size*/ )
{
   return contactPool_.malloc();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Dynamic deallocation of a single Contact object.
 *
 * \param rawMemory Pointer to the raw memory.
 * \return void
 */
inline void Contact::operator delete( void* rawMemory )
{
   contactPool_.free( rawMemory );
}
//*************************************************************************************************




//=================================================================================================
//
//  IMPULSE AND FORCE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Force transfer to both contacting rigid bodies.
 *
 * \param f The acting force on the first contacting rigid body.
 * \return void
 */
inline void Contact::applyForce( const Vec3& f ) const
{
   b1_->addForceAtPos(  f, gpos_ );
   b2_->addForceAtPos( -f, gpos_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Impulse transfer to both contacting rigid bodies.
 *
 * \param j The acting impulse on the first contacting rigid body.
 * \return void
 */
inline void Contact::applyImpulse( const Vec3& j ) const
{
   b1_->addImpulseAtPos(  j, gpos_ );
   b2_->addImpulseAtPos( -j, gpos_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for contacts.
 * \ingroup contact
 *
 * \param os Reference to the output stream.
 * \param c Reference to a constant contact object.
 * \return Reference to the output stream.
 */
inline std::ostream& operator<<( std::ostream& os, const Contact& c )
{
   const char* tab="";
   os << "--" << pe_MAGENTA << "CONTACT PARAMETERS" << pe_OLDCOLOR
      << "---------------------------------------------------------\n";
   using std::setw;

   os << tab << " Contact " << c.getID() << " at position " << c.getPosition() << "\n";

   os << tab << "   Body 1            = " << c.getBody1()->getSystemID() << "\n"
      << tab << "   Body 2            = " << c.getBody2()->getSystemID() << "\n"
      << tab << "   Normal            = " << c.getNormal() << "\n"
      << tab << "   Distance          = " << c.getDistance() << "\n"
      << tab << "   Locality          = " << ( c.getBody1()->isRemote() ? "remote" : "local") << "-" << ( c.getBody2()->isRemote() ? "remote" : "local") << " contact\n";

   if( verboseMode )
   {
      os << tab << "   Restitution       = " << c.getRestitution() << "\n"
         << tab << "   Stiffness         = " << c.getStiffness() << "\n"
         << tab << "   Damping_n         = " << c.getDampingN() << "\n"
         << tab << "   Damping_t         = " << c.getDampingT() << "\n"
         << tab << "   Friction coeff.   = " << c.getFriction() << "\n"
         << tab << "   Contact type      = " << (c.getType() == colliding ? "colliding" : (c.getType() == resting ? "resting" : "separating")) << "\n"
         << tab << "   Rel. normal vel.  = " << c.getNormalRelVel() << "\n";
   }
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for contact handles.
 * \ingroup contact
 *
 * \param os Reference to the output stream.
 * \param c Constant contact handle.
 * \return Reference to the output stream.
 */
inline std::ostream& operator<<( std::ostream& os, ConstContactID c )
{
   return operator<<( os, *c );
}
//*************************************************************************************************

} // namespace pe

#endif
