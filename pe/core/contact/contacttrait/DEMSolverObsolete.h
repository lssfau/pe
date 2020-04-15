//=================================================================================================
/*!
 *  \file pe/core/contact/contacttrait/DEMSolverObsolete.h
 *  \brief Specialization of the ContactTrait class template for the old Discrete Element solver
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

#ifndef _PE_CORE_CONTACT_CONTACTTRAIT_DEMSOLVEROBSOLETE_H_
#define _PE_CORE_CONTACT_CONTACTTRAIT_DEMSOLVEROBSOLETE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/rigidbody/BodyCast.h>
#include <pe/core/contact/ContactBase.h>
#include <pe/core/contact/contacttrait/Default.h>
#include <pe/core/Materials.h>
#include <pe/core/response/Types.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/Types.h>
#include <pe/math/MatrixMxN.h>
#include <pe/math/shims/Square.h>
#include <pe/math/Vector3.h>
#include <pe/math/VectorN.h>
#include <pe/system/ConstraintConfig.h>
#include <pe/system/LCPConfig.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  SPECIALIZATION FOR THE DISCRETE ELEMENT SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Contact customization class for the collision response.
 * \ingroup core
 *
 * The ContactTrait class template is a customization class for contacts in general.
 * Its main purpose is the customization of the Contact class for the selected collision
 * response algorithm (see pe::pe_CONSTRAINT_SOLVER).\n
 * Depending on the used algorithm, a contact may require additional data or functionality
 * to efficiently support the collision response calculations. In order to add this specific
 * functionality or data and to adapt contacts to a particular algorithm, the base template
 * needs to be specialized.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class ContactTrait< C<CD,FD,BG,response::DEMSolverObsolete> > : public ContactBase
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

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline bool hasEffectiveRadius      () const;
   inline real getEffectiveRadius      () const;
   inline real getEffectiveYoungModulus() const;
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   real radius_;  //!< The effective radius of the contact.
   real young_;   //!< The effective Young's modulus of the contact.
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
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline ContactTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos,
                                                                      const Vec3& normal, real dist )
   : ContactBase( g1, g2, gpos, normal, dist )  // Initialization of the ContactBase base object
   , radius_    ( 0 )                           // The effective radius of the contact
   , young_     ( 0 )                           // The effective Young's modulus of the contact
{
   // Calculating the effective Young's modulus
   young_ = Material::getYoungModulus( g1->getMaterial(), g2->getMaterial() );

   // Calculating the effective radius in case of a sphere/sphere collision
   if( g1->getType() == sphereType && g2->getType() == sphereType )
   {
      const real r1( static_body_cast<Sphere>( g1 )->getRadius() );
      const real r2( static_body_cast<Sphere>( g2 )->getRadius() );

      radius_ = (r1*r2) / (r1+r2);
   }
}
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
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline ContactTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal,
                                                                      const Vec3& e1, const Vec3& e2, real dist )
   : ContactBase( g1, g2, gpos, normal, e1, e2, dist )  // Initialization of the ContactBase base object
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the ContactTrait destructor.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline ContactTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::~ContactTrait()
{}
//*************************************************************************************************


//*************************************************************************************************
// TODO
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline bool ContactTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::hasEffectiveRadius() const
{
   return ( radius_ != real(0) );
}
//*************************************************************************************************


//*************************************************************************************************
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline real ContactTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::getEffectiveRadius() const
{
   return radius_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the effective Young's modulus of the contact.
 *
 * \return The effective Young's modulus of the contact.
 *
 * This function returns the effective Young's modulus of the contact, which is calculated
 * according to the material properties of the two colliding rigid bodies:

      \f[ \frac{1}{E_{eff}} = \frac{1 - \nu_1^2}{E_1} + \frac{1 - \nu_2^2}{E_2}, \f]

 * where \f$ E \f$ are the according Young's moduli and \f$ \nu \f$ are the according Poisson's
 * ratios of the two bodies.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline real ContactTrait< C<CD,FD,BG,response::DEMSolverObsolete> >::getEffectiveYoungModulus() const
{
   return young_;
}
//*************************************************************************************************

} // namespace pe

#endif
