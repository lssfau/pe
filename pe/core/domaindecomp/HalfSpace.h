//=================================================================================================
/*!
 *  \file pe/core/domaindecomp/HalfSpace.h
 *  \brief Header file for the HalfSpace class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#ifndef _PE_CORE_DOMAINDECOMP_HALFSPACE_H_
#define _PE_CORE_DOMAINDECOMP_HALFSPACE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/core/domaindecomp/ProcessGeometry.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a half space for specifying subdomains.
 * \ingroup domaindecomp
 *
 * This class represents a half space occupied by an MPI process. It is defined by an
 * infinite plane that divides the global space into two separate half spaces. This plane is
 * characterized by the following equation:
 *
 *                                \f[ ax + by + cz = d , \f]
 *
 * where \a a, \a b and \a c are the x, y and z component of the normal vector. The normal
 * vector of the plane is a normalized vector that is defined to point towards the half space
 * of the MPI process. \a d is the distance/displacement from the origin of the global
 * world frame to the plane. A positive value of \a d indicates that the origin of the global
 * world frame is outside the half space, whereas a negative value of \a d indicates that the
 * origin is inside the half space. A value of 0 therefore indicates that the origin is on the
 * surface of the plane:
 *
 *  - > 0: The global origin is outside the half space\n
 *  - < 0: The global origin is inside the half space\n
 *  - = 0: The global origin is on the surface of the separating plane
 *
 * In the following example the remote process subdomain is specified by a single half space. The
 * normal is pointing inside the subdomain of the remote process.
 * \image html process.png
 * \image latex process.eps "Half space process geometry" width=520pt
 */
class PE_PUBLIC HalfSpace : public ProcessGeometry
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit HalfSpace( real a, real b, real c, real d, real dx=0 );
   explicit HalfSpace( const Vec3& normal, real d, real dx=0 );
   explicit HalfSpace( real a, real b, real c, real x, real y, real z, real dx=0 );
   explicit HalfSpace( const Vec3& normal, const Vec3& gpos, real dx=0 );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vec3& getNormal()       const;
   inline real        getDisplacement() const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool intersectsWith( ConstBodyID         b   ) const;
   virtual bool intersectsWith( ConstSphereID       s   ) const;
   virtual bool intersectsWith( ConstBoxID          b   ) const;
   virtual bool intersectsWith( ConstCapsuleID      c   ) const;
   virtual bool intersectsWith( ConstCylinderID     c   ) const;
   virtual bool intersectsWith( ConstTriangleMeshID obj ) const;
   virtual bool intersectsWith( ConstUnionID        u   ) const;
   virtual bool containsPoint        ( const Vec3& gpos ) const;
   virtual bool containsPointStrictly( const Vec3& gpos ) const;
   //@}
   //**********************************************************************************************

   //**Debugging functions***************************************************************************
   /*!\name Debugging functions */
   //@{
   virtual void extractHalfSpaces( std::list< std::pair<Vec3, real> >& halfspaces ) const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os                  ) const;
   void print( std::ostream& os, const char* tab ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 normal_;  //!< Normal of the boundary plane of the half space.
                  /*!< This is the normal of the boundary plane that defines the half space.
                       It is stored in reference to the global world frame and is always
                       pointing inside the half space. */
   real d_;       //!< Displacement from the origin of the global world frame.
                  /*!< The displacement can be categorized in the following way:\n
                        - > 0: The global origin is outside the half space\n
                        - < 0: The global origin is inside the half space\n
                        - = 0: The global origin is on the surface of the half space */
   real dx_;      //!< The size of the overlap between this and the remote MPI process.
                  /*!< The overlap is always larger or equal to the contactThreshold, i.e. in
                       the range \f$ [contactThreshold..\infty] \f$. */
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the normal of the half space boundary in reference to the global world frame.
 *
 * \return The normal of the half space boundary.
 */
inline const Vec3& HalfSpace::getNormal() const
{
   return normal_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the displacement/distance from the origin of the global world frame.
 *
 * \return The displacement of the half space boundary.
 *
 * A positive displacement value indicates that the origin of the global world frame is
 * not contained in the half space, whereas a negative value indicates that the origin is
 * contained in the half space.
 */
inline real HalfSpace::getDisplacement() const
{
   return d_;
}
//*************************************************************************************************

} // namespace pe

#endif
