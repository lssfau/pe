//=================================================================================================
/*!
 *  \file pe/core/rigidbody/CapsuleBase.h
 *  \brief Base class for the capsule geometry
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2013-2014 Tobias Scharpff
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

#ifndef _PE_CORE_RIGIDBODY_CAPSULEBASE_H_
#define _PE_CORE_RIGIDBODY_CAPSULEBASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/GeomPrimitive.h>
#include <pe/core/Types.h>
#include <pe/math/Constants.h>
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
/*!\brief Base class for the capsule geometry.
 * \ingroup capsule
 *
 * The CapsuleBase class represents the base class for the capsule geometry. It provides
 * the basic functionality of a capsule. For a full description of the capsule geometry,
 * see the Capsule class description.
 */
class CapsuleBase : public GeomPrimitive
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit CapsuleBase( id_t sid, id_t uid, const Vec3& gpos, real radius,
                         real length, MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~CapsuleBase() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline real getRadius() const;
   inline real getLength() const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline virtual Vec3 support( const Vec3& d ) const;
   inline virtual Vec3 supportContactThreshold( const Vec3& d ) const;
   //@}
   //**********************************************************************************************

protected:
   //**Volume, mass and density functions**********************************************************
   /*!\name Volume, mass and density functions */
   //@{
   static inline real calcVolume( real radius, real length );
   static inline real calcMass( real radius, real length, real density );
   static inline real calcDensity( real radius, real length, real mass );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual void calcBoundingBox();  // Calculation of the axis-aligned bounding box
   void calcInertia();      // Calculation of the moment of inertia
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   real radius_;  //!< The radius of the cylinder part and the caps on both ends of the cylinder.
   real length_;  //!< The length of the cylinder part.
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
/*!\brief Returns the radius of the capsule.
 *
 * \return The radius of the capsule.
 */
inline real CapsuleBase::getRadius() const
{
   return radius_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the length of the cylinder part.
 *
 * \return The length of the cylinder part.
 */
inline real CapsuleBase::getLength() const
{
   return length_;
}
//*************************************************************************************************




//=================================================================================================
//
//  VOLUME, MASS AND DENSITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculates the volume of a capsule for a given radius and length.
 *
 * \param radius The radius of the cylinder part and the caps on both ends of the cylinder.
 * \param length The length of the cylinder part.
 * \return The volume of the capsule.
 */
inline real CapsuleBase::calcVolume( real radius, real length )
{
   return M_PI*radius*radius*( ( static_cast<real>( 4 ) / static_cast<real>( 3 ) )*radius + length );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the mass of a capsule for a given radius, length and density.
 *
 * \param radius The radius of the cylinder part and the caps on both ends of the cylinder.
 * \param length The length of the cylinder part.
 * \param density The density of the capsule.
 * \return The total mass of the capsule.
 */
inline real CapsuleBase::calcMass( real radius, real length, real density )
{
   return M_PI*radius*radius*( ( static_cast<real>( 4 ) / static_cast<real>( 3 ) )*radius + length ) * density;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the density of a capsule for a given radius, length and mass.
 *
 * \param radius The radius of the cylinder part and the caps on both ends of the cylinder.
 * \param length The length of the cylinder part.
 * \param mass The total mass of the capsule.
 * \return The density of the capsule.
 */
inline real CapsuleBase::calcDensity( real radius, real length, real mass )
{
   return mass / ( M_PI*radius*radius*( ( static_cast<real>( 4 ) / static_cast<real>( 3 ) )*radius + length ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \return The support point in world-frame coordinates in direction a\ d.
 */
inline Vec3 CapsuleBase::support( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised");

   const Vec3 bfD = vectorFromWFtoBF(d); //d in body frame coordinates

   const Vec3 supportSegment = Vec3( ((real(0.0) < bfD[0]) - (bfD[0] < real(0.0)))*length_*real(0.5), real(0.0), real(0.0));
   const Vec3 supportSphere = radius_*d;

   return gpos_ + vectorFromBFtoWF(supportSegment) + supportSphere;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates
 * \return The support point in world-frame coordinates in direction a\ d extended by a vector in
 *         direction \a d of length \a pe::contactThreshold.
 */
inline Vec3 CapsuleBase::supportContactThreshold( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised");
   return support(d) + d*contactThreshold;
}
//*************************************************************************************************

} // namespace pe

#endif
