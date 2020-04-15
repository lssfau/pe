//=================================================================================================
/*!
 *  \file pe/core/rigidbody/CylinderBase.h
 *  \brief Base class for the cylinder geometry
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

#ifndef _PE_CORE_RIGIDBODY_CYLINDERBASE_H_
#define _PE_CORE_RIGIDBODY_CYLINDERBASE_H_


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
/*!\brief Base class for the cylinder geometry.
 * \ingroup cylinder
 *
 * The CylinderBase class represents the base class for the cylinder geometry. It provides
 * the basic functionality of a cylinder. For a full description of the cylinder geometry,
 * see the Cylinder class description.
 */
class CylinderBase : public GeomPrimitive
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit CylinderBase( id_t sid, id_t uid, const Vec3& gpos, real radius,
                          real length, MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~CylinderBase() = 0;
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
/*!\brief Returns the radius of the cylinder.
 *
 * \return The radius of the cylinder.
 */
inline real CylinderBase::getRadius() const
{
   return radius_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the length of the cylinder part.
 *
 * \return The length of the cylinder part.
 */
inline real CylinderBase::getLength() const
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
/*!\brief Calculates the volume of a cylinder for a given radius and length.
 *
 * \param radius The radius of the cylinder.
 * \param length The length of the cylinder.
 * \return The volume of the cylinder.
 */
inline real CylinderBase::calcVolume( real radius, real length )
{
   return M_PI*radius*radius*length;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the mass of a cylinder for a given radius, length and density.
 *
 * \param radius The radius of the cylinder.
 * \param length The length of the cylinder.
 * \param density The density of the cylinder.
 * \return The total mass of the cylinder.
 */
inline real CylinderBase::calcMass( real radius, real length, real density )
{
   return M_PI*radius*radius*length*density;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the density of a cylinder for a given radius, length and mass.
 *
 * \param radius The radius of the cylinder.
 * \param length The length of the cylinder.
 * \param mass The total mass of the cylinder.
 * \return The density of the cylinder.
 */
inline real CylinderBase::calcDensity( real radius, real length, real mass )
{
   return mass / ( M_PI*radius*radius*length );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \return The support point in world-frame coordinates in direction a\ d.
 */
inline Vec3 CylinderBase::support( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised");

   const Vec3 bfD = vectorFromWFtoBF(d); //d in body frame coordinates

   const Vec3 supportSegment = Vec3( ((real(0.0) < bfD[0]) - (bfD[0] < real(0.0)))*length_*real(0.5), real(0.0), real(0.0) );

   const Vec3 discD = Vec3(0.0, bfD[1], bfD[2]).getNormalized();
   const Vec3 supportDisc = radius_* discD;

   return gpos_ + vectorFromBFtoWF(supportSegment + supportDisc);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates
 * \return The support point in world-frame coordinates in direction a\ d extended by a vector in
 *         direction \a d of length \a pe::contactThreshold.
 */
inline Vec3 CylinderBase::supportContactThreshold( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised");
   return support(d) + d*contactThreshold;
}
//*************************************************************************************************

} // namespace pe

#endif
