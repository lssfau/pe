//=================================================================================================
/*!
 *  \file pe/core/rigidbody/BoxBase.h
 *  \brief Base class for the box geometry
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

#ifndef _PE_CORE_RIGIDBODY_BOXBASE_H_
#define _PE_CORE_RIGIDBODY_BOXBASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/GeomPrimitive.h>
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
/*!\brief Base class for the box geometry.
 * \ingroup box
 *
 * The BoxBase class represents the base class for the box geometry. It provides the basic
 * functionality of a box. For a full description of the box geometry, see the Box class
 * description.
 */
class BoxBase : public GeomPrimitive
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit BoxBase( id_t sid, id_t uid, const Vec3& gpos,
                     const Vec3& lengths, MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~BoxBase() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vec3& getLengths() const;
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
   static inline real calcVolume( real x, real y, real z );
   static inline real calcVolume( const Vec3& l );
   static inline real calcMass( real x, real y, real z, real density );
   static inline real calcMass( const Vec3& l, real density );
   static inline real calcDensity( real x, real y, real z, real mass );
   static inline real calcDensity( const Vec3& l, real mass );
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
   Vec3 lengths_;  //!< Lengths of the x-, y- and z-sides of the box.
                   /*!< All side lengths are constrained to values larger than 0. */
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
/*!\brief Returns the side lengths of the box.
 *
 * \return The side lengths of the box.
 */
inline const Vec3& BoxBase::getLengths() const
{
   return lengths_;
}
//*************************************************************************************************




//=================================================================================================
//
//  VOLUME, MASS AND DENSITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculates the volume of a box for given side lengths.
 *
 * \param x The x-length of the box.
 * \param y The y-length of the box.
 * \param z The z-length of the box.
 * \return The volume of the box.
 */
inline real BoxBase::calcVolume( real x, real y, real z )
{
   return x * y * z;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the volume of a box for given side lengths.
 *
 * \param l The side lengths of the box.
 * \return The volume of the box.
 */
inline real BoxBase::calcVolume( const Vec3& l )
{
   return l[0] * l[1] * l[2];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the mass of a box for given side lengths and density.
 *
 * \param x The x-length of the box.
 * \param y The y-length of the box.
 * \param z The z-length of the box.
 * \param density The density of the box.
 * \return The total mass of the box.
 */
inline real BoxBase::calcMass( real x, real y, real z, real density )
{
   return x * y * z * density;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the mass of a box for given side lengths and density.
 *
 * \param l The side lengths of the box.
 * \param density The density of the box.
 * \return The total mass of the box.
 */
inline real BoxBase::calcMass( const Vec3& l, real density )
{
   return l[0] * l[1] * l[2] * density;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the density of a box for given side lengths and mass.
 *
 * \param x The x-length of the box.
 * \param y The y-length of the box.
 * \param z The z-length of the box.
 * \param mass The total mass of the box.
 * \return The density of the box.
 */
inline real BoxBase::calcDensity( real x, real y, real z, real mass )
{
   return mass / ( x * y * z );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the density of a box for given side lengths and mass.
 *
 * \param l The side lengths of the box.
 * \param mass The total mass of the box.
 * \return The density of the box.
 */
inline real BoxBase::calcDensity( const Vec3& l, real mass )
{
   return mass / ( l[0] * l[1] * l[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \return The support point in world-frame coordinates in direction a\ d.
 */
inline Vec3 BoxBase::support( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised");

   const Vec3 bfD = vectorFromWFtoBF(d); //d in body frame coordinates

   /*as there is no signum funktion in the std-lib I found this
   http://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
   template <typename T> 
   int sgn(T val) {
      return (T(0) < val) - (val < T(0));
   }
   */

   //As it is save to say we have atleast one component of the d-vector != 0 we can use
   Vec3 relativSupport = Vec3( ((real(0.0) < bfD[0]) - (bfD[0] < real(0.0)))*lengths_[0]*real(0.5), 
                               ((real(0.0) < bfD[1]) - (bfD[1] < real(0.0)))*lengths_[1]*real(0.5), 
                               ((real(0.0) < bfD[2]) - (bfD[2] < real(0.0)))*lengths_[2]*real(0.5) );

   return gpos_ + vectorFromBFtoWF(relativSupport);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \return The support point in world-frame coordinates in direction a\ d extended by a vector in
 *         direction \a d of length \a pe::contactThreshold.
 */
inline Vec3 BoxBase::supportContactThreshold( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised" );
   return support(d) + d*contactThreshold;
}
//*************************************************************************************************

} // namespace pe

#endif
