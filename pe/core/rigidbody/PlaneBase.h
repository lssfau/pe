//=================================================================================================
/*!
 *  \file pe/core/rigidbody/PlaneBase.h
 *  \brief Base class for the plane geometry
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

#ifndef _PE_CORE_RIGIDBODY_PLANEBASE_H_
#define _PE_CORE_RIGIDBODY_PLANEBASE_H_


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
/*!\brief Base class for the plane geometry.
 * \ingroup plane
 *
 * The PlaneBase class represents the base class for the plane geometry. It provides
 * the basic functionality of a plane. For a full description of the plane geometry,
 * see the Plane class description.
 */
class PlaneBase : public GeomPrimitive
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit PlaneBase( id_t sid, id_t uid, const Vec3& gpos, const Vec3& normal,
                       real d, MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~PlaneBase() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vec3& getNormal()       const;
   inline real        getDisplacement() const;
   //@}
   //**********************************************************************************************

protected:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual void calcBoundingBox();  // Calculation of the axis-aligned bounding box
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   template< typename C > friend class CollisionSystem;
   /*! \endcond */
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 normal_;  //!< Normal of the plane in reference to the global world frame.
                  /*!< The normal of the plane is always pointing towards the halfspace
                       outside the plane. */
   real d_;       //!< Plane displacement from the origin.
                  /*!< The displacement can be categorized in the following way:\n
                        - > 0: The global origin is inside the plane\n
                        - < 0: The global origin is outside the plane\n
                        - = 0: The global origin is on the surface of the plane */
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
/*!\brief Returns the normal of the plane in reference to the global world frame.
 *
 * \return The normal of the plane.
 */
inline const Vec3& PlaneBase::getNormal() const
{
   return normal_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the displacement/distance from the origin of the global world frame.
 *
 * \return The displacement of the plane.
 *
 * A positive displacement value indicates that the origin of the global world frame is contained
 * in the plane, whereas a negative value indicates that the origin is not contained in the plane.
 */
inline real PlaneBase::getDisplacement() const
{
   return d_;
}
//*************************************************************************************************

} // namespace pe

#endif
