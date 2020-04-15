//=================================================================================================
/*!
 *  \file pe/core/rigidbody/GeomPrimitive.h
 *  \brief Header file for the GeomPrimitive class
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

#ifndef _PE_CORE_RIGIDBODY_GEOMPRIMITIVE_H_
#define _PE_CORE_RIGIDBODY_GEOMPRIMITIVE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/GeomType.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/core/Types.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup geometric_primitive Geometric primitives
 * \ingroup rigid_body
 */
/*!\brief Base class for all primitive geometries.
 * \ingroup geometric_primitive
 *
 * The GeomPrimitive class is the abstract basis for all geometric primitives of the rigid body
 * physics engine. The class provides the common data members and the common functionality for
 * all geometry classes.
 */
class GeomPrimitive : public RigidBody
{
public:
   //**Type definitions****************************************************************************
   struct Parameters : public RigidBody::Parameters {
      MaterialID material_;
   };
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline MaterialID getMaterial() const;
   //@}
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit GeomPrimitive( GeomType type, bool finite, bool visible,
                           id_t sid, id_t uid, MaterialID material );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~GeomPrimitive() = 0;
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   MaterialID material_;  //!< The material of the geometric primitive.
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
/*!\brief Returns the material of the geometric primitive.
 *
 * \return The coefficient of restitution of the rigid body.
 */
inline MaterialID GeomPrimitive::getMaterial() const
{
   return material_;
}
//*************************************************************************************************

} // namespace pe

#endif
