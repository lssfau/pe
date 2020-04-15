//=================================================================================================
/*!
 *  \file pe/core/attachable/GravityBase.h
 *  \brief Header file for the GravityBase class
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

#ifndef _PE_CORE_ATTACHABLE_GRAVITYBASE_H_
#define _PE_CORE_ATTACHABLE_GRAVITYBASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/attachable/ForceGenerator.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for the gravity force generator.
 * \ingroup force_generator
 *
 * The GravityBase class represents the base class for the gravity force generator. It
 * provides the basic functionality of a gravity force generator. For a full description
 * of gravity force generators, see the Gravity class description.
 */
class GravityBase : public ForceGenerator
{
private:
   //**Type definitions****************************************************************************
   typedef ForceGenerator  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit GravityBase( id_t sid, BodyID body, const Vec3& gravity );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~GravityBase();
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline BodyID      getBody   ();
   inline ConstBodyID getBody   () const;
   inline const Vec3& getGravity() const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   virtual void setVisible( bool visible );
           void setGravity( const Vec3& gravity );
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 gravity_;  //!< The exerted gravity.
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
/*!\brief Returns the attached rigid body.
 *
 * \return The attached rigid body.
 */
inline BodyID GravityBase::getBody()
{
   return bodies_[0];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the attached rigid body.
 *
 * \return The attached rigid body.
 */
inline ConstBodyID GravityBase::getBody() const
{
   return bodies_[0];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the currently acting gravity.
 *
 * \return The acting gravity.
 */
inline const Vec3& GravityBase::getGravity() const
{
   return gravity_;
}
//*************************************************************************************************

} // namespace pe

#endif
