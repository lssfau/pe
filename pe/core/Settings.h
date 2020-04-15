//=================================================================================================
/*!
 *  \file pe/core/Settings.h
 *  \brief Header file for the world settings
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

#ifndef _PE_CORE_SETTINGS_H_
#define _PE_CORE_SETTINGS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/NonCreatable.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Settings of the rigid body simulation world.
 * \ingroup core
 *
 * The Settings class represents the current settings of the rigid body simulation world.
 * It offers the functionality to acquire the current settings (as for instance the damping
 * factor and the gravity) for all modules of the physics engine. Although it is closely
 * coupled to the World class, it is implemented as a separate class to minimize coupling
 * for other classes.
 */
class PE_PROTECTED Settings : private NonCreatable
{
public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   static inline bool        forceReset();
   static inline real        damping();
   static inline const Vec3& gravity();
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static bool forceReset_;  //!< Flag for the automatic force resetting after a simulation step.
   static real damping_;     //!< Damping factor of the simulation world.
   static Vec3 gravity_;     //!< Gravity of the simulated system.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class World;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the current force reset flag.
 *
 * \return \a true if automatic force resetting is switched on, \a false if not.
 */
inline bool Settings::forceReset()
{
   return forceReset_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current damping factor.
 *
 * \return The current damping.
 */
inline real Settings::damping()
{
   return damping_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current gravity.
 *
 * \return The current gravity.
 */
inline const Vec3& Settings::gravity()
{
   return gravity_;
}
//*************************************************************************************************

} // namespace pe

#endif
