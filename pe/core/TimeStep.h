//=================================================================================================
/*!
 *  \file pe/core/TimeStep.h
 *  \brief Global time step configuration
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

#ifndef _PE_CORE_TIMESTEP_H_
#define _PE_CORE_TIMESTEP_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/system/Precision.h>
#include <pe/util/NonCreatable.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global time step configuration for the rigid body simulation world.
 * \ingroup core
 *
 * The TimeStep class represents the configuration of the current rigid body simulation time
 * step. It offers the functionality to acquire the number and size of the current time step
 * for all modules of the physics engine. Although it is closely coupled with the World class
 * it is implemented as a separate class to minimize coupling for other classes.
 */
class TimeStep : private NonCreatable
{
private:
   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class World;
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   static inline unsigned int step();
   static inline real         size();
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   static inline void         step( unsigned int step );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static unsigned int step_;  //!< The current time step.
   static real         size_;  //!< The size of the current time step.
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
/*!\brief Returns the current time step.
 *
 * \return The current time step.
 */
inline unsigned int TimeStep::step()
{
   return step_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the size of the current time step.
 *
 * \return The size of the current time step.
 */
inline real TimeStep::size()
{
   return size_;
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Sets the current time step.
 *
 * \param step The time step number.
 */
inline void TimeStep::step( unsigned int step )
{
   step_ = step;
}
//*************************************************************************************************

} // namespace pe

#endif
