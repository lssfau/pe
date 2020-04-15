//=================================================================================================
/*!
 *  \file pe/core/Trigger.h
 *  \brief Trigger interface class
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

#ifndef _PE_CORE_TRIGGER_H_
#define _PE_CORE_TRIGGER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Trigger interface class.
 * \ingroup core
 *
 * The Trigger class is an abstract base class that offers the functionality to automatically
 * trigger every active Trigger object at the end of every time step. Every class derived from
 * the Trigger class must implement the trigger() function to define the action to be performed
 * when the trigger event occurs.
 */
class Trigger
{
private:
   //**Type definitions****************************************************************************
   typedef PtrVector<Trigger,NoDelete>  Triggers;  //!< Vector container for active triggers.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Trigger();
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Trigger();
   //@}
   //**********************************************************************************************

   //**Trigger functions***************************************************************************
   /*!\name Trigger functions */
   //@{
   virtual void trigger() = 0;
   //@}
   //**********************************************************************************************

private:
   //**Trigger functions***************************************************************************
   /*!\name Trigger functions */
   //@{
   static inline void triggerAll();
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static Triggers triggers_;  //!< The currently active triggers.
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
//  TRIGGER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Triggers all active trigger objects.
 *
 * \return void
 */
inline void Trigger::triggerAll()
{
   if( triggers_.isEmpty() ) return;

   for( Triggers::Iterator t=triggers_.begin(); t!=triggers_.end(); ++t ) {
      t->trigger();
   }
}
//*************************************************************************************************

} // namespace pe

#endif
