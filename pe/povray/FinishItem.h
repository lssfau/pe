//=================================================================================================
/*!
 *  \file pe/povray/FinishItem.h
 *  \brief Base class for all POV-Ray finish components
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

#ifndef _PE_POVRAY_FINISHITEM_H_
#define _PE_POVRAY_FINISHITEM_H_


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for all POV-Ray finish components.
 * \ingroup povray_finish
 *
 * The FinishItem class represents the base class for a single component of a POV-Ray finish.
 * Every finish component must derive from this class in order to qualify as valid component
 * for a POV-Ray finish. Examples for FinishItems are for instance the Ambient, the Diffuse or
 * the Specular classes.
 */
class FinishItem
{
protected:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline FinishItem();
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for the FinishItem class.
 */
inline FinishItem::FinishItem()
{}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
