//=================================================================================================
/*!
 *  \file pe/povray/Modifier.h
 *  \brief Base class for all POV-Ray modifiers
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

#ifndef _PE_POVRAY_MODIFIER_H_
#define _PE_POVRAY_MODIFIER_H_


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup povray_modifier Modifiers
 * \ingroup povray
 */
/*!\brief Base class for all POV-Ray modifiers.
 * \ingroup povray_modifier
 *
 * The Modifier class represents the base class for all POV-Ray modifiers. Every modifier must
 * derive from this class in order to qualify as valid POV-Ray modifier. Examples for Modifiers
 * are for instance the Turbulence class, the Frequency class or the Phase class.
 */
class Modifier
{
protected:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline Modifier();
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
/*!\brief Default constructor for the Modifier class.
 */
inline Modifier::Modifier()
{}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
