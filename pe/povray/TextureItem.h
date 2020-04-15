//=================================================================================================
/*!
 *  \file pe/povray/TextureItem.h
 *  \brief Base class for all POV-Ray texture components
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

#ifndef _PE_POVRAY_TEXTUREITEM_H_
#define _PE_POVRAY_TEXTUREITEM_H_


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for all POV-Ray texture components.
 * \ingroup povray_texture
 *
 * The TextureItem class represents the base class for a single component of a POV-Ray texture.
 * Every texture component must derive from this class in order to qualify as valid component
 * for a POV-Ray texture. Examples for TextureItems are for instance the Pigment, the Finish or
 * the Normal classes.
 */
class TextureItem
{
protected:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline TextureItem();
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
/*!\brief Default constructor for the TextureItem class.
 */
inline TextureItem::TextureItem()
{}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
