//=================================================================================================
/*!
 *  \file src/povray/DefaultTexture.cpp
 *  \brief Source file for the DefaultTexture class
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/povray/TexturePolicy.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor of the TexturePolicy class.
 */
TexturePolicy::~TexturePolicy()
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the default initial texture of the given body.
 *
 * \param body The body to be assigned the initial texture.
 * \return The initial texture of the given body.
 *
 * This function returns the initial default POV-Ray texture of the given body. This default
 * texture is a plain black color texture.
 */
const Texture TexturePolicy::getTexture( ConstBodyID /*body*/ ) const
{
   return Texture();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the default initial texture of the given joint.
 *
 * \param joint The joint to be assigned the initial texture.
 * \return The initial texture of the given joint.
 *
 * This function returns the initial default POV-Ray texture of the given joint. This default
 * texture is a plain black color texture.
 */
const Texture TexturePolicy::getTexture( ConstJointID /*joint*/ ) const
{
   return Texture();
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
