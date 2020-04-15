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

#include <pe/core/MPI.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/povray/DefaultTexture.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor of the DefaultTexture class.
 *
 * \param texture The default texture to be initially assigned to all bodies.
 */
DefaultTexture::DefaultTexture( const Texture& texture )
   : finite_  ( texture )  // The default texture for finite bodies
   , infinite_( texture )  // The default texture for infinite bodies
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor of the DefaultTexture class.
 *
 * \param finite The default texture to be initially assigned to all finite bodies.
 * \param infinite The default texture to be initially assigned to all infinite bodies.
 */
DefaultTexture::DefaultTexture( const Texture& finite, const Texture& infinite )
   : finite_  ( finite   )  // The default texture for finite bodies
   , infinite_( infinite )  // The default texture for infinite bodies
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor of the DefaultTexture class.
 */
DefaultTexture::~DefaultTexture()
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the initial texture of the given body.
 *
 * \param body The body to be assigned the default texture.
 * \return The initial texture of the given body.
 */
const Texture DefaultTexture::getTexture( ConstBodyID body ) const
{
   if( body->isFinite() ) return finite_;
   else return infinite_;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
