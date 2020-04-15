//=================================================================================================
/*!
 *  \file pe/povray/DefaultTexture.h
 *  \brief Header file for the DefaultTexture class
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

#ifndef _PE_POVRAY_DEFAULTTEXTURE_H_
#define _PE_POVRAY_DEFAULTTEXTURE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/povray/Texture.h>
#include <pe/povray/TexturePolicy.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a uniform texture policy.
 * \ingroup povray
 *
 * The DefaultTexture class represents the simplest texture policy to assign a POV-Ray texture
 * to newly created rigid bodies. Its policy is to initially assign every body the same texture.
 * The following example demonstrates how the DefaultTexture policy is used:

   \code
   // The texture initially assigned to all (finite and infinite) bodies
   pe::povray::CustomTexture texture( "T_Stone9 scale 10.0" );  // Declared in the "stones.inc" header

   // Setting the new texture policy
   pe::povray::WriterID pov = pe::povray::activateWriter();
   pov->setTexturePolicy( pe::povray::DefaultTexture( texture ) );

   ...

   // Every newly created rigid body is automatically assigned the default texture
   createSphere( 1, 2.0, -3.0, 4.0, 0.5, iron );
   \endcode

 * Note that it is possible to assign different default textures to finite and infinite rigid
 * bodies!
 */
class PE_PUBLIC DefaultTexture : public TexturePolicy
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit DefaultTexture( const Texture& texture );
   explicit DefaultTexture( const Texture& finite, const Texture& infinite );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   virtual ~DefaultTexture();
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   using TexturePolicy::getTexture;
   virtual const Texture getTexture( ConstBodyID body ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Texture finite_;    //!< The default texture for finite bodies.
   Texture infinite_;  //!< The default texture for infinite bodies.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
