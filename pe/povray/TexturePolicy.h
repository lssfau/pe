//=================================================================================================
/*!
 *  \file pe/povray/TexturePolicy.h
 *  \brief Header file for the TexturePolicy base class
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

#ifndef _PE_POVRAY_TEXTUREPOLICY_H_
#define _PE_POVRAY_TEXTUREPOLICY_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/Types.h>
#include <pe/povray/Texture.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Texture policy interface class.
 * \ingroup povray
 *
 * The TexturePolicy class is a base class for all texture policies used by the POV-Ray writer.
 * Its purpose is the abstract, automatic assignment of POV-Ray texture to newly created objects.
 * The following example demonstrates the use of the simplest texture policy, the DefaultTexture:

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
 */
class PE_PUBLIC TexturePolicy
{
public:
   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~TexturePolicy();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual const Texture getTexture( ConstBodyID  body  ) const;
   virtual const Texture getTexture( ConstJointID joint ) const;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
