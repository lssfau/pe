//=================================================================================================
/*!
 *  \file pe/irrlicht/LightSource.h
 *  \brief Header file for an Irrlicht light source
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

#ifndef _PE_IRRLICHT_LIGHTSOURCE_H_
#define _PE_IRRLICHT_LIGHTSOURCE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/irrlicht/Types.h>


namespace pe {

namespace irrlicht {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup irrlicht_lightsource Light source
 * \ingroup irrlicht
 */
/*!\brief An Irrlicht light source.
 * \ingroup irrlicht_lightsource
 *
 * The LightSource class represents an Irrlicht light source. The class acts as a wrapper for
 * an Irrlicht light scene node and offers most of the available features.
 */
class LightSource
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline LightSource( ::irr::scene::ILightSceneNode* light );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   void setPosition( float px, float py, float pz );
   void setAmbient( float red, float green, float blue );
   void setDiffuse( float red, float green, float blue );
   void setSpecular( float red, float green, float blue );
   void setRadius( float radius );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   ::irr::scene::ILightSceneNode* light_;  //!< Irrlicht light source handle.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The constructor of the Irrlicht LightSource class.
 *
 * \param light The Irrlicht light source handle
 */
inline LightSource::LightSource( ::irr::scene::ILightSceneNode* light )
   : light_(light)  // Irrlicht light source handle
{}
//*************************************************************************************************

} // namespace irrlicht

} // namespace pe

#endif
