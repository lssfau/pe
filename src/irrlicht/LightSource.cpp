//=================================================================================================
/*!
 *  \file src/irrlicht/LightSource.cpp
 *  \brief Source file for an Irrlicht light source
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


#if HAVE_IRRLICHT

//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/irrlicht/LightSource.h>


//*************************************************************************************************
// Irrlicht includes
//*************************************************************************************************

#include <irrlicht/irrlicht.h>


namespace pe {

namespace irrlicht {

//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the position of the light source.
 *
 * \param px The x-component of the global position.
 * \param py The y-component of the global position.
 * \param pz The z-component of the global position.
 * \return void
 */
void LightSource::setPosition( float px, float py, float pz )
{
   light_->getLightData().Position.set( px, py, pz );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the ambient color of the light source.
 *
 * \param red The red ambient color channel of the light source \f$ [0..1] \f$.
 * \param green The green ambient color channel of the light source \f$ [0..1] \f$.
 * \param blue The blue ambient color channel of the light source \f$ [0..1] \f$.
 * \return void
 */
void LightSource::setAmbient( float red, float green, float blue )
{
   light_->getLightData().AmbientColor.set( red, green, blue );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the diffuse color of the light source.
 *
 * \param red The red diffuse color channel of the light source \f$ [0..1] \f$.
 * \param green The green diffuse color channel of the light source \f$ [0..1] \f$.
 * \param blue The blue diffuse color channel of the light source \f$ [0..1] \f$.
 * \return void
 */
void LightSource::setDiffuse( float red, float green, float blue )
{
   light_->getLightData().DiffuseColor.set( red, green, blue );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the specular color of the light source.
 *
 * \param red The red specular color channel of the light source \f$ [0..1] \f$.
 * \param green The green specular color channel of the light source \f$ [0..1] \f$.
 * \param blue The blue specular color channel of the light source \f$ [0..1] \f$.
 * \return void
 */
void LightSource::setSpecular( float red, float green, float blue )
{
   light_->getLightData().SpecularColor.set( red, green, blue );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the influence radius of the light source.
 *
 * \param radius The influence radius of the light source.
 * \return void
 */
void LightSource::setRadius( float radius )
{
   light_->getLightData().Radius = radius;
}
//*************************************************************************************************

} // namespace irrlicht

} // namespace pe

#endif
