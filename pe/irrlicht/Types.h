//=================================================================================================
/*!
 *  \file pe/irrlicht/Types.h
 *  \brief Header file for irrlicht module type definitions
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

#ifndef _PE_IRRLICHT_TYPES_H_
#define _PE_IRRLICHT_TYPES_H_


//=================================================================================================
//
//  ::irr NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

namespace irr {
   class IrrlichtDevice;
   namespace video {
      class IVideoDriver;
      class ITexture;
   }
   namespace scene {
      class ISceneManager;
      class ISceneNode;
      class IAnimatedMesh;
      class ICameraSceneNode;
      class IDummyTransformationSceneNode;
      class ILightSceneNode;
   }
}

#endif
