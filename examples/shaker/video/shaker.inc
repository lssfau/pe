//=================================================================================================
/*
 *  POV-Ray include file for the shaker example
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


/*============
== Includes ==
============*/

#include "colors.inc"
#include "woods.inc"
#include "metals.inc"
#include "stones.inc"
#include "skies.inc"


//sky_sphere {
//   S_Cloud2
//}
plane{<0,1,0>,2 hollow
   texture{ pigment {color rgb<0.1,0.35,0.8>*0.8}
      finish {ambient 1  diffuse 0}
   }

   texture {
      pigment {
         bozo turbulence 0.75
         octaves 6  omega 0.7 lambda 2 
         color_map {
            [0.0  color rgb <0.95, 0.95, 0.95> ]
               [0.05  color rgb <1, 1, 1>*1.25 ]
               [0.15 color rgb <0.85, 0.85, 0.85> ]
               [0.55 color rgbt <1, 1, 1, 1>*1 ]
               [1.0 color rgbt <1, 1, 1, 1>*1 ]
         }

         translate< 3, 0,-1>
         scale <0.3, 0.4, 0.2>*10
      }
      finish {ambient 1 diffuse 0}
   }

   scale 10000
}
fog{distance 300000 color White}

light_source {
  <2,1,2>, <1,1,1>
  parallel
  point_at <1, 0, 0.5>
}

/*================
== Declarations ==
================*/

#declare FloorTexture =
texture {
   pigment {
      image_map { jpeg "planks.jpg" map_type 0 interpolate 2 }
      warp{repeat x flip x}
      warp{repeat y flip y}
   }
   finish {
      reflection 0.0
      ambient 0.1
   }
   scale 3
   rotate <-90,-0,-0>
}

#include "glass.inc"
#declare TextureContainer =
texture {
   pigment { Orange }
   normal {
      bumps 0.04
      scale 0.2
   }
};
#declare TextureContainerGlass =
texture {
   pigment {
      Col_Glass_Clear
   }
   finish {
      F_Glass4
   }
   normal {
      bumps 0.04
      scale 0.2
   }
};
/*#declare InteriorContainer =
interior {
   I_Glass_Dispersion1
};
*/

#declare TextureContainerN = TextureContainer;
#declare TextureContainerE = TextureContainer;
#declare TextureContainerS = TextureContainer;
#declare TextureContainerW = TextureContainer;
#declare TextureContainerB = TextureContainer;

#declare TextureCapsules =
texture {
   pigment {
      image_map {jpeg "styrofoam.jpg"}
      rotate <0,90,0>
      scale 0.25
   }
};
#declare TextureTennis =
texture {
   pigment {
      uv_mapping
      image_map {png "tennisball.png"}
   }
};

#declare TextureCrates =
texture {
   pigment {
      uv_mapping
      image_map {png "crate.png"}
   }
};

#declare TextureAll = TextureTennis
#declare Texture0 = TextureAll
#declare Texture1 = TextureAll
#declare Texture2 = TextureAll
#declare Texture3 = TextureAll
#declare Texture4 = TextureAll
#declare Texture5 = TextureAll
#declare Texture6 = TextureAll
#declare Texture7 = TextureAll
#declare Texture8 = TextureAll
#declare Texture9 = TextureAll
#declare Texture10 = TextureAll
#declare Texture11 = TextureAll
#declare Texture12 = TextureAll
#declare Texture13 = TextureAll
#declare Texture14 = TextureAll
#declare Texture15 = TextureAll
#declare Texture16 = TextureAll
#declare Texture17 = TextureAll
#declare Texture18 = TextureAll
