//=================================================================================================
/*
 *  POV-Ray include file for the hourglass example
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

#include "metals.inc"
#include "colors.inc"
#include "glass.inc"
#include "skies.inc"
#include "stones.inc"
#include "woods.inc"


/*================
== Declarations ==
================*/

#declare GlassTexture =
texture {
   pigment {
      Col_Glass_Clear
   }
   finish {
      F_Glass5
   }
   normal {
      bumps 0.07
      scale 1
   }
}

#declare GranularTexture1 =
texture {
   // Sand texture
   pigment {
      color rgb <0.9,0.8,0.5>
   }
   finish {
      ambient 0.2 phong 0.7 reflection 0.1
   }
   normal {
      granite 1.0
      scale 0.1
   }

   // Striped texture
   /*pigment {
      P_Silver1
   }
   finish {
      ambient 0.3 phong 0.9 reflection 0.3
   }
   normal {
      granite 0.3
      scale 0.1
   }*/
};

#declare GranularTexture2 =
texture {
   // Sand texture
   pigment {
      color rgb <0.9,0.8,0.5>
   }
   finish {
      ambient 0.2 phong 0.7 reflection 0.1
   }
   normal {
      granite 1.0
      scale 0.1
   }

   // Striped texture
   /*pigment {
      P_Copper1
   }
   finish {
      ambient 0.3 phong 0.9 reflection 0.3
   }
   normal {
      granite 0.3
      scale 0.1
   }*/
};

// Blue texture
#declare Texture0 =
texture {
   pigment {
      color rgbf <0,0,1,0>
   }
   finish {
      ambient 0.3
   }
};

// Cyan texture
#declare Texture1 =
texture {
   pigment {
      color rgbf <0,0.5,0.8,0>
   }
   finish {
      ambient 0.3
   }
};

// Turquoise texture
#declare Texture2 =
texture {
   pigment {
      color rgbf <0,0.8,0.5,0>
   }
   finish {
      ambient 0.3
   }
};

// Green texture
#declare Texture3 =
texture {
   pigment {
      color rgbf <0,0.7,0,0>
   }
   finish {
      ambient 0.3
   }
};

// Dulcet texture
#declare Texture4 =
texture {
   pigment {
      color rgbf <0.4,0.9,0,0>
   }
   finish {
      ambient 0.3
   }
};

// Dulcet2 texture
#declare Texture5 =
texture {
   pigment {
      color rgbf <0.7,1,0,0>
   }
   finish {
      ambient 0.3
   }
};

// Yellow texture
#declare Texture6 =
texture {
   pigment {
      color rgbf <1,1,0,0>
   }
   finish {
      ambient 0.3
   }
};

// Yolk texture
#declare Texture7 =
texture {
   pigment {
      color rgbf <1,0.7,0,0>
   }
   finish {
      ambient 0.3
   }
};

// Orange texture
#declare Texture8 =
texture {
   pigment {
      color rgbf <1,0.4,0,0>
   }
   finish {
      ambient 0.3
   }
};

// Red texture
#declare Texture9 =
texture {
   pigment {
      color rgbf <1,0,0,0>
   }
   finish {
      ambient 0.3
   }
};

// Magenta texture
#declare Texture10 =
texture {
   pigment {
      color rgbf <0.75,0,0.5,0>
   }
   finish {
      ambient 0.3
   }
};

// Purple texture
#declare Texture11 =
texture {
   pigment {
      color rgbf <0.75,0,1,0>
   }
   finish {
      ambient 0.3
   }
};

// Violet texture
#declare Texture12 =
texture {
   pigment {
      color rgbf <0.4,0,1,0>
   }
   finish {
      ambient 0.3
   }
};


/*===============
== Environment ==
===============*/

// Sky sphere
sky_sphere {
   S_Cloud1
}

// Hourglass
merge
{
   // Upper hourglass
   difference {
      cone {
         <0, 9, 0>, 9.8446,
         <0, -0.001, 0>, 1.7
         texture {
            GlassTexture
            finish { ambient 0.4 }
         }
      }
      cone {
         <0, 9, 0>, 9.6446,
         <0, -0.001, 0>, 1.5
         texture {
            GlassTexture
            finish { ambient 0.4 }
         }
      }
   }

   // Lower hourglass
   difference {
      cone {
         <0, 0.001, 0>, 1.7,
         <0, -9, 0>, 9.8446
         open
         texture {
            GlassTexture
            finish { ambient 0.4 }
         }
      }
      cone {
         <0, 0.001, 0>, 1.5,
         <0, -9, 0>, 9.6446
         open
         texture {
            GlassTexture
            finish { ambient 0.4 }
         }
      }
   }
}

// Upper hourglass lid
merge {
   torus {
      10, 0.5
      texture {
         T_Wood1
         scale 1.4
         finish { ambient 0.4 }
      }
      translate <0,9.5,0>
   }
   cylinder {
      <0,9,0>, <0,10,0>, 10
      texture {
         T_Wood1
         scale 1.4
         finish { ambient 0.4 }
      }
   }
   rotate <0,21,0>
}

// Lower hourglass lid
merge {
   torus {
      10, 0.5
      texture {
         T_Wood1
         scale 1.4
         finish { ambient 0.4 }
      }
      translate <0,-9.5,0>
   }
   cylinder {
      <0,-9,0>, <0,-10,0>, 10
      texture {
         T_Wood1
         scale 1.4
         finish { ambient 0.4 }
      }
   }
   rotate <0,-13,0>
}

// Marble column
cylinder {
   <0,-10,0>, <0,-40,0>, 15
   texture {
      T_Grnt10
      normal {
         agate 0.5
         scale 0.2
      }
      scale 7.0
   }
}

// Ground plane
plane {
   <0,1,0>, -40
   texture {
      pigment {
         image_map { tga "pave.tga" map_type 0 interpolate 2 }
      }
      rotate <-90,-0,-0>
      scale 60
   }
}
