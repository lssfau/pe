//=================================================================================================
/*
 *  POV-Ray include file for the granular media example
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


/*================
== Declarations ==
================*/

#declare GroundTexture =
texture {
   pigment {
      image_map { tga "pave.tga" map_type 0 interpolate 2 }
   }
   rotate <-90,-0,-0>
   scale 40
}

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
   pigment {
      P_Silver1
   }
   finish {
      ambient 0.2 phong 0.9 reflection 0.3
   }
   normal {
      granite 0.2
   }
};

#declare GranularTexture2 =
texture {
   pigment {
      P_Copper1
   }
   finish {
      ambient 0.2 phong 0.9 reflection 0.3
   }
   normal {
      granite 0.2
   }
};

// Blue texture
#declare Texture0 =
texture {
   pigment {
      color rgbf <0,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Cyan texture
#declare Texture1 =
texture {
   pigment {
      color rgbf <0,0.5,0.8,0>
   }
   finish {
      ambient 0.2
   }
};

// Turquoise texture
#declare Texture2 =
texture {
   pigment {
      color rgbf <0,0.8,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Green texture
#declare Texture3 =
texture {
   pigment {
      color rgbf <0,0.7,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Dulcet texture
#declare Texture4 =
texture {
   pigment {
      color rgbf <0.4,0.9,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Dulcet2 texture
#declare Texture5 =
texture {
   pigment {
      color rgbf <0.7,1,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Yellow texture
#declare Texture6 =
texture {
   pigment {
      color rgbf <1,1,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Yolk texture
#declare Texture7 =
texture {
   pigment {
      color rgbf <1,0.7,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Orange texture
#declare Texture8 =
texture {
   pigment {
      color rgbf <1,0.4,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Red texture
#declare Texture9 =
texture {
   pigment {
      color rgbf <1,0,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Magenta texture
#declare Texture10 =
texture {
   pigment {
      color rgbf <0.75,0,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Purple texture
#declare Texture11 =
texture {
   pigment {
      color rgbf <0.75,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Violet texture
#declare Texture12 =
texture {
   pigment {
      color rgbf <0.4,0,1,0>
   }
   finish {
      ambient 0.2
   }
};


/*===============
== Environment ==
===============*/

// Sky sphere
sky_sphere {
   S_Cloud1
}
