//=================================================================================================
/*
 *  POV-Ray include file for the mpinano example
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
#include "metals.inc"
#include "stones.inc"


/*================
== Declarations ==
================*/

#declare GroundTexture =
texture {
   pigment {
      image_map {
         gif "ground.gif" map_type 0 interpolate 2
      }
   }
   scale 20
   rotate <-90,-0,-0>
}

#declare WallTexture =
texture {
   pigment {
      image_map {
         gif "wall.gif" map_type 0 interpolate 2
      }
   }
   scale 20
}

// Blue texture
#declare Texture0 =
texture {
   pigment {
      color rgbf <0,0,1,0>
   }
   finish {
      ambient 0.2
   }
   normal {
      granite 0.4
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
   normal {
      granite 0.4
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
   normal {
      granite 0.4
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
   normal {
      granite 0.4
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
   normal {
      granite 0.4
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
   normal {
      granite 0.4
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
   normal {
      granite 0.4
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
   normal {
      granite 0.4
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
   normal {
      granite 0.4
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
   normal {
      granite 0.4
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
   normal {
      granite 0.4
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
   normal {
      granite 0.4
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
   normal {
      granite 0.4
   }
};
