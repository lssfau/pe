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

#declare BrickTexture =
texture {
   pigment {
      image_map { gif "brick.gif" map_type 0 interpolate 2 }
   }
   normal {
      marble 0.7
      scale 2
   }
   scale 3.5
   rotate <-0,-0,-90>
}


/*===============
== Environment ==
===============*/

// Sky sphere
sky_sphere {
   S_Cloud1
}
