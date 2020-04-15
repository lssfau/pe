//=================================================================================================
/*
 *  POV-Ray include file for the domino example
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


/*================
== Declarations ==
================*/

#declare Texture0 =
texture {
   pigment {
      image_map { png "0.png" map_type 0 once interpolate 2 }
   }
   scale <2, 1.5, 4>
   rotate <-90,-0,-0>
   finish {
      ambient 0.8
   }
}

#declare Texture1 =
texture {
   pigment {
      image_map { png "1.png" map_type 0 once interpolate 2 }
   }
   scale <2, 1.5, 4>
   rotate <-90,-0,-0>
   finish {
      ambient 0.8
   }
}

#declare Texture2 =
texture {
   pigment {
      image_map { png "2.png" map_type 0 once interpolate 2 }
   }
   scale <2, 1.5, 4>
   rotate <-90,-0,-0>
   finish {
      ambient 0.8
   }
}

#declare Texture3 =
texture {
   pigment {
      image_map { png "3.png" map_type 0 once interpolate 2 }
   }
   scale <2, 1.5, 4>
   rotate <-90,-0,-0>
   finish {
      ambient 0.8
   }
}

#declare Texture4 =
texture {
   pigment {
      image_map { png "4.png" map_type 0 once interpolate 2 }
   }
   scale <2, 1.5, 4>
   rotate <-90,-0,-0>
   finish {
      ambient 0.8
   }
}

#declare Texture5 =
texture {
   pigment {
      image_map { png "5.png" map_type 0 once interpolate 2 }
   }
   scale <2, 1.5, 4>
   rotate <-90,-0,-0>
   finish {
      ambient 0.8
   }
}

#declare Texture6 =
texture {
   pigment {
      image_map { png "6.png" map_type 0 once interpolate 2 }
   }
   scale <2, 1.5, 4>
   rotate <-90,-0,-0>
   finish {
      ambient 0.8
   }
}

#declare Separator =
texture {
   pigment {
      image_map { png "separator.png" map_type 0 once interpolate 2 }
   }
   scale <2, 0.2, 0.4>
   rotate <-90,-0,-0>
   finish {
      ambient 0.8
   }
}

#declare DominoBackground =
texture {
   T_Wood2
   finish {
      ambient 0.5
   }
}

#declare FloorTexture =
texture {
   pigment {
      image_map { gif "floor.gif" map_type 0 interpolate 2 }
   }
   finish {
      reflection 0.3
      ambient 0.1
   }
   scale 10
   rotate <-90,-0,-0>
}

#declare BallTexture =
texture {
   T_Chrome_4C
}

#declare GibbetTexture =
texture {
   T_Wood23
   finish {
      ambient 0.4
   }
}
