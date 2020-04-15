//=================================================================================================
/*
 *  POV-Ray include file for the labyrinth example
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
#include "skies.inc"
#include "stones.inc"
#include "woods.inc"


/*================
== Declarations ==
================*/

#declare WallTexture =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5>
   }
   normal {
      granite 0.8
      scale 0.2
   }
}

#declare GranularTexture1 =
texture {
   pigment {
      P_Silver1
   }
   finish {
      ambient 0.3 phong 0.9 reflection 0.3
   }
   normal {
      granite 0.3
      scale 0.1
   }
};

#declare GranularTexture2 =
texture {
   pigment {
      P_Copper1
   }
   finish {
      ambient 0.3 phong 0.9 reflection 0.3
   }
   normal {
      granite 0.3
      scale 0.1
   }
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

// Background plane
plane {
   <0,0,-1>, -1
   texture { WallTexture }
}

// First row
prism {
   0, 1.5, 4
   <-2,0>, <2,0>, <0,2>, <-2,0>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <3,0>, <7,0>, <5,2>, <3,0>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <8,0>, <12,0>, <10,2>, <8,0>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <13,0>, <17,0>, <15,2>, <13,0>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <18,0>, <22,0>, <20,2>, <18,0>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <23,0>, <27,0>, <25,2>, <23,0>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Second row
prism {
   0, 1.5, 4
   <0.5,3>, <4.5,3>, <2.5,5>, <0.5,3>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <5.5,3>, <9.5,3>, <7.5,5>, <5.5,3>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <10.5,3>, <14.5,3>, <12.5,5>, <10.5,3>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <15.5,3>, <19.5,3>, <17.5,5>, <15.5,3>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <20.5,3>, <24.5,3>, <22.5,5>, <20.5,3>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Third row
prism {
   0, 1.5, 4
   <-2,6>, <2,6>, <0,8>, <-2,6>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <3,6>, <7,6>, <5,8>, <3,6>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   < 8,6>, <12,6>, <10,8>, <8,6>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <13,6>, <17,6>, <15,8>, <13,6>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <18,6>, <22,6>, <20,8>, <18,6>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <23,6>, <27,6>, <25,8>, <23,6>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Fourth row
prism {
   0, 1.5, 4
   <0.5,9>, <4.5,9>, <2.5,11>, <0.5,9>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <5.5,9>, <9.5,9>, <7.5,11>, <5.5,9>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <10.5,9>, <14.5,9>, <12.5,11>, <10.5,9>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <15.5,9>, <19.5,9>, <17.5,11>, <15.5,9>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <20.5,9>, <24.5,9>, <22.5,11>, <20.5,9>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Fifth row
prism {
   0, 1.5, 4
   <-2,12>, <2,12>, <0,14>, <-2,12>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <3,12>, <7,12>, <5,14>, <3,12>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   < 8,12>, <12,12>, <10,14>, <8,12>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <13,12>, <17,12>, <15,14>, <13,12>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <18,12>, <22,12>, <20,14>, <18,12>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <23,12>, <27,12>, <25,14>, <23,12>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Sixth row
prism {
   0, 1.5, 4
   <0.5,15>, <4.5,15>, <2.5,17>, <0.5,15>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <5.5,15>, <9.5,15>, <7.5,17>, <5.5,15>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <10.5,15>, <14.5,15>, <12.5,17>, <10.5,15>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <15.5,15>, <19.5,15>, <17.5,17>, <15.5,15>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <20.5,15>, <24.5,15>, <22.5,17>, <20.5,15>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Seventh row
prism {
   0, 1.5, 4
   <-2,18>, <2,18>, <0,20>, <-2,18>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <3,18>, <7,18>, <5,20>, <3,18>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   < 8,18>, <12,18>, <10,20>, <8,18>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <13,18>, <17,18>, <15,20>, <13,18>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <18,18>, <22,18>, <20,20>, <18,18>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <23,18>, <27,18>, <25,20>, <23,18>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Eigth row
prism {
   0, 1.5, 4
   <0.5,21>, <4.5,21>, <2.5,23>, <0.5,21>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <5.5,21>, <9.5,21>, <7.5,23>, <5.5,21>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <10.5,21>, <14.5,21>, <12.5,23>, <10.5,21>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <15.5,21>, <19.5,21>, <17.5,23>, <15.5,21>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <20.5,21>, <24.5,21>, <22.5,23>, <20.5,21>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Ninth row
prism {
   0, 1.5, 4
   <-2,24>, <2,24>, <0,26>, <-2,24>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <3,24>, <7,24>, <5,26>, <3,24>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <8,24>, <12,24>, <10,26>, <8,24>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <13,24>, <17,24>, <15,26>, <13,24>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <18,24>, <22,24>, <20,26>, <18,24>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <23,24>, <27,24>, <25,26>, <23,24>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Tenth row
prism {
   0, 1.5, 4
   <0.5,27>, <4.5,27>, <2.5,29>, <0.5,27>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <5.5,27>, <9.5,27>, <7.5,29>, <5.5,27>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <10.5,27>, <14.5,27>, <12.5,29>, <10.5,27>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <15.5,27>, <19.5,27>, <17.5,29>, <15.5,27>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <20.5,27>, <24.5,27>, <22.5,29>, <20.5,27>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Eleventh row
prism {
   0, 1.5, 4
   <-2,30>, <2,30>, <0,32>, <-2,30>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <3,30>, <7,30>, <5,32>, <3,30>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <8,30>, <12,30>, <10,32>, <8,30>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <13,30>, <17,30>, <15,32>, <13,30>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <18,30>, <22,30>, <20,32>, <18,30>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <23,30>, <27,30>, <25,32>, <23,30>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Twelfth row
prism {
   0, 1.5, 4
   <0.5,33>, <4.5,33>, <2.5,35>, <0.5,33>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <5.5,33>, <9.5,33>, <7.5,35>, <5.5,33>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <10.5,33>, <14.5,33>, <12.5,35>, <10.5,33>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <15.5,33>, <19.5,33>, <17.5,35>, <15.5,33>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <20.5,33>, <24.5,33>, <22.5,35>, <20.5,33>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Thirteenth
prism {
   0, 1.5, 4
   <-2,36>, <2,36>, <0,38>, <-2,36>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <3,36>, <7,36>, <5,38>, <3,36>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <8,36>, <12,36>, <10,38>, <8,36>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <13,36>, <17,36>, <15,38>, <13,36>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <18,36>, <22,36>, <20,38>, <18,36>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <23,36>, <27,36>, <25,38>, <23,36>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Fourteenth row
prism {
   0, 1.5, 4
   <0.5,39>, <4.5,39>, <2.5,41>, <0.5,39>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <5.5,39>, <9.5,39>, <7.5,41>, <5.5,39>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <10.5,39>, <14.5,39>, <12.5,41>, <10.5,39>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <15.5,39>, <19.5,39>, <17.5,41>, <15.5,39>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <20.5,39>, <24.5,39>, <22.5,41>, <20.5,39>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Fifteenth row
prism {
   0, 1.5, 4
   <-2,42>, <2,42>, <0,44>, <-2,42>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <3,42>, <7,42>, <5,44>, <3,42>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <8,42>, <12,42>, <10,44>, <8,42>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <13,42>, <17,42>, <15,44>, <13,42>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <18,42>, <22,42>, <20,44>, <18,42>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <23,42>, <27,42>, <25,44>, <23,42>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

// Sixteenth row
prism {
   0, 1.5, 4
   <0.5,45>, <4.5,45>, <2.5,47>, <0.5,45>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <5.5,45>, <9.5,45>, <7.5,47>, <5.5,45>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <10.5,45>, <14.5,45>, <12.5,47>, <10.5,45>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <15.5,45>, <19.5,45>, <17.5,47>, <15.5,45>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}

prism {
   0, 1.5, 4
   <20.5,45>, <24.5,45>, <22.5,47>, <20.5,45>
   rotate <-90,0,0>
   translate <0,0,1.25>
   texture { WallTexture }
}
