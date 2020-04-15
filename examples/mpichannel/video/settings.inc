//=================================================================================================
/*
 *  POV-Ray include file for the mpiimpact example
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


/*===================================
== Camera position and orientation ==
===================================*/
// Camera aspect ratio: 4/3
//  => Up-angle=26,565, Up/Down-angle=53,13, Right-angle=33,69, Left/Right-angle=67,38

#declare lx = 150;
#declare ly = 63;
#declare lz = 21;
//#declare pos  = <0.5*lx,lz,0>;
//#declare pos  = <0.5*lx,0.5*lz,0>;
#declare look = <0.5*lx,0.5*lz,0.5*ly>;
#declare fact = -9;
//#declare fact = -2.6;

#declare yyy = 60;
#declare xxx = 90;
#if (clock < yyy)
   #declare pos  = <0.5*lx,0.5*lz + sin(0.25*pi)*0.5*lz, 0.5*ly - cos(0.25*pi)*0.5*lz>;
   #declare upv  = <0,cos(0.25*pi),sin(0.25*pi)>;
#else
   #if (clock < yyy+xxx)
      #declare pos  = <0.5*lx,0.5*lz + sin(0.25*pi+(clock-yyy)/xxx*0.25*pi)*0.5*lz, 0.5*ly - cos(0.25*pi+(clock-yyy)/xxx*0.25*pi)*0.5*lz>;
      #declare upv  = <0,cos(0.25*pi+(clock-yyy)/xxx*0.25*pi),sin(0.25*pi+(clock-yyy)/xxx*0.25*pi)>;
   #else
      #declare pos  = <0.5*lx,lz,0.5*ly>;
      #declare upv  = <0,0,1>;
   #end
#end


camera { right 16/9*x location pos+(look-pos)*fact look_at look sky upv }


/*=================
== Light sources ==
=================*/

light_source { pos+(look-pos)*fact color rgb <1,1,1> }


/*==============
== Background ==
==============*/

background { color rgbf <0,0,0,0> }


#declare TBoundary =
texture {
	pigment {
		color rgbf <0.5,0.5,0.5,0>
	}
	finish {
		ambient 0.2
	}
};

#declare TParticle0 =
texture {
   pigment {
      color rgb <118,19,0>/255
   }
   finish {
      ambient 0.2
   }
};

#declare TParticle1 =
texture {
   pigment {
      color rgb <178,100,0>/255
   }
   finish {
      ambient 0.2
   }
};

#declare TParticle2 =
texture {
   pigment {
      color rgb <237,172,10>/255
   }
   finish {
      ambient 0.2
   }
};

#declare TParticle3 =
texture {
   pigment {
      color rgb <255,219,112>/255
   }
   finish {
      ambient 0.2
   }
};

#declare TGreen =
texture {
   pigment {
      color rgbf <0,1,0,0>
   }
   finish {
      ambient 0.2
   }
};

#declare TObstacle      = TBoundary;
#declare TUnknown       = TGreen;


// Cylinder
merge {
   cylinder {
      <-10.5,0,0>, <10.5,0,0>, 10
      texture {
         pigment {
            //color rgbf <0.7,0.7,0.7,0>
            color rgb <45,4,0>/255
         }
      }
   }
   rotate <90,-0,-90>
   translate <40,10.5,0.5*ly>
}

// Box
box {
   <-5,-10,-10.5>, <5,10,10.5>
   texture {
      pigment {
         //color rgbf <0.7,0.7,0.7,0>
         color rgb <45,4,0>/255
      }
   }
   rotate <90,-0,0>
   translate <45,10.5,31.5>
}
