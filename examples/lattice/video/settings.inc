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



/*===================================
== Camera position and orientation ==
===================================*/
// Camera aspect ratio: 4/3
//  => Up-angle=26,565, Up/Down-angle=53,13, Right-angle=33,69, Left/Right-angle=67,38

#declare lx = 10;
#declare ly = 10.3923;
#declare lz = 8.53197;
#declare pos  = <0.0*lx,0.5*lz,0.5*ly>;
#declare look = <0.5*lx,0.5*lz,0.5*ly>;

#declare fact = -2.5;

camera { right 16/9*x location pos+(look-pos)*fact look_at look sky <0,1,0> }
//camera { orthographic right 16/9*x location pos+(look-pos)*fact look_at look sky <0,1,0> angle 20 }


/*=================
== Light sources ==
=================*/

light_source { pos+(look-pos)*fact color rgb <1,1,1> }


/*==============
== Background ==
==============*/

background { color rgbf <0,0,0,0> }

/*================
== Declarations ==
================*/

// Blue texture
#declare TProcess0 =
texture {
   pigment {
      color rgbf <0,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Cyan texture
#declare TProcess1 =
texture {
   pigment {
      color rgbf <0,0.5,0.8,0>
   }
   finish {
      ambient 0.2
   }
};
