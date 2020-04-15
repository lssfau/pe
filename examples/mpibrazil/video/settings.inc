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

#declare lx = 37.0125;
#declare ly = 37.0125;
#declare pos  = <0.5*lx,25,-5>;
#declare look = <0.5*lx,10,0.5*ly>;

#if (clock < 200)
   #declare fact = -1.8;
#else
   #if (clock < 600)
      #declare fact = -1.8+(clock - 200)/400*1.47;
   #else
      #if (clock < 2800)
         #declare fact = -0.33;
      #else
         #if (clock < 3200)
            #declare fact = -0.33+(clock - 2800)/400*0.33;
         #else
            #declare fact = 0;
         #end
      #end
   #end
#end

camera { right 16/9*x location pos+(look-pos)*fact look_at look sky <0,1,0> }


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
		color rgbf <0,0,0,0>
	}
	finish {
		ambient 0.2
	}
}

#declare TBlue =
texture {
   pigment {
      color rgbf <0,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

#declare TRed =
texture {
   pigment {
      color rgbf <1,0,0,0>
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

#declare TBoundaryXMin  = TBoundary;
#declare TBoundaryXMax  = TBoundary;
#declare TBoundaryYMin  = TBoundary;
#declare TBoundaryYMax  = TBoundary;
#declare TBoundaryZMin  = TBoundary;
#declare TBoundaryZMax  = TBoundary;

#declare TProcess0      = TBlue;
#declare TProcess1      = TRed;
#declare TUnknown       = TGreen;


// http://www.colourlovers.com/palette/3287088/mute
#declare TGranular0 =
texture {
   pigment { color rgb <220,214,178>/255 }
   finish  { diffuse 0.8                 }
};
#declare TGranular1 = TGranular0;
#declare TGranular2 =
texture {
   pigment { color rgb <178,185,140>/255 }
   finish  { diffuse 0.8                 }
};
#declare TGranular3 = TGranular2;
#declare TGranular4 =
texture {
   pigment { color rgb <100,139,134>/255 }
   finish  { diffuse 0.8                 }
};
#declare TGranular5 = TGranular4;
#declare TGranular6 =
texture {
   pigment { color rgb <89,90,90>/255 }
   finish  { diffuse 0.8                 }
};
#declare TGranular7 = TGranular6;
#declare TGranular8 =
texture {
   pigment { color rgb <78,58,67>/255 }
   finish  { diffuse 0.8                 }
};
#declare TGranular9 = TGranular8;
