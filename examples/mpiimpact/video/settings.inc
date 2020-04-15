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

#include "colors.inc"
#include "metals.inc"
#include "textures.inc"
#include "stones.inc"

/*
camera {
	right x*16/9
	location <25.2,14.15,-29.1738>
	look_at <25.2,13.3,25.2>
	sky <0,1,0>
}
*/

/*
light_source {
	<25.2,14.15,-29.1738>, color rgbf <1,1,1,0>
	fade_distance 64
	fade_power 2
}
*/

#declare TextureSand = texture {
	T_Stone33
};

#declare TextureGround =
texture {
	pigment {
		color rgbf <0,0,0,0>
	}
	finish {
		ambient 0.2
	}
}

#declare TextureChrome =
texture {
	T_Chrome_5E
};

#declare TextureBlue =
texture {
   pigment {
      color rgbf <0,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

#declare TextureRed =
texture {
   pigment {
      color rgbf <1,0,0,0>
   }
   finish {
      ambient 0.2
   }
};

#declare TextureGreen =
texture {
   pigment {
      color rgbf <0,1,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Fake impactor
/*
#if (clock < 39)
	sphere {
		<0,0,0>, 4.158
		texture {
			TextureImpactor
		}
		rotate <90.0652,-0.108401,0.0842747>
		translate <25.2043,7.85597+(39-clock)*4.42945,25.2078>
	}
#end
*/

#declare Texture0 = TextureSand;
#declare Texture1 = Texture0;
#declare TextureImpactor = TextureChrome;

/*
#declare Texture0 = TextureBlue;
#declare Texture1 = TextureRed;
#declare TextureImpactor = TextureGreen;
*/
