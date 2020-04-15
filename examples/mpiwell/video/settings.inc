//=================================================================================================
/*
 *  POV-Ray include file for the mpiwell example
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


/*================
== Declarations ==
================*/

// Process 0: Black texture
#declare Texture0 =
texture {
   pigment {
      color rgbf <0.1,0.1,0.1,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 1: Orange texture
#declare Texture1 =
texture {
   pigment {
      color rgbf <1,0.6,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 2: Red texture
#declare Texture2 =
texture {
   pigment {
      color rgbf <1,0,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 3: Purple texture
#declare Texture3 =
texture {
   pigment {
      color rgbf <0.75,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 4: Blue texture
#declare Texture4 =
texture {
   pigment {
      color rgbf <0,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 5: Green texture
#declare Texture5 =
texture {
   pigment {
      color rgbf <0,0.7,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 6: Yellow texture
#declare Texture6 =
texture {
   pigment {
      color rgbf <1,1,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 7: Blue texture
#declare Texture7 =
texture {
   pigment {
      color rgbf <0,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 8: Cyan texture
#declare Texture8 =
texture {
   pigment {
      color rgbf <0,0.5,0.8,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 9: Turquoise texture
#declare Texture9 =
texture {
   pigment {
      color rgbf <0,0.8,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 10: Green texture
#declare Texture10 =
texture {
   pigment {
      color rgbf <0,0.7,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 11: Dulcet texture
#declare Texture11 =
texture {
   pigment {
      color rgbf <0.5,1,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 12: Yellow texture
#declare Texture12 =
texture {
   pigment {
      color rgbf <1,1,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 13: Yolk texture
#declare Texture13 =
texture {
   pigment {
      color rgbf <1,0.7,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 14: Orange texture
#declare Texture14 =
texture {
   pigment {
      color rgbf <1,0.4,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 15: Red texture
#declare Texture15 =
texture {
   pigment {
      color rgbf <1,0,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 16: Magenta texture
#declare Texture16 =
texture {
   pigment {
      color rgbf <0.75,0,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 17: Purple texture
#declare Texture17 =
texture {
   pigment {
      color rgbf <0.75,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 18: Violet texture
#declare Texture18 =
texture {
   pigment {
      color rgbf <0.4,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 19
#declare Texture19 =
texture {
   pigment {
      color rgbf <1,0.7,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 20
#declare Texture20 =
texture {
   pigment {
      color rgbf <1,0.47,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 21
#declare Texture21 =
texture {
   pigment {
      color rgbf <1,0.24,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 22
#declare Texture22 =
texture {
   pigment {
      color rgbf <1,0,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 23
#declare Texture23 =
texture {
   pigment {
      color rgbf <0.8,0,0.3,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 24
#declare Texture24 =
texture {
   pigment {
      color rgbf <0.75,0,0.6,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 25
#declare Texture25 =
texture {
   pigment {
      color rgbf <0.75,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 26
#declare Texture26 =
texture {
   pigment {
      color rgbf <0.5,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 27
#declare Texture27 =
texture {
   pigment {
      color rgbf <0.25,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 28
#declare Texture28 =
texture {
   pigment {
      color rgbf <0,0,1,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 29
#declare Texture29 =
texture {
   pigment {
      color rgbf <0,0.24,0.66,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 30
#declare Texture30 =
texture {
   pigment {
      color rgbf <0,0.48,0.33,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 31
#declare Texture31 =
texture {
   pigment {
      color rgbf <0,0.7,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 32
#declare Texture32 =
texture {
   pigment {
      color rgbf <0.33,0.8,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 33
#declare Texture33 =
texture {
   pigment {
      color rgbf <0.66,0.9,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 34
#declare Texture34 =
texture {
   pigment {
      color rgbf <1,1,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 35
#declare Texture35 =
texture {
   pigment {
      color rgbf <1,0.66,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 36
#declare Texture36 =
texture {
   pigment {
      color rgbf <1,0.33,0,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 37
#declare Texture37 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 38
#declare Texture38 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 39
#declare Texture39 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 40
#declare Texture40 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 41
#declare Texture41 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 42
#declare Texture42 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 43
#declare Texture43 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 44
#declare Texture44 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 45
#declare Texture45 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 46
#declare Texture46 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 47
#declare Texture47 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 48
#declare Texture48 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 49
#declare Texture49 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 50
#declare Texture50 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 51
#declare Texture51 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 52
#declare Texture52 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 53
#declare Texture53 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 54
#declare Texture54 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 55
#declare Texture55 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 56
#declare Texture56 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 57
#declare Texture57 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 58
#declare Texture58 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 59
#declare Texture59 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 60
#declare Texture60 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 61
#declare Texture61 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 62
#declare Texture62 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 63
#declare Texture63 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 64
#declare Texture64 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 65
#declare Texture65 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 66
#declare Texture66 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 67
#declare Texture67 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 68
#declare Texture68 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 69
#declare Texture69 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 70
#declare Texture70 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 71
#declare Texture71 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 72
#declare Texture72 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 73
#declare Texture73 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 74
#declare Texture74 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 75
#declare Texture75 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 76
#declare Texture76 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 77
#declare Texture77 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 78
#declare Texture78 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 79
#declare Texture79 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 80
#declare Texture80 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 81
#declare Texture81 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 82
#declare Texture82 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 83
#declare Texture83 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 84
#declare Texture84 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 85
#declare Texture85 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 86
#declare Texture86 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 87
#declare Texture87 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 88
#declare Texture88 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 89
#declare Texture89 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 90
#declare Texture90 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};

// Process 91
#declare Texture91 =
texture {
   pigment {
      color rgbf <0.5,0.5,0.5,0>
   }
   finish {
      ambient 0.2
   }
};


/*=================
== Light sources ==
=================*/

light_source {
   <10,350,-320>
   color rgbf <1,1,1,0>
   area_light
   <3,0,0>, <0,1.81525,2.38849>, 3, 3
}

light_source {
   <0,400,0>
   color rgbf <1,1,1,0>
   parallel
   point_at <0,0,0>
   shadowless
}
