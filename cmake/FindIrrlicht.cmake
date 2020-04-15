#==================================================================================================
#
#  CMake detection script for the Irrlicht library
#
#  Copyright (C) 2009 Klaus Iglberger
#
#  This file is part of pe.
#
#  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
#  General Public License as published by the Free Software Foundation, either version 3 of the
#  License, or (at your option) any later version.
#
#  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along with pe. If not,
#  see <http://www.gnu.org/licenses/>.
#
#==================================================================================================

FIND_PATH( IRRLICHT_INCLUDE_DIR irrlicht.h PATHS /usr/include/irrlicht /usr/local/include/irrlicht )

FIND_LIBRARY( IRRLICHT_LIBRARY NAMES Irrlicht PATHS /usr/lib /usr/lib64 /usr/local/lib /usr/local/lib64 )

IF( IRRLICHT_LIBRARY AND IRRLICHT_INCLUDE_DIR )
   SET( IRRLICHT_FOUND TRUE )
ENDIF( IRRLICHT_LIBRARY AND IRRLICHT_INCLUDE_DIR )
