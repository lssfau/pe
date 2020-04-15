#==================================================================================================
#
#  CMakeLists for the pe physics engine
#
#  Copyright (C) 2009 Klaus Iglberger
#                2012 Tobias Preclik
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

CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( pe )

IF( POLICY CMP0015 )
   CMAKE_POLICY( SET CMP0015 NEW )
ENDIF()

#==================================================================================================
#
# Definition of global options
#
#==================================================================================================

SET( CMAKE_BUILD_TYPE RELEASE CACHE STRING "The default build type for the whole project is RELEASE." )
SET( LIBRARY_TYPE     STATIC  CACHE STRING "Build type of library (STATIC/SHARED/BOTH)."              )
SET( BLAS             OFF     CACHE BOOL   "Build support for BLAS."                                  )
SET( MPI              OFF     CACHE BOOL   "Build support for MPI."                                   )
SET( MPI_PREINCLUDE   OFF     CACHE BOOL   "Include mpi.h ahead of all other headers."                )
SET( OPENCL           OFF     CACHE BOOL   "Build support for OpenCL."                                )
SET( IRRLICHT         OFF     CACHE BOOL   "Build support for Irrlicht engine."                       )
SET( EXAMPLES         OFF     CACHE BOOL   "Build examples."                                          )
SET( PACKAGE_NAME     pe                                                                              )
SET( PACKAGE_VERSION  trunk                                                                           )
SET( BOOST_INCLUDEDIR ""      CACHE PATH   "The boost include directory."                             )
SET( BOOST_LIBRARYDIR ""      CACHE PATH   "The boost library directory."                             )
SET( PREPROCESSOR_FLAGS ""    CACHE PATH   "Additional preprocessor flags."                           )
SET( COMPILER_FLAGS   ""      CACHE PATH   "Additional compiler flags."                               )
SET( LINKER_FLAGS     ""      CACHE PATH   "Additional linker flags when linking the examples."       )
SET( MOBILE_INFINITE  OFF     CACHE BOOL   "Build with support for mobile infinite bodies."           )
SET( pe_PRECISION     ""      CACHE STRING "Floating point data type of the physics engine."          )
SET( pe_COMPLEMENTARITY_SOLVER "" CACHE STRING "Selection of the complementarity solver for LCP-based collision response algorithms." )
SET( pe_COARSE_COLLISION_DETECTOR "" CACHE STRING "Coarse collision detection algorithm."             )
SET( pe_FINE_COLLISION_DETECTOR "" CACHE STRING "Fine collision detection algorithm."                 )
SET( pe_BATCH_GENERATOR ""    CACHE STRING "Batch generation algorithm."                              )
SET( pe_CONSTRAINT_SOLVER ""  CACHE STRING "Collision response algorithm."                            )

SET_PROPERTY( CACHE pe_COMPLEMENTARITY_SOLVER PROPERTY STRINGS "" Lemke PGS CPG )
SET_PROPERTY( CACHE pe_PRECISION PROPERTY STRINGS "" float double "long double" )
SET_PROPERTY( CACHE pe_COARSE_COLLISION_DETECTOR PROPERTY STRINGS "" ExhaustiveSearch HashGrids SweepAndPrune )
SET_PROPERTY( CACHE pe_FINE_COLLISION_DETECTOR PROPERTY STRINGS "" MaxContacts )
SET_PROPERTY( CACHE pe_BATCH_GENERATOR PROPERTY STRINGS "" SingleBatch UnionFind )
SET_PROPERTY( CACHE pe_CONSTRAINT_SOLVER PROPERTY STRINGS "" FrictionlessSolver BoxFrictionSolver ConeFrictionSolver PolyhedralFrictionSolver DEMSolver DEMSolverObsolete FFDSolver HardContactSemiImplicitTimesteppingSolvers )

IF( OPENCL )
   GET_PROPERTY( pe_CONSTRAINT_SOLVER_STRINGS CACHE pe_CONSTRAINT_SOLVER PROPERTY STRINGS )
   SET_PROPERTY( CACHE pe_CONSTRAINT_SOLVER PROPERTY STRINGS ${pe_CONSTRAINT_SOLVER_STRINGS} OpenCLSolver )
ENDIF()

# Guide selection of boost libraries by FindBoost.cmake
SET( Boost_USE_STATIC_LIBS OFF CACHE BOOL "Force static linking of Boost libraries." )

# Boost threads is only available as a multithreaded version so force FindBoost.cmake to use the multithreaded versions.
SET( Boost_USE_MULTITHREADED ON )

# Setting the build type (default=release)
IF( NOT CMAKE_BUILD_TYPE )
   SET ( CMAKE_BUILD_TYPE "RELEASE" CACHE STRING "The default build type is RELEASE." FORCE )
ENDIF()

# Setting proper options for the compiler
IF( NOT CMAKE_BUILD_TYPE STREQUAL RELEASE AND NOT CMAKE_BUILD_TYPE STREQUAL DEBUG )
   MESSAGE( ERROR ": Unknown build type!" )
ENDIF()

# Setting compiler flags analogously to the configure script (and clear default compiler flags set
# by cmake for all compilers supported by the configure script to ensure compatibility)
IF( CMAKE_COMPILER_IS_GNUCXX )
   OPTION( PE_CXX_COMPILER_IS_GNU "Use GNU compiler" ON )
   SET( CMAKE_CXX_FLAGS "-Wall -Wextra -Wshadow -Woverloaded-virtual -Wno-unused-local-typedefs -ansi --param inline-unit-growth=150 --param max-inline-insns-single=500 --param large-function-growth=600" CACHE STRING "Compiler flags for GNU C++ compiler." FORCE )
ELSE()
   OPTION( PE_CXX_COMPILER_IS_GNU "Use GNU compiler" OFF )
ENDIF()
MARK_AS_ADVANCED( PE_CXX_COMPILER_IS_GNU )

IF( ${CMAKE_CXX_COMPILER} MATCHES "icpc" )
   OPTION( PE_CXX_COMPILER_IS_INTEL "Use Intel compiler" ON )
   SET( CMAKE_CXX_FLAGS "-Wall -Wshadow -Woverloaded-virtual -ansi" CACHE STRING "Compiler flags for Intel C++ compiler." FORCE )
ELSE()
   OPTION( PE_CXX_COMPILER_IS_INTEL "Use Intel compiler" OFF )
ENDIF()
MARK_AS_ADVANCED( PE_CXX_COMPILER_IS_INTEL )

IF( ${CMAKE_CXX_COMPILER} MATCHES "clang" OR "${CMAKE_CXX_COMPILER_ARG1}" MATCHES "clang" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" )
   OPTION( PE_CXX_COMPILER_IS_CLANG "Use clang compiler" ON )
   SET( CMAKE_CXX_FLAGS "-Wextra -Wshadow -Woverloaded-virtual" CACHE STRING "Compiler flags for Clang++ compiler." FORCE )
ELSE()
   OPTION( PE_CXX_COMPILER_IS_CLANG "Use clang compiler" OFF )
ENDIF()
MARK_AS_ADVANCED( PE_CXX_COMPILER_IS_CLANG )

IF( ${CMAKE_CXX_COMPILER} MATCHES "xlc" OR "${CMAKE_CXX_COMPILER_ARG1}" MATCHES "xlc" )
   OPTION( PE_CXX_COMPILER_IS_IBM "Use IBM compiler" ON )
ELSE()
   OPTION( PE_CXX_COMPILER_IS_IBM "Use IBM compiler" OFF )
ENDIF()
MARK_AS_ADVANCED( PE_CXX_COMPILER_IS_IBM )

IF( MSVC )
   OPTION( PE_CXX_COMPILER_IS_MSVC "Use Visual Studio compiler" ON )
ELSE()
   OPTION( PE_CXX_COMPILER_IS_MSVC "Use Visual Studio compiler" OFF )
ENDIF()
MARK_AS_ADVANCED( PE_CXX_COMPILER_IS_MSVC )

# Setting user compiler and linker flags
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PREPROCESSOR_FLAGS} ${COMPILER_FLAGS}" )
SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${LINKER_FLAGS}" )

# Assert that static library is built
IF ( LIBRARY_TYPE STREQUAL SHARED OR LIBRARY_TYPE STREQUAL BOTH )
   SET( PE_SHARED_LIB_BUILD BOOL ON )
   SET( DEFINES "${DEFINES} PE_SHARED_LIB_BUILD=1" )
   MESSAGE( WARNING ": Shared library builds are currently experimental." )
ENDIF()

#GCC 5+ ABI selection
IF( PE_CXX_COMPILER_IS_GNU )
   IF( NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.0.0" )
      OPTION ( PE_USE_CPP11_ABI "On GCC 5+ use the C++11 ABI" ON )
      IF( PE_USE_CPP11_ABI )
         SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=1" )
      ELSE()
         SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=0" )
      ENDIF()
   ENDIF()
ENDIF()

IF( MOBILE_INFINITE )
   SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMOBILE_INFINITE=1" )
ENDIF()



#==================================================================================================
#
# Visual Studio Setup
#
#==================================================================================================

IF ( PE_CXX_COMPILER_IS_MSVC )
   string( REGEX REPLACE "[/-]W[0-4]" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ) # remove default warning flags
   
   OPTION ( PE_GROUP_PROJECTS  "Flag if the projects are grouped or in a flat hierarchy"  ON )
   OPTION ( PE_GROUP_FILES     "Flag if the files are grouped or in a flat hierarchy"     ON )
   set_property ( GLOBAL PROPERTY USE_FOLDERS ${PE_GROUP_PROJECTS} )
   
   ADD_DEFINITIONS ( "-DNOMINMAX" )                # Disable Min/Max-Macros
   ADD_DEFINITIONS ( "-D_WIN32_WINNT=0x501" )      # Minimum Windows versions is Windows XP
   ADD_DEFINITIONS ( "-DWINVER=0x501" )            # Minimum Windows versions is Windows XP
   ADD_DEFINITIONS ( "-D_CRT_SECURE_NO_WARNINGS" ) # disable warnings promoting Microsoft's security enhanced CRT
   ADD_DEFINITIONS ( "-D_SCL_SECURE_NO_WARNINGS" ) # disable warnings triggered by Microsoft's checked iterators
   
   SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -MP" ) # enable multi-threaded compiling
   
ENDIF ( )




#==================================================================================================
#
# 3rd party software
#
#==================================================================================================

SET( CMAKE_MODULE_PATH ${pe_SOURCE_DIR}/cmake )

# Configuration of the Boost library
SET( pe_REQUIRED_MIN_BOOST_VERSION "1.48" )

OPTION( PE_BUILD_WITH_BOOST_THREAD "Build with boost thread library support" ON )
IF( PE_CXX_COMPILER_IS_IBM )
   SET( PE_BUILD_WITH_BOOST_THREAD OFF CACHE BOOL "Build with boost thread library support" FORCE )
ENDIF()

IF( PE_BUILD_WITH_BOOST_THREAD )
   SET( pe_REQUIRED_BOOST_COMPONENTS thread system filesystem program_options )
ELSE()
   SET( pe_REQUIRED_BOOST_COMPONENTS system filesystem program_options )
ENDIF()

SET( Boost_ADDITIONAL_VERSIONS
     "1.45" "1.45.0" "1.46" "1.46.0" "1.46.1" "1.47" "1.47.0" "1.48" "1.48.0"
     "1.49" "1.49.0" "1.50" "1.50.0" "1.51" "1.51.0" "1.52" "1.52.0" "1.53"
     "1.53.0" "1.54" "1.54.0" "1.55" "1.55.0" "1.56" "1.56.0" "1.57" "1.57.0"
     "1.58" "1.58.0" "1.59" "1.59.0" "1.60" "1.60.0" "1.61" "1.61.0"
     "1.62" "1.62.0" "1.63" "1.63.0" "1.64" "1.64.0" )

FIND_PACKAGE( Boost ${pe_REQUIRED_MIN_BOOST_VERSION} COMPONENTS ${pe_REQUIRED_BOOST_COMPONENTS} REQUIRED )

IF( Boost_FOUND )
   INCLUDE_DIRECTORIES( SYSTEM ${Boost_INCLUDE_DIRS} )
   LINK_DIRECTORIES   ( ${Boost_LIBRARY_DIRS} )
   LINK_LIBRARIES     ( ${Boost_LIBRARIES}    )
   # Disable Boost auto-linking since it conflicts with cmake:
   ADD_DEFINITIONS    ( -DBOOST_ALL_NO_LIB    )
ENDIF()

# Configuration of the BLAS library
IF( BLAS )
   SET( BLAS_INCLUDE_DIR "" CACHE STRING "BLAS include path." )
   SET( BLAS_LIBRARY_1   "" CACHE STRING "BLAS library 1."    )
   SET( BLAS_LIBRARY_2   "" CACHE STRING "BLAS library 2."    )
   SET( BLAS_LIBRARY_3   "" CACHE STRING "BLAS library 3."    )
   INCLUDE_DIRECTORIES( SYSTEM ${BLAS_INCLUDE_DIR} )
   LINK_LIBRARIES     ( ${BLAS_LIBRARY_1} ${BLAS_LIBRARY_2} ${BLAS_LIBRARY_3} )
   SET( HAVE_BLAS BOOL ON )
   SET( DEFINES "${DEFINES} HAVE_BLAS=1" )
ELSE()
   SET( DEFINES "${DEFINES} HAVE_BLAS=0" )
ENDIF()

# Configuration of the Irrlicht library
IF( IRRLICHT )
   FIND_PACKAGE( Irrlicht REQUIRED )

   IF( NOT IRRLICHT_FOUND )
      MESSAGE( ERROR ": Irrlicht engine not found!" )
   ENDIF()

   INCLUDE_DIRECTORIES( SYSTEM ${IRRLICHT_INCLUDE_DIR} )
   LINK_DIRECTORIES   ( ${IRRLICHT_LIBRARY_DIR} )
   LINK_LIBRARIES     ( ${IRRLICHT_LIBRARY}     )

   SET( MEDIA_PATH "${PROJECT_SOURCE_DIR}/media/" )
   SET( HAVE_IRRLICHT BOOL ON )
   SET( DEFINES "${DEFINES} PE_MEDIA_PATH=${MEDIA_PATH} HAVE_IRRLICHT=1" )
ELSE()
   SET( DEFINES "${DEFINES} PE_MEDIA_PATH= HAVE_IRRLICHT=0" )
ENDIF()

# Configuration of the MPI library
IF( MPI )
   FIND_PACKAGE( MPI )
   IF( MPI_FOUND )
      INCLUDE_DIRECTORIES( SYSTEM ${MPI_CXX_INCLUDE_PATH} )
      LINK_DIRECTORIES( ${MPI_CXX_LIBRARY_PATH} )
      LINK_LIBRARIES( ${MPI_CXX_LIBRARIES} )
      SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS}" )
      SET( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MPI_CXX_LINK_FLAGS}" )
      SET( HAVE_MPI BOOL ON )
      SET( DEFINES "${DEFINES} HAVE_MPI=1" )
   ELSE()
      SET( DEFINES "${DEFINES} HAVE_MPI=0" )
   ENDIF( MPI_FOUND )
ELSE()
   SET( DEFINES "${DEFINES} HAVE_MPI=0" )
ENDIF()

# Configuration of the OpenCL library
IF( OPENCL )
   SET( OPENCL_KERNEL_PATH "${PROJECT_SOURCE_DIR}/src/core/opencl/" )
   SET( HAVE_OPENCL BOOL ON )
   SET( DEFINES "${DEFINES} PE_OPENCL_KERNEL_PATH=${OPENCL_KERNEL_PATH} HAVE_OPENCL=1" )
ELSE()
   SET( DEFINES "${DEFINES} PE_OPENCL_KERNEL_PATH= HAVE_OPENCL=0" )
ENDIF()

# Create config.h from template and force inclusion
CONFIGURE_FILE( ${pe_SOURCE_DIR}/cmake/config.h.in ${PROJECT_BINARY_DIR}/config.h )
IF( MPI AND MPI_PREINCLUDE )
	FILE( APPEND ${PROJECT_BINARY_DIR}/config.h "#include <mpi.h>" )
ENDIF()

IF( PE_CXX_COMPILER_IS_MSVC )
   SET( CMAKE_CXX_FLAGS "\"/FI ${PROJECT_BINARY_DIR}/config.h\" ${CMAKE_CXX_FLAGS}" )
ELSEIF( PE_CXX_COMPILER_IS_IBM )
   SET( CMAKE_CXX_FLAGS "-qinclude=${PROJECT_BINARY_DIR}/config.h ${CMAKE_CXX_FLAGS}" )
ELSE()
   SET( CMAKE_CXX_FLAGS "-include ${PROJECT_BINARY_DIR}/config.h ${CMAKE_CXX_FLAGS}" )
ENDIF()




#==================================================================================================
#
# Build of the physics engine modules
#
#==================================================================================================

FILE( GLOB_RECURSE CONFIG_SOURCES_ALL   src/config/*.cpp   pe/config/*.h   )
FILE( GLOB_RECURSE CORE_SOURCES_ALL     src/core/*.cpp     pe/core/*.h     )
FILE( GLOB_RECURSE IRRLICHT_SOURCES_ALL src/irrlicht/*.cpp pe/irrlicht/*.h )
FILE( GLOB_RECURSE MATH_SOURCES_ALL     src/math/*.cpp     pe/math/*.h     )
FILE( GLOB_RECURSE OPENDX_SOURCES_ALL   src/opendx/*.cpp   pe/opendx/*.h   )
FILE( GLOB_RECURSE POVRAY_SOURCES_ALL   src/povray/*.cpp   pe/povray/*.h   )
FILE( GLOB_RECURSE SUPPORT_SOURCES_ALL  src/support/*.cpp  pe/support/*.h  )
FILE( GLOB_RECURSE SYSTEM_SOURCES_ALL   src/system/*.cpp   pe/system/*.h   )
FILE( GLOB_RECURSE UTIL_SOURCES_ALL     src/util/*.cpp     pe/util/*.h     )
FILE( GLOB_RECURSE VTK_SOURCES_ALL      src/vtk/*.cpp      pe/vtk/*.h      )

SET( PE_SOURCES ${CONFIG_SOURCES_ALL} ${CORE_SOURCES_ALL} ${MATH_SOURCES_ALL} ${OPENDX_SOURCES_ALL} ${POVRAY_SOURCES_ALL} ${SUPPORT_SOURCES_ALL} ${SYSTEM_SOURCES_ALL} ${UTIL_SOURCES_ALL} ${VTK_SOURCES_ALL} )

IF( IRRLICHT )
   SET( PE_SOURCES ${PE_SOURCES} ${IRRLICHT_SOURCES_ALL} )
ENDIF()

# Make sure all objects depend on the forced include.
SET_SOURCE_FILES_PROPERTIES( ${PE_SOURCES} PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
SET( PE_SOURCES ${PE_SOURCES} ${PROJECT_BINARY_DIR}/config.h )

INCLUDE_DIRECTORIES( . )

SET( LIBRARY_OUTPUT_PATH lib )

IF( LIBRARY_TYPE STREQUAL STATIC OR LIBRARY_TYPE STREQUAL BOTH )
   ADD_LIBRARY( pe_static STATIC ${PE_SOURCES} )
   SET_TARGET_PROPERTIES( pe_static PROPERTIES OUTPUT_NAME pe )
   # The library target "pe_static" will be named "pe.lib" with Microsoft tools.
   # This conflicts with the "pe.lib" import library corresponding to "pe.dll",
   # so we add a "lib" prefix (which is default on other platforms anyway):
   SET_TARGET_PROPERTIES( pe_static PROPERTIES PREFIX "lib" )

   ADD_LIBRARY( libpe ALIAS pe_static )
ENDIF()

IF( LIBRARY_TYPE STREQUAL SHARED OR LIBRARY_TYPE STREQUAL BOTH )
   ADD_LIBRARY( pe_shared SHARED ${PE_SOURCES} )
   IF( CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER} MATCHES "icpc" )
      # CXXFLAGS automatically also used for linking
      SET_TARGET_PROPERTIES( pe_shared PROPERTIES COMPILE_FLAGS "-fPIC -fvisibility=hidden -fvisibility-inlines-hidden" )
   ENDIF()
   SET_TARGET_PROPERTIES( pe_shared PROPERTIES OUTPUT_NAME pe )
   SET_TARGET_PROPERTIES( pe_shared PROPERTIES COMPILE_DEFINITIONS PE_SHARED_LIB_SELECT_EXPORTS )

   IF( LIBRARY_TYPE STREQUAL SHARED )
      ADD_LIBRARY( libpe ALIAS pe_shared )
   ENDIF()
ENDIF()




#==================================================================================================
#
# Build of the examples
#
#=================================================================================================

IF( EXAMPLES )

   # General settings for all examples
   INCLUDE_DIRECTORIES( . )
   LINK_DIRECTORIES( "${PROJECT_BINARY_DIR}" )
   LINK_LIBRARIES( ${Boost_LIBRARIES} )

   IF( LIBRARY_TYPE STREQUAL STATIC )
      LINK_LIBRARIES( pe_static )
   ELSE()
      LINK_LIBRARIES( pe_shared )
   ENDIF()

   IF( IRRLICHT )
      LINK_LIBRARIES( Irrlicht GLU Xxf86vm Xext X11 )
   ENDIF()

   # Boxstack example
   ADD_EXECUTABLE( boxstack examples/boxstack/BoxStack.cpp )
   SET_SOURCE_FILES_PROPERTIES( examples/boxstack/BoxStack.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/boxstack )
   SET_TARGET_PROPERTIES( boxstack PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/boxstack/ )
   SET_TARGET_PROPERTIES( boxstack PROPERTIES OUTPUT_NAME boxstack )
   ADD_DEPENDENCIES( boxstack libpe )

   # Chain example
   ADD_EXECUTABLE( chain examples/chain/Chain.cpp )
   SET_SOURCE_FILES_PROPERTIES( examples/chain/Chain.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/chain )
   SET_TARGET_PROPERTIES( chain PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/chain/ )
   SET_TARGET_PROPERTIES( chain PROPERTIES OUTPUT_NAME chain )
   ADD_DEPENDENCIES( chain libpe )

   # Newton's cradle example
   ADD_EXECUTABLE( cradle examples/cradle/Cradle.cpp )
   SET_SOURCE_FILES_PROPERTIES( examples/cradle/Cradle.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/cradle )
   SET_TARGET_PROPERTIES( cradle PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/cradle/ )
   SET_TARGET_PROPERTIES( cradle PROPERTIES OUTPUT_NAME cradle )
   ADD_DEPENDENCIES( cradle libpe )

   # Domino example
   ADD_EXECUTABLE( domino examples/domino/Domino.cpp )
   SET_SOURCE_FILES_PROPERTIES( examples/domino/Domino.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/domino )
   SET_TARGET_PROPERTIES( domino PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/domino/ )
   SET_TARGET_PROPERTIES( domino PROPERTIES OUTPUT_NAME domino )
   ADD_DEPENDENCIES( domino libpe )

   # Granular example
   ADD_EXECUTABLE( granular examples/granular/Granular.cpp )
   SET_SOURCE_FILES_PROPERTIES( examples/granular/Granular.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/granular )
   SET_TARGET_PROPERTIES( granular PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/granular/ )
   SET_TARGET_PROPERTIES( granular PROPERTIES OUTPUT_NAME granular )
   ADD_DEPENDENCIES( granular libpe )

   # Lattice example
   IF( pe_CONSTRAINT_SOLVER STREQUAL HardContactSemiImplicitTimesteppingSolvers OR pe_CONSTRAINT_SOLVER STREQUAL "" )
      ADD_EXECUTABLE( lattice examples/lattice/Lattice.cpp )
      SET_SOURCE_FILES_PROPERTIES( examples/lattice/Lattice.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/lattice )
      SET_TARGET_PROPERTIES( lattice PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/lattice/ )
      SET_TARGET_PROPERTIES( lattice PROPERTIES OUTPUT_NAME lattice )
      ADD_DEPENDENCIES( lattice libpe )
   ENDIF()

   # Nano example
   ADD_EXECUTABLE( nano examples/nano/Nano.cpp )
   SET_SOURCE_FILES_PROPERTIES( examples/nano/Nano.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/nano )
   SET_TARGET_PROPERTIES( nano PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/nano/ )
   SET_TARGET_PROPERTIES( nano PROPERTIES OUTPUT_NAME nano )
   ADD_DEPENDENCIES( nano libpe )

   # Well example
   ADD_EXECUTABLE( well examples/well/Well.cpp )
   SET_SOURCE_FILES_PROPERTIES( examples/well/Well.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
   FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/well )
   SET_TARGET_PROPERTIES( well PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/well/ )
   SET_TARGET_PROPERTIES( well PROPERTIES OUTPUT_NAME well )
   ADD_DEPENDENCIES( well libpe )

   # Shaker example
   IF( MOBILE_INFINITE )
      ADD_EXECUTABLE( shaker examples/shaker/Shaker.cpp )
      SET_SOURCE_FILES_PROPERTIES( examples/shaker/Shaker.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/shaker )
      SET_TARGET_PROPERTIES( shaker PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/shaker/ )
      SET_TARGET_PROPERTIES( shaker PROPERTIES OUTPUT_NAME shaker )
      ADD_DEPENDENCIES( shaker libpe )
   ENDIF()

   IF( MPI AND MPI_CXX_FOUND )
      # MPI-Brazil example
      IF( pe_CONSTRAINT_SOLVER STREQUAL HardContactSemiImplicitTimesteppingSolvers OR pe_CONSTRAINT_SOLVER STREQUAL "" )
         ADD_EXECUTABLE( mpibrazil examples/mpibrazil/MPIBrazil.cpp )
         SET_SOURCE_FILES_PROPERTIES( examples/mpibrazil/MPIBrazil.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
         FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpibrazil )
         SET_TARGET_PROPERTIES( mpibrazil PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpibrazil/ )
         SET_TARGET_PROPERTIES( mpibrazil PROPERTIES OUTPUT_NAME mpibrazil )
         ADD_DEPENDENCIES( mpibrazil libpe )
      ENDIF()

      # MPI-Channel example
      IF( pe_CONSTRAINT_SOLVER STREQUAL HardContactSemiImplicitTimesteppingSolvers OR pe_CONSTRAINT_SOLVER STREQUAL "" )
         ADD_EXECUTABLE( mpichannel examples/mpichannel/MPIChannel.cpp )
         SET_SOURCE_FILES_PROPERTIES( examples/mpichannel/MPIChannel.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
         FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpichannel )
         SET_TARGET_PROPERTIES( mpichannel PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpichannel/ )
         SET_TARGET_PROPERTIES( mpichannel PROPERTIES OUTPUT_NAME mpichannel )
         ADD_DEPENDENCIES( mpichannel libpe )
      ENDIF()

      # MPI-Cube example
      ADD_EXECUTABLE( mpicube examples/mpicube/MPICube.cpp )
      SET_SOURCE_FILES_PROPERTIES( examples/mpicube/MPICube.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpicube )
      SET_TARGET_PROPERTIES( mpicube PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpicube/ )
      SET_TARGET_PROPERTIES( mpicube PROPERTIES OUTPUT_NAME mpicube )
      ADD_DEPENDENCIES( mpicube libpe )

      # MPI-Granular example
      IF( pe_CONSTRAINT_SOLVER STREQUAL HardContactSemiImplicitTimesteppingSolvers OR pe_CONSTRAINT_SOLVER STREQUAL FFDSolver OR pe_CONSTRAINT_SOLVER STREQUAL "" )
         ADD_EXECUTABLE( mpigranular examples/mpigranular/MPIGranular.cpp )
         SET_SOURCE_FILES_PROPERTIES( examples/mpigranular/MPIGranular.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
         FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpigranular )
         SET_TARGET_PROPERTIES( mpigranular PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpigranular/ )
         SET_TARGET_PROPERTIES( mpigranular PROPERTIES OUTPUT_NAME mpigranular )
         ADD_DEPENDENCIES( mpigranular libpe )
      ENDIF()

      # MPI-Hourglass example
      IF( pe_CONSTRAINT_SOLVER STREQUAL HardContactSemiImplicitTimesteppingSolvers OR pe_CONSTRAINT_SOLVER STREQUAL FFDSolver OR pe_CONSTRAINT_SOLVER STREQUAL "" )
         ADD_EXECUTABLE( mpihourglass examples/mpihourglass/MPIHourglass.cpp )
         SET_SOURCE_FILES_PROPERTIES( examples/mpihourglass/MPIHourglass.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
         FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpihourglass )
         SET_TARGET_PROPERTIES( mpihourglass PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpihourglass/ )
         SET_TARGET_PROPERTIES( mpihourglass PROPERTIES OUTPUT_NAME mpihourglass )
         ADD_DEPENDENCIES( mpihourglass libpe )
      ENDIF()

      # MPI-Impact example
      IF( pe_CONSTRAINT_SOLVER STREQUAL HardContactSemiImplicitTimesteppingSolvers OR pe_CONSTRAINT_SOLVER STREQUAL DEMSolver OR pe_CONSTRAINT_SOLVER STREQUAL DEMSolverObsolete OR pe_CONSTRAINT_SOLVER STREQUAL "" )
         ADD_EXECUTABLE( mpiimpact examples/mpiimpact/MPIImpact.cpp )
         SET_SOURCE_FILES_PROPERTIES( examples/mpiimpact/MPIImpact.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
         FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpiimpact )
         SET_TARGET_PROPERTIES( mpiimpact PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpiimpact/ )
         SET_TARGET_PROPERTIES( mpiimpact PROPERTIES OUTPUT_NAME mpiimpact )
         ADD_DEPENDENCIES( mpiimpact libpe )
      ENDIF()

      # MPI-Labyrinth example
      IF( pe_CONSTRAINT_SOLVER STREQUAL HardContactSemiImplicitTimesteppingSolvers OR pe_CONSTRAINT_SOLVER STREQUAL FFDSolver OR pe_CONSTRAINT_SOLVER STREQUAL "" )
         ADD_EXECUTABLE( mpilabyrinth examples/mpilabyrinth/MPILabyrinth.cpp )
         SET_SOURCE_FILES_PROPERTIES( examples/mpilabyrinth/MPILabyrinth.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
         FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpilabyrinth )
         SET_TARGET_PROPERTIES( mpilabyrinth PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpilabyrinth/ )
         SET_TARGET_PROPERTIES( mpilabyrinth PROPERTIES OUTPUT_NAME mpilabyrinth )
         ADD_DEPENDENCIES( mpilabyrinth libpe )
      ENDIF()

      # MPI-Lattice example
      IF( pe_CONSTRAINT_SOLVER STREQUAL HardContactSemiImplicitTimesteppingSolvers OR pe_CONSTRAINT_SOLVER STREQUAL FFDSolver OR pe_CONSTRAINT_SOLVER STREQUAL "" )
         ADD_EXECUTABLE( mpilattice examples/mpilattice/MPILattice.cpp )
         SET_SOURCE_FILES_PROPERTIES( examples/mpilattice/MPILattice.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
         FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpilattice )
         SET_TARGET_PROPERTIES( mpilattice PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpilattice/ )
         SET_TARGET_PROPERTIES( mpilattice PROPERTIES OUTPUT_NAME mpilattice )
         ADD_DEPENDENCIES( mpilattice libpe )
      ENDIF()

      # MPI-LSS example
      ADD_EXECUTABLE( mpilss examples/mpilss/MPILSS.cpp )
      SET_SOURCE_FILES_PROPERTIES( examples/mpilss/MPILSS.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
      FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpilss )
      SET_TARGET_PROPERTIES( mpilss PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpilss/ )
      SET_TARGET_PROPERTIES( mpilss PROPERTIES OUTPUT_NAME mpilss )
      ADD_DEPENDENCIES( mpilss libpe )

      # MPI-Nano example
      IF( pe_CONSTRAINT_SOLVER STREQUAL HardContactSemiImplicitTimesteppingSolvers OR pe_CONSTRAINT_SOLVER STREQUAL FFDSolver OR pe_CONSTRAINT_SOLVER STREQUAL "" )
         ADD_EXECUTABLE( mpinano examples/mpinano/MPINano.cpp )
         SET_SOURCE_FILES_PROPERTIES( examples/mpinano/MPINano.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
         FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpinano )
         SET_TARGET_PROPERTIES( mpinano PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpinano/ )
         SET_TARGET_PROPERTIES( mpinano PROPERTIES OUTPUT_NAME mpinano )
         ADD_DEPENDENCIES( mpinano libpe )
      ENDIF()

      # MPI-Stair example
      IF( pe_CONSTRAINT_SOLVER STREQUAL HardContactSemiImplicitTimesteppingSolvers OR pe_CONSTRAINT_SOLVER STREQUAL FFDSolver OR pe_CONSTRAINT_SOLVER STREQUAL "" )
         ADD_EXECUTABLE( mpistair examples/mpistair/MPIStair.cpp )
         SET_SOURCE_FILES_PROPERTIES( examples/mpistair/MPIStair.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
         FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpistair )
         SET_TARGET_PROPERTIES( mpistair PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpistair/ )
         SET_TARGET_PROPERTIES( mpistair PROPERTIES OUTPUT_NAME mpistair )
         ADD_DEPENDENCIES( mpistair libpe )
      ENDIF()

      # MPI-Well example
      IF( pe_CONSTRAINT_SOLVER STREQUAL FFDSolver OR pe_CONSTRAINT_SOLVER STREQUAL "" )
      # ADD_EXECUTABLE( mpiwell examples/mpiwell/MPIWell.cpp )
      # SET_SOURCE_FILES_PROPERTIES( examples/mpiwell/MPIWell.cpp PROPERTIES OBJECT_DEPENDS ${PROJECT_BINARY_DIR}/config.h )
      # FILE( MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/examples/mpiwell )
      # SET_TARGET_PROPERTIES( mpiwell PROPERTIES RUNTIME_OUTPUT_DIRECTORY examples/mpiwell/ )
      # SET_TARGET_PROPERTIES( mpiwell PROPERTIES OUTPUT_NAME mpiwell )
      # ADD_DEPENDENCIES( mpiwell libpe )
      ENDIF()
   ENDIF()

ENDIF()




#==================================================================================================
#
# Visual Studio Source Tree Generation
#
#=================================================================================================

IF( MSVC )
   FILE( GLOB_RECURSE MSVC_SOURCES_ALL  RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp                   )
   FILE( GLOB_RECURSE MSVC_HEADERS_ALL  RELATIVE ${PROJECT_SOURCE_DIR} pe/*.h                      )
   FILE( GLOB_RECURSE MSVC_EXAMPLES_ALL RELATIVE ${PROJECT_SOURCE_DIR} examples/*.cpp examples/*.h )

   FOREACH( SOURCE IN LISTS MSVC_SOURCES_ALL MSVC_HEADERS_ALL MSVC_EXAMPLES_ALL )
      GET_FILENAME_COMPONENT( SOURCE_PATH "${SOURCE}" PATH )
      STRING( REPLACE "/" "\\" SOURCE_GROUP_NAME "${SOURCE_PATH}" )
      SOURCE_GROUP( "${SOURCE_GROUP_NAME}" FILES "${SOURCE}" )
   ENDFOREACH()

   SOURCE_GROUP( "" FILES ${PROJECT_BINARY_DIR}/config.h )
ENDIF()




#==================================================================================================
#
# Doxygen documentation
#
#==================================================================================================

FIND_PACKAGE( Doxygen )
IF( DOXYGEN_EXECUTABLE )
   FILE( WRITE ${PROJECT_BINARY_DIR}/doc/Doxyfile.inject "
PROJECT_NAME     = ${PACKAGE_NAME}
PROJECT_NUMBER   = ${PACKAGE_VERSION}
OUTPUT_DIRECTORY = ${PROJECT_BINARY_DIR}/doc
INPUT            = ${CMAKE_SOURCE_DIR}/pe ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/tutorial
IMAGE_PATH       = ${CMAKE_SOURCE_DIR}/doc/pics/jpg ${CMAKE_SOURCE_DIR}/doc/pics/png ${CMAKE_SOURCE_DIR}/doc/pics/eps
EXTRA_PACKAGES   = ${CMAKE_SOURCE_DIR}/doc/pe
PREDEFINED       = ${DEFINES} PE_PUBLIC= PE_PRIVATE= PE_PROTECTED= PE_NOTHROW=" )
   ADD_CUSTOM_TARGET( doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/Doxyfile WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/doc COMMENT "Generating documentation" )
ENDIF()

