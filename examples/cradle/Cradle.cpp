//=================================================================================================
/*!
 *  \file examples/Cradle.cpp
 *  \brief Example file for the pe physics engine
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cstring>
#include <iostream>
#include <string>
#include <pe/engine.h>
#include <pe/support.h>
using namespace pe;


//*************************************************************************************************
// Irrlicht includes
//*************************************************************************************************

#include <pe/irrlicht.h>
#if HAVE_IRRLICHT
using namespace pe::irrlicht;
#endif




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the newton's cradle example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char* argv[] )
{
   // Total number of time steps
   const unsigned int timesteps( 100000 );

   // Visualization variables
   bool povray  ( true );
   bool irrlicht( true );

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
      ( "file", value<std::string>()->default_value( "cradle.pe" ), "pe file describing cradle" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-povray" ) > 0 )
      povray = false;
   if( vm.count( "no-irrlicht" ) > 0 )
      irrlicht = false;

   // Selecting the parameter file
   std::string file( vm[ "file" ].as<std::string>() );

   std::cout << "\n Selected parameter file = " << file << "\n\n";

   // Initialization of the simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -2.0 );

   // Extracting the rigid body parameters
   BodyReader reader;
   reader.readFile( file.c_str(), povray );

   if( reader.hasError() ) {
      std::cerr << "\n ERROR DURING RIGID BODY SETUP\n" << reader.getError() << "\n";
      return EXIT_FAILURE;
   }

   // Setup of the Irrlicht visualization
#if HAVE_IRRLICHT
   if( irrlicht ) {
      ViewerID viewer = activateViewer( opengl, 800, 600 );
      viewer->addPointLightSource( 16.0F, 4.0F, 60.0F, 1.0f, 1.0f, 1.0f );
      viewer->addFPSCamera( -40.0F, -54.0F, 27.0F, 0.0F, 0.0F, -3.0F );
   }
#else
   UNUSED( irrlicht );
#endif

   // Simulation loop
   std::cout << "\n--RIGID BODY SIMULATION---------------------------------------------------------" << std::endl;

   for( unsigned int timestep=0; timestep<timesteps; ++timestep )
   {
      std::cout << "\r Simulating time step " << timestep+1 << "   " << std::flush;
      world->simulationStep( 0.001 );
   }

   std::cout << "\n--------------------------------------------------------------------------------\n" << std::endl;

   return EXIT_SUCCESS;
}
//*************************************************************************************************
