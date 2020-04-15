//=================================================================================================
/*!
 *  \file support/CommandLineInterface.cpp
 *  \brief Implementation of a default command-line interface for pe examples.
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

#include <pe/support/CommandLineInterface.h>
#include <pe/util/Assert.h>


namespace pe {




//=================================================================================================
//
//  HELPER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Helper function to evaluate options for the contact solver base class.
 *
 * \param contactSolver A reference to the current contact solver.
 * \return void
 */
template<>
void CommandLineInterface::evaluateContactSolverOptions< response::ContactSolver<Config> >( response::ContactSolver<Config>& contactSolver ) {
   if( vm_.count( "max-iterations" ) > 0 )
      contactSolver.setMaxIterations( vm_[ "max-iterations" ].as<size_t>() );
   if( vm_.count( "threshold" ) > 0 )
      contactSolver.setThreshold( vm_[ "threshold" ].as<real>() );
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Helper function to register options for the contact solver base class.
 *
 * \return void
 */
template<>
void CommandLineInterface::addContactSolverOptions< response::ContactSolver<Config> >() {
   desc_.add_options()
      ("max-iterations", value<size_t>()->default_value( response::lcp::maxIterations ), "the maximum number of iterations per time step" )
      ("threshold,t", value<real>()->default_value( response::lcp::threshold ), "the convergence threshold" )
   ;
}
//*************************************************************************************************

#if HAVE_OPENCL
//*************************************************************************************************
/*!\brief Helper function to evaluate options for the OpenCL contact solver.
 *
 * \param contactSolver A reference to the current contact solver.
 * \return void
 */
template<>
void CommandLineInterface::evaluateContactSolverOptions< response::OpenCLSolver<Config, NullType, NullType> >( response::OpenCLSolver<Config, NullType, NullType>& contactSolver ) {
   evaluateContactSolverOptions< response::ContactSolver<Config> >( contactSolver );
   if( vm_.count( "colors" ) > 0 )
      contactSolver.setColorLimit( vm_[ "colors" ].as<size_t>() );
   if( vm_.count( "omega" ) > 0 )
      contactSolver.setRelaxationParameter( vm_[ "omega" ].as<real>() );
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Helper function to register options for the OpenCL contact solver.
 *
 * \return void
 */
template<>
void CommandLineInterface::addContactSolverOptions< response::OpenCLSolver<Config, NullType, NullType> >() {
   addContactSolverOptions< response::ContactSolver<Config> >();
   desc_.add_options()
      ("colors,c", value<size_t>(), "limit of colors assigned in the solving process" )
      ("omega,w", value<real>(), "relaxation parameter in the solving process" )
   ;
}
//*************************************************************************************************
#endif




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*! \brief Default constructor for the CommandLineInterface class.
 */
CommandLineInterface::CommandLineInterface() : desc_("Allowed options") {
   desc_.add_options()
      ("help,h", "produce help message")
#if HAVE_IRRLICHT
      ("no-irrlicht", "deactivate Irrlicht visualization")
#endif
      ("no-povray", "deactivate PovRay visualization")
      ("no-vtk", "deactivate VTK visualization")
#if HAVE_OPENCL
      ("list-platforms", "list OpenCL platforms")
      ("list-devices", value<int>(), "list OpenCL devices")
      ("platform", value<int>(), "choose OpenCL platform")
      ("device", value<int>(), "choose OpenCL device")
#endif
      ("seed,s", value<size_t>(), "set random number seed")
   ;
   addContactSolverOptions<ContactSolver>();
}
//*************************************************************************************************




//=================================================================================================
//
//  CLI OPTION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \brief Parses the command line arguments and stores the option values.
 *
 * \param argc The number of arguments in the argument values array.
 * \param argv The array containing pointers to C-strings with the command line arguments.
 * \return void
 *
 * The function also processes the help argument to avoid exceptions thrown due to command
 * line requirements that were not met.
 */
void CommandLineInterface::parse(int argc, char* argv[]) {
   try {
      store(parse_command_line(argc, argv, desc_), vm_);

      // Process --help/-h
      if( vm_.count( "help" ) > 0 ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << desc_ << std::endl;
         }
         pe::exit( EXIT_FAILURE );
      }

      notify(vm_);
   }
   catch( std::exception& err ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cerr << "Error " << err.what() << "\n\n";
         std::cerr << desc_ << std::endl;
      }
      pe::exit( EXIT_FAILURE );
   }
}
//*************************************************************************************************

//*************************************************************************************************
/*! \brief Evaluates all parsed options.
 *
 * \return void
 */
void CommandLineInterface::evaluateOptions() {
   if( vm_.count( "seed" ) > 0 )
      setSeed( vm_[ "seed" ].as<uint32_t>() + MPISettings::rank() );
#if HAVE_OPENCL
   if( vm_.count( "list-platforms" ) > 0 ) {
      using namespace gpusolve::opencl::util;
      SystemInfo info;
      std::vector<cl::Platform>& platforms( info.platforms() );

      size_t platform_index( 0 );
      for( std::vector<cl::Platform>::const_iterator it = platforms.begin(); it != platforms.end(); ++it ) {
         std::cout << "Platform #" << platform_index++ << ":" << std::endl;
         std::cout << *it << std::endl;
      }
      exit( EXIT_FAILURE );
   }
   if( vm_.count( "list-devices" ) > 0 ) {
      using namespace gpusolve::opencl::util;
      SystemInfo info;
      cl::Platform& platform = info.platforms()[ vm_[ "list-devices" ].as<int>() ];
      std::vector<cl::Device> devices;
      platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);

      size_t device_index( 0 );
      for( std::vector<cl::Device>::const_iterator it = devices.begin(); it != devices.end(); ++it ) {
         std::cout << "Device #" << device_index++ << ":" << std::endl;
         std::cout << *it << std::endl;
      }
      exit( EXIT_FAILURE );
   }
   if( vm_.count( "platform" ) > 0 )
      gpusolve::opencl::OpenCLSystemInterface::setPlatform( vm_[ "platform" ].as<int>() );
   if( vm_.count( "device" ) > 0 )
      gpusolve::opencl::OpenCLSystemInterface::setDevice( vm_[ "device" ].as<int>() );

   std::cout << "Spawning the gpusolve system interface:" << std::endl;
   gpusolve::opencl::OpenCLSystemInterface& system( gpusolve::opencl::OpenCLSystemInterface::Instance() );
   std::cout << system.platform() << std::endl;
   std::cout << system.device() << std::endl;
#endif

   evaluateContactSolverOptions( theCollisionSystem()->getContactSolver() );
}
//*************************************************************************************************




//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

std::auto_ptr<CommandLineInterface> CommandLineInterface::instance_;

} // namespace pe
