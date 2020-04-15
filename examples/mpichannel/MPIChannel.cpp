//=================================================================================================
/*!
 *  \file MPIChannel.cpp
 *  \brief Example file for the pe physics engine
 *
 *  Copyright (C) 2014 Tobias Preclik
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

#include <pe/core/MPISystem.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <pe/engine.h>
#include <pe/support.h>
#include <cmath>
#include <cstddef>
#include <iostream>

using namespace pe;
using namespace pe::timing;
using namespace pe::povray;
using boost::filesystem::path;




// Assert statically that only DEM solvers or a hard contact solver is used since parameters are tuned for it.
#define pe_CONSTRAINT_MUST_BE_EITHER_TYPE(A, B) typedef \
   pe::CONSTRAINT_TEST< \
      pe::CONSTRAINT_MUST_BE_SAME_TYPE_FAILED< \
         pe::IsSame<A,B>::value \
      >::value > \
   pe_JOIN( CONSTRAINT_MUST_BE_SAME_TYPE_TYPEDEF, __LINE__ );

typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactSemiImplicitTimesteppingSolvers>::Config         TargetConfig1;
pe_CONSTRAINT_MUST_BE_EITHER_TYPE(Config, TargetConfig1);


//*************************************************************************************************
class Checkpointer {
public:

   Checkpointer( path checkpointsPath = path( "checkpoints/" ) ) : checkpointsPath_( checkpointsPath ) {
   }

   void setPath( path checkpointsPath = path( "checkpoints/" ) ) {
      checkpointsPath_ = checkpointsPath;
   }

   void write( std::string name, bool povray ) {
      boost::filesystem::create_directories( checkpointsPath_ );
      bbwriter_.writeFileAsync( ( checkpointsPath_ / ( name + ".peb" ) ).string().c_str() );
      pe_PROFILING_SECTION {
         timing::WcTimer timeWait;
         timeWait.start();
         bbwriter_.wait();
         timeWait.end();
         MPI_Barrier( MPI_COMM_WORLD );

         pe_LOG_DEBUG_SECTION( log ) {
            log << "BodyBinaryWriter::wait() took " << timeWait.total() << "s on rank " << MPISettings::rank() << ".\n";
         }
      }

      std::ofstream fout( ( checkpointsPath_ / ( name + ".txt" ) ).string().c_str() );
      fout << std::setprecision( 17 );
      fout << "world_timesteps " << TimeStep::step() << "\n";
      if( povray ) {
         WriterID pov( activateWriter() );
         fout << "povray_steps "   << pov->getSteps()       << "\n"
              << "povray_counter " << pov->getFileCounter() << "\n";
      }
      fout << std::flush;
   }

   void read( std::string name, bool povray ) {
      bbreader_.readFile( ( checkpointsPath_ / ( name + ".peb" ) ).string().c_str() );

      std::ifstream fin( ( checkpointsPath_ / ( name + ".txt" ) ).string().c_str() );
      std::string key, value;
      std::map<std::string, std::string> paramMap;
      while( fin ) {
         fin >> key >> value;
         if( !fin )
            break;
         paramMap[key] = value;
      }

      TimeStep::step( boost::lexical_cast<unsigned int>( paramMap["world_timesteps"] ) );
      if( povray ) {
         WriterID pov( activateWriter() );
         pov->setSteps      ( boost::lexical_cast<size_t>( paramMap["povray_steps"]   ) );
         pov->setFileCounter( boost::lexical_cast<size_t>( paramMap["povray_counter"] ) );
      }
   }

   void flush() {
      bbwriter_.wait();
   }

private:

   BodyBinaryWriter bbwriter_;
   BodyBinaryReader bbreader_;
   path             checkpointsPath_;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Custom Texture policy.
 *
 * This class assigns textures to the particles according to their size they have.
 */
class UserIDTexturePolicy : public TexturePolicy
{
   public:
      explicit UserIDTexturePolicy() {}
      virtual ~UserIDTexturePolicy() {}

      virtual const pe::povray::Texture getTexture( ConstBodyID body ) const
      {
         switch( body->getID() )
         {
            case 0:   return CustomTexture( "TBoundary"  );
            case 1:   return CustomTexture( "TBoundary"  );
            case 2:   return CustomTexture( "TBoundary"  );
            case 3:   return CustomTexture( "TObstacle"  );
            case 4:   return CustomTexture( "TParticle0"  );
            case 5:   return CustomTexture( "TParticle1"  );
            case 6:   return CustomTexture( "TParticle2"  );
            case 7:   return CustomTexture( "TParticle3"  );
            default:  return CustomTexture( "TUnknown" );
         }
      }

      using TexturePolicy::getTexture;
};
//*************************************************************************************************


//*************************************************************************************************
struct DataReductionGrid {
   DataReductionGrid( RectilinearGrid& dd, Vec3 domainlen, Vector3<int> ng, Vector3<int> nd ) : dd_(dd), domainlen_(domainlen), ng_(ng), nd_(nd), fhOpen_(false), counter_( 0 ), numSpheres_( 0 ) {
   }

   void wait() {
      if( fhOpen_ ) {
         MPI_Status status;
         MPI_Wait( &request_, &status );
         MPI_File_close( &fh_ );
         fhOpen_ = false;
      }
   }

   ~DataReductionGrid() {
      wait();
   }

   void writeFileAsync() {

      // wait until we can reuse buffers
      wait();

      const int tag=12851;
      Vector3<int> coord = dd_.getCoords();

      Vector3<int> extent_lb( std::max( (int)std::ceil ( (domainlen_[0]/nd_[0]) / ( domainlen_[0]/ng_[0] ) * (coord[0]  )     ) - 1, 0        ),
                              std::max( (int)std::ceil ( (domainlen_[1]/nd_[1]) / ( domainlen_[1]/ng_[1] ) * (coord[1]  )     ) - 1, 0        ),
                              std::max( (int)std::ceil ( (domainlen_[2]/nd_[2]) / ( domainlen_[2]/ng_[2] ) * (coord[2]  )     ) - 1, 0        ) );
      Vector3<int> extent_ub( std::min( (int)std::floor( (domainlen_[0]/nd_[0]) / ( domainlen_[0]/ng_[0] ) * (coord[0]+1) - 1 ) + 1, ng_[0]-1 ),
                              std::min( (int)std::floor( (domainlen_[1]/nd_[1]) / ( domainlen_[1]/ng_[1] ) * (coord[1]+1) - 1 ) + 1, ng_[1]-1 ),
                              std::min( (int)std::floor( (domainlen_[2]/nd_[2]) / ( domainlen_[2]/ng_[2] ) * (coord[2]+1) - 1 ) + 1, ng_[2]-1 ) );

      extent_.clear();
      extent_.resize( (extent_ub[0]-extent_lb[0]+1)*(extent_ub[1]-extent_lb[1]+1)*(extent_ub[2]-extent_lb[2]+1) );

      WorldID world( theWorld() );
      numSpheres_ = 0;
      for( World::Iterator it = world->begin(); it != world->end(); ++it ) {
         if( it->isRemote() || it->getID() < 4  )
            continue;

         Vector3<int> cell( std::max( std::min( (int)std::floor( it->getPosition()[0]/( domainlen_[0]/ng_[0] ) ), ng_[0]-1 ), 0 ),
                            std::max( std::min( (int)std::floor( it->getPosition()[1]/( domainlen_[1]/ng_[1] ) ), ng_[1]-1 ), 0 ),
                            std::max( std::min( (int)std::floor( it->getPosition()[2]/( domainlen_[2]/ng_[2] ) ), ng_[2]-1 ), 0 ) );

         if( cell[0] < extent_lb[0] || cell[0] > extent_ub[0] || cell[1] < extent_lb[1] || cell[1] > extent_ub[1] || cell[2] < extent_lb[2] || cell[2] > extent_ub[2] )
            throw std::runtime_error( "Particle out of data reduction extent." );

         cell -= extent_lb;
         int cell_index( cell[2] * (extent_ub[1] - extent_lb[1] + 1) * (extent_ub[0] - extent_lb[0] + 1) + cell[1] * (extent_ub[0] - extent_lb[0] + 1) + cell[0] );

         extent_[cell_index].v_ = extent_[cell_index].v_ * (static_cast<real>( extent_[cell_index].num_ )/static_cast<real>(extent_[cell_index].num_ + 1)) + it->getLinearVel() / (extent_[cell_index].num_ + 1);
         extent_[cell_index].num_++;
         numSpheres_++;
      }

      Vector3<int> outextent_lb( (int)std::floor( (domainlen_[0]/nd_[0]) / ( domainlen_[0]/ng_[0] ) * (coord[0]  ) ),
                                 (int)std::floor( (domainlen_[1]/nd_[1]) / ( domainlen_[1]/ng_[1] ) * (coord[1]  ) ),
                                 (int)std::floor( (domainlen_[2]/nd_[2]) / ( domainlen_[2]/ng_[2] ) * (coord[2]  ) ) );

      Vector3<int> outextent_ub( (int)std::floor( (domainlen_[0]/nd_[0]) / ( domainlen_[0]/ng_[0] ) * (coord[0]+1) ) - 1,
                                 (int)std::floor( (domainlen_[1]/nd_[1]) / ( domainlen_[1]/ng_[1] ) * (coord[1]+1) ) - 1,
                                 (int)std::floor( (domainlen_[2]/nd_[2]) / ( domainlen_[2]/ng_[2] ) * (coord[2]+1) ) - 1 );

      SendBuffer<> sendbuffers[3][3][3];

      int offset[3][4] = { { extent_lb[0], outextent_lb[0], outextent_ub[0]+1, extent_ub[0] + 1 },
                           { extent_lb[1], outextent_lb[1], outextent_ub[1]+1, extent_ub[1] + 1 },
                           { extent_lb[2], outextent_lb[2], outextent_ub[2]+1, extent_ub[2] + 1 } };

      for( int ix=0; ix < 3; ++ix ) {
         for( int iy=0; iy < 3; ++iy ) {
            for( int iz=0; iz < 3; ++iz ) {
               if( ix == 1 && iy == 1 && iz == 1 )
                  continue;

               int n((offset[0][ix+1] - offset[0][ix])*(offset[1][iy+1] - offset[1][iy])*(offset[2][iz+1] - offset[2][iz]));
               sendbuffers[iz][iy][ix] << n;

               for( int jx = offset[0][ix]; jx < offset[0][ix+1]; ++jx ) {
                  for( int jy = offset[1][iy]; jy < offset[1][iy+1]; ++jy ) {
                     for( int jz = offset[2][iz]; jz < offset[2][iz+1]; ++jz ) {
                        Vector3<int> cell( jx, jy, jz );
                        cell -= extent_lb;

                        int cell_index( cell[2] * (extent_ub[1] - extent_lb[1] + 1) * (extent_ub[0] - extent_lb[0] + 1) + cell[1] * (extent_ub[0] - extent_lb[0] + 1) + cell[0] );
                        sendbuffers[iz][iy][ix] << jx << jy << jz << extent_[cell_index].num_ << extent_[cell_index].v_[0] << extent_[cell_index].v_[1] << extent_[cell_index].v_[2];
                     }
                  }
               }
            }
         }
      }

      MPI_Request requests[27];
      int requestsNum = 0;
      MPI_Status stats[26];
      for( int ix=0; ix < 3; ++ix ) {
         for( int iy=0; iy < 3; ++iy ) {
            for( int iz=0; iz < 3; ++iz ) {
               if( ix == 1 && iy == 1 && iz == 1 )
                  continue;
               if( coord[0] + ix - 1 < 0 || coord[0] + ix - 1 >= nd_[0] || coord[1] + iy - 1 < 0 || coord[1] + iy - 1 >= nd_[1] || coord[2] + iz - 1 < 0 || coord[2] + iz - 1 >= nd_[2] )
                  continue;

               MPI_Isend( sendbuffers[iz][iy][ix].ptr(), static_cast<int>( sendbuffers[iz][iy][ix].size() ), MPITrait<byte>::getType(), dd_.getRank( Vector3<size_t>(coord[0] + ix - 1, coord[1] + iy - 1, coord[2] + iz - 1) ), tag, MPISettings::comm(), &requests[requestsNum++]);
            }
         }
      }

      RecvBuffer<> recvbuffers[3][3][3];

      for( int ix=0; ix < 3; ++ix ) {
         for( int iy=0; iy < 3; ++iy ) {
            for( int iz=0; iz < 3; ++iz ) {
               if( ix == 1 && iy == 1 && iz == 1 )
                  continue;
               if( coord[0] + ix - 1 < 0 || coord[0] + ix - 1 >= nd_[0] || coord[1] + iy - 1 < 0 || coord[1] + iy - 1 >= nd_[1] || coord[2] + iz - 1 < 0 || coord[2] + iz - 1 >= nd_[2] )
                  continue;

               //see src/core/collisionsystem/MPICommunication.cpp
               const MPI_Comm     comm( MPISettings::comm() );
               const MPI_Datatype type( MPITrait<byte>::getType() );
               MPI_Status status;
               int count( 0 );

               // Probing for the next incoming MPI message
               MPI_Probe( MPI_ANY_SOURCE, tag, comm, &status );

               // Estimating the size of the MPI message
               MPI_Get_count( &status,   // The communication status
                     type,      // Data type of the elements to be received
                     &count );  // The number of elements in the MPI message

               Vector3<int> rccoord = dd_.getCoords(status.MPI_SOURCE);
               int rank = status.MPI_SOURCE;

               // Receiving the MPI message
               recvbuffers[rccoord[2]-coord[2]+1][rccoord[1]-coord[1]+1][rccoord[0]-coord[0]+1].resize( count );
               MPI_Recv( recvbuffers[rccoord[2]-coord[2]+1][rccoord[1]-coord[1]+1][rccoord[0]-coord[0]+1].ptr(),  // Initial address of the receive buffer
                     count,                 // Number of elements to be received
                     type,                  // Data type of each receive buffer element
                     rank,                  // Rank of the source process
                     tag,                   // MPI communication tag
                     comm,                  // The MPI communicator
                     &status );             // The communication status

            }
         }
      }

      MPI_Waitall( requestsNum, requests, stats );

      for( int ix=0; ix < 3; ++ix ) {
         for( int iy=0; iy < 3; ++iy ) {
            for( int iz=0; iz < 3; ++iz ) {
               if( ix == 1 && iy == 1 && iz == 1 )
                  continue;
               if( coord[0] + ix - 1 < 0 || coord[0] + ix - 1 >= nd_[0] || coord[1] + iy - 1 < 0 || coord[1] + iy - 1 >= nd_[1] || coord[2] + iz - 1 < 0 || coord[2] + iz - 1 >= nd_[2] )
                  continue;

               int n;
               recvbuffers[iz][iy][ix] >> n;
               for( int i = 0; i < n; ++i ) {
                  Vector3<int> cell;
                  int num;
                  Vec3 v;
                  recvbuffers[iz][iy][ix] >> cell[0] >> cell[1] >> cell[2] >> num >> v[0] >> v[1] >> v[2];
                  if( cell[0] < outextent_lb[0] || cell[0] > outextent_ub[0] || cell[1] < outextent_lb[1] || cell[1] > outextent_ub[1] || cell[2] < outextent_lb[2] || cell[2] > outextent_ub[2] )
                     throw std::runtime_error( "Received invalid datum." );

                  cell -= extent_lb;
                  int cell_index( cell[2] * (extent_ub[1] - extent_lb[1] + 1) * (extent_ub[0] - extent_lb[0] + 1) + cell[1] * (extent_ub[0] - extent_lb[0] + 1) + cell[0] );
                  if( num != 0 ) {
                     extent_[cell_index].v_   = extent_[cell_index].v_ * ( static_cast<real>( extent_[cell_index].num_ ) / static_cast<real>( extent_[cell_index].num_ + num ) ) + v * ( static_cast<real>( num ) / static_cast<real>( extent_[cell_index].num_ + num ) );
                     extent_[cell_index].num_ = num + extent_[cell_index].num_;
                  }
               }
            }
         }
      }

      {
         std::stringstream sout;
         //sout << std::fixed;

         if( dd_.getRank() == 0 ) {
            sout << "<?xml version=\"1.0\"?>\n"
                 << "<VTKFile type=\"ImageData\" version=\"0.1\">\n"
                 << "\t<ImageData WholeExtent=\"0 " << ng_[0] << " 0 " << ng_[1] << " 0 " << ng_[2] << "\" Origin=\"0 0 0\" Spacing=\"" << domainlen_[0]/ng_[0] << " " << domainlen_[1]/ng_[1] << " " << domainlen_[2]/ng_[2] <<"\">\n";
         }

         sout << "\t\t<Piece Extent=\"" << outextent_lb[0] << " " << outextent_ub[0]+1 << " " << outextent_lb[1] << " " << outextent_ub[1]+1 << " " << outextent_lb[2] << " " << outextent_ub[2]+1 << "\">\n"
              << "\t\t\t<CellData Scalars=\"num\" Vectors=\"vel\">\n"
              << "\t\t\t\t<DataArray type=\"Int32\" Name=\"num\" format=\"ascii\" NumberOfComponents=\"1\">\n";
         for( int jz = outextent_lb[2]; jz <= outextent_ub[2]; ++jz ) {
            for( int jy = outextent_lb[1]; jy <= outextent_ub[1]; ++jy ) {
               for( int jx = outextent_lb[0]; jx <= outextent_ub[0]; ++jx ) {
                  Vector3<int> cell( jx, jy, jz );
                  cell -= extent_lb;

                  int cell_index( cell[2] * (extent_ub[1] - extent_lb[1] + 1) * (extent_ub[0] - extent_lb[0] + 1) + cell[1] * (extent_ub[0] - extent_lb[0] + 1) + cell[0] );
                  sout << "\t\t\t\t\t" << extent_[cell_index].num_ << "\n";
               }
            }
         }
         sout << "\t\t\t\t</DataArray>\n"
              << "\t\t\t\t<DataArray type=\"Float64\" Name=\"vel\" format=\"ascii\" NumberOfComponents=\"3\">\n";
         for( int jz = outextent_lb[2]; jz <= outextent_ub[2]; ++jz ) {
            for( int jy = outextent_lb[1]; jy <= outextent_ub[1]; ++jy ) {
               for( int jx = outextent_lb[0]; jx <= outextent_ub[0]; ++jx ) {
                  Vector3<int> cell( jx, jy, jz );
                  cell -= extent_lb;

                  int cell_index( cell[2] * (extent_ub[1] - extent_lb[1] + 1) * (extent_ub[0] - extent_lb[0] + 1) + cell[1] * (extent_ub[0] - extent_lb[0] + 1) + cell[0] );
                  sout << "\t\t\t\t\t" << extent_[cell_index].v_[0] << " " << extent_[cell_index].v_[1] << " " << extent_[cell_index].v_[2] << "\n";
               }
            }
         }
         sout << "\t\t\t\t</DataArray>\n"
              << "\t\t\t</CellData>\n"
              << "\t\t</Piece>\n";

         if( dd_.getRank() == nd_[0]*nd_[1]*nd_[2]-1 ) {
            sout << "\t</ImageData>\n"
                 << "</VTKFile>\n";
         }

         chunk_ = sout.str();
      }

      size_t localSize( chunk_.size() ), localOffset( 0 );
      MPI_Exscan( &localSize, &localOffset, 1, MPITrait<size_t>::getType(), MPI_SUM, MPISettings::comm() );

      std::stringstream fnout;
      fnout << "reduction/reduction" << counter_++ << ".vti";
      std::string fn( fnout.str() );

      fhOpen_ = true;
      MPI_File_open( MPISettings::comm(), &fn[0], MPI_MODE_WRONLY | MPI_MODE_CREATE, MPI_INFO_NULL, &fh_ );
      MPI_File_iwrite_at( fh_, localOffset, &chunk_[0], static_cast<int>( localSize ), MPI_BYTE, &request_ );
   }

   struct ReducedDatum {
      Vec3 v_;
      int num_;

      ReducedDatum() : v_(), num_( 0 ) {}
   };

   RectilinearGrid& dd_;
   Vec3 domainlen_;
   Vector3<int> ng_, nd_;

   MPI_File fh_;
   bool fhOpen_;
   std::string chunk_;
   std::vector<ReducedDatum> extent_;
   int counter_;
   MPI_Request request_;
   size_t numSpheres_;
};
//*************************************************************************************************




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the mpichannel example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char** argv )
{
   WcTimer setupTime, simTime, checkpointerTime;
   setupTime.start();

   // MPI Initialization
   MPI_Init( &argc, &argv );

   // Conversion factors
   const real   second                   ( 1.0e-3 );                  // Conversion factor from unitless time to seconds.
   const real   kilogram                 ( 1.0e-3 );                  // Conversion factor from unitless weight to kilograms.
   const real   meter                    ( 1.0e-3 );                  // Conversion factor from unitless length to meters.
   const real   g_cm3                    ( 1.0e+3 );                  // g/cm^3 = 1000 kg/m^3 = 1000*kilogram/meter^3
   const real   m_s2                     ( 1.0e+3 );                  // meter/second^2
   const real   m_s                      ( 1.0e+0 );                  // meter/second

   UNUSED( kilogram );


   //----------------------------------------------------------------------------------------------
   // Hard-coded simulation parameters

         size_t seed                     ( 12345 );                   // Seed for the random number generation
   const real   gravity                  ( 9.81 / m_s2 );             // Acceleration along negative z direction in mm/ms^2=10^3*m/s^2. The gravity of earth is 9.81m/s^2.

   const real   duration                 ( 10 / second );             // The duration of the simulation
   const real   dt_max                   ( 1.0e-5 / second );         // The maximum size of the time steps limiting time discretization errors.

   const int    px                       ( 16 );                      // Number of processes in x direction
   const int    py                       ( 10 );                      // Number of processes in y direction
   const int    pz                       ( 4 );                       // Number of processes in z direction

         bool   povray                   ( true );                    // Switches the POV-Ray visualization on and off.
   const size_t povray_fps               ( 300 );                     // Target frames per second for the POV-Ray visualization.
   const size_t datard_fps               ( 300 );                     // Target frames per second for the POV-Ray visualization.
   const path   povray_path              ( "video/" );                // The path where to store the visualization data.

   const int    granular_nz              ( 5*pz );
   const int    granular_ny              ( 6*py );

   // Properties of the granular particles:
   const real   granular_r               ( 0.5e-3 / meter );          // The radius of particles in mm. Sand grains range from 0.063mm to 2mm in size.
   const real   granular_spacing         ( 0.1 * granular_r );        // Initial spacing in-between two spherical particles in mm.
   const real   granular_dist            ( 2.0 * granular_r + granular_spacing );
   const real   granular_density         ( 2.65 / g_cm3 );            // Density of the granular particles is 2.65 g/cm^3, that is similar to quartz.

   const bool   granular_spherical       ( true );

   const real   static_cof               ( 0.4 / 2 );                 // Coefficient of static friction. Roughly 0.85 with high variation depending on surface roughness for low stresses. Note: pe doubles the input coefficient of friction for material-material contacts.
   const real   dynamic_cof              ( static_cof );              // Coefficient of dynamic friction. Similar to static friction for low speed friction.

   // Calculate domain \Omega = [0; lx] x [0; ly] x (0; lz):
   const real   lx                       ( 15.0e-2 / meter );                                       // Length of the simulation domain in x-dimension.
   const real   ly                       ( granular_ny * granular_dist );                                       // Length of the simulation domain in y-dimension.
   const real   lz                       ( granular_nz * granular_dist );                                       // Length of the simulation domain in z-dimension.

   const real   inflow_velocity          ( 1.0 / m_s );
   const real   obstacle_y               ( 0.5*ly );
   const real   obstacle_r               ( 10.0e-3 / meter );


   //----------------------------------------------------------------------------------------------
   // Evaluation of command line arguments

   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
      ( "resume", value<std::string>()->default_value( "" ), "the checkpoint to resume" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();

   // Override hard-coded povray settings by command line options
   if( vm.count( "no-povray" ) > 0 )
      povray = false;

   // Supply initial seed for the random number generation (prefer seed given on command line)
   if( vm.count( "seed" ) > 0 )
      seed = vm[ "seed" ].as<uint32_t>();
   else
      setSeed( seed + MPISettings::rank() );

   std::string resume( vm[ "resume" ].as<std::string>() );


   //----------------------------------------------------------------------------------------------
   // Derived quantities


   const real   l_dd                     ( std::min( lx/px, std::min( ly/py, lz/pz ) ) );  // Minimum distance to non-nearest neighbor subdomain.
   const real   v_max                    ( (real(1) / real(250)) * granular_r / dt_max );
   const size_t povray_spacing           ( (std::size_t) std::ceil( 1 / ( second * dt_max * povray_fps ) ) );  // The number of simulation steps between two frames.
   const size_t datard_spacing           ( (std::size_t) std::ceil( 1 / ( second * dt_max * datard_fps ) ) );  // The number of simulation steps between two frames.
   const size_t timesteps_total          ( (std::size_t)( std::ceil( ( duration / dt_max ) / povray_spacing ) ) * povray_spacing );

   //----------------------------------------------------------------------------------------------
   // Aliases

   WorldID      world                    ( theWorld() );


   //----------------------------------------------------------------------------------------------
   // Global variables

   bool         timeout                  ( false );
   bool         statusupdate             ( false );
   double       last_statusupdate        ( 0 );
   real         maximumPenetration       ( 0 );
   size_t       numContacts              ( 0 );
   size_t       numBodies                ( 0 );
   size_t       t0                       ( 0 );


   //----------------------------------------------------------------------------------------------
   // Parameter assertions

   UNUSED( l_dd );
   pe_INTERNAL_ASSERT( granular_r < l_dd, "Granular matter too large for nearest neighbor communication." );
   pe_INTERNAL_ASSERT( ( real( 1 ) / ( dt_max * povray_spacing ) / second - povray_fps ) / povray_fps < 0.01, "Effective visualization frame rate deviates from prescribed frame rate by more than 1%." );


   //----------------------------------------------------------------------------------------------
   // Setup of Domain Decomposition

   RectilinearGrid grid;
   grid.connect( Vec3(0, 0, 0), Vec3(lx, ly, lz), Vector3<size_t>(px, py, pz), Vector3<BoundaryCondition>( boundaryConditionOpen, boundaryConditionOpen, boundaryConditionOpen ), Vector3<BoundaryCondition>( boundaryConditionOutflow, boundaryConditionOpen, boundaryConditionOpen ) );

   pe_LOG_DEBUG_SECTION( log ) {
      log << "The domain decomposition grid comprises the domain [0; " << lx << "] x [0; " << ly << "] x (0; " << lz << "), where open boundary conditions are applied in x-, y- and z-directions except at the upper end of the x-dimension where an outflow boundary condition is imposed.\n";
   }


   //----------------------------------------------------------------------------------------------
   // Setup of the POV-Ray visualization

   WriterID pov;

   if( povray ) {
      pov = activateWriter();
      pov->setSpacing( povray_spacing );
      pov->setFilename( ( povray_path / "pic%.pov" ).string().c_str() );
      pov->include( "settings.inc" );
      pov->setDecorations( false );
      pov->setTexturePolicy( UserIDTexturePolicy() );
   }


   //----------------------------------------------------------------------------------------------
   // Setup of the collision system

   CollisionSystemID cs( theCollisionSystem() );
   cs->setRelaxationParameter( 0.75 );
   cs->setMaxIterations( 10 );
   cs->setErrorReductionParameter( 0.8 );
   //cs->setRelaxationModel( CollisionSystem<Config>::InelasticFrictionlessContact );
   //cs->setRelaxationModel( CollisionSystem<Config>::InelasticGeneralizedMaximumDissipationContact );

   //----------------------------------------------------------------------------------------------
   // Setup of the simulation domain

   world->setGravity( 0, 0, -gravity );
   world->setDamping( 1 );               // Deactivate damping.

   MaterialID granular_material = createMaterial( "granular", granular_density, 0, static_cof, dynamic_cof, real( 0.5 ), 1, 1, 0, 0 );
   MaterialID boundary_material = createMaterial( "boundary", 1,                0, static_cof, dynamic_cof, real( 0.5 ), 1, 1, 0, 0 );

   Checkpointer         checkpointer;
   path                 checkpoint_path( "checkpoints/" );            // The path where to store the checkpointing data

   PlaneID   propulsion( 0 );
   CapsuleID capsule( 0 );
   BoxID     box( 0 );
   if( resume.empty() ) {

      // Setup of confining walls
      pe_GLOBAL_SECTION
      {
         propulsion = createPlane( 0,  Vec3(+1, 0, 0), 0,   boundary_material, false );
         createPlane( 1,  Vec3(0, +1, 0), 0,   boundary_material, false );
         createPlane( 1,  Vec3(0, -1, 0), -ly, boundary_material, false );
         createPlane( 1,  Vec3(0, 0, +1), 0,   boundary_material, false );
         createPlane( 1,  Vec3(0, 0, -1), -lz, boundary_material, false );

         capsule = createCapsule( 3, Vec3( 4.00e-2 / meter, obstacle_y, 0.5*lz ), obstacle_r, lz, boundary_material, false );
         capsule->rotate( Vec3(0, 1, 0), 0.5*M_PI );
         capsule->setFixed( true );

         box = createBox( 2, Vec3( 4.00e-2 / meter + 0.5*obstacle_r, obstacle_y, 0.5*lz ), Vec3( obstacle_r, 2*obstacle_r, lz ), boundary_material, false );
         box->setFixed( true );
      }

      // Deterministic setup of the particles (iterate over all points in the grid which are strictly inside our subdomain or on _any_ boundary)
      size_t x_min = std::max( (int)ceil ( (grid.getCoords()[0]    ) * (lx / px) / granular_dist - real(1.0) ), 0                );
      size_t y_min = std::max( (int)ceil ( (grid.getCoords()[1]    ) * (ly / py) / granular_dist - real(1.0) ), 0                );
      size_t z_min = std::max( (int)ceil ( (grid.getCoords()[2]    ) * (lz / pz) / granular_dist - real(1.0) ), 0                );
      size_t x_max =           (int)floor( (grid.getCoords()[0] + 1) * (lx / px) / granular_dist + real(0.5) )                    ;
      size_t y_max = std::min( (int)floor( (grid.getCoords()[1] + 1) * (ly / py) / granular_dist + real(0.5) ), (int)granular_ny );
      size_t z_max = std::min( (int)floor( (grid.getCoords()[2] + 1) * (lz / pz) / granular_dist + real(0.5) ), (int)granular_nz );

      for( size_t i_x = x_min; i_x < x_max; ++i_x ) {
         for( size_t i_y = y_min; i_y < y_max; ++i_y ) {
            for( size_t i_z = z_min; i_z < z_max; ++i_z ) {
               Vec3 position(
                     ( i_x + real(0.5) ) * granular_dist,
                     ( i_y + real(0.5) ) * granular_dist,
                     ( i_z + real(0.5) ) * granular_dist );

               // Explicitly test points whether they are located on the boundary
               if( !theWorld()->ownsPoint( position ) )
                  continue;

               int userid = 0;
               if( position[1] > obstacle_y + 0.1*ly )
                  userid = 7;
               else if( position[1] > obstacle_y )
                  userid = 6;
               else if(position[1] > obstacle_y-0.1*ly )
                  userid = 5;
               else
                  userid = 4;

               BodyID particleBody( 0 );
               if( granular_spherical ) {
                  SphereID particle = createSphere( userid, position, granular_r, granular_material );

                  if( overlapSphereCapsule( particle, capsule ) ) {
                     destroy( particle );
                     continue;
                  }

                  if( overlapSphereBox( particle, box ) ) {
                     destroy( particle );
                     continue;
                  }

                  particleBody = particle;
               }
               else {
                  UnionID particle = createGranularParticle( userid, position, granular_r, granular_material );

                  if( overlapCapsuleUnion( capsule, particle ) ) {
                     destroy( particle );
                     continue;
                  }

                  if( overlapBoxUnion( box, particle ) ) {
                     destroy( particle );
                     continue;
                  }

                  particleBody = particle;
               }


               if( granular_dist == 0 )
                  particleBody->setLinearVel( inflow_velocity, 0, 0 );
               else
                  particleBody->setLinearVel( inflow_velocity, pe::rand(-granular_dist, granular_dist)/(3*granular_r/inflow_velocity), pe::rand(-granular_dist, granular_dist)/(3*granular_r/inflow_velocity) );
            }
         }
      }

      propulsion->setLinearVel( inflow_velocity, 0, 0 );

      if( povray )
         pov->writeFile( ( povray_path / "init.pov" ).string().c_str() );
   }
   else {
      // Resume from checkpoint
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "Resuming checkpoint \"" << resume << "\"..." << std::endl;
      }

      checkpointer.setPath( checkpoint_path / resume );
      checkpointer.read( resume, povray );
      // PovRay textures are reassigned through texture policy

      propulsion = (PlaneID)  ( *findUserID( world->begin(), world->end(), 0 ) );
      box        = (BoxID)    ( *findUserID( world->begin(), world->end(), 2 ) );
      capsule    = (CapsuleID)( *findUserID( world->begin(), world->end(), 3 ) );

      if( povray )
         pov->writeFile( ( povray_path / "resume.pov" ).string().c_str() );

      t0 = TimeStep::step();
   }

   // Synchronization of the MPI processes
   world->synchronize();


   //----------------------------------------------------------------------------------------------
   // Output of the simulation settings

   pe_EXCLUSIVE_SECTION( 0 ) {
      if( inflow_velocity > v_max )
         std::cout << "WARNING: Inflow velocity too high." << std::endl;
      if( real(1) / ( povray_fps * second ) <= dt_max )
         std::cout << "WARNING: Visualization rate too high." << std::endl;

      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "------------------------------------------------------------\n"
                << " Size of the domain                  = [0mm; " << lx << "mm] x [0mm; " << ly << "mm] x [0mm; " << lz << "mm]\n"
                << " Number of MPI processes             = (" << px << ", " << py << ", " << pz << ")\n"
                << " Povray Visualization framerate      = "  << povray_fps << "fps\n"
                << " Datareduction framerate             = "  << datard_fps << "fps\n"
                << " Particles per second                = "  << granular_ny*granular_nz/(2*granular_r/inflow_velocity)/second << "/s\n"
                << " Radius of particles                 = "  << granular_r << "mm\n"
                << " Initial spacing between particles   = "  << granular_spacing << "mm\n"
                << " Seed of the random number generator = "  << getSeed() << "\n"
                << " Duration of simulation              = "  << duration << "ms (" << timesteps_total << " time steps)\n"
                << " Maximum time step                   = "  << dt_max << "ms\n"
                << "------------------------------------------------------------------------------" << std::endl;
   }


   //----------------------------------------------------------------------------------------------
   // Simulation loop

   std::vector<real> buffer( 5 );

   DataReductionGrid drgrid( grid, Vec3(lx, ly, lz), Vector3<int>(3*px, 2*py, 2*pz), Vector3<int>(px,py,pz) );

   setupTime.end();
   MPI_Barrier( MPI_COMM_WORLD );
   simTime.start();
   
   while( TimeStep::step() < timesteps_total ) {
      if( isnan( propulsion->getPosition() ) ) {
         pe_LOG_ERROR_SECTION( log ) {
            log << "Position of propulsion plane became not a number. Aborting to prevent hang.\n";
         }
         pe::abort();
      }

      if( propulsion->getPosition()[0] > 2*granular_r ) {
         // Reset propulsion plane
         {
            Vec3 position( propulsion->getPosition() );
            position[0] = 0;
            propulsion->setPosition( position );
         }

         // Generation of new particles
         size_t y_min = std::max( (int)ceil ( (grid.getCoords()[1]    ) * (ly / py) / granular_dist - real(1.0) ), 0                );
         size_t z_min = std::max( (int)ceil ( (grid.getCoords()[2]    ) * (lz / pz) / granular_dist - real(1.0) ), 0                );
         size_t y_max = std::min( (int)floor( (grid.getCoords()[1] + 1) * (ly / py) / granular_dist + real(0.5) ), (int)granular_ny );
         size_t z_max = std::min( (int)floor( (grid.getCoords()[2] + 1) * (lz / pz) / granular_dist + real(0.5) ), (int)granular_nz );

         if( grid.getCoords()[0] == 0 ) {
            for( size_t i_y = y_min; i_y < y_max; ++i_y ) {
               for( size_t i_z = z_min; i_z < z_max; ++i_z ) {
                  Vec3 position(
                        granular_r,
                        ( i_y + real(0.5) ) * granular_dist,
                        ( i_z + real(0.5) ) * granular_dist );

                  // Explicitly test points whether they are located on the boundary
                  if( !theWorld()->ownsPoint( position ) )
                     continue;

                  int userid = 0;
                  if( position[1] > obstacle_y+0.1*ly )
                     userid = 7;
                  else if( position[1] > obstacle_y )
                     userid = 6;
                  else if(position[1] > obstacle_y-0.1*ly )
                     userid = 5;
                  else
                     userid = 4;
               
                  BodyID particle( 0 );
                  if( granular_spherical )
                     particle = createSphere( userid, position, granular_r, granular_material );
                  else
                     particle = createGranularParticle( userid, position, granular_r, granular_material );

                  if( granular_dist == 0 )
                     particle->setLinearVel( inflow_velocity, 0, 0 );
                  else
                     particle->setLinearVel( inflow_velocity, pe::rand(-granular_dist, granular_dist)/(3*granular_r/inflow_velocity), pe::rand(-granular_dist, granular_dist)/(3*granular_r/inflow_velocity) );
               }
            }
         }

         // Synchronization of the MPI processes
         world->synchronize();
      }

      // Checkpoint every 100000 time steps
#if 0
      if( TimeStep::step() % 100000 == 0 && TimeStep::step() != t0 ) {
         checkpointerTime.start();

         std::stringstream sstr;
         sstr << "snapshot" << TimeStep::step() / 10000;
         checkpointer.setPath( checkpoint_path / sstr.str() );
         checkpointer.write( sstr.str(), povray );

         checkpointerTime.end();
      }
#endif

      // All-to-all communication
      if( TimeStep::step() % 100 == 0 ) {
         buffer[0] = timeout      ? 0 : 1;
         buffer[1] = statusupdate ? 0 : 1;
         buffer[2] = -cs->getMaximumPenetration();

         MPI_Allreduce( MPI_IN_PLACE, &buffer[0], 3, MPITrait<real>::getType(), MPI_MIN, MPISettings::comm() );

         timeout      = buffer[0] == 0;
         statusupdate = buffer[1] == 0;
         maximumPenetration = -buffer[2];

         numBodies = 0;
         for( World::Iterator it = world->begin(); it != world->end(); ++it ) {
            if( it->isRemote() || it->getID() < 4  )
               continue;

            numBodies++;
         }

         buffer[0] = cs->getNumberOfContacts();
         buffer[1] = numBodies;

         MPI_Allreduce( MPI_IN_PLACE, &buffer[0], 2, MPITrait<real>::getType(), MPI_SUM, MPISettings::comm() );

         numContacts  = static_cast<size_t>( buffer[0] );
         numBodies    = static_cast<size_t>( buffer[1] );

         if( timeout )
            break;

         if( statusupdate ) {
            statusupdate = false;

            MemoryMeter mm;
            mm.stop();

            pe_LOG_DEBUG_SECTION( log ) {
               log << "------------------------------------------------------------------------------\n"
                   << " Total memory allocated      = " << TimeStep::step() << " " << mm.lastAllocation() << "bytes\n"
                   << " Total memory in use         = " << TimeStep::step() << " " << mm.lastInUse()     << "bytes\n"
                   << "------------------------------------------------------------------------------\n";
            }

            theCollisionSystem()->logProfilingSummary();
         }
      }

      if( TimeStep::step() % datard_spacing == 0 ) {
         drgrid.writeFileAsync();
      }

      world->simulationStep( dt_max );

      pe_EXCLUSIVE_SECTION( 0 ) {
         simTime.lap();

         // Update status if last status output was at least 60 seconds ago
         if( simTime.total() - last_statusupdate >= 60 ) {
            last_statusupdate = simTime.total();
            statusupdate = true;

            size_t wcl        ( (size_t)( simTime.total() ) );
            size_t wcl_hours  ( ( wcl                                   ) / 3600 );
            size_t wcl_minutes( ( wcl - wcl_hours*3600                  ) / 60   );
            size_t wcl_seconds( ( wcl - wcl_hours*3600 - wcl_minutes*60 )        );

            size_t eta        ( (size_t)( simTime.average() * ( timesteps_total - TimeStep::step() ) )  );
            size_t eta_hours  ( ( eta                                   ) / 3600 );
            size_t eta_minutes( ( eta - eta_hours*3600                  ) / 60   );
            size_t eta_seconds( ( eta - eta_hours*3600 - eta_minutes*60 )        );

            std::cout << "------------------------------------------------------------------------------\n"
                      << " Simulation time      = " << TimeStep::step()*dt_max << "ms\n"
                      << " Number of time steps = " << TimeStep::step() << "/" << timesteps_total << " (" << std::floor( (real)(TimeStep::step()) / (real)(timesteps_total) * 10000 ) / 100 << "%)\n"
                      << " Timestep size        = " << dt_max << "ms\n"
                      << " Wall Clock Time      = " << wcl_hours << ":" << std::setfill('0') << std::setw(2) << wcl_minutes << ":" << std::setfill('0') << std::setw(2) << wcl_seconds << " (" << simTime.average() << "s per timestep)\n"
                      << " ETA                  = " << eta_hours << ":" << std::setfill('0') << std::setw(2) << eta_minutes << ":" << std::setfill('0') << std::setw(2) << eta_seconds << "\n"
                      << " Maximum penetration  = " << maximumPenetration << "\n"
                      << " Number of contacts   = " << numContacts << "\n"
                      << " Number of particles  = " << numBodies << "\n"
                      << "------------------------------------------------------------------------------\n" << std::endl;
         }

         // Checkpoint and quit simulation if 23:55 hours have passed
         if( simTime.total() >= 24*60*60 - 5*60 )
            timeout = true;
      }
   }
   simTime.end();

   // Checkpoint and print histogram after final settling phase.
   if( TimeStep::step() == timesteps_total && TimeStep::step() != t0 ) {
      checkpointerTime.start();

      checkpointer.setPath( checkpoint_path / "end" );
      checkpointer.write( "end", povray );

      if( povray )
         pov->writeFile( ( povray_path / "end.pov" ).string().c_str() );

      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "Ended simulation at time " << TimeStep::step()*dt_max << " (" << TimeStep::step() << ").\n";
      }

      checkpointerTime.end();
   }
   // Checkpoint on timeout.
   else if( timeout && TimeStep::step() != t0 ) {
      checkpointerTime.start();

      checkpointer.setPath( checkpoint_path / "timeout" );
      checkpointer.write( "timeout", povray );

      if( povray )
         pov->writeFile( ( povray_path / "timeout.pov" ).string().c_str() );

      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "Wrote checkpoint before running out of time at time " << TimeStep::step()*dt_max << " (" << TimeStep::step() << ")." << std::endl;
      }

      checkpointerTime.end();
   }


   //----------------------------------------------------------------------------------------------
   // Simulation timing results

   pe_EXCLUSIVE_SECTION( 0 ) {
      size_t wcl        ( (size_t)( simTime.total() ) );
      size_t wcl_hours  ( ( wcl                                   ) / 3600 );
      size_t wcl_minutes( ( wcl - wcl_hours*3600                  ) / 60   );
      size_t wcl_seconds( ( wcl - wcl_hours*3600 - wcl_minutes*60 )        );

      std::cout << "------------------------------------------------------------------------------\n"
                << " Simulation time      = " << TimeStep::step()*dt_max << "ms\n"
                << " Number of time steps = " << TimeStep::step() << "\n"
                << " Timestep size        = " << dt_max << "ms\n"
                << " Wall Clock Time      = " << wcl_hours << ":" << std::setfill('0') << std::setw(2) << wcl_minutes << ":" << std::setfill('0') << std::setw(2) << wcl_seconds << " (" << simTime.average() << "s per timestep)\n"
                << "------------------------------------------------------------------------------\n\n"
                << "Timing results reduced over all time steps on current process:\n" << std::fixed << std::setprecision(4)
                << "code part              " << "min time  "                                    << "   " << "max time  "                                    << "   " << "avg time  "                                        << "   " << "total time"                                      << "   " << "executions"                                           << "\n"
                << "--------------------   " << "----------"                                    << "   " << "----------"                                    << "   " << "----------"                                        << "   " << "----------"                                      << "   " << "----------"                                           << "\n"
                << "setup                  " << std::setw(10) << setupTime.min()                << "   " << std::setw(10) << setupTime.max()                << "   " << std::setw(10) << setupTime.average()                << "   " << std::setw(10) << setupTime.total()                << "   " << std::setw(10) << setupTime.getCounter()                << "\n"
                << "simulation step        " << std::setw(10) << simTime.min()                  << "   " << std::setw(10) << simTime.max()                  << "   " << std::setw(10) << simTime.average()                  << "   " << std::setw(10) << simTime.total()                  << "   " << std::setw(10) << simTime.getCounter()                  << "\n"
                << "  checkpointer         " << std::setw(10) << checkpointerTime.min()         << "   " << std::setw(10) << checkpointerTime.max()         << "   " << std::setw(10) << checkpointerTime.average()         << "   " << std::setw(10) << checkpointerTime.total()         << "   " << std::setw(10) << checkpointerTime.getCounter()         << "\n"
                << "--------------------   " << "----------"                                    << "   " << "----------"                                    << "   " << "----------"                                        << "   " << "----------"                                      << "   " << "----------"                                           << "\n";
   }

   theCollisionSystem()->logProfilingSummary();


   //----------------------------------------------------------------------------------------------
   // Cleanup

   checkpointer.flush();
   drgrid.wait();
   MPI_Finalize();
}
//*************************************************************************************************
