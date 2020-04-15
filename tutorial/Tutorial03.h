//=================================================================================================
/*!
 *  \file Tutorial04.h
 *  \brief Physics Engine Tutorial 04
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

#ifndef _PE_TUTORIAL03_H_
#define _PE_TUTORIAL03_H_


//*************************************************************************************************
/*!\page tutorial03 Tutorial 3: Compound geometries
 *
 * \image html chain1.jpg
 *
 * In the first tutorial you had a first encounter with how to set up a rigid body simulation
 * in the \b pe physics engine. However, tutorial 1 only used geometric primitives to simulate
 * a box stack scenario. In this tutorial, we will focus on the setup of compound geometries.
 * Compound geometries are called \b unions in the \b pe engine and are build from an arbitrary
 * number of geometric primitives. From an implementational point of view, unions are a kind of
 * container for other rigid bodies. However, from a logical point of view, unions are just an
 * other kind of rigid bodies, i.e. they are rigid bodies on their own. The following example
 * gives you a first feeling about unions:

   \code
   // Let's assume that the following lines of code are creating and initializing a new union
   // according to our requirements. We will have a closer look on the setup of unions in the
   // remainder of this tutorial. Note that just as there are handles for geometric primitives
   // (SphereID, BoxID, ...) there are also handles for unions (UnionID).
   UnionID u = ...;
   ...

   // Unions are rigid bodies on their own, i.e. you can treat them just as any other rigid body.
   // For instance, you can set the global position of a union and specify the linear and angular
   // velocity of the union.
   u->setPosition  ( ... );
   u->setLinearVel ( ... );
   u->setAngularVel( ... );

   // It is as well possible to rotate and translate a complete union. All rigid bodies contained
   // in the union behave accordingly, i.e. actions performed on the entire union are (potentially)
   // changing all rigid bodies inside the union. For instance, in case the union is rotated, all
   // contained rigid bodies rotate around the center of mass of the union to their new position.
   u->rotate   ( ... );
   u->translate( ... );

   // It is also possible to make a union visible or invisible in all active visualization systems.
   // Setting a union visible makes all contained rigid bodies visible, making a union invisible
   // makes all contained bodies invisible. Also in this case the same rule applies: any action
   // performed on the entire union affects each contained rigid body.
   u->setVisible();

   // Although the rigid bodies contained in a union are technically a part of the union, it is
   // still possible to change their position and orientation. The following for-loop traverses
   // all contained rigid bodies and rotates them to their final position. Note that for every
   // position or orientation change of a rigid body contained in a union, the properties (as for
   // example the center of mass or the inertia) of the union are adapted to the new settings.
   // Also note, that setting the linear or angular velocity of a rigid body contained in a union
   // has no effect!
   const Union::Iterator begin( u->begin() );
   const Union::Iterator end  ( u->end()   );

   for( Union::Iterator body=begin; body!=end; ++body ) {
      body->rotate( ... );
   }

   ...

   // In order to destroy a union, the destroy() function can be used as for any other rigid body.
   // In case of the union, all rigid bodies contained in the union are destroy along with the
   // union.
   destroy( u );
   \endcode

 * In order to demonstrate how to create unions, we will take a close look at the setup of the
 * chain links in the chain example of the \b pe physics engine (see the illustration at the top
 * of this tutorial). This complete example can be found in "<install-path>/examples/chain/".
 * The following extract from this example shows the setup of the entire chain. The chain is
 * assembled by creating a new union for every single chain link. These chain links are placed
 * and oriented such that they build a perfectly aligned chain in the beginning of the simulation.

   \code
   int main( int argc, char** argv )
   {
      ...

      // Chain parameters
      const size_t N ( 5   );  // Total number of chain links
      const real   L ( 8.0 );  // Length of a single chain link
      const real   W ( 4.0 );  // Width of a single chain link
      const real   R ( 0.5 );  // Thickness of the chain link
      const real   Z ( 5.0 );  // Initial height of the chain

      ...

      // Setup of the chain
      const real disp( L - real(3)*R );
      const real totalLength( L + (N-1)*disp );

      Vec3 gpos( ( L - totalLength ) / real(2), 0.0, Z );
      real rot( PI/4.0 );

      for( size_t i=1; i<=N; ++i )
      {
         // Creating a new chain link
         UnionID link = createChainLink( i, gpos, R, L, W, iron );
         link->rotate( rot, 0.0, 0.0 );
         link->setLinearVel ( rand<real>(-0.3,0.3), rand<real>(-0.3,0.3), rand<real>( 0.0,0.3) );
         link->setAngularVel( rand<real>(-0.2,0.2), rand<real>(-0.2,0.2), rand<real>(-0.2,0.2) );

         // Adapting the global position and rotation
         gpos[0] += disp;    // Incrementing the global position
         rot     += PI/2.0;  // Incrementing the rotation
      }

      ...
   }
   \endcode

 * The setup of every single chain link is done by the 'createChainLink()' function provided by
 * the \b pe engine. Let's take a closer look at what's happening inside this function:
 *
 * \image html chainlink.png

   \code
   UnionID createChainLink( size_t uid, const Vec3& gpos, real radius, real length, real width,
                            MaterialID material, bool visible )
   {
      // The function starts with the calculation of some parameters that are necessary
      // for the setup of the chain link.
      const real diag( std::sqrt( width ) );               // Length of the diagonal capsules
      const real disp( std::sqrt( sqr(diag)/real(2) )  );  // Position offset for the diagonal capsules
      const real hl  ( length / real(2) );                 // Half length of the chain link
      const real hw  ( width  / real(2) );                 // Half width of the chain link
      const real px  ( hl - disp/real(2) );                // x-position of the diagonal capsules
      const real py  ( hw - disp/real(2) );                // y-position of the diagonal capsules

      // Adjusting the length and width of the chain link
      length = length - real(2)*disp;
      width  = width  - real(2)*disp;

      // Creating the union
      // The 'createUnion()' function creates a new, empty union. This function takes only two
      // arguments: the user-specific ID of the union and a boolean value that indicates whether
      // the union is visible or not. In case the union is visible and an invisible rigid body
      // is added to the union, the body is automatically turned visible (or vice versa).
      UnionID link = createUnion( uid, visible );

      // In the following, we will create several capsules and add them to the union. The setup
      // of the capsules is performed exactly as if we would use them separately. However, by
      // adding them to the union afterwards, they are made part of the union and are no longer
      // separate rigid bodies.

      // Creating the first capsule
      CapsuleID c1 = createCapsule( 1,  0, -hw, 0, radius, length, material );
      link->add( c1 );

      // Creating the second capsule
      CapsuleID c2 = createCapsule( 2,  0,  hw, 0, radius, length, material );
      link->add( c2 );

      // Creating the third capsule
      CapsuleID c3 = createCapsule( 3, -hl, 0, 0, radius, width, material );
      c3->rotate( 0.0, 0.0, PI/2 );
      link->add( c3 );

      // Creating the fourth capsule
      CapsuleID c4 = createCapsule( 4,  hl, 0, 0, radius, width, material );
      c4->rotate( 0.0, 0.0, PI/2 );
      link->add( c4 );

      // Creating the fifth, sixth, seventh and eighth capsule
      ...

      // Setting the global position of the union
      // So far we have created the union such that its center of mass is the origin of the
      // global coordinate system. By setting the global position of the union, we shift the
      // entire union to the desired position, i.e. the contained rigid bodies are moved such
      // that the center of mass of the union is at the specified position.
      link->setPosition( gpos );

      // Finally, after the setup of the union is complete, we return the handle to the union.
      return link;
   }
   \endcode

 * Hopefully this tutorial gave you the impression that unions are a very useful tool to create
 * more complex geometries built from primitives and that the \b pe engine makes it easy to
 * assemble these union. For more informations about unions, see the Union class description and
 * feel free to take a closer look at the complete source code of the chain example that can be
 * found in the corresponding example directory "<install-path>/examples/chain/".
 */
//*************************************************************************************************

#endif
