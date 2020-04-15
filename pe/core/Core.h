//=================================================================================================
/*!
 *  \file pe/core/Core.h
 *  \brief Core module documentation
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

#ifndef _PE_CORE_CORE_H_
#define _PE_CORE_CORE_H_


//=================================================================================================
//
//  DOXYGEN DOCUMENTATION
//
//=================================================================================================

//*************************************************************************************************
//! Namespace of the \b pe rigid body physics engine.
namespace pe {

//! Namespace of the Lemke LCP solver.
namespace lemke {}

//! Namespace of the projected Gauss-Seidel (PGS) LCP solver.
namespace pgs {}

}
//*************************************************************************************************


//*************************************************************************************************
/*!\defgroup core Core module */
//*************************************************************************************************


//*************************************************************************************************
/*!\mainpage pe Rigid Body Physics Engine Documentation
 *
 * \image html logo.jpg
 *
 *
 * \section intro Introduction
 *
 * Welcome to the \b pe physics engine documentation. This documentation is supposed to give
 * you any information you need in order to use the \b pe engine without any problems. The
 * documentation is structured in the following way. If you take a look to the top of this
 * html page, you will see several tabs. Currently you are on the "Main Page", which gives
 * an basic overview of the engine, provides some first impressions (see section \ref examples)
 * and is the starting point for all tutorials (see section \ref tutorials). If you are new to
 * the engine, this may be an excellent starting point to get a first look-and-feel for the
 * \b pe engine. The "Modules" tab gives you an overview of the different modules of the \b pe
 * engine and works as the primary reference point to get quickly to the information you are
 * looking for. The "Namespaces", "Classes" and "Files" tabs can also be used to quickly gain
 * access to specific parts of the engine. However, you should already have some experience
 * with the engine if you use these tabs, because you might get lost pretty quickly ;-)
 *
 *
 * \n \section goals Goals and focus
 *
 * The \b pe physics engine is a C++ framework for rigid body dynamic simulations. One of the
 * basic philosophies of the \b pe physics engine is the combination of fast and accurate rigid
 * body simulations. For this goal, the \b pe engine offers both algorithms for the physically
 * accurate rigid body simulation as well as algorithms focused primarily on feasible results
 * and performance. Additionally, the \b pe framework is a very flexible framework that allows
 * for an easy extension and addition of new features and algorithms. The following items sum
 * up the goals of the \b pe physics engine:
 *
 * - physically accurate rigid body dynamics
 * - rigid body dynamics suited for computer games
 * - extensibility
 * - exceptional C++ implementation
 * - intuitive and easy to use interface
 *
 *
 * \n \section tutorials Tutorials
 *
 * - \ref tutorial00\n
 *   This tutorial shows the very basics of the \b pe physics engine. It demonstrates the basic
 *   concepts and its naming conventions. This tutorial is a precondition for the later tutorials
 *   and is strongly recommended for everybody new to the \b pe engine before the first steps in
 *   Tutorial 1.
 *
 * - \ref tutorial01\n
 *   Tutorial 1 explains the first steps in creating a rigid body simulation with the \b pe engine.
 *
 * - \ref tutorial02\n
 *   The focus of tutorial 2 is the POV-Ray module of the \b pe engine. It demonstrates the use
 *   of the POV-Ray writer, camera, light sources and textures.
 *
 * - \ref tutorial03\n
 *   Tutorial 1 only demonstrated the setup of geometric primitives. Tutorial 3 explains the setup
 *   of compound geometries in the \b pe engine.
 *
 * - \ref tutorial04\n
 *   The fourth tutorial demonstrates how it is possible to run MPI parallel simulations with the
 *   \b pe physics engine. It explains the differences to standard simulations and all additional
 *   functionality for a MPI parallel execution.
 *
 *
 * \n \section examples Simulation examples
 *
 * \subsection cradle Newton's cradle
 * A first example for rigid body dynamics is the classical Newton's cradle scenario: one of five
 * equally sized spheres hits four spheres in a row, which results in the fifth sphere starting
 * to move with exactly the same momentum and kinetic energy as the first sphere, which in turn
 * stops moving.\n\n
 * \image html cradle.jpg
 *
 * \subsection billard Billard
 * The following pictures show another very common example for rigid body physics: the billard
 * game. Although the billard table is only approximated very crudely, the images give a nice
 * impression of a realistic scenario for the \b pe physics engine.\n\n
 * \image html billard.jpg
 *
 * \subsection wreckingball Wrecking ball
 * The following images show the simulation of a wrecking ball consisting of three interlocked
 * chain links. This simulation demonstrates the capability to create compound geometries from
 * geometric primitives.\n\n
 * \image html wreckingball.jpg
 *
 * \subsection generator Sphere generator
 * The sphere generator is a special scenario for the demonstration of a large number of random
 * rigid bodies piling in a narrow cavity. The "generator" to the upper left shoots random rigid
 * bodies into the cavity, which quickly start to interact with lots of other rigid bodies.\n\n
 * \image html generator.jpg
 *
 * \subsection well Rigid body well
 * The "well" example demonstrates the simulation of a large number of random, interacting rigid
 * bodies. The simulation demonstrated in the images handles 5000 so-called tristar unions (each
 * consisting of three capsules).\n\n
 * \image html well.jpg
 *
 * \subsection chain Chain
 * A demonstration for the capabilities of the union compound geometry is the simulation of several
 * hundred interlinked chain links. Every single link consists of four capsules that are combined
 * within a union.
 * \image html chain2.jpg
 *
 * \subsection castle Castle
 * A very sophisticated example is the "castle" scenario that shows the destruction of an ancient
 * castle by a huge swinging ram. The castle itself is built of several hundred boxes, the ram
 * is a single capsule geometry and the chain consists of several hundred interlinked chain links,
 * each consisting of four individual capsules.
 * \image html castle.jpg
 *
 *
 * \n \section acknowledgements Acknowledgements
 *
 * The \b pe physics engine makes use of the following open source libraries:
 *
 * \image html featuring.jpg
 */
//*************************************************************************************************

#endif
