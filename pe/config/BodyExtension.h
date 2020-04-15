//=================================================================================================
/*!
 *  \file pe/config/BodyExtension.h
 *  \brief Configuration class for basic properties of all rigid bodies
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
/*!\brief Configuration class for basic properties of all rigid bodies.
 * \ingroup config
 *
 * The BodyExtension class offers the possibility to extend rigid bodies and to adapt them
 * to a specific simulation purpose or to add specific properties (data and functionality)
 * to all rigid bodies.\n
 * The BodyExtension class is a direct base class for the RigidBody class that is used as
 * base class for all rigid bodies of the \b pe physics engine. By adding user-specific data
 * and/or functions to the BodyExtension class, it is possible to extend the functionality
 * for all rigid bodies. The following example demonstrates how a user-specific charge is
 * added to all rigid bodies. Note that this charge is NOT used by the physics engine to
 * automatically calculate electric-magnetic forces acting between the rigid bodies, but
 * that this is only a possibility to store user-defined data with every single rigid body.

   \code
   class BodyExtension
   {
    public:
      inline real getCharge() const { return charge_; }
      inline void setCharge( real charge ) { charge_ = charge; }

    private:
      real charge_;
   };
   \endcode

 * In order to avoid naming ambiguities between user-specific data and function members and
 * already defined \b pe specific data and function members, please take a look at the details
 * of the RigidBody class description!
 */
class BodyExtension
{};
//*************************************************************************************************
