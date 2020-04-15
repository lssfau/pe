//=================================================================================================
/*!
 *  \file pe/vtk/WriterID.h
 *  \brief Id definition for the vtk writer.
 *
 *  Copyright (C) 2012 Simon Bogner
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

#ifndef _PE_VTK_WRITERID_H_
#define _PE_VTK_WRITERID_H_

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/shared_ptr.hpp>


namespace pe {

namespace vtk {

//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

class Writer;


//*************************************************************************************************
/*!\brief Handle for an VTK Writer.
 * \ingroup vtk
 */
typedef boost::shared_ptr<Writer>  WriterID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant VTK Writer.
 * \ingroup vtk
 */
typedef boost::shared_ptr<const Writer>  ConstWriterID;
//*************************************************************************************************

} // namespace vtk

} // namespace pe

#endif /* WRITERID_H_ */
