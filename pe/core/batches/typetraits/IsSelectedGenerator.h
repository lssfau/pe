//=================================================================================================
/*!
 *  \file pe/core/batches/typetraits/IsSelectedGenerator.h
 *  \brief Header file for the IsSelectedGenerator type trait
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

#ifndef _PE_CORE_BATCHES_TYPETRAITS_ISSELECTEDGENERATOR_H_
#define _PE_CORE_BATCHES_TYPETRAITS_ISSELECTEDGENERATOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/batches/typetraits/IsSameGenerator.h>
#include <pe/util/FalseType.h>
#include <pe/util/TrueType.h>
#include <pe/system/Collisions.h>


namespace pe {

namespace batches {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Compile time check for batch generator types.
 * \ingroup batch_generation_type_traits
 *
 * This class tests if the given data type \a A is the compile time selected batch generator
 * type pe::pe_BATCH_GENERATOR. If \a A is the same data type, then the \a value member
 * enumeration is set to 1, the nested type definition \a Type is \a TrueType, and the class
 * derives from \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the
 * class derives from \a FalseType.\n
 * The following examples demonstrates the use of the type trait with the assumption that
 * pe::pe_BATCH_GENERATOR is set to UnionFind:

   \code
   pe::IsSelectedGenerator< UnionFind >::value    // Evaluates to 1
   pe::IsSelectedGenerator< UnionFind >::Type     // Results in TrueType
   pe::IsSelectedGenerator< UnionFind >           // Is derived from TrueType
   pe::IsSelectedGenerator< SingleBatch >::value  // Evaluates to 0
   pe::IsSelectedGenerator< SingleBatch >::Type   // Results in FalseType
   pe::IsSelectedGenerator< SingleBatch>          // Is derived from FalseType
   \endcode
 */
template< template<typename> class T >
struct IsSelectedGenerator : public IsSameGenerator<T,pe_BATCH_GENERATOR>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsSameGenerator<T,pe_BATCH_GENERATOR>::value };
   typedef typename IsSameGenerator<T,pe_BATCH_GENERATOR>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace batches

} // namespace pe

#endif
