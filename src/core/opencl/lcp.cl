#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable
#pragma OPENCL EXTENSION cl_khr_global_int32_extended_atomics : enable
/*
__kernel void FillBuffer(__global float4* buffer, const uint size,
   const float4 value) {

   uint pos = get_global_id(0);
   const uint stride = get_num_groups(0) * get_local_size(0);

   while(pos < size) {
      buffer[pos] = value;
      pos += stride;
   }
}
*/

/*!\brief Fills a buffer with a constant value.
// \ingroup opencl
//
// \param buffer The OpenCL buffer to fill.
// \param size The number of elements to assign to.
// \param value The value to assign.
*/
__kernel void FillBuffer(__global uint* buffer, const uint size,
   const uint value) {

   uint pos = get_global_id(0);
   const uint stride = get_num_groups(0) * get_local_size(0);

   while(pos < size) {
      buffer[pos] = value;
      pos += stride;
   }
}

/*!\brief Initializes contact centric buffers from body centric buffers.
// \ingroup opencl
//
// \param contactsInvMass The contact storage for the inverse mass.
// \param contactsInvInertia The contact storage for the inverse inertia tensor.
// \param bodiesInvMass The body storage for the inverse mass.
// \param bodiesInvInertia The body storage for the inverse inertia tensor.
// \param bodiesMutex The body storage for the mutexes.
// \param bodies The bodies involved in each contact.
// \param status The status of each contact (active or inactive).
// \param size The number of contacts.
// \param offsetContacts Size of one component of the contact storage data in the contacts* buffers.
// \param offsetBodies Size of one component of the body storage data in the bodies* buffers.
//
// Copy the inverse mass and the inverse inertia tensors from body centric
// buffers to contact centric buffers. Also initializes the mutexes in the
// body centric data.
*/
__kernel void InitContactsMassInertia(__global float* contactsInvMass,
   __global float4* contactsInvInertia, __global const float* bodiesInvMass,
   __global const float4* bodiesInvInertia, __global uint* bodiesMutex,
   __global const uint2* bodies, __global const uint* status, const uint size,
   const uint offsetContacts, const uint offsetBodies) {

   uint index = get_global_id(0);
   const uint stride = get_num_groups(0) * get_local_size(0);

   while(index < size && status[index]) {
      uint2 bIDs = bodies[index];

      // inverse mass
      contactsInvMass[index] = bodiesInvMass[bIDs.s0];
      contactsInvMass[index + offsetContacts] = bodiesInvMass[bIDs.s1];

      // inverse inertia
      uint pos = index;

      // body A
      for(uint i = 0; i < 3; ++i, pos += offsetContacts)
         contactsInvInertia[pos] = bodiesInvInertia[bIDs.s0 + i * offsetBodies];

      // body B
      for(uint i = 0; i < 3; ++i, pos += offsetContacts)
         contactsInvInertia[pos] = bodiesInvInertia[bIDs.s1 + i * offsetBodies];

      // init lock
      bodiesMutex[bIDs.s0] = 0;
      bodiesMutex[bIDs.s1] = 0;

      index += stride;
   }
}

/*!\brief Copy the body positions and velocities from body centric buffers to contact centric buffers.
// \ingroup opencl
//
// \param contactsGPos The contact storage for positions of the involved bodies.
// \param contactsLinVel The contact storage for linear velocities of the involved bodies.
// \param contactsAngVel The contact storage for angular velocities of the involved bodies.
// \param bodiesGpos The body storage for body positions.
// \param bodiesLinVel The body storage for linear velocities.
// \param bodiesAngVel The body storage for angular velocities.
// \param bodies The bodies involved in each contact.
// \param status The status of each contact (active or inactive).
// \param size The number of contacts.
// \param offsetContacts Size of one component of the contact storage data in the contacts* buffers.
*/
__kernel void InitContactsCoordinates(__global float4* contactsGPos,
   __global float4* contactsLinVel, __global float4* contactsAngVel,
   __global const float4* bodiesGPos, __global const float4* bodiesLinVel,
   __global const float4* bodiesAngVel, __global const uint2* bodies,
   __global const uint* status, const uint size, const uint offsetContacts) {

   uint index = get_global_id(0);
   const uint stride = get_num_groups(0) * get_local_size(0);

   while(index < size && status[index]) {
      uint2 bIDs = bodies[index];

      // global position
      contactsGPos[index] = bodiesGPos[bIDs.s0];
      contactsGPos[index + offsetContacts] = bodiesGPos[bIDs.s1];

      // linear velocity
      contactsLinVel[index] = bodiesLinVel[bIDs.s0];
      contactsLinVel[index + offsetContacts] = bodiesLinVel[bIDs.s1];

      // angular velocity
      contactsAngVel[index] = bodiesAngVel[bIDs.s0];
      contactsAngVel[index + offsetContacts] = bodiesAngVel[bIDs.s1];

      index += stride;
   }
}

/*!\brief Compute the predicted contact gaps based on current linear and angular velocity estimates.
// \ingroup opencl
//
// \param contactsPhi The contact storage for contact gaps.
// \param contactsR The contact storage for the contact positions relative to the centroids of the involved bodies.
// \param contactsGPos The contact storage for positions of the involved bodies.
// \param contactsLinVel The contact storage for linear velocities of the involved bodies.
// \param contactsAngVel The contact storage for angular velocities of the involved bodies.
// \param bodies The bodies involved in each contact.
// \param size The number of contacts.
// \param offset Offset within one component of the contact storage data to the data of the current color.
// \param offsetContacts Size of one component of the contact storage data in the contacts* buffers.
// \param dt The current time step size.
*/
__kernel void CalcPhi(__global float4* contactsPhi,
   __global const float4* contactsR, __global const float4* contactsGPos,
   __global const float4* bodiesLinVel, __global const float4* bodiesAngVel,
   __global const uint2* bodies, const uint size, const uint offset,
   const uint offsetContacts, const float dt) {

   uint index = get_global_id(0);
   const uint stride = get_num_groups(0) * get_local_size(0);

   while(index < size) {
      const uint contactId = index + offset;

      // body IDs
      uint2 bIDs = bodies[contactId];

      // body A
      uint pos = contactId;
      float4 rA = contactsR[pos];
      float4 pA = contactsGPos[pos] + bodiesLinVel[bIDs.s0] * dt + rA -
         cross(rA, bodiesAngVel[bIDs.s0]) * dt;

      // body B
      pos += offsetContacts;
      float4 rB = contactsR[pos];
      float4 pB = contactsGPos[pos] + bodiesLinVel[bIDs.s1] * dt + rB -
         cross(rB, bodiesAngVel[bIDs.s1]) * dt;

      // phi
      float4 phi = pA - pB;
      phi.w = 0.0f;
      contactsPhi[contactId] = phi;

      index += stride;
   }
}

/*!\brief Compute the contact reactions based on the predicted contact gaps.
// \ingroup opencl
//
// \param contactsNormal The contact storage for the local coordinate frames with contact normal and tangentials.
// \param contactsDiagonal The contact storage for the 3x3 diagonal blocks of the system matrix.
// \param contactsPhi The contact storage for the predicted contact gaps on input. On output the contact reaction changes are stored here.
// \param contactsP The contact storage for the contact reactions. Is updated by the contact reaction changes.
// \param contactsMu The contact storage for the coefficients of friction.
// \param size The number of contacts.
// \param offset Offset within one component of the contact storage data to the data of the current color.
// \param offsetContacts Size of one component of the contact storage data in the contacts* buffers.
// \param updateResidual Flag indicating whether to compute residuals or not. The residual values are stored in the fourth value of each contact reaction tuple in contactsP as the absolute value of the contact reaction change.
*/
__kernel void CalcP(__global const float4* contactsNormal,
   __global const float4* contactsDiagonal, __global float4* contactsPhi,
   __global float4* contactsP, __global const float* contactsMu,
   const uint size, const uint offset, const uint offsetContacts,
   const uint updateResidual) {

   uint index = get_global_id(0);
   const uint stride = get_num_groups(0) * get_local_size(0);

   while(index < size) {
      const uint contactId = index + offset;
      uint pos = contactId;

      float4 phi = contactsPhi[contactId];

      float4 phiC;
      phiC.x = -dot(contactsNormal[pos], phi);
      pos += offsetContacts;
      phiC.y = -dot(contactsNormal[pos], phi);
      pos += offsetContacts;
      phiC.z = -dot(contactsNormal[pos], phi);
      phiC.w = 0.0f;

      pos = contactId;
      float4 dx;
      dx.x = dot(contactsDiagonal[pos], phiC);
      pos += offsetContacts;
      dx.y = dot(contactsDiagonal[pos], phiC);
      pos += offsetContacts;
      dx.z = dot(contactsDiagonal[pos], phiC);
      dx.w = 0.0f;

      float4 p = contactsP[contactId];
      float flimit = p.x * contactsMu[contactId];
      p.w = 0.0f;

      float4 pNew = p + dx;

      pNew.x = max(0.0f, pNew.x);
      pNew.y = max(-flimit, min(flimit, pNew.y));
      pNew.z = max(-flimit, min(flimit, pNew.z));

      dx = pNew - p;
      p = pNew;

      if(updateResidual)
         p.w = max(fabs(dx.x), max(fabs(dx.y), fabs(dx.z)));

      // use phi as temporary storage for dx
      contactsPhi[contactId] = dx;

      contactsP[contactId] = p;

      index += stride;
   }
}

/*!\brief Compute the incremental velocity changes of the bodies involved in collisions.
// \ingroup opencl
//
// \param contactsNormal The contact storage for the local coordinate frames with contact normal and tangentials.
// \param contactsPhi The contact storage for contact gaps abused for contact reaction changes.
// \param contactsInvMass The contact storage for the inverse mass.
// \param contactsInvInertia The contact storage for the inverse inertia tensor.
// \param contactsReductionInfo Information on where in the update cache to store the calculated velocity changes.
// \param contactsR The contact storage for the contact positions relative to the centroids of the involved bodies.
// \param cacheCount Auxiliary data generated for supplying it to the segmented reduction of the incremental velocity changes.
// \param cacheV Update cache buffer for linear velocities.
// \param cacheW Update cache buffer for angular velocities.
// \param size The number of contacts.
// \param offset Offset within one component of the contact storage data to the data of the current color.
// \param offsetContacts Size of one component of the contact storage data in the contacts* buffers.
// \param omega The under- or overrelaxation parameter. Must be in the range (0; 2).
*/
__kernel void CalcVelocities(__global const float4* contactsNormal,
   __global const float4* contactsPhi, __global const float* contactsInvMass,
   __global const float4* contactsInvInertia,
   __global const int4* contactsReductionInfo,
   __global const float4* contactsR, __global uint* cacheCount,
   __global float4* cacheV, __global float4* cacheW,
   uint size, uint offset, uint offsetContacts, float omega) {

   uint index = get_global_id(0);
   const uint stride = get_num_groups(0) * get_local_size(0);

   while(index < size) {
      const uint contactId = index + offset;
      uint pos = contactId;

      // phi used as temporary storage for dx
      float4 dx = contactsPhi[contactId];

      float4 pwf = contactsNormal[pos] * dx.x;
      pos += offsetContacts;
      pwf += contactsNormal[pos] * dx.y;
      pos += offsetContacts;
      pwf += contactsNormal[pos] * dx.z;
      pwf *= omega;
      pwf.w = 0.0f;

      // cache management
      int4 redInfo = contactsReductionInfo[contactId];
      int2 cacheOffsets = redInfo.s01;
      if(cacheOffsets.s0 != -1)
         cacheCount[redInfo.s0] = redInfo.s2;
      if(cacheOffsets.s1 != -1)
         cacheCount[redInfo.s1] = redInfo.s3;

      //
      // body A
      //
      pos = contactId;

      float4 deltaV = pwf * contactsInvMass[pos];
      deltaV.w = 0.0f;
      if(cacheOffsets.s0 != -1)
         cacheV[cacheOffsets.s0] = deltaV;

      // rCrossed = r x pwf
      float4 rCrossed = cross(contactsR[pos], pwf);

      // deltaW = Iinv * (r x pwf)
      float4 deltaW;
      deltaW.x = dot(contactsInvInertia[pos], rCrossed);
      pos += offsetContacts;
      deltaW.y = dot(contactsInvInertia[pos], rCrossed);
      pos += offsetContacts;
      deltaW.z = dot(contactsInvInertia[pos], rCrossed);
      deltaW.w = 0.0f;
      if(cacheOffsets.s0 != -1)
         cacheW[cacheOffsets.s0] = deltaW;

      //
      // body B
      //
      pwf = -pwf;
      pos = contactId + offsetContacts;

      deltaV = pwf * contactsInvMass[pos];
      deltaV.w = 0.0f;
      if(cacheOffsets.s1 != -1)
         cacheV[cacheOffsets.s1] = deltaV;


      // rCrossed = r x -pwf
      rCrossed = cross(contactsR[pos], pwf);

      // deltaW = Iinv * (r x -pwf)
      pos += 2 * offsetContacts;
      deltaW.x = dot(contactsInvInertia[pos], rCrossed);
      pos += offsetContacts;
      deltaW.y = dot(contactsInvInertia[pos], rCrossed);
      pos += offsetContacts;
      deltaW.z = dot(contactsInvInertia[pos], rCrossed);
      deltaW.w = 0.0f;
      if(cacheOffsets.s1 != -1)
         cacheW[cacheOffsets.s1] = deltaW;

      index += stride;
   }
}

/*!\brief Sum up the incremental velocity changes of the bodies involved in collisions to the velocities in the body storage.
// \ingroup opencl
//
// \param contactsNormal The contact storage for the local coordinate frames with contact normal and tangentials.
// \param contactsPhi The contact storage for contact gaps abused for contact reaction changes.
// \param contactsInvMass The contact storage for the inverse mass.
// \param contactsInvInertia The contact storage for the inverse inertia tensor.
// \param contactsR The contact storage for the contact positions relative to the centroids of the involved bodies.
// \param bodies The bodies involved in each contact.
// \param bodiesLinVel The body storage for linear velocities.
// \param bodiesAngVel The body storage for angular velocities.
// \param size The number of contacts.
// \param offset Offset within one component of the contact storage data to the data of the current color.
// \param offsetContacts Size of one component of the contact storage data in the contacts* buffers.
// \param omega The under- or overrelaxation parameter. Must be in the range (0; 2).
*/
__kernel void CalcVelocitiesAtomic(__global const float4* contactsNormal,
   __global const float4* contactsPhi, __global const float* contactsInvMass,
   __global const float4* contactsInvInertia,
   __global const float4* contactsR,
   __global const uint2* bodies,
   __global float4* bodiesLinVel,
   __global float4* bodiesAngVel,
   __global uint* bodiesMutex,
   uint size, uint offset, uint offsetContacts, float omega) {

   uint index = get_global_id(0);

   if(index >= size)
      return;

   index += offset;
   uint pos = index;

   // phi used as temporary storage for dx
   float4 dx = contactsPhi[index];

   float4 pwf = contactsNormal[pos] * dx.x;
   pos += offsetContacts;
   pwf += contactsNormal[pos] * dx.y;
   pos += offsetContacts;
   pwf += contactsNormal[pos] * dx.z;
   pwf *= omega;

   // body IDs
   uint2 bIDs = bodies[index];

   //
   // body A
   //
   pos = index;

   float4 deltaV = pwf * contactsInvMass[pos];
   deltaV.w = 0.0f;

   // rCrossed = r x pwf
   float4 rCrossed = cross(contactsR[pos], pwf);

   // deltaW = Iinv * (r x pwf)
   float4 deltaW;
   deltaW.x = dot(contactsInvInertia[pos], rCrossed);
   pos += offsetContacts;
   deltaW.y = dot(contactsInvInertia[pos], rCrossed);
   pos += offsetContacts;
   deltaW.z = dot(contactsInvInertia[pos], rCrossed);
   deltaW.w = 0.0f;

   int locked = 0;
   do {
      locked = atom_cmpxchg(bodiesMutex + bIDs.s0, 0, 1);

      if(!locked) {
         bodiesLinVel[bIDs.s0] += deltaV;
         bodiesAngVel[bIDs.s0] += deltaW;
         bodiesMutex[bIDs.s0] = 0;
      }
      barrier(CLK_LOCAL_MEM_FENCE);
   } while(locked);

   //
   // body B
   //
   pwf = -pwf;
   pos = index + offsetContacts;

   deltaV = pwf * contactsInvMass[pos];
   deltaV.w = 0.0f;

   // rCrossed = r x -pwf
   rCrossed = cross(contactsR[pos], pwf);

   // deltaW = Iinv * (r x -pwf)
   pos += 2 * offsetContacts;
   deltaW.x = dot(contactsInvInertia[pos], rCrossed);
   pos += offsetContacts;
   deltaW.y = dot(contactsInvInertia[pos], rCrossed);
   pos += offsetContacts;
   deltaW.z = dot(contactsInvInertia[pos], rCrossed);
   deltaW.w = 0.0f;

   do {
      locked = atom_cmpxchg(bodiesMutex + bIDs.s1, 0, 1);

      if(!locked) {
         bodiesLinVel[bIDs.s1] += deltaV;
         bodiesAngVel[bIDs.s1] += deltaW;
         bodiesMutex[bIDs.s1] = 0;
      }
      barrier(CLK_LOCAL_MEM_FENCE);
   } while(locked);
}

/*!\brief Reduces the incremental velocity changes of the bodies involved in collisions.
// \ingroup opencl
//
// \param cacheCount Auxiliary data for the segmented reduction of the incremental velocity changes. Initially the elements in the segments need to be numbered consecutively starting from 0.
// \param cacheV Update cache buffer for linear velocities.
// \param cacheW Update cache buffer for angular velocities.
// \param size The number of cache entries for the current color.
// \param step Equals 1 for the first reduction step and is incremented for each further reduction step.
*/
__kernel void ReduceVelocities(__global uint* cacheCount,
   __global float4* cacheV, __global float4* cacheW, const uint size,
   const uint step) {

   uint pos = get_global_id(0);
   const uint stride = get_num_groups(0) * get_local_size(0);

   while(pos < size) {
      uint reductionCount = cacheCount[pos];
      const uint offset = 0x1 << (step - 1);
      const uint next = pos - offset;

      if(reductionCount & offset) {
         cacheV[next] += cacheV[pos];
         cacheW[next] += cacheW[pos];
         cacheCount[pos] = 0;
      }
      pos += stride;
   }
}

/*!\brief Transfers the reduced velocity changes to the body centric data structures.
// \ingroup opencl
//
// \param offsets Contains as off index baseOffset the index of the reduced velocity update in the update cache for each body. Typically the bases vector of OpenCLColoredContacts is used with baseOffset chosen accordingly.
// \param cacheV Update cache buffer for linear velocities.
// \param cacheW Update cache buffer for angular velocities.
// \param bodiesLinVel The body storage for linear velocities.
// \param bodiesAngVel The body storage for angular velocities.
// \param bodiesContacts Marks which colors are next to each body.
// \param size The number of bodies.
// \param color The color of the contacts relaxed in the current sweep.
// \param baseOffset Offset into the offsets buffer as off which the indices into the update cache are stored consecutively for all bodies.
*/
__kernel void UpdateVelocities(__global const uint* offsets,
   __global const float4* cacheV, __global const float4* cacheW,
   __global float4* bodiesLinVel, __global float4* bodiesAngVel,
   __global const uint* bodiesContacts, const uint size, const uint color,
   const uint baseOffset) {

   uint pos = get_global_id(0);
   const uint stride = get_num_groups(0) * get_local_size(0);

   while(pos < size) {
      bool active = (bodiesContacts[pos] >> color) & 0x1;
      if(active) {
         uint offset = offsets[baseOffset + pos];

         bodiesLinVel[pos] += cacheV[offset];
         bodiesAngVel[pos] += cacheW[offset];
      }
      pos += stride;
   }
}

/*!\brief Reduces the fourth value in each tuple of the contactsP buffer by selecting the maximum.
// \ingroup opencl
//
// \param contactsP The contact storage for the contact reactions. The fourth value typically includes the residual value.
// \param residual The output buffer. Each workgroup stores its reduced value as off offset.
// \param buffer A temporary reduction buffer in the local storage comprising one float entry per workgroup item.
// \param size The number of contacts.
// \param offset Offset as off which reduction results should be written in contactsP.
*/
__kernel void ReduceResidual(__global const float4* contactsP, __global float* residual,
   __local float* buffer, const uint size, const uint offset) {

   const uint idx = get_local_id(0);
   uint pos = get_global_id(0);
   const uint stride = get_num_groups(0) * get_local_size(0);

   buffer[idx] = 0.0f;

   while(pos < size) {
      buffer[idx] = max(buffer[idx], contactsP[pos].w);
      pos += stride;
   }

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 64)
      buffer[idx] = max(buffer[idx], buffer[idx + 64]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 32)
      buffer[idx] = max(buffer[idx], buffer[idx + 32]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 16)
      buffer[idx] = max(buffer[idx], buffer[idx + 16]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 8)
      buffer[idx] = max(buffer[idx], buffer[idx + 8]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 4)
      buffer[idx] = max(buffer[idx], buffer[idx + 4]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 2)
      buffer[idx] = max(buffer[idx], buffer[idx + 2]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(!idx) {
      float res = max(buffer[0], buffer[1]);
      residual[offset + get_group_id(0)] = res;
   }
}

/*!\brief Reduces the velocity magnitudes by selecting the maximum.
// \ingroup opencl
//
// \param bodiesLinVel The body storage for linear velocities whose magnitudes are to be reduced.
// \param bodiesStatus Identifies fixed bodies by zero values.
// \param maxVelocity The output buffer. Each workgroup stores its reduced value as off the beginning of the buffer.
// \param buffer A temporary reduction buffer in the local storage comprising one float entry per workgroup item.
// \param size The number of bodies.
*/
__kernel void ReduceMaxVelocity(__global const float4* bodiesLinVel,
   __global const uint* bodiesStatus, __global float* maxVelocity,
   __local float* buffer, const uint size) {

   const uint idx = get_local_id(0);
   uint pos = get_global_id(0);
   const uint stride = get_num_groups(0) * get_local_size(0);

   buffer[idx] = 0.0f;

   while(pos < size) {
      if(bodiesStatus[pos]) {
         float4 vel = bodiesLinVel[pos];
         vel.w = 0.0f;
         buffer[idx] = max(buffer[idx], length(vel));
      }
      pos += stride;
   }

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 64)
      buffer[idx] = max(buffer[idx], buffer[idx + 64]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 32)
      buffer[idx] = max(buffer[idx], buffer[idx + 32]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 16)
      buffer[idx] = max(buffer[idx], buffer[idx + 16]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 8)
      buffer[idx] = max(buffer[idx], buffer[idx + 8]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 4)
      buffer[idx] = max(buffer[idx], buffer[idx + 4]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(idx < 2)
      buffer[idx] = max(buffer[idx], buffer[idx + 2]);

   barrier(CLK_LOCAL_MEM_FENCE);

   if(!idx) {
      float res = max(buffer[0], buffer[1]);
      maxVelocity[get_group_id(0)] = res;
   }
}

