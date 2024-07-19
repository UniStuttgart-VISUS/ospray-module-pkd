// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

// ======================================================================== //
// Modified 2018-2020 VISUS - University Stuttgart                          //
// ======================================================================== //

// ospray
#include "common/Data.ih"
#include "common/Ray.ih"
#include "common/Intersect.ih"
#include "geometry/Geometry.ih"
#include "rkcommon/math/box.ih"
#include "rkcommon/math/vec.ih"
// this module
#include "PKDGeometryShared.h"
#include "PKDGeometry.ih"

// uniform int rayID = 0;

#define LOD 0

inline varying bool PKDGeometry_intersectPrim(PKDGeometry *uniform self,
    uniform PKDParticle &p,
    uniform primID_t primID,
    varying Ray &ray,
    const struct RTCIntersectFunctionNArguments *uniform args)
{
  // Ray Tracing Gems Chapter 7
  const vec3f oc = ray.org - make_vec3f(p.position[0], p.position[1], p.position[2]);
  const float sqrRad = self->global_radius * self->global_radius;

  const float b = dot(-1.0f * oc, ray.dir);
  const vec3f temp = oc + b * ray.dir;
  const float delta = sqrRad - dot(temp, temp);

  if (delta < 0.0f)
    return false;

  const float c = dot(oc, oc) - sqrRad;
  const float sign = signbits(b) ? 1.0f : -1.0f;
  const float q = b + sign * sqrt(delta);

  {
    const float temp = min(c / q, q);
    if (temp > ray.t0 && temp < ray.t) {
      ray.primID = primID;
      ray.geomID = args->geomID;
      ray.t = temp;
      ray.Ng = ray.t * ray.dir + oc;
      return true;
    }
  }
  return false;
}

struct ThreePhaseStackEntry
{
  varying float t_in, t_out, t_sphere_out;
  uniform primID_t sphereID;
  uniform primID_t farChildID;
};

uniform float get_coord(uniform vec3f const& p, int dim) {
  if (dim == 0)
    return p.x;
  if (dim == 1)
    return p.y;
  if (dim == 2)
    return p.z;
}

uniform unsigned int get_dim(uniform float const x) {
  return (uniform unsigned int)x & 3;
}

inline void pkd_traverse_packet(uniform PKDGeometry *uniform self,
    varying Ray &ray,
    const varying float rdir[3],
    const varying float org[3],
    const varying float t_in_0,
    const varying float t_out_0,
    const uniform unsigned int dir_sign[3],
    const uniform bool isShadowRay,
    const struct RTCIntersectFunctionNArguments *uniform args)
{
  // ++rayID;
  // print("Ray %\n",rayID);
  //uniform bool dbg = false; //(rayID == 338);

  varying ThreePhaseStackEntry stack[32];
  varying ThreePhaseStackEntry *uniform stackPtr = stack;

  uniform primID_t nodeID = 0;
  uniform unsigned int dim = 0;

  float t_in = t_in_0;
  float t_out = t_out_0;
  const float radius = self->global_radius;
  const uniform primID_t numInnerNodes = self->num_innerNodes;
  const uniform primID_t numParticles = self->num_particles;
  const uniform PKDParticle *uniform const particle =
      (PKDParticle * uniform const) self->position;
  //if (dbg) print("ENTER TRAVERSAL\n");
  uniform PKDParticle p;
  while (1) {
    // ------------------------------------------------------------------
    // do traversal step(s) as long as possible
    // ------------------------------------------------------------------
    // if (dbg) print("TRAV %\n",nodeID);
    while (1) {
      // if (dbg) print("DOWN %\n",nodeID);

      if (t_in > t_out)
        break;

      //getParticle(self, p, nodeID);
      p = particle[nodeID];

      if (nodeID >= numInnerNodes) {
        // this is a leaf node - can't to to a leaf, anyway. Intersect
        // the prim, and be done with it.
        // if (dbg) print("LEAFISEC0\n");
        PKDGeometry_intersectPrim(self, p, nodeID, ray, args);
        // if (dbg) print("LEAFISEC1\n");
        //if (dbg) print("LEAFISEC1 % %\n", nodeID, numInnerNodes);
        /*if (dbg)
          print("POS % % %\n", p.position[0], p.position[1], p.position[2]);*/
        if (isShadowRay && ray.primID >= 0)
          return;
        break;
      }

      // TODO: This is cullign incorrectly?
      /*if (self->innerNode_attributeMask) {
        const uniform uint32 nodeAttrBits =
            self->innerNode_attributeMask[nodeID];
        if ((nodeAttrBits & self->transferFunction_activeBinBits) == 0)
          break;
      }*/

      // #if !DIM_FROM_DEPTH
      // INT3 *uniform intPtr = (INT3 *uniform)self->particle;
      // dim = intPtr[nodeID].x & 3;
      //dim = get_dim(p.position[0]);
      //dim = ((unsigned int)p.position[0]) & 3;
      {
        const uniform float *uniform pos = &p.position[0];        
        dim = ((unsigned int *uniform)pos)[0] & 3;
      }
      // #endif
      /*if (dbg)
        print("DIM %\n", dim);*/

      const uniform unsigned int sign = dir_sign[dim];

      // ------------------------------------------------------------------
      // traversal step: compute distance, then compute intervals for front and
      // back side
      // ------------------------------------------------------------------
      const float org_to_node_dim = p.position[dim] - org[dim];
      //const float org_to_node_dim = get_coord(p, dim) - org[dim];

      const float t_plane_0 = (org_to_node_dim - radius) * rdir[dim];
      const float t_plane_1 = (org_to_node_dim + radius) * rdir[dim];

      const float t_plane_nr = min(t_plane_0, t_plane_1);
      const float t_plane_fr = max(t_plane_0, t_plane_1);

      const float t_farChild_in = max(t_in, t_plane_nr);
      const float t_farChild_out = t_out;
      const float t_nearChild_out = min(t_out, t_plane_fr);

      // catch the case where all ray segments are on far side
      if (none(t_in < t_nearChild_out)) {
        if (none(t_farChild_in < t_farChild_out)) {
          break;
        } else {
          t_in = t_farChild_in;
          t_out = t_farChild_out;
          nodeID = 2 * nodeID + 2 - sign;
          continue;
        }
      }

      unmasked
      {
        stackPtr->t_in = 1e20f;
        stackPtr->t_out = -1e20f;
        stackPtr->t_sphere_out = -1e20f;
      }
      stackPtr->farChildID = 2 * nodeID + 2 - sign;

      stackPtr->t_in = t_farChild_in;
      stackPtr->t_out = t_farChild_out;
      stackPtr->t_sphere_out = t_nearChild_out;

      t_out = t_nearChild_out;

      stackPtr->sphereID = nodeID;

      if (any(t_farChild_in < t_farChild_out))
        ++stackPtr;

      if (none(t_in < t_out))
        break;

      // if (dbg) print("nodeID a0 %\n",nodeID);
      nodeID = min(2 * nodeID + 1 + sign, numParticles - 1);
      // if (dbg) print("nodeID a1 %\n",nodeID);
      continue;
    }
    // ------------------------------------------------------------------
    // couldn't go down any further; pop a node from stack
    // ------------------------------------------------------------------
    while (1) {
      // if (dbg) print("POP\n");
      // pop as long as we have to ... or until nothing is left to pop.
      /*if (dbg)
        print("POP\n");*/
      if (stackPtr == stack) {
        /*if (dbg)
          print("LEAVE TRAVERSAL\n");*/
        return;
      }
      unmasked
      {
        t_in = stackPtr[-1].t_in;
        t_out = min(stackPtr[-1].t_out, ray.t);
      }
      --stackPtr;

      // check if the node is still active (all the traversal since it
      // originally got pushed may have shortened the ray)
      if (none(t_in < t_out))
        continue;

      // intersect the actual node...
      if (t_in < min(stackPtr->t_sphere_out, ray.t)) {
        /*uniform Particle p;
        getParticle(self, p, stackPtr->sphereID);*/
        uniform PKDParticle p = particle[stackPtr->sphereID];
        PKDGeometry_intersectPrim(self, p, stackPtr->sphereID, ray, args);
        if (isShadowRay && ray.primID >= 0)
          return;
      }

      // do the distance test again, we might just have shortened the ray...
      unmasked
      {
        t_out = min(t_out, ray.t);
      }
      // if (dbg) print("nodeID b0 %\n",nodeID);
      nodeID = min(stackPtr->farChildID, numParticles - 1);
      // if (dbg) print("nodeID b1 %\n",nodeID);
#if DIM_FROM_DEPTH
      dim = stackPtr->dim;
#endif
      break;
    }
  }
  // if (dbg) print("DONE TRAVERSAL\n");
  /*if (dbg)
    print("DONE TRAVERSAL\n");*/
}

/*! generic traverse/occluded function that splits the packet into
  subpackets of equal sige, and then calls the appropiate
  constant-sign traverse function. this method works for both shadow
  and primary rays, as indicated by the 'isShadowRay' flag */
inline void pkd_traverse_packet(uniform PKDGeometry *uniform self,
    varying Ray &ray,
    uniform bool isShadowRay,
    const struct RTCIntersectFunctionNArguments *uniform args)
{
  float t_in = ray.t0, t_out = ray.t;
  //intersectBox(ray, self->bounds, t_in, t_out);
  Intersections isect = intersectBox(ray.org, ray.dir, self->bounds);
  t_in = isect.entry.t;
  t_out = isect.exit.t;

  if (t_out < t_in)
    return;

  //print("TRAVERSE1\n");

  const varying float rdir[3] = {
      safe_rcp(ray.dir.x), safe_rcp(ray.dir.y), safe_rcp(ray.dir.z)};
  const varying float org[3] = {ray.org.x, ray.org.y, ray.org.z};

  uniform unsigned int dir_sign[3];
  if (ray.dir.z > 0.f) {
    dir_sign[2] = 0;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay, args);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay, args);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay, args);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay, args);
      }
    }
  } else {
    dir_sign[2] = 1;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay, args);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay, args);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay, args);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay, args);
      }
    }
  }
}

/*! the 'virtual' traverse function for a pkd geometry */
unmasked void PKDGeometry_intersect_packet(
    const struct RTCIntersectFunctionNArguments *uniform args)
{
  if (!args->valid[programIndex]) {
    return;
  }
  // this assumes that the args->rayhit is actually a pointer toa varying ray!
  varying Ray *uniform ray = (varying Ray * uniform) args->rayhit;
  uniform PKDGeometry *uniform self =
      (uniform PKDGeometry * uniform) args->geometryUserPtr;

  pkd_traverse_packet(self, *ray, false, args);
  if (ray->geomID == args->geomID) {
    ray->instID = args->context->instID[0];
  }
}

/*! the 'virtual' occluded function for a pkd geometry */
unmasked void PKDGeometry_occluded_packet(
    const struct RTCIntersectFunctionNArguments *uniform args)
{
  if (!args->valid[programIndex]) {
    return;
  }
  // this assumes that the args->rayhit is actually a pointer toa varying ray!
  varying Ray *uniform ray = (varying Ray * uniform) args->rayhit;
  uniform PKDGeometry *uniform self =
      (uniform PKDGeometry * uniform) args->geometryUserPtr;

  pkd_traverse_packet(self, *ray, true, args);
  if (ray->geomID == args->geomID) {
    ray->instID = args->context->instID[0];
    ray->t = neg_inf;
  }
}