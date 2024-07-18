// ospray
#include "common/Data.ih"
#include "common/DifferentialGeometry.ih"
#include "common/Ray.ih"
#include "geometry/Geometry.ih"
#include "rkcommon/math/box.ih"
#include "rkcommon/math/vec.ih"
// c++ chared
#include "PKDGeometryShared.h"

inline float safe_rcp(float f) 
{ return (abs(f) < 1e-20f)?1e20f:rcp(f); }

inline void intersectBox(varying const Ray& ray, uniform const box3f& bounds, varying float& t0, varying float& t1) {
    vec3f t_lower = (bounds.lower - ray.org) / ray.dir;
    vec3f t_upper = (bounds.upper - ray.org) / ray.dir;

    vec3f t_min3 = min(t_lower, t_upper);
    vec3f t_max3 = max(t_lower, t_upper);

    t0 = max(t0, max(t_min3.x, max(t_min3.y, t_min3.z)));
    t1 = min(t1, min(t_max3.x, min(t_max3.y, t_max3.z)));
}

inline varying bool PKDGeometry_intersectPrim(
                                                  uniform vec3f const& p,
                                                  uniform unsigned int const primID,
                                                  uniform float const sqrRad,
                                                  varying Ray &ray)
{
  // Ray Tracing Gems Chapter 7
  const vec3f oc = ray.org - p;

  const float b = dot(-1.0f*oc, ray.dir);
  const vec3f temp = oc + b*ray.dir;
  const float delta = sqrRad - dot(temp, temp);

  if (delta < 0.0f) return false;

  const float c = dot(oc, oc) - sqrRad;
  const float sign = signbits(b) ? 1.0f : -1.0f;
  const float q = b+sign*sqrt(delta);

  {
    const float temp = min(c/q,q);
    if (temp > ray.t0 && temp < ray.t) {
      ray.primID = primID;
      // ray.geomID = self->super.geomID;
      ray.t      = temp;
      ray.Ng     = ray.t * ray.dir + oc;
      return true;
    }
  }
  return false;
}

struct ThreePhaseStackEntry
{
  varying float t_in, t_out, t_sphere_out;
  uniform unsigned int sphereID;
  uniform unsigned int farChildID;
};

inline float get_coord(uniform vec3f const& p, uniform unsigned int const dim) {
if (dim == 0) {
return p.x;
}
if (dim == 1) {
return p.y;
}
if (dim == 2) {
return p.z;
}
}

inline void pkd_traverse_packet(uniform PKDGeometry *uniform self,
                                varying Ray &ray,
                                const varying float rdir[3],
                                const varying float org[3],
                                const varying float t_in_0,
                                const varying float t_out_0,
                                const uniform size_t dir_sign[3],
                                const uniform bool isShadowRay)
{
  // ++rayID;
  // print("Ray %\n",rayID);
  // uniform bool dbg = false; //(rayID == 338);

  print("TRAVERSE2\n");

  varying ThreePhaseStackEntry stack[64];
  varying ThreePhaseStackEntry *uniform stackPtr = stack;

  uniform unsigned int nodeID = 0;
  uniform unsigned int dim    = 0;

  float t_in                                  = t_in_0;
  float t_out                                 = t_out_0;
  const uniform float radius                  = self->global_radius;
  const uniform float sqr_radius              = radius*radius;
  const uniform unsigned int numParticles     = self->num_particles;
  //const uniform vec3f *uniform const position = self->position;
  // if (dbg) print("ENTER TRAVERSAL\n");
  uniform vec3f p;
  while (1) {
    // ------------------------------------------------------------------
    // do traversal step(s) as long as possible
    // ------------------------------------------------------------------
    // if (dbg) print("TRAV %\n",nodeID);
    while (1) {
      // if (dbg) print("DOWN %\n",nodeID);

      if (t_in > t_out)
        break;

      p = get_vec3f(self->position, nodeID);

      {
        // this is a leaf node - can't to to a leaf, anyway. Intersect
        // the prim, and be done with it.
        // if (dbg) print("LEAFISEC0\n");
        PKDGeometry_intersectPrim(p, nodeID, sqr_radius, ray);
        // if (dbg) print("LEAFISEC1\n");
        if (isShadowRay && ray.primID >= 0)
          return;
        //break;
      }

      // #if !DIM_FROM_DEPTH
      // INT3 *uniform intPtr = (INT3 *uniform)self->particle;
      // dim = intPtr[nodeID].x & 3;
      //dim = p.dim;
      dim = (*((unsigned int *uniform)&p.x)) & 3;
      // #endif
      print("DIM %\n", dim);

      const uniform size_t sign = dir_sign[dim];

      // ------------------------------------------------------------------
      // traversal step: compute distance, then compute intervals for front and
      // back side
      // ------------------------------------------------------------------
      const float org_to_node_dim = get_coord(p, dim) - org[dim];

      const float t_plane_0 = (org_to_node_dim - radius) * rdir[dim];
      const float t_plane_1 = (org_to_node_dim + radius) * rdir[dim];

      const float t_plane_nr = min(t_plane_0, t_plane_1);
      const float t_plane_fr = max(t_plane_0, t_plane_1);

      const float t_farChild_in   = max(t_in, t_plane_nr);
      const float t_farChild_out  = t_out;
      const float t_nearChild_out = min(t_out, t_plane_fr);

      // catch the case where all ray segments are on far side
      if (none(t_in < t_nearChild_out)) {
        if (none(t_farChild_in < t_farChild_out)) {
          break;
        } else {
          t_in   = t_farChild_in;
          t_out  = t_farChild_out;
          nodeID = 2 * nodeID + 2 - sign;
          continue;
        }
      }

      unmasked
      {
        stackPtr->t_in         = 1e20f;
        stackPtr->t_out        = -1e20f;
        stackPtr->t_sphere_out = -1e20f;
      }
      stackPtr->farChildID = 2 * nodeID + 2 - sign;

      stackPtr->t_in         = t_farChild_in;
      stackPtr->t_out        = t_farChild_out;
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
      if (stackPtr == stack) {
        return;
      }
      unmasked
      {
        t_in  = stackPtr[-1].t_in;
        t_out = min(stackPtr[-1].t_out, ray.t);
      }
      --stackPtr;

      // check if the node is still active (all the traversal since it
      // originally got pushed may have shortened the ray)
      if (none(t_in < t_out))
        continue;

      // intersect the actual node...
      if (t_in < min(stackPtr->t_sphere_out, ray.t)) {
        uniform vec3f p;
        p = get_vec3f(self->position, stackPtr->sphereID);
        PKDGeometry_intersectPrim(p, stackPtr->sphereID, sqr_radius, ray);
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

      break;
    }
  }
  // if (dbg) print("DONE TRAVERSAL\n");
}

inline void pkd_traverse_packet(uniform PKDGeometry *uniform self,
                                varying Ray &ray,
                                uniform bool isShadowRay)
{
  float t_in = ray.t0, t_out = ray.t;
  intersectBox(ray, self->bounds, t_in, t_out);
  print("TRAVERSE1\n");
  if (t_out < t_in)
    return;

  const varying float rdir[3] = {
      safe_rcp(ray.dir.x), safe_rcp(ray.dir.y), safe_rcp(ray.dir.z)};
  const varying float org[3] = {ray.org.x, ray.org.y, ray.org.z};

  uniform size_t dir_sign[3];
  if (ray.dir.z > 0.f) {
    dir_sign[2] = 0;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay);
      }
    }
  } else {
    dir_sign[2] = 1;
    if (ray.dir.y > 0.f) {
      dir_sign[1] = 0;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay);
      }
    } else {
      dir_sign[1] = 1;
      if (ray.dir.x > 0.f) {
        dir_sign[0] = 0;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay);
      } else {
        dir_sign[0] = 1;
        pkd_traverse_packet(
            self, ray, rdir, org, t_in, t_out, dir_sign, isShadowRay);
      }
    }
  }
}



void PKDGeometry_postIntersect(const Geometry *uniform geometry,
    varying DifferentialGeometry &dg,
    const varying Ray &ray,
    uniform int64 flags)
{
    const PKDGeometry *uniform self =
        (const PKDGeometry *uniform)geometry;
    dg.Ng = dg.Ns = normalize(ray.Ng);

    float radius = self->global_radius;

    dg.epsilon = radius * ulpEpsilon;

    if (flags & DG_COLOR) {
        vec4f color = make_vec4f(self->global_color.x/255.f, self->global_color.y/255.f, self->global_color.z/255.f, self->global_color.w/255.f);
        /*if (and(valid(self->color), !self->has_global_color)) {
            vec4uc tmp_color = get_vec4uc(self->color, ray.primID);
            color = make_vec4f(tmp_color.x/255.f, tmp_color.y/255.f, tmp_color.z/255.f, tmp_color.w/255.f);
        }*/
    }
}

void PKDGeometry_intersect_kernel(
    const RTCIntersectFunctionNArguments *uniform args,
    const uniform bool isOcclusionTest)
{
if (!args->valid[programIndex])
    return;

    PKDGeometry *uniform self = (PKDGeometry * uniform) args->geometryUserPtr;

  // this assumes that the args->rayhit is actually a pointer to a varying ray!
  varying Ray *uniform ray = (varying Ray * uniform) args->rayhit;

  pkd_traverse_packet(self, *ray, isOcclusionTest);
}

export void PKDGeometry_bounds(
    const RTCBoundsFunctionArguments *uniform args)
{
    uniform PKDGeometry *uniform geom = (uniform PKDGeometry *uniform)args->geometryUserPtr;
    box3fa *uniform out = (box3fa *uniform)args->bounds_o;
    *out = make_box3fa(geom->bounds.lower, geom->bounds.upper);

    print("BOUNDS\n");
}

export void PKDGeometry_intersect(
    const struct RTCIntersectFunctionNArguments *uniform args)
{
    PKDGeometry_intersect_kernel(args, false);
}

export void PKDGeometry_occluded(
    const struct RTCIntersectFunctionNArguments *uniform args)
{
    PKDGeometry_intersect_kernel(args, true);
}

export void *uniform PKDGeometry_postIntersect_addr()
{
    return PKDGeometry_postIntersect;
}