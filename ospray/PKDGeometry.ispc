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
#include "geometry/Geometry.ih"
#include "rkcommon/math/box.ih"
#include "rkcommon/math/vec.ih"
// this module
#include "PKDGeometry.ih"
#include "PKDGeometryShared.h"

static void PKDGeometry_postIntersect(uniform Geometry *uniform geometry,
    varying DifferentialGeometry &dg,
    const varying Ray &ray,
    uniform int64 flags)
{
  uniform PKDGeometry *uniform THIS = (uniform PKDGeometry * uniform) geometry;

  dg.Ng = dg.Ns = ray.Ng;

  if ((flags & DG_COLOR)) {
    dg.color = make_vec4f(1.f, 0.f, 0.f, 1.f);
    if (THIS->has_global_color) {
      dg.color = make_vec4f(THIS->global_color.x / 255.f,
          THIS->global_color.y / 255.f,
          THIS->global_color.z / 255.f,
          THIS->global_color.w / 255.f);
    } else {
      varying vec4uc const c = THIS->color[ray.primID];
      dg.color = make_vec4f(c.x / 255.f, c.y / 255.f, c.z / 255.f, c.w / 255.f);
    }
  }
}

export void PKDGeometry_bounds(const RTCBoundsFunctionArguments *uniform args)
{
  uniform PKDGeometry *uniform geom =
      (uniform PKDGeometry * uniform) args->geometryUserPtr;
  box3fa *uniform out = (box3fa * uniform) args->bounds_o;
  *out = make_box3fa(geom->bounds.lower, geom->bounds.upper);
}

export void PKDGeometry_intersect(
    const struct RTCIntersectFunctionNArguments *uniform args)
{
  // PKDGeometry_intersect_kernel(args, false);
  PKDGeometry_intersect_packet(args);
}

export void PKDGeometry_occluded(
    const struct RTCIntersectFunctionNArguments *uniform args)
{
  // PKDGeometry_intersect_kernel(args, true);
  PKDGeometry_occluded_packet(args);
}

export void *uniform PKDGeometry_postIntersect_addr()
{
  return PKDGeometry_postIntersect;
}
