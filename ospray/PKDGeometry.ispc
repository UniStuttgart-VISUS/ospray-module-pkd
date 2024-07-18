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
#include "PKDGeometryShared.h"
#include "PKDGeometry.ih"


static void PKDGeometry_postIntersect(uniform Geometry *uniform geometry,
    varying DifferentialGeometry &dg,
    const varying Ray &ray,
    uniform int64 flags)
{
  uniform PKDGeometry *uniform THIS =
      (uniform PKDGeometry * uniform) geometry;

  dg.Ng = dg.Ns = ray.Ng;

  if ((flags & DG_COLOR)) {
    dg.color = make_vec4f(1.f, 0.f, 0.f, 1.f);
  }
}

export void PKDGeometry_bounds(
    const RTCBoundsFunctionArguments *uniform args)
{
  uniform PKDGeometry *uniform geom =
      (uniform PKDGeometry * uniform) args->geometryUserPtr;
  box3fa *uniform out = (box3fa * uniform) args->bounds_o;
  *out = make_box3fa(geom->bounds.lower, geom->bounds.upper);
}

/*! creates a new pkd geometry */
//export void *uniform PartiKDGeometry_create(void *uniform cppEquivalent)
//{
//  uniform PartiKDGeometry *uniform geom = uniform new uniform PartiKDGeometry;
//  Geometry_Constructor(&geom->geometry,
//      cppEquivalent,
//      PartiKDGeometry_postIntersect,
//      NULL,
//      0,
//      NULL);
//  return geom;
//}

/*! helper function that recomputes the bin-bits from the transfer
    function for parameter range culling. note this function does
    _NOT_ change anything in the tree itself, it only re-bins the
    transfer function*/
//export void PartiKDGeometry_updateTransferFunction(
//    void *uniform _THIS, void *uniform _transferFunction)
//{
//  PartiKDGeometry *uniform THIS = (PartiKDGeometry * uniform) _THIS;
//  TransferFunction *uniform transferFunction =
//      (TransferFunction * uniform) _transferFunction;
//  THIS->transferFunction_activeBinBits = 0;
//  for (uniform int i = 0; i < 32; i++) {
//    uniform float a0 = i / 32.f;
//    uniform float a1 = (i + 1) / 32.f - 1e-5f;
//    vec2f range = make_vec2f(a0, a1);
//    uniform float alphaRange = extract(
//        transferFunction->getMaxOpacityInRange(transferFunction, range), 0);
//    if (alphaRange >= .5f)
//      THIS->transferFunction_activeBinBits |= (1UL << i);
//  }
//}

/*! 'constructor' for a newly created pkd geometry */
//export void PartiKDGeometry_set(void *uniform _geom,
//    void *uniform _model,
//    uniform bool isQuantized,
//    uniform bool useSPMD,
//    uniform bool isVec4,
//    uniform int colorType,
//    void *uniform transferFunction,
//    float uniform particleRadius,
//    uniform uint64 numParticles,
//    uniform uint64 numInnerNodes,
//    PKDParticle *uniform particle,
//    float *uniform attribute,
//    uint32 *uniform innerNode_attributeMask,
//    uniform box3f &centerBounds,
//    uniform box3f &sphereBounds,
//    uniform float attr_lo,
//    uniform float attr_hi)
//{
//  uniform PartiKDGeometry *uniform geom =
//      (uniform PartiKDGeometry * uniform) _geom;
//  uniform Model *uniform model = (uniform Model * uniform) _model;
//
//  RTCGeometry embreeGeom =
//      rtcNewGeometry(ispc_embreeDevice(), RTC_GEOMETRY_TYPE_USER);
//  uniform uint32 geomID =
//      rtcAttachGeometry(model->embreeSceneHandle, embreeGeom);
//
//  geom->geometry.model = model;
//  geom->isQuantized = isQuantized;
//  geom->isVec4 = isVec4;
//  geom->colorType = colorType;
//  geom->geometry.geomID = geomID;
//  geom->particleRadius = particleRadius;
//  geom->particle = particle;
//  geom->numParticles = numParticles;
//  geom->numInnerNodes = numInnerNodes;
//  geom->centerBounds = centerBounds;
//  geom->sphereBounds = sphereBounds;
//  geom->attribute = attribute;
//  geom->attr_lo = attr_lo;
//  geom->attr_hi = attr_hi;
//  geom->innerNode_attributeMask = innerNode_attributeMask;
//  geom->epsilon = geom->particleRadius / 100.0;
//
//  geom->transferFunction = (TransferFunction * uniform) transferFunction;
//  if (transferFunction) {
//    PartiKDGeometry_updateTransferFunction(geom, transferFunction);
//  }
//
//  rtcSetGeometryUserData(embreeGeom, geom);
//  rtcSetGeometryUserPrimitiveCount(embreeGeom, 1);
//  rtcSetGeometryBoundsFunction(
//      embreeGeom, (uniform RTCBoundsFunction)&PartiKDGeometry_bounds, geom);
//
//  if (useSPMD) {
//    print("creating PKD with ***SPMD*** traversal\n");
//    rtcSetGeometryIntersectFunction(embreeGeom,
//        (uniform RTCIntersectFunctionN)&PartiKDGeometry_intersect_spmd);
//    rtcSetGeometryOccludedFunction(embreeGeom,
//        (uniform RTCOccludedFunctionN)&PartiKDGeometry_occluded_spmd);
//  } else {
//    rtcSetGeometryIntersectFunction(embreeGeom,
//        (uniform RTCIntersectFunctionN)&PartiKDGeometry_intersect_packet);
//    rtcSetGeometryOccludedFunction(embreeGeom,
//        (uniform RTCOccludedFunctionN)&PartiKDGeometry_occluded_packet);
//  }
//  rtcCommitGeometry(embreeGeom);
//  rtcReleaseGeometry(embreeGeom);
//}




export void PKDGeometry_intersect(
    const struct RTCIntersectFunctionNArguments *uniform args)
{
  //PKDGeometry_intersect_kernel(args, false);
  PKDGeometry_intersect_packet(args);
}

export void PKDGeometry_occluded(
    const struct RTCIntersectFunctionNArguments *uniform args)
{
  //PKDGeometry_intersect_kernel(args, true);
  PKDGeometry_occluded_packet(args);
}

export void *uniform PKDGeometry_postIntersect_addr()
{
  return PKDGeometry_postIntersect;
}

