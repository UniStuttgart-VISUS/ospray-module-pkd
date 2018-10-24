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

// ospray
#include "common/Model.ih"
#include "common/Ray.ih"
#include "geometry/Geometry.ih"
#include "math/box.ih"
#include "math/vec.ih"
#include "transferFunction/LinearTransferFunction.ih"
// this module
#include "PKDConfig.h"
#include "PKDGeometry.ih"
// embree
#include "geometry/Geometry.ih"
#if PKD_LIDAR_ENABLED
#include "ColorMask.h"
#endif

vec4f decodeColor(float colorCode, uniform int colorType, uniform PartiKDGeometry *uniform THIS) {
  unsigned int *color_code = (unsigned int *)(&colorCode);
  if (colorType == 1 || colorType == 3) {
    float r = (color_code[0] & 0xff) / 255.0f;
    float g = ((color_code[0] >> 8) & 0xff) / 255.0f;
    float b = ((color_code[0] >> 16) & 0xff) / 255.0f;
    return make_vec4f(r, g, b, 1.0f);
  } else if (colorType == 2 || colorType == 4) {
    float r = (color_code[0] >> 24) / 255.0f;
    float g = ((color_code[0] >> 16) & 0xff) / 255.0f;
    float b = ((color_code[0] >> 8) & 0xff) / 255.0f;
    float a = (color_code[0] & 0xff) / 255.0f;
    return make_vec4f(r, g, b, a);
  } else if (colorType == 5) {
    return make_vec4f(THIS->transferFunction->getColorForValue(THIS->transferFunction, colorCode), 1.0f);
  }
  return make_vec4f(0.0f, 0.0f, 0.0f, 0.0f);
}

vec4f decodeColorInt(unsigned int colorCode, uniform int colorType, uniform PartiKDGeometry *uniform THIS) {
  if (colorType == 1 || colorType == 3) {
    float r = ((float)(colorCode >> 24)) / 255.0f;
    float g = ((float)((colorCode >> 16) & 0xff)) / 255.0f;
    float b = ((float)((colorCode >> 8) & 0xff)) / 255.0f;
    return make_vec4f(r, g, b, 1.0f);
  } else if (colorType == 2 || colorType == 4) {
    float r = ((float)(colorCode >> 24)) / 255.0f;
    float g = ((float)((colorCode >> 16) & 0xff)) / 255.0f;
    float b = ((float)((colorCode >> 8) & 0xff)) / 255.0f;
    float a = ((float)(colorCode & 0xff)) / 255.0f;
    return make_vec4f(r, g, b, a);
  } else if (colorType == 5) {
    float color = ((float)colorCode) / 255.0f;
    return make_vec4f(THIS->transferFunction->getColorForValue(THIS->transferFunction, colorCode), 1.0f);
  }
  return make_vec4f(0.0f, 0.0f, 0.0f, 0.0f);
}

static void PartiKDGeometry_postIntersect(uniform Geometry *uniform geometry,
                                          uniform Model *uniform model,
                                          varying DifferentialGeometry &dg,
                                          const varying Ray &ray,
                                          uniform int64 flags)
{
  uniform PartiKDGeometry *uniform THIS =
      (uniform PartiKDGeometry * uniform) geometry;

  dg.Ng = dg.Ns = ray.Ng;

#if PKD_LIDAR_ENABLED
  if ((flags & DG_COLOR) && (THIS->attribute != NULL)){
    uniform int *uniform attribArray = (uniform int *uniform)THIS->attribute;
    const int attrib = attribArray[ray.primID];
    dg.color = make_vec4f(GET_RED(attrib) / 255.0, GET_GREEN(attrib) / 255.0,
        GET_BLUE(attrib) / 255.0, 1.0);
  }
#else
  if ((flags & DG_COLOR) && THIS->attribute != NULL &&
      THIS->transferFunction != NULL && !THIS->isVec4) {
#if 1
    uniform float *uniform attribArray = THIS->attribute;
    const uniform float attrib_lo      = THIS->attr_lo;
    const uniform float attrib_hi      = THIS->attr_hi;

    float attrib_org = attribArray[ray.primID];
      
    const float attrib
      = (attrib_org - attrib_lo)
      * rcp(attrib_hi - attrib_lo + 1e-10f);
    // if (attrib >= 1.f || attrib <= 0.f) 
    //   print("ATTRIB OUT OF RANGE:\n org %\n remapped %\n ID %:%\n range % %\n",
    //         attrib_org,attrib,primID64,ray.primID,attrib_lo,attrib_hi);
    // compute alpha value from attribute value
    const float alpha = THIS->transferFunction->getOpacityForValue(THIS->transferFunction,
                                                                   attrib);
    const vec3f color = THIS->transferFunction->getColorForValue(THIS->transferFunction,
                                                                 attrib);
    dg.color = make_vec4f(color.x,color.y,color.z,1.0);
#else
    foreach_unique(upper64 in ray.primID_hi64)
    {
      if (upper64)
        print("UPPER: %\n", upper64);
      uniform uint64 offs = upper64;
      offs                = offs << 32;
      uniform float *uniform attribArray =
          THIS->attribute + offs;  //; // + (((unsigned int64)upper64)<<32);
      // normalize attribute to the [0,1] range (by normalizing relative
      // to the attribute range stored in the min max BVH's root node
      const uniform float attrib_lo = THIS->attr_lo;
      const uniform float attrib_hi = THIS->attr_hi;
      attrib_array += (unsigned int32)ray.primID;
      const float attrib_org = attribArray[0];  //(unsigned int32)ray.primID];
      const float attrib =
          (attrib_org - attrib_lo) * rcp(attrib_hi - attrib_lo + 1e-10f);
      if (attrib >= 1.f || attrib <= 0.f)
        print(
            "ATTRIB OUT OF RANGE:\n org %\n remapped %\n ID %:%\n range % %\n",
            attrib_org,
            attrib,
            offs,
            ray.primID,
            attrib_lo,
            attrib_hi);
      // compute alpha value from attribute value
      const float alpha = THIS->transferFunction->getOpacityForValue(
          THIS->transferFunction, attrib);
      const vec3f color = THIS->transferFunction->getColorForValue(
          THIS->transferFunction, attrib);
      dg.color = make_vec4f(color.x, color.y, color.z, alpha);
    }
#endif
  }
  
#endif
  if ((flags & DG_COLOR) && THIS->isVec4) {
      if (THIS->colorType != 0) {
          uniform PKDParticle4 *uniform particle_array = (PKDParticle4 *)THIS->particle;
          PKDParticle4 particle_org;
          /* uint64 primID64 = (uint32)ray.primID_hi64;
          primID64 <<= 32;
          primID64 += (uint32)ray.primID; */
          foreach_unique(pID in ray.primID) {
              uniform PKDParticle4 *uniform ptr = particle_array + pID;
              particle_org = ptr[0];
          }
          dg.color = decodeColorInt(particle_org.color, THIS->colorType, THIS);
          //dg.color = make_vec4f(0.0f, 0.0f, 1.0f, 1.0f);
      } else {
          dg.color = make_vec4f(1.0f, 0.0f, 0.0f, 1.0f);
      }
  }

  //dg.color = make_vec4f(1.0f, 1.0f, 0.0f, 1.0f);
}

unmasked void PartiKDGeometry_bounds(const RTCBoundsFunctionArguments *uniform args)
{
  uniform PartiKDGeometry *uniform geom = (uniform PartiKDGeometry *uniform)args->geometryUserPtr;
  box3fa *uniform out = (box3fa *uniform)args->bounds_o;
  *out = make_box3fa(geom->sphereBounds.lower, geom->sphereBounds.upper);
}

/*! creates a new pkd geometry */
export void *uniform PartiKDGeometry_create(void *uniform cppEquivalent)
{
  uniform PartiKDGeometry *uniform geom = uniform new uniform PartiKDGeometry;
  Geometry_Constructor(&geom->geometry,
                       cppEquivalent,
                       PartiKDGeometry_postIntersect,
                       NULL,
                       0,
                       NULL);
  return geom;
}

/*! helper function that recomputes the bin-bits from the transfer
    function for parameter range culling. note this function does
    _NOT_ change anything in the tree itself, it only re-bins the
    transfer function*/
export void PartiKDGeometry_updateTransferFunction(
    void *uniform _THIS, void *uniform _transferFunction)
{
  PartiKDGeometry *uniform THIS = (PartiKDGeometry * uniform) _THIS;
  TransferFunction *uniform transferFunction =
      (TransferFunction * uniform) _transferFunction;
  THIS->transferFunction_activeBinBits = 0;
  for (uniform int i = 0; i < 32; i++) {
    uniform float a0         = i / 32.f;
    uniform float a1         = (i + 1) / 32.f - 1e-5f;
    vec2f range              = make_vec2f(a0, a1);
    uniform float alphaRange = extract(
        transferFunction->getMaxOpacityInRange(transferFunction, range), 0);
    if (alphaRange >= .5f)
      THIS->transferFunction_activeBinBits |= (1UL << i);
  }
}

/*! 'constructor' for a newly created pkd geometry */
export void PartiKDGeometry_set(void *uniform _geom,
                                void *uniform _model,
                                uniform bool isQuantized,
                                uniform bool useSPMD,
                                uniform bool isVec4,
                                uniform int colorType,
                                void *uniform transferFunction,
                                float uniform particleRadius,
                                uniform uint64 numParticles,
                                uniform uint64 numInnerNodes,
                                PKDParticle *uniform particle,
                                float *uniform attribute,
                                uint32 *uniform innerNode_attributeMask,
                                uniform box3f &centerBounds,
                                uniform box3f &sphereBounds,
                                uniform float attr_lo,
                                uniform float attr_hi)
{
  uniform PartiKDGeometry *uniform geom =
      (uniform PartiKDGeometry * uniform) _geom;
  uniform Model *uniform model = (uniform Model * uniform) _model;

  RTCGeometry embreeGeom = rtcNewGeometry(ispc_embreeDevice(), RTC_GEOMETRY_TYPE_USER);
  uniform uint32 geomID = rtcAttachGeometry(model->embreeSceneHandle, embreeGeom);
  
  geom->geometry.model  = model;
  geom->isQuantized     = isQuantized;
  geom->isVec4          = isVec4;
  geom->colorType       = colorType;
  geom->geometry.geomID = geomID;
  geom->particleRadius  = particleRadius;
  geom->particle        = particle;
  geom->numParticles    = numParticles;
  geom->numInnerNodes   = numInnerNodes;
  geom->centerBounds    = centerBounds;
  geom->sphereBounds    = sphereBounds;
  geom->attribute       = attribute;
  geom->attr_lo         = attr_lo;
  geom->attr_hi         = attr_hi;
  geom->innerNode_attributeMask = innerNode_attributeMask;
  geom->epsilon = geom->particleRadius / 100.0;

  geom->transferFunction = (TransferFunction *uniform)transferFunction;
  if (transferFunction)  {
    PartiKDGeometry_updateTransferFunction(geom, transferFunction);
  }

  rtcSetGeometryUserData(embreeGeom, geom);
  rtcSetGeometryUserPrimitiveCount(embreeGeom, 1);
  rtcSetGeometryBoundsFunction(embreeGeom,
      (uniform RTCBoundsFunction)&PartiKDGeometry_bounds, geom);

  if (useSPMD) {
    print("creating PKD with ***SPMD*** traversal\n");
    rtcSetGeometryIntersectFunction(embreeGeom,
        (uniform RTCIntersectFunctionN)&PartiKDGeometry_intersect_spmd);
    rtcSetGeometryOccludedFunction(embreeGeom,
        (uniform RTCOccludedFunctionN)&PartiKDGeometry_occluded_spmd);
  } else {
    rtcSetGeometryIntersectFunction(embreeGeom,
        (uniform RTCIntersectFunctionN)&PartiKDGeometry_intersect_packet);
    rtcSetGeometryOccludedFunction(embreeGeom,
        (uniform RTCOccludedFunctionN)&PartiKDGeometry_occluded_packet);
  }
  rtcCommitGeometry(embreeGeom);
  rtcReleaseGeometry(embreeGeom);
}

