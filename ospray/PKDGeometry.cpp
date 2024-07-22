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

#include "PKDGeometry.h"
// ospray
//#include "ospray/common/Model.h"
//#include "ospray/common/OSPCommon.h"
// ispc exports
#include "PKDGeometry_ispc.h"
//#include "dllexport.h"

namespace ospray {
namespace pkd {

//! Constructor
PKDGeometry::PKDGeometry()
{
  getSh()->super.postIntersect = ispc::PKDGeometry_postIntersect_addr();
}

void PKDGeometry::commit()
{
  global_radius = getParam<float>("global_radius", 0.5f);

  has_global_color = getParam<bool>("has_global_color", true);
  globalColorData = getParamDataT<unsigned char>("global_color");
  global_color = vec4uc(255, 0, 0, 255);
  if (globalColorData) {
    global_color.x = globalColorData->as<unsigned char>()[0];
    global_color.y = globalColorData->as<unsigned char>()[1];
    global_color.z = globalColorData->as<unsigned char>()[2];
    global_color.w = globalColorData->as<unsigned char>()[3];
  }

  positionData = getParamDataT<vec3f>("position");
  colorData = getParamDataT<vec4uc>("color");

  num_particles = getParam<unsigned int>("num_particles");

  boundsData = getParamDataT<float>("bounds");
  bounds.lower.x = boundsData->as<float>()[0];
  bounds.lower.y = boundsData->as<float>()[1];
  bounds.lower.z = boundsData->as<float>()[2];
  bounds.upper.x = boundsData->as<float>()[3];
  bounds.upper.y = boundsData->as<float>()[4];
  bounds.upper.z = boundsData->as<float>()[5];

  treeletsData = getParamDataT<ispc::PKDTreelet>("treelets");
  
  if (treeletsData) {
    createEmbreeUserGeometry(
        (RTCBoundsFunction)&ispc::PKDGeometry_bounds_treelets,
        (RTCIntersectFunctionN)&ispc::PKDGeometry_intersect_treelets,
        (RTCOccludedFunctionN)&ispc::PKDGeometry_occluded_treelets);
  } else {
    createEmbreeUserGeometry((RTCBoundsFunction)&ispc::PKDGeometry_bounds,
        (RTCIntersectFunctionN)&ispc::PKDGeometry_intersect,
        (RTCOccludedFunctionN)&ispc::PKDGeometry_occluded);
  }
  getSh()->position = positionData->data();
  getSh()->color = has_global_color ? nullptr : colorData->data();
  getSh()->global_radius = global_radius;
  getSh()->has_global_color = has_global_color;
  getSh()->global_color = global_color;
  getSh()->num_particles = num_particles;
  getSh()->num_innerNodes = num_particles / 2;
  getSh()->bounds = bounds;
  getSh()->treelets = treeletsData ? treeletsData->data() : nullptr;
  getSh()->super.numPrimitives = numPrimitives();

  postCreationInfo();
}

} // namespace pkd
} // namespace ospray
