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

#pragma once

#include "rkcommon/math/box.h"
#include "rkcommon/math/vec.h"

#include "geometry/Geometry.h"
//#include "ospray/common/Data.h"
//#include "ospray/transferFunction/TransferFunction.h"

#include "PKDGeometryShared.h"

namespace ospray {
namespace pkd {

using namespace rkcommon;

/*! the actual ospray geometry for a PartiKD */
struct PKDGeometry : public AddStructShared<Geometry, ispc::PKDGeometry>
{
  //! Constructor
  PKDGeometry();
  virtual ~PKDGeometry() = default;

  std::string toString() const override {
    return "ospray::pkd::PKDGeometry";
  }

  void commit() override;

  size_t numPrimitives() const override
  {
    return positionData ? 1 : 0;
  }

  /*! return bounding box of particle centers */
  /*box3f getBounds() const;
  vec4f getParticle(size_t i) const;*/

 protected:
  Ref<DataT<vec3f> const> positionData;
  Ref<DataT<vec4uc> const> colorData;
    
  unsigned int num_particles;

  float global_radius;
  Ref<DataT<unsigned char> const> globalColorData;
  vec4uc global_color;

  bool has_global_color;

  Ref<DataT<float> const> boundsData;
  box3f bounds;
};

} // namespace pkd
} // ::ospray
