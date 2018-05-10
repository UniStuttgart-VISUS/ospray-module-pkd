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

#include "PKDGeometry.h"
#include "PKDConfig.h"
// ospray
#include "ospray/common/Model.h"
// ispc exports
#include "PKDGeometry_ispc.h"
#include "dllexport.h"

namespace ospray {
  using std::endl;
  using std::cout;

  //! Constructor
  PartiKDGeometry::PartiKDGeometry()
    : particleRadius(.02f)
  {
    PING;
    ispcEquivalent = ispc::PartiKDGeometry_create(this);
  }

    vec3f decodeParticle(size_t i) {
      size_t mask = (1<<20)-1;
      size_t ix = (i>> 2)&mask;
      size_t iy = (i>>22)&mask;
      size_t iz = (i>>42)&mask;
      return vec3f(ix,iy,iz);
    }

  
  vec4f PartiKDGeometry::getParticle(size_t i) const 
  {
    switch(format) {
    case OSP_FLOAT4: return particle4f[i];
    case OSP_FLOAT3: return vec4f(particle3f[i], 1.0f);
    case OSP_ULONG: return vec4f(decodeParticle(particle1ul[i]), 1.0f);
    default: NOTIMPLEMENTED;
    };
  }


  /*! return bounding box of particle centers */
  box3f PartiKDGeometry::getBounds() const
  {
    box3f b = empty;
    for (size_t i=0;i<numParticles;i++) {
      auto pt = getParticle(i);
      b.extend(vec3f(pt.x, pt.y, pt.z));
    }
    return b;
  }

  uint32 getAttributeBits(float val, float lo, float hi)
  {
    // cout << " attrbits: " << val << " (" << lo << "," << hi << ")" << endl;
    if (hi == lo) return 1;
    int bit = std::min((int)31,int(32*((val-lo)/float(hi-lo))));
    return 1<<bit;
  }

  /*! gets called whenever any of this node's dependencies got changed */
  void PartiKDGeometry::dependencyGotChanged(ManagedObject *object)
  {
    ispc::PartiKDGeometry_updateTransferFunction(this->getIE(),transferFunction->getIE());
  }


  /*! \brief integrates this geometry's primitives into the respective
    model's acceleration structure */
  void PartiKDGeometry::finalize(Model *model) 
  {
    // -------------------------------------------------------
    // parse parameters, using hard assertions (exceptions) for now.
    //
    // note:
    // - "float radius" *MUST* be defined with the object
    // - "data<vec3f> particles' *MUST* be defined for the object
    // -------------------------------------------------------
    particleData = getParamData("position");
    if (!particleData)
      throw std::runtime_error("#osp:pkd: no 'position' data found with object");

    particle     = particleData->data;
    numParticles = particleData->numItems;
    format = particleData->type;
    bool isVec4 = format == OSP_FLOAT4;
    bool isQuantized = format == OSP_ULONG;
    PRINT(isQuantized);
    //const box3f _centerBounds = getBounds(); //< TODO unnecessary work

    Ref<Data> bboxData = getParamData("bbox");
    float const* bbox = reinterpret_cast<float const*>(bboxData->data);
    vec3f const lower(bbox[0], bbox[1], bbox[2]);
    vec3f const upper(bbox[3], bbox[4], bbox[5]);
    box3f const centerBounds(lower, upper);

    /*PRINT(_centerBounds);
    PRINT(centerBounds);*/

    attributeData = getParamData("attribute",NULL);
    transferFunction = (TransferFunction*)getParamObject("transferFunction",NULL);
    if (transferFunction) {
      transferFunction->registerListener(this);
    }

    bool useSPMD = getParam1i("useSPMD",0);

    int colorType = getParam1i("colorType", 0);

    particleRadius = getParamf("radius",0.f);
    if (particleRadius <= 0.f)
      throw std::runtime_error("#osp:pkd: invalid radius (<= 0.f)");
    const float expectedRadius
      = (centerBounds.size().x+centerBounds.size().y+centerBounds.size().z)*powf(numParticles,1.f/3.f);
    if (particleRadius > 10.f*expectedRadius)
      cout << "#osp:pkd: Warning - particle radius is pretty big for given particle configuration !?" << endl;
    
    const box3f sphereBounds(centerBounds.lower - vec3f(particleRadius),
                             centerBounds.upper + vec3f(particleRadius));
    size_t numInnerNodes = numParticles/2;


    // compute attribute mask and attrib lo/hi values
    float attr_lo = 0.f, attr_hi = 0.f;
    uint32 *binBitsArray = NULL;
    attribute = (float*)(attributeData?attributeData->data:NULL);

    // Attribute culling on the lidar type-punned RGB data doesn't make sense, so don't do it
#if !PARTIKD_LIDAR_ENABLED
    if (attribute) {
      cout << "#osp:pkd: found attribute, computing range and min/max bit array" << endl;
      attr_lo = attr_hi = attribute[0];
      for (size_t i=0;i<numParticles;i++)
        { attr_lo = std::min(attr_lo,attribute[i]); attr_hi = std::max(attr_hi,attribute[i]); }

      binBitsArray = new uint32[numInnerNodes];
      size_t numBytesRangeTree = numInnerNodes * sizeof(uint32);
      cout << "#osp:pkd: num bytes in range tree " << numBytesRangeTree << endl;
      for (long long pID=numInnerNodes-1;pID>=0;--pID) {
        size_t lID = 2*pID+1;
        size_t rID = lID+1;
        uint32 lBits = 0, rBits = 0;
        if (rID < numInnerNodes)
          rBits = binBitsArray[rID];
        else if (rID < numParticles)
          rBits = getAttributeBits(attribute[rID],attr_lo,attr_hi);
        if (lID < numInnerNodes)
          lBits = binBitsArray[lID];
        else if (lID < numParticles)
          lBits = getAttributeBits(attribute[lID],attr_lo,attr_hi);
        binBitsArray[pID] = lBits|rBits;
        // cout << " bits " << pID << " : " << (int*)lBits << " " << (int*)rBits << endl;
      }
      cout << "#osp:pkd: found attribute [" << attr_lo << ".." << attr_hi << "], root bits " << (int*)(int64)binBitsArray[0] << endl;
    }
#endif



    cout << "#osp:pkd: ColorType: " << colorType << endl;
    cout << "#osp:pkd: isVec4: " << isVec4 << endl;

    // -------------------------------------------------------
    // actually create the ISPC-side geometry now
    // -------------------------------------------------------
    ispc::PartiKDGeometry_set(getIE(),model->getIE(),isQuantized,useSPMD,isVec4,colorType,
                              transferFunction?transferFunction->getIE():NULL,
                              particleRadius,
                              numParticles,
                              numInnerNodes,
                              (ispc::PKDParticle*)particle,
                              attribute,binBitsArray,
                              (ispc::box3f&)centerBounds,(ispc::box3f&)sphereBounds,
                              attr_lo,attr_hi);
  }    

  OSP_REGISTER_GEOMETRY(PartiKDGeometry,pkd_geometry);

} // ::ospray

extern "C" OSPRAY_PKD_EXPORT void ospray_init_module_pkd()
{
  std::cout << "#osp:pkd: loading 'pkd' module" << std::endl;
}
