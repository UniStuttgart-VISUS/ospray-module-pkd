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

#undef NDEBUG

#include "PKD.h"
#include "ospcommon/xml/XML.h"

#include "ospcommon/constants.h"

namespace ospray {
  namespace sg {

    using std::string;
    using std::cout;
    using std::endl;

    //! constructor
    PKDGeometry::PKDGeometry() 
      : Geometry("pkd_geometry")
    {
      PING;
    }

#if 0
    PKDGeometry::~PKDGeometry() 
    {
      for (int i=0;i<attribute.size();i++) delete attribute[i]; 
      if (ospGeometry) { ospRelease(ospGeometry); ospGeometry = NULL; }
    }

    //! return bounding box of this node (in local space)
    box3f PKDGeometry::getBounds() 
    {
      box3f bounds = ospcommon::EmptyTy();
      for (size_t i=0;i<numParticles;i++)
        bounds.extend(getParticle(i));
      bounds.lower -= vec3f(radius);
      bounds.upper += vec3f(radius);
      return bounds;
    }

    void PKDGeometry::render(RenderContext &ctx)
    {
      assert(!ospGeometry);

      // check if the data has changed at all
      if (lastModified <= lastCommitted) 
        // ... and return if not
        return;

      // if no geometry exists, create it.
      if (!ospGeometry) {
        // make sure the module is loaded
        // ospLoadModule("alpha_spheres");
        ospLoadModule("pkd");

        // and create the geometry
        if (useOldAlphaSpheresCode) {
          cout << "USING ALPHA-SPHERES" << endl;
          ospGeometry = ospNewGeometry("alpha_spheres");
        } else {
          ospGeometry = ospNewGeometry("pkd_geometry");
        }

        if (!ospGeometry)
          throw std::runtime_error("#osp:sg:PKDGeometry: could not create ospray 'pkd_geometry'");
        assert(ospGeometry);

        // assign a default material (for now.... eventually we might
        // want to do a 'real' material
        OSPMaterial mat = ospNewMaterial(ctx.integrator?ctx.integrator->getOSPHandle():NULL,"default");
        if (mat) {
          vec3f kd(.7f);
          vec3f ks(.3f);
          ospSet3fv(mat,"kd",&kd.x);
          ospSet3fv(mat,"ks",&ks.x);
          ospSet1f(mat,"Ns",99.f);
          ospCommit(mat);
        }


        ospSetMaterial(ospGeometry,mat);

        // and finally, add this geometry to the model
        ospAddGeometry(ctx.world->ospModel,ospGeometry);
      }
      
      // if true, we're forcing a immediate paging-in of all data
      // (otherwise the data is only mmap'ed and paged in on demand)
      bool forcePageIn = true;
        
      // create the particle array
      if (!ospPositionData) {
        if (numParticles == 0)
          throw std::runtime_error("osp:sg:PKDGeometry: no 'position' data defined in PKD geometry");

        size_t numPositionBytes;
        if (format == OSP_FLOAT3) {
          ospPositionData = ospNewData(numParticles,OSP_FLOAT3,particle3f,
                                       OSP_DATA_SHARED_BUFFER);
          numPositionBytes = sizeof(vec3f)*numParticles;
        } else {
          ospPositionData = ospNewData(numParticles,OSP_ULONG,particle3f,
                                       OSP_DATA_SHARED_BUFFER);
          numPositionBytes = sizeof(long long)*numParticles;
        }
        ospCommit(ospPositionData);
        ospSetData(ospGeometry,"position",ospPositionData);
        
        cout << "#osp:pkd: numbytes for raw p-k-d tree: " << numPositionBytes << endl;
        if (forcePageIn) {
          cout << "#osp:pkd: FORCED page-in of entire position array" << endl;
          madvise(particle3f,numPositionBytes,MADV_WILLNEED);
        }
      }
      
      // check if transfer function exists, and is updated
      if (!transferFunction) {
        ospSetObject(ospGeometry,"transferFunction",NULL);
    } else {
        transferFunction->render(ctx);
        if (transferFunction->getLastCommitted() >= this->lastCommitted) 
          ospSetObject(ospGeometry,"transferFunction",transferFunction->getOSPHandle());
      }

      // assign attribute, if available - we use the first attribute for now
      if (!attribute.empty()) {
        if (attribute[0]->ospData == NULL) {
          attribute[0]->ospData = ospNewData(numParticles,OSP_FLOAT,attribute[0]->value,
                                             OSP_DATA_SHARED_BUFFER);
          ospSetData(ospGeometry,"attribute",attribute[0]->ospData);
          size_t numAttributeBytes = sizeof(float)*numParticles;
          cout << "#osp:pkd: numbytes for particle attribute: " << numAttributeBytes << endl;
          if (forcePageIn) {
            cout << "#osp:pkd: FORCED page-in of entire attribute array" << endl;
            madvise(attribute[0]->value,numParticles*sizeof(float),MADV_WILLNEED);
          }
        }
      }

      // set the particle radius - right now we have a fixed radius for all particles
      if (radius == 0)
        std::cout << "#osp:sg:pkd: warning - radius is 0" << std::endl;
      else {
        assert(radius > 0.f);
        ospSet1f(ospGeometry,"radius",radius);
      }
      ospCommit(ospGeometry);
      lastCommitted = TimeStamp::now();
    }

    vec3f decodeParticle(size_t i) {
      size_t mask = (1<<20)-1;
      size_t ix = (i>> 2)&mask;
      size_t iy = (i>>22)&mask;
      size_t iz = (i>>42)&mask;
      return vec3f(ix,iy,iz);
    }

    vec3f PKDGeometry::getParticle(size_t i) const 
    {
      switch(format) {
      case OSP_FLOAT3: return particle3f[i];
      case OSP_ULONG: return decodeParticle(particle1ul[i]);
      default: NOTIMPLEMENTED;
      };
    }

    //! \brief Initialize this node's value from given corresponding XML node 
    void PKDGeometry::setFromXML(const xml::Node *const node, const unsigned char *binBasePtr)
    {
      std::string tfcnName = node->getProp("transferFunction");
      if (!tfcnName.empty()) {
        transferFunction = dynamic_cast<sg::TransferFunction*>(sg::findNamedNode(tfcnName));
      }
      for (size_t childID=0;childID<node->child.size();childID++) {
        xml::Node *child = node->child[childID];
        if (child->name == "position") {
          numParticles = child->getPropl("count");
          std::string format = child->getProp("format");
          if (format == "vec3f" || format == "float3") {
            particle3f = (vec3f*)(binBasePtr+child->getPropl("ofs"));
            this->format = OSP_FLOAT3;
          } else {
            std::cout << "#osp:sg:PKDGeometry: found " << numParticles
                      << " QUANTIZED particles." << endl;
            particle1ul = (uint64_t*)(binBasePtr+child->getPropl("ofs"));
            this->format = OSP_ULONG;
          }
          particleBounds = getBounds();
          std::cout << "#osp:sg:PKDGeometry: found " << numParticles
                    << " particles, bounds=" << particleBounds << std::endl;
          continue;
        } 

        if (child->name == "useOldAlphaSpheresCode") {
          useOldAlphaSpheresCode = child->getPropl("value");
          if (useOldAlphaSpheresCode) std::cout << "#osp:sg:PKDGeometry: SWITCHING TO OLD ALPHA-SPHERES CODE" << std::endl;
          continue;
        } 
      
        if (child->name == "radius") {
          radius = atof(child->content.c_str());
          std::cout << "#osp:sg:PKDGeometry: found radius " << radius << std::endl;
          continue;
        } 
      
        if (child->name == "attribute") {
          Attribute *attrib = new Attribute;
          attrib->name = child->getProp("name");
          size_t count = child->getPropl("count");
          attrib->value = (float *)(binBasePtr+child->getPropl("ofs"));
          attrib->minValue = attrib->maxValue = attrib->value[0];
          for (size_t i=0;i<count;i++) {
            attrib->minValue = std::min(attrib->minValue,attrib->value[i]);
            attrib->maxValue = std::max(attrib->maxValue,attrib->value[i]);
          }
          std::cout << "#osp:sg:PKDGeometry: found attribute '"+attrib->name+"' (" << attrib->minValue << ":" << attrib->maxValue << ")" << std::endl;
          attribute.push_back(attrib);
          continue;
        }

        std::cout << "#osp:sg:PKDGeometry: Warning - unknown child field type '" << child->name << "'" << std::endl;
      }

      if (!attribute.empty() && !transferFunction) {
        std::cout << "#osp:sg:PKDGeometry: Warning - PKD has attributes, but no transfer function... creating one." << std::endl;
        transferFunction = new TransferFunction();
      }
    }


    //! set radius to use for the spheres
    void PKDGeometry::setRadius(const float radius)
    { 
      this->radius = radius; 
      lastModified = TimeStamp::now(); 
    }

    //! set transferFunction to use for the spheres
    void PKDGeometry::setTransferFunction(Ref<sg::TransferFunction> transferFunction)
    { 
      this->transferFunction = transferFunction; 
      lastModified = TimeStamp::now(); 
    }
#endif

    OSP_REGISTER_SG_NODE(PKDGeometry);

  } // ::ospray::sg
} // ::ospray


