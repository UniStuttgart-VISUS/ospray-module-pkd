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

#include <thread>
#include "PartiKD.h"
#include "PKDConfig.h"
#include "../ospray/MinMaxBVH2.h"

#include "ospcommon/constants.h"
#include "ospcommon/FileName.h"

#define CHECK 1

//#define DIM_FROM_DEPTH 1
//#define DIM_ROUND_ROBIN 1

namespace ospray {
  using std::endl;
  using std::cout;

  void PartiKD::setDim(size_t ID, int dim) const
  {
#if DIM_FROM_DEPTH
    return;
#else
    vec3f &particle = (vec3f &)this->model->position[ID];
    int &pxAsInt = (int &)particle.x;
    pxAsInt = (pxAsInt & ~3) | dim;
#endif
  }

  struct SubtreeIterator {
    size_t curInLevel;
    size_t maxInLevel;
    size_t current;

    __forceinline SubtreeIterator(size_t root)
      : curInLevel(0), maxInLevel(1), current(root) 
    {}
    __forceinline operator size_t() const { return current; }
    __forceinline void operator++() {
      ++current;
      ++curInLevel;
      if (curInLevel == maxInLevel) {
        current = PartiKD::leftChildOf(current-maxInLevel);
        maxInLevel += maxInLevel;
        curInLevel = 0;
      }
    }
    __forceinline SubtreeIterator &operator=(const SubtreeIterator &other) {
      curInLevel = other.curInLevel;
      maxInLevel = other.maxInLevel;
      current    = other.current;
      return *this;
    }
  };

  struct PKDBuildJob {
    const PartiKD *const pkd;
    const size_t nodeID;
    const box3f  bounds;
    const size_t depth;
    __forceinline PKDBuildJob(const PartiKD *pkd, size_t nodeID, box3f bounds, size_t depth) 
      : pkd(pkd), nodeID(nodeID), bounds(bounds), depth(depth) 
    {};
  };

  void *pkdBuildThread(void *arg)
  {
    PKDBuildJob *job = (PKDBuildJob *)arg;
    job->pkd->buildRec(job->nodeID,job->bounds,job->depth);
    delete job;
    return NULL;
  } 

  //#define FAST 1

#if FAST
#  define POS(idx,dim) position[idx].x
#else
#  define POS(idx,dim) pos(idx,dim)
#endif


  void PartiKD::buildRec(const size_t nodeID, 
                         const box3f &bounds,
                         const size_t depth) const
  {
    // if (depth < 4)
    // std::cout << "#osp:pkd: building subtree " << nodeID << std::endl;
    if (!hasLeftChild(nodeID)) 
      // has no children -> it's a valid kd-tree already :-)
      return;
    
    // we have at least one child.
#if DIM_ROUND_ROBIN
    const size_t dim = depth % 3;
#else
    const size_t dim = maxDim(bounds.size());
    // if (depth < 4) { PRINT(bounds); printf("depth %ld-> dim %ld\n",depth,dim); }
#endif
    const size_t N = numParticles;
#if FAST
    ParticleModel::vec_t *const position = (ParticleModel::vec_t*)(&model->position[0].x+dim);
#endif
    if (!hasRightChild(nodeID)) {
      // no right child, but not a leaf emtpy. must have exactly one
      // child on the left. see if we have to swap, but otherwise
      // nothing to do.
      size_t lChild = leftChildOf(nodeID);
      if (POS(lChild,dim) > POS(nodeID,dim)) 
        swap(nodeID,lChild);
      // and done
      setDim(nodeID,dim);
      return;
    }
 
    {
#if 1
    // we have a left and a right subtree, each of at least 1 node.
    SubtreeIterator l0(leftChildOf(nodeID));
    SubtreeIterator r0(rightChildOf(nodeID));

# if 1
    SubtreeIterator l((size_t)l0); //(leftChildOf(nodeID));
    SubtreeIterator r((size_t)r0); //(rightChildOf(nodeID));
# else
    SubtreeIterator l = l0; //(leftChildOf(nodeID));
    SubtreeIterator r = r0; //(rightChildOf(nodeID));
# endif
    

    // size_t numSwaps = 0, numComps = 0;
    float rootPos = POS(nodeID,dim);
    while(1) {

      while (isValidNode(l,N) && (POS(l,dim) <= rootPos)) ++l; 
      while (isValidNode(r,N) && (POS(r,dim) >= rootPos)) ++r; 

      if (isValidNode(l,N)) {
        if (isValidNode(r,N)) {
          // both mis-mathces valid, just swap them and go on
          swap(l,r); ++l; ++r;
          continue;
        } else {
          // mis-match on left side, but nothing on right side to swap with: swap with root
          // --> can't go on on right side any more, but can still compact matches on left
          l0 = l;
          ++l;
          while (isValidNode(l,N)) {
            if (POS(l,dim) <= rootPos) { swap(l0,l); ++l0; }
            ++l;
          }
          swap(nodeID,l0); ++l0;
          rootPos = POS(nodeID,dim);

          l = l0;
          r = r0;
          continue;
        }
      } else {
        if (isValidNode(r,N)) {
          // mis-match on left side, but nothing on right side to swap with: swap with root
          // --> can't go on on right side any more, but can still compact matches on left
          r0 = r;
          ++r;
          while (isValidNode(r,N)) {
            if (POS(r,dim) >= rootPos) { swap(r0,r); ++r0; }
            ++r;
          }
          swap(nodeID,r0); ++r0;
          rootPos = POS(nodeID,dim);
          
          l = l0;
          r = r0;
          continue;
        } else {
          // no mis-match on either side ... done.
          break;
        }
      }
    }
#else
    size_t lRoot = leftChildOf(nodeID);
    size_t rRoot = rightChildOf(nodeID);
    {
      // build max-heap on left
      SubtreeIterator l(lRoot); ++l;
      while (isValidNode(l)) {
        // heap up
        size_t cur = l;
        while (cur != lRoot) {
          size_t parent = parentOf(cur);
          if (pos(cur,dim) <= pos(parent,dim)) break;
          swap(cur,parent);
          cur = parent;
        }
        ++l;
      }
    }
      
    {
      // build min-heap on right
      SubtreeIterator r(rRoot); ++r;
      while (isValidNode(r)) {
        // heap up
        size_t cur = r;
        while (cur != rRoot) {
          size_t parent = parentOf(cur);
          if (pos(cur,dim) >= pos(parent,dim)) break;
          swap(cur,parent);
          cur = parent;
        }
        ++r;
      }
    }

    while(1) {
      // lRoot has biggest in left tree; rRoot has smallest in right tree
      if (pos(lRoot,dim) <= pos(rRoot,dim)) {
        // left and right subtree's dont' overlap. check root.
        if (pos(nodeID,dim) < pos(lRoot,dim)) {
          swap(nodeID,lRoot);
        } else if (pos(nodeID,dim) > pos(rRoot,dim)) {
          swap(nodeID,rRoot);
        } 
        // done, tree is balanced
        break;
      }

      swap(lRoot,rRoot);

      // update max-heap on left: bubble up biggest child, until no longer possible
      {
        size_t cur = lRoot;
        while (1) {
          size_t lChild = leftChildOf(cur), rChild = lChild+1;
          if (!isValidNode(lChild)) break;
          size_t target = lChild;
          if (isValidNode(rChild) && pos(rChild,dim) > pos(lChild,dim)) target = rChild;
          if (pos(cur,dim) >= pos(target,dim)) break;
          swap(cur,target);
          cur = target;
        }
      }

      // update min-heap on right: bubble up smallest child, until no longer possible
      {
        size_t cur = rRoot;
        while (1) {
          size_t lChild = leftChildOf(cur), rChild = lChild+1;
          if (!isValidNode(lChild)) break;
          size_t target = lChild;
          if (isValidNode(rChild) && pos(rChild,dim) < pos(lChild,dim)) target = rChild;
          if (pos(cur,dim) <= pos(target,dim)) break;
          swap(cur,target);
          cur = target;
        }
      }
    }
#endif

#if CHECK
    // check tree:
    {
      SubtreeIterator l(leftChildOf(nodeID));
      SubtreeIterator r(rightChildOf(nodeID));
      while (isValidNode(l)) {
        if (!(pos(l,dim) <= pos(nodeID,dim)))
          throw std::runtime_error("error in building. not a valid kd-tree...");
        ++l;
      }
      while (isValidNode(r)) {
        if (!(pos(r,dim) >= pos(nodeID,dim)))
          throw std::runtime_error("error in building. not a valid kd-tree...");
        ++r;
      }
    }
#endif
    }

    box3f lBounds = bounds;
    box3f rBounds = bounds;
    
    setDim(nodeID,dim);
      
    lBounds.upper[dim] = rBounds.lower[dim] = pos(nodeID,dim);

    if ((numLevels - depth) > 20) {
      std::thread lThread([&](){
        pkdBuildThread(new PKDBuildJob(this,leftChildOf(nodeID),lBounds,depth+1));
      });
      buildRec(rightChildOf(nodeID),rBounds,depth+1);
      lThread.join();
    } else {
        buildRec(leftChildOf(nodeID),lBounds,depth+1);
        buildRec(rightChildOf(nodeID),rBounds,depth+1);
      }
  }

  inline void PartiKD::swap(const size_t a, const size_t b) const 
  { 
    std::swap(model->position[a],model->position[b]);
    for (size_t i=0;i<model->attribute.size();i++)
      std::swap(model->attribute[i]->value[a],model->attribute[i]->value[b]);
    if (!model->type.empty())
      std::swap(model->type[a],model->type[b]);
  }

  void PartiKD::build(ParticleModel *model) 
  {
    PING;
    assert(this->model == NULL);
    assert(model);
    this->model = model;


    assert(!model->position.empty());
    numParticles = model->position.size();
    assert(numParticles <= (1ULL << 31));

#if 0
    cout << "#osp:pkd: TEST: RANDOMIZING PARTICLES" << endl;
    for (size_t i=numParticles-1;i>0;--i) {
      size_t j = size_t(drand48()*i);
      if (i != j) swap(i,j);
    }
    cout << "#osp:pkd: RANDOMIZED" << endl;
#endif

    numInnerNodes = numInnerNodesOf(numParticles);

    // determine num levels
    numLevels = 0;
    size_t nodeID = 0;
    while (isValidNode(nodeID)) { ++numLevels; nodeID = leftChildOf(nodeID); }
    PRINT(numLevels);

    const box3f &bounds = model->getBounds();
    std::cout << "#osp:pkd: bounds of model " << bounds << std::endl;
    std::cout << "#osp:pkd: number of input particles " << numParticles << std::endl;
    buildRec(0,bounds,0);
  }

  //! save to xml+binary file(s)
  void PartiKD::saveOSP(const std::string &fileName)
  {
    FILE *xml = fopen(fileName.c_str(),"w");
    assert(xml);
    const std::string binFileName = fileName + "bin";
    FILE *bin = fopen(binFileName.c_str(),"wb");

    fprintf(xml,"<?xml version=\"1.0\"?>\n");

    fprintf(xml,"<OSPRay>\n"); 
    { 
      saveOSP(xml,bin);
    } 
    fprintf(xml,"</OSPRay>\n");

    fclose(bin);
    fclose(xml);
  }


  //! save to xml+binary file(s)
  void PartiKD::saveOSPQuantized(const std::string &fileName)
  {
    FILE *xml = fopen(fileName.c_str(),"w");
    assert(xml);
    const std::string binFileName = fileName + "bin";
    FILE *bin = fopen(binFileName.c_str(),"wb");

    fprintf(xml,"<?xml version=\"1.0\"?>\n");

    fprintf(xml,"<OSPRay>\n"); 
    { 
      saveOSPQuantized(xml,bin);
    } 
    fprintf(xml,"</OSPRay>\n");

    fclose(bin);
    fclose(xml);
  }


  void PartiKD::saveOSPQuantized(FILE *xml, FILE *bin)
  {
    printf("#osp:pkd: writing quantized version");
    fprintf(xml,"<PKDGeometry>\n");
      // fprintf(xml,"<Renderer type=\"PKDSplatter\" name=\"splat\">\n");
      //    fprintf(xml,"<PKDGeometry>\n");

    box3f bounds = model->getBounds();
    fprintf(xml,"<position ofs=\"%li\" count=\"%li\" format=\"uint64\"/>\n",
    // fprintf(xml,"<data name=\"particles\" ofs=\"%li\" count=\"%li\" format=\"uint64\"/>\n",
            ftell(bin),numParticles);
    for (int i=0;i<model->position.size();i++) {
      vec3f p = model->position[i];
      uint64 dim = ((int&)p.x) & 3;
      
      uint64 ix = uint64((1<<20) * (p.x-bounds.lower.x) / (bounds.upper.x-bounds.lower.x));
      uint64 iy = uint64((1<<20) * (p.y-bounds.lower.y) / (bounds.upper.y-bounds.lower.y));
      uint64 iz = uint64((1<<20) * (p.z-bounds.lower.z) / (bounds.upper.z-bounds.lower.z));
      
      ix = std::max(std::min(ix,((uint64)1<<20)-1),(uint64)0);
      iy = std::max(std::min(iy,((uint64)1<<20)-1),(uint64)0);
      iz = std::max(std::min(iz,((uint64)1<<20)-1),(uint64)0);
      
      uint64 quantized = (ix << 2) | (iy << 22) | (iz << 42) | dim;
      fwrite(&quantized,sizeof(quantized),1,bin);
    }

    if (model->radius > 0.)
      fprintf(xml,"<radius>%f</radius>\n",model->radius);
    fprintf(xml,"<useOldAlphaSpheresCode value=\"0\"/>\n");
    fprintf(xml,"</PKDGeometry>\n");
  }

  //! save to xml+binary file(s)
  void PartiKD::saveOSP(FILE *xml, FILE *bin)
  {
    fprintf(xml,"<PKDGeometry>\n");

    fprintf(xml,"<position ofs=\"%li\" count=\"%li\" format=\"vec3f\"/>\n",
            ftell(bin),numParticles);
    fwrite(&model->position[0],sizeof(ParticleModel::vec_t),numParticles,bin);
    for (int i=0;i<model->attribute.size();i++) {
      ParticleModel::Attribute *attr = model->attribute[i];
      fprintf(xml,"<attribute name=\"%s\" ofs=\"%li\" count=\"%li\" format=\"float\"/>\n",
              attr->name.c_str(),ftell(bin),numParticles);
      fwrite(&attr->value[0],sizeof(float),numParticles,bin);
    }
    if (!model->type.empty()) {
      float *f = new float[model->type.size()];
      for (int i=0;i<model->type.size();i++) f[i] = model->type[i];
      fprintf(xml,"<attribute name=\"atomType\" ofs=\"%li\" count=\"%li\" format=\"float\"/>\n",
              ftell(bin),numParticles);
      fwrite(f,sizeof(float),numParticles,bin);
      delete[] f;
    }
    if (model->radius > 0.)
      fprintf(xml,"<radius>%f</radius>\n",model->radius);
    fprintf(xml,"<useOldAlphaSpheresCode value=\"0\"/>\n");
    fprintf(xml,"</PKDGeometry>\n");

    // fprintf(xml,"<Renderer type=\"ao1\" name=\"default\">\n");
    // fprintf(xml,"</Renderer>\n");
  }

  void partiKDMain(int ac, char **av)
  {
    std::vector<ospcommon::FileName> input;
    std::string output, outputQuantized;
    ParticleModel model;
    bool roundRobin = false;

    for (int i=1;i<ac;i++) {
      std::string arg = av[i];
      if (arg[0] == '-') {
        if (arg == "-o") {
          output = av[++i];
        } else if (arg == "--radius") {
          model.radius = atof(av[++i]);
        } else if (arg == "--quantize") {
          if (i+1 >= ac || av[i+1][0] == '-')
            throw std::runtime_error("no filename passed to '--quantize'");
          outputQuantized = av[++i];
        } else if (arg == "--round-robin") {
          roundRobin = true;
        } else {
          throw std::runtime_error("unknown parameter '"+arg+"'");
        }
      } else {
        input.push_back(arg);
      }
    }
    if (input.empty()) {
      throw std::runtime_error("no input file(s) specified");
    }
    if (output == "")
      throw std::runtime_error("no output file specified");
    
    if (model.radius == 0.f)
      std::cout << "#osp:pkd: no radius specified on command line" << std::endl;

    // load the input(s)
    for (int i=0;i<input.size();i++) {
      cout << "#osp:pkd: loading " << input[i] << endl;
      model.load(input[i]);
    }

    if (model.radius == 0.f) {
      throw std::runtime_error("no radius specified via either command line or model file");
    }
#if PKD_LIDAR_ENABLED
    // WILL: lidar hack to re-scale data sets to not have so much precision issues with shadows/ao
    // We can't compare the box against embree::empty?
    if (input[0].ext() == "las" || input[0].ext() == "laz"){
      std::cout << "lidar data bounds: " << model.lidar_current_bounds << "\n";
      vec3f diag = model.lidar_current_bounds.upper - model.lidar_current_bounds.lower;
      vec3f axis_scale(1, 1, 1);
      // Find the largest axis and re-scale it into some smaller range, [-100, 100]
      int largest_axis = 0;
      if (diag.y > diag.x){
        if (diag.z > diag.y){
            largest_axis = 2;
            axis_scale.x = diag.x / diag.z;
            axis_scale.y = diag.y / diag.z;
        } else {
            largest_axis = 1;
            axis_scale.x = diag.x / diag.y;
            axis_scale.z = diag.z / diag.y;
        }
      } else {
        if (diag.z > diag.x){
            largest_axis = 2;
            axis_scale.x = diag.x / diag.z;
            axis_scale.y = diag.y / diag.z;
        } else {
            largest_axis = 0;
            axis_scale.y = diag.y / diag.x;
            axis_scale.z = diag.z / diag.x;
        }
      }
      vec3f new_min = vec3f(-100) * axis_scale;
      vec3f new_max = vec3f(100) * axis_scale;
      std::cout << "axis_scale " << axis_scale << ", new_min = " << new_min
          << ", new_max = " << new_max << "\n";
      for (size_t i = 0; i < model.position.size(); ++i){
        model.position[i] = ((new_max - new_min) * (model.position[i] - model.lidar_current_bounds.lower))
            / diag + new_min;
      }
    }
#endif

    double before = getSysTime();
    std::cout << "#osp:pkd: building tree ..." << std::endl;
    PartiKD partiKD(roundRobin);
    partiKD.build(&model);
    double after = getSysTime();
    std::cout << "#osp:pkd: tree built (" << (after-before) << " sec)" << std::endl;

    std::cout << "#osp:pkd: writing binary data to " << output << endl;
    partiKD.saveOSP(output);
    if (outputQuantized != "") {
      std::cout << "#osp:pkd: writing QUANTIZED binary data to " << outputQuantized << endl;
      partiKD.saveOSPQuantized(outputQuantized);
    }

    std::cout << "#osp:pkd: done." << endl;
  }
}

using std::cout;
using std::endl;

int main(int ac, char **av)
{
  try {
    ospray::partiKDMain(ac,av);
  } catch (std::runtime_error(e)) {
    cout << "#osp:pkd (fatal): " << e.what() << endl;
    cout << "usage:" << endl;
    cout << "./ospPartiKD <inputfile(s)> -o output.pkd --radius <radius> [--round-robin] [--quantize quantized.pkd]\n" << endl;
    
  }
}
