#pragma once

#include "geometry/GeometryShared.h"

#ifdef __cplusplus
namespace ispc {
#endif // __cplusplus

// using namespace rkcommon;

// struct PKDParticle {
//   float position[3];
// };
//
// struct PKDParticle4 {
//     float position[3];
//     unsigned int color;
// };
//
// struct INT3 {
//   int32 x,y,z;
// };

struct PKDGeometry
{
  // inherit from "Geometry" class: since ISPC doesn't support
  // inheritance we simply put the "parent" class as the first
  // member; this way any typecast to the parent class will get the
  // right members (including 'virtual' function pointers, etc)
  Geometry super;

  Data1D position;
  //Data1D color;

  float global_radius;
  vec4uc global_color;

  bool has_global_color;

  unsigned int num_particles;
  unsigned int num_innerNodes;

  box3f bounds;

#if 0
  //! flag specifying whether this is a quantized version of the particles
  bool isQuantized;

  //! flag specifying that color is encoded in the fourth position component
  bool isVec4;

  //! specifies type of color: o none, 1 RGBu8, 2 RGBAu8, 3 RGBf, 4 RGBAf, 5 I
  int colorType;

  //! number of particles
  uint64_t numParticles;
  //! number of inner nodes
  uint64_t numInnerNodes;

  //! array of particles, in kd-tree order
  PKDParticle *particle;
  
  // /*! gives the split dim for each inner node (if non-round robin
  //   split dim was used during construction), or NULL (in which case
  //   dim==(depth%3) */
  // unsigned int32 *innerNode_splitDim; 

  //! bounding box of particle centers
  //box3f centerBounds;

  //! bounding box of complete particles (centerBounds+radius)
  //box3f sphereBounds;

  /*! (maximum) particle radius */
  float particleRadius;

  /*! ray epsilon to avoid self-intersections, like the spheres geom */
  float epsilon;
#endif

#ifdef __cplusplus
  PKDGeometry()
      : global_radius(0.5f),
        global_color(vec4uc(255, 0, 0, 255)),
        has_global_color(true),
        num_particles(0)
  {}
};
} // namespace ispc
#else
};
#endif // __cplusplus