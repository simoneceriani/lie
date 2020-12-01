#pragma once
#include "SE3.h"
#include "SE3Alg.h"

namespace lie {

  template<typename S>
  Pose3D<S> exp(const SE3Alg<S>& g);

}
