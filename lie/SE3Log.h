#pragma once
#include "SE3.h"
#include "SE3Alg.h"

namespace lie {

  template<typename S>
  SE3Alg<S> log(const Pose3D<S>& g);

}
