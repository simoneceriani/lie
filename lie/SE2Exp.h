#pragma once
#include "SE2.h"
#include "SE2Alg.h"

namespace lie {

  template<typename S>
  Pose2D<S> exp(const SE2Alg<S>& g);

}
