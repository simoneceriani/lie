#pragma once
#include <Eigen/Core>
#include "Tagged.h"

#include "SE2.h"

namespace lie {

  template <class S>
  using SE2Params = Tagged<S, 3, 1, struct TagSE2Params>;

  template<typename S>
  Pose2D<S> params2Pose(const SE2Params<S>& g);

  template<typename S>
  SE2Params<S> pose2Params(const Pose2D<S>& g);

  using SE2Paramsd = SE2Params<double>;
  using SE2Paramsf = SE2Params<float>;

}
