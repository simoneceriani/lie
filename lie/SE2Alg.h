#pragma once
#include <Eigen/Core>
#include "Tagged.h"

namespace lie {

  template <class S>
  using SE2Alg = Tagged<S, 3, 1, struct TagSE2Alg>;

  using SE2Algd = SE2Alg<double>;
  using SE2Algf = SE2Alg<float>;

}

