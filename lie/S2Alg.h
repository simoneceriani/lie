#pragma once

#include <Eigen/Core>
#include "Tagged.h"

namespace lie {

  template <class S>
  using S2Alg = Tagged<S, 2, 1, struct TagS2Alg>;

  using S2Algd = S2Alg<double>;
  using S2Algf = S2Alg<float>;

}