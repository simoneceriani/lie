#pragma once

#include <Eigen/Core>

#include "Tagged.h"

namespace lie {

  template <class S>
  using S2 = Tagged<S, 3, 1, struct TagS2>;

  using S2d = S2<double>;
  using S2f = S2<float>;

  template <class S>
  Eigen::Matrix<S, 3, 3>  s2toRot(const S2<S> & vec);

}