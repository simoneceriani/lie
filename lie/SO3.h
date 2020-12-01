#pragma once
#include <Eigen/Core>
#include "Tagged.h"


namespace lie {

  template <class S>
  using Rot3D = Tagged<S, 3, 3, struct TagSO3>;

  using Rot3Dd = Rot3D<double>;
  using Rot3Df = Rot3D<float>;


}

