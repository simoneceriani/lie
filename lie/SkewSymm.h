#pragma once
#include <Eigen/Core>

namespace lie {

  template<class S>
  Eigen::Matrix<S, 3, 3> skewSymm(const Eigen::Ref<const Eigen::Matrix<S, 3, 1>>& a);

}

