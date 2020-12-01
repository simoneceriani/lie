#include "SkewSymm.h"

namespace lie {

  template<class S>
  Eigen::Matrix<S, 3, 3> skewSymm(const Eigen::Ref<const Eigen::Matrix<S, 3, 1>>& v) {
    Eigen::Matrix<S, 3, 3> R;

    R.diagonal().setZero();
    R(0, 1) = -v(2);
    R(0, 2) = v(1);
    R(1, 2) = -v(0);
    R(1, 0) = -R(0, 1);
    R(2, 0) = -R(0, 2);
    R(2, 1) = -R(1, 2);

    return R;
  }

  template Eigen::Matrix<double, 3, 3> skewSymm<double>(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& a);
  template Eigen::Matrix<float, 3, 3> skewSymm<float>(const Eigen::Ref<const Eigen::Matrix<float, 3, 1>>& a);


}

