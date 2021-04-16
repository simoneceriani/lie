#include "S2.h"

namespace lie {

  template <class S>
  Eigen::Matrix<S, 3, 3> s2toRot(const S2<S>& vec) {

    double alpha = std::atan2(vec().z(), vec().y());
    double r = std::sqrt(vec().z() * vec().z() + vec().y() * vec().y());
    double c = cos(alpha);
    double s = sin(alpha);

    Eigen::Matrix<S, 3, 3> R;
    R.col(0) = vec();
    R.col(1) = Eigen::Matrix<S, 3, 1>(-r, vec().x() * c, vec().x() * s);
    R.col(2) = Eigen::Matrix<S, 3, 1>(0, -s, c);

    return R;
  }

  template Eigen::Matrix<double, 3, 3> s2toRot(const S2<double>& vec);
  template Eigen::Matrix<float, 3, 3> s2toRot(const S2<float>& vec);


}