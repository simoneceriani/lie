#include <Eigen/Core>
#include "Adjoint.h"
#include "SkewSymm.h"


namespace lie {

  template<class S>
  Eigen::Matrix<S, 6, 6> adjoint(const Pose3D<S>& p) {
    Eigen::Matrix<S, 6, 6> ad;
    ad.template topLeftCorner<3, 3>() = p.linear();
    ad.template topRightCorner<3, 3>() = skewSymm<S>(p.translation()) * p.linear();
    ad.template bottomRightCorner<3, 3>() = p.linear();
    ad.template bottomLeftCorner<3, 3>().setZero();
    return ad;
  }

  template Eigen::Matrix<double, 6, 6> adjoint(const Pose3D<double>& p);
  template Eigen::Matrix<float, 6, 6> adjoint(const Pose3D<float>& p);


}

