#pragma once
#include <Eigen/Core>
#include "SE3.h"


namespace lie {

  template<class S>
  Eigen::Matrix<S, 6, 6> adjoint(const Pose3D<S>& p);

}

