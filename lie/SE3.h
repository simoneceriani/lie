#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>



namespace lie {

  template <class S>
  using Pose3D = Eigen::Transform<S, 3, Eigen::Isometry>;

  using Pose3Dd = Pose3D<double>;
  using Pose3Df = Pose3D<float>;

  template<typename S>
  using Vector6 = Eigen::Matrix<S, 6, 1>;

  using Vector6d = Vector6<double>;
  using Vector6f = Vector6<float>;

}

