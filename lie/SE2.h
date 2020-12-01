#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>



namespace lie {

  template <class S>
  using Pose2D = Eigen::Transform<S, 2, Eigen::Isometry>;

  using Pose2Dd = Pose2D<double>;
  using Pose2Df = Pose2D<float>;


}

