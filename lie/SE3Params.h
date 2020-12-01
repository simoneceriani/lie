#pragma once
#include "SE3.h"
#include <Eigen/Core>
#include "Tagged.h"

namespace lie {

  struct TagSE3Params {};

  template <class S>
  class SE3Params : public Tagged<S, 6, 1, TagSE3Params> {

  public:

    SE3Params() {

    }

    template <class Derived>
    explicit SE3Params(const Eigen::EigenBase<Derived>& d)
      : eigenExtras::Tagged<S, 6, 1, TagSE3Params>(d) {

    }

    explicit SE3Params(S x, S y, S z, S yaw, S pitch, S roll);


    Eigen::Block<Eigen::Matrix<S, 6, 1>, 3, 1> translation() {
      return this->values().template head<3>();
    }

    const Eigen::Block<const Eigen::Matrix<S, 6, 1>, 3, 1> translation() const {
      return this->values().template head<3>();
    }

    Eigen::Block<Eigen::Matrix<S, 6, 1>, 3, 1> eulerAngles() {
      return this->values().template tail<3>();
    }

    const Eigen::Block<const Eigen::Matrix<S, 6, 1>, 3, 1> eulerAngles() const {
      return this->values().template tail<3>();
    }


  };

  template<typename S>
  Pose3D<S> params2Pose(const SE3Params<S>& g);

  template<typename S>
  SE3Params<S> pose2Params(const Pose3D<S>& g);

  using SE3Paramsd = SE3Params<double>;
  using SE3Paramsf = SE3Params<float>;

}
