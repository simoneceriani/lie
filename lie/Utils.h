#pragma once

#include <Eigen/Core>


namespace lie {

  template<typename S>
  Eigen::Matrix<S, 3, 1> normalize(const Eigen::Ref<const Eigen::Matrix<S, 3, 1>>& in, Eigen::Ref<Eigen::Matrix<S, 3, 3>> d_out_d_in);

  template<typename S>
  Eigen::Matrix<S, 3, 6> d_expg0p_d_g(const Eigen::Ref<const Eigen::Matrix<S, 3, 1>>& in);

}

