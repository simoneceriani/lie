#include <Eigen/Core>
#include "SkewSymm.h"


namespace lie {

  template<typename S>
  Eigen::Matrix<S, 3, 1> normalize(const Eigen::Ref<const Eigen::Matrix<S, 3, 1>>& in, Eigen::Ref<Eigen::Matrix<S, 3, 3>> d_out_d_in) {
    Eigen::Matrix<S, 3, 1> out;

    Eigen::Matrix<S, 3, 1> in2 = in.cwiseAbs2();
    S normSq = in2.sum();
    S norm = std::sqrt(normSq);
    S norm_3_2 = normSq * norm;

    out = in / norm;

    d_out_d_in(0, 0) = 1 / norm - in2.x() / norm_3_2;
    d_out_d_in(0, 1) = -in.x() * in.y() / norm_3_2;
    d_out_d_in(0, 2) = -in.x() * in.z() / norm_3_2;

    d_out_d_in(1, 0) = d_out_d_in(0, 1);
    d_out_d_in(1, 1) = 1 / norm - in2.y() / norm_3_2;
    d_out_d_in(1, 2) = -in.y() * in.z() / norm_3_2;

    d_out_d_in(2, 0) = d_out_d_in(0, 2);
    d_out_d_in(2, 1) = d_out_d_in(1, 2);
    d_out_d_in(2, 2) = 1 / norm - in2.z() / norm_3_2;

    return out;
  }


  template<typename S>
  Eigen::Matrix<S, 3, 6> d_expg0p_d_g(const Eigen::Ref<const Eigen::Matrix<S, 3, 1>>& in) {
    Eigen::Matrix<S, 3, 6> ret;
    ret.template topLeftCorner<3, 3>().setIdentity();
    ret.template topRightCorner<3, 3>() = -skewSymm<S>(in);
    return ret;
  }


  template Eigen::Matrix<double, 3, 1> normalize<double>(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& in, Eigen::Ref<Eigen::Matrix<double, 3, 3>> d_out_d_in);
  template Eigen::Matrix<float, 3, 1> normalize<float>(const Eigen::Ref<const Eigen::Matrix<float, 3, 1>>& in, Eigen::Ref<Eigen::Matrix<float, 3, 3>> d_out_d_in);


  template Eigen::Matrix<double, 3, 6 > d_expg0p_d_g(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& in);
  template Eigen::Matrix<float, 3, 6 > d_expg0p_d_g(const Eigen::Ref<const Eigen::Matrix<float, 3, 1>>& in);
}

