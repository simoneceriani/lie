#include "SE3Exp.h"
#include "Constants.h"


namespace lie {

  template<typename S>
  Pose3D<S> exp(const SE3Alg<S>& g) {
    Pose3D<S> ret;
    auto w2 = g.values().template tail<3>().cwiseAbs2();
    const S theta_sq = w2.sum();  // sum of the squares
    const S theta = std::sqrt(theta_sq);

    if (theta < Constants<S>::eps) {
      const auto& v = g.values();
      S tmp5 = S(0.5) - theta_sq * (S(1.0 / 24.0) - S(1.0 / 720.0) * theta_sq);
      S tmp6 = v(3) * v(4) * tmp5;
      S tmp7 = S(1.0) - theta_sq * (S(1.0 / 120.0) * theta_sq + S(1.0 / 6.0));
      S tmp8 = v(3) * v(5) * tmp5;
      S tmp9 = v(4) * v(5) * tmp5;
      S tmp10 = v(2) * v(4) - v(1) * v(5);
      S tmp11 = v(1) * v(3) - v(0) * v(4);
      S tmp12 = v(0) * v(5) - v(2) * v(3);
      S tmp13 = S(1.0 / 6.0) - theta_sq * (S(1.0 / 120.0) - S(1.0 / 5040.0) * theta_sq);


      ret.linear()(0, 0) = 1 - (w2(2) + w2(1)) * tmp5;
      ret.linear()(1, 0) = v(5) * tmp7 + tmp6;
      ret.linear()(2, 0) = tmp8 - v(4) * tmp7;
      ret.linear()(0, 1) = tmp6 - v(5) * tmp7;
      ret.linear()(1, 1) = 1 - (w2(2) + w2(0)) * tmp5;
      ret.linear()(2, 1) = v(3) * tmp7 + tmp9;
      ret.linear()(0, 2) = v(4) * tmp7 + tmp8;
      ret.linear()(1, 2) = tmp9 - v(3) * tmp7;
      ret.linear()(2, 2) = 1 - (w2(1) + w2(0)) * tmp5;

      ret.translation()(0) = (v(4) * tmp11 - v(5) * tmp12) * tmp13 + tmp10 * tmp5 + v(0);
      ret.translation()(1) = (v(5) * tmp10 - v(3) * tmp11) * tmp13 + tmp12 * tmp5 + v(1);
      ret.translation()(2) = (v(3) * tmp12 - v(4) * tmp10) * tmp13 + tmp11 * tmp5 + v(2);



    }
    else {
      const auto& v = g.values();

      S tmp5 = 1 / theta_sq;
      S tmp7 = 1 - cos(theta);
      S tmp8 = v(3) * v(4) * tmp5 * tmp7;
      S tmp9 = 1 / theta;
      S tmp10 = sin(theta);
      S tmp11 = v(3) * v(5) * tmp5 * tmp7;
      S tmp12 = v(4) * v(5) * tmp5 * tmp7;
      S tmp13 = v(2) * v(4) - v(1) * v(5);
      S tmp14 = v(1) * v(3) - v(0) * v(4);
      S tmp15 = v(0) * v(5) - v(2) * v(3);
      S tmp16 = 1 - tmp9 * tmp10;


      //variable: res
      ret.linear()(0, 0) = 1 - (w2(2) + w2(1)) * tmp5 * tmp7;
      ret.linear()(1, 0) = v(5) * tmp9 * tmp10 + tmp8;
      ret.linear()(2, 0) = tmp11 - v(4) * tmp9 * tmp10;
      ret.linear()(0, 1) = tmp8 - v(5) * tmp9 * tmp10;
      ret.linear()(1, 1) = 1 - (w2(2) + w2(0)) * tmp5 * tmp7;
      ret.linear()(2, 1) = v(3) * tmp9 * tmp10 + tmp12;
      ret.linear()(0, 2) = v(4) * tmp9 * tmp10 + tmp11;
      ret.linear()(1, 2) = tmp12 - v(3) * tmp9 * tmp10;
      ret.linear()(2, 2) = 1 - (w2(1) + w2(0)) * tmp5 * tmp7;

      ret.translation()(0) = (v(4) * tmp14 - v(5) * tmp15) * tmp5 * tmp16 + tmp13 * tmp5 * tmp7 + v(0);
      ret.translation()(1) = (v(5) * tmp13 - v(3) * tmp14) * tmp5 * tmp16 + tmp15 * tmp5 * tmp7 + v(1);
      ret.translation()(2) = (v(3) * tmp15 - v(4) * tmp13) * tmp5 * tmp16 + tmp14 * tmp5 * tmp7 + v(2);

    }

    ret.matrix().row(3).template head<3>().setZero();
    ret.matrix()(3, 3) = 1;
    return ret;
  }

  template Pose3D<double> exp<double>(const SE3Alg<double>& g);
  template Pose3D<float> exp<float>(const SE3Alg<float>& g);

}

