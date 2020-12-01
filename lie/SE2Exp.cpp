#include "SE2Exp.h"


namespace lie {

  template<typename S>
  Pose2D<S> exp(const SE2Alg<S>& g) {


    S cosTh = std::cos(g.values()(2));
    S sinTh = std::sin(g.values()(2));
    S sinTh_Th, m1cosTh_Th;
    if (std::abs(g.values()(2)) < Eigen::NumTraits<S>::epsilon()) {
      sinTh_Th = 1;
      m1cosTh_Th = 0;
    }
    else {
      sinTh_Th = sinTh / g.values()(2);
      m1cosTh_Th = (1 - cosTh) / g.values()(2);
    }

    Pose2D<S> ret;
    ret.linear()(0, 0) = cosTh;
    ret.linear()(0, 1) = -sinTh;
    ret.linear()(1, 0) = sinTh;
    ret.linear()(1, 1) = cosTh;

    ret.translation().x() = g.values().x() * sinTh_Th - g.values().y() * m1cosTh_Th;
    ret.translation().y() = g.values().x() * m1cosTh_Th + g.values().y() * sinTh_Th;
    return ret;


  }

  template Pose2D<double> exp<double>(const SE2Alg<double>& g);
  template Pose2D<float> exp<float>(const SE2Alg<float>& g);

}

