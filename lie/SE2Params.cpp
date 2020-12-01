#include "SE2Params.h"


namespace lie {

  template<typename S>
  Pose2D<S> params2Pose(const SE2Params<S>& g) {
    Pose2D<S> T;
    const S angle = g.values()(2);
    T.linear()(0, 0) = cos(angle);
    T.linear()(0, 1) = -sin(angle);
    T.linear()(1, 0) = -T.linear()(0, 1);
    T.linear()(1, 1) = T.linear()(0, 0);
    T.translation()(0) = g.values()(0);
    T.translation()(1) = g.values()(1);
    return T;
  }


  template<typename S>
  SE2Params<S> pose2Params(const Pose2D<S>& g) {
    SE2Params<S> ret;
    ret.values().template head<2>() = g.translation();
    ret.values()(2) = std::atan2(g.linear()(1, 0), g.linear()(0, 0));
    return ret;
  }


  template Pose2D<double> params2Pose<double>(const SE2Params<double>& g);
  template Pose2D<float> params2Pose<float>(const SE2Params<float>& g);

  template SE2Params<double> pose2Params<double>(const Pose2D<double>& g);
  template SE2Params<float> pose2Params<float>(const Pose2D<float>& g);

}

