#include "SE3Log.h"
#include "Constants.h"

#include "SO3Log.h"


namespace lie {

  template<typename S>
  SE3Alg<S> log(const Pose3D<S>& g) {
    SE3Alg<S> ret;

    // SO3 log
    Rot3D<S> pl(g.linear());
    auto vl = log(pl);
    ret.values().template tail<3>() = vl.values();

    const S theta_sq = vl.values().cwiseAbs2().sum();
    const S theta = sqrt(theta_sq);

    S a, b, c;
    //std::cout << "theta_sq " << theta_sq << std::endl;
    if (theta < Constants<S>::eps) {
      a = 1 - theta_sq * (S(1.0 / 6) + theta_sq * S(1.0 / 120));
      b = S(0.5) - theta_sq * (S(1.0 / 24) - theta_sq * S(1.0 / 720));
      c = S(1.0 / 6) - theta_sq * (S(1.0 / 120) - theta_sq * S(1.0 / 5040));
    }
    else {
      const S inv_theta = 1 / theta;
      const S inv_theta_sq = 1 / theta_sq;
      a = inv_theta * sin(theta);
      b = inv_theta_sq * (1 - cos(theta));
      c = inv_theta_sq * (1 - a);
    }

    S d;
    if (theta_sq < Constants<S>::eps25) {
      d = S(1.0 / 12) + theta_sq * (S(1.0 / 720) + theta_sq * S(1.0 / 30240));
    }
    else if (theta_sq > 9) {
      d = (b - S(0.5) * a) / (b * theta_sq);
    }
    else {
      d = (b * S(0.5) - c) / a;
    }

    auto wxt = vl.values().cross(g.translation());
    auto wxwxt = vl.values().cross(wxt);

    ret.values().template head<3>() = g.translation() - 0.5 * wxt + d * wxwxt;

    return ret;
  }

  template SE3Alg<double> log<double>(const Pose3D<double>& g);
  template SE3Alg<float> log<float>(const Pose3D<float>& g);

}

