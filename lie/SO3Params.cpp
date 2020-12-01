#include "SO3Params.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include "Constants.h"


namespace lie {

  template<typename S>
  Rot3D<S> params2Pose(const SO3Params<S>& g) {
    Rot3D<S> R;

    const S yaw_z = g.values()(0);
    const S pitch_y = g.values()(1);
    const S roll_x = g.values()(2);

    const S c_a = cos(yaw_z);
    const S s_a = sin(yaw_z);
    const S c_b = cos(pitch_y);
    const S s_b = sin(pitch_y);
    const S c_c = cos(roll_x);
    const S s_c = sin(roll_x);

    R()(0, 0) = c_a * c_b;
    R()(1, 0) = s_a * c_b;
    R()(2, 0) = -s_b;

    R()(0, 1) = c_a * s_b * s_c - s_a * c_c;
    R()(1, 1) = s_a * s_b * s_c + c_a * c_c;
    R()(2, 1) = c_b * s_c;

    R()(0, 2) = c_a * s_b * c_c + s_a * s_c;
    R()(1, 2) = s_a * s_b * c_c - c_a * s_c;
    R()(2, 2) = c_b * c_c;

    return R;
  }


  template<typename S>
  SO3Params<S> pose2Params(const Rot3D<S>& Rm) {
    SO3Params<S> ret;

    const auto& R = Rm();

    const S pitch_y = std::atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
    S yaw_z, roll_x;
    if (((S)M_PI - std::abs(pitch_y)) > Constants<S>::eps25) { // no singularity!
      yaw_z = std::atan2(R(1, 0), R(0, 0));
      roll_x = std::atan2(R(2, 1), R(2, 2));
    }
    else { // singularity! in this case either yaw or roll can be arbitrarily selected
      yaw_z = 0;
      roll_x = std::atan2(R(0, 1), R(1, 1));
    }

    return SO3Params<S>(Eigen::Matrix<S, 3, 1>(yaw_z, pitch_y, roll_x));
  }


  template Rot3D<double> params2Pose<double>(const SO3Params<double>& g);
  template Rot3D<float> params2Pose<float>(const SO3Params<float>& g);

  template SO3Params<double> pose2Params<double>(const Rot3D<double>& g);
  template SO3Params<float> pose2Params<float>(const Rot3D<float>& g);

}

