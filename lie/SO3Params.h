#pragma once
#include <Eigen/Core>
#include "Tagged.h"
#include "SO3.h"

namespace lie {

  template <class S>
  using SO3Params = Tagged<S, 3, 1, struct TagSO3Params>;

  template<typename S>
  Rot3D<S> params2Pose(const SO3Params<S>& g);

  template<typename S>
  SO3Params<S> pose2Params(const Rot3D<S>& g);

  using SO3Paramsd = SO3Params<double>;
  using SO3Paramsf = SO3Params<float>;

}
