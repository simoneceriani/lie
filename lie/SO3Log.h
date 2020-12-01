#pragma once
#include "SO3.h"
#include "SO3Alg.h"

namespace lie {

  template<typename S>
  SO3Alg<S> log(const Rot3D<S>& g);

}
