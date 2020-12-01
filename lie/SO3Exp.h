#pragma once
#include "SO3.h"
#include "SO3Alg.h"

namespace lie {

  template<typename S>
  Rot3D<S> exp(const SO3Alg<S>& g);

}
