#include <catch2/catch_test_macros.hpp>

#include "lie/SE2.h"
#include "lie/SE2Alg.h"
#include "lie/SE2Exp.h"
#include "lie/SE2Params.h"

static const int N_RANDOM;

TEST_CASE( "SE2 params2Pose2params", "[SE2]" ) {
    
  for (int i = 0; i < N_RANDOM; i++) {
    lie::SE2Paramsd par;
    par().setRandom();
    auto pose = lie::params2Pose(par);
    auto par2 = lie::pose2Params(pose);
    REQUIRE((par2() - par()).squaredNorm() < Eigen::NumTraits<double>::dummy_precision());
  }

}