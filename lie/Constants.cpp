#include "Constants.h"

#include <limits>
#include <Eigen/Core>



namespace lie {

  const double Constants<double>::eps = std::numeric_limits<double>::epsilon();
  const double Constants<double>::eps25 = std::numeric_limits<double>::epsilon() * 25;
  const double Constants<double>::epsDummy = Eigen::NumTraits<double>::dummy_precision();
  const double Constants<double>::log_cos_approx_1 = (double)0.999856;
  const double Constants<double>::log_cos_approx_m1 = (double)-0.999856;


  const float  Constants<float>::eps = std::numeric_limits<float>::epsilon();
  const float  Constants<float>::eps25 = std::numeric_limits<float>::epsilon() * 25;
  const float  Constants<float>::epsDummy = Eigen::NumTraits<float>::dummy_precision();
  const float  Constants<float>::log_cos_approx_1 = (float)0.999856;
  const float  Constants<float>::log_cos_approx_m1 = (float)-0.999856;


}

