#pragma once


namespace lie {

  template <typename S>
  struct Constants {
  };


  // specific for type

  template <>
  struct Constants<double> {
    static const double eps;
    static const double eps25;
    static const double epsDummy;
    static const double log_cos_approx_1;
    static const double log_cos_approx_m1;
  };

  template <>
  struct Constants<float> {
    static const float eps;
    static const float eps25;
    static const float epsDummy;
    static const float log_cos_approx_1;
    static const float log_cos_approx_m1;
  };

}

