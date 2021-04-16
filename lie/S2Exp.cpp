#include "S2Exp.h"

#include <array>

namespace lie {

  template<class S>
  S2<S> exp(S2Alg<S> delta) {
    S norm2 = delta().squaredNorm();
    S cos_norm, sinc_norm;

    static S const taylor_0_bound = Eigen::NumTraits<double>::epsilon();
    static S const taylor_2_bound = sqrt(taylor_0_bound);
    static S const taylor_n_bound = sqrt(taylor_2_bound);
    //static S const test = Eigen::NumTraits<double>::dummy_precision();

    if (norm2 > taylor_n_bound) {
      S x = std::sqrt(norm2);
      cos_norm = std::cos(x);
      sinc_norm = std::sin(x) / x;
    }
    else {
      static const std::array<S, 7>  inv = { S(1 / 3.), S(1 / 4.), S(1 / 5.), S(1 / 6.), S(1 / 7.), S(1 / 8.), S(1 / 9.) };
      S cosi = 1.;
      S sinci = 1.;
      S term = -1 / 2. * norm2;
      for (int i = 0; i < 3; ++i) {
        cosi += term;
        term *= inv[2 * i];
        sinci += term;
        term *= -inv[2 * i + 1] * norm2;
      }
      cos_norm = cosi;
      sinc_norm = sinci;
    }

    S2<S> ret;
    ret()(0) = cos_norm;
    ret()(1) = delta()(0) * sinc_norm;
    ret()(2) = delta()(1) * sinc_norm;
       
    return ret;
  }

  template<class S>
  Eigen::Matrix<S, 3, 2> d_Rv_expd_d_expd0(const S2<S>& v) {
    auto R = s2toRot(v);

    //     [0 0]
    // R * [1 0]
    //     [0 1]

    return R.rightCols<2>();
  }


  template S2<double> exp<double>(S2Alg<double> g);
  template S2<float> exp<float>(S2Alg<float> g);

  template Eigen::Matrix<double, 3, 2> d_Rv_expd_d_expd0(const S2<double>& v);
  template Eigen::Matrix<float, 3, 2> d_Rv_expd_d_expd0(const S2<float>& v);

}