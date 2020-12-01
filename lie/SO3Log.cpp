#include "SO3Log.h"
#include "Constants.h"

#define _USE_MATH_DEFINES
#include <math.h>


namespace lie {

  template <typename T> int signum(T val) {
    return (T(0) < val) - (val < T(0));
  }

  template<typename S>
  SO3Alg<S> log(const Rot3D<S>& Rm) {
    const auto& R = Rm();

    SO3Alg<S> ret;
    ret.values()(0) = S(0.5) * (R(2, 1) - R(1, 2));
    ret.values()(1) = S(0.5) * (R(0, 2) - R(2, 0));
    ret.values()(2) = S(0.5) * (R(1, 0) - R(0, 1));

    const S ct = S(0.5) * (R.trace() - 1);
    const S st2 = ret.values().cwiseAbs2().sum();
    const S st = std::sqrt(st2);

    if (ct > Constants<S>::log_cos_approx_1) {
      const S f = 1
        + st2
        * (S(1.0 / 6.0)
          + st2 * (S(3.0 / 40.0) + st2 * S(5.0 / 112.0)));
      ret.values() *= f;
    }
    else if (ct > Constants<S>::log_cos_approx_m1) {
      const S theta = acos(ct);
      const S f = theta / st;
      ret.values() *= f;
    }
    else {
      const S theta = S(M_PI) - asin(st);
      const S invB = (theta * theta) / (1 - ct);

      const S w00 = invB * (R(0, 0) - ct);
      const S w11 = invB * (R(1, 1) - ct);
      const S w22 = invB * (R(2, 2) - ct);

      const S w01 = invB * S(0.5) * (R(1, 0) + R(0, 1));
      const S w02 = invB * S(0.5) * (R(2, 0) + R(0, 2));
      const S w12 = invB * S(0.5) * (R(2, 1) + R(1, 2));

      if (w00 > w11) {

        if (w00 > w22) {
          ret.values()(0) = signum(ret.values()(0)) * sqrt(w00);
          const S inv_w0 = 1 / ret.values()(0);
          ret.values()(1) = w01 * inv_w0;
          ret.values()(2) = w02 * inv_w0;
        }
        else {
          ret.values()(2) = signum(ret.values()(2)) * sqrt(w22);
          const S inv_w2 = 1 / ret.values()(2);
          ret.values()(0) = w02 * inv_w2;
          ret.values()(1) = w12 * inv_w2;
        }
      }
      else if (w11 > w22) {
        ret.values()(1) = signum(ret.values()(1)) * sqrt(w11);
        const S inv_w1 = 1 / ret.values()(1);
        ret.values()(0) = w01 * inv_w1;
        ret.values()(2) = w12 * inv_w1;
      }
      else {
        ret.values()(2) = signum(ret.values()(2)) * sqrt(w22);
        const S inv_w2 = 1 / ret.values()(2);
        ret.values()(0) = w02 * inv_w2;
        ret.values()(1) = w12 * inv_w2;
      }
    }
    return ret;
  }

  template SO3Alg<double> log<double>(const Rot3D<double>& g);
  template SO3Alg<float> log<float>(const Rot3D<float>& g);

}

