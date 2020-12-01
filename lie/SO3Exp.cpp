#include "SO3Exp.h"
#include "Constants.h"


namespace lie {

  template<typename S>
  Rot3D<S> exp(const SO3Alg<S>& g) {
    const auto& w = g.values();
    auto w2 = w.cwiseAbs2();
    const S theta_sq = w2.sum();  // sum of the squares
    const S theta = std::sqrt(theta_sq);

    S a, b;
    if (theta < Constants<S>::eps) {
      a = S(1.0) - theta_sq * (S(1.0 / 6) + theta_sq * S(1.0 / 120));
      b = S(0.5) - theta_sq * (S(1.0 / 24) - theta_sq * S(1.0 / 720));
    }
    else {
      a = sin(theta) / theta;
      b = (S(1.0) - cos(theta)) / theta_sq;
    }


    Rot3D<S> ret;
    auto& R = ret();

    R(0, 0) = 1 - b * (w2(1) + w2(2));
    R(1, 1) = 1 - b * (w2(0) + w2(2));
    R(2, 2) = 1 - b * (w2(0) + w2(1));

    const S Bab = b * w(0) * w(1);
    const S Bac = b * w(0) * w(2);
    const S Bbc = b * w(1) * w(2);

    R(0, 1) = Bab - a * w(2);
    R(1, 0) = Bab + a * w(2);
    R(0, 2) = Bac + a * w(1);
    R(2, 0) = Bac - a * w(1);
    R(1, 2) = Bbc - a * w(0);
    R(2, 1) = Bbc + a * w(0);

    return ret;
  }

  template Rot3D<double> exp<double>(const SO3Alg<double>& g);
  template Rot3D<float> exp<float>(const SO3Alg<float>& g);

}

