#pragma once
#include <Eigen/Core>

template<class T, int DimRows, int DimCols, class Tag>
class Tagged {
public:

  using Matrix = Eigen::Matrix<T, DimRows, DimCols>;

private:
  Matrix _values;

public:

  Tagged() {

  }

  template <class Derived>
  explicit Tagged(const Eigen::EigenBase<Derived>& d)
    : _values(d) {

  }

  const Matrix& values() const {
    return _values;
  }

  Matrix& values() {
    return _values;
  }

  const Matrix& operator()() const {
    return _values;
  }

  Matrix& operator()() {
    return _values;
  }

};