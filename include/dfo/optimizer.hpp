#pragma once

#include <vector>
#include <Eigen/Core>

namespace dfo {

template<int N>
struct OptimizationResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OptimizationResult() : converged(false), num_iterations(0) {}

  bool converged;
  int num_iterations;

  Eigen::Matrix<double, N, 1> x;
  double y;
};

template<int N>
class Optimizer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Optimizer() {}
  virtual ~Optimizer() {}

  using Result = OptimizationResult<N>;
  using VectorN = Eigen::Matrix<double, N, 1>;
  using Function = std::function<double(const VectorN&)>;

  virtual Result optimize(const Function& function, const VectorN& x0) = 0;

  template <typename Func>
  void set_callback(const Func& f) {
    callback = f;
  }

  template <typename Func>
  void set_particles_callback(const Func& f) {
    particles_callback = f;
  }

protected:
  std::function<void(const VectorN&)> callback;
  std::function<void(const std::vector<VectorN, Eigen::aligned_allocator<VectorN>>&)> particles_callback;
};
}