#pragma once

#include <numeric>
#include <dfo/optimizer.hpp>

namespace dfo {

template <int N>
class NelderMead : public Optimizer<N> {
public:
  struct Params {
    Params() : init_step(0.1), alpha(1.0), gamma(2.0), rho(0.5), sigma(0.5), max_iterations(1024), convergence_var_thresh(1e-5) {}

    double init_step;
    double alpha;
    double gamma;
    double rho;
    double sigma;

    int max_iterations;
    double convergence_var_thresh;
  };

  using typename Optimizer<N>::Result;
  using typename Optimizer<N>::Function;
  using typename Optimizer<N>::VectorN;
  using VectorM = Eigen::Matrix<double, N + 1, 1>;  // value & sample

  NelderMead(const Params& params) : params(params) {}
  virtual ~NelderMead() {}

  virtual Result optimize(const Function& function, const VectorN& x0) override {
    Result result;

    std::vector<VectorM, Eigen::aligned_allocator<VectorM>> x(1);
    x[0][0] = function(x0);
    x[0].template tail<N>() = x0;

    for(int i=0; i<N; i++) {
      VectorM xi;
      xi.template tail<N>() = x0;
      xi.template tail<N>()[i] += params.init_step;

      xi[0] = function(xi.template tail<N>());
      x.push_back(xi);
    }


    for(int i=0; i<params.max_iterations; i++) {
      result.num_iterations = i;
      std::sort(x.begin(), x.end(), [=](const VectorM& lhs, const VectorM& rhs) { return lhs[0] < rhs[0]; });
      if (is_converged(x)) {
        result.converged = true;
        break;
      }

      VectorM xo = std::accumulate(x.begin(), x.end() - 1, VectorM::Zero().eval()) / (x.size() - 1);
      xo[0] = function(xo.template tail<N>());

      VectorM xr = xo + params.alpha * (xo - x.back());
      xr[0] = function(xr.template tail<N>());

      if (x[0][0] <= xr[0] && xr[0] < x[N - 1][0]) {
        x.back() = xr;
      } else if (xr[0] < x[0][0]) {
        VectorM xe = xo + params.gamma * (xo - x.back());
        xe[0] = function(xe.template tail<N>());

        if (xe[0] < xr[0]) {
          x.back() = xe;
        } else {
          x.back() = xr;
        }
      } else {
        VectorM xc = xo + params.rho * (xo - x.back());
        xc[0] = function(xc.template tail<N>());

        if (xc[0] < x.back()[0]) {
          x.back() = xc;
        } else {
          for (int j = 1; j < x.size(); j++) {
            x[j] = x[0] + params.rho * (x[j] - x[0]);
            x[j][0] = function(x[j].template tail<N>());
          }
        }
      }

      if (this->callback) {
        this->callback(x[0].template tail<N>());
      }

      if (this->particles_callback) {
        std::vector<VectorN, Eigen::aligned_allocator<VectorN>> particles(x.size());
        std::transform(x.begin(), x.end(), particles.begin(), [=](const VectorM& xi) { return xi.template tail<N>(); });
        this->particles_callback(particles);
      }
    }

    result.x = x[0].template tail<N>();
    result.y = x[0][0];
    return result;
  }

private:
  bool is_converged(const std::vector<VectorM, Eigen::aligned_allocator<VectorM>>& x) {
    VectorM mean = std::accumulate(x.begin(), x.end(), VectorM::Zero().eval()) / x.size();
    VectorM var = VectorM::Zero();
    for (const auto& xi : x) {
      var = var.array() + (xi - mean).array().square();
    }

    return var.template tail<N>().sum() < params.convergence_var_thresh;
  }

private:
  const Params params;
};

}  // namespace dfo