#pragma once

#include <numeric>
#include <dfo/optimizer.hpp>

namespace dfo {

template <int N>
class DirectionalDirectSearch : public Optimizer<N> {
public:
  struct Params {
    Params() : init_alpha(0.5), min_alpha(1e-5), alpha_dec_factor(1.0 / 8.0), alpha_inc_factor(2.0), max_iterations(1024) {}

    double init_alpha;
    double min_alpha;
    double alpha_dec_factor;
    double alpha_inc_factor;

    int max_iterations;
  };

  using typename Optimizer<N>::Result;
  using typename Optimizer<N>::Function;
  using typename Optimizer<N>::VectorN;

  DirectionalDirectSearch(const Params& params) : params(params) {}
  virtual ~DirectionalDirectSearch() {}

  virtual Result optimize(const Function& function, const VectorN& x0) override {
    Result result;

    double alpha = params.init_alpha;
    result.x = x0;
    result.y = function(x0);

    for (int i = 0; i < params.max_iterations; i++) {
      result.num_iterations = i;

      result.y = function(result.x);
      if (!coordinate_search(function, result.x, result.y, alpha)) {
        alpha *= params.alpha_dec_factor;
      } else {
        alpha *= params.alpha_inc_factor;
      }

      if(this->callback) {
        this->callback(result.x);
      }

      if(alpha < params.min_alpha) {
        break;
      }
    }

    return result;
  }

private:
  bool coordinate_search(const Function& function, VectorN& x, double& y, double alpha) {
    const double y0 = y;

    for(int i=0; i < N; i++) {
      VectorN dir = VectorN::Zero();
      dir[i] = 1.0;

      if(try_direction(function, x, y, dir, alpha)) {
        continue;
      }

      try_direction(function, x, y, -dir, alpha);
    }

    return y < y0;
  }

  bool try_direction(const Function& function, VectorN& x, double& y, const VectorN& dir, double alpha) {
    bool decreased = false;

    while(true) {
      const VectorN xi = x + alpha * dir;
      const double yi = function(xi);

      if(yi > y) {
        break;
      }

      decreased = true;
      x = xi;
      y = yi;
    }

    return decreased;
  }

private:
  const Params params;
};

}  // namespace dfo
