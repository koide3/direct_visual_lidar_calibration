#pragma once

#include <iostream>
#include <vlcal/calib/cost_calculator.hpp>

namespace vlcal {

class CostScaler : public CostCalculator {
public:
  CostScaler(const CostCalculator::Ptr& cost, const double scale = 1.0) : scale(scale), cost(cost), init_cost(-1.0) {}
  ~CostScaler() override {}

  virtual void update_correspondences(const Eigen::Isometry3d& T_camera_lidar) override {  //
    std::cout << "--- T_camera_lidar ---" << std::endl << T_camera_lidar.matrix() << std::endl;

    cost->update_correspondences(T_camera_lidar);
  }

  virtual double calculate(const Eigen::Isometry3d& T_camera_lidar) override {
    double x = cost->calculate(T_camera_lidar);
    if (x < 0.0) {
      std::cerr << "error: negative cost!!" << std::endl;
    }

    if (init_cost < 0.0) {
      init_cost = x;
    }

    return scale * x / init_cost;
  }

  virtual void visualize(const Eigen::Isometry3d& T_camera_lidar) override {  //
    cost->visualize(T_camera_lidar);
  }

private:
  const double scale;
  CostCalculator::Ptr cost;

  double init_cost;
};

}  // namespace vlcal
