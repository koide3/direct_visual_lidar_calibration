#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vlcal {

class CostCalculator {
public:
  using Ptr = std::shared_ptr<CostCalculator>;
  using ConstPtr = std::shared_ptr<const CostCalculator>;

  CostCalculator() {}
  virtual ~CostCalculator() {}

  virtual double calculate(const Eigen::Isometry3d& T_camera_lidar) = 0;
};

}  // namespace vlcal
