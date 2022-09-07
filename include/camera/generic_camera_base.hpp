#pragma once

#include <memory>
#include <iostream>
#include <Eigen/Core>
#include <camera/traits.hpp>

namespace ceres {
template <typename T, int N>
struct Jet;
}  // namespace ceres

namespace camera {

class GenericCameraBase {
public:
  using Ptr = std::shared_ptr<GenericCameraBase>;
  using ConstPtr = std::shared_ptr<const GenericCameraBase>;

  GenericCameraBase() {}
  virtual ~GenericCameraBase() {}

  virtual Eigen::Vector2d project(const Eigen::Vector3d& point_3d) const = 0;

  virtual Eigen::Vector2d operator()(const Eigen::Vector3d& point_3d) const = 0;

  virtual Eigen::Matrix<ceres::Jet<double, 7>, 2, 1> operator()(const Eigen::Matrix<ceres::Jet<double, 7>, 3, 1>& point_3d) const = 0;
};

}  // namespace camera