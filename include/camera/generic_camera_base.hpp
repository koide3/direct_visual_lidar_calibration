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

/**
 * @brief Generic camera projection class
 */
class GenericCameraBase {
public:
  using Ptr = std::shared_ptr<GenericCameraBase>;
  using ConstPtr = std::shared_ptr<const GenericCameraBase>;

  GenericCameraBase() {}
  virtual ~GenericCameraBase() {}

  /**
   * @brief Project a 3D point into the image space
   */
  virtual Eigen::Vector2d project(const Eigen::Vector3d& point_3d) const = 0;

  /**
   * @brief Project a 3D point into the image space (syntex sugger of project())
   */
  virtual Eigen::Vector2d operator()(const Eigen::Vector3d& point_3d) const = 0;

  /**
   * @brief Projection with autodiff
   * @note  Is there a better way to accept Jets with different number of params without exposing the implementation of the projection func?
   */
  virtual Eigen::Matrix<ceres::Jet<double, 7>, 2, 1> operator()(const Eigen::Matrix<ceres::Jet<double, 7>, 3, 1>& point_3d) const = 0;
};

}  // namespace camera