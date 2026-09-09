#pragma once

#include <memory>
#include <iostream>
#include <cmath>
#include <limits>
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

  // Geometry checks run on the real values before projection, including for
  // autodiff costs. A camera may see rays with negative optical z.
  virtual bool is_valid(const Eigen::Vector3d& point) const {
    if (!point.allFinite()) {
      return false;
    }
    const double norm = point.stableNorm();
    return std::isfinite(norm) && norm > 1e-12;
  }

  // Check the complete floor(pixel)+[-before,+after] sampling footprint.
  virtual bool is_pixel_valid(const Eigen::Vector2d& pixel, int width, int height, int before = 0, int after = 0) const {
    return pixel.allFinite() && before >= 0 && after >= 0 && width > before + after && height > before + after &&
           pixel.x() >= before && pixel.y() >= before && pixel.x() < width - after && pixel.y() < height - after;
  }

  // NaN means callers should retain the legacy estimated field of view.
  virtual double max_theta_rad() const { return std::numeric_limits<double>::quiet_NaN(); }

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
