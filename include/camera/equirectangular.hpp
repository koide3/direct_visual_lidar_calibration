#pragma once

#include <Eigen/Core>
#include <camera/traits.hpp>

namespace camera {

/**
 * @brief Equirectangular omnidirectional projection.
 *        "From google street view to 3d city models", ICCVW, 2009
 */
struct EquirectangularProjection {
  template <typename T, typename T2>
  auto operator()(const T* const intrinsic, const T* const distortion, const Eigen::Matrix<T2, 3, 1>& point_3d) const -> Eigen::Matrix<decltype(T() * T2()), 2, 1> {
    if (point_3d.squaredNorm() < 1e-3) {
      return {intrinsic[0] / 2, intrinsic[1] / 2};
    }

    const auto bearing = point_3d.normalized().eval();

    const auto lat = -asin(bearing[1]);
    const auto lon = atan2(bearing[0], bearing[2]);

    const auto x = intrinsic[0] * (0.5 + lon / (2.0 * M_PI));
    const auto y = intrinsic[1] * (0.5 - lat / M_PI);

    return {x, y};
  }
};

// traits
template <>
struct CameraModelTraits<EquirectangularProjection> {
  static constexpr int num_intrinsic_params = 2;
  static constexpr int num_distortion_params = 0;

  static std::string projection_model() { return "equirectangular"; }

  static std::string distortion_model() { return "none"; }

  static Eigen::Vector4d init_intrinsic(int width, int height, const Eigen::VectorXd& pinhole_intrinsic) { return pinhole_intrinsic; }

  static Eigen::Matrix<double, 5, 1> init_distortion() { return Eigen::Matrix<double, 5, 1>::Zero(); }
};

}  // namespace camera