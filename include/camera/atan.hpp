#pragma once

#include <Eigen/Core>
#include <camera/traits.hpp>

namespace camera {

/**
 * @brief ATAN projection model.
 *        "Straight  lines  have  to  be  straight", MVA, 2021
 */
struct ATANProjection {
  template <typename T, typename T2>
  auto distort(const T* const distortion, const Eigen::Matrix<T2, 2, 1>& pt) const -> Eigen::Matrix<decltype(T() * T2()), 2, 1> {
    const auto& d0 = distortion[0];
    const auto r = pt.norm();
    if (r < 1e-3 || d0 < 1e-7) {
      return pt;
    }

    const auto d1 = 1.0 / d0;
    const auto d2 = 2.0 * tan(d0 / 2.0);
    const auto distortion_factor = d1 * atan(r * d2) / r;

    return distortion_factor * pt;
  }

  template <typename T, typename T2>
  auto operator()(const T* const intrinsic, const T* const distortion, const Eigen::Matrix<T2, 3, 1>& point_3d) const -> Eigen::Matrix<decltype(T() * T2()), 2, 1> {
    const auto pt_2d = (point_3d.template head<2>() / point_3d.z()).eval();
    const auto pt_d = distort(distortion, pt_2d);

    const auto& fx = intrinsic[0];
    const auto& fy = intrinsic[1];
    const auto& cx = intrinsic[2];
    const auto& cy = intrinsic[3];

    return {fx * pt_d[0] + cx, fy * pt_d[1] + cy};
  }
};

// traits
template <>
struct CameraModelTraits<ATANProjection> {
  static constexpr int num_intrinsic_params = 4;
  static constexpr int num_distortion_params = 1;

  static std::string projection_model() { return "atan"; }

  static std::string distortion_model() { return "atan"; }

  static Eigen::Vector4d init_intrinsic(int width, int height, const Eigen::VectorXd& pinhole_intrinsic) { return pinhole_intrinsic; }

  static Eigen::Matrix<double, 1, 1> init_distortion() { return Eigen::Matrix<double, 1, 1>(0.1); }
};

}  // namespace camera