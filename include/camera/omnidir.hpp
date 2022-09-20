#pragma once

#include <Eigen/Core>
#include <camera/traits.hpp>

namespace camera {

/**
 * @brief Unified omnidirectional projection model + plumb_bob distortion
 * https://github.com/opencv/opencv_contrib/blob/master/modules/ccalib/src/omnidir.cpp
 */
struct OmnidirectionalProjection {
  template <typename T, typename T2>
  auto operator()(const T* const intrinsic, const T* const distortion, const Eigen::Matrix<T2, 3, 1>& point_3d) const -> Eigen::Matrix<decltype(T() * T2()), 2, 1> {
    const auto& fx = intrinsic[0];
    const auto& fy = intrinsic[1];
    const auto& cx = intrinsic[2];
    const auto& cy = intrinsic[3];
    const auto& xi = intrinsic[4];

    const auto& k1 = distortion[0];
    const auto& k2 = distortion[1];
    const auto& p1 = distortion[2];
    const auto& p2 = distortion[3];

    const auto pt_s = point_3d.normalized().eval();
    const auto pt_u = (pt_s.template head<2>() / (pt_s.z() + xi)).eval();

    const auto r2 = pt_u.squaredNorm();
    const auto r4 = r2 * r2;

    const auto dr = (1.0 + k1 * r2 + k2 * r4);
    const auto x2 = pt_u[0] * pt_u[0];
    const auto y2 = pt_u[1] * pt_u[1];
    const auto xy = pt_u[0] * pt_u[1];

    const auto nx = pt_u[0] * dr + 2.0 * p1 * xy + p2 * (r2 + 2.0 * x2);
    const auto ny = pt_u[1] * dr + p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy;

    return {fx * nx + cx, fy * ny + cy};
  }
};

// traits
template <>
struct CameraModelTraits<OmnidirectionalProjection> {
  static constexpr int num_intrinsic_params = 5;
  static constexpr int num_distortion_params = 4;

  static std::string projection_model() { return "unified"; }

  static std::string distortion_model() { return "plumn_bob"; }

  static Eigen::Matrix<double, 5, 1> init_intrinsic(int width, int height, const Eigen::VectorXd& pinhole_intrinsic) {
    Eigen::Matrix<double, 5, 1> intrinsic;
    intrinsic << pinhole_intrinsic, 1.0;
    return intrinsic;
  }

  static Eigen::Matrix<double, 4, 1> init_distortion() { return Eigen::Matrix<double, 4, 1>::Zero(); }
};

}  // namespace camera