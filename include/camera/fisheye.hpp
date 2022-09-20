#pragma once

#include <Eigen/Core>
#include <camera/traits.hpp>

namespace camera {

/**
 * @brief Pinhole projection + fisheye distortion (compatible with OpenCV maybe)
 * https://docs.opencv.org/4.5.2/db/d58/group__calib3d__fisheye.html
*/
struct FisheyeProjection {
  template <typename T, typename T2>
  auto operator()(const T* const intrinsic, const T* const distortion, const Eigen::Matrix<T2, 3, 1>& point_3d) const -> Eigen::Matrix<decltype(T() * T2()), 2, 1> {
    const auto r = point_3d.template head<2>().norm();
    const auto theta = atan2(r, abs(point_3d.z()));
    const auto theta2 = pow(theta, 2);
    const auto theta4 = pow(theta, 4);
    const auto theta6 = pow(theta, 6);
    const auto theta8 = pow(theta, 8);

    const auto& k1 = distortion[0];
    const auto& k2 = distortion[1];
    const auto& k3 = distortion[2];
    const auto& k4 = distortion[3];

    const auto theta_d = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    const auto pt_d = ((theta_d / r) * point_3d.template head<2>()).eval();

    const auto& fx = intrinsic[0];
    const auto& fy = intrinsic[1];
    const auto& cx = intrinsic[2];
    const auto& cy = intrinsic[3];

    return {fx * pt_d[0] + cx, fy * pt_d[1] + cy};
  }
};

// traits
template <>
struct CameraModelTraits<FisheyeProjection> {
  static constexpr int num_intrinsic_params = 4;
  static constexpr int num_distortion_params = 4;

  static std::string projection_model() { return "pinhole"; }

  static std::string distortion_model() { return "fisheye"; }

  static Eigen::Vector4d init_intrinsic(int width, int height, const Eigen::VectorXd& pinhole_intrinsic) { return pinhole_intrinsic; }

  static Eigen::Matrix<double, 4, 1> init_distortion() { return Eigen::Matrix<double, 4, 1>::Zero(); }
};

}  // namespace camera