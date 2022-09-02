#pragma once

#include <Eigen/Core>
#include <camera/traits.hpp>

namespace camera {

struct ATANProjection {
  template <typename T>
  Eigen::Matrix<T, 2, 1> distort(const T* const distortion, const Eigen::Matrix<T, 2, 1>& pt) const {
    const T& d0 = distortion[0];
    T r = pt.norm();
    if (r < 1e-3 || d0 < 1e-7) return pt;

    const T d1 = 1.0 / d0;
    const T d2 = 2.0 * tan(d0 / 2.0);
    T distortionFactor = d1 * atan(r * d2) / r;

    return distortionFactor * pt;
  }

  template <typename T, typename T2>
  Eigen::Matrix<T, 2, 1> operator()(const T* const intrinsic, const T* const distortion, const Eigen::Matrix<T2, 3, 1>& point_3d) const {
    Eigen::Matrix<T, 2, 1> pt_2d = point_3d.template head<2>() / point_3d.z();
    Eigen::Matrix<T, 2, 1> pt_d = distort(distortion, pt_2d);

    const T& fx = intrinsic[0];
    const T& fy = intrinsic[1];
    const T& cx = intrinsic[2];
    const T& cy = intrinsic[3];

    return Eigen::Matrix<T, 2, 1>(fx * pt_d[0] + cx, fy * pt_d[1] + cy);
  }
};

// traits
template <>
struct CameraModelTraits<ATANProjection> {
  static constexpr int num_intrinsic_params = 4;
  static constexpr int num_distortion_params = 1;
  static constexpr double max_fov = M_PI_2;

  static std::string projection_model() { return "atan"; }

  static std::string distortion_model() { return "atan"; }

  static Eigen::Vector4d init_intrinsic(int width, int height, const Eigen::VectorXd& pinhole_intrinsic) { return pinhole_intrinsic; }

  static Eigen::Matrix<double, 1, 1> init_distortion() { return Eigen::Matrix<double, 1, 1>(0.1); }
};

}  // namespace camera