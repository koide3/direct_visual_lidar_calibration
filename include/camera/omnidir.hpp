#pragma once

#include <Eigen/Core>
#include <camera/traits.hpp>

namespace camera {

// Unified omnidirectional camera model
// https://github.com/opencv/opencv_contrib/blob/master/modules/ccalib/src/omnidir.cpp
struct OmnidirectionalProjection {
  template <typename T, typename T2>
  Eigen::Matrix<T, 2, 1> operator()(const T* const intrinsic, const T* const distortion, const Eigen::Matrix<T2, 3, 1>& point_3d) const {
    const auto& fx = intrinsic[0];
    const auto& fy = intrinsic[1];
    const auto& cx = intrinsic[2];
    const auto& cy = intrinsic[3];
    const auto& xi = intrinsic[4];

    const auto& k1 = distortion[0];
    const auto& k2 = distortion[1];
    const auto& p1 = distortion[2];
    const auto& p2 = distortion[3];

    Eigen::Matrix<T2, 3, 1> pt_s = point_3d.normalized();
    Eigen::Matrix<T, 2, 1> pt_u = pt_s.template head<2>() / (pt_s.z() + xi);

    T r2 = pt_u.squaredNorm();
    T r4 = r2 * r2;

    T dr = (1.0 + k1 * r2 + k2 * r4);
    T x2 = pt_u[0] * pt_u[0];
    T y2 = pt_u[1] * pt_u[1];
    T xy = pt_u[0] * pt_u[1];

    Eigen::Matrix<T, 2, 1> pt_d;
    pt_d[0] = pt_u[0] * dr + 2.0 * p1 * xy + p2 * (r2 + 2.0 * x2);
    pt_d[1] = pt_u[1] * dr + p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy;

    return Eigen::Matrix<T, 2, 1>(fx * pt_d[0] + cx, fy * pt_d[1] + cy);
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