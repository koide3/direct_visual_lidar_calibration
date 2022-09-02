#pragma once

#include <Eigen/Core>
#include <camera/traits.hpp>

namespace camera {

struct PinholeProjection {
  template <typename T>
  Eigen::Matrix<T, 2, 1> distort(const T* const distortion, const Eigen::Matrix<T, 2, 1>& pt) const {
    const T& k1 = distortion[0];
    const T& k2 = distortion[1];
    const T& k3 = distortion[4];

    const T& p1 = distortion[2];
    const T& p2 = distortion[3];

    T x2 = pt.x() * pt.x();
    T y2 = pt.y() * pt.y();
    T xy = pt.x() * pt.y();

    T r2 = x2 + y2;
    T r4 = r2 * r2;
    T r6 = r2 * r4;

    T r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    T t_coeff1 = 2.0 * pt.x() * pt.y();
    T t_coeff2 = r2 + 2.0 * x2;
    T t_coeff3 = r2 + 2.0 * y2;

    Eigen::Matrix<T, 2, 1> pt_d;
    pt_d.x() = r_coeff * pt.x() + p1 * t_coeff1 + p2 * t_coeff2;
    pt_d.y() = r_coeff * pt.y() + p1 * t_coeff3 + p2 * t_coeff1;

    return pt_d;
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
struct CameraModelTraits<PinholeProjection> {
  static constexpr int num_intrinsic_params = 4;
  static constexpr int num_distortion_params = 5;
  static constexpr double max_fov = M_PI_2;

  static std::string projection_model() { return "pinhole"; }

  static std::string distortion_model() { return "plumb_bob"; }

  static Eigen::Vector4d init_intrinsic(int width, int height, const Eigen::VectorXd& pinhole_intrinsic) { return pinhole_intrinsic; }

  static Eigen::Matrix<double, 5, 1> init_distortion() { return Eigen::Matrix<double, 5, 1>::Zero(); }
};

}  // namespace camera