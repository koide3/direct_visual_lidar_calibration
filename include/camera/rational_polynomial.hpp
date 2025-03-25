#pragma once
#include <Eigen/Core>
#include <string>

namespace camera {
/**
 * @brief Rational polynomial projection (compatible with OpenCV)
 */
struct RationalPolynomialProjection {
  template <typename T, typename T2>
  auto distort(const T* const distortion, const Eigen::Matrix<T2, 2, 1>& pt) const -> Eigen::Matrix<decltype(T() * T2()), 2, 1> {
    // Extract distortion parameters
    const auto& k1 = distortion[0];
    const auto& k2 = distortion[1];
    const auto& p1 = distortion[2];
    const auto& p2 = distortion[3];
    const auto& k3 = distortion[4];
    const auto& k4 = distortion[5];
    const auto& k5 = distortion[6];
    const auto& k6 = distortion[7];

    const auto x2 = pt.x() * pt.x();
    const auto y2 = pt.y() * pt.y();
    const auto xy = pt.x() * pt.y();
    const auto r2 = x2 + y2;
    const auto r4 = r2 * r2;
    const auto r6 = r2 * r4;

    // Rational polynomial distortion
    const auto numerator = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    const auto denominator = 1.0 + k4 * r2 + k5 * r4 + k6 * r6;

    // Avoid division by zero
    const auto r_coeff = denominator > T(1e-8) ? numerator / denominator : numerator;

    // Tangential distortion
    const auto t_coeff1 = 2.0 * pt.x() * pt.y();
    const auto t_coeff2 = r2 + 2.0 * x2;
    const auto t_coeff3 = r2 + 2.0 * y2;

    const auto x = r_coeff * pt.x() + p1 * t_coeff1 + p2 * t_coeff2;
    const auto y = r_coeff * pt.y() + p1 * t_coeff3 + p2 * t_coeff1;

    return {x, y};
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

// Traits for rational polynomial camera model
template <>
struct CameraModelTraits<RationalPolynomialProjection> {
  static constexpr int num_intrinsic_params = 4;
  static constexpr int num_distortion_params = 8;

  static std::string projection_model() { return "pinhole"; }
  static std::string distortion_model() { return "rational_polynomial"; }

  static Eigen::Vector4d init_intrinsic(int width, int height, const Eigen::VectorXd& pinhole_intrinsic) { return pinhole_intrinsic; }

  static Eigen::Matrix<double, 8, 1> init_distortion() { return Eigen::Matrix<double, 8, 1>::Zero(); }
};

}  // namespace camera
