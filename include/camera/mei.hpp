#pragma once

#include <string>
#include <Eigen/Core>
#include <camera/traits.hpp>

namespace camera {

// MEI unified projection with Brown distortion [k1,k2,p1,p2,k3].
// The factory supplies the physical branch and field-of-view validity checks.
struct MEIProjection {
  template <typename T, typename T2>
  auto operator()(const T* const intrinsic, const T* const distortion, const Eigen::Matrix<T2, 3, 1>& point_3d) const
    -> Eigen::Matrix<decltype(T() * T2()), 2, 1> {
    // Rescaling first avoids overflow in the squared norm. It does not change
    // the bearing or its derivative because the projection is homogeneous.
    const auto scaled = (point_3d / point_3d.cwiseAbs().maxCoeff()).eval();
    const auto sphere = scaled.normalized().eval();
    const auto plane = (sphere.template head<2>() / (sphere.z() + intrinsic[4])).eval();
    const auto x2 = plane.x() * plane.x();
    const auto y2 = plane.y() * plane.y();
    const auto xy = plane.x() * plane.y();
    const auto r2 = x2 + y2;
    const auto radial = 1.0 + distortion[0] * r2 + distortion[1] * r2 * r2 + distortion[4] * r2 * r2 * r2;
    const auto x = plane.x() * radial + 2.0 * distortion[2] * xy + distortion[3] * (r2 + 2.0 * x2);
    const auto y = plane.y() * radial + distortion[2] * (r2 + 2.0 * y2) + 2.0 * distortion[3] * xy;
    return {intrinsic[0] * x + intrinsic[2], intrinsic[1] * y + intrinsic[3]};
  }
};

template <>
struct CameraModelTraits<MEIProjection> {
  static constexpr int num_intrinsic_params = 5;
  static constexpr int num_distortion_params = 5;
  static std::string projection_model() { return "mei"; }
  static std::string distortion_model() { return "plumb_bob"; }
  static Eigen::Matrix<double, 5, 1> init_intrinsic(int, int, const Eigen::VectorXd& pinhole_intrinsic) {
    Eigen::Matrix<double, 5, 1> intrinsic;
    intrinsic << pinhole_intrinsic, 1.0;
    return intrinsic;
  }
  static Eigen::Matrix<double, 5, 1> init_distortion() { return Eigen::Matrix<double, 5, 1>::Zero(); }
};

}  // namespace camera
