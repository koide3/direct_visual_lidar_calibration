#pragma once

#include <memory>
#include <iostream>
#include <Eigen/Core>
#include <camera/traits.hpp>
#include <camera/generic_camera_base.hpp>

namespace ceres {
template <typename T, int N>
struct Jet;
}  // namespace ceres

namespace camera {

template <typename Projection>
class GenericCamera : public GenericCameraBase {
public:
  GenericCamera(const Eigen::VectorXd& intrinsic, const Eigen::VectorXd& distortion) : intrinsic(intrinsic), distortion(distortion) {}

  virtual Eigen::Vector2d project(const Eigen::Vector3d& point_3d) const override { //
    return (*this)(point_3d);
  }

  virtual Eigen::Vector2d operator()(const Eigen::Vector3d& point_3d) const override {
    Projection proj;
    return proj(intrinsic.data(), distortion.data(), point_3d);
  }

  virtual Eigen::Matrix<ceres::Jet<double, 7>, 2, 1> operator()(const Eigen::Matrix<ceres::Jet<double, 7>, 3, 1>& point_3d) const override {
    Projection proj;
    return proj(intrinsic.data(), distortion.data(), point_3d);
  }

private:
  Eigen::VectorXd intrinsic;
  Eigen::VectorXd distortion;
};

}  // namespace camera