#pragma once

#include <memory>
#include <iostream>
#include <Eigen/Core>
#include <camera/traits.hpp>

namespace camera {

class GenericCameraBase {
public:
  using Ptr = std::shared_ptr<GenericCameraBase>;
  using ConstPtr = std::shared_ptr<const GenericCameraBase>;

  GenericCameraBase() {}
  virtual ~GenericCameraBase() {}

  virtual bool in_max_fov(const Eigen::Vector3d& point_3d) const = 0;

  Eigen::Vector2d project(const Eigen::Vector3d& point_3d) const { return (*this)(point_3d); }

  virtual Eigen::Vector2d operator()(const Eigen::Vector3d& point_3d) const = 0;
};

template <typename Projection>
class GenericCamera : public GenericCameraBase {
public:
  GenericCamera(const Eigen::VectorXd& intrinsic, const Eigen::VectorXd& distortion)
  : min_z(std::cos(CameraModelTraits<Projection>::max_fov)),
    intrinsic(intrinsic),
    distortion(distortion) {
    std::cout << "min_z:" << min_z << std::endl;
  }

  virtual bool in_max_fov(const Eigen::Vector3d& point_3d) const override {
    return point_3d.z() / point_3d.norm() > min_z;
  }

  virtual Eigen::Vector2d operator()(const Eigen::Vector3d& point_3d) const override {
    Projection proj;
    return proj(intrinsic.data(), distortion.data(), point_3d);
  }

private:
  const double min_z;
  Eigen::VectorXd intrinsic;
  Eigen::VectorXd distortion;
};

}  // namespace camera