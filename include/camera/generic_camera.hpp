#pragma once

#include <memory>
#include <Eigen/Core>

namespace camera {

class GenericCameraBase {
public:
  using Ptr = std::shared_ptr<GenericCameraBase>;
  using ConstPtr = std::shared_ptr<const GenericCameraBase>;

  GenericCameraBase() {}
  virtual ~GenericCameraBase() {}

  virtual Eigen::Vector2d operator()(const Eigen::Vector3d& point_3d) const = 0;
};

template <typename Projection>
class GenericCamera : public GenericCameraBase {
public:
  GenericCamera(const Eigen::VectorXd& intrinsic, const Eigen::VectorXd& distortion) : intrinsic(intrinsic), distortion(distortion) {}

  virtual Eigen::Vector2d operator()(const Eigen::Vector3d& point_3d) const override {
    Projection proj;
    return proj(intrinsic.data(), distortion.data(), point_3d);
  }

private:
  Eigen::VectorXd intrinsic;
  Eigen::VectorXd distortion;
};

}  // namespace camera