#include <camera/generic_camera.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <dfo/nelder_mead.hpp>

namespace camera {

double GenericCameraBase::estimate_fov(const Eigen::Vector2i& image_size) const {
  const std::vector<Eigen::Vector2d> target_corners = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(image_size[0] / 2, 0.0), Eigen::Vector2d(0.0, image_size[1] / 2)};

  const auto to_dir = [](const Eigen::Vector2d& x) {
    return Eigen::AngleAxisd(x[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(x[1], Eigen::Vector3d::UnitY()) * Eigen::Vector3d::UnitZ();
  };

  double max_fov = 0.0;
  for (const auto& corner : target_corners) {
    const auto f = [&](const Eigen::Vector2d& x) {
      const Eigen::Vector3d dir = to_dir(x);
      return (corner - this->project(dir)).squaredNorm();
    };

    dfo::NelderMead<2>::Params params;
    dfo::NelderMead<2> optimizer(params);
    auto result = optimizer.optimize(f, Eigen::Vector2d::Zero());

    const auto dir = to_dir(result.x);
    const double fov = std::acos(dir.normalized().z());

    if(fov > max_fov) {
      max_fov = fov;
    }
  }

  return max_fov;
}

}  // namespace camera
