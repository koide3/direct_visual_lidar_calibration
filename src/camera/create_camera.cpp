#include <camera/create_camera.hpp>

#include <ceres/jet.h>

#include <camera/atan.hpp>
#include <camera/pinhole.hpp>
#include <camera/fisheye.hpp>
#include <camera/omnidir.hpp>
#include <camera/equirectangular.hpp>
#include <camera/generic_camera.hpp>

#include <vlcal/common/console_colors.hpp>

namespace camera {

template <typename Projection>
camera::GenericCameraBase::ConstPtr create_camera(const std::vector<double>& intrinsics, const std::vector<double>& distortion_coeffs) {
  if (intrinsics.size() != camera::CameraModelTraits<Projection>::num_intrinsic_params) {
    std::cerr << vlcal::console::bold_red << "error: num of intrinsic parameters mismatch!!" << vlcal::console::reset << std::endl;
    return nullptr;
  }

  std::vector<double> dist_coeffs(camera::CameraModelTraits<Projection>::num_distortion_params, 0.0);
  for (int i = 0; i < std::min(distortion_coeffs.size(), dist_coeffs.size()); i++) {
    dist_coeffs[i] = distortion_coeffs[i];
  }

  return std::make_shared<camera::GenericCamera<Projection>>(
    Eigen::Map<const Eigen::VectorXd>(intrinsics.data(), intrinsics.size()),
    Eigen::Map<const Eigen::VectorXd>(dist_coeffs.data(), dist_coeffs.size()));
}

camera::GenericCameraBase::ConstPtr create_camera(const std::string& camera_model, const std::vector<double>& intrinsics, const std::vector<double>& distortion_coeffs) {
  if (camera_model == "plumb_bob") {
    return create_camera<camera::PinholeProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "fisheye" || camera_model == "equidistant") {
    return create_camera<camera::FisheyeProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "atan") {
    return create_camera<camera::ATANProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "omnidir") {
    return create_camera<camera::OmnidirectionalProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "equirectangular") {
    return create_camera<camera::EquirectangularProjection>(intrinsics, distortion_coeffs);
  }

  std::cerr << vlcal::console::bold_red << "error: unknown camera model " << camera_model << vlcal::console::reset << std::endl;
  return nullptr;
}

}  // namespace camera
