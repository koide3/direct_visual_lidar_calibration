#include <camera/create_camera.hpp>
#include <algorithm>

#include <ceres/jet.h>

#include <camera/atan.hpp>
#include <camera/pinhole.hpp>
#include <camera/fisheye.hpp>
#include <camera/omnidir.hpp>
#include <camera/mei.hpp>
#include <camera/equirectangular.hpp>
#include <camera/rational_polynomial.hpp>
#include <camera/generic_camera.hpp>

#include <vlcal/common/console_colors.hpp>

namespace camera {

namespace {

class ValidatedCamera : public GenericCameraBase {
public:
  ValidatedCamera(GenericCameraBase::ConstPtr camera, const CameraValidity& validity, double mei_xi)
  : camera(std::move(camera)), theta(validity.max_theta_deg * M_PI / 180.0), min_z(std::cos(theta)),
    mask(validity.image_mask.clone()), mei_xi(mei_xi) {}

  Eigen::Vector2d project(const Eigen::Vector3d& point) const override { return camera->project(point); }
  Eigen::Vector2d operator()(const Eigen::Vector3d& point) const override { return (*camera)(point); }
  Eigen::Matrix<ceres::Jet<double, 7>, 2, 1> operator()(const Eigen::Matrix<ceres::Jet<double, 7>, 3, 1>& point) const override {
    return (*camera)(point);
  }

  bool is_valid(const Eigen::Vector3d& point) const override {
    if (!camera->is_valid(point)) {
      return false;
    }
    const double z = point.z() / point.stableNorm();
    if (!std::isfinite(z) || z < min_z - 1e-12) {
      return false;
    }
    // For xi > 1, denominator positivity alone admits the folded branch.
    return !std::isfinite(mei_xi) || (z + mei_xi > 1e-12 && 1.0 + mei_xi * z > 1e-12);
  }

  bool is_pixel_valid(const Eigen::Vector2d& pixel, int width, int height, int before = 0, int after = 0) const override {
    if (!camera->is_pixel_valid(pixel, width, height, before, after)) {
      return false;
    }
    if (mask.empty()) {
      return true;
    }
    if (mask.cols != width || mask.rows != height) {
      return false;
    }
    const int x = static_cast<int>(std::floor(pixel.x()));
    const int y = static_cast<int>(std::floor(pixel.y()));
    for (int dy = -before; dy <= after; ++dy) {
      for (int dx = -before; dx <= after; ++dx) {
        if (mask.at<unsigned char>(y + dy, x + dx) == 0) {
          return false;
        }
      }
    }
    return true;
  }

  double max_theta_rad() const override { return theta; }

private:
  const GenericCameraBase::ConstPtr camera;
  const double theta;
  const double min_z;
  const cv::Mat mask;
  const double mei_xi;
};

bool is_mei_model(const std::string& model) { return model == "mei" || model == "MEI"; }

}  // namespace

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

static camera::GenericCameraBase::ConstPtr create_camera_unchecked(const std::string& camera_model, const std::vector<double>& intrinsics, const std::vector<double>& distortion_coeffs) {
  if (camera_model == "plumb_bob") {
    return create_camera<camera::PinholeProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "fisheye" || camera_model == "equidistant") {
    return create_camera<camera::FisheyeProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "atan") {
    return create_camera<camera::ATANProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "omnidir") {
    return create_camera<camera::OmnidirectionalProjection>(intrinsics, distortion_coeffs);
  } else if (is_mei_model(camera_model)) {
    return create_camera<camera::MEIProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "equirectangular") {
    return create_camera<camera::EquirectangularProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "rational_polynomial") {
    return create_camera<camera::RationalPolynomialProjection>(intrinsics, distortion_coeffs);
  }

  std::cerr << vlcal::console::bold_red << "error: unknown camera model " << camera_model << vlcal::console::reset << std::endl;
  return nullptr;
}

camera::GenericCameraBase::ConstPtr create_camera(
  const std::string& camera_model, const std::vector<double>& intrinsics, const std::vector<double>& distortion_coeffs) {
  if (is_mei_model(camera_model)) {
    return create_camera(camera_model, intrinsics, distortion_coeffs, CameraValidity());
  }
  return create_camera_unchecked(camera_model, intrinsics, distortion_coeffs);
}

camera::GenericCameraBase::ConstPtr create_camera(
  const std::string& camera_model, const std::vector<double>& intrinsics, const std::vector<double>& distortion_coeffs, const CameraValidity& validity) {
  const auto finite = [](double value) { return std::isfinite(value); };
  if (!std::isfinite(validity.max_theta_deg) || validity.max_theta_deg <= 0.0 || validity.max_theta_deg > 180.0 ||
      (!validity.image_mask.empty() && validity.image_mask.type() != CV_8UC1) ||
      !std::all_of(intrinsics.begin(), intrinsics.end(), finite) || !std::all_of(distortion_coeffs.begin(), distortion_coeffs.end(), finite)) {
    std::cerr << "error: invalid camera parameters, half-angle, or image mask" << std::endl;
    return nullptr;
  }
  if (is_mei_model(camera_model) && (intrinsics.size() != 5 || distortion_coeffs.size() != 5 || intrinsics[0] <= 0.0 || intrinsics[1] <= 0.0)) {
    std::cerr << "error: MEI requires [fx,fy,cx,cy,xi] and [k1,k2,p1,p2,k3], with positive focal lengths" << std::endl;
    return nullptr;
  }
  auto camera = create_camera_unchecked(camera_model, intrinsics, distortion_coeffs);
  if (!camera) {
    return nullptr;
  }
  const double xi = is_mei_model(camera_model) ? intrinsics[4] : std::numeric_limits<double>::quiet_NaN();
  return std::make_shared<ValidatedCamera>(camera, validity, xi);
}

}  // namespace camera
