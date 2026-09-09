#pragma once

#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <camera/generic_camera_base.hpp>

namespace camera {

/**
 * @brief Create a generic camera model
 * @param camera_model       Camera projection model (plumb_bob, fisheye, atan, omnidir, mei, or equirectangular)
 * @param intrinsics         Camera intrinsic parameters
 * @param distortion_coeffs  Camera distortion coefficients
 */
camera::GenericCameraBase::ConstPtr create_camera(const std::string& camera_model, const std::vector<double>& intrinsics, const std::vector<double>& distortion_coeffs);

struct CameraValidity {
  double max_theta_deg = 95.0;
  cv::Mat image_mask;  // Optional CV_8UC1, nonzero pixels are valid.
};

camera::GenericCameraBase::ConstPtr create_camera(
  const std::string& camera_model,
  const std::vector<double>& intrinsics,
  const std::vector<double>& distortion_coeffs,
  const CameraValidity& validity);

}  // namespace vlcal
