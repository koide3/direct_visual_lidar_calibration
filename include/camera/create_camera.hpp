#pragma once

#include <memory>
#include <camera/generic_camera_base.hpp>

namespace camera {

/**
 * @brief Create a generic camera model
 * @param camera_model       Camera projection model (plumb_bob, fisheye, atan, omnidir, or equirectangular)
 * @param intrinsics         Camera intrinsic parameters
 * @param distortion_coeffs  Camera distortion coefficients
 */
camera::GenericCameraBase::ConstPtr create_camera(const std::string& camera_model, const std::vector<double>& intrinsics, const std::vector<double>& distortion_coeffs);

}  // namespace vlcal
