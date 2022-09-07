#pragma once

#include <memory>
#include <camera/generic_camera_base.hpp>

namespace camera {

camera::GenericCameraBase::ConstPtr create_camera(const std::string& camera_model, const std::vector<double>& intrinsics, const std::vector<double>& distortion_coeffs);

}  // namespace vlcal
