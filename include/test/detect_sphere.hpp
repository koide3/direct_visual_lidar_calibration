#pragma once

#include <gtsam_ext/types/frame.hpp>

Eigen::Vector4d detect_sphere_ransac(const gtsam_ext::Frame::ConstPtr& points);